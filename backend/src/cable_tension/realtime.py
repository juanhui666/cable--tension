"""已知犁轨迹求解器的有状态同步传感器执行入口。

每个会话拥有一个持久动态运行状态。传感器数据包依次经过校验、公共时基
插值区间转换、求解和采样，全部成功后才作为一次原子推进提交。失败数据包
不会改变已接收的 sequence 或继承的物理状态。
"""

from __future__ import annotations

import math
import threading
import uuid
from collections import deque
from copy import deepcopy
from dataclasses import dataclass, replace
from time import perf_counter

from .simulation import (
    CurrentSample,
    MotionSample,
    ScalarSample,
    TimeHistoryFrame,
    TimeHistoryPoint,
)
from .dynamic_laying import (
    KnownPloughSample,
    advance_known_plough_runtime,
    initialize_known_plough_runtime,
    minimum_bend_radius_status,
    sample_known_plough_runtime,
)


CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS = 1.0e-9
REALTIME_PACKET_INTERVAL_S = 1.0
REALTIME_MAX_SENSOR_GAP_S = 1.5
PASSIVE_PLOUGH_FORWARD_TOLERANCE_MPS = 1.0e-9
PASSIVE_PLOUGH_SAMPLE_STEP_MAX_S = 0.01
_KINEMATICS_EPSILON = 1.0e-12


class PassivePloughDomainError(ValueError):
    """船速超出前向被动拖曳模型适用域。"""


@dataclass(frozen=True)
class PassivePloughKinematicSample:
    """被动拖曳犁在一个内部子步时刻的艏向和前进速度。"""

    time_s: float
    heading_rad: float
    speed_mps: float


def passive_plough_forward_speed(
    vessel_velocity_x_mps: float,
    vessel_velocity_y_mps: float,
    plough_heading_rad: float,
) -> float:
    """返回船速在犁艏向上的投影，并拒绝反向拖曳输入。"""

    projection = (
        vessel_velocity_x_mps * math.cos(plough_heading_rad)
        + vessel_velocity_y_mps * math.sin(plough_heading_rad)
    )
    if projection < -PASSIVE_PLOUGH_FORWARD_TOLERANCE_MPS:
        raise PassivePloughDomainError(
            "vessel velocity projected onto the plough heading must be non-negative "
            "for the forward passive-tow model "
            f"(projection={projection:.12g} m/s)"
        )
    return max(0.0, projection)


def passive_plough_kinematic_samples(
    *,
    start_time_s: float,
    end_time_s: float,
    layback_m: float,
    initial_heading_rad: float,
    start_vessel_velocity_x_mps: float,
    start_vessel_velocity_y_mps: float,
    end_vessel_velocity_x_mps: float,
    end_vessel_velocity_y_mps: float,
    step_max_s: float = PASSIVE_PLOUGH_SAMPLE_STEP_MAX_S,
) -> tuple[PassivePloughKinematicSample, ...]:
    """按拖车方程生成固定后拖距离所需的内部犁运动子步。"""

    duration_s = end_time_s - start_time_s
    if duration_s < -_KINEMATICS_EPSILON:
        raise ValueError("passive plough interval end time must not precede start time")
    if layback_m < 0.0 or not math.isfinite(layback_m):
        raise ValueError("passive plough layback must be finite and non-negative")
    if step_max_s <= 0.0 or not math.isfinite(step_max_s):
        raise ValueError("passive plough sample step must be finite and positive")

    start_speed = passive_plough_forward_speed(
        start_vessel_velocity_x_mps,
        start_vessel_velocity_y_mps,
        initial_heading_rad,
    )
    samples = [
        PassivePloughKinematicSample(
            time_s=start_time_s,
            heading_rad=initial_heading_rad,
            speed_mps=start_speed,
        )
    ]
    if duration_s <= _KINEMATICS_EPSILON:
        return tuple(samples)

    step_count = max(1, math.ceil(duration_s / step_max_s))
    step_s = duration_s / step_count
    heading = initial_heading_rad

    def velocity_at(fraction: float) -> tuple[float, float]:
        return (
            start_vessel_velocity_x_mps
            + fraction * (end_vessel_velocity_x_mps - start_vessel_velocity_x_mps),
            start_vessel_velocity_y_mps
            + fraction * (end_vessel_velocity_y_mps - start_vessel_velocity_y_mps),
        )

    def heading_rate(
        current_heading: float,
        velocity_x_mps: float,
        velocity_y_mps: float,
    ) -> float:
        if layback_m <= _KINEMATICS_EPSILON:
            return 0.0
        return (
            -velocity_x_mps * math.sin(current_heading)
            + velocity_y_mps * math.cos(current_heading)
        ) / layback_m

    for index in range(step_count):
        fraction_0 = index / step_count
        fraction_mid = (index + 0.5) / step_count
        fraction_1 = (index + 1.0) / step_count
        velocity_0 = velocity_at(fraction_0)
        velocity_mid = velocity_at(fraction_mid)
        velocity_1 = velocity_at(fraction_1)
        k1 = heading_rate(heading, *velocity_0)
        heading_mid_1 = heading + 0.5 * step_s * k1
        passive_plough_forward_speed(*velocity_mid, heading_mid_1)
        k2 = heading_rate(heading_mid_1, *velocity_mid)
        heading_mid_2 = heading + 0.5 * step_s * k2
        passive_plough_forward_speed(*velocity_mid, heading_mid_2)
        k3 = heading_rate(heading_mid_2, *velocity_mid)
        heading_end = heading + step_s * k3
        passive_plough_forward_speed(*velocity_1, heading_end)
        k4 = heading_rate(heading_end, *velocity_1)
        heading += step_s * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        heading = math.atan2(math.sin(heading), math.cos(heading))
        speed = passive_plough_forward_speed(*velocity_1, heading)
        samples.append(
            PassivePloughKinematicSample(
                time_s=start_time_s + (index + 1) * step_s,
                heading_rad=heading,
                speed_mps=speed,
            )
        )
    return tuple(samples)


def validate_realtime_current_representation(
    interpolation: str,
    velocity_x_mps: float,
    velocity_y_mps: float,
    speed_mps: float | None,
    direction_unwrapped_deg: float | None,
) -> None:
    """校验数据包的笛卡尔/极坐标双重表示。"""

    if interpolation == "cartesian_linear":
        return
    if interpolation != "polar_unwrapped":
        raise ValueError("current interpolation is not supported")
    if speed_mps is None or direction_unwrapped_deg is None:
        raise ValueError("polar current packets require speed and unwrapped direction")
    if speed_mps < 0.0:
        raise ValueError("current speed must be non-negative")
    radians = math.radians(direction_unwrapped_deg)
    expected_x = speed_mps * math.cos(radians)
    expected_y = speed_mps * math.sin(radians)
    if not math.isclose(
        velocity_x_mps,
        expected_x,
        rel_tol=0.0,
        abs_tol=CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS,
    ) or not math.isclose(
        velocity_y_mps,
        expected_y,
        rel_tol=0.0,
        abs_tol=CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS,
    ):
        raise ValueError(
            "polar current Cartesian components must match speed*cos/sin(direction) "
            f"within {CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS:.0e} m/s"
        )


@dataclass(frozen=True)
class SynchronizedEndpointSample:
    """一个已完成坐标转换的端点位置和速度采样。"""

    x_m: float
    y_m: float
    z_m: float
    velocity_x_mps: float
    velocity_y_mps: float
    velocity_z_mps: float


@dataclass(frozen=True)
class RealtimeSensorPacket:
    """实时会话原子处理的一个公共时基数据包。"""

    sequence: int
    time_s: float
    vessel: SynchronizedEndpointSample
    plough: SynchronizedEndpointSample
    payout_speed_mps: float
    plough_position_source: str
    plough_exit_speed_mps: float | None
    current_velocity_x_mps: float
    current_velocity_y_mps: float
    current_interpolation: str = "cartesian_linear"
    current_speed_mps: float | None = None
    current_direction_unwrapped_deg: float | None = None
    plough_position_uncertainty_m: float | None = None
    measured_top_tension_n: float | None = None
    plough_heading_rad: float | None = None
    plough_layback_m: float | None = None


@dataclass(frozen=True)
class RealtimeBendRadiusConstraint:
    """一个实时帧对应的最小弯曲半径约束状态。"""

    minimum_m: float | None
    limit_m: float | None
    margin_m: float | None
    status: str


@dataclass(frozen=True)
class RealtimeFrameResult:
    """实时会话返回的最新帧及计时依据。"""

    session_id: str
    sequence: int
    time_s: float
    compute_wall_s: float
    realtime_factor: float | None
    plough_exit_speed_mps: float
    plough_exit_speed_source: str
    plough_position_source: str
    plough_position_uncertainty_m: float | None
    measured_top_tension_n: float | None
    top_tension_residual_n: float | None
    plough_inlet_horizontal_angle_deg: float
    plough_inlet_vertical_angle_deg: float
    point: TimeHistoryPoint
    frame: TimeHistoryFrame
    integration_time_step_min_s: float | None
    integration_time_step_max_s: float | None
    axial_constraint_residual_max_m: float | None
    bend_radius_constraint: RealtimeBendRadiusConstraint


def realtime_bend_radius_constraint(
    minimum_m: float | None,
    limit_m: float | None,
) -> RealtimeBendRadiusConstraint:
    """按离线汇总相同口径形成当前实时帧的弯曲约束状态。"""

    finite_minimum = (
        None
        if minimum_m is None or not math.isfinite(minimum_m)
        else float(minimum_m)
    )
    finite_limit = None if limit_m is None else float(limit_m)
    margin_m = (
        None
        if finite_minimum is None or finite_limit is None
        else finite_minimum - finite_limit
    )
    return RealtimeBendRadiusConstraint(
        minimum_m=finite_minimum,
        limit_m=finite_limit,
        margin_m=margin_m,
        status=minimum_bend_radius_status(
            minimum_radius_m=finite_minimum,
            limit_m=finite_limit,
        ),
    )


class RealtimeSessionError(ValueError):
    """不改变已接收状态的结构化会话错误。"""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


class RealtimeSimulationSession:
    """持有并串行推进一个持久的已知犁轨迹仿真。"""

    def __init__(
        self,
        *,
        session_id: str,
        base_case,
        initial_packet: RealtimeSensorPacket,
        max_sensor_gap_s: float = REALTIME_MAX_SENSOR_GAP_S,
        frame_buffer_size: int = 120,
    ) -> None:
        """使用必需的 sequence=0 状态数据包初始化会话。

        初始数据包定义 t=0 时的两端边界和材料通量，只用于初始化运行状态一次，
        不会在每次更新前重放。
        """

        if not session_id:
            raise ValueError("session_id is required")
        if max_sensor_gap_s <= 0.0:
            raise ValueError("sensor gap limit must be positive")
        if frame_buffer_size <= 0:
            raise ValueError("frame_buffer_size must be positive")
        self.session_id = session_id
        self.base_case = base_case
        self.max_sensor_gap_s = float(max_sensor_gap_s)
        self._lock = threading.Lock()
        self._frames: deque[RealtimeFrameResult] = deque(maxlen=frame_buffer_size)

        self._validate_initial_packet(initial_packet)
        initial_case = self._case_for_packets(initial_packet, initial_packet)
        self._runtime = initialize_known_plough_runtime(initial_case)
        self._runtime.dt_max_s = min(self._runtime.dt_max_s, 0.01)
        self._previous_packet: RealtimeSensorPacket | None = None
        self._packet = initial_packet
        sample = sample_known_plough_runtime(self._runtime, initial_case)
        self._latest = self._result(
            packet=initial_packet,
            sample=sample,
            compute_wall_s=0.0,
            previous_time_s=None,
        )
        self._frames.append(self._latest)

    @property
    def current_sequence(self) -> int:
        """返回最近一次原子接收的数据包 sequence。"""

        return self._packet.sequence

    @property
    def current_time_s(self) -> float:
        """返回继承的求解器状态已经推进到的物理时间。"""

        return self._runtime.time_s

    @property
    def latest(self) -> RealtimeFrameResult:
        """返回最新接收帧，不推进会话。"""

        return self._latest

    @property
    def frames(self) -> tuple[RealtimeFrameResult, ...]:
        """返回定长帧缓冲区的不可变快照。"""

        return tuple(self._frames)

    def advance(self, packet: RealtimeSensorPacket) -> RealtimeFrameResult:
        """原子推进并提交一个严格有序的传感器数据包。

        非阻塞锁拒绝并发写入；求解或采样失败时恢复运行状态快照，
        调用方修正数据后可重试同一 sequence。
        """

        if not self._lock.acquire(blocking=False):
            raise RealtimeSessionError("session_busy", "the session is already advancing")
        try:
            self._validate_next_packet(packet)
            previous_packet = self._packet
            step_case = self._case_for_packets(
                previous_packet,
                packet,
                preceding=self._previous_packet,
            )
            # 求解器会原位修改持久运行状态；数据包和输出帧均成功前，
            # 保留完整事务快照。
            runtime_before = deepcopy(self._runtime)
            started = perf_counter()
            try:
                advance_known_plough_runtime(
                    self._runtime,
                    step_case,
                    target_time_s=packet.time_s,
                )
                sample = sample_known_plough_runtime(self._runtime, step_case)
                compute_wall = perf_counter() - started
                result = self._result(
                    packet=packet,
                    sample=sample,
                    compute_wall_s=compute_wall,
                    previous_time_s=previous_packet.time_s,
                )
            except Exception:
                self._runtime = runtime_before
                raise
            # 仅在动态状态和响应均有效后提交数据包历史。
            self._previous_packet = previous_packet
            self._packet = packet
            self._latest = result
            self._frames.append(result)
            return result
        finally:
            self._lock.release()

    def _validate_initial_packet(self, packet: RealtimeSensorPacket) -> None:
        self._validate_packet_values(packet)
        if packet.sequence != 0:
            raise RealtimeSessionError("sequence_conflict", "initial sequence must be 0")
        if not math.isclose(packet.time_s, 0.0, abs_tol=1.0e-9):
            raise RealtimeSessionError("non_monotonic_time", "initial time_s must be 0")

    def _validate_next_packet(self, packet: RealtimeSensorPacket) -> None:
        self._validate_packet_values(packet)
        if packet.sequence != self._packet.sequence + 1:
            raise RealtimeSessionError("sequence_conflict", "sequence must increment by one")
        time_step_s = packet.time_s - self._packet.time_s
        if time_step_s <= 1.0e-12:
            raise RealtimeSessionError("non_monotonic_time", "time_s must strictly increase")
        if time_step_s > self.max_sensor_gap_s + 1.0e-12:
            raise RealtimeSessionError("sensor_gap", "sensor time gap exceeds the session limit")
        if abs(time_step_s - REALTIME_PACKET_INTERVAL_S) > 1.0e-9:
            raise RealtimeSessionError("invalid_time_step", "time_s must increment by exactly 1.0 s")

    @staticmethod
    def _validate_packet_values(packet: RealtimeSensorPacket) -> None:
        """在接触会话状态前校验所有物理量均为有限值。"""

        numeric_values = (
            packet.time_s,
            packet.vessel.x_m,
            packet.vessel.y_m,
            packet.vessel.z_m,
            packet.vessel.velocity_x_mps,
            packet.vessel.velocity_y_mps,
            packet.vessel.velocity_z_mps,
            packet.plough.x_m,
            packet.plough.y_m,
            packet.plough.z_m,
            packet.plough.velocity_x_mps,
            packet.plough.velocity_y_mps,
            packet.plough.velocity_z_mps,
            packet.payout_speed_mps,
            packet.current_velocity_x_mps,
            packet.current_velocity_y_mps,
        )
        optional_numeric_values = (
            packet.plough_exit_speed_mps,
            packet.current_speed_mps,
            packet.current_direction_unwrapped_deg,
            packet.plough_position_uncertainty_m,
            packet.measured_top_tension_n,
            packet.plough_heading_rad,
            packet.plough_layback_m,
        )
        if packet.sequence < 0 or any(not math.isfinite(float(value)) for value in numeric_values):
            raise RealtimeSessionError("invalid_packet", "sensor packet values must be finite")
        if any(value is not None and not math.isfinite(float(value)) for value in optional_numeric_values):
            raise RealtimeSessionError("invalid_packet", "optional sensor packet values must be finite")
        if packet.plough_position_source not in {"estimated", "measured"}:
            raise RealtimeSessionError(
                "invalid_packet",
                "plough position source must be estimated or measured",
            )
        try:
            validate_realtime_current_representation(
                packet.current_interpolation,
                packet.current_velocity_x_mps,
                packet.current_velocity_y_mps,
                packet.current_speed_mps,
                packet.current_direction_unwrapped_deg,
            )
        except ValueError as exc:
            raise RealtimeSessionError("invalid_packet", str(exc)) from exc
        if packet.payout_speed_mps < 0.0 or (
            packet.plough_exit_speed_mps is not None and packet.plough_exit_speed_mps < 0.0
        ):
            raise RealtimeSessionError("invalid_packet", "material speeds must be non-negative")
        if packet.plough_position_uncertainty_m is not None and packet.plough_position_uncertainty_m < 0.0:
            raise RealtimeSessionError("invalid_packet", "plough position uncertainty must be non-negative")
        if packet.plough_layback_m is not None and packet.plough_layback_m < 0.0:
            raise RealtimeSessionError("invalid_packet", "plough layback must be non-negative")
        if packet.measured_top_tension_n is not None and packet.measured_top_tension_n < 0.0:
            raise RealtimeSessionError("invalid_packet", "measured top tension must be non-negative")

    def _case_for_packets(
        self,
        start: RealtimeSensorPacket,
        end: RealtimeSensorPacket,
        *,
        preceding: RealtimeSensorPacket | None = None,
    ):
        """构造用于推进继承状态的插值区间。

        前一数据包保留采样运动合同所需的左侧导数上下文。此处不重复应用
        初始化字段；返回工况仅驱动下一个物理时间区间。
        """

        from .simulation import CurrentSample, MotionSample, ScalarSample

        interpolation_packets = self._distinct_packets(preceding, start, end)
        vessel_samples, plough_samples, plough_exit_samples = (
            self._boundary_and_material_samples(interpolation_packets)
        )
        payout_samples = tuple(
            ScalarSample(packet.time_s, packet.payout_speed_mps)
            for packet in interpolation_packets
        )
        current_samples = tuple(
            CurrentSample(
                packet.time_s,
                packet.current_velocity_x_mps,
                packet.current_velocity_y_mps,
                interpolation=packet.current_interpolation,
                speed_mps=packet.current_speed_mps,
                direction_unwrapped_deg=packet.current_direction_unwrapped_deg,
            )
            for packet in interpolation_packets
        )
        current_speed = (
            end.current_speed_mps
            if end.current_interpolation == "polar_unwrapped" and end.current_speed_mps is not None
            else math.hypot(end.current_velocity_x_mps, end.current_velocity_y_mps)
        )
        current_direction = (
            end.current_direction_unwrapped_deg % 360.0
            if end.current_interpolation == "polar_unwrapped" and end.current_direction_unwrapped_deg is not None
            else math.degrees(math.atan2(end.current_velocity_y_mps, end.current_velocity_x_mps)) % 360.0
            if current_speed > 1.0e-12
            else self.base_case.current_direction_deg
        )
        return replace(
            self.base_case,
            current_speed_mps=current_speed,
            current_direction_deg=current_direction,
            payout_initial_speed_mps=start.payout_speed_mps,
            payout_final_speed_mps=end.payout_speed_mps,
            plough_exit_speed_mps=self._effective_plough_exit_speed(end),
            vessel_motion_segments=(),
            plough_motion_segments=(),
            payout_speed_segments=(),
            vessel_motion_samples=vessel_samples,
            plough_motion_samples=plough_samples,
            payout_speed_samples=payout_samples,
            plough_exit_speed_samples=plough_exit_samples,
            current_samples=current_samples,
        )

    @staticmethod
    def _effective_plough_exit_speed(packet: RealtimeSensorPacket) -> float:
        """由内部显式值或犁运动速度确定 q_p。

        生产 API 不接收显式 q_p，因此公开数据包由犁入口运动速度派生。
        dataclass 中的显式值只保留给求解器内部测试与程序化调用。
        """

        if packet.plough_exit_speed_mps is not None:
            return packet.plough_exit_speed_mps
        return math.sqrt(
            packet.plough.velocity_x_mps**2
            + packet.plough.velocity_y_mps**2
            + packet.plough.velocity_z_mps**2
        )

    @classmethod
    def _boundary_and_material_samples(
        cls,
        packets: tuple[RealtimeSensorPacket, ...],
    ) -> tuple[tuple[MotionSample, ...], tuple[MotionSample, ...], tuple[ScalarSample, ...]]:
        """为估算犁区间生成与求解步长一致的被动拖曳子步。"""

        if not packets:
            return (), (), ()
        vessel_samples = [cls._motion_sample(packets[0].time_s, packets[0].vessel)]
        plough_samples = [cls._motion_sample(packets[0].time_s, packets[0].plough)]
        plough_exit_samples = [
            ScalarSample(packets[0].time_s, cls._effective_plough_exit_speed(packets[0]))
        ]

        for start, end in zip(packets, packets[1:]):
            if (
                start.plough_position_source == "estimated"
                and end.plough_position_source == "estimated"
                and start.plough_heading_rad is not None
                and start.plough_layback_m is not None
                and start.plough_exit_speed_mps is None
                and end.plough_exit_speed_mps is None
            ):
                kinematic_samples = passive_plough_kinematic_samples(
                    start_time_s=start.time_s,
                    end_time_s=end.time_s,
                    layback_m=start.plough_layback_m,
                    initial_heading_rad=start.plough_heading_rad,
                    start_vessel_velocity_x_mps=start.vessel.velocity_x_mps,
                    start_vessel_velocity_y_mps=start.vessel.velocity_y_mps,
                    end_vessel_velocity_x_mps=end.vessel.velocity_x_mps,
                    end_vessel_velocity_y_mps=end.vessel.velocity_y_mps,
                )
                duration_s = end.time_s - start.time_s
                for sample in kinematic_samples[1:]:
                    fraction = (sample.time_s - start.time_s) / duration_s
                    vessel = cls._interpolate_endpoint(start.vessel, end.vessel, fraction)
                    depth_m = start.plough.z_m + fraction * (end.plough.z_m - start.plough.z_m)
                    plough = SynchronizedEndpointSample(
                        x_m=vessel.x_m - start.plough_layback_m * math.cos(sample.heading_rad),
                        y_m=vessel.y_m - start.plough_layback_m * math.sin(sample.heading_rad),
                        z_m=depth_m,
                        velocity_x_mps=sample.speed_mps * math.cos(sample.heading_rad),
                        velocity_y_mps=sample.speed_mps * math.sin(sample.heading_rad),
                        velocity_z_mps=0.0,
                    )
                    vessel_samples.append(cls._motion_sample(sample.time_s, vessel))
                    plough_samples.append(cls._motion_sample(sample.time_s, plough))
                    plough_exit_samples.append(ScalarSample(sample.time_s, sample.speed_mps))
                continue

            vessel_samples.append(cls._motion_sample(end.time_s, end.vessel))
            plough_samples.append(cls._motion_sample(end.time_s, end.plough))
            plough_exit_samples.append(
                ScalarSample(end.time_s, cls._effective_plough_exit_speed(end))
            )

        return tuple(vessel_samples), tuple(plough_samples), tuple(plough_exit_samples)

    @staticmethod
    def _interpolate_endpoint(
        start: SynchronizedEndpointSample,
        end: SynchronizedEndpointSample,
        fraction: float,
    ) -> SynchronizedEndpointSample:
        """按公共时基线性插值端点位置和速度。"""

        def interpolate(start_value: float, end_value: float) -> float:
            return start_value + fraction * (end_value - start_value)

        return SynchronizedEndpointSample(
            x_m=interpolate(start.x_m, end.x_m),
            y_m=interpolate(start.y_m, end.y_m),
            z_m=interpolate(start.z_m, end.z_m),
            velocity_x_mps=interpolate(start.velocity_x_mps, end.velocity_x_mps),
            velocity_y_mps=interpolate(start.velocity_y_mps, end.velocity_y_mps),
            velocity_z_mps=interpolate(start.velocity_z_mps, end.velocity_z_mps),
        )

    @staticmethod
    def _distinct_packets(*packets: RealtimeSensorPacket | None):
        distinct: list[RealtimeSensorPacket] = []
        for packet in packets:
            if packet is None:
                continue
            if distinct and math.isclose(
                distinct[-1].time_s,
                packet.time_s,
                rel_tol=0.0,
                abs_tol=1.0e-12,
            ):
                distinct[-1] = packet
            else:
                distinct.append(packet)
        return tuple(distinct)

    @staticmethod
    def _motion_sample(time_s: float, endpoint: SynchronizedEndpointSample) -> MotionSample:
        return MotionSample(
            time_s=time_s,
            x_m=endpoint.x_m,
            y_m=endpoint.y_m,
            z_m=endpoint.z_m,
            velocity_x_mps=endpoint.velocity_x_mps,
            velocity_y_mps=endpoint.velocity_y_mps,
            velocity_z_mps=endpoint.velocity_z_mps,
        )

    def _result(
        self,
        *,
        packet: RealtimeSensorPacket,
        sample: KnownPloughSample,
        compute_wall_s: float,
        previous_time_s: float | None,
    ) -> RealtimeFrameResult:
        """封装已接收的求解器采样，不将测量值反馈到求解过程。

        船端实测张力只用于形成残差；求解帧和端点反力仍是返回客户端的权威状态。
        """

        physical_step = None if previous_time_s is None else packet.time_s - previous_time_s
        realtime_factor = (
            None
            if physical_step is None or compute_wall_s <= 0.0
            else physical_step / compute_wall_s
        )
        effective_plough_exit_speed = self._effective_plough_exit_speed(packet)
        top_tension = float(sample.point.top_tension_n)
        measured_top_tension = packet.measured_top_tension_n
        horizontal_angle, vertical_angle = self._plough_inlet_angles(sample.frame)
        bend_radius_constraint = realtime_bend_radius_constraint(
            sample.frame.minimum_bend_radius_m,
            self.base_case.min_bending_radius_m,
        )
        return RealtimeFrameResult(
            session_id=self.session_id,
            sequence=packet.sequence,
            time_s=packet.time_s,
            compute_wall_s=compute_wall_s,
            realtime_factor=realtime_factor,
            plough_exit_speed_mps=effective_plough_exit_speed,
            plough_exit_speed_source=(
                "explicit" if packet.plough_exit_speed_mps is not None else "plough_motion_inferred"
            ),
            plough_position_source=packet.plough_position_source,
            plough_position_uncertainty_m=packet.plough_position_uncertainty_m,
            measured_top_tension_n=measured_top_tension,
            top_tension_residual_n=(
                None if measured_top_tension is None else measured_top_tension - top_tension
            ),
            plough_inlet_horizontal_angle_deg=horizontal_angle,
            plough_inlet_vertical_angle_deg=vertical_angle,
            point=sample.point,
            frame=sample.frame,
            integration_time_step_min_s=self._runtime.integration_time_step_min_s,
            integration_time_step_max_s=self._runtime.integration_time_step_max_s,
            axial_constraint_residual_max_m=self._runtime.axial_constraint_residual_max_m,
            bend_radius_constraint=bend_radius_constraint,
        )

    @staticmethod
    def _plough_inlet_angles(frame: object) -> tuple[float, float]:
        points = frame.points
        if len(points) < 2:
            raise RuntimeError("plough inlet direction requires at least two cable nodes")
        previous, inlet = points[-2], points[-1]
        dx = float(inlet.x_m - previous.x_m)
        dy = float(inlet.y_m - previous.y_m)
        dz = float(inlet.z_m - previous.z_m)
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length <= 1.0e-12:
            raise RuntimeError("plough inlet direction is undefined for a zero-length final segment")
        horizontal_angle = math.degrees(math.atan2(dy, dx))
        vertical_angle = math.degrees(math.atan2(dz, math.hypot(dx, dy)))
        return horizontal_angle, vertical_angle


class RealtimeSessionRegistry:
    """活动内存实时会话的线程安全管理器。"""

    def __init__(self) -> None:
        self._sessions: dict[str, RealtimeSimulationSession] = {}
        self._lock = threading.Lock()

    def create(
        self,
        *,
        base_case,
        initial_packet: RealtimeSensorPacket,
        max_sensor_gap_s: float,
    ) -> RealtimeSimulationSession:
        """仅在初始化成功后创建并登记会话。"""

        session_id = uuid.uuid4().hex
        session = RealtimeSimulationSession(
            session_id=session_id,
            base_case=base_case,
            initial_packet=initial_packet,
            max_sensor_gap_s=max_sensor_gap_s,
        )
        with self._lock:
            self._sessions[session_id] = session
        return session

    def get(self, session_id: str) -> RealtimeSimulationSession | None:
        """返回指定活动会话；id 未知时返回 ``None``。"""

        with self._lock:
            return self._sessions.get(session_id)

    def delete(self, session_id: str) -> bool:
        """移除指定会话，并返回该会话此前是否存在。"""

        with self._lock:
            return self._sessions.pop(session_id, None) is not None
