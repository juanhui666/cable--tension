"""仿真的输入、插值、校验和输出合同。

本模块是离线与实时执行共用的语义边界：定义运动与海流时程，
校验物理取值域并记录求解器输出，但不执行动态时间积分。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .materials import CableParameters, get_material


_MIN_SPEED_MPS = 1.0e-12
_LONGITUDINAL_SPEED_MATCH_ABS_TOLERANCE_MPS = 1.0e-9
_CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS = 1.0e-9
_DEFAULT_DYNAMIC_CABLE = get_material("POWER_500KV")
SOLVER_ID = "known_plough_ale_xpbd"
_SEGMENT_INTERPOLATIONS = {"linear", "smootherstep", "sampled_smootherstep"}


def _sampled_smootherstep_interval(
    fraction: float,
    *,
    duration_s: float,
    sample_interval_s: float | None,
) -> tuple[int, int, float]:
    if sample_interval_s is None or sample_interval_s <= 0.0:
        raise ValueError("sample_interval_s must be positive for sampled smootherstep")
    interval_count = max(1, round(duration_s / sample_interval_s))
    if not math.isclose(interval_count * sample_interval_s, duration_s, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError("sample_interval_s must divide segment duration_s")
    scaled = max(0.0, min(1.0, fraction)) * interval_count
    lower = min(int(math.floor(scaled)), interval_count - 1)
    upper = lower + 1
    return lower, interval_count, max(0.0, min(1.0, scaled - lower))

def _smootherstep(fraction: float) -> float:
    u = max(0.0, min(1.0, fraction))
    return 6.0 * u**5 - 15.0 * u**4 + 10.0 * u**3

def segment_interpolation_fraction(
    interpolation: str,
    fraction: float,
    *,
    duration_s: float = 1.0,
    sample_interval_s: float | None = None,
) -> float:
    """计算归一化的分段插值合同。"""

    u = max(0.0, min(1.0, fraction))
    if interpolation == "linear":
        return u
    if interpolation == "smootherstep":
        return _smootherstep(u)
    if interpolation == "sampled_smootherstep":
        lower, count, local = _sampled_smootherstep_interval(
            u,
            duration_s=duration_s,
            sample_interval_s=sample_interval_s,
        )
        start = _smootherstep(lower / count)
        end = _smootherstep((lower + 1) / count)
        return start + (end - start) * local
    raise ValueError(f"unsupported segment interpolation: {interpolation}")

def segment_interpolation_integral(
    interpolation: str,
    fraction: float,
    *,
    duration_s: float = 1.0,
    sample_interval_s: float | None = None,
) -> float:
    """将归一化插值从零积分到 ``fraction``。"""

    u = max(0.0, min(1.0, fraction))
    if interpolation == "linear":
        return 0.5 * u**2
    if interpolation == "smootherstep":
        return u**6 - 3.0 * u**5 + 2.5 * u**4
    if interpolation == "sampled_smootherstep":
        lower, count, local = _sampled_smootherstep_interval(
            u,
            duration_s=duration_s,
            sample_interval_s=sample_interval_s,
        )
        node_values = [_smootherstep(index / count) for index in range(lower + 2)]
        completed = sum(
            0.5 * (node_values[index] + node_values[index + 1]) / count
            for index in range(lower)
        )
        start = node_values[lower]
        end = node_values[lower + 1]
        return completed + (start * local + 0.5 * (end - start) * local**2) / count
    raise ValueError(f"unsupported segment interpolation: {interpolation}")

def segment_interpolation_derivative(
    interpolation: str,
    fraction: float,
    *,
    duration_s: float = 1.0,
    sample_interval_s: float | None = None,
) -> float:
    """对归一化插值关于归一化时间求导。"""

    u = max(0.0, min(1.0, fraction))
    if interpolation == "linear":
        return 1.0
    if interpolation == "smootherstep":
        return 30.0 * u**2 * (u - 1.0) ** 2
    if interpolation == "sampled_smootherstep":
        lower, count, _ = _sampled_smootherstep_interval(
            u,
            duration_s=duration_s,
            sample_interval_s=sample_interval_s,
        )
        return count * (_smootherstep((lower + 1) / count) - _smootherstep(lower / count))
    raise ValueError(f"unsupported segment interpolation: {interpolation}")

@dataclass(frozen=True)
class MotionSegment:
    """一个给定的平面运动指令分段。

    ``heading_deg`` 是作业航迹坐标系中的速度矢量方位角回退值：0 deg 对应纵向/纵荡
    对齐的 ``+X``，90 deg 对应横向/横荡对齐的 ``+Y``，并非船舶陀螺艏向。
    四个速度分量字段齐全时，导缆点/犁端的直接实测速度分量优先。
    速度遵循分段插值合同，位置采用该插值的解析时间积分。
    """

    duration_s: float
    start_speed_mps: float
    end_speed_mps: float
    heading_deg: float
    interpolation: str = "linear"
    sample_interval_s: float | None = None
    start_velocity_x_mps: float | None = None
    start_velocity_y_mps: float | None = None
    end_velocity_x_mps: float | None = None
    end_velocity_y_mps: float | None = None

@dataclass(frozen=True)
class MotionSample:
    """作业航迹坐标系中的一个实测端点运动采样。

    ``x_m``/``y_m`` 是从船体/导航坐标转换后的导缆点或犁入口实测位置；
    可选速度分量是在同一坐标系中实测或滤波后的端点速度。
    """

    time_s: float
    x_m: float
    y_m: float
    z_m: float | None = None
    velocity_x_mps: float | None = None
    velocity_y_mps: float | None = None
    velocity_z_mps: float | None = None

@dataclass(frozen=True)
class SpeedSegment:
    """一个带显式插值方式的标量速度指令分段。"""

    duration_s: float
    start_speed_mps: float
    end_speed_mps: float
    interpolation: str = "linear"
    sample_interval_s: float | None = None

@dataclass(frozen=True)
class ScalarSample:
    """一个时间同步的标量输入采样。"""

    time_s: float
    value: float

@dataclass(frozen=True)
class CurrentSample:
    """作业坐标系中的一个时间同步水平水流速度采样。"""

    time_s: float
    velocity_x_mps: float
    velocity_y_mps: float
    interpolation: str = "cartesian_linear"
    speed_mps: float | None = None
    direction_unwrapped_deg: float | None = None

@dataclass(frozen=True)
class TimeHistoryPoint:
    """一组采样时刻的端点、几何和载荷结果。"""

    time_s: float
    top_tension_n: float
    has_contact: bool
    contact_transition_x_m: float | None
    contact_transition_y_m: float | None
    suspended_length_m: float
    iterations: int
    plough_x_m: float | None = None
    plough_y_m: float | None = None
    plough_z_m: float | None = None
    plough_inlet_tension_n: float | None = None
    contact_transition_tension_n: float | None = None
    plough_boundary_tension_n: float | None = None
    plough_adjacent_segment_tension_n: float | None = None
    plough_entry_angle_deg: float | None = None
    minimum_bend_radius_m: float | None = None
    minimum_bend_radius_node_index: int | None = None
    minimum_bend_radius_left_segment_m: float | None = None
    minimum_bend_radius_right_segment_m: float | None = None
    minimum_bend_radius_turn_angle_deg: float | None = None
    minimum_bend_radius_node_depth_m: float | None = None
    minimum_bend_radius_near_seabed: bool | None = None
    minimum_bend_radius_excluded_tail_nodes: int | None = None
    minimum_bend_radius_raw_m: float | None = None
    minimum_bend_radius_raw_node_index: int | None = None
    minimum_bend_radius_raw_left_segment_m: float | None = None
    minimum_bend_radius_raw_right_segment_m: float | None = None
    minimum_bend_radius_raw_turn_angle_deg: float | None = None
    minimum_bend_radius_raw_node_depth_m: float | None = None
    minimum_bend_radius_raw_near_seabed: bool | None = None
    material_suspended_length_m: float | None = None
    geometric_length_deficit_m: float | None = None
    contact_transition_arc_length_m: float | None = None
    free_span_material_length_m: float | None = None
    seabed_contact_length_m: float | None = None
    seabed_normal_reaction_n: float | None = None

@dataclass(frozen=True)
class TimeHistoryFramePoint:
    """动态三维帧中的一个缆线节点。

    坐标以船端/顶部节点为原点，``z_m`` 正方向表示向下的水深方向。
    """

    index: int
    x_m: float
    y_m: float
    z_m: float
    tension_n: float

@dataclass(frozen=True)
class TimeHistoryFrame:
    """从已知犁轨迹动态状态采样的一帧三维缆型。"""

    time_s: float
    points: list[TimeHistoryFramePoint]
    segment_tensions_n: tuple[float, ...] = ()
    boundary: str = "known_plough_trajectory"
    vessel_x_m: float | None = None
    vessel_y_m: float | None = None
    vessel_z_m: float | None = None
    plough_x_m: float | None = None
    plough_y_m: float | None = None
    plough_z_m: float | None = None
    minimum_bend_radius_m: float | None = None
    minimum_bend_radius_node_index: int | None = None
    minimum_bend_radius_left_segment_m: float | None = None
    minimum_bend_radius_right_segment_m: float | None = None
    minimum_bend_radius_turn_angle_deg: float | None = None
    minimum_bend_radius_node_depth_m: float | None = None
    minimum_bend_radius_near_seabed: bool | None = None
    minimum_bend_radius_excluded_tail_nodes: int | None = None
    minimum_bend_radius_raw_m: float | None = None
    minimum_bend_radius_raw_node_index: int | None = None
    minimum_bend_radius_raw_left_segment_m: float | None = None
    minimum_bend_radius_raw_right_segment_m: float | None = None
    minimum_bend_radius_raw_turn_angle_deg: float | None = None
    minimum_bend_radius_raw_node_depth_m: float | None = None
    minimum_bend_radius_raw_near_seabed: bool | None = None

@dataclass(frozen=True)
class DynamicCaseInput:
    """一次已知犁轨迹离线或实时仿真的输入。

    ``total_duration_s`` 是物理积分时域；``transition_duration_s`` 控制摘要
    变速段插值，并标记一个输出采样端点。显式分段或同步采样序列优先于摘要输入。
    """

    case_name: str
    current_speed_mps: float
    speed_change: str
    vessel_initial_speed_mps: float
    vessel_final_speed_mps: float
    transition_duration_s: float
    water_depth_m: float
    diameter_m: float = _DEFAULT_DYNAMIC_CABLE.diameter_m
    weight_air_n_per_m: float = _DEFAULT_DYNAMIC_CABLE.weight_air_n_per_m
    submerged_weight_n_per_m: float = _DEFAULT_DYNAMIC_CABLE.submerged_weight_n_per_m
    tangential_drag_coefficient: float = _DEFAULT_DYNAMIC_CABLE.tangential_drag_coefficient
    normal_drag_coefficient: float = _DEFAULT_DYNAMIC_CABLE.normal_drag_coefficient
    axial_stiffness_n: float = _DEFAULT_DYNAMIC_CABLE.axial_stiffness_n
    element_count: int = 32
    total_duration_s: float = 360.0
    current_direction_deg: float = 90.0
    current_bottom_speed_mps: float | None = None
    current_profile_exponent: float = 1.0
    integration_time_step_max_s: float | None = None
    payout_initial_speed_mps: float | None = None
    payout_final_speed_mps: float | None = None
    length_boundary_source: str = "known_plough_trajectory"
    vessel_initial_x_m: float = 0.0
    vessel_initial_y_m: float = 0.0
    vessel_heading_deg: float = 0.0
    plough_initial_x_m: float | None = None
    plough_initial_y_m: float | None = None
    plough_initial_z_m: float | None = None
    plough_speed_mps: float | None = None
    plough_exit_speed_mps: float | None = None
    plough_heading_deg: float | None = None
    initial_suspended_length_m: float | None = None
    min_bending_radius_m: float | None = None
    vessel_motion_segments: tuple[MotionSegment, ...] = ()
    plough_motion_segments: tuple[MotionSegment, ...] = ()
    vessel_motion_samples: tuple[MotionSample, ...] = ()
    plough_motion_samples: tuple[MotionSample, ...] = ()
    payout_speed_segments: tuple[SpeedSegment, ...] = ()
    payout_speed_samples: tuple[ScalarSample, ...] = ()
    plough_exit_speed_samples: tuple[ScalarSample, ...] = ()
    current_samples: tuple[CurrentSample, ...] = ()

@dataclass(frozen=True)
class TimeHistoryResult:
    """标量摘要及采样时程输出。"""

    case_name: str
    diameter_m: float
    weight_air_n_per_m: float
    submerged_weight_n_per_m: float
    tangential_drag_coefficient: float
    normal_drag_coefficient: float
    axial_stiffness_n: float
    current_speed_mps: float
    current_direction_deg: float
    speed_change: str
    vessel_initial_speed_mps: float
    vessel_final_speed_mps: float
    transition_duration_s: float
    total_duration_s: float
    water_depth_m: float
    element_count: int
    payout_initial_speed_mps: float
    payout_final_speed_mps: float
    length_boundary_source: str
    initial_suspended_length_m: float | None
    solver_id: str
    initial_tension_n: float
    extreme_tension_n: float
    steady_tension_n: float
    history: list[TimeHistoryPoint]
    frames: list[TimeHistoryFrame]
    current_bottom_speed_mps: float | None = None
    current_profile_exponent: float = 1.0
    plough_speed_mps: float | None = None
    plough_exit_speed_mps: float | None = None
    plough_exit_speed_source: str = "not_applicable"
    plough_inlet_tension_final_n: float | None = None
    contact_transition_tension_final_n: float | None = None
    plough_boundary_tension_final_n: float | None = None
    plough_adjacent_segment_tension_final_n: float | None = None
    plough_tension_status: str = "not_applicable"
    minimum_bend_radius_min_m: float | None = None
    minimum_bend_radius_limit_m: float | None = None
    minimum_bend_radius_margin_m: float | None = None
    minimum_bend_radius_status: str = "not_configured"
    minimum_bend_radius_time_s: float | None = None
    minimum_bend_radius_node_index: int | None = None
    minimum_bend_radius_left_segment_m: float | None = None
    minimum_bend_radius_right_segment_m: float | None = None
    minimum_bend_radius_turn_angle_deg: float | None = None
    minimum_bend_radius_node_depth_m: float | None = None
    minimum_bend_radius_near_seabed: bool | None = None
    minimum_bend_radius_excluded_tail_nodes: int | None = None
    minimum_bend_radius_raw_m: float | None = None
    minimum_bend_radius_raw_time_s: float | None = None
    minimum_bend_radius_raw_node_index: int | None = None
    minimum_bend_radius_raw_left_segment_m: float | None = None
    minimum_bend_radius_raw_right_segment_m: float | None = None
    minimum_bend_radius_raw_turn_angle_deg: float | None = None
    minimum_bend_radius_raw_node_depth_m: float | None = None
    minimum_bend_radius_raw_near_seabed: bool | None = None
    integration_time_step_max_s: float | None = None
    integration_time_step_min_s: float | None = None
    spatial_step_mean_m: float | None = None
    spatial_step_min_m: float | None = None
    xpbd_iterations_per_step: int | None = None
    xpbd_iterations_per_step_min: int | None = None
    xpbd_iterations_per_step_max: int | None = None
    xpbd_iteration_limit_per_solve: int | None = None
    axial_constraint_residual_max_m: float | None = None
    geometric_length_deficit_max_m: float | None = None
    geometric_length_deficit_final_m: float | None = None
    vessel_motion_segments: tuple[MotionSegment, ...] = ()
    plough_motion_segments: tuple[MotionSegment, ...] = ()
    vessel_motion_samples: tuple[MotionSample, ...] = ()
    plough_motion_samples: tuple[MotionSample, ...] = ()
    payout_speed_segments: tuple[SpeedSegment, ...] = ()

def cable_parameters_from_dynamic_case(case: DynamicCaseInput) -> CableParameters:
    """仅构造已知犁轨迹求解器实际使用的材料字段。"""

    return CableParameters(
        name="SIMULATION_INPUT",
        diameter_m=case.diameter_m,
        weight_air_n_per_m=case.weight_air_n_per_m,
        submerged_weight_n_per_m=case.submerged_weight_n_per_m,
        tangential_drag_coefficient=case.tangential_drag_coefficient,
        normal_drag_coefficient=case.normal_drag_coefficient,
        axial_stiffness_n=case.axial_stiffness_n,
        min_bending_radius_m=case.min_bending_radius_m,
    )


def validate_dynamic_case(
    case: DynamicCaseInput,
    *,
    allowed_length_boundary_sources: set[str] | None = None,
) -> None:
    """在求解器修改状态前校验一组完整工程输入。"""

    allowed_sources = allowed_length_boundary_sources or {"known_plough_trajectory"}
    if not case.case_name.strip():
        raise ValueError("case_name must not be empty")
    if case.speed_change not in {"steady", "accel", "decel"}:
        raise ValueError("speed_change must be steady, accel, or decel")
    if case.length_boundary_source not in allowed_sources:
        allowed = " or ".join(sorted(allowed_sources))
        raise ValueError(f"length_boundary_source must be {allowed}")

    positive_values = {
        "diameter_m": case.diameter_m,
        "weight_air_n_per_m": case.weight_air_n_per_m,
        "submerged_weight_n_per_m": case.submerged_weight_n_per_m,
        "axial_stiffness_n": case.axial_stiffness_n,
        "transition_duration_s": case.transition_duration_s,
        "total_duration_s": case.total_duration_s,
        "water_depth_m": case.water_depth_m,
    }
    for name, value in positive_values.items():
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f"{name} must be finite and greater than 0")
    if case.transition_duration_s > case.total_duration_s:
        raise ValueError("transition_duration_s must not exceed total_duration_s")
    if isinstance(case.element_count, bool) or not isinstance(case.element_count, int):
        raise ValueError("element_count must be an integer")
    if not 2 <= case.element_count <= 256:
        raise ValueError("element_count must be between 2 and 256")

    nonnegative_values = {
        "current_speed_mps": case.current_speed_mps,
        "vessel_initial_speed_mps": case.vessel_initial_speed_mps,
        "vessel_final_speed_mps": case.vessel_final_speed_mps,
        "tangential_drag_coefficient": case.tangential_drag_coefficient,
        "normal_drag_coefficient": case.normal_drag_coefficient,
    }
    optional_nonnegative_values = {
        "current_bottom_speed_mps": case.current_bottom_speed_mps,
        "payout_initial_speed_mps": case.payout_initial_speed_mps,
        "payout_final_speed_mps": case.payout_final_speed_mps,
        "plough_speed_mps": case.plough_speed_mps,
        "plough_exit_speed_mps": case.plough_exit_speed_mps,
    }
    optional_positive_values = {
        "integration_time_step_max_s": case.integration_time_step_max_s,
        "initial_suspended_length_m": case.initial_suspended_length_m,
        "min_bending_radius_m": case.min_bending_radius_m,
    }
    for name, value in nonnegative_values.items():
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"{name} must be finite and non-negative")
    for name, value in optional_nonnegative_values.items():
        if value is not None and (not math.isfinite(value) or value < 0.0):
            raise ValueError(f"{name} must be finite and non-negative")
    for name, value in optional_positive_values.items():
        if value is not None and (not math.isfinite(value) or value <= 0.0):
            raise ValueError(f"{name} must be finite and greater than 0 when provided")
    if case.current_profile_exponent <= 0.0 or not math.isfinite(case.current_profile_exponent):
        raise ValueError("current_profile_exponent must be finite and greater than 0")
    for name, value in {
        "vessel_initial_x_m": case.vessel_initial_x_m,
        "vessel_initial_y_m": case.vessel_initial_y_m,
    }.items():
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    for name, value in {
        "plough_initial_x_m": case.plough_initial_x_m,
        "plough_initial_y_m": case.plough_initial_y_m,
        "plough_initial_z_m": case.plough_initial_z_m,
    }.items():
        if value is not None and not math.isfinite(value):
            raise ValueError(f"{name} must be finite when provided")
    if case.plough_initial_z_m is not None and not 0.0 <= case.plough_initial_z_m <= case.water_depth_m:
        raise ValueError("plough_initial_z_m must be between 0 and water_depth_m")
    for name, value in {
        "current_direction_deg": case.current_direction_deg,
        "vessel_heading_deg": case.vessel_heading_deg,
        "plough_heading_deg": case.plough_heading_deg,
    }.items():
        if value is not None and (not math.isfinite(value) or not 0.0 <= value <= 360.0):
            raise ValueError(f"{name} must be finite and between 0 and 360")

    if case.speed_change == "accel" and case.vessel_final_speed_mps <= case.vessel_initial_speed_mps:
        raise ValueError("vessel_final_speed_mps must exceed vessel_initial_speed_mps for accel")
    if case.speed_change == "decel" and case.vessel_final_speed_mps >= case.vessel_initial_speed_mps:
        raise ValueError("vessel_final_speed_mps must be below vessel_initial_speed_mps for decel")
    if case.speed_change == "steady" and not math.isclose(
        case.vessel_final_speed_mps,
        case.vessel_initial_speed_mps,
        rel_tol=0.0,
        abs_tol=1.0e-12,
    ):
        raise ValueError("vessel speeds must match for steady")

    _validate_motion_segments("vessel_motion_segments", case.vessel_motion_segments)
    _validate_motion_segments("plough_motion_segments", case.plough_motion_segments)
    _validate_motion_samples("vessel_motion_samples", case.vessel_motion_samples)
    _validate_motion_samples(
        "plough_motion_samples",
        case.plough_motion_samples,
        water_depth_m=case.water_depth_m,
    )
    _validate_speed_segments("payout_speed_segments", case.payout_speed_segments)
    _validate_scalar_samples("payout_speed_samples", case.payout_speed_samples)
    _validate_scalar_samples("plough_exit_speed_samples", case.plough_exit_speed_samples)
    _validate_current_samples(case.current_samples)

    if case.length_boundary_source == "known_plough_trajectory":
        if not case.plough_motion_samples:
            for name, value in (
                ("plough_initial_x_m", case.plough_initial_x_m),
                ("plough_initial_y_m", case.plough_initial_y_m),
                ("plough_initial_z_m", case.plough_initial_z_m),
            ):
                if value is None or not math.isfinite(value):
                    raise ValueError(f"{name} is required and must be finite")
        if not case.plough_motion_segments and not case.plough_motion_samples:
            if case.plough_speed_mps is None or case.plough_heading_deg is None:
                raise ValueError(
                    "plough_speed_mps and plough_heading_deg are required without plough motion inputs"
                )
        if case.plough_initial_z_m is not None and not 0.0 <= case.plough_initial_z_m <= case.water_depth_m:
            raise ValueError("plough_initial_z_m must be between 0 and water_depth_m")
        if case.initial_suspended_length_m is None:
            raise ValueError("initial_suspended_length_m is required")

    validate_inferred_plough_exit_speed_domain(case)


def longitudinal_speeds_match(vessel_velocity_x_mps: float, plough_velocity_x_mps: float) -> bool:
    """判断两个 +X 边界速度是否仅存在浮点噪声差异。"""

    return math.isclose(
        vessel_velocity_x_mps,
        plough_velocity_x_mps,
        rel_tol=0.0,
        abs_tol=_LONGITUDINAL_SPEED_MATCH_ABS_TOLERANCE_MPS,
    )

def validate_inferred_plough_exit_speed_domain(case: DynamicCaseInput) -> None:
    """船端与犁端 +X 速度不同时要求显式提供 q_p。"""

    if case.plough_exit_speed_mps is not None or case.plough_exit_speed_samples:
        return
    for time_s in _longitudinal_speed_check_times(case):
        vessel_velocity_x = _boundary_longitudinal_velocity(case, "vessel", time_s)
        plough_velocity_x = _boundary_longitudinal_velocity(case, "plough", time_s)
        if longitudinal_speeds_match(vessel_velocity_x, plough_velocity_x):
            continue
        raise ValueError(
            "plough_exit_speed_mps is required when vessel and plough longitudinal +X "
            f"velocities differ at time_s={time_s:.12g} "
            f"(vessel={vessel_velocity_x:.12g} m/s, plough={plough_velocity_x:.12g} m/s)"
        )

def _longitudinal_speed_check_times(case: DynamicCaseInput) -> tuple[float, ...]:
    """返回足以验证 q_p 回退速度域合同的检查时刻。

    边界、采样节点及每个多项式区间内的六个点共同检查 smootherstep 指令，
    避免相同端点值掩盖区间内部的纵向速度不一致。
    """

    total_duration = max(0.0, float(case.total_duration_s))
    boundaries = {0.0, total_duration, min(max(float(case.transition_duration_s), 0.0), total_duration)}
    for segments in (case.vessel_motion_segments, case.plough_motion_segments):
        elapsed = 0.0
        for segment in segments:
            start = elapsed
            elapsed += float(segment.duration_s)
            if (
                segment.interpolation == "sampled_smootherstep"
                and segment.sample_interval_s is not None
            ):
                sample_count = round(segment.duration_s / segment.sample_interval_s)
                boundaries.update(
                    start + index * segment.sample_interval_s
                    for index in range(1, sample_count)
                    if 0.0 < start + index * segment.sample_interval_s < total_duration
                )
            if 0.0 < elapsed < total_duration:
                boundaries.add(elapsed)
    for samples in (case.vessel_motion_samples, case.plough_motion_samples):
        boundaries.update(
            float(sample.time_s)
            for sample in samples
            if 0.0 <= sample.time_s <= total_duration
        )
    ordered = sorted(boundaries)
    check_times = set(ordered)
    for boundary in ordered:
        if 0.0 < boundary < total_duration:
            check_times.add(math.nextafter(boundary, math.inf))
    # 每个合并区间内，所有受支持的纵向输入合同最高为五次多项式；
    # 因此六个不同的内部比较点可确认整个区间相等，而非只比较端点。
    for start, end in zip(ordered, ordered[1:]):
        if end <= start:
            continue
        check_times.update(start + (end - start) * index / 7.0 for index in range(1, 7))
    return tuple(sorted(check_times))

def _boundary_longitudinal_velocity(case: DynamicCaseInput, endpoint: str, time_s: float) -> float:
    samples = case.vessel_motion_samples if endpoint == "vessel" else case.plough_motion_samples
    if samples:
        return _sampled_longitudinal_velocity(samples, time_s)
    segments = case.vessel_motion_segments if endpoint == "vessel" else case.plough_motion_segments
    if segments:
        return _segmented_longitudinal_velocity(segments, time_s)
    if endpoint == "vessel":
        return math.cos(math.radians(case.vessel_heading_deg)) * _vessel_speed(case, time_s)
    return math.cos(math.radians(case.plough_heading_deg or 0.0)) * (case.plough_speed_mps or 0.0)

def _sampled_longitudinal_velocity(samples: tuple[MotionSample, ...], time_s: float) -> float:
    if len(samples) == 1:
        return float(samples[0].velocity_x_mps or 0.0)
    if time_s <= samples[0].time_s:
        if _motion_sample_has_horizontal_velocity(samples[0]):
            return float(samples[0].velocity_x_mps)
        return _sample_position_slope_x(samples[0], samples[1])
    for start, end in zip(samples, samples[1:]):
        if time_s <= end.time_s:
            if _motion_sample_has_horizontal_velocity(start) and _motion_sample_has_horizontal_velocity(end):
                fraction = (time_s - start.time_s) / max(end.time_s - start.time_s, _MIN_SPEED_MPS)
                return float(start.velocity_x_mps + (end.velocity_x_mps - start.velocity_x_mps) * fraction)
            return _sample_position_slope_x(start, end)
    if _motion_sample_has_horizontal_velocity(samples[-1]):
        return float(samples[-1].velocity_x_mps)
    return _sample_position_slope_x(samples[-2], samples[-1])

def _motion_sample_has_horizontal_velocity(sample: MotionSample) -> bool:
    return sample.velocity_x_mps is not None and sample.velocity_y_mps is not None

def _sample_position_slope_x(start: MotionSample, end: MotionSample) -> float:
    return float((end.x_m - start.x_m) / max(end.time_s - start.time_s, _MIN_SPEED_MPS))

def _segmented_longitudinal_velocity(segments: tuple[MotionSegment, ...], time_s: float) -> float:
    remaining = max(0.0, time_s)
    last_segment = None
    for segment in segments:
        last_segment = segment
        if remaining <= segment.duration_s:
            fraction = segment_interpolation_fraction(
                segment.interpolation,
                remaining / max(segment.duration_s, _MIN_SPEED_MPS),
                duration_s=segment.duration_s,
                sample_interval_s=segment.sample_interval_s,
            )
            start_x, end_x = _motion_segment_longitudinal_endpoints(segment)
            return float(start_x + (end_x - start_x) * fraction)
        remaining -= segment.duration_s
    if last_segment is None:
        return 0.0
    return _motion_segment_longitudinal_endpoints(last_segment)[1]

def _motion_segment_longitudinal_endpoints(segment: MotionSegment) -> tuple[float, float]:
    components = (
        segment.start_velocity_x_mps,
        segment.start_velocity_y_mps,
        segment.end_velocity_x_mps,
        segment.end_velocity_y_mps,
    )
    if all(value is not None for value in components):
        return float(segment.start_velocity_x_mps), float(segment.end_velocity_x_mps)
    route_x = math.cos(math.radians(segment.heading_deg))
    return route_x * segment.start_speed_mps, route_x * segment.end_speed_mps

def _validate_motion_segments(name: str, segments: tuple[MotionSegment, ...]) -> None:
    """校验完整的平面速度指令及插值元数据。"""

    for index, segment in enumerate(segments):
        scalar_values = (
            segment.duration_s,
            segment.start_speed_mps,
            segment.end_speed_mps,
            segment.heading_deg,
        )
        optional_components = (
            segment.start_velocity_x_mps,
            segment.start_velocity_y_mps,
            segment.end_velocity_x_mps,
            segment.end_velocity_y_mps,
        )
        if any(not math.isfinite(value) for value in scalar_values):
            raise ValueError(f"{name}[{index}] scalar fields must be finite")
        if any(value is not None and not math.isfinite(value) for value in optional_components):
            raise ValueError(f"{name}[{index}] velocity components must be finite")
        if any(value is not None for value in optional_components) and not all(
            value is not None for value in optional_components
        ):
            raise ValueError(f"{name}[{index}] must provide all four horizontal velocity components")
        if segment.duration_s <= 0.0:
            raise ValueError(f"{name}[{index}].duration_s must be greater than 0")
        if segment.start_speed_mps < 0.0 or segment.end_speed_mps < 0.0:
            raise ValueError(f"{name}[{index}] speeds must be greater than or equal to 0")
        if not 0.0 <= segment.heading_deg <= 360.0:
            raise ValueError(f"{name}[{index}].heading_deg must be between 0 and 360")
        if segment.interpolation not in _SEGMENT_INTERPOLATIONS:
            raise ValueError(f"{name}[{index}].interpolation is not supported")
        _validate_segment_sampling(name, index, segment)

def _validate_motion_samples(
    name: str,
    samples: tuple[MotionSample, ...],
    *,
    water_depth_m: float | None = None,
) -> None:
    """校验按公共时基严格排序的端点采样。"""

    previous_time: float | None = None
    for index, sample in enumerate(samples):
        required_values = (sample.time_s, sample.x_m, sample.y_m)
        optional_values = (
            sample.z_m,
            sample.velocity_x_mps,
            sample.velocity_y_mps,
            sample.velocity_z_mps,
        )
        if any(not math.isfinite(value) for value in required_values):
            raise ValueError(f"{name}[{index}] required fields must be finite")
        if any(value is not None and not math.isfinite(value) for value in optional_values):
            raise ValueError(f"{name}[{index}] optional fields must be finite")
        velocity_values = optional_values[1:]
        if any(value is not None for value in velocity_values) and not all(
            value is not None for value in velocity_values
        ):
            raise ValueError(f"{name}[{index}] must provide all three velocity components")
        if sample.time_s < 0.0:
            raise ValueError(f"{name}[{index}].time_s must be greater than or equal to 0")
        if previous_time is not None and sample.time_s <= previous_time:
            raise ValueError(f"{name}[{index}].time_s must be strictly increasing")
        if index == 0 and not math.isclose(sample.time_s, 0.0, abs_tol=1.0e-9):
            raise ValueError(f"{name}[0].time_s must be 0")
        previous_time = sample.time_s
        if sample.z_m is not None and water_depth_m is not None and (sample.z_m < 0.0 or sample.z_m > water_depth_m):
            raise ValueError(f"{name}[{index}].z_m must be between 0 and water_depth_m")

def _validate_current_samples(samples: tuple[CurrentSample, ...]) -> None:
    """校验海流采样及其笛卡尔/极坐标双重表示。"""

    previous_time: float | None = None
    for index, sample in enumerate(samples):
        if not math.isfinite(sample.time_s) or sample.time_s < 0.0:
            raise ValueError(f"current_samples[{index}].time_s must be finite and non-negative")
        if index == 0 and not math.isclose(sample.time_s, 0.0, abs_tol=1.0e-9):
            raise ValueError("current_samples[0].time_s must be 0")
        if previous_time is not None and sample.time_s <= previous_time:
            raise ValueError(f"current_samples[{index}].time_s must be strictly increasing")
        previous_time = sample.time_s
        if not math.isfinite(sample.velocity_x_mps) or not math.isfinite(sample.velocity_y_mps):
            raise ValueError(f"current_samples[{index}] velocity components must be finite")
        if sample.interpolation not in {"cartesian_linear", "polar_unwrapped"}:
            raise ValueError(f"current_samples[{index}].interpolation is not supported")
        if sample.interpolation == "polar_unwrapped":
            if sample.speed_mps is None or sample.direction_unwrapped_deg is None:
                raise ValueError(f"current_samples[{index}] polar fields are required")
            if not math.isfinite(sample.speed_mps) or not math.isfinite(sample.direction_unwrapped_deg):
                raise ValueError(f"current_samples[{index}] polar fields must be finite")
            if sample.speed_mps < 0.0:
                raise ValueError(f"current_samples[{index}].speed_mps must be non-negative")
            radians = math.radians(sample.direction_unwrapped_deg)
            expected_x = sample.speed_mps * math.cos(radians)
            expected_y = sample.speed_mps * math.sin(radians)
            if not math.isclose(
                sample.velocity_x_mps,
                expected_x,
                rel_tol=0.0,
                abs_tol=_CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS,
            ) or not math.isclose(
                sample.velocity_y_mps,
                expected_y,
                rel_tol=0.0,
                abs_tol=_CURRENT_POLAR_COMPONENT_ABS_TOLERANCE_MPS,
            ):
                raise ValueError(
                    f"current_samples[{index}] Cartesian components must match "
                    "speed*cos/sin(direction)"
                )


def _validate_scalar_samples(name: str, samples: tuple[ScalarSample, ...]) -> None:
    previous_time: float | None = None
    for index, sample in enumerate(samples):
        if not math.isfinite(sample.time_s) or sample.time_s < 0.0:
            raise ValueError(f"{name}[{index}].time_s must be finite and non-negative")
        if index == 0 and not math.isclose(sample.time_s, 0.0, abs_tol=1.0e-9):
            raise ValueError(f"{name}[0].time_s must be 0")
        if previous_time is not None and sample.time_s <= previous_time:
            raise ValueError(f"{name}[{index}].time_s must be strictly increasing")
        if not math.isfinite(sample.value) or sample.value < 0.0:
            raise ValueError(f"{name}[{index}].value must be finite and non-negative")
        previous_time = sample.time_s

def _validate_speed_segments(name: str, segments: tuple[SpeedSegment, ...]) -> None:
    for index, segment in enumerate(segments):
        if any(
            not math.isfinite(value)
            for value in (segment.duration_s, segment.start_speed_mps, segment.end_speed_mps)
        ):
            raise ValueError(f"{name}[{index}] scalar fields must be finite")
        if segment.duration_s <= 0.0:
            raise ValueError(f"{name}[{index}].duration_s must be greater than 0")
        if segment.start_speed_mps < 0.0 or segment.end_speed_mps < 0.0:
            raise ValueError(f"{name}[{index}] speeds must be greater than or equal to 0")
        if segment.interpolation not in _SEGMENT_INTERPOLATIONS:
            raise ValueError(f"{name}[{index}].interpolation is not supported")
        _validate_segment_sampling(name, index, segment)

def _validate_segment_sampling(name: str, index: int, segment: MotionSegment | SpeedSegment) -> None:
    if segment.interpolation != "sampled_smootherstep":
        return
    if (
        segment.sample_interval_s is None
        or not math.isfinite(segment.sample_interval_s)
        or segment.sample_interval_s <= 0.0
    ):
        raise ValueError(f"{name}[{index}].sample_interval_s must be positive")
    interval_count = round(segment.duration_s / segment.sample_interval_s)
    if interval_count < 1 or not math.isclose(
        interval_count * segment.sample_interval_s,
        segment.duration_s,
        rel_tol=0.0,
        abs_tol=1.0e-9,
    ):
        raise ValueError(f"{name}[{index}].sample_interval_s must divide duration_s")

def _vessel_speed(case: DynamicCaseInput, time_s: float) -> float:
    """返回指定采样时刻的线性给定船速。"""

    if time_s >= case.transition_duration_s:
        return case.vessel_final_speed_mps
    fraction = max(0.0, min(1.0, time_s / max(case.transition_duration_s, 1.0e-12)))
    return case.vessel_initial_speed_mps + (case.vessel_final_speed_mps - case.vessel_initial_speed_mps) * fraction


def build_sample_times(case: DynamicCaseInput, points: int) -> list[float]:
    """覆盖完整物理时域，并在可行时保留转变段终点。"""

    times = [case.total_duration_s * index / (points - 1) for index in range(points)]
    if 0.0 < case.transition_duration_s < case.total_duration_s and points > 2:
        closest = min(
            range(1, points - 1),
            key=lambda index: abs(times[index] - case.transition_duration_s),
        )
        times[closest] = case.transition_duration_s
    return times
