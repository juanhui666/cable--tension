"""生产使用的节点坐标动态铺缆模型。

维护结构：

* 材料单元辅助函数通过运动的导缆点/犁端边界输运参考长度、动量、动能和轴向应变矩；
* 载荷组装在当前 P1 缆线状态上计算重力、Morison 载荷及可选接触力；
* 运行时 API 初始化、推进并采样一个持久状态；
* 每个动态步先对 ALE 网格重分区，再预测无约束运动，随后施加轴向与海床约束并恢复反力；
* 重映射辅助函数严格守恒参考长度和动量，同时重建 P1 动能并记录动能重映射误差；
  诊断函数将求解状态转换为工程输出。

节点坐标和网格速度是主要未知量，材料参考长度单独跟踪。每个已接收步的活动跨距满足
``L_next = L_prev + q_f*dt - q_p*dt``。重网格可以移动节点或改变节点数量，
但不得生成额外材料，也不得替换给定的端点位置。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from .axial_constraints import axial_constraint_residual_m, solve_global_axial_constraint_step
from .contact import (
    SEABED_CONTACT_TOLERANCE_M,
    SegmentContactProfile,
    build_segment_contact_profile,
    seabed_friction,
)
from .simulation import (
    TimeHistoryFrame,
    TimeHistoryFramePoint,
    TimeHistoryPoint,
    TimeHistoryResult,
    SOLVER_ID,
    build_sample_times,
    cable_parameters_from_dynamic_case,
    segment_interpolation_derivative,
    segment_interpolation_fraction,
    segment_interpolation_integral,
    validate_dynamic_case,
)
from .geometry import Vector3, segment_vectors
from .hydrodynamics import current_at, morison_drag
from .materials import (
    AXIAL_ADDED_MASS_COEFFICIENT,
    NORMAL_ADDED_MASS_COEFFICIENT,
    CableParameters,
    StepConditions,
)


_SEAWATER_DENSITY_KG_M3 = 1025.0
_GRAVITY_MPS2 = 9.8
_MIN_MASS = 1.0e-12
_MIN_LENGTH = 1.0e-12
_SEABED_CONTACT_TOLERANCE_M = SEABED_CONTACT_TOLERANCE_M
_SEABED_FRICTION_COEFFICIENT = 0.6
_MAX_NODE_CFL_FRACTION = 0.25
_MIN_INTERNAL_TIME_STEP_S = 1.0e-4
_SEGMENT_SPACING_FLOOR_FRACTION = 0.25
_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES = 2
_KNOWN_PLOUGH_XPBD_MIN_ITERATIONS = 1
_KNOWN_PLOUGH_XPBD_ITERATIONS = 100
_KNOWN_PLOUGH_AXIAL_RESIDUAL_TOLERANCE_M = 1.0e-10
_REMESH_PROJECTION_MAX_ITERATIONS = 6000
_REMESH_PROJECTION_REL_TOLERANCE = 1.0e-12


class CableGeometryInfeasibleError(RuntimeError):
    """请求的端点几何与活动缆长无法形成可行离散缆型。"""


# 状态记录将网格运动学与守恒材料数据分开保存。
@dataclass(frozen=True)
class DynamicLayingState:
    """一次基于节点的 ALE 仿真状态。

    ``positions`` 和 ``velocities`` 描述计算网格；``rest_lengths_m`` 与材料控制体
    描述网格承载的参考材料。内部网格重分区时，端点节点仍保持给定值。
    """

    time_s: float
    positions: tuple[Vector3, ...]
    velocities: tuple[Vector3, ...]
    rest_lengths_m: tuple[float, ...]
    paid_length_m: float
    laid_length_m: float
    contact_flags: tuple[bool, ...]
    length_lambdas_n_s2: tuple[float, ...] = ()
    contact_lambdas_n_s2: tuple[float, ...] = ()
    segment_tensions_n: tuple[float, ...] = ()
    length_constraint_reactions_n: tuple[float, ...] = ()
    contact_normal_reactions_n: tuple[float, ...] = ()
    payout_buffer_m: float = 0.0
    laydown_buffer_m: float = 0.0
    laid_segment_lengths_m: tuple[float, ...] = ()
    material_suspended_length_m: float = 0.0
    known_plough_material_control_volume: _KnownPloughMaterialControlVolume | None = None
    geometric_length_deficit_m: float = 0.0
    axial_solve_iterations: int = 0
    axial_constraint_residual_m: float = 0.0
    material_remap_energy_error_per_linear_density_m3_s2: float = 0.0
    material_remap_energy_error_cumulative_per_linear_density_m3_s2: float = 0.0
    # 固定端点/全局动量投影引入的数值能量增量。
    material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2: float = 0.0

    @property
    def suspended_length_m(self) -> float:
        """悬空长度遵循放缆连续性状态。"""

        return max(0.0, self.paid_length_m - self.laid_length_m)


@dataclass(frozen=True)
class _MaterialCellIntegral:
    """一个按结构线密度归一化的材料单元积分量。

    长度单位为 m，单位线密度动量为 m^2/s，单位线密度总动能为 m^3/s^2，
    两个轴向应变矩的单位均为 m。第一轴向矩带符号，使 L+A 等于当前 P1 分段长度。
    P/K 是重分区的守恒输入和审计量；动态速度状态采用 P1 表示。
    """

    length_m: float = 0.0
    momentum_per_linear_density_m2_s: Vector3 = (0.0, 0.0, 0.0)
    kinetic_energy_per_linear_density_m3_s2: float = 0.0
    axial_strain_integral_m: float = 0.0
    axial_strain_squared_integral_m: float = 0.0


@dataclass(frozen=True)
class _MaterialCellTransport:
    cells: tuple[_MaterialCellIntegral, ...]
    outgoing_cell: _MaterialCellIntegral


@dataclass(frozen=True)
class _EndpointMaterialCutCell:
    """一个按结构线密度归一化的端点材料切片。"""

    length_m: float = 0.0
    momentum_per_linear_density_m2_s: Vector3 = (0.0, 0.0, 0.0)
    kinetic_energy_per_linear_density_m3_s2: float = 0.0
    axial_strain_integral_m: float = 0.0
    axial_strain_squared_integral_m: float = 0.0


@dataclass(frozen=True)
class _KnownPloughMaterialControlVolume:
    """两端切割单元及活动跨段的分布材料积分量。"""

    fairlead_cumulative_inflow_m: float = 0.0
    plough_cumulative_outflow_m: float = 0.0
    fairlead_cut_cell: _EndpointMaterialCutCell = _EndpointMaterialCutCell()
    plough_cut_cell: _EndpointMaterialCutCell = _EndpointMaterialCutCell()
    material_cells: tuple[_MaterialCellIntegral, ...] = ()
    fairlead_cumulative_integral: _MaterialCellIntegral = _MaterialCellIntegral()
    plough_cumulative_integral: _MaterialCellIntegral = _MaterialCellIntegral()




@dataclass(frozen=True)
class _MaterialSliceIntegral:
    """一个材料切片的积分状态，单位分别为 kg、kg m/s 和 J。"""

    mass_kg: float
    momentum_kg_mps: Vector3
    kinetic_energy_j: float


@dataclass(frozen=True)
class _BendRadiusDiagnostic:
    radius_m: float
    node_index: int | None = None
    left_segment_m: float | None = None
    right_segment_m: float | None = None
    turn_angle_deg: float | None = None
    node_depth_m: float | None = None
    near_seabed: bool | None = None


@dataclass
class KnownPloughRuntime:
    """增量求解的持久状态及定长数值指标。"""

    state: DynamicLayingState
    cable: CableParameters
    time_s: float
    dt_max_s: float
    steps: int
    integration_time_step_min_s: float | None
    integration_time_step_max_s: float | None
    axial_iterations_min: int | None
    axial_iterations_max: int | None
    axial_constraint_residual_max_m: float | None
    bend_radius_diagnostic: _BendRadiusDiagnostic
    bend_radius_min_m: float
    bend_radius_time_s: float | None
    raw_bend_radius_diagnostic: _BendRadiusDiagnostic
    raw_bend_radius_min_m: float
    raw_bend_radius_time_s: float | None


@dataclass(frozen=True)
class KnownPloughSample:
    """由持久已知犁轨迹状态生成的一组输出采样。"""

    point: TimeHistoryPoint
    frame: TimeHistoryFrame


def _optional_min(current, value):
    return value if current is None else min(current, value)


def _optional_max(current, value):
    return value if current is None else max(current, value)


def _validate_material_cell_integral(cell: _MaterialCellIntegral) -> None:
    if not math.isfinite(cell.length_m) or cell.length_m < 0.0:
        raise ValueError("material-cell length must be finite and non-negative")
    if not all(math.isfinite(component) for component in cell.momentum_per_linear_density_m2_s):
        raise ValueError("material-cell momentum integral must be finite")
    if (
        not math.isfinite(cell.kinetic_energy_per_linear_density_m3_s2)
        or cell.kinetic_energy_per_linear_density_m3_s2 < 0.0
    ):
        raise ValueError("material-cell kinetic-energy integral must be finite and non-negative")
    if not math.isfinite(cell.axial_strain_integral_m):
        raise ValueError("material-cell axial-strain integral must be finite")
    if (
        not math.isfinite(cell.axial_strain_squared_integral_m)
        or cell.axial_strain_squared_integral_m < 0.0
    ):
        raise ValueError(
            "material-cell axial-strain-squared integral must be finite and non-negative"
        )


def _validate_material_cell_moment_feasibility(cell: _MaterialCellIntegral) -> None:
    """校验一个归一化材料单元状态的 Cauchy 界。"""

    _validate_material_cell_integral(cell)
    momentum_squared = _dot(
        cell.momentum_per_linear_density_m2_s,
        cell.momentum_per_linear_density_m2_s,
    )
    kinetic_bound = (
        2.0 * cell.length_m * cell.kinetic_energy_per_linear_density_m3_s2
    )
    kinetic_tolerance = 64.0 * math.ulp(max(momentum_squared, kinetic_bound))
    if momentum_squared > kinetic_bound + kinetic_tolerance:
        raise RuntimeError("material-cell momentum and kinetic moments are infeasible")
    strain_squared = cell.axial_strain_integral_m**2
    strain_bound = cell.length_m * cell.axial_strain_squared_integral_m
    strain_tolerance = 64.0 * math.ulp(max(strain_squared, strain_bound))
    if strain_squared > strain_bound + strain_tolerance:
        raise RuntimeError("material-cell axial-strain moments are infeasible")


def _material_cell_mean_signed_geometric_axial_strain(
    cell: _MaterialCellIntegral,
) -> float:
    _validate_material_cell_integral(cell)
    if cell.length_m <= _MIN_LENGTH:
        return 0.0
    mean_strain = cell.axial_strain_integral_m / cell.length_m
    if mean_strain <= -1.0:
        raise RuntimeError(
            "material-cell mean signed geometric axial strain has non-positive P1 length"
        )
    return mean_strain


def _scaled_material_cell(
    cell: _MaterialCellIntegral,
    length_m: float,
) -> _MaterialCellIntegral:
    """返回均匀材料切片，并按长度缩放单元的全部广延量矩。"""

    if length_m <= _MIN_LENGTH:
        return _MaterialCellIntegral()
    if cell.length_m <= _MIN_LENGTH or length_m > cell.length_m + _MIN_LENGTH:
        raise ValueError("material-cell slice length lies outside its parent cell")
    fraction = min(1.0, max(0.0, length_m / cell.length_m))
    return _MaterialCellIntegral(
        length_m=length_m,
        momentum_per_linear_density_m2_s=_mul(
            cell.momentum_per_linear_density_m2_s,
            fraction,
        ),
        kinetic_energy_per_linear_density_m3_s2=(
            cell.kinetic_energy_per_linear_density_m3_s2 * fraction
        ),
        axial_strain_integral_m=cell.axial_strain_integral_m * fraction,
        axial_strain_squared_integral_m=(
            cell.axial_strain_squared_integral_m * fraction
        ),
    )


def _sum_material_cells(
    cells: tuple[_MaterialCellIntegral, ...] | list[_MaterialCellIntegral],
) -> _MaterialCellIntegral:
    """采用稳定的浮点归约求和单元广延量矩。"""

    return _MaterialCellIntegral(
        length_m=math.fsum(cell.length_m for cell in cells),
        momentum_per_linear_density_m2_s=(
            math.fsum(cell.momentum_per_linear_density_m2_s[0] for cell in cells),
            math.fsum(cell.momentum_per_linear_density_m2_s[1] for cell in cells),
            math.fsum(cell.momentum_per_linear_density_m2_s[2] for cell in cells),
        ),
        kinetic_energy_per_linear_density_m3_s2=math.fsum(
            cell.kinetic_energy_per_linear_density_m3_s2 for cell in cells
        ),
        axial_strain_integral_m=math.fsum(
            cell.axial_strain_integral_m for cell in cells
        ),
        axial_strain_squared_integral_m=math.fsum(
            cell.axial_strain_squared_integral_m for cell in cells
        ),
    )


def _add_material_cells(
    left: _MaterialCellIntegral,
    right: _MaterialCellIntegral,
) -> _MaterialCellIntegral:
    return _sum_material_cells((left, right))


def _subtract_material_cell(
    cell: _MaterialCellIntegral,
    removed: _MaterialCellIntegral,
) -> _MaterialCellIntegral:
    """从母材料单元中扣除已经积分的切片。"""

    remaining_length = cell.length_m - removed.length_m
    if remaining_length <= _MIN_LENGTH:
        return _MaterialCellIntegral()
    return _MaterialCellIntegral(
        length_m=remaining_length,
        momentum_per_linear_density_m2_s=_sub(
            cell.momentum_per_linear_density_m2_s,
            removed.momentum_per_linear_density_m2_s,
        ),
        kinetic_energy_per_linear_density_m3_s2=(
            cell.kinetic_energy_per_linear_density_m3_s2
            - removed.kinetic_energy_per_linear_density_m3_s2
        ),
        axial_strain_integral_m=(
            cell.axial_strain_integral_m - removed.axial_strain_integral_m
        ),
        axial_strain_squared_integral_m=max(
            0.0,
            cell.axial_strain_squared_integral_m
            - removed.axial_strain_squared_integral_m,
        ),
    )


# 守恒材料输运与分布载荷组装。
def _transport_material_cell_integrals(
    cells: tuple[_MaterialCellIntegral, ...],
    *,
    incoming_cell: _MaterialCellIntegral,
    outgoing_length_m: float,
    target_cell_lengths_m: tuple[float, ...],
    outgoing_cell_override: _MaterialCellIntegral | None = None,
) -> _MaterialCellTransport:
    """将开放边界单元积分量守恒输运到目标网格。"""

    for cell in (*cells, incoming_cell):
        _validate_material_cell_integral(cell)
    if outgoing_cell_override is not None:
        _validate_material_cell_integral(outgoing_cell_override)
    if not math.isfinite(outgoing_length_m) or outgoing_length_m < 0.0:
        raise ValueError("outgoing material length must be finite and non-negative")
    if any(
        not math.isfinite(length) or length <= _MIN_LENGTH
        for length in target_cell_lengths_m
    ):
        raise ValueError("target material-cell lengths must be finite and positive")
    if (
        incoming_cell.length_m <= _MIN_LENGTH
        and outgoing_length_m <= _MIN_LENGTH
        and tuple(cell.length_m for cell in cells) == target_cell_lengths_m
    ):
        return _MaterialCellTransport(cells=cells, outgoing_cell=_MaterialCellIntegral())

    material = list(cells)
    if incoming_cell.length_m > _MIN_LENGTH:
        material.insert(0, incoming_cell)
    available_length = math.fsum(cell.length_m for cell in material)
    if outgoing_length_m > available_length + _MIN_LENGTH:
        raise ValueError("outgoing material length exceeds the available material domain")

    outgoing_parts: list[_MaterialCellIntegral] = []
    remaining_outflow = outgoing_length_m
    if outgoing_cell_override is not None:
        tolerance = 64.0 * math.ulp(
            max(abs(outgoing_length_m), abs(outgoing_cell_override.length_m))
        )
        if abs(outgoing_cell_override.length_m - outgoing_length_m) > tolerance:
            raise ValueError("outgoing material override length does not match outflow")
        if outgoing_length_m > material[-1].length_m + _MIN_LENGTH:
            raise ValueError("exact outgoing material slice spans more than one cell")
        if outgoing_length_m > _MIN_LENGTH:
            outgoing_parts.append(outgoing_cell_override)
            remaining = _subtract_material_cell(material[-1], outgoing_cell_override)
            if remaining.length_m <= _MIN_LENGTH:
                material.pop()
            else:
                material[-1] = remaining
        remaining_outflow = 0.0
    while remaining_outflow > _MIN_LENGTH:
        if not material:
            raise RuntimeError("material transport exhausted the source domain")
        source = material[-1]
        take = min(remaining_outflow, source.length_m)
        removed = _scaled_material_cell(source, take)
        outgoing_parts.append(removed)
        remaining = _subtract_material_cell(source, removed)
        if remaining.length_m <= _MIN_LENGTH:
            material.pop()
        else:
            material[-1] = remaining
        remaining_outflow -= take

    expected_target_length = available_length - outgoing_length_m
    actual_target_length = math.fsum(target_cell_lengths_m)
    length_scale = max(1.0, abs(expected_target_length), abs(actual_target_length))
    length_tolerance = 64.0 * math.ulp(length_scale)
    if abs(actual_target_length - expected_target_length) > length_tolerance:
        raise ValueError("target material-cell lengths do not close the boundary length balance")

    transported: list[_MaterialCellIntegral] = []
    source_index = 0
    for target_length in target_cell_lengths_m:
        target_parts: list[_MaterialCellIntegral] = []
        remaining_target = target_length
        while remaining_target > _MIN_LENGTH:
            if source_index >= len(material):
                raise RuntimeError("target material grid exceeds the transported source domain")
            source = material[source_index]
            take = min(remaining_target, source.length_m)
            part = _scaled_material_cell(source, take)
            target_parts.append(part)
            remaining_source = _subtract_material_cell(source, part)
            if remaining_source.length_m <= _MIN_LENGTH:
                source_index += 1
            else:
                material[source_index] = remaining_source
            remaining_target -= take
        aggregate = _sum_material_cells(target_parts)
        transported.append(replace(aggregate, length_m=target_length))

    if any(cell.length_m > length_tolerance for cell in material[source_index:]):
        raise RuntimeError("transported material remains after filling the target grid")
    return _MaterialCellTransport(
        cells=tuple(transported),
        outgoing_cell=_sum_material_cells(outgoing_parts),
    )






def _integrate_linear_material_tail_slice(
    *,
    structural_linear_density_kg_m: float,
    parent_length_m: float,
    exit_length_m: float,
    left_material_velocity_mps: Vector3,
    right_material_velocity_mps: Vector3,
) -> _MaterialSliceIntegral:
    """对尾端流出切片内线性变化的速度进行精确积分。"""

    density = float(structural_linear_density_kg_m)
    parent_length = float(parent_length_m)
    exit_length = float(exit_length_m)
    if not math.isfinite(density) or density <= 0.0:
        raise ValueError("structural_linear_density_kg_m must be finite and positive")
    if not math.isfinite(parent_length) or parent_length <= 0.0:
        raise ValueError("parent_length_m must be finite and positive")
    if not math.isfinite(exit_length) or exit_length < 0.0 or exit_length > parent_length:
        raise ValueError("exit_length_m must lie in [0, parent_length_m]")
    if not all(
        math.isfinite(component)
        for velocity in (left_material_velocity_mps, right_material_velocity_mps)
        for component in velocity
    ):
        raise ValueError("material velocities must be finite")
    if exit_length <= _MIN_LENGTH:
        return _MaterialSliceIntegral(0.0, (0.0, 0.0, 0.0), 0.0)
    fraction = exit_length / parent_length
    slice_left_velocity = _add(
        right_material_velocity_mps,
        _mul(
            _sub(left_material_velocity_mps, right_material_velocity_mps),
            fraction,
        ),
    )
    momentum_per_density = _mul(
        _add(slice_left_velocity, right_material_velocity_mps),
        0.5 * exit_length,
    )
    kinetic_energy_per_density = _linear_material_cell_kinetic_energy(
        slice_left_velocity,
        right_material_velocity_mps,
        exit_length,
    )
    return _MaterialSliceIntegral(
        mass_kg=density * exit_length,
        momentum_kg_mps=_mul(momentum_per_density, density),
        kinetic_energy_j=density * kinetic_energy_per_density,
    )




def _p1_morison_segment_nodal_forces(
    case: StepConditions,
    segment,
    *,
    left_material_velocity_mps: Vector3,
    right_material_velocity_mps: Vector3,
) -> tuple[Vector3, Vector3]:
    """采用三点 Gauss 积分和 P1 形函数积分 Morison 阻力。"""

    left_force = (0.0, 0.0, 0.0)
    right_force = (0.0, 0.0, 0.0)
    coordinates_weights = (
        (0.5 * (1.0 - math.sqrt(3.0 / 5.0)), 5.0 / 18.0),
        (0.5, 4.0 / 9.0),
        (0.5 * (1.0 + math.sqrt(3.0 / 5.0)), 5.0 / 18.0),
    )
    for coordinate, weight in coordinates_weights:
        left_shape = 1.0 - coordinate
        right_shape = coordinate
        depth = left_shape * segment.start[2] + right_shape * segment.end[2]
        material_velocity = _add(
            _mul(left_material_velocity_mps, left_shape),
            _mul(right_material_velocity_mps, right_shape),
        )
        water_velocity = current_at(
            depth_m=depth,
            water_depth_m=case.water_depth_m,
            current_surface_mps=case.current_surface_mps,
            current_bottom_mps=case.current_bottom_mps,
            current_profile_exponent=case.current_profile_exponent,
            current_direction_deg=case.current_direction_deg,
        )
        gauss_drag = morison_drag(
            seawater_density=_SEAWATER_DENSITY_KG_M3,
            diameter_m=case.cable.diameter_m,
            segment_length_m=weight * segment.length_m,
            tangent=segment.tangent,
            relative_velocity=_sub(material_velocity, water_velocity),
            tangential_coefficient=case.cable.tangential_drag_coefficient,
            normal_coefficient=case.cable.normal_drag_coefficient,
        )
        left_force = _add(left_force, _mul(gauss_drag, left_shape))
        right_force = _add(right_force, _mul(gauss_drag, right_shape))
    return left_force, right_force


def compute_forces(
    case: StepConditions,
    state: DynamicLayingState,
    *,
    seabed_depth_m: float | None = None,
    payout_speed_mps: float | None = None,
    plough_exit_speed_mps: float | None = None,
    seabed_friction_coefficient: float = _SEABED_FRICTION_COEFFICIENT,
    include_axial_tension: bool = True,
) -> tuple[Vector3, ...]:
    """根据轴向张力、重量、阻力和接触力计算节点载荷。"""

    _validate_state(state)
    payout_speed = (
        payout_speed_mps
        if payout_speed_mps is not None
        else case.payout_speed_mps
        if case.payout_speed_mps is not None
        else 0.0
    )
    forces = [(0.0, 0.0, 0.0) for _ in state.positions]
    segments = segment_vectors(state.positions)
    material_control = state.known_plough_material_control_volume
    material_node_velocities = (
        _known_plough_node_material_velocities(
            positions=state.positions,
            grid_velocities=state.velocities,
            rest_lengths_m=state.rest_lengths_m,
            fairlead_speed_mps=payout_speed,
            plough_speed_mps=plough_exit_speed_mps,
        )
        if material_control is not None and material_control.material_cells
        else ()
    )
    segment_material_speeds = _segment_material_flow_speeds(
        state.rest_lengths_m,
        fairlead_speed_mps=payout_speed,
        plough_speed_mps=plough_exit_speed_mps,
    )
    for segment, rest_length, material_speed in zip(
        segments,
        state.rest_lengths_m,
        segment_material_speeds,
    ):
        left = segment.index
        right = left + 1
        if include_axial_tension:
            axial_tension = _segment_tension(case, segment.length_m, rest_length)
            axial_force = _mul(segment.tangent, axial_tension)
            forces[left] = _add(forces[left], axial_force)
            forces[right] = _sub(forces[right], axial_force)

        if material_node_velocities:
            left_drag, right_drag = _p1_morison_segment_nodal_forces(
                case,
                segment,
                left_material_velocity_mps=material_node_velocities[left],
                right_material_velocity_mps=material_node_velocities[right],
            )
            forces[left] = _add(forces[left], left_drag)
            forces[right] = _add(forces[right], right_drag)
        else:
            midpoint_depth = 0.5 * (segment.start[2] + segment.end[2])
            water_velocity = current_at(
                depth_m=midpoint_depth,
                water_depth_m=case.water_depth_m,
                current_surface_mps=case.current_surface_mps,
                current_bottom_mps=case.current_bottom_mps,
                current_profile_exponent=case.current_profile_exponent,
                current_direction_deg=case.current_direction_deg,
            )
            midpoint_velocity = _mul(
                _add(state.velocities[left], state.velocities[right]),
                0.5,
            )
            material_velocity = _segment_material_velocity(
                node_velocity=midpoint_velocity,
                tangent=segment.tangent,
                payout_speed_mps=material_speed,
            )
            relative_velocity = _sub(material_velocity, water_velocity)
            drag = morison_drag(
                seawater_density=_SEAWATER_DENSITY_KG_M3,
                diameter_m=case.cable.diameter_m,
                segment_length_m=segment.length_m,
                tangent=segment.tangent,
                relative_velocity=relative_velocity,
                tangential_coefficient=case.cable.tangential_drag_coefficient,
                normal_coefficient=case.cable.normal_drag_coefficient,
            )
            half_drag = _mul(drag, 0.5)
            forces[left] = _add(forces[left], half_drag)
            forces[right] = _add(forces[right], half_drag)
        weight = (0.0, 0.0, case.cable.submerged_weight_n_per_m * rest_length)
        half_weight = _mul(weight, 0.5)
        forces[left] = _add(forces[left], half_weight)
        forces[right] = _add(forces[right], half_weight)
    if seabed_depth_m is not None:
        node_material_speeds = _node_material_flow_speeds(
            state.rest_lengths_m,
            fairlead_speed_mps=payout_speed,
            plough_speed_mps=plough_exit_speed_mps,
        )
        for index, (force, position, contact) in enumerate(zip(forces, state.positions, state.contact_flags)):
            if contact or position[2] >= seabed_depth_m:
                normal_reaction = max(force[2], 0.0)
                material_velocity = _add(
                    state.velocities[index],
                    _mul(_node_tangent(segments, index), node_material_speeds[index]),
                )
                friction = seabed_friction(
                    normal_force_n=normal_reaction,
                    tangential_velocity=material_velocity,
                    friction_coefficient=seabed_friction_coefficient,
                )
                forces[index] = _add((force[0], force[1], min(force[2], 0.0)), friction)
    return tuple(forces)






def _target_segment_length(
    rest_lengths_m: tuple[float, ...],
    requested_length_m: float | None,
) -> float:
    if requested_length_m is not None:
        if requested_length_m <= 0.0:
            raise ValueError("target_segment_length_m must be positive")
        return requested_length_m
    positive_lengths = sorted(length for length in rest_lengths_m if length > _MIN_LENGTH)
    if not positive_lengths:
        return 1.0
    return positive_lengths[len(positive_lengths) // 2]


def _insert_payout_nodes(
    state: DynamicLayingState,
    *,
    payout_increment_m: float,
    target_segment_length_m: float,
    dt_s: float,
) -> DynamicLayingState:
    """累积导缆点流入量，并按守恒方式拆分首个单元。

    参考长度先进入控制体，再改变拓扑；每次拆分均以物理单位传递已存反力场，
    避免仅因离散变化而在重网格过程中产生力脉冲。
    """

    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    _validate_state(state)
    if not state.rest_lengths_m:
        return replace(state, payout_buffer_m=0.0)
    payout_increment = max(0.0, state.payout_buffer_m + payout_increment_m)
    should_split_top = payout_increment > _MIN_LENGTH
    rest_lengths = list(state.rest_lengths_m)
    rest_lengths[0] += payout_increment
    length_reactions = _state_physical_length_reactions(state)
    segment_tensions = _state_physical_segment_tensions(state, length_reactions)
    contact_reactions = _state_physical_contact_reactions(state)
    remeshed = replace(
        state,
        rest_lengths_m=tuple(rest_lengths),
        length_lambdas_n_s2=tuple(reaction * dt_s * dt_s for reaction in length_reactions),
        contact_lambdas_n_s2=tuple(reaction * dt_s * dt_s for reaction in contact_reactions),
        segment_tensions_n=segment_tensions,
        length_constraint_reactions_n=length_reactions,
        contact_normal_reactions_n=contact_reactions,
        payout_buffer_m=0.0,
    )
    split_count = 0
    while (
        should_split_top
        and remeshed.rest_lengths_m
        and remeshed.rest_lengths_m[0] > 1.5 * target_segment_length_m
    ):
        if split_count > 10000:
            raise RuntimeError("payout remesh inserted too many nodes in one step")
        remeshed = _split_first_segment_conservatively(
            remeshed,
            first_child_rest_length_m=target_segment_length_m,
            dt_s=dt_s,
        )
        split_count += 1
    return remeshed


def _split_first_segment_conservatively(
    state: DynamicLayingState,
    *,
    first_child_rest_length_m: float,
    dt_s: float,
) -> DynamicLayingState:
    """在不改变线性有限元状态的条件下拆分首个材料区间。"""

    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    _validate_state(state)
    parent_rest_length = state.rest_lengths_m[0]
    if not _MIN_LENGTH < first_child_rest_length_m < parent_rest_length - _MIN_LENGTH:
        raise ValueError("first child rest length must lie strictly inside the parent segment")
    second_child_rest_length = parent_rest_length - first_child_rest_length_m
    child_rest_lengths = [first_child_rest_length_m, second_child_rest_length]
    material_fraction = first_child_rest_length_m / parent_rest_length
    new_position = _add(
        state.positions[0],
        _mul(_sub(state.positions[1], state.positions[0]), material_fraction),
    )
    child_velocities = _conservative_material_velocity_transfer(
        old_velocities=list(state.velocities[:2]),
        old_rest_lengths=[parent_rest_length],
        new_rest_lengths=child_rest_lengths,
    )
    length_reactions = _state_physical_length_reactions(state)
    segment_tensions = _state_physical_segment_tensions(state, length_reactions)
    contact_reactions = _state_physical_contact_reactions(state)
    new_contact_reaction = (
        contact_reactions[0]
        + material_fraction * (contact_reactions[1] - contact_reactions[0])
    )
    new_contact_flag = _sample_material_contact_flag(
        list(state.contact_flags[:2]),
        [0.0, parent_rest_length],
        first_child_rest_length_m,
    )
    split_reactions = (length_reactions[0], length_reactions[0], *length_reactions[1:])
    split_tensions = (segment_tensions[0], segment_tensions[0], *segment_tensions[1:])
    split_contact_reactions = (
        contact_reactions[0],
        new_contact_reaction,
        *contact_reactions[1:],
    )
    return replace(
        state,
        positions=(state.positions[0], new_position, *state.positions[1:]),
        velocities=(*child_velocities, *state.velocities[2:]),
        rest_lengths_m=(*child_rest_lengths, *state.rest_lengths_m[1:]),
        contact_flags=(state.contact_flags[0], new_contact_flag, *state.contact_flags[1:]),
        length_lambdas_n_s2=tuple(reaction * dt_s * dt_s for reaction in split_reactions),
        contact_lambdas_n_s2=tuple(
            reaction * dt_s * dt_s for reaction in split_contact_reactions
        ),
        segment_tensions_n=split_tensions,
        length_constraint_reactions_n=split_reactions,
        contact_normal_reactions_n=split_contact_reactions,
    )


def _state_physical_length_reactions(state: DynamicLayingState) -> tuple[float, ...]:
    count = len(state.rest_lengths_m)
    if len(state.length_constraint_reactions_n) == count:
        return tuple(max(0.0, value) for value in state.length_constraint_reactions_n)
    if len(state.segment_tensions_n) == count:
        return tuple(max(0.0, value) for value in state.segment_tensions_n)
    return tuple(0.0 for _ in range(count))


def _state_physical_segment_tensions(
    state: DynamicLayingState,
    fallback: tuple[float, ...],
) -> tuple[float, ...]:
    if len(state.segment_tensions_n) == len(state.rest_lengths_m):
        return tuple(max(0.0, value) for value in state.segment_tensions_n)
    return fallback


def _state_physical_contact_reactions(state: DynamicLayingState) -> tuple[float, ...]:
    if len(state.contact_normal_reactions_n) == len(state.positions):
        return tuple(max(0.0, value) for value in state.contact_normal_reactions_n)
    return tuple(0.0 for _ in state.positions)




def _local_segment_length(index: int, rest_lengths_m: tuple[float, ...]) -> float:
    adjacent: list[float] = []
    if index > 0 and index - 1 < len(rest_lengths_m):
        adjacent.append(rest_lengths_m[index - 1])
    if index < len(rest_lengths_m):
        adjacent.append(rest_lengths_m[index])
    positive = [length for length in adjacent if length > _MIN_LENGTH]
    if not positive:
        return 1.0
    return min(positive)




def _padded_values(values: tuple[float, ...], count: int) -> list[float]:
    padded = list(values[:count])
    if len(padded) < count:
        padded.extend(0.0 for _ in range(count - len(padded)))
    return padded


def _apply_contact_friction(
    *,
    positions: tuple[Vector3, ...],
    previous_positions: tuple[Vector3, ...],
    velocities: tuple[Vector3, ...],
    contact_flags: tuple[bool, ...],
    contact_normal_reactions_n: tuple[float, ...],
    masses: tuple[float, ...],
    payout_speed_mps: float,
    rest_lengths_m: tuple[float, ...] = (),
    plough_exit_speed_mps: float | None = None,
    dt_s: float,
    friction_coefficient: float,
    update_positions: bool = True,
) -> tuple[tuple[Vector3, ...], tuple[Vector3, ...]]:
    """对接触材料施加 Coulomb 冲量，而非只作用于网格节点。

    ALE 材料速度包含材料相对网格的局部穿越速度。冲量设有限幅，摩擦不会在一个
    时间步内反转水平滑移；端点清零保持本辅助函数的固定导缆点约定。
    """

    if friction_coefficient <= 0.0:
        return positions, velocities
    next_positions = list(positions)
    next_velocities = list(velocities)
    try:
        segments = segment_vectors(positions)
    except ValueError:
        segments = []
    if rest_lengths_m and len(rest_lengths_m) != len(positions) - 1:
        raise ValueError("rest_lengths_m must have one entry per segment")
    node_material_speeds = (
        _node_material_flow_speeds(
            rest_lengths_m,
            fairlead_speed_mps=payout_speed_mps,
            plough_speed_mps=plough_exit_speed_mps,
        )
        if rest_lengths_m
        else tuple(payout_speed_mps for _ in positions)
    )
    for index, contact in enumerate(contact_flags):
        if index == 0 or not contact:
            continue
        normal_reaction = contact_normal_reactions_n[index] if index < len(contact_normal_reactions_n) else 0.0
        if normal_reaction <= 0.0:
            continue
        tangent = _node_tangent(segments, index)
        material_velocity = _add(next_velocities[index], _mul(tangent, node_material_speeds[index]))
        horizontal_speed = math.hypot(material_velocity[0], material_velocity[1])
        if horizontal_speed <= _MIN_LENGTH:
            continue
        max_delta_v = friction_coefficient * normal_reaction * dt_s / max(masses[index], _MIN_MASS)
        delta_v = min(horizontal_speed, max_delta_v)
        correction = (-material_velocity[0] / horizontal_speed * delta_v, -material_velocity[1] / horizontal_speed * delta_v, 0.0)
        velocity = next_velocities[index]
        next_velocity = (velocity[0] + correction[0], velocity[1] + correction[1], min(velocity[2], 0.0))
        next_velocities[index] = next_velocity
        if not update_positions:
            continue
        previous = previous_positions[index]
        position = next_positions[index]
        next_positions[index] = (
            previous[0] + next_velocity[0] * dt_s,
            previous[1] + next_velocity[1] * dt_s,
            position[2],
        )
    next_velocities[0] = (0.0, 0.0, 0.0)
    next_positions[0] = (0.0, 0.0, 0.0)
    return tuple(next_positions), tuple(next_velocities)
















def _safe_unit(vector: Vector3 | None) -> Vector3:
    if vector is None:
        return (0.0, 0.0, 1.0)
    magnitude = _norm(vector)
    if magnitude <= _MIN_LENGTH or not math.isfinite(magnitude):
        return (0.0, 0.0, 1.0)
    return _mul(vector, 1.0 / magnitude)










# 离线时程与实时会话共用的持久运行时 API。
def solve_known_plough_time_history(dynamic_case, *, points: int = 361):
    """求解一组给定船端至犁端时程。"""

    if isinstance(points, bool) or not isinstance(points, int):
        raise ValueError("points must be an integer")
    if not 3 <= points <= 1001:
        raise ValueError("points must be between 3 and 1001")
    return _solve_known_plough_time_history(dynamic_case, points=points)



def initialize_known_plough_runtime(dynamic_case) -> KnownPloughRuntime:
    """创建一个持久的已知犁轨迹运行状态，不推进物理时间。"""

    validate_dynamic_case(
        dynamic_case,
        allowed_length_boundary_sources={"known_plough_trajectory"},
    )
    cable = cable_parameters_from_dynamic_case(dynamic_case)
    state = _initial_known_plough_state(dynamic_case, cable)
    _feasible_bend_projection_radius_m(
        requested_radius_m=cable.min_bending_radius_m,
        rest_lengths_m=state.rest_lengths_m,
        top_position=state.positions[0],
        bottom_position=state.positions[-1],
    )
    bend_radius = _minimum_bend_radius_diagnostic(
        state.positions,
        exclude_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
    )
    raw_bend_radius = _minimum_bend_radius_diagnostic(state.positions)
    return KnownPloughRuntime(
        state=state,
        cable=cable,
        time_s=0.0,
        dt_max_s=_resolved_time_step_max_s(dynamic_case),
        steps=0,
        integration_time_step_min_s=None,
        integration_time_step_max_s=None,
        axial_iterations_min=None,
        axial_iterations_max=None,
        axial_constraint_residual_max_m=None,
        bend_radius_diagnostic=bend_radius,
        bend_radius_min_m=bend_radius.radius_m,
        bend_radius_time_s=0.0 if bend_radius.node_index is not None else None,
        raw_bend_radius_diagnostic=raw_bend_radius,
        raw_bend_radius_min_m=raw_bend_radius.radius_m,
        raw_bend_radius_time_s=0.0 if raw_bend_radius.node_index is not None else None,
    )


def advance_known_plough_runtime(
    runtime: KnownPloughRuntime,
    dynamic_case,
    *,
    target_time_s: float,
) -> KnownPloughRuntime:
    """将持久的已知犁轨迹状态推进到更晚的物理时刻。

    缩短最后一个内部时间步，使状态精确到达请求的输出时刻。规范时间同步消除
    累积舍入误差，但不改变物理时间步序列。
    """

    canonical_time = _canonical_synchronized_time_s(
        dynamic_case,
        runtime.time_s,
        completed_steps=runtime.steps,
    )
    if canonical_time != runtime.time_s:
        runtime.time_s = canonical_time
        runtime.state = replace(runtime.state, time_s=canonical_time)
    if target_time_s < runtime.time_s - 1.0e-9:
        raise ValueError("target_time_s must not precede the runtime time")
    while runtime.time_s + 1.0e-9 < target_time_s:
        step_start_time = runtime.time_s
        natural_dt = _time_history_step_limit_s(
            dynamic_case,
            runtime.state,
            base_step_s=runtime.dt_max_s,
        )
        remaining_time = target_time_s - step_start_time
        time_roundoff = 64.0 * math.ulp(
            max(abs(step_start_time), abs(target_time_s), abs(natural_dt), 1.0)
        )
        dt = (
            natural_dt
            if abs(remaining_time - natural_dt) <= time_roundoff
            else min(natural_dt, remaining_time)
        )
        case_at_time = _operation_case_at_time(
            dynamic_case,
            runtime.cable,
            step_start_time,
        )
        runtime.state = _step_known_plough_dynamic(
            dynamic_case,
            case_at_time,
            runtime.state,
            time_s=step_start_time,
            dt_s=dt,
        )
        runtime.time_s = step_start_time + dt
        canonical_time = _canonical_synchronized_time_s(
            dynamic_case,
            runtime.time_s,
            completed_steps=runtime.steps + 1,
        )
        if canonical_time != runtime.time_s:
            runtime.time_s = canonical_time
            runtime.state = replace(runtime.state, time_s=canonical_time)
        runtime.integration_time_step_min_s = _optional_min(runtime.integration_time_step_min_s, dt)
        runtime.integration_time_step_max_s = _optional_max(runtime.integration_time_step_max_s, dt)
        runtime.axial_iterations_min = _optional_min(
            runtime.axial_iterations_min,
            runtime.state.axial_solve_iterations,
        )
        runtime.axial_iterations_max = _optional_max(
            runtime.axial_iterations_max,
            runtime.state.axial_solve_iterations,
        )
        runtime.axial_constraint_residual_max_m = _optional_max(
            runtime.axial_constraint_residual_max_m,
            runtime.state.axial_constraint_residual_m,
        )
        runtime.steps += 1

        bend_radius = _minimum_bend_radius_diagnostic(
            runtime.state.positions,
            exclude_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
        )
        if bend_radius.radius_m < runtime.bend_radius_min_m:
            runtime.bend_radius_diagnostic = bend_radius
            runtime.bend_radius_min_m = bend_radius.radius_m
            runtime.bend_radius_time_s = runtime.time_s
        raw_bend_radius = _minimum_bend_radius_diagnostic(runtime.state.positions)
        if raw_bend_radius.radius_m < runtime.raw_bend_radius_min_m:
            runtime.raw_bend_radius_diagnostic = raw_bend_radius
            runtime.raw_bend_radius_min_m = raw_bend_radius.radius_m
            runtime.raw_bend_radius_time_s = runtime.time_s
    return runtime


def sample_known_plough_runtime(runtime: KnownPloughRuntime, dynamic_case) -> KnownPloughSample:
    """构造最新帧输出，不改变运行状态。

    分段约束反力、端点控制体反力和接触过渡插值保持为相互独立的输出口径。
    """

    time_s = runtime.time_s
    state = runtime.state
    case_at_time = _operation_case_at_time(
        dynamic_case,
        runtime.cable,
        time_s,
    )
    segment_tensions = _known_plough_output_segment_tensions(dynamic_case, case_at_time, state, time_s)
    point_tensions = list(
        _point_tensions_from_segment_tensions(state, segment_tensions)
    )
    length_constraint_reactions = _length_constraint_reactions_from_dynamic_state(state)
    fairlead_adjacent_tension = (
        length_constraint_reactions[0]
        if length_constraint_reactions
        else segment_tensions[0]
        if segment_tensions
        else 0.0
    )
    top_tension = _fairlead_boundary_tension_from_dynamic_state(
        dynamic_case,
        case_at_time,
        state,
        time_s,
        adjacent_segment_tension_n=fairlead_adjacent_tension,
    )
    if point_tensions:
        point_tensions[0] = top_tension
    plough_adjacent_tension = (
        length_constraint_reactions[-1]
        if length_constraint_reactions
        else segment_tensions[-1]
        if segment_tensions
        else top_tension
    )
    plough_boundary_tension = _plough_boundary_tension_from_dynamic_state(
        dynamic_case,
        case_at_time,
        state,
        time_s,
        adjacent_segment_tension_n=plough_adjacent_tension,
    )
    vessel = _vessel_position(dynamic_case, time_s)
    plough = _plough_position(dynamic_case, time_s)
    contact_profile = _state_contact_profile(state, dynamic_case.water_depth_m)
    plough_inlet_tension, contact_transition_tension = _plough_and_contact_transition_tensions(
        segment_tensions=segment_tensions,
        rest_lengths_m=state.rest_lengths_m,
        contact_profile=contact_profile,
    )
    contact_transition = contact_profile.tdp_point if contact_profile.has_contact else None
    bend_radius = _minimum_bend_radius_diagnostic(
        state.positions,
        exclude_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
    )
    raw_bend_radius = _minimum_bend_radius_diagnostic(state.positions)
    entry_angle = _plough_entry_angle_deg(state.positions)
    iterations = max(runtime.steps * len(state.positions), len(state.positions))

    point = TimeHistoryPoint(
        time_s=float(time_s),
        top_tension_n=float(top_tension),
        has_contact=contact_profile.has_contact,
        contact_transition_x_m=(None if contact_transition is None else float(contact_transition[0])),
        contact_transition_y_m=(None if contact_transition is None else float(contact_transition[1])),
        suspended_length_m=float(state.suspended_length_m),
        iterations=iterations,
        plough_x_m=float(plough[0]),
        plough_y_m=float(plough[1]),
        plough_z_m=float(plough[2]),
        plough_inlet_tension_n=float(plough_inlet_tension),
        contact_transition_tension_n=contact_transition_tension,
        plough_boundary_tension_n=float(plough_boundary_tension),
        plough_adjacent_segment_tension_n=float(plough_adjacent_tension),
        plough_entry_angle_deg=float(entry_angle),
        minimum_bend_radius_m=float(bend_radius.radius_m),
        minimum_bend_radius_node_index=bend_radius.node_index,
        minimum_bend_radius_left_segment_m=bend_radius.left_segment_m,
        minimum_bend_radius_right_segment_m=bend_radius.right_segment_m,
        minimum_bend_radius_turn_angle_deg=bend_radius.turn_angle_deg,
        minimum_bend_radius_node_depth_m=bend_radius.node_depth_m,
        minimum_bend_radius_near_seabed=bend_radius.near_seabed,
        minimum_bend_radius_excluded_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
        minimum_bend_radius_raw_m=float(raw_bend_radius.radius_m),
        minimum_bend_radius_raw_node_index=raw_bend_radius.node_index,
        minimum_bend_radius_raw_left_segment_m=raw_bend_radius.left_segment_m,
        minimum_bend_radius_raw_right_segment_m=raw_bend_radius.right_segment_m,
        minimum_bend_radius_raw_turn_angle_deg=raw_bend_radius.turn_angle_deg,
        minimum_bend_radius_raw_node_depth_m=raw_bend_radius.node_depth_m,
        minimum_bend_radius_raw_near_seabed=raw_bend_radius.near_seabed,
        material_suspended_length_m=float(
            state.material_suspended_length_m
            if state.material_suspended_length_m > _MIN_LENGTH
            else state.suspended_length_m
        ),
        geometric_length_deficit_m=float(state.geometric_length_deficit_m),
        contact_transition_arc_length_m=(
            float(contact_profile.tdp_arc_length_m) if contact_profile.has_contact else None
        ),
        free_span_material_length_m=float(contact_profile.suspended_length_m),
        seabed_contact_length_m=float(contact_profile.contact_length_m),
        seabed_normal_reaction_n=float(contact_profile.normal_resultant_n),
    )
    frame = TimeHistoryFrame(
        time_s=float(time_s),
        points=[
            TimeHistoryFramePoint(
                index=index,
                x_m=float(position[0]),
                y_m=float(position[1]),
                z_m=float(position[2]),
                tension_n=float(point_tensions[index]),
            )
            for index, position in enumerate(state.positions)
        ],
        segment_tensions_n=tuple(float(tension) for tension in segment_tensions),
        boundary="known_plough_trajectory",
        vessel_x_m=float(vessel[0]),
        vessel_y_m=float(vessel[1]),
        vessel_z_m=float(vessel[2]),
        plough_x_m=float(plough[0]),
        plough_y_m=float(plough[1]),
        plough_z_m=float(plough[2]),
        minimum_bend_radius_m=float(bend_radius.radius_m),
        minimum_bend_radius_node_index=bend_radius.node_index,
        minimum_bend_radius_left_segment_m=bend_radius.left_segment_m,
        minimum_bend_radius_right_segment_m=bend_radius.right_segment_m,
        minimum_bend_radius_turn_angle_deg=bend_radius.turn_angle_deg,
        minimum_bend_radius_node_depth_m=bend_radius.node_depth_m,
        minimum_bend_radius_near_seabed=bend_radius.near_seabed,
        minimum_bend_radius_excluded_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
        minimum_bend_radius_raw_m=float(raw_bend_radius.radius_m),
        minimum_bend_radius_raw_node_index=raw_bend_radius.node_index,
        minimum_bend_radius_raw_left_segment_m=raw_bend_radius.left_segment_m,
        minimum_bend_radius_raw_right_segment_m=raw_bend_radius.right_segment_m,
        minimum_bend_radius_raw_turn_angle_deg=raw_bend_radius.turn_angle_deg,
        minimum_bend_radius_raw_node_depth_m=raw_bend_radius.node_depth_m,
        minimum_bend_radius_raw_near_seabed=raw_bend_radius.near_seabed,
    )
    return KnownPloughSample(point=point, frame=frame)


def _solve_known_plough_time_history(dynamic_case, *, points: int = 361):
    """求解给定船端与犁端之间的悬空跨段。"""

    validate_dynamic_case(
        dynamic_case,
        allowed_length_boundary_sources={"known_plough_trajectory"},
    )
    cable = cable_parameters_from_dynamic_case(dynamic_case)
    sample_times = build_sample_times(dynamic_case, points)
    state = _initial_known_plough_state(dynamic_case, cable)
    samples: list[tuple[float, DynamicLayingState, int]] = []
    current_time = 0.0
    steps = 0
    next_sample = 0
    dt_max = _resolved_time_step_max_s(dynamic_case)
    used_dts: list[float] = []
    axial_iteration_counts: list[int] = []
    axial_constraint_residuals: list[float] = []
    bend_radius_diagnostic = _minimum_bend_radius_diagnostic(
        state.positions,
        exclude_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
    )
    bend_radius_min = bend_radius_diagnostic.radius_m
    bend_radius_time_s = 0.0 if bend_radius_diagnostic.node_index is not None else None
    raw_bend_radius_diagnostic = _minimum_bend_radius_diagnostic(state.positions)
    raw_bend_radius_min = raw_bend_radius_diagnostic.radius_m
    raw_bend_radius_time_s = 0.0 if raw_bend_radius_diagnostic.node_index is not None else None

    while next_sample < len(sample_times) and sample_times[next_sample] <= 1.0e-9:
        samples.append((sample_times[next_sample], state, len(state.positions)))
        next_sample += 1

    while current_time + 1.0e-9 < dynamic_case.total_duration_s:
        canonical_time = _canonical_synchronized_time_s(
            dynamic_case,
            current_time,
            completed_steps=steps,
        )
        if canonical_time != current_time:
            current_time = canonical_time
            state = replace(state, time_s=canonical_time)
        step_start_time = current_time
        step_start_state = state
        dt = min(
            _time_history_step_limit_s(dynamic_case, state, base_step_s=dt_max),
            dynamic_case.total_duration_s - current_time,
        )
        case_at_time = _operation_case_at_time(
            dynamic_case,
            cable,
            step_start_time,
        )
        state = _step_known_plough_dynamic(
            dynamic_case,
            case_at_time,
            state,
            time_s=step_start_time,
            dt_s=dt,
        )
        current_time = step_start_time + dt
        canonical_time = _canonical_synchronized_time_s(
            dynamic_case,
            current_time,
            completed_steps=steps + 1,
        )
        if canonical_time != current_time:
            current_time = canonical_time
            state = replace(state, time_s=canonical_time)
        used_dts.append(dt)
        axial_iteration_counts.append(state.axial_solve_iterations)
        axial_constraint_residuals.append(state.axial_constraint_residual_m)
        steps += 1
        candidate_bend_radius = _minimum_bend_radius_diagnostic(
            state.positions,
            exclude_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
        )
        if candidate_bend_radius.radius_m < bend_radius_min:
            bend_radius_diagnostic = candidate_bend_radius
            bend_radius_min = candidate_bend_radius.radius_m
            bend_radius_time_s = current_time
        raw_candidate_bend_radius = _minimum_bend_radius_diagnostic(state.positions)
        if raw_candidate_bend_radius.radius_m < raw_bend_radius_min:
            raw_bend_radius_diagnostic = raw_candidate_bend_radius
            raw_bend_radius_min = raw_candidate_bend_radius.radius_m
            raw_bend_radius_time_s = current_time

        while next_sample < len(sample_times) and sample_times[next_sample] <= current_time + 1.0e-9:
            target_time = sample_times[next_sample]
            if abs(target_time - current_time) <= 1.0e-9:
                sample_state = state
            elif target_time <= step_start_time + 1.0e-9:
                sample_state = step_start_state
            else:
                sample_dt = target_time - step_start_time
                sample_case = _operation_case_at_time(
                    dynamic_case,
                    cable,
                    step_start_time,
                )
                sample_state = _step_known_plough_dynamic(
                    dynamic_case,
                    sample_case,
                    step_start_state,
                    time_s=step_start_time,
                    dt_s=sample_dt,
                )
            samples.append(
                (
                    target_time,
                    sample_state,
                    max(steps * len(sample_state.positions), len(sample_state.positions)),
                )
            )
            next_sample += 1

    history: list[TimeHistoryPoint] = []
    frames: list[TimeHistoryFrame] = []
    for time_s, sample_state, iterations in samples:
        case_at_time = _operation_case_at_time(
            dynamic_case,
            cable,
            time_s,
        )
        segment_tensions = _known_plough_output_segment_tensions(dynamic_case, case_at_time, sample_state, time_s)
        point_tensions = list(
            _point_tensions_from_segment_tensions(sample_state, segment_tensions)
        )
        length_constraint_reactions = _length_constraint_reactions_from_dynamic_state(sample_state)
        fairlead_adjacent_tension = (
            length_constraint_reactions[0]
            if length_constraint_reactions
            else segment_tensions[0]
            if segment_tensions
            else 0.0
        )
        top_tension = _fairlead_boundary_tension_from_dynamic_state(
            dynamic_case,
            case_at_time,
            sample_state,
            time_s,
            adjacent_segment_tension_n=fairlead_adjacent_tension,
        )
        if point_tensions:
            point_tensions[0] = top_tension
        plough_adjacent_tension = (
            length_constraint_reactions[-1]
            if length_constraint_reactions
            else segment_tensions[-1]
            if segment_tensions
            else top_tension
        )
        plough_boundary_tension = _plough_boundary_tension_from_dynamic_state(
            dynamic_case,
            case_at_time,
            sample_state,
            time_s,
            adjacent_segment_tension_n=plough_adjacent_tension,
        )
        vessel = _vessel_position(dynamic_case, time_s)
        plough = _plough_position(dynamic_case, time_s)
        contact_profile = _state_contact_profile(sample_state, dynamic_case.water_depth_m)
        plough_inlet_tension, contact_transition_tension = _plough_and_contact_transition_tensions(
            segment_tensions=segment_tensions,
            rest_lengths_m=sample_state.rest_lengths_m,
            contact_profile=contact_profile,
        )
        contact_transition = contact_profile.tdp_point if contact_profile.has_contact else None
        sample_bend_radius = _minimum_bend_radius_diagnostic(
            sample_state.positions,
            exclude_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
        )
        raw_sample_bend_radius = _minimum_bend_radius_diagnostic(sample_state.positions)
        minimum_bend_radius = sample_bend_radius.radius_m
        entry_angle = _plough_entry_angle_deg(sample_state.positions)
        history.append(
            TimeHistoryPoint(
                time_s=float(time_s),
                top_tension_n=float(top_tension),
                has_contact=contact_profile.has_contact,
                contact_transition_x_m=(None if contact_transition is None else float(contact_transition[0])),
                contact_transition_y_m=(None if contact_transition is None else float(contact_transition[1])),
                suspended_length_m=float(sample_state.suspended_length_m),
                iterations=iterations,
                plough_x_m=float(plough[0]),
                plough_y_m=float(plough[1]),
                plough_z_m=float(plough[2]),
                plough_inlet_tension_n=float(plough_inlet_tension),
                contact_transition_tension_n=contact_transition_tension,
                plough_boundary_tension_n=float(plough_boundary_tension),
                plough_adjacent_segment_tension_n=float(plough_adjacent_tension),
                plough_entry_angle_deg=float(entry_angle),
                minimum_bend_radius_m=float(minimum_bend_radius),
                minimum_bend_radius_node_index=sample_bend_radius.node_index,
                minimum_bend_radius_left_segment_m=sample_bend_radius.left_segment_m,
                minimum_bend_radius_right_segment_m=sample_bend_radius.right_segment_m,
                minimum_bend_radius_turn_angle_deg=sample_bend_radius.turn_angle_deg,
                minimum_bend_radius_node_depth_m=sample_bend_radius.node_depth_m,
                minimum_bend_radius_near_seabed=sample_bend_radius.near_seabed,
                minimum_bend_radius_excluded_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
                minimum_bend_radius_raw_m=float(raw_sample_bend_radius.radius_m),
                minimum_bend_radius_raw_node_index=raw_sample_bend_radius.node_index,
                minimum_bend_radius_raw_left_segment_m=raw_sample_bend_radius.left_segment_m,
                minimum_bend_radius_raw_right_segment_m=raw_sample_bend_radius.right_segment_m,
                minimum_bend_radius_raw_turn_angle_deg=raw_sample_bend_radius.turn_angle_deg,
                minimum_bend_radius_raw_node_depth_m=raw_sample_bend_radius.node_depth_m,
                minimum_bend_radius_raw_near_seabed=raw_sample_bend_radius.near_seabed,
                material_suspended_length_m=float(
                    sample_state.material_suspended_length_m
                    if sample_state.material_suspended_length_m > _MIN_LENGTH
                    else sample_state.suspended_length_m
                ),
                geometric_length_deficit_m=float(sample_state.geometric_length_deficit_m),
                contact_transition_arc_length_m=(
                    float(contact_profile.tdp_arc_length_m) if contact_profile.has_contact else None
                ),
                free_span_material_length_m=float(contact_profile.suspended_length_m),
                seabed_contact_length_m=float(contact_profile.contact_length_m),
                seabed_normal_reaction_n=float(contact_profile.normal_resultant_n),
            )
        )
        frames.append(
            TimeHistoryFrame(
                time_s=float(time_s),
                points=[
                    TimeHistoryFramePoint(
                        index=index,
                        x_m=float(position[0]),
                        y_m=float(position[1]),
                        z_m=float(position[2]),
                        tension_n=float(point_tensions[index]),
                    )
                    for index, position in enumerate(sample_state.positions)
                ],
                segment_tensions_n=tuple(float(tension) for tension in segment_tensions),
                boundary="known_plough_trajectory",
                vessel_x_m=float(vessel[0]),
                vessel_y_m=float(vessel[1]),
                vessel_z_m=float(vessel[2]),
                plough_x_m=float(plough[0]),
                plough_y_m=float(plough[1]),
                plough_z_m=float(plough[2]),
                minimum_bend_radius_m=float(minimum_bend_radius),
                minimum_bend_radius_node_index=sample_bend_radius.node_index,
                minimum_bend_radius_left_segment_m=sample_bend_radius.left_segment_m,
                minimum_bend_radius_right_segment_m=sample_bend_radius.right_segment_m,
                minimum_bend_radius_turn_angle_deg=sample_bend_radius.turn_angle_deg,
                minimum_bend_radius_node_depth_m=sample_bend_radius.node_depth_m,
                minimum_bend_radius_near_seabed=sample_bend_radius.near_seabed,
                minimum_bend_radius_excluded_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
                minimum_bend_radius_raw_m=float(raw_sample_bend_radius.radius_m),
                minimum_bend_radius_raw_node_index=raw_sample_bend_radius.node_index,
                minimum_bend_radius_raw_left_segment_m=raw_sample_bend_radius.left_segment_m,
                minimum_bend_radius_raw_right_segment_m=raw_sample_bend_radius.right_segment_m,
                minimum_bend_radius_raw_turn_angle_deg=raw_sample_bend_radius.turn_angle_deg,
                minimum_bend_radius_raw_node_depth_m=raw_sample_bend_radius.node_depth_m,
                minimum_bend_radius_raw_near_seabed=raw_sample_bend_radius.near_seabed,
            )
        )

    top_tensions = [point.top_tension_n for point in history]
    plough_tensions = [
        point.plough_inlet_tension_n
        for point in history
        if point.plough_inlet_tension_n is not None
    ]
    final_plough_boundary_tension = history[-1].plough_boundary_tension_n
    final_plough_adjacent_tension = history[-1].plough_adjacent_segment_tension_n
    final_plough_exit_speed, plough_exit_speed_source = _plough_exit_material_speed(
        dynamic_case,
        _vessel_velocity(dynamic_case, sample_times[-1]),
        time_s=sample_times[-1],
    )
    length_deficits = [
        point.geometric_length_deficit_m
        for point in history
        if point.geometric_length_deficit_m is not None
    ]
    bend_radius_limit = cable.min_bending_radius_m
    bend_radius_margin = (
        None
        if not math.isfinite(bend_radius_min) or bend_radius_limit is None
        else bend_radius_min - bend_radius_limit
    )
    return TimeHistoryResult(
        case_name=dynamic_case.case_name,
        diameter_m=dynamic_case.diameter_m,
        weight_air_n_per_m=dynamic_case.weight_air_n_per_m,
        submerged_weight_n_per_m=dynamic_case.submerged_weight_n_per_m,
        tangential_drag_coefficient=dynamic_case.tangential_drag_coefficient,
        normal_drag_coefficient=dynamic_case.normal_drag_coefficient,
        axial_stiffness_n=dynamic_case.axial_stiffness_n,
        current_speed_mps=dynamic_case.current_speed_mps,
        current_direction_deg=dynamic_case.current_direction_deg,
        speed_change=dynamic_case.speed_change,
        vessel_initial_speed_mps=dynamic_case.vessel_initial_speed_mps,
        vessel_final_speed_mps=dynamic_case.vessel_final_speed_mps,
        transition_duration_s=dynamic_case.transition_duration_s,
        total_duration_s=dynamic_case.total_duration_s,
        water_depth_m=dynamic_case.water_depth_m,
        element_count=dynamic_case.element_count,
        payout_initial_speed_mps=_initial_payout_speed(dynamic_case),
        payout_final_speed_mps=_final_payout_speed(dynamic_case),
        length_boundary_source="known_plough_trajectory",
        initial_suspended_length_m=dynamic_case.initial_suspended_length_m,
        solver_id=SOLVER_ID,
        initial_tension_n=history[0].top_tension_n,
        extreme_tension_n=min(top_tensions) if dynamic_case.speed_change == "accel" else max(top_tensions),
        steady_tension_n=history[-1].top_tension_n,
        history=history,
        frames=frames,
        current_bottom_speed_mps=dynamic_case.current_bottom_speed_mps,
        current_profile_exponent=dynamic_case.current_profile_exponent,
        plough_speed_mps=dynamic_case.plough_speed_mps,
        plough_exit_speed_mps=final_plough_exit_speed,
        plough_exit_speed_source=plough_exit_speed_source,
        plough_inlet_tension_final_n=plough_tensions[-1] if plough_tensions else None,
        contact_transition_tension_final_n=history[-1].contact_transition_tension_n,
        plough_boundary_tension_final_n=final_plough_boundary_tension,
        plough_adjacent_segment_tension_final_n=final_plough_adjacent_tension,
        plough_tension_status=_plough_tension_status(
            boundary_tension_n=final_plough_boundary_tension,
            adjacent_segment_tension_n=final_plough_adjacent_tension,
        ),
        minimum_bend_radius_min_m=bend_radius_min if math.isfinite(bend_radius_min) else None,
        minimum_bend_radius_limit_m=bend_radius_limit,
        minimum_bend_radius_margin_m=bend_radius_margin,
        minimum_bend_radius_status=minimum_bend_radius_status(
            minimum_radius_m=bend_radius_min if math.isfinite(bend_radius_min) else None,
            limit_m=bend_radius_limit,
        ),
        minimum_bend_radius_time_s=bend_radius_time_s,
        minimum_bend_radius_node_index=bend_radius_diagnostic.node_index,
        minimum_bend_radius_left_segment_m=bend_radius_diagnostic.left_segment_m,
        minimum_bend_radius_right_segment_m=bend_radius_diagnostic.right_segment_m,
        minimum_bend_radius_turn_angle_deg=bend_radius_diagnostic.turn_angle_deg,
        minimum_bend_radius_node_depth_m=bend_radius_diagnostic.node_depth_m,
        minimum_bend_radius_near_seabed=bend_radius_diagnostic.near_seabed,
        minimum_bend_radius_excluded_tail_nodes=_KNOWN_PLOUGH_RMIN_EXCLUDED_TAIL_NODES,
        minimum_bend_radius_raw_m=raw_bend_radius_min if math.isfinite(raw_bend_radius_min) else None,
        minimum_bend_radius_raw_time_s=raw_bend_radius_time_s,
        minimum_bend_radius_raw_node_index=raw_bend_radius_diagnostic.node_index,
        minimum_bend_radius_raw_left_segment_m=raw_bend_radius_diagnostic.left_segment_m,
        minimum_bend_radius_raw_right_segment_m=raw_bend_radius_diagnostic.right_segment_m,
        minimum_bend_radius_raw_turn_angle_deg=raw_bend_radius_diagnostic.turn_angle_deg,
        minimum_bend_radius_raw_node_depth_m=raw_bend_radius_diagnostic.node_depth_m,
        minimum_bend_radius_raw_near_seabed=raw_bend_radius_diagnostic.near_seabed,
        integration_time_step_max_s=max(used_dts) if used_dts else None,
        integration_time_step_min_s=min(used_dts) if used_dts else None,
        spatial_step_mean_m=_mean_positive_length(state.rest_lengths_m),
        spatial_step_min_m=_min_positive_length(state.rest_lengths_m),
        xpbd_iterations_per_step=max(axial_iteration_counts) if axial_iteration_counts else None,
        xpbd_iterations_per_step_min=min(axial_iteration_counts) if axial_iteration_counts else None,
        xpbd_iterations_per_step_max=max(axial_iteration_counts) if axial_iteration_counts else None,
        xpbd_iteration_limit_per_solve=_KNOWN_PLOUGH_XPBD_ITERATIONS,
        axial_constraint_residual_max_m=(
            max(axial_constraint_residuals) if axial_constraint_residuals else None
        ),
        geometric_length_deficit_max_m=max(length_deficits) if length_deficits else None,
        geometric_length_deficit_final_m=history[-1].geometric_length_deficit_m,
        vessel_motion_segments=dynamic_case.vessel_motion_segments,
        plough_motion_segments=dynamic_case.plough_motion_segments,
        vessel_motion_samples=dynamic_case.vessel_motion_samples,
        plough_motion_samples=dynamic_case.plough_motion_samples,
        payout_speed_segments=dynamic_case.payout_speed_segments,
    )


# 初始几何与材料状态构造。
def _initial_known_plough_state(dynamic_case, cable=None) -> DynamicLayingState:
    """构造 t=0 时的几何、网格速度和材料控制体。

    给定的活动参考长度必须能够跨接两端。优先采用悬空悬链线；接触或几何回退
    只为同一动态状态合同提供初值，不改变输入长度。
    """

    if cable is None:
        cable = cable_parameters_from_dynamic_case(dynamic_case)
    vessel = _vessel_position(dynamic_case, 0.0)
    plough = _plough_position(dynamic_case, 0.0)
    vessel_velocity = _vessel_velocity(dynamic_case, 0.0)
    plough_velocity = _plough_velocity(dynamic_case, 0.0)
    element_count = dynamic_case.element_count
    direct_distance = max(_norm(_sub(plough, vessel)), _MIN_LENGTH)
    if dynamic_case.initial_suspended_length_m is None:
        raise ValueError("initial_suspended_length_m is required for known_plough_trajectory")
    active_length = float(dynamic_case.initial_suspended_length_m)
    if active_length + 1.0e-9 < direct_distance:
        raise ValueError("initial_suspended_length_m must be no less than the initial endpoint distance")
    static_initial_profile = _initial_endpoint_catenary_profile(
        vessel,
        plough,
        element_count=element_count,
        suspended_length_m=active_length,
        submerged_weight_n_per_m=cable.submerged_weight_n_per_m,
        water_depth_m=dynamic_case.water_depth_m,
    )
    contact_flags: tuple[bool, ...]
    if static_initial_profile is not None:
        positions, segment_tensions = static_initial_profile
        contact_flags = tuple(False for _ in positions)
    else:
        contact_initial_profile = _initial_endpoint_catenary_with_laid_tail_profile(
            vessel,
            plough,
            element_count=element_count,
            active_length_m=active_length,
            submerged_weight_n_per_m=cable.submerged_weight_n_per_m,
            water_depth_m=dynamic_case.water_depth_m,
        )
        if contact_initial_profile is not None:
            positions, segment_tensions, contact_flags = contact_initial_profile
        else:
            positions = _initial_endpoint_curve(
                vessel,
                plough,
                element_count=element_count,
                water_depth_m=dynamic_case.water_depth_m,
            )
            segment_tensions = ()
            contact_flags = tuple(False for _ in positions)
    rest_length = active_length / element_count
    velocities = tuple(
        _add(vessel_velocity, _mul(_sub(plough_velocity, vessel_velocity), index / element_count))
        for index in range(element_count + 1)
    )
    case_at_time = _operation_case_at_time(
        dynamic_case,
        cable,
        0.0,
    )
    rest_lengths = tuple(rest_length for _ in range(element_count))
    if not segment_tensions:
        segment_tensions = _step_dynamic_segment_tensions(
            case_at_time,
            positions=positions,
            velocities=velocities,
            rest_lengths_m=rest_lengths,
            payout_speed_mps=_payout_speed(dynamic_case, 0.0),
            terminal_tension_n=0.0,
        )
    initial_state = DynamicLayingState(
        time_s=0.0,
        positions=positions,
        velocities=velocities,
        rest_lengths_m=rest_lengths,
        paid_length_m=active_length,
        laid_length_m=0.0,
        contact_flags=contact_flags,
        segment_tensions_n=segment_tensions,
        material_suspended_length_m=active_length,
    )
    payout_speed = _payout_speed(dynamic_case, 0.0)
    plough_speed, _ = _plough_exit_material_speed(
        dynamic_case,
        vessel_velocity,
        time_s=0.0,
    )
    return replace(
        initial_state,
        known_plough_material_control_volume=_KnownPloughMaterialControlVolume(
            material_cells=_material_cells_from_state(
                initial_state,
                fairlead_speed_mps=payout_speed,
                plough_speed_mps=plough_speed,
            ),
        ),
    )




def _initial_endpoint_catenary_profile(
    vessel: Vector3,
    plough: Vector3,
    *,
    element_count: int,
    suspended_length_m: float,
    submerged_weight_n_per_m: float,
    water_depth_m: float,
) -> tuple[tuple[Vector3, ...], tuple[float, ...]] | None:
    """构造动态推进前使用的可行静态悬链线初值。

    当给定跨距和悬空长度无法定义非退化悬链线时，返回 ``None``，
    由几何回退方式完成初始化。
    """

    if element_count <= 0:
        return None
    if suspended_length_m <= 0.0 or submerged_weight_n_per_m <= 0.0:
        return None
    horizontal_vector = (plough[0] - vessel[0], plough[1] - vessel[1])
    horizontal_span_m = math.hypot(horizontal_vector[0], horizontal_vector[1])
    vertical_drop_m = plough[2] - vessel[2]
    straight_span_m = math.hypot(horizontal_span_m, vertical_drop_m)
    if horizontal_span_m <= 1.0e-9 or suspended_length_m <= straight_span_m * (1.0 + 1.0e-12):
        return None
    if abs(vertical_drop_m) >= suspended_length_m:
        return None
    reduced_length_m = math.sqrt(max(suspended_length_m * suspended_length_m - vertical_drop_m * vertical_drop_m, 0.0))
    if reduced_length_m <= horizontal_span_m:
        return None
    try:
        parameter_m = _solve_initial_catenary_parameter(
            horizontal_span_m=horizontal_span_m,
            reduced_length_m=reduced_length_m,
        )
    except ValueError:
        return None
    half_dimensionless_span = horizontal_span_m / (2.0 * parameter_m)
    mean_dimensionless_height = math.atanh(vertical_drop_m / suspended_length_m)
    vessel_argument = mean_dimensionless_height + half_dimensionless_span
    horizontal_tension_n = submerged_weight_n_per_m * parameter_m
    vessel_sinh = math.sinh(vessel_argument)
    max_depth = max(vessel[2], plough[2], water_depth_m)
    horizontal_unit = (horizontal_vector[0] / horizontal_span_m, horizontal_vector[1] / horizontal_span_m)
    positions: list[Vector3] = []
    for index in range(element_count + 1):
        fraction = index / element_count
        argument = math.asinh(vessel_sinh - fraction * suspended_length_m / parameter_m)
        horizontal_distance = parameter_m * (vessel_argument - argument)
        depth = vessel[2] + parameter_m * (math.cosh(vessel_argument) - math.cosh(argument))
        if depth > max_depth + 1.0e-6:
            return None
        positions.append(
            (
                vessel[0] + horizontal_unit[0] * horizontal_distance,
                vessel[1] + horizontal_unit[1] * horizontal_distance,
                depth,
            )
        )
    positions[0] = vessel
    positions[-1] = plough
    segment_arc_length_m = suspended_length_m / element_count
    tensions = tuple(
        horizontal_tension_n
        * math.cosh(
            math.asinh(
                vessel_sinh
                - ((index + 0.5) * segment_arc_length_m) / parameter_m
            )
        )
        for index in range(element_count)
    )
    return tuple(positions), tuple(max(0.0, tension) for tension in tensions)


def _initial_endpoint_catenary_with_laid_tail_profile(
    vessel: Vector3,
    plough: Vector3,
    *,
    element_count: int,
    active_length_m: float,
    submerged_weight_n_per_m: float,
    water_depth_m: float,
) -> tuple[tuple[Vector3, ...], tuple[float, ...], tuple[bool, ...]] | None:
    """构造 TDP 处斜率为零、后接平直已铺尾段的悬链线。"""

    if element_count <= 0 or active_length_m <= 0.0 or submerged_weight_n_per_m <= 0.0:
        return None
    if abs(plough[2] - water_depth_m) > 1.0e-6 or vessel[2] >= water_depth_m:
        return None
    horizontal_vector = (plough[0] - vessel[0], plough[1] - vessel[1])
    route_span_m = math.hypot(horizontal_vector[0], horizontal_vector[1])
    vertical_drop_m = water_depth_m - vessel[2]
    if route_span_m <= _MIN_LENGTH or vertical_drop_m <= _MIN_LENGTH:
        return None
    excess_ratio = (active_length_m - route_span_m) / vertical_drop_m
    if not 0.0 < excess_ratio < 1.0:
        return None

    touchdown_argument = _solve_touchdown_catenary_argument(excess_ratio)
    parameter_m = vertical_drop_m / (math.cosh(touchdown_argument) - 1.0)
    suspended_horizontal_m = parameter_m * touchdown_argument
    suspended_arc_m = parameter_m * math.sinh(touchdown_argument)
    laid_tail_m = route_span_m - suspended_horizontal_m
    if laid_tail_m <= 1.0e-9:
        return None
    if abs((suspended_arc_m + laid_tail_m) - active_length_m) > 1.0e-7:
        return None

    route_unit = (horizontal_vector[0] / route_span_m, horizontal_vector[1] / route_span_m)
    positions: list[Vector3] = []
    contact_flags: list[bool] = []
    for node_index in range(element_count + 1):
        station_m = active_length_m * node_index / element_count
        if station_m <= suspended_arc_m:
            remaining_arc_m = max(0.0, suspended_arc_m - station_m)
            argument = math.asinh(remaining_arc_m / parameter_m)
            route_distance_m = suspended_horizontal_m - parameter_m * argument
            depth_m = water_depth_m - parameter_m * (math.cosh(argument) - 1.0)
        else:
            route_distance_m = suspended_horizontal_m + (station_m - suspended_arc_m)
            depth_m = water_depth_m
        positions.append(
            (
                vessel[0] + route_unit[0] * route_distance_m,
                vessel[1] + route_unit[1] * route_distance_m,
                min(water_depth_m, max(vessel[2], depth_m)),
            )
        )
        contact_flags.append(
            0 < node_index < element_count and station_m >= suspended_arc_m - 1.0e-9
        )
    positions[0] = vessel
    positions[-1] = plough

    horizontal_tension_n = submerged_weight_n_per_m * parameter_m
    tensions: list[float] = []
    segment_length_m = active_length_m / element_count
    for segment_index in range(element_count):
        center_station_m = (segment_index + 0.5) * segment_length_m
        if center_station_m < suspended_arc_m:
            remaining_arc_m = suspended_arc_m - center_station_m
            argument = math.asinh(remaining_arc_m / parameter_m)
            tensions.append(horizontal_tension_n * math.cosh(argument))
        else:
            tensions.append(horizontal_tension_n)
    return tuple(positions), tuple(tensions), tuple(contact_flags)


def _solve_touchdown_catenary_argument(excess_ratio: float) -> float:
    """求解接触悬链线参数方程 (sinh(u)-u)/(cosh(u)-1)。"""

    if not 0.0 < excess_ratio < 1.0:
        raise ValueError("excess_ratio must lie strictly between zero and one")

    def ratio(argument: float) -> float:
        return (math.sinh(argument) - argument) / (math.cosh(argument) - 1.0)

    low = 1.0e-8
    high = 1.0
    while ratio(high) < excess_ratio:
        high *= 2.0
        if high > 64.0:
            raise ValueError("failed to bracket touchdown catenary argument")
    for _ in range(120):
        mid = 0.5 * (low + high)
        if ratio(mid) < excess_ratio:
            low = mid
        else:
            high = mid
    return high


def _solve_initial_catenary_parameter(*, horizontal_span_m: float, reduced_length_m: float) -> float:
    """根据水平跨距和约化弧长求解悬链线参数。"""

    def span_for(parameter_m: float) -> float:
        argument = horizontal_span_m / (2.0 * parameter_m)
        if argument > 700.0:
            return math.inf
        return 2.0 * parameter_m * math.sinh(argument)

    low = max(horizontal_span_m, 1.0e-9) / 1400.0
    high = max(horizontal_span_m, reduced_length_m, 1.0)
    while span_for(high) > reduced_length_m:
        high *= 2.0
        if high > 1.0e18:
            raise ValueError("failed to bracket initial catenary parameter")
    for _ in range(120):
        mid = 0.5 * (low + high)
        if span_for(mid) > reduced_length_m:
            low = mid
        else:
            high = mid
    return high


def _initial_endpoint_curve(
    vessel: Vector3,
    plough: Vector3,
    *,
    element_count: int,
    water_depth_m: float,
) -> tuple[Vector3, ...]:
    direct = _sub(plough, vessel)
    horizontal_span = math.hypot(direct[0], direct[1])
    sag = min(max(0.03 * max(horizontal_span, water_depth_m), 0.5), max(0.0, water_depth_m - max(vessel[2], plough[2])))
    points: list[Vector3] = []
    for index in range(element_count + 1):
        fraction = index / element_count
        baseline = _add(vessel, _mul(direct, fraction))
        sag_z = sag * 4.0 * fraction * (1.0 - fraction)
        points.append((baseline[0], baseline[1], min(water_depth_m, baseline[2] + sag_z)))
    points[0] = vessel
    points[-1] = plough
    return tuple(points)


# 物理时间积分与耦合约束。
def _step_known_plough_dynamic(
    dynamic_case,
    case: StepConditions,
    state: DynamicLayingState,
    *,
    time_s: float,
    dt_s: float,
    seabed_friction_coefficient: float = _SEABED_FRICTION_COEFFICIENT,
) -> DynamicLayingState:
    """在给定端点轨迹之间推进一个 ALE/XPBD 时间步。

    计算顺序不可交换：先在参考域闭合材料通量，将材料重分区到网格，再计算
    非轴向载荷、预测网格运动、施加轴向与海床约束并恢复反力，最后使守恒材料
    单元与已接收状态同步。
    """

    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    _validate_state(state)
    next_time = time_s + dt_s
    vessel = _vessel_position(dynamic_case, next_time)
    plough = _plough_position(dynamic_case, next_time)
    vessel_velocity = _vessel_velocity(dynamic_case, next_time)
    plough_velocity = _plough_velocity(dynamic_case, next_time)
    payout_speed = _payout_speed(dynamic_case, time_s)
    plough_transfer_speed, _ = _plough_exit_material_speed(
        dynamic_case,
        _vessel_velocity(dynamic_case, time_s),
        time_s=time_s,
    )
    previous_material_length = _state_material_suspended_length(state)
    payout_increment = payout_speed * dt_s
    laydown_increment = plough_transfer_speed * dt_s
    # 参考材料连续性与端点几何跨距相互独立。
    material_active_length = (
        previous_material_length + payout_increment - laydown_increment
    )
    configured_element_count = int(dynamic_case.element_count)
    material_control = state.known_plough_material_control_volume
    uniform_ale_eligible = (
        not any(state.contact_flags)
        and len(state.rest_lengths_m) == configured_element_count
        and len(state.positions) == configured_element_count + 1
        and len(state.velocities) == configured_element_count + 1
        and len(state.contact_flags) == configured_element_count + 1
        and material_control is not None
        and len(material_control.material_cells) == configured_element_count
    )
    # 接触或非均匀拓扑采用局部守恒输运；悬空且节点数固定的跨段可直接使用均匀 ALE 重分区。
    if not uniform_ale_eligible:
        fallback_transport_state = _synchronize_material_cells_from_state(
            state,
            fairlead_speed_mps=payout_increment / dt_s,
            plough_speed_mps=laydown_increment / dt_s,
        )
        material_state = _advance_known_plough_material_flow(
            fallback_transport_state,
            payout_increment_m=payout_increment,
            laydown_increment_m=laydown_increment,
            target_segment_length_m=_target_segment_length(
                state.rest_lengths_m,
                None,
            ),
            dt_s=dt_s,
            seabed_depth_m=dynamic_case.water_depth_m,
        )
        material_state = replace(
            material_state,
            paid_length_m=state.paid_length_m + payout_increment,
            laid_length_m=state.laid_length_m + laydown_increment,
            material_suspended_length_m=material_active_length,
        )
    else:
        material_state = _rezone_known_plough_uniform_ale(
            state,
            case=case,
            new_active_length_m=material_active_length,
            element_count=configured_element_count,
            payout_increment_m=payout_increment,
            laydown_increment_m=laydown_increment,
            dt_s=dt_s,
            fairlead_prescribed_acceleration=_vessel_acceleration_vector(
                dynamic_case,
                next_time,
            ),
            plough_prescribed_acceleration=_plough_acceleration(
                dynamic_case,
                next_time,
            ),
        )
    material_active_length = sum(material_state.rest_lengths_m)
    geometric_length_deficit = max(0.0, _norm(_sub(plough, vessel)) - material_active_length)
    paid_length = material_state.paid_length_m
    laid_length = material_state.laid_length_m
    rest_lengths = material_state.rest_lengths_m
    anchored_state = DynamicLayingState(
        time_s=material_state.time_s,
        positions=material_state.positions,
        velocities=material_state.velocities,
        rest_lengths_m=rest_lengths,
        paid_length_m=material_state.paid_length_m,
        laid_length_m=material_state.laid_length_m,
        contact_flags=material_state.contact_flags,
        length_lambdas_n_s2=tuple(0.0 for _ in rest_lengths),
        contact_lambdas_n_s2=tuple(0.0 for _ in material_state.positions),
        segment_tensions_n=(),
        length_constraint_reactions_n=(),
        contact_normal_reactions_n=tuple(0.0 for _ in material_state.positions),
        material_suspended_length_m=material_active_length,
        known_plough_material_control_volume=(
            material_state.known_plough_material_control_volume
        ),
        geometric_length_deficit_m=geometric_length_deficit,
        material_remap_energy_error_per_linear_density_m3_s2=(
            material_state.material_remap_energy_error_per_linear_density_m3_s2
        ),
        material_remap_energy_error_cumulative_per_linear_density_m3_s2=(
            material_state.material_remap_energy_error_cumulative_per_linear_density_m3_s2
        ),
        material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2=(
            material_state.material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2
        ),
    )
    # 轴向张力由 XPBD 恢复，因此显式预测器排除该项，避免重复施加同一内力。
    forces = list(compute_forces(
        case,
        anchored_state,
        seabed_depth_m=None,
        payout_speed_mps=payout_speed,
        plough_exit_speed_mps=plough_transfer_speed,
        include_axial_tension=False,
    ))
    masses = _node_masses(case, anchored_state)
    inverse_masses = tuple(
        0.0 if index in (0, len(anchored_state.positions) - 1) else 1.0 / max(mass, _MIN_MASS)
        for index, mass in enumerate(masses)
    )
    axial_inverse_mass_matrices = _node_inverse_mass_matrices(
        case,
        anchored_state,
        fixed_indices=(0, len(anchored_state.positions) - 1),
    )
    accelerations = _directional_node_accelerations(case, anchored_state, tuple(forces))
    predicted_velocities = [
        _add(velocity, _mul(acceleration, dt_s))
        for velocity, acceleration in zip(anchored_state.velocities, accelerations)
    ]
    predicted_velocities[0] = vessel_velocity
    predicted_velocities[-1] = plough_velocity
    predicted_velocities = list(_limit_endpoint_span_velocities(tuple(predicted_velocities), rest_lengths, dt_s))
    predicted_positions = [
        _add(position, _mul(velocity, dt_s))
        for position, velocity in zip(anchored_state.positions, predicted_velocities)
    ]
    predicted_positions[0] = vessel
    predicted_positions[-1] = plough
    # 端点是强制边界数据；耦合轴向与海床投影只移动内部自由度。
    (
        constrained_positions,
        length_lambdas,
        contact_lambdas,
        axial_solve_iterations,
        axial_constraint_residual,
    ) = _solve_xpbd_endpoint_constraints(
        case,
        positions=tuple(predicted_positions),
        rest_lengths_m=rest_lengths,
        inverse_masses=inverse_masses,
        inverse_mass_matrices=axial_inverse_mass_matrices,
        seabed_depth_m=dynamic_case.water_depth_m,
        dt_s=dt_s,
        iterations=_KNOWN_PLOUGH_XPBD_ITERATIONS,
        minimum_iterations=_KNOWN_PLOUGH_XPBD_MIN_ITERATIONS,
        top_position=vessel,
        bottom_position=plough,
        initial_constraint_reactions_n=(),
    )
    constrained_velocities = [
        _mul(_sub(position, previous), 1.0 / dt_s)
        for previous, position in zip(anchored_state.positions, constrained_positions)
    ]
    constrained_velocities[0] = vessel_velocity
    constrained_velocities[-1] = plough_velocity
    contact_flags = tuple(
        False if index in (0, len(constrained_positions) - 1) else position[2] >= dynamic_case.water_depth_m - _SEABED_CONTACT_TOLERANCE_M
        for index, position in enumerate(constrained_positions)
    )
    contact_normal_reactions = tuple(
        max(0.0, lambda_value / (dt_s * dt_s))
        for lambda_value in contact_lambdas
    )
    merged_positions = constrained_positions
    current_material_control = material_state.known_plough_material_control_volume
    friction_state = DynamicLayingState(
        time_s=state.time_s,
        positions=constrained_positions,
        velocities=tuple(constrained_velocities),
        rest_lengths_m=rest_lengths,
        paid_length_m=paid_length,
        laid_length_m=laid_length,
        contact_flags=contact_flags,
        material_suspended_length_m=material_active_length,
        known_plough_material_control_volume=current_material_control,
        geometric_length_deficit_m=geometric_length_deficit,
        axial_solve_iterations=axial_solve_iterations,
        axial_constraint_residual_m=axial_constraint_residual,
        material_remap_energy_error_per_linear_density_m3_s2=(
            material_state.material_remap_energy_error_per_linear_density_m3_s2
        ),
        material_remap_energy_error_cumulative_per_linear_density_m3_s2=(
            material_state.material_remap_energy_error_cumulative_per_linear_density_m3_s2
        ),
        material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2=(
            material_state.material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2
        ),
    )
    _, constrained_velocities = _apply_contact_friction(
        positions=constrained_positions,
        previous_positions=merged_positions,
        velocities=tuple(constrained_velocities),
        contact_flags=contact_flags,
        contact_normal_reactions_n=contact_normal_reactions,
        masses=_node_masses(case, friction_state),
        payout_speed_mps=payout_speed,
        rest_lengths_m=rest_lengths,
        plough_exit_speed_mps=plough_transfer_speed,
        dt_s=dt_s,
        friction_coefficient=seabed_friction_coefficient,
        update_positions=False,
    )
    constrained_velocities = tuple(constrained_velocities)
    constrained_velocities = (
        vessel_velocity,
        *constrained_velocities[1:-1],
        plough_velocity,
    )
    # XPBD 乘子的单位为力乘时间平方；此转换得到下游张力输出采用的分段反力场。
    length_constraint_reactions = _segment_tensions_from_length_constraints(
        length_lambdas,
        dt_s=dt_s,
        expected_count=len(rest_lengths),
    )
    load_recursive_tensions = _step_dynamic_segment_tensions(
        case,
        positions=constrained_positions,
        velocities=constrained_velocities,
        rest_lengths_m=rest_lengths,
        payout_speed_mps=payout_speed,
        plough_exit_speed_mps=plough_transfer_speed,
        terminal_tension_n=0.0,
    )
    segment_tensions = length_constraint_reactions or load_recursive_tensions
    next_state = DynamicLayingState(
        time_s=state.time_s + dt_s,
        positions=constrained_positions,
        velocities=constrained_velocities,
        rest_lengths_m=rest_lengths,
        paid_length_m=paid_length,
        laid_length_m=laid_length,
        contact_flags=contact_flags,
        length_lambdas_n_s2=length_lambdas,
        contact_lambdas_n_s2=contact_lambdas,
        segment_tensions_n=segment_tensions,
        length_constraint_reactions_n=length_constraint_reactions,
        contact_normal_reactions_n=contact_normal_reactions,
        material_suspended_length_m=material_active_length,
        known_plough_material_control_volume=current_material_control,
        geometric_length_deficit_m=geometric_length_deficit,
        axial_solve_iterations=axial_solve_iterations,
        axial_constraint_residual_m=axial_constraint_residual,
        material_remap_energy_error_per_linear_density_m3_s2=(
            material_state.material_remap_energy_error_per_linear_density_m3_s2
        ),
        material_remap_energy_error_cumulative_per_linear_density_m3_s2=(
            material_state.material_remap_energy_error_cumulative_per_linear_density_m3_s2
        ),
        material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2=(
            material_state.material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2
        ),
    )
    next_payout_speed = _payout_speed(dynamic_case, next_time)
    next_plough_transfer_speed, _ = _plough_exit_material_speed(
        dynamic_case,
        vessel_velocity,
        time_s=next_time,
    )
    next_state = _synchronize_material_cells_from_state(
        next_state,
        fairlead_speed_mps=next_payout_speed,
        plough_speed_mps=next_plough_transfer_speed,
    )
    return next_state




def _solve_xpbd_endpoint_constraints(
    case: StepConditions,
    *,
    positions: tuple[Vector3, ...],
    rest_lengths_m: tuple[float, ...],
    inverse_masses: tuple[float, ...],
    inverse_mass_matrices=None,
    seabed_depth_m: float,
    dt_s: float,
    iterations: int,
    minimum_iterations: int | None = None,
    top_position: Vector3,
    bottom_position: Vector3,
    initial_constraint_reactions_n: tuple[float, ...] = (),
) -> tuple[tuple[Vector3, ...], tuple[float, ...], tuple[float, ...], int, float]:
    """在端点固定条件下迭代求解耦合拉伸约束和海床约束。

    轴向约束为单侧约束，松弛段不承受压缩乘子；海床乘子同样保持非负。
    每次迭代后恢复端点坐标，防止投影移动给定的船端或犁端边界。
    """

    solved = list(positions)
    solved[0] = top_position
    solved[-1] = bottom_position
    if initial_constraint_reactions_n and len(initial_constraint_reactions_n) != len(rest_lengths_m):
        raise ValueError("initial_constraint_reactions_n must have one entry per segment")
    length_lambdas = [
        max(0.0, reaction) * dt_s * dt_s
        for reaction in (
            initial_constraint_reactions_n
            if initial_constraint_reactions_n
            else (0.0 for _ in rest_lengths_m)
        )
    ]
    contact_lambdas = [0.0 for _ in solved]
    axial_stiffness = max(case.cable.axial_stiffness_n, _MIN_MASS)
    bend_projection_radius_m = _feasible_bend_projection_radius_m(
        requested_radius_m=case.cable.min_bending_radius_m,
        rest_lengths_m=rest_lengths_m,
        top_position=top_position,
        bottom_position=bottom_position,
    )
    maximum_iterations = max(1, iterations)
    convergence_check_start = (
        maximum_iterations
        if minimum_iterations is None
        else min(maximum_iterations, max(1, minimum_iterations))
    )
    axial_residual_m = math.inf
    for iteration_index in range(maximum_iterations):
        axial_step = solve_global_axial_constraint_step(
            positions=solved,
            rest_lengths_m=rest_lengths_m,
            inverse_masses_per_kg=inverse_masses,
            inverse_mass_matrices_per_kg=inverse_mass_matrices,
            axial_stiffness_n=axial_stiffness,
            dt_s=dt_s,
            lambdas_n_s2=length_lambdas,
        )
        solved = list(axial_step.positions)
        length_lambdas = list(axial_step.lambdas_n_s2)
        solved[0] = top_position
        solved[-1] = bottom_position
        _apply_minimum_bend_radius_constraints(
            solved,
            inverse_masses=inverse_masses,
            minimum_radius_m=bend_projection_radius_m,
        )
        solved[0] = top_position
        solved[-1] = bottom_position
        for index in range(1, len(solved) - 1):
            penetration = solved[index][2] - seabed_depth_m
            if penetration <= 0.0:
                continue
            wi = inverse_masses[index]
            if wi <= _MIN_MASS:
                continue
            contact_lambdas[index] += penetration / wi
            solved[index] = (solved[index][0], solved[index][1], seabed_depth_m)
        _apply_segment_spacing_floor_constraints(
            solved,
            rest_lengths_m=rest_lengths_m,
            inverse_masses=inverse_masses,
        )
        solved[0] = top_position
        solved[-1] = bottom_position
        if iteration_index + 1 >= convergence_check_start:
            axial_residual_m = axial_constraint_residual_m(
                positions=solved,
                rest_lengths_m=rest_lengths_m,
                lambdas_n_s2=length_lambdas,
                axial_stiffness_n=axial_stiffness,
                dt_s=dt_s,
            )
            if axial_residual_m <= _KNOWN_PLOUGH_AXIAL_RESIDUAL_TOLERANCE_M:
                break
    if axial_residual_m > _KNOWN_PLOUGH_AXIAL_RESIDUAL_TOLERANCE_M:
        raise RuntimeError(
            "global axial constraints did not converge: "
            f"residual {axial_residual_m:.6g} m exceeds "
            f"{_KNOWN_PLOUGH_AXIAL_RESIDUAL_TOLERANCE_M:.6g} m"
        )
    return (
        tuple(solved),
        tuple(length_lambdas),
        tuple(contact_lambdas),
        iteration_index + 1,
        axial_residual_m,
    )


def _feasible_bend_projection_radius_m(
    *,
    requested_radius_m: float | None,
    rest_lengths_m: tuple[float, ...],
    top_position: Vector3,
    bottom_position: Vector3,
) -> float | None:
    """校验固定端点和活动弧长能否实现请求的最小弯曲半径。"""

    if requested_radius_m is None:
        return None
    if not math.isfinite(requested_radius_m):
        raise ValueError("requested minimum bend radius must be finite")
    if requested_radius_m <= 0.0:
        raise ValueError("requested minimum bend radius must be positive")
    if requested_radius_m <= _MIN_LENGTH:
        return requested_radius_m
    arc_length_m = sum(rest_lengths_m)
    chord_length_m = math.dist(top_position, bottom_position)
    if arc_length_m <= chord_length_m + _MIN_LENGTH:
        return requested_radius_m
    chord_ratio = max(0.0, min(1.0, chord_length_m / arc_length_m))
    lower_angle = 0.0
    upper_angle = math.pi
    for _ in range(80):
        half_angle = 0.5 * (lower_angle + upper_angle)
        ratio = math.sin(half_angle) / half_angle
        if ratio > chord_ratio:
            lower_angle = half_angle
        else:
            upper_angle = half_angle
    equivalent_arc_radius_m = arc_length_m / (lower_angle + upper_angle)
    if requested_radius_m > equivalent_arc_radius_m * (1.0 + 1.0e-12):
        raise ValueError(
            "requested minimum bend radius "
            f"{requested_radius_m:.12g} m exceeds maximum feasible radius "
            f"{equivalent_arc_radius_m:.12g} m for the fixed endpoints and active arc length"
        )
    return requested_radius_m


def _apply_minimum_bend_radius_constraints(
    positions: list[Vector3],
    *,
    inverse_masses: tuple[float, ...],
    minimum_radius_m: float | None,
) -> None:
    """配置工程最小弯曲半径时减小局部折角。"""

    if minimum_radius_m is None or minimum_radius_m <= _MIN_LENGTH or len(positions) < 3:
        return
    for index in range(1, len(positions) - 1):
        wi = inverse_masses[index]
        if wi <= _MIN_MASS:
            continue
        previous = positions[index - 1]
        current = positions[index]
        next_point = positions[index + 1]
        first = _sub(current, previous)
        second = _sub(next_point, current)
        first_length = _norm(first)
        second_length = _norm(second)
        if first_length <= _MIN_LENGTH or second_length <= _MIN_LENGTH:
            continue
        dot = max(-1.0, min(1.0, _dot(first, second) / (first_length * second_length)))
        turn = math.acos(dot)
        max_turn = 0.5 * (first_length + second_length) / minimum_radius_m
        if turn <= max_turn:
            continue
        chord = _sub(next_point, previous)
        chord_length2 = max(_dot(chord, chord), _MIN_LENGTH)
        fraction = max(0.0, min(1.0, _dot(_sub(current, previous), chord) / chord_length2))
        foot = _add(previous, _mul(chord, fraction))
        relaxation = min(0.65, max(0.05, (turn - max_turn) / max(turn, _MIN_LENGTH)))
        positions[index] = _add(current, _mul(_sub(foot, current), relaxation))


def _apply_segment_spacing_floor_constraints(
    positions: list[Vector3],
    *,
    rest_lengths_m: tuple[float, ...],
    inverse_masses: tuple[float, ...],
) -> None:
    """防止松弛节点坍缩为小于网格尺度的接触簇。"""

    for index, rest_length in enumerate(rest_lengths_m):
        floor_length = _SEGMENT_SPACING_FLOOR_FRACTION * rest_length
        if floor_length <= _MIN_LENGTH:
            continue
        start = positions[index]
        end = positions[index + 1]
        delta = _sub(end, start)
        length = _norm(delta)
        if length >= floor_length:
            continue
        direction = _spacing_floor_direction(positions, index, delta)
        correction = floor_length - length
        wi = inverse_masses[index]
        wj = inverse_masses[index + 1]
        total_weight = wi + wj
        if total_weight <= _MIN_MASS:
            continue
        if wi > _MIN_MASS:
            positions[index] = _sub(start, _mul(direction, correction * wi / total_weight))
        if wj > _MIN_MASS:
            positions[index + 1] = _add(end, _mul(direction, correction * wj / total_weight))


def _spacing_floor_direction(
    positions: list[Vector3],
    index: int,
    delta: Vector3,
) -> Vector3:
    length = _norm(delta)
    if length > _MIN_LENGTH:
        return _mul(delta, 1.0 / length)
    if index > 0:
        previous = _sub(positions[index], positions[index - 1])
        previous_length = _norm(previous)
        if previous_length > _MIN_LENGTH:
            return _mul(previous, 1.0 / previous_length)
    if index + 2 < len(positions):
        next_delta = _sub(positions[index + 2], positions[index + 1])
        next_length = _norm(next_delta)
        if next_length > _MIN_LENGTH:
            return _mul(next_delta, 1.0 / next_length)
    return (1.0, 0.0, 0.0)


def minimum_bend_radius_status(
    *,
    minimum_radius_m: float | None,
    limit_m: float | None,
) -> str:
    if limit_m is None:
        return "not_configured"
    if minimum_radius_m is None or not math.isfinite(minimum_radius_m):
        return "not_available"
    if minimum_radius_m + 1.0e-6 < limit_m:
        return "below_limit"
    return "ok"


def _plough_tension_status(
    *,
    boundary_tension_n: float | None,
    adjacent_segment_tension_n: float | None,
) -> str:
    """判断犁端边界张力是否由悬跨段承担。"""

    if boundary_tension_n is None or adjacent_segment_tension_n is None:
        return "not_available"
    boundary = max(0.0, boundary_tension_n)
    adjacent = max(0.0, adjacent_segment_tension_n)
    if boundary <= 1.0e-9:
        return "free_or_unset"
    if adjacent <= max(1.0e-6, 0.01 * boundary):
        return "slack_or_unclosed"
    if adjacent < 0.5 * boundary:
        return "low_adjacent_tension"
    return "carried"




def _state_material_suspended_length(state: DynamicLayingState) -> float:
    material_length = state.material_suspended_length_m
    if material_length > _MIN_LENGTH and math.isfinite(material_length):
        return material_length
    return state.suspended_length_m




def _plough_exit_material_speed(
    dynamic_case,
    vessel_velocity: Vector3,
    *,
    time_s: float | None = None,
) -> tuple[float, str]:
    """返回下端材料边界速度及其声明来源。

    犁出口编码器测量值是下端材料通量边界。该测量不可用时，工程回退值取作业航迹
    纵向船速，不采用横向速度或速度模值。
    """

    samples = getattr(dynamic_case, "plough_exit_speed_samples", ())
    if samples:
        if time_s is None:
            time_s = samples[-1].time_s
        return _sampled_scalar_value(samples, time_s), "measured"
    prescribed_speed = getattr(dynamic_case, "plough_exit_speed_mps", None)
    if prescribed_speed is not None:
        return prescribed_speed, "explicit"
    return _vessel_longitudinal_material_speed(vessel_velocity), "vessel_longitudinal_inferred"


def _vessel_longitudinal_material_speed(vessel_velocity: Vector3) -> float:
    """根据船舶沿作业航迹 +X 的速度推定下端材料流出速度。

    横向运动会改变三维边界，但不计入该标量回退值。当前工程合同规定材料流出速度
    非负，因此反向纵向运动会被截断。
    """

    return max(0.0, vessel_velocity[0])


# P1 材料速度重构与守恒单元积分。
def _known_plough_node_material_velocities(
    *,
    positions: tuple[Vector3, ...],
    grid_velocities: tuple[Vector3, ...],
    rest_lengths_m: tuple[float, ...],
    fairlead_speed_mps: float,
    plough_speed_mps: float,
) -> tuple[Vector3, ...]:
    """合成 ALE 网格速度与材料相对网格的输运速度。

    结果是 Morison 载荷和接触摩擦采用的缆线材料物理速度；材料穿越任一运动边界时，
    仅使用网格速度并不充分。
    """

    segments = segment_vectors(positions)
    flow_speeds = _node_material_flow_speeds(
        rest_lengths_m,
        fairlead_speed_mps=fairlead_speed_mps,
        plough_speed_mps=plough_speed_mps,
    )
    return tuple(
        _add(
            grid_velocity,
            _mul(_node_tangent(segments, index), flow_speed),
        )
        for index, (grid_velocity, flow_speed) in enumerate(
            zip(grid_velocities, flow_speeds)
        )
    )


def _post_flux_material_p1_reference(
    state: DynamicLayingState,
    *,
    payout_increment_m: float,
    laydown_increment_m: float,
    fairlead_speed_mps: float,
    plough_speed_mps: float,
) -> tuple[tuple[float, ...], tuple[Vector3, ...]]:
    """返回通量更新后定义在新活动坐标上的 P1 材料场。"""

    old_coordinates = _cumulative_coordinates(list(state.rest_lengths_m))
    old_active_length = old_coordinates[-1]
    retained_old_length = old_active_length - laydown_increment_m
    tolerance = 64.0 * math.ulp(max(old_active_length, abs(retained_old_length), 1.0))
    if retained_old_length < -tolerance:
        raise RuntimeError("plough outflow exceeds the active P1 material field")
    retained_old_length = max(0.0, retained_old_length)
    old_material_velocities = _known_plough_node_material_velocities(
        positions=state.positions,
        grid_velocities=state.velocities,
        rest_lengths_m=state.rest_lengths_m,
        fairlead_speed_mps=fairlead_speed_mps,
        plough_speed_mps=plough_speed_mps,
    )
    source_coordinates = [0.0]
    source_velocities = [old_material_velocities[0]]
    if payout_increment_m > _MIN_LENGTH:
        source_coordinates.append(payout_increment_m)
        source_velocities.append(old_material_velocities[0])
    for coordinate, velocity in zip(old_coordinates[1:], old_material_velocities[1:]):
        if coordinate >= retained_old_length - tolerance:
            break
        source_coordinates.append(payout_increment_m + coordinate)
        source_velocities.append(velocity)
    new_active_length = payout_increment_m + retained_old_length
    tail_velocity = (
        _sample_monotone_material_vectors(
            list(old_material_velocities),
            old_coordinates,
            [retained_old_length],
        )[0]
        if retained_old_length > _MIN_LENGTH
        else old_material_velocities[0]
    )
    if new_active_length - source_coordinates[-1] > tolerance:
        source_coordinates.append(new_active_length)
        source_velocities.append(tail_velocity)
    else:
        source_coordinates[-1] = new_active_length
        source_velocities[-1] = tail_velocity
    if len(source_coordinates) < 2:
        raise RuntimeError("post-flux P1 material field has no active interval")
    return tuple(source_coordinates), tuple(source_velocities)


def _p1_cross_mass_rhs(
    source_coordinates_m: tuple[float, ...],
    source_velocities_mps: tuple[Vector3, ...],
    target_coordinates_m: tuple[float, ...],
) -> tuple[Vector3, ...]:
    """以 O(N) 复杂度积分目标 P1 形函数与源 P1 场的乘积。"""

    if len(source_coordinates_m) != len(source_velocities_mps):
        raise ValueError("source P1 coordinates and velocities must align")
    if len(source_coordinates_m) < 2 or len(target_coordinates_m) < 2:
        raise ValueError("P1 cross-mass integration requires non-empty grids")
    if any(
        right <= left
        for left, right in zip(source_coordinates_m, source_coordinates_m[1:])
    ) or any(
        right <= left
        for left, right in zip(target_coordinates_m, target_coordinates_m[1:])
    ):
        raise ValueError("P1 cross-mass coordinates must be strictly increasing")
    domain_scale = max(
        abs(source_coordinates_m[-1]),
        abs(target_coordinates_m[-1]),
        1.0,
    )
    domain_tolerance = 64.0 * math.ulp(domain_scale)
    if (
        abs(source_coordinates_m[0] - target_coordinates_m[0]) > domain_tolerance
        or abs(source_coordinates_m[-1] - target_coordinates_m[-1]) > domain_tolerance
    ):
        raise ValueError("source and target P1 grids must cover the same domain")
    rhs = [(0.0, 0.0, 0.0) for _ in target_coordinates_m]
    source_index = 0
    target_index = 0
    gauss_offset = 1.0 / math.sqrt(3.0)
    while (
        source_index < len(source_coordinates_m) - 1
        and target_index < len(target_coordinates_m) - 1
    ):
        source_left = source_coordinates_m[source_index]
        source_right = source_coordinates_m[source_index + 1]
        target_left = target_coordinates_m[target_index]
        target_right = target_coordinates_m[target_index + 1]
        overlap_left = max(source_left, target_left)
        overlap_right = min(source_right, target_right)
        if overlap_right > overlap_left:
            midpoint = 0.5 * (overlap_left + overlap_right)
            half_width = 0.5 * (overlap_right - overlap_left)
            for sign in (-1.0, 1.0):
                coordinate = midpoint + sign * gauss_offset * half_width
                source_fraction = (
                    (coordinate - source_left) / (source_right - source_left)
                )
                target_fraction = (
                    (coordinate - target_left) / (target_right - target_left)
                )
                source_velocity = _add(
                    _mul(source_velocities_mps[source_index], 1.0 - source_fraction),
                    _mul(source_velocities_mps[source_index + 1], source_fraction),
                )
                rhs[target_index] = _add(
                    rhs[target_index],
                    _mul(source_velocity, half_width * (1.0 - target_fraction)),
                )
                rhs[target_index + 1] = _add(
                    rhs[target_index + 1],
                    _mul(source_velocity, half_width * target_fraction),
                )
        if source_right <= target_right:
            source_index += 1
        if target_right <= source_right:
            target_index += 1
    if not all(math.isfinite(component) for value in rhs for component in value):
        raise RuntimeError("P1 cross-mass right-hand side is non-finite")
    return tuple(rhs)


def _factor_material_velocity_mass_tridiagonal(
    diagonal: list[float],
    off_diagonal: list[float],
) -> tuple[list[float], list[float]]:
    """分解固定的 P1 一致质量三对角矩阵，供重复求解使用。"""

    if not diagonal or len(off_diagonal) != len(diagonal) - 1:
        raise ValueError("invalid material velocity tridiagonal dimensions")
    if not all(math.isfinite(value) and value > 0.0 for value in diagonal):
        raise RuntimeError("material velocity mass diagonal must be finite and positive")
    if not all(math.isfinite(value) for value in off_diagonal):
        raise RuntimeError("material velocity mass off-diagonal must be finite")
    pivots = [diagonal[0]]
    lower_factors: list[float] = []
    for index, coupling in enumerate(off_diagonal, start=1):
        lower_factor = coupling / pivots[-1]
        pivot = diagonal[index] - lower_factor * coupling
        if not math.isfinite(lower_factor):
            raise RuntimeError("material velocity LDL factor is non-finite")
        if not math.isfinite(pivot) or pivot <= 0.0:
            raise RuntimeError("material velocity mass matrix is not positive definite")
        lower_factors.append(lower_factor)
        pivots.append(pivot)
    return pivots, lower_factors


def _solve_factored_material_velocity_mass_tridiagonal(
    pivots: list[float],
    lower_factors: list[float],
    right_hand_side: list[float],
) -> list[float]:
    """使用预分解质量矩阵求解一个 P1 速度分量。"""

    if (
        not pivots
        or len(right_hand_side) != len(pivots)
        or len(lower_factors) != len(pivots) - 1
    ):
        raise ValueError("invalid factored material velocity system dimensions")
    if not all(math.isfinite(value) for value in right_hand_side):
        raise RuntimeError("material velocity right-hand side is non-finite")
    forward = [right_hand_side[0]]
    for index in range(1, len(pivots)):
        forward.append(
            right_hand_side[index]
            - lower_factors[index - 1] * forward[index - 1]
        )
    diagonal_solution = [
        value / pivot for value, pivot in zip(forward, pivots)
    ]
    solution = [0.0 for _ in pivots]
    solution[-1] = diagonal_solution[-1]
    for index in range(len(pivots) - 2, -1, -1):
        solution[index] = (
            diagonal_solution[index]
            - lower_factors[index] * solution[index + 1]
        )
    if not all(math.isfinite(value) for value in solution):
        raise RuntimeError("material velocity tridiagonal solution is non-finite")
    return solution


def _unconstrained_material_velocity_l2_projection(
    l2_rhs_per_linear_density_m2_s: tuple[Vector3, ...],
    rest_lengths_m: tuple[float, ...],
) -> tuple[Vector3, ...]:
    """将 P1 参考场无约束投影到目标 P1 网格。"""

    if not rest_lengths_m:
        raise ValueError("material velocity L2 projection requires at least one segment")
    if len(l2_rhs_per_linear_density_m2_s) != len(rest_lengths_m) + 1:
        raise ValueError("material velocity L2 right-hand side must have one value per node")
    if not all(
        math.isfinite(component)
        for value in l2_rhs_per_linear_density_m2_s
        for component in value
    ):
        raise RuntimeError("material velocity L2 right-hand side is non-finite")
    diagonal = [rest_lengths_m[0] / 3.0]
    diagonal.extend(
        (rest_lengths_m[index - 1] + rest_lengths_m[index]) / 3.0
        for index in range(1, len(rest_lengths_m))
    )
    diagonal.append(rest_lengths_m[-1] / 3.0)
    off_diagonal = [length / 6.0 for length in rest_lengths_m]
    pivots, lower_factors = _factor_material_velocity_mass_tridiagonal(
        diagonal,
        off_diagonal,
    )
    components = [
        _solve_factored_material_velocity_mass_tridiagonal(
            pivots,
            lower_factors,
            [value[component] for value in l2_rhs_per_linear_density_m2_s],
        )
        for component in range(3)
    ]
    return tuple(
        tuple(components[component][index] for component in range(3))
        for index in range(len(rest_lengths_m) + 1)
    )


def _material_node_velocities_from_cells(
    cells: tuple[_MaterialCellIntegral, ...],
    rest_lengths_m: tuple[float, ...],
    *,
    left_endpoint_velocity_mps: Vector3,
    right_endpoint_velocity_mps: Vector3,
    l2_rhs_per_linear_density_m2_s: tuple[Vector3, ...] | None = None,
    target_total_momentum_per_linear_density_m2_s: Vector3 | None = None,
) -> tuple[Vector3, ...]:
    """以 O(N) 复杂度将速度场约束 L2 投影为连续线性场。"""

    if len(cells) != len(rest_lengths_m) or not cells:
        raise ValueError("material velocity reconstruction requires one cell per segment")
    if len(rest_lengths_m) < 2:
        raise ValueError("material velocity reconstruction requires at least two segments")
    for cell, rest_length in zip(cells, rest_lengths_m):
        _validate_material_cell_integral(cell)
        tolerance = 64.0 * math.ulp(max(abs(cell.length_m), abs(rest_length)))
        if abs(cell.length_m - rest_length) > tolerance:
            raise ValueError("material velocity reconstruction grid does not match cells")

    endpoints = (left_endpoint_velocity_mps, right_endpoint_velocity_mps)
    if l2_rhs_per_linear_density_m2_s is not None:
        if len(l2_rhs_per_linear_density_m2_s) != len(rest_lengths_m) + 1:
            raise ValueError("material velocity L2 right-hand side must have one value per node")
        if not all(
            math.isfinite(component)
            for value in l2_rhs_per_linear_density_m2_s
            for component in value
        ):
            raise RuntimeError("material velocity L2 right-hand side is non-finite")
    target_total_momentum = (
        target_total_momentum_per_linear_density_m2_s
        if target_total_momentum_per_linear_density_m2_s is not None
        else _sum_material_cells(cells).momentum_per_linear_density_m2_s
    )
    if not all(math.isfinite(component) for component in target_total_momentum):
        raise RuntimeError("target material momentum is non-finite")
    diagonal = [
        (left_length + right_length) / 3.0
        for left_length, right_length in zip(
            rest_lengths_m,
            rest_lengths_m[1:],
        )
    ]
    off_diagonal = [length / 6.0 for length in rest_lengths_m[1:-1]]
    pivots, lower_factors = _factor_material_velocity_mass_tridiagonal(
        diagonal,
        off_diagonal,
    )
    interior_weights = [
        0.5 * (left_length + right_length)
        for left_length, right_length in zip(
            rest_lengths_m,
            rest_lengths_m[1:],
        )
    ]
    inverse_mass_times_weight = _solve_factored_material_velocity_mass_tridiagonal(
        pivots,
        lower_factors,
        interior_weights,
    )
    denominator = math.fsum(
        weight * value
        for weight, value in zip(
            interior_weights,
            inverse_mass_times_weight,
        )
    )
    if not math.isfinite(denominator) or denominator <= 0.0:
        raise RuntimeError("material velocity Schur denominator must be finite and positive")

    interior_components: list[list[float]] = []
    for component in range(3):
        if l2_rhs_per_linear_density_m2_s is None:
            interior_rhs = [
                0.5
                * (
                    cells[index - 1].momentum_per_linear_density_m2_s[component]
                    + cells[index].momentum_per_linear_density_m2_s[component]
                )
                - (
                    rest_lengths_m[0] / 6.0 * endpoints[0][component]
                    if index == 1
                    else 0.0
                )
                - (
                    rest_lengths_m[-1] / 6.0 * endpoints[1][component]
                    if index == len(rest_lengths_m) - 1
                    else 0.0
                )
                for index in range(1, len(rest_lengths_m))
            ]
        else:
            interior_rhs = [
                l2_rhs_per_linear_density_m2_s[index][component]
                - (
                    rest_lengths_m[0] / 6.0 * endpoints[0][component]
                    if index == 1
                    else 0.0
                )
                - (
                    rest_lengths_m[-1] / 6.0 * endpoints[1][component]
                    if index == len(rest_lengths_m) - 1
                    else 0.0
                )
                for index in range(1, len(rest_lengths_m))
            ]
        target_interior_momentum = (
            target_total_momentum[component]
            - 0.5 * rest_lengths_m[0] * endpoints[0][component]
            - 0.5 * rest_lengths_m[-1] * endpoints[1][component]
        )
        inverse_mass_times_rhs = _solve_factored_material_velocity_mass_tridiagonal(
            pivots,
            lower_factors,
            interior_rhs,
        )
        multiplier = (
            math.fsum(
                weight * value
                for weight, value in zip(
                    interior_weights,
                    inverse_mass_times_rhs,
                )
            )
            - target_interior_momentum
        ) / denominator
        if not math.isfinite(multiplier):
            raise RuntimeError("material velocity Schur multiplier is non-finite")
        interior_components.append(
            [
                value - correction * multiplier
                for value, correction in zip(
                    inverse_mass_times_rhs,
                    inverse_mass_times_weight,
                )
            ]
        )
    reconstructed = [left_endpoint_velocity_mps]
    reconstructed.extend(
        tuple(interior_components[component][index] for component in range(3))
        for index in range(len(rest_lengths_m) - 1)
    )
    reconstructed.append(right_endpoint_velocity_mps)
    if not all(math.isfinite(component) for value in reconstructed for component in value):
        raise RuntimeError("material-cell velocity reconstruction is non-finite")
    return tuple(reconstructed)




def _material_cells_from_state(
    state: DynamicLayingState,
    *,
    fairlead_speed_mps: float,
    plough_speed_mps: float,
) -> tuple[_MaterialCellIntegral, ...]:
    """在每个参考单元上积分当前 P1 材料状态。

    单元长度、动量、动能和应变矩共同构成后续开放边界输运与重分区所需的守恒源状态。
    """

    material_velocities = _known_plough_node_material_velocities(
        positions=state.positions,
        grid_velocities=state.velocities,
        rest_lengths_m=state.rest_lengths_m,
        fairlead_speed_mps=fairlead_speed_mps,
        plough_speed_mps=plough_speed_mps,
    )
    cells = []
    for segment, rest_length, left_velocity, right_velocity in zip(
        segment_vectors(state.positions),
        state.rest_lengths_m,
        material_velocities,
        material_velocities[1:],
    ):
        mean_axial_strain = (
            segment.length_m / max(rest_length, _MIN_LENGTH) - 1.0
        )
        cells.append(_MaterialCellIntegral(
            length_m=rest_length,
            momentum_per_linear_density_m2_s=_mul(
                _add(left_velocity, right_velocity),
                0.5 * rest_length,
            ),
            kinetic_energy_per_linear_density_m3_s2=(
                _linear_material_cell_kinetic_energy(
                    left_velocity,
                    right_velocity,
                    rest_length,
                )
            ),
            axial_strain_integral_m=(
                rest_length * mean_axial_strain
            ),
            axial_strain_squared_integral_m=(
                rest_length * mean_axial_strain**2
            ),
        ))
    return tuple(cells)


def _validate_material_cells_match_p1_geometry(state: DynamicLayingState) -> None:
    control = state.known_plough_material_control_volume
    if control is None or len(control.material_cells) != len(state.rest_lengths_m):
        raise RuntimeError("P1 geometry compatibility requires one material cell per segment")
    for segment, rest_length, cell in zip(
        segment_vectors(state.positions),
        state.rest_lengths_m,
        control.material_cells,
    ):
        length_scale = max(abs(segment.length_m), abs(rest_length), abs(cell.length_m), 1.0)
        tolerance = 64.0 * math.ulp(length_scale)
        if abs(cell.length_m - rest_length) > tolerance:
            raise RuntimeError("material-cell reference length does not match the P1 grid")
        implied_length = cell.length_m + cell.axial_strain_integral_m
        if implied_length <= _MIN_LENGTH:
            raise RuntimeError("material-cell axial strain implies a non-positive P1 length")
        if abs(implied_length - segment.length_m) > tolerance:
            raise RuntimeError("material-cell axial strain does not match the P1 geometry")


def _synchronize_material_cells_from_state(
    state: DynamicLayingState,
    *,
    fairlead_speed_mps: float,
    plough_speed_mps: float,
) -> DynamicLayingState:
    """根据已求解 P1 速度重建 P/K，同时保留应变方差。"""

    control = state.known_plough_material_control_volume
    if control is None:
        return state
    resolved_cells = _material_cells_from_state(
        state,
        fairlead_speed_mps=fairlead_speed_mps,
        plough_speed_mps=plough_speed_mps,
    )
    if control.material_cells:
        synchronized_cells = tuple(
            replace(
                resolved,
                axial_strain_squared_integral_m=(
                    resolved.axial_strain_squared_integral_m
                    + max(
                        0.0,
                        transported.axial_strain_squared_integral_m
                        - transported.axial_strain_integral_m**2
                        / transported.length_m,
                    )
                ),
            )
            for transported, resolved in zip(
                control.material_cells,
                resolved_cells,
            )
        )
        for cell in synchronized_cells:
            _validate_material_cell_moment_feasibility(cell)
        resolved_cells = synchronized_cells
    return replace(
        state,
        known_plough_material_control_volume=replace(
            control,
            material_cells=resolved_cells,
        ),
    )


def _endpoint_cut_cell_from_material_cell(
    cell: _MaterialCellIntegral,
) -> _EndpointMaterialCutCell:
    return _EndpointMaterialCutCell(
        length_m=cell.length_m,
        momentum_per_linear_density_m2_s=cell.momentum_per_linear_density_m2_s,
        kinetic_energy_per_linear_density_m3_s2=(
            cell.kinetic_energy_per_linear_density_m3_s2
        ),
        axial_strain_integral_m=cell.axial_strain_integral_m,
        axial_strain_squared_integral_m=cell.axial_strain_squared_integral_m,
    )


# 开放边界材料输运与 ALE 重分区。
def _advance_known_plough_material_control_volume(
    state: DynamicLayingState,
    *,
    payout_increment_m: float,
    laydown_increment_m: float,
    dt_s: float,
    target_rest_lengths_m: tuple[float, ...],
) -> _KnownPloughMaterialControlVolume:
    """推进两端切割单元及分布材料状态。

    分布单元与边界累计积分必须共同闭合同一参考长度平衡，
    包括材料反复跨越不足一个单元长度的情况。
    """

    _validate_state(state)
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    payout_increment = max(0.0, float(payout_increment_m))
    laydown_increment = max(0.0, float(laydown_increment_m))
    if not math.isfinite(payout_increment) or not math.isfinite(laydown_increment):
        raise ValueError("material increments must be finite and non-negative")

    fairlead_speed = payout_increment / dt_s
    plough_speed = laydown_increment / dt_s
    material_velocities = _known_plough_node_material_velocities(
        positions=state.positions,
        grid_velocities=state.velocities,
        rest_lengths_m=state.rest_lengths_m,
        fairlead_speed_mps=fairlead_speed,
        plough_speed_mps=plough_speed,
    )
    control = state.known_plough_material_control_volume or _KnownPloughMaterialControlVolume()
    material_cells = control.material_cells or _material_cells_from_state(
        state,
        fairlead_speed_mps=fairlead_speed,
        plough_speed_mps=plough_speed,
    )
    first_cell = material_cells[0]
    first_mean_strain = _material_cell_mean_signed_geometric_axial_strain(first_cell)
    first_mean_strain_squared = (
        first_cell.axial_strain_squared_integral_m / first_cell.length_m
        if first_cell.axial_strain_squared_integral_m > 0.0
        else first_mean_strain**2
    )
    incoming_cell = _MaterialCellIntegral(
        length_m=payout_increment,
        momentum_per_linear_density_m2_s=_mul(material_velocities[0], payout_increment),
        kinetic_energy_per_linear_density_m3_s2=(
            0.5 * payout_increment * _dot(material_velocities[0], material_velocities[0])
        ),
        axial_strain_integral_m=payout_increment * first_mean_strain,
        axial_strain_squared_integral_m=payout_increment * first_mean_strain_squared,
    )
    fairlead_cut_cell = _endpoint_cut_cell_from_material_cell(incoming_cell)
    last_rest_length = state.rest_lengths_m[-1]
    if laydown_increment > last_rest_length + _MIN_LENGTH:
        raise RuntimeError("plough material outflow exceeds the final material segment in one step")
    plough_integral = _integrate_linear_material_tail_slice(
        structural_linear_density_kg_m=1.0,
        parent_length_m=last_rest_length,
        exit_length_m=min(laydown_increment, last_rest_length),
        left_material_velocity_mps=material_velocities[-2],
        right_material_velocity_mps=material_velocities[-1],
    )
    last_cell = material_cells[-1]
    outflow_fraction = (
        0.0 if last_cell.length_m <= _MIN_LENGTH else laydown_increment / last_cell.length_m
    )
    outgoing_cell = _MaterialCellIntegral(
        length_m=laydown_increment,
        momentum_per_linear_density_m2_s=plough_integral.momentum_kg_mps,
        kinetic_energy_per_linear_density_m3_s2=plough_integral.kinetic_energy_j,
        axial_strain_integral_m=last_cell.axial_strain_integral_m * outflow_fraction,
        axial_strain_squared_integral_m=(
            last_cell.axial_strain_squared_integral_m * outflow_fraction
        ),
    )
    transported = _transport_material_cell_integrals(
        material_cells,
        incoming_cell=incoming_cell,
        outgoing_length_m=laydown_increment,
        target_cell_lengths_m=target_rest_lengths_m,
        outgoing_cell_override=outgoing_cell,
    )
    plough_cut_cell = _endpoint_cut_cell_from_material_cell(transported.outgoing_cell)
    return _KnownPloughMaterialControlVolume(
        fairlead_cumulative_inflow_m=(
            control.fairlead_cumulative_inflow_m + payout_increment
        ),
        plough_cumulative_outflow_m=(
            control.plough_cumulative_outflow_m + laydown_increment
        ),
        fairlead_cut_cell=fairlead_cut_cell,
        plough_cut_cell=plough_cut_cell,
        material_cells=transported.cells,
        fairlead_cumulative_integral=_add_material_cells(
            control.fairlead_cumulative_integral,
            incoming_cell,
        ),
        plough_cumulative_integral=_add_material_cells(
            control.plough_cumulative_integral,
            transported.outgoing_cell,
        ),
    )




def _rezone_known_plough_uniform_ale(
    state: DynamicLayingState,
    *,
    case: StepConditions,
    new_active_length_m: float,
    element_count: int,
    payout_increment_m: float,
    laydown_increment_m: float,
    dt_s: float,
    fairlead_prescribed_acceleration: Vector3 = (0.0, 0.0, 0.0),
    plough_prescribed_acceleration: Vector3 = (0.0, 0.0, 0.0),
) -> DynamicLayingState:
    """将悬空的已知犁轨迹跨段保守重分区为均匀 ALE 单元。

    该操作在物理时间上是瞬时的。重分区后重新初始化 XPBD 乘子和反力场；
    端点支反力与生产时间步更新分开计算。几何投影只改变计算表示，
    活动参考长度仍等于调用方给定的边界通量平衡。
    """

    _validate_state(state)
    if element_count < 2 or element_count != len(state.rest_lengths_m):
        raise ValueError("uniform ALE requires the existing fixed segment count")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be finite and positive")
    increments = (float(payout_increment_m), float(laydown_increment_m))
    if any(not math.isfinite(value) or value < 0.0 for value in increments):
        raise ValueError("uniform ALE material increments must be finite and non-negative")
    if not math.isfinite(new_active_length_m) or new_active_length_m <= _MIN_LENGTH:
        raise ValueError("uniform ALE active length must be finite and positive")
    if any(state.contact_flags):
        raise RuntimeError("uniform ALE rezoning only supports a fully suspended span")
    control = state.known_plough_material_control_volume
    if control is None or len(control.material_cells) != element_count:
        raise RuntimeError("uniform ALE requires authoritative distributed material cells")
    for cell in control.material_cells:
        _validate_material_cell_moment_feasibility(cell)
    _validate_material_cells_match_p1_geometry(state)

    old_active_length = math.fsum(state.rest_lengths_m)
    expected_active_length = old_active_length + increments[0] - increments[1]
    length_tolerance = 64.0 * math.ulp(
        max(abs(old_active_length), abs(expected_active_length), abs(new_active_length_m))
    )
    if abs(new_active_length_m - expected_active_length) > length_tolerance:
        raise ValueError("uniform ALE active length does not close the boundary flux")
    uniform_length = new_active_length_m / element_count
    target_rest_lengths = tuple(uniform_length for _ in range(element_count))
    if (
        increments == (0.0, 0.0)
        and state.rest_lengths_m == target_rest_lengths
    ):
        return replace(
            state,
            material_remap_energy_error_per_linear_density_m3_s2=0.0,
            material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2=0.0,
        )

    fairlead_speed = increments[0] / dt_s
    plough_speed = increments[1] / dt_s
    source_coordinates, source_material_velocities = (
        _post_flux_material_p1_reference(
            state,
            payout_increment_m=increments[0],
            laydown_increment_m=increments[1],
            fairlead_speed_mps=fairlead_speed,
            plough_speed_mps=plough_speed,
        )
    )
    transported_control = _advance_known_plough_material_control_volume(
        state,
        payout_increment_m=increments[0],
        laydown_increment_m=increments[1],
        dt_s=dt_s,
        target_rest_lengths_m=target_rest_lengths,
    )
    for cell in transported_control.material_cells:
        _validate_material_cell_moment_feasibility(cell)

    old_coordinates = _cumulative_coordinates(list(state.rest_lengths_m))
    target_coordinates = _cumulative_coordinates(list(target_rest_lengths))
    l2_rhs = _p1_cross_mass_rhs(
        source_coordinates,
        source_material_velocities,
        tuple(target_coordinates),
    )
    old_seed_coordinates = [
        old_active_length * coordinate / new_active_length_m
        for coordinate in target_coordinates
    ]
    sampled_positions = list(
        _sample_monotone_material_vectors(
            list(state.positions),
            old_coordinates,
            old_seed_coordinates,
        )
    )
    sampled_positions[0] = state.positions[0]
    sampled_positions[-1] = state.positions[-1]
    target_deformed_lengths = [
        cell.length_m
        * (1.0 + _material_cell_mean_signed_geometric_axial_strain(cell))
        for cell in transported_control.material_cells
    ]
    endpoint_span = _norm(_sub(state.positions[-1], state.positions[0]))
    target_total = math.fsum(target_deformed_lengths)
    longest_target = max(target_deformed_lengths)
    minimum_span = max(0.0, 2.0 * longest_target - target_total)
    projection_tolerance = 64.0 * math.ulp(
        max(abs(endpoint_span), abs(target_total), 1.0)
    )
    if (
        endpoint_span > target_total + projection_tolerance
        or endpoint_span + projection_tolerance < minimum_span
    ):
        raise CableGeometryInfeasibleError(
            "uniform ALE target segment lengths are geometrically infeasible"
        )
    if endpoint_span <= _MIN_LENGTH:
        raise RuntimeError("uniform ALE requires distinct fixed endpoints")
    if abs(target_total - endpoint_span) <= projection_tolerance:
        chord_tangent = _mul(
            _sub(state.positions[-1], state.positions[0]),
            1.0 / endpoint_span,
        )
        coordinate = 0.0
        projected_positions = [state.positions[0]]
        for target_length in target_deformed_lengths[:-1]:
            coordinate += target_length
            projected_positions.append(
                _add(state.positions[0], _mul(chord_tangent, coordinate))
            )
        projected_positions.append(state.positions[-1])
    else:
        projected = _project_open_chain_segment_lengths(
            sampled_positions,
            target_deformed_lengths,
        )
        if projected is None:
            raise RuntimeError("uniform ALE open-chain position projection failed")
        projected_positions = projected
        projected_positions[0] = state.positions[0]
        projected_positions[-1] = state.positions[-1]
    maximum_length_error = max(
        abs(_norm(_sub(right, left)) - target)
        for left, right, target in zip(
            projected_positions,
            projected_positions[1:],
            target_deformed_lengths,
        )
    )
    if maximum_length_error > projection_tolerance:
        raise RuntimeError("uniform ALE projected segment lengths exceed roundoff tolerance")

    new_segments = segment_vectors(tuple(projected_positions))
    flow_speeds = _node_material_flow_speeds(
        target_rest_lengths,
        fairlead_speed_mps=fairlead_speed,
        plough_speed_mps=plough_speed,
    )
    endpoint_material_velocities = (
        _add(
            state.velocities[0],
            _mul(_node_tangent(new_segments, 0), flow_speeds[0]),
        ),
        _add(
            state.velocities[-1],
            _mul(_node_tangent(new_segments, element_count), flow_speeds[-1]),
        ),
    )
    cell_momentum = _sum_material_cells(
        transported_control.material_cells
    ).momentum_per_linear_density_m2_s
    material_velocities = _material_node_velocities_from_cells(
        transported_control.material_cells,
        target_rest_lengths,
        left_endpoint_velocity_mps=endpoint_material_velocities[0],
        right_endpoint_velocity_mps=endpoint_material_velocities[1],
        l2_rhs_per_linear_density_m2_s=l2_rhs,
        target_total_momentum_per_linear_density_m2_s=cell_momentum,
    )
    free_projection = _unconstrained_material_velocity_l2_projection(
        l2_rhs,
        target_rest_lengths,
    )
    resolved_momentum = _consistent_linear_material_momentum(
        list(material_velocities),
        list(target_rest_lengths),
    )
    momentum_scale = max(1.0, _norm(resolved_momentum), _norm(cell_momentum))
    momentum_tolerance = 64.0 * math.ulp(momentum_scale)
    if _norm(_sub(resolved_momentum, cell_momentum)) > momentum_tolerance:
        raise RuntimeError("uniform ALE resolved momentum does not close material cells")
    resolved_kinetic_energy = (
        _consistent_linear_structural_kinetic_energy_per_linear_density(
            list(material_velocities),
            list(target_rest_lengths),
        )
    )
    free_projection_energy = (
        _consistent_linear_structural_kinetic_energy_per_linear_density(
            list(free_projection),
            list(target_rest_lengths),
        )
    )
    cell_kinetic_energy = math.fsum(
        cell.kinetic_energy_per_linear_density_m3_s2
        for cell in transported_control.material_cells
    )
    source_kinetic_energy = (
        _consistent_linear_structural_kinetic_energy_per_linear_density(
            list(source_material_velocities),
            [
                right - left
                for left, right in zip(
                    source_coordinates,
                    source_coordinates[1:],
                )
            ],
        )
    )
    source_energy_tolerance = _kinetic_energy_roundoff_tolerance(
        cell_kinetic_energy,
        source_kinetic_energy,
        old_node_count=len(state.positions),
        new_node_count=len(source_material_velocities),
    )
    if abs(source_kinetic_energy - cell_kinetic_energy) > source_energy_tolerance:
        raise RuntimeError("uniform ALE source P1 field and transported K disagree")
    energy_tolerance = _kinetic_energy_roundoff_tolerance(
        cell_kinetic_energy,
        resolved_kinetic_energy,
        old_node_count=len(state.positions),
        new_node_count=len(projected_positions),
    )
    energy_tolerance = max(
        energy_tolerance,
        _kinetic_energy_roundoff_tolerance(
            cell_kinetic_energy,
            free_projection_energy,
            old_node_count=len(state.positions),
            new_node_count=len(projected_positions),
        ),
    )
    if free_projection_energy > cell_kinetic_energy + energy_tolerance:
        raise RuntimeError("uniform ALE remap has unexplained positive kinetic-energy error")
    remap_energy_error = resolved_kinetic_energy - cell_kinetic_energy
    constraint_projection_numerical_energy_increment = (
        resolved_kinetic_energy - free_projection_energy
    )
    rezoned_cells: list[_MaterialCellIntegral] = []
    for cell, left_velocity, right_velocity in zip(
        transported_control.material_cells,
        material_velocities,
        material_velocities[1:],
    ):
        resolved_cell_kinetic_energy = _linear_material_cell_kinetic_energy(
            left_velocity,
            right_velocity,
            cell.length_m,
        )
        resolved_cell_momentum = _mul(
            _add(left_velocity, right_velocity),
            0.5 * cell.length_m,
        )
        rezoned_cell = replace(
            cell,
            momentum_per_linear_density_m2_s=resolved_cell_momentum,
            kinetic_energy_per_linear_density_m3_s2=resolved_cell_kinetic_energy,
        )
        _validate_material_cell_moment_feasibility(rezoned_cell)
        rezoned_cells.append(rezoned_cell)
    transported_control = replace(
        transported_control,
        material_cells=tuple(rezoned_cells),
    )
    grid_velocities = tuple(
        _sub(
            material_velocity,
            _mul(_node_tangent(new_segments, index), flow_speeds[index]),
        )
        for index, material_velocity in enumerate(material_velocities)
    )
    grid_velocities = (
        state.velocities[0],
        *grid_velocities[1:-1],
        state.velocities[-1],
    )

    remapped_state = replace(
        state,
        positions=tuple(projected_positions),
        velocities=grid_velocities,
        rest_lengths_m=target_rest_lengths,
        paid_length_m=state.paid_length_m + increments[0],
        laid_length_m=state.laid_length_m + increments[1],
        contact_flags=tuple(False for _ in projected_positions),
        length_lambdas_n_s2=tuple(0.0 for _ in target_rest_lengths),
        contact_lambdas_n_s2=tuple(0.0 for _ in projected_positions),
        segment_tensions_n=(),
        length_constraint_reactions_n=(),
        contact_normal_reactions_n=tuple(0.0 for _ in projected_positions),
        material_suspended_length_m=new_active_length_m,
        known_plough_material_control_volume=transported_control,
        geometric_length_deficit_m=max(0.0, endpoint_span - new_active_length_m),
        material_remap_energy_error_per_linear_density_m3_s2=remap_energy_error,
        material_remap_energy_error_cumulative_per_linear_density_m3_s2=(
            state.material_remap_energy_error_cumulative_per_linear_density_m3_s2
            + remap_energy_error
        ),
        material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2=(
            constraint_projection_numerical_energy_increment
        ),
    )
    _validate_state(remapped_state)
    return remapped_state


def _advance_known_plough_material_flow(
    state: DynamicLayingState,
    *,
    payout_increment_m: float,
    laydown_increment_m: float,
    target_segment_length_m: float,
    dt_s: float,
    seabed_depth_m: float | None = None,
    defer_tail_remesh: bool = False,
) -> DynamicLayingState:
    """在一个通量区间内更新几何和守恒材料单元。

    几何响应流入与流出的净差，材料控制体则分别记录两端边界通量。
    因此，两端通量相等时可以输运材料，而不改变活动参考长度。
    """

    payout_increment = max(0.0, payout_increment_m)
    laydown_increment = max(0.0, laydown_increment_m)
    net_increment = payout_increment - laydown_increment
    geometry_seed = replace(state, known_plough_material_control_volume=None)
    if abs(net_increment) <= _MIN_LENGTH:
        geometry_state = geometry_seed
    elif net_increment > 0.0:
        geometry_state = _insert_payout_nodes(
            geometry_seed,
            payout_increment_m=net_increment,
            target_segment_length_m=target_segment_length_m,
            dt_s=dt_s,
        )
    else:
        withdrawal_increment = -net_increment
        minimum_segment_length = max(
            _MIN_LENGTH,
            withdrawal_increment / _MAX_NODE_CFL_FRACTION,
        )
        geometry_state = _withdraw_known_plough_tail_length(
            geometry_seed,
            laydown_increment_m=withdrawal_increment,
            minimum_segment_length_m=minimum_segment_length,
            dt_s=dt_s,
            seabed_depth_m=seabed_depth_m,
            defer_remesh=defer_tail_remesh,
        )
    control = _advance_known_plough_material_control_volume(
        state,
        payout_increment_m=payout_increment,
        laydown_increment_m=laydown_increment,
        dt_s=dt_s,
        target_rest_lengths_m=geometry_state.rest_lengths_m,
    )
    return replace(
        geometry_state,
        known_plough_material_control_volume=control,
    )












def _withdraw_known_plough_tail_length(
    state: DynamicLayingState,
    *,
    laydown_increment_m: float,
    minimum_segment_length_m: float,
    dt_s: float,
    seabed_depth_m: float | None = None,
    defer_remesh: bool = False,
) -> DynamicLayingState:
    """在不移动端点的条件下从犁端移除参考材料。

    仅当尾部切割完全消耗单元材料后，整个单元才会消失；保留的尾段需要重网格，
    避免末端分段长度趋近于零。
    """

    if laydown_increment_m <= _MIN_LENGTH or not state.rest_lengths_m:
        return state
    positions = list(state.positions)
    velocities = list(state.velocities)
    rest_lengths = list(state.rest_lengths_m)
    contact_flags = list(state.contact_flags)
    length_reactions = list(_state_physical_length_reactions(state))
    segment_tensions = list(_state_physical_segment_tensions(state, tuple(length_reactions)))
    contact_reactions = list(_state_physical_contact_reactions(state))
    length_lambdas = [reaction * dt_s * dt_s for reaction in length_reactions]
    contact_lambdas = [reaction * dt_s * dt_s for reaction in contact_reactions]
    remaining = laydown_increment_m
    min_length = max(_MIN_LENGTH, minimum_segment_length_m)
    while remaining > _MIN_LENGTH and rest_lengths:
        if defer_remesh:
            if remaining >= rest_lengths[-1] - _MIN_LENGTH:
                raise RuntimeError(
                    "known-plough deferred tail remesh cannot consume a complete segment in one step"
                )
            rest_lengths[-1] -= remaining
            remaining = 0.0
            break
        if len(rest_lengths) == 1:
            withdrawn = min(remaining, max(0.0, rest_lengths[-1] - _MIN_LENGTH))
            rest_lengths[-1] -= withdrawn
            remaining -= withdrawn
            break
        length_to_remesh = max(0.0, rest_lengths[-1] - min_length)
        if remaining + _MIN_LENGTH < length_to_remesh:
            rest_lengths[-1] -= remaining
            remaining = 0.0
            break
        rest_lengths[-1] -= length_to_remesh
        remaining = max(0.0, remaining - length_to_remesh)
        remeshed = _remesh_known_plough_tail_window(
            positions=positions,
            velocities=velocities,
            rest_lengths_m=rest_lengths,
            contact_flags=contact_flags,
            length_lambdas_n_s2=length_lambdas,
            segment_tensions_n=segment_tensions,
            length_constraint_reactions_n=length_reactions,
            contact_lambdas_n_s2=contact_lambdas,
            contact_normal_reactions_n=contact_reactions,
            dt_s=dt_s,
            seabed_depth_m=seabed_depth_m,
        )
        if remeshed is None:
            raise RuntimeError(
                "known-plough tail remesh projection failed; refusing to advance a sub-grid segment"
            )
        (
            positions,
            velocities,
            rest_lengths,
            contact_flags,
            length_lambdas,
            segment_tensions,
            length_reactions,
            contact_lambdas,
            contact_reactions,
        ) = remeshed
        if remaining <= _MIN_LENGTH:
            break
    return DynamicLayingState(
        time_s=state.time_s,
        positions=tuple(positions),
        velocities=tuple(velocities),
        rest_lengths_m=tuple(rest_lengths),
        paid_length_m=state.paid_length_m,
        laid_length_m=state.laid_length_m,
        contact_flags=tuple(contact_flags),
        length_lambdas_n_s2=tuple(length_lambdas[: len(rest_lengths)]),
        contact_lambdas_n_s2=tuple(contact_lambdas[: len(positions)]),
        segment_tensions_n=tuple(segment_tensions[: len(rest_lengths)]),
        length_constraint_reactions_n=tuple(length_reactions[: len(rest_lengths)]),
        contact_normal_reactions_n=tuple(contact_reactions[: len(positions)]),
        payout_buffer_m=state.payout_buffer_m,
        laydown_buffer_m=state.laydown_buffer_m,
        laid_segment_lengths_m=state.laid_segment_lengths_m,
        material_suspended_length_m=state.material_suspended_length_m,
        geometric_length_deficit_m=state.geometric_length_deficit_m,
        axial_solve_iterations=state.axial_solve_iterations,
        axial_constraint_residual_m=state.axial_constraint_residual_m,
    )


def _remesh_known_plough_tail_window(
    *,
    positions: list[Vector3],
    velocities: list[Vector3],
    rest_lengths_m: list[float],
    contact_flags: list[bool],
    length_lambdas_n_s2: list[float],
    segment_tensions_n: list[float],
    length_constraint_reactions_n: list[float],
    contact_lambdas_n_s2: list[float],
    contact_normal_reactions_n: list[float],
    dt_s: float,
    seabed_depth_m: float | None = None,
) -> tuple[
    list[Vector3],
    list[Vector3],
    list[float],
    list[bool],
    list[float],
    list[float],
    list[float],
    list[float],
    list[float],
] | None:
    """通过守恒材料状态投影移除一个尾部节点。"""

    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")
    # 优先选择最小守恒窗口；海床接触几何或动能门限可能需要增加一个内部自由度。
    for window_segment_count in range(3, min(4, len(rest_lengths_m)) + 1):
        remeshed = _remesh_known_plough_tail_window_with_count(
            positions=positions,
            velocities=velocities,
            rest_lengths_m=rest_lengths_m,
            contact_flags=contact_flags,
            length_lambdas_n_s2=length_lambdas_n_s2,
            segment_tensions_n=segment_tensions_n,
            length_constraint_reactions_n=length_constraint_reactions_n,
            contact_lambdas_n_s2=contact_lambdas_n_s2,
            contact_normal_reactions_n=contact_normal_reactions_n,
            dt_s=dt_s,
            window_segment_count=window_segment_count,
            seabed_depth_m=seabed_depth_m,
        )
        if remeshed is None:
            continue
        start_segment = len(rest_lengths_m) - window_segment_count
        try:
            old_energy = _lumped_structural_kinetic_energy_per_linear_density(
                velocities[start_segment:],
                rest_lengths_m[start_segment:],
            )
            new_energy = _lumped_structural_kinetic_energy_per_linear_density(
                remeshed[1][start_segment:],
                remeshed[2][start_segment:],
            )
        except OverflowError:
            continue
        if (
            not math.isfinite(old_energy)
            or old_energy < 0.0
            or not math.isfinite(new_energy)
            or new_energy < 0.0
        ):
            continue
        try:
            tolerance = _kinetic_energy_roundoff_tolerance(
                old_energy,
                new_energy,
                old_node_count=window_segment_count + 1,
                new_node_count=window_segment_count,
            )
        except ValueError:
            continue
        energy_increase = new_energy - old_energy
        if math.isfinite(energy_increase) and energy_increase <= tolerance:
            return remeshed
    return None


def _remesh_known_plough_tail_window_with_count(
    *,
    positions: list[Vector3],
    velocities: list[Vector3],
    rest_lengths_m: list[float],
    contact_flags: list[bool],
    length_lambdas_n_s2: list[float],
    segment_tensions_n: list[float],
    length_constraint_reactions_n: list[float],
    contact_lambdas_n_s2: list[float],
    contact_normal_reactions_n: list[float],
    dt_s: float,
    window_segment_count: int,
    seabed_depth_m: float | None,
) -> tuple[
    list[Vector3],
    list[Vector3],
    list[float],
    list[bool],
    list[float],
    list[float],
    list[float],
    list[float],
    list[float],
] | None:
    """在保持长度和积分场的条件下粗化尾部窗口。

    节点位置投影到重映射后的变形长度；P1 速度采用守恒传递；XPBD 乘子先转换为
    物理反力单位，再存入新网格。
    """

    if window_segment_count < 3 or window_segment_count > len(rest_lengths_m):
        return None
    start_segment = len(rest_lengths_m) - window_segment_count
    old_rest_lengths = rest_lengths_m[start_segment:]
    if any(length <= _MIN_LENGTH for length in old_rest_lengths):
        return None
    old_positions = positions[start_segment:]
    old_velocities = velocities[start_segment:]
    old_contact_reactions = contact_normal_reactions_n[start_segment:]
    old_coordinates = _cumulative_coordinates(old_rest_lengths)
    new_segment_count = window_segment_count - 1
    new_rest_length = old_coordinates[-1] / new_segment_count
    new_rest_lengths = [new_rest_length for _ in range(new_segment_count)]
    new_coordinates = [new_rest_length * index for index in range(new_segment_count + 1)]

    old_deformed_lengths = [
        _norm(_sub(right, left))
        for left, right in zip(old_positions, old_positions[1:])
    ]
    old_tensile_strains = [
        max(0.0, deformed_length / rest_length - 1.0)
        for deformed_length, rest_length in zip(old_deformed_lengths, old_rest_lengths)
    ]
    target_deformed_lengths = [
        (1.0 + _material_interval_root_mean_square(
            old_tensile_strains,
            old_coordinates,
            new_coordinates[index],
            new_coordinates[index + 1],
        )) * new_rest_length
        for index in range(new_segment_count)
    ]
    sampled_positions = [
        _sample_material_vector(old_positions, old_coordinates, coordinate)
        for coordinate in new_coordinates
    ]
    projected_positions = _project_open_chain_segment_lengths(
        sampled_positions,
        target_deformed_lengths,
    )
    if projected_positions is None:
        return None
    if seabed_depth_m is not None:
        seabed_projected_positions = _project_open_chain_segment_lengths_with_seabed(
            projected_positions,
            target_deformed_lengths,
            seabed_depth_m=seabed_depth_m,
        )
        if seabed_projected_positions is None:
            return None
        projected_positions = seabed_projected_positions

    projected_velocities = _conservative_material_velocity_transfer(
        old_velocities=old_velocities,
        old_rest_lengths=old_rest_lengths,
        new_rest_lengths=new_rest_lengths,
    )
    sampled_contact_reactions = [
        _sample_material_scalar(old_contact_reactions, old_coordinates, coordinate)
        for coordinate in new_coordinates
    ]
    sampled_contact_flags = [
        _sample_material_contact_flag(
            contact_flags[start_segment:],
            old_coordinates,
            coordinate,
        )
        for coordinate in new_coordinates
    ]

    def remap_segment_energy_field(values: list[float]) -> list[float]:
        old_values = values[start_segment:]
        return [
            _material_interval_root_mean_square(
                old_values,
                old_coordinates,
                new_coordinates[index],
                new_coordinates[index + 1],
            )
            for index in range(new_segment_count)
        ]

    remapped_tensions = remap_segment_energy_field(segment_tensions_n)
    remapped_reactions = remap_segment_energy_field(length_constraint_reactions_n)

    return (
        positions[:start_segment] + projected_positions,
        velocities[:start_segment] + projected_velocities,
        rest_lengths_m[:start_segment] + new_rest_lengths,
        contact_flags[:start_segment] + sampled_contact_flags,
        [
            reaction * dt_s * dt_s
            for reaction in length_constraint_reactions_n[:start_segment] + remapped_reactions
        ],
        segment_tensions_n[:start_segment] + remapped_tensions,
        length_constraint_reactions_n[:start_segment] + remapped_reactions,
        [
            reaction * dt_s * dt_s
            for reaction in contact_normal_reactions_n[:start_segment] + sampled_contact_reactions
        ],
        contact_normal_reactions_n[:start_segment] + sampled_contact_reactions,
    )


def _cumulative_coordinates(lengths: list[float]) -> list[float]:
    coordinates = [0.0]
    for length in lengths:
        coordinates.append(coordinates[-1] + length)
    return coordinates


def _material_interval_average(
    values: list[float],
    coordinates: list[float],
    start: float,
    end: float,
) -> float:
    interval = end - start
    if interval <= _MIN_LENGTH:
        return 0.0
    integral = 0.0
    for index, value in enumerate(values):
        overlap = max(
            0.0,
            min(end, coordinates[index + 1]) - max(start, coordinates[index]),
        )
        integral += value * overlap
    return integral / interval


def _material_interval_root_mean_square(
    values: list[float],
    coordinates: list[float],
    start: float,
    end: float,
) -> float:
    return math.sqrt(
        max(
            0.0,
            _material_interval_average(
                [value * value for value in values],
                coordinates,
                start,
                end,
            ),
        )
    )


def _material_sample_interval(coordinates: list[float], coordinate: float) -> tuple[int, float]:
    if coordinate <= coordinates[0]:
        return 0, 0.0
    if coordinate >= coordinates[-1]:
        return len(coordinates) - 2, 1.0
    for index in range(len(coordinates) - 1):
        if coordinate <= coordinates[index + 1]:
            span = coordinates[index + 1] - coordinates[index]
            fraction = (coordinate - coordinates[index]) / max(span, _MIN_LENGTH)
            return index, fraction
    return len(coordinates) - 2, 1.0


def _sample_material_vector(
    values: list[Vector3],
    coordinates: list[float],
    coordinate: float,
) -> Vector3:
    index, fraction = _material_sample_interval(coordinates, coordinate)
    return _add(values[index], _mul(_sub(values[index + 1], values[index]), fraction))


def _sample_monotone_material_vectors(
    values: list[Vector3],
    coordinates: list[float],
    target_coordinates: list[float],
) -> tuple[Vector3, ...]:
    """通过一次 O(N+M) 前向遍历采样已排序的材料坐标。"""

    if len(values) != len(coordinates) or len(values) < 2:
        raise ValueError("material vector samples must match at least two coordinates")
    if not all(math.isfinite(coordinate) for coordinate in coordinates):
        raise ValueError("material coordinates must be finite")
    if any(right <= left for left, right in zip(coordinates, coordinates[1:])):
        raise ValueError("material coordinates must be strictly increasing")
    if not all(math.isfinite(coordinate) for coordinate in target_coordinates):
        raise ValueError("target material coordinates must be finite")
    if any(
        right < left
        for left, right in zip(target_coordinates, target_coordinates[1:])
    ):
        raise ValueError("target material coordinates must be non-decreasing")

    sampled: list[Vector3] = []
    source_index = 0
    last_interval = len(coordinates) - 2
    for coordinate in target_coordinates:
        if coordinate <= coordinates[0]:
            interval = 0
            fraction = 0.0
        elif coordinate >= coordinates[-1]:
            interval = last_interval
            fraction = 1.0
        else:
            while (
                source_index < last_interval
                and coordinate > coordinates[source_index + 1]
            ):
                source_index += 1
            interval = source_index
            span = coordinates[interval + 1] - coordinates[interval]
            fraction = (coordinate - coordinates[interval]) / span
        sampled.append(
            _add(
                values[interval],
                _mul(
                    _sub(values[interval + 1], values[interval]),
                    fraction,
                ),
            )
        )
    return tuple(sampled)


def _sample_material_scalar(
    values: list[float],
    coordinates: list[float],
    coordinate: float,
) -> float:
    index, fraction = _material_sample_interval(coordinates, coordinate)
    return values[index] + fraction * (values[index + 1] - values[index])


def _sample_material_contact_flag(
    values: list[bool],
    coordinates: list[float],
    coordinate: float,
) -> bool:
    index, fraction = _material_sample_interval(coordinates, coordinate)
    if fraction <= _MIN_LENGTH:
        return values[index]
    if 1.0 - fraction <= _MIN_LENGTH:
        return values[index + 1]
    return values[index] and values[index + 1]


def _consistent_linear_material_momentum(
    velocities: list[Vector3],
    rest_lengths: list[float],
) -> Vector3:
    """按单位线密度精确积分分段线性材料速度。"""

    if len(velocities) != len(rest_lengths) + 1:
        raise ValueError("velocities must contain one value per material node")
    momentum = (0.0, 0.0, 0.0)
    for left, right, length in zip(velocities, velocities[1:], rest_lengths):
        momentum = _add(momentum, _mul(_add(left, right), 0.5 * length))
    return momentum


def _consistent_linear_structural_kinetic_energy_per_linear_density(
    velocities: list[Vector3],
    rest_lengths: list[float],
) -> float:
    """对分段线性速度场精确积分 0.5 |v|^2。"""

    if len(velocities) != len(rest_lengths) + 1:
        raise ValueError("velocities must contain one value per material node")
    return math.fsum(
        _linear_material_cell_kinetic_energy(left, right, length)
        for left, right, length in zip(velocities, velocities[1:], rest_lengths)
    )


def _linear_material_cell_kinetic_energy(
    left_velocity: Vector3,
    right_velocity: Vector3,
    length_m: float,
) -> float:
    """在一个线性材料速度单元上积分 0.5 |v|^2。"""

    velocity_sum = _add(left_velocity, right_velocity)
    velocity_difference = _sub(right_velocity, left_velocity)
    return (
        length_m
        / 24.0
        * math.fsum(
            (
                3.0 * _dot(velocity_sum, velocity_sum),
                _dot(velocity_difference, velocity_difference),
            )
        )
    )








def _conservative_material_velocity_transfer(
    *,
    old_velocities: list[Vector3],
    old_rest_lengths: list[float],
    new_rest_lengths: list[float],
) -> list[Vector3]:
    """返回满足动量守恒的质量加权最近速度场。

    材料插值结果作为参考场。端点速度固定时，在集中结构动量守恒约束下最小化
    ``sum(m_i * |v_i-u_i|^2)``，会在每个自由节点得到相同的矢量修正：
    ``v_i = u_i + (P_old-P_interp)/sum(m_free)``。
    """

    old_coordinates = _cumulative_coordinates(old_rest_lengths)
    new_coordinates = _cumulative_coordinates(new_rest_lengths)
    if len(old_velocities) != len(old_coordinates):
        raise ValueError("old velocities must contain one value per material node")
    if not new_rest_lengths:
        raise ValueError("new rest lengths must not be empty")
    projected = [
        _sample_material_vector(old_velocities, old_coordinates, coordinate)
        for coordinate in new_coordinates
    ]
    old_momentum = _lumped_material_momentum(old_velocities, old_rest_lengths)
    new_momentum = _lumped_material_momentum(projected, new_rest_lengths)
    new_tributaries = _node_tributary_lengths(new_rest_lengths)
    free_mass = sum(new_tributaries[1:-1])
    momentum_error = _sub(old_momentum, new_momentum)
    if free_mass <= _MIN_MASS:
        if _norm(momentum_error) > 1.0e-12:
            raise RuntimeError(
                "conservative velocity transfer has no internal degree of freedom"
            )
        return projected
    correction = _mul(momentum_error, 1.0 / free_mass)
    for index in range(1, len(projected) - 1):
        projected[index] = _add(projected[index], correction)
    return projected


def _lumped_material_momentum(
    velocities: list[Vector3],
    rest_lengths: list[float],
) -> Vector3:
    tributaries = _node_tributary_lengths(rest_lengths)
    if len(velocities) != len(tributaries):
        raise ValueError("velocities must contain one value per material node")
    momentum = (0.0, 0.0, 0.0)
    for velocity, tributary in zip(velocities, tributaries):
        momentum = _add(momentum, _mul(velocity, tributary))
    return momentum


def _lumped_structural_kinetic_energy_per_linear_density(
    velocities: list[Vector3],
    rest_lengths: list[float],
) -> float:
    """返回集中结构动能除以恒定线密度的结果。"""

    tributaries = _node_tributary_lengths(rest_lengths)
    if len(velocities) != len(tributaries):
        raise ValueError("velocities must contain one value per material node")
    return 0.5 * math.fsum(
        tributary * _dot(velocity, velocity)
        for velocity, tributary in zip(velocities, tributaries)
    )


def _kinetic_energy_roundoff_tolerance(
    old_energy: float,
    new_energy: float,
    *,
    old_node_count: int,
    new_node_count: int,
) -> float:
    """估计两次三维集中动能计算及比较过程的舍入误差上界。"""

    if old_node_count <= 0 or new_node_count <= 0:
        raise ValueError("kinetic-energy node counts must be positive")
    if (
        not math.isfinite(old_energy)
        or old_energy < 0.0
        or not math.isfinite(new_energy)
        or new_energy < 0.0
    ):
        raise ValueError("kinetic energies must be finite and non-negative")
    energy_scale = max(abs(old_energy), abs(new_energy))
    if energy_scale == 0.0:
        return 0.0
    # 每个三维节点项最多包含三次乘法、两次加法和一次质量乘法；
    # 误差界同时计入累加及最终缩放/比较。
    operation_bound = 8 * (old_node_count + new_node_count) + 4
    tolerance = operation_bound * math.ulp(energy_scale)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("kinetic-energy roundoff tolerance must be finite and non-negative")
    return tolerance


def _node_tributary_lengths(rest_lengths: list[float]) -> list[float]:
    tributaries = [0.0 for _ in range(len(rest_lengths) + 1)]
    for index, rest_length in enumerate(rest_lengths):
        tributaries[index] += 0.5 * rest_length
        tributaries[index + 1] += 0.5 * rest_length
    return tributaries


# 守恒重网格采用的几何投影。
def _project_open_chain_segment_lengths(
    positions: list[Vector3],
    target_lengths: list[float],
) -> list[Vector3] | None:
    """在保持两端点的条件下，将开放链投影到目标长度。

    投影前先检查可行性。直接构造尽量保留初值的横向形状；当舍入误差或几何条件
    阻碍局部精确闭合时，回退到迭代求解器。
    """

    if len(positions) != len(target_lengths) + 1 or len(positions) < 3:
        return None
    total_length = math.fsum(target_lengths)
    endpoint_distance = _norm(_sub(positions[-1], positions[0]))
    longest = max(target_lengths, default=0.0)
    minimum_endpoint_distance = max(0.0, 2.0 * longest - total_length)
    tolerance = 1.0e-11 * max(1.0, total_length)
    if endpoint_distance > total_length + tolerance or endpoint_distance + tolerance < minimum_endpoint_distance:
        return None
    if endpoint_distance <= _MIN_LENGTH:
        return None

    start = positions[0]
    chord_direction = _mul(_sub(positions[-1], start), 1.0 / endpoint_distance)
    transverse_offsets = []
    for position in positions:
        relative = _sub(position, start)
        axial_coordinate = _dot(relative, chord_direction)
        transverse_offsets.append(_sub(relative, _mul(chord_direction, axial_coordinate)))
    transverse_offsets[0] = (0.0, 0.0, 0.0)
    transverse_offsets[-1] = (0.0, 0.0, 0.0)
    transverse_steps = [
        _sub(right, left)
        for left, right in zip(transverse_offsets, transverse_offsets[1:])
    ]
    transverse_step_lengths = [_norm(step) for step in transverse_steps]

    if max(transverse_step_lengths, default=0.0) <= _MIN_LENGTH and total_length > endpoint_distance + tolerance:
        normal = _stable_transverse_direction(chord_direction)
        transverse_offsets = [
            _mul(normal, math.sin(math.pi * index / len(target_lengths)))
            for index in range(len(positions))
        ]
        transverse_offsets[0] = (0.0, 0.0, 0.0)
        transverse_offsets[-1] = (0.0, 0.0, 0.0)
        transverse_steps = [
            _sub(right, left)
            for left, right in zip(transverse_offsets, transverse_offsets[1:])
        ]
        transverse_step_lengths = [_norm(step) for step in transverse_steps]

    def axial_reach(scale: float) -> float:
        return math.fsum(
            math.sqrt(max(0.0, target * target - (scale * transverse) ** 2))
            for target, transverse in zip(target_lengths, transverse_step_lengths)
        )

    positive_limits = [
        target / transverse
        for target, transverse in zip(target_lengths, transverse_step_lengths)
        if transverse > _MIN_LENGTH
    ]
    if not positive_limits:
        if abs(total_length - endpoint_distance) > tolerance:
            return None
        transverse_scale = 0.0
    else:
        lower = 0.0
        upper = min(positive_limits) * (1.0 - 1.0e-12)
        if axial_reach(upper) > endpoint_distance + tolerance:
            return _project_open_chain_segment_lengths_iterative(
                positions,
                target_lengths,
            )
        for _ in range(160):
            middle = 0.5 * (lower + upper)
            if axial_reach(middle) > endpoint_distance:
                lower = middle
            else:
                upper = middle
        transverse_scale = upper

    axial_increments = [
        math.sqrt(max(0.0, target * target - (transverse_scale * transverse) ** 2))
        for target, transverse in zip(target_lengths, transverse_step_lengths)
    ]
    axial_scale = endpoint_distance / max(math.fsum(axial_increments), _MIN_LENGTH)
    projected = [start]
    axial_coordinate = 0.0
    for index in range(1, len(positions) - 1):
        axial_coordinate += axial_increments[index - 1] * axial_scale
        projected.append(
            _add(
                _add(start, _mul(chord_direction, axial_coordinate)),
                _mul(transverse_offsets[index], transverse_scale),
            )
        )
    projected.append(positions[-1])
    errors = [
        (
            abs(_norm(_sub(right, left)) - target),
            64.0 * math.ulp(max(abs(target), 1.0)),
        )
        for left, right, target in zip(projected, projected[1:], target_lengths)
    ]
    if any(error > local_tolerance for error, local_tolerance in errors):
        return _project_open_chain_segment_lengths_iterative(
            positions,
            target_lengths,
        )
    return projected


def _project_open_chain_segment_lengths_iterative(
    positions: list[Vector3],
    target_lengths: list[float],
) -> list[Vector3] | None:
    """采用 O(N) 最小范数 Newton 修正投影一条具有可行初值的链。"""

    if len(positions) != len(target_lengths) + 1 or len(positions) < 3:
        return None
    origin = positions[0]
    projected = [_sub(position, origin) for position in positions]
    fixed_start = projected[0]
    fixed_end = projected[-1]
    local_tolerances = [
        64.0 * math.ulp(max(abs(target), 1.0))
        for target in target_lengths
    ]
    segment_count = len(target_lengths)
    for _ in range(64):
        directions: list[Vector3] = []
        residuals: list[float] = []
        for index, (left, right, target) in enumerate(
            zip(projected, projected[1:], target_lengths)
        ):
            delta = _sub(right, left)
            length = _norm(delta)
            directions.append(
                _mul(delta, 1.0 / length)
                if length > _MIN_LENGTH
                else _spacing_floor_direction(projected, index, delta)
            )
            residuals.append(length - target)
        if all(
            abs(residual) <= local_tolerance
            for residual, local_tolerance in zip(residuals, local_tolerances)
        ):
            translated = [_add(origin, position) for position in projected]
            translated[0] = positions[0]
            translated[-1] = positions[-1]
            if all(
                abs(_norm(_sub(right, left)) - target)
                <= _segment_length_backward_error_tolerance(left, right, target)
                for left, right, target in zip(
                    translated,
                    translated[1:],
                    target_lengths,
                )
            ):
                return translated
            return None
        normal_diagonal: list[float] = []
        normal_off_diagonal: list[float] = []
        for index, direction in enumerate(directions):
            normal_diagonal.append(
                1.0 if index in (0, segment_count - 1) else 2.0
            )
            if index + 1 < segment_count:
                normal_off_diagonal.append(
                    -_dot(direction, directions[index + 1])
                )
        multipliers = _solve_symmetric_positive_tridiagonal(
            normal_diagonal,
            normal_off_diagonal,
            residuals,
        )
        if multipliers is None:
            return None
        interior_corrections = [
            _add(
                _mul(directions[index], -multipliers[index]),
                _mul(directions[index + 1], multipliers[index + 1]),
            )
            for index in range(segment_count - 1)
        ]
        projected = [
            fixed_start,
            *(
                _add(position, correction)
                for position, correction in zip(
                    projected[1:-1],
                    interior_corrections,
                )
            ),
            fixed_end,
        ]
        if not all(
            math.isfinite(component)
            for position in projected
            for component in position
        ):
            return None
    return None


def _segment_length_backward_error_tolerance(
    left: Vector3,
    right: Vector3,
    target_length: float,
) -> float:
    """估计局部求解误差与全局坐标表示误差之和的上界。"""

    local_solver_tolerance = 64.0 * math.ulp(max(abs(target_length), 1.0))
    coordinate_representation_error = math.hypot(
        *(
            0.5 * math.ulp(left[axis]) + 0.5 * math.ulp(right[axis])
            for axis in range(3)
        )
    )
    return local_solver_tolerance + coordinate_representation_error


def _solve_symmetric_positive_tridiagonal(
    diagonal: list[float],
    off_diagonal: list[float],
    right_hand_side: list[float],
) -> list[float] | None:
    """采用 LDL^T 求解有限 SPD 三对角方程组，不回退到稠密求解。"""

    if (
        not diagonal
        or len(off_diagonal) != len(diagonal) - 1
        or len(right_hand_side) != len(diagonal)
    ):
        raise ValueError("invalid symmetric tridiagonal system dimensions")
    if not all(
        math.isfinite(value)
        for value in (*diagonal, *off_diagonal, *right_hand_side)
    ):
        return None
    matrix_scale = max(
        max(abs(value) for value in diagonal),
        max((abs(value) for value in off_diagonal), default=0.0),
    )
    pivot_tolerance = 64.0 * math.ulp(matrix_scale)
    pivots = [diagonal[0]]
    lower_factors: list[float] = []
    if pivots[0] <= pivot_tolerance:
        return None
    for index, coupling in enumerate(off_diagonal, start=1):
        lower_factor = coupling / pivots[-1]
        pivot = diagonal[index] - lower_factor * coupling
        if (
            not math.isfinite(lower_factor)
            or not math.isfinite(pivot)
            or pivot <= pivot_tolerance
        ):
            return None
        lower_factors.append(lower_factor)
        pivots.append(pivot)
    forward = [right_hand_side[0]]
    for index in range(1, len(pivots)):
        forward.append(
            right_hand_side[index]
            - lower_factors[index - 1] * forward[index - 1]
        )
    diagonal_solution = [
        value / pivot for value, pivot in zip(forward, pivots)
    ]
    solution = [0.0 for _ in pivots]
    solution[-1] = diagonal_solution[-1]
    for index in range(len(solution) - 2, -1, -1):
        solution[index] = (
            diagonal_solution[index]
            - lower_factors[index] * solution[index + 1]
        )
    return solution if all(math.isfinite(value) for value in solution) else None


def _project_open_chain_segment_lengths_with_seabed(
    positions: list[Vector3],
    target_lengths: list[float],
    *,
    seabed_depth_m: float,
) -> list[Vector3] | None:
    """强制满足重网格后的分段长度，同时禁止穿透海床。"""

    if len(positions) != len(target_lengths) + 1 or len(positions) < 3:
        return None
    if positions[0][2] > seabed_depth_m or positions[-1][2] > seabed_depth_m:
        return None
    tolerance = _REMESH_PROJECTION_REL_TOLERANCE * max(1.0, sum(target_lengths))
    seabed_chain = _project_tail_chain_onto_seabed(
        positions,
        target_lengths,
        seabed_depth_m=seabed_depth_m,
    )
    if seabed_chain is not None:
        return seabed_chain
    endpoint_delta = _sub(positions[-1], positions[0])
    endpoint_distance = _norm(endpoint_delta)
    if endpoint_distance > _MIN_LENGTH:
        chord_direction = _mul(endpoint_delta, 1.0 / endpoint_distance)
        reflected: list[Vector3] = []
        for position in positions:
            relative = _sub(position, positions[0])
            chord_point = _add(
                positions[0],
                _mul(chord_direction, _dot(relative, chord_direction)),
            )
            reflected.append(_sub(_mul(chord_point, 2.0), position))
        reflection_error = max(
            abs(math.dist(left, right) - target)
            for left, right, target in zip(reflected, reflected[1:], target_lengths)
        )
        if (
            reflection_error <= tolerance
            and max(position[2] for position in reflected) <= seabed_depth_m
        ):
            return reflected
    solved = [list(position) for position in positions]
    fixed_start = tuple(positions[0])
    fixed_end = tuple(positions[-1])
    for position in solved[1:-1]:
        position[2] = min(position[2], seabed_depth_m)
    for _ in range(_REMESH_PROJECTION_MAX_ITERATIONS):
        for segment_indices in (
            range(len(target_lengths)),
            range(len(target_lengths) - 1, -1, -1),
        ):
            for index in segment_indices:
                left = solved[index]
                right = solved[index + 1]
                delta = [right[axis] - left[axis] for axis in range(3)]
                length = math.sqrt(sum(component * component for component in delta))
                if length <= _MIN_LENGTH:
                    continue
                error = length - target_lengths[index]
                left_weight = 0.0 if index == 0 else 1.0
                right_weight = 0.0 if index + 1 == len(solved) - 1 else 1.0
                weight_sum = left_weight + right_weight
                if weight_sum <= 0.0:
                    continue
                for axis in range(3):
                    correction = error * delta[axis] / length
                    left[axis] += left_weight * correction / weight_sum
                    right[axis] -= right_weight * correction / weight_sum
                if index > 0:
                    left[2] = min(left[2], seabed_depth_m)
                if index + 1 < len(solved) - 1:
                    right[2] = min(right[2], seabed_depth_m)
        solved[0] = list(fixed_start)
        solved[-1] = list(fixed_end)
        maximum_error = max(
            abs(math.dist(left, right) - target)
            for left, right, target in zip(solved, solved[1:], target_lengths)
        )
        if maximum_error <= tolerance:
            return [tuple(position) for position in solved]
    return None


def _project_tail_chain_onto_seabed(
    positions: list[Vector3],
    target_lengths: list[float],
    *,
    seabed_depth_m: float,
) -> list[Vector3] | None:
    """将满足约束的最长尾部区间置于海床上。"""

    if len(target_lengths) < 2:
        return None
    start = positions[0]
    end = positions[-1]
    if start[2] > seabed_depth_m + _SEABED_CONTACT_TOLERANCE_M:
        return None
    horizontal_delta = (end[0] - start[0], end[1] - start[1], 0.0)
    horizontal_distance = _norm(horizontal_delta)
    if horizontal_distance <= _MIN_LENGTH:
        route_direction = (1.0, 0.0, 0.0)
    else:
        route_direction = _mul(horizontal_delta, 1.0 / horizontal_distance)
    tolerance = _REMESH_PROJECTION_REL_TOLERANCE * max(1.0, sum(target_lengths))
    segment_count = len(target_lengths)
    for contact_segment_count in range(segment_count - 1, 0, -1):
        prefix_segment_count = segment_count - contact_segment_count
        contact_lengths = target_lengths[prefix_segment_count:]
        contact_length = sum(contact_lengths)
        first_contact = (
            end[0] - route_direction[0] * contact_length,
            end[1] - route_direction[1] * contact_length,
            seabed_depth_m,
        )
        prefix_lengths = target_lengths[:prefix_segment_count]
        prefix_distance = _norm(_sub(first_contact, start))
        prefix_total = sum(prefix_lengths)
        prefix_longest = max(prefix_lengths, default=0.0)
        prefix_minimum_distance = max(0.0, 2.0 * prefix_longest - prefix_total)
        if (
            prefix_distance > prefix_total + tolerance
            or prefix_distance + tolerance < prefix_minimum_distance
        ):
            continue
        if prefix_segment_count == 1:
            prefix = [start, first_contact]
        else:
            prefix_seed = positions[:prefix_segment_count] + [first_contact]
            prefix = _project_open_chain_segment_lengths(prefix_seed, prefix_lengths)
            if prefix is None:
                continue
        if max(position[2] for position in prefix) > seabed_depth_m:
            prefix_delta = _sub(prefix[-1], prefix[0])
            prefix_distance = _norm(prefix_delta)
            if prefix_distance <= _MIN_LENGTH:
                continue
            prefix_direction = _mul(prefix_delta, 1.0 / prefix_distance)
            reflected: list[Vector3] = []
            for position in prefix:
                relative = _sub(position, prefix[0])
                chord_point = _add(
                    prefix[0],
                    _mul(prefix_direction, _dot(relative, prefix_direction)),
                )
                reflected.append(_sub(_mul(chord_point, 2.0), position))
            prefix = reflected
        if max(position[2] for position in prefix) > seabed_depth_m + tolerance:
            continue
        contact_nodes = [first_contact]
        cursor = first_contact
        for length in contact_lengths:
            cursor = (
                cursor[0] + route_direction[0] * length,
                cursor[1] + route_direction[1] * length,
                seabed_depth_m,
            )
            contact_nodes.append(cursor)
        contact_nodes[-1] = end
        projected = prefix + contact_nodes[1:]
        maximum_error = max(
            abs(math.dist(left, right) - target)
            for left, right, target in zip(projected, projected[1:], target_lengths)
        )
        if maximum_error <= tolerance:
            return projected
    return None


def _stable_transverse_direction(direction: Vector3) -> Vector3:
    candidate = (0.0, 0.0, 1.0)
    if abs(_dot(direction, candidate)) > 0.9:
        candidate = (0.0, 1.0, 0.0)
    transverse = _sub(candidate, _mul(direction, _dot(direction, candidate)))
    return _safe_unit(transverse)


def _limit_endpoint_span_velocities(
    velocities: tuple[Vector3, ...],
    rest_lengths_m: tuple[float, ...],
    dt_s: float,
) -> tuple[Vector3, ...]:
    limited = list(velocities)
    for index in range(1, len(limited) - 1):
        speed = _norm(limited[index])
        if speed <= _MIN_LENGTH or not math.isfinite(speed):
            limited[index] = (0.0, 0.0, 0.0)
            continue
        local_length = _local_segment_length(index, rest_lengths_m)
        max_speed = _MAX_NODE_CFL_FRACTION * local_length / max(dt_s, _MIN_LENGTH)
        if speed > max_speed:
            limited[index] = _mul(limited[index], max_speed / speed)
    return tuple(limited)


def _vessel_position(dynamic_case, time_s: float) -> Vector3:
    if getattr(dynamic_case, "vessel_motion_samples", ()):
        return _sampled_motion_position(dynamic_case.vessel_motion_samples, time_s, default_z=0.0)
    if getattr(dynamic_case, "vessel_motion_segments", ()):
        offset = _motion_displacement(dynamic_case.vessel_motion_segments, time_s)
        return (
            dynamic_case.vessel_initial_x_m + offset[0],
            dynamic_case.vessel_initial_y_m + offset[1],
            0.0,
        )
    direction = _heading_unit(dynamic_case.vessel_heading_deg)
    distance = _vessel_distance(dynamic_case, time_s)
    return (
        dynamic_case.vessel_initial_x_m + direction[0] * distance,
        dynamic_case.vessel_initial_y_m + direction[1] * distance,
        0.0,
    )


def _plough_position(dynamic_case, time_s: float) -> Vector3:
    """按采样、显式坐标、偏移量的优先级确定犁端位置。"""

    if getattr(dynamic_case, "plough_motion_samples", ()):
        default_z = dynamic_case.plough_initial_z_m if dynamic_case.plough_initial_z_m is not None else 0.0
        return _sampled_motion_position(dynamic_case.plough_motion_samples, time_s, default_z=default_z)
    if dynamic_case.plough_initial_x_m is None or dynamic_case.plough_initial_y_m is None or dynamic_case.plough_initial_z_m is None:
        raise ValueError("plough initial position is required")
    if getattr(dynamic_case, "plough_motion_segments", ()):
        offset = _motion_displacement(dynamic_case.plough_motion_segments, time_s)
        return (
            dynamic_case.plough_initial_x_m + offset[0],
            dynamic_case.plough_initial_y_m + offset[1],
            dynamic_case.plough_initial_z_m,
        )
    direction = _heading_unit(dynamic_case.plough_heading_deg or 0.0)
    distance = (dynamic_case.plough_speed_mps or 0.0) * time_s
    return (
        dynamic_case.plough_initial_x_m + direction[0] * distance,
        dynamic_case.plough_initial_y_m + direction[1] * distance,
        dynamic_case.plough_initial_z_m,
    )


def _vessel_velocity(dynamic_case, time_s: float) -> Vector3:
    if getattr(dynamic_case, "vessel_motion_samples", ()):
        return _sampled_motion_velocity(dynamic_case.vessel_motion_samples, time_s, default_z=0.0)
    if getattr(dynamic_case, "vessel_motion_segments", ()):
        return _motion_velocity(dynamic_case.vessel_motion_segments, time_s)
    direction = _heading_unit(dynamic_case.vessel_heading_deg)
    speed = _vessel_speed(dynamic_case, time_s)
    return (direction[0] * speed, direction[1] * speed, 0.0)


def _plough_velocity(dynamic_case, time_s: float) -> Vector3:
    if getattr(dynamic_case, "plough_motion_samples", ()):
        default_z = dynamic_case.plough_initial_z_m if dynamic_case.plough_initial_z_m is not None else 0.0
        return _sampled_motion_velocity(dynamic_case.plough_motion_samples, time_s, default_z=default_z)
    if getattr(dynamic_case, "plough_motion_segments", ()):
        return _motion_velocity(dynamic_case.plough_motion_segments, time_s)
    direction = _heading_unit(dynamic_case.plough_heading_deg or 0.0)
    speed = dynamic_case.plough_speed_mps or 0.0
    return (direction[0] * speed, direction[1] * speed, 0.0)


def _plough_acceleration(dynamic_case, time_s: float) -> Vector3:
    if getattr(dynamic_case, "plough_motion_samples", ()):
        default_z = dynamic_case.plough_initial_z_m if dynamic_case.plough_initial_z_m is not None else 0.0
        return _sampled_motion_acceleration(dynamic_case.plough_motion_samples, time_s, default_z=default_z)
    if getattr(dynamic_case, "plough_motion_segments", ()):
        return _motion_acceleration(dynamic_case.plough_motion_segments, time_s)
    return (0.0, 0.0, 0.0)


def _vessel_acceleration_vector(dynamic_case, time_s: float) -> Vector3:
    if getattr(dynamic_case, "vessel_motion_samples", ()):
        return _sampled_motion_acceleration(dynamic_case.vessel_motion_samples, time_s, default_z=0.0)
    if getattr(dynamic_case, "vessel_motion_segments", ()):
        return _motion_acceleration(dynamic_case.vessel_motion_segments, time_s)
    direction = _heading_unit(dynamic_case.vessel_heading_deg)
    return _mul(direction, _vessel_acceleration(dynamic_case, time_s))


def _sampled_motion_position(samples, time_s: float, *, default_z: float) -> Vector3:
    """插值实测位置，并将超出采样时域的请求截断到端点。"""

    if not samples:
        return (0.0, 0.0, default_z)
    if len(samples) == 1 or time_s <= samples[0].time_s:
        return _motion_sample_position(samples[0], default_z=default_z)
    for start, end in zip(samples, samples[1:]):
        if time_s <= end.time_s:
            duration = max(end.time_s - start.time_s, _MIN_LENGTH)
            fraction = max(0.0, min(1.0, (time_s - start.time_s) / duration))
            start_pos = _motion_sample_position(start, default_z=default_z)
            end_pos = _motion_sample_position(end, default_z=default_z)
            return (
                start_pos[0] + (end_pos[0] - start_pos[0]) * fraction,
                start_pos[1] + (end_pos[1] - start_pos[1]) * fraction,
                start_pos[2] + (end_pos[2] - start_pos[2]) * fraction,
            )
    last = samples[-1]
    last_pos = _motion_sample_position(last, default_z=default_z)
    last_velocity = _sampled_motion_velocity(samples, last.time_s, default_z=default_z)
    dt = max(0.0, time_s - last.time_s)
    return (
        last_pos[0] + last_velocity[0] * dt,
        last_pos[1] + last_velocity[1] * dt,
        last_pos[2] + last_velocity[2] * dt,
    )


def _sampled_motion_velocity(samples, time_s: float, *, default_z: float) -> Vector3:
    """插值实测速度；缺少速度时对相邻位置采样求差。"""

    if not samples:
        return (0.0, 0.0, 0.0)
    if len(samples) == 1:
        return _motion_sample_velocity_or_default(samples[0])
    if time_s <= samples[0].time_s:
        if _motion_sample_has_velocity(samples[0]):
            return _motion_sample_velocity_or_default(samples[0])
        return _sample_velocity_between(samples[0], samples[1], default_z=default_z)
    for start, end in zip(samples, samples[1:]):
        if time_s <= end.time_s:
            if _motion_sample_has_velocity(start) and _motion_sample_has_velocity(end):
                duration = max(end.time_s - start.time_s, _MIN_LENGTH)
                fraction = max(0.0, min(1.0, (time_s - start.time_s) / duration))
                start_velocity = _motion_sample_velocity_or_default(start)
                end_velocity = _motion_sample_velocity_or_default(end)
                return (
                    start_velocity[0] + (end_velocity[0] - start_velocity[0]) * fraction,
                    start_velocity[1] + (end_velocity[1] - start_velocity[1]) * fraction,
                    start_velocity[2] + (end_velocity[2] - start_velocity[2]) * fraction,
                )
            return _sample_velocity_between(start, end, default_z=default_z)
    last = samples[-1]
    if _motion_sample_has_velocity(last):
        return _motion_sample_velocity_or_default(last)
    return _sample_velocity_between(samples[-2], last, default_z=default_z)


def _sampled_motion_acceleration(samples, time_s: float, *, default_z: float) -> Vector3:
    if len(samples) < 2 or time_s > samples[-1].time_s:
        return (0.0, 0.0, 0.0)
    sample_pairs = zip(samples, samples[1:])
    for start, end in sample_pairs:
        if time_s > end.time_s:
            continue
        if not (_motion_sample_has_velocity(start) and _motion_sample_has_velocity(end)):
            return (0.0, 0.0, 0.0)
        duration = max(end.time_s - start.time_s, _MIN_LENGTH)
        start_velocity = _motion_sample_velocity_or_default(start)
        end_velocity = _motion_sample_velocity_or_default(end)
        return _mul(_sub(end_velocity, start_velocity), 1.0 / duration)
    return (0.0, 0.0, 0.0)


def _sample_velocity_between(start, end, *, default_z: float) -> Vector3:
    duration = max(end.time_s - start.time_s, _MIN_LENGTH)
    start_pos = _motion_sample_position(start, default_z=default_z)
    end_pos = _motion_sample_position(end, default_z=default_z)
    return (
        (end_pos[0] - start_pos[0]) / duration,
        (end_pos[1] - start_pos[1]) / duration,
        (end_pos[2] - start_pos[2]) / duration,
    )


def _motion_sample_position(sample, *, default_z: float) -> Vector3:
    return (
        float(sample.x_m),
        float(sample.y_m),
        float(default_z if sample.z_m is None else sample.z_m),
    )


def _motion_sample_has_velocity(sample) -> bool:
    return sample.velocity_x_mps is not None and sample.velocity_y_mps is not None


def _motion_sample_velocity_or_default(sample) -> Vector3:
    if not _motion_sample_has_velocity(sample):
        return (0.0, 0.0, 0.0)
    return (
        float(sample.velocity_x_mps),
        float(sample.velocity_y_mps),
        float(0.0 if sample.velocity_z_mps is None else sample.velocity_z_mps),
    )


def _motion_displacement(segments, time_s: float) -> Vector3:
    """积分有序运动分段，并在分段结束后保持末速度继续外推位移。"""

    remaining = max(0.0, time_s)
    x = 0.0
    y = 0.0
    last_segment = None
    for segment in segments:
        last_segment = segment
        duration = max(segment.duration_s, _MIN_LENGTH)
        elapsed = min(remaining, duration)
        if elapsed > 0.0:
            start_velocity, end_velocity = _segment_velocity_endpoints(segment)
            fraction = elapsed / duration
            integrated_fraction = segment_interpolation_integral(
                segment.interpolation,
                fraction,
                duration_s=duration,
                sample_interval_s=segment.sample_interval_s,
            )
            x += start_velocity[0] * elapsed
            x += (end_velocity[0] - start_velocity[0]) * duration * integrated_fraction
            y += start_velocity[1] * elapsed
            y += (end_velocity[1] - start_velocity[1]) * duration * integrated_fraction
        remaining -= elapsed
        if remaining <= _MIN_LENGTH:
            break
    if remaining > _MIN_LENGTH and last_segment is not None:
        _, end_velocity = _segment_velocity_endpoints(last_segment)
        x += end_velocity[0] * remaining
        y += end_velocity[1] * remaining
    return (x, y, 0.0)


def _motion_velocity(segments, time_s: float) -> Vector3:
    """计算有序运动分段，并在分段结束后保持末速度。"""

    remaining = max(0.0, time_s)
    last_segment = None
    for segment in segments:
        last_segment = segment
        duration = max(segment.duration_s, _MIN_LENGTH)
        if remaining <= duration:
            fraction = segment_interpolation_fraction(
                segment.interpolation,
                remaining / duration,
                duration_s=duration,
                sample_interval_s=segment.sample_interval_s,
            )
            start_velocity, end_velocity = _segment_velocity_endpoints(segment)
            return (
                start_velocity[0] + (end_velocity[0] - start_velocity[0]) * fraction,
                start_velocity[1] + (end_velocity[1] - start_velocity[1]) * fraction,
                0.0,
            )
        remaining -= duration
    if last_segment is None:
        return (0.0, 0.0, 0.0)
    _, end_velocity = _segment_velocity_endpoints(last_segment)
    return (end_velocity[0], end_velocity[1], 0.0)


def _motion_acceleration(segments, time_s: float) -> Vector3:
    remaining = max(0.0, time_s)
    for segment in segments:
        duration = max(segment.duration_s, _MIN_LENGTH)
        if remaining <= duration:
            start_velocity, end_velocity = _segment_velocity_endpoints(segment)
            derivative = segment_interpolation_derivative(
                segment.interpolation,
                remaining / duration,
                duration_s=duration,
                sample_interval_s=segment.sample_interval_s,
            )
            return _mul(_sub(end_velocity, start_velocity), derivative / duration)
        remaining -= duration
    return (0.0, 0.0, 0.0)


def _segment_velocity_endpoints(segment) -> tuple[Vector3, Vector3]:
    vector_fields = (
        getattr(segment, "start_velocity_x_mps", None),
        getattr(segment, "start_velocity_y_mps", None),
        getattr(segment, "end_velocity_x_mps", None),
        getattr(segment, "end_velocity_y_mps", None),
    )
    if all(value is not None for value in vector_fields):
        start_x, start_y, end_x, end_y = vector_fields
        return (
            (float(start_x), float(start_y), 0.0),
            (float(end_x), float(end_y), 0.0),
        )
    direction = _heading_unit(segment.heading_deg)
    return (
        (direction[0] * segment.start_speed_mps, direction[1] * segment.start_speed_mps, 0.0),
        (direction[0] * segment.end_speed_mps, direction[1] * segment.end_speed_mps, 0.0),
    )


def _heading_unit(degrees: float) -> Vector3:
    radians = math.radians(degrees)
    return (math.cos(radians), math.sin(radians), 0.0)


def _vessel_distance(dynamic_case, time_s: float) -> float:
    if time_s <= 0.0:
        return 0.0
    if time_s <= dynamic_case.transition_duration_s:
        acceleration = (dynamic_case.vessel_final_speed_mps - dynamic_case.vessel_initial_speed_mps) / max(dynamic_case.transition_duration_s, _MIN_LENGTH)
        return dynamic_case.vessel_initial_speed_mps * time_s + 0.5 * acceleration * time_s * time_s
    ramp_distance = 0.5 * (dynamic_case.vessel_initial_speed_mps + dynamic_case.vessel_final_speed_mps) * dynamic_case.transition_duration_s
    return ramp_distance + dynamic_case.vessel_final_speed_mps * (time_s - dynamic_case.transition_duration_s)


def _plough_entry_angle_deg(positions: tuple[Vector3, ...]) -> float:
    if len(positions) < 2:
        return 0.0
    tangent = _safe_unit(_sub(positions[-1], positions[-2]))
    horizontal = math.hypot(tangent[0], tangent[1])
    return math.degrees(math.atan2(abs(tangent[2]), max(horizontal, _MIN_LENGTH)))




def _minimum_bend_radius_diagnostic(
    positions: tuple[Vector3, ...],
    *,
    exclude_tail_nodes: int = 0,
) -> _BendRadiusDiagnostic:
    """返回内部最小外接圆半径及其局部几何信息。

    排除尾部节点可避免将给定端点附近的几何误报为自由跨段弯曲半径违规。
    """

    if len(positions) < 3:
        return _BendRadiusDiagnostic(radius_m=math.inf)
    best = _BendRadiusDiagnostic(radius_m=math.inf)
    segment_lengths = [
        _norm(_sub(end, start))
        for start, end in zip(positions, positions[1:])
    ]
    positive_lengths = sorted(length for length in segment_lengths if length > _MIN_LENGTH)
    if not positive_lengths:
        return _BendRadiusDiagnostic(radius_m=1.0e12)
    reference_length = positive_lengths[len(positive_lengths) // 2]
    degenerate_cutoff = 0.10 * reference_length
    seabed_like_depth = max(position[2] for position in positions)
    seabed_cluster_tolerance = max(_SEABED_CONTACT_TOLERANCE_M, 0.005 * reference_length)
    last_included_node_index = len(positions) - 2 - max(0, exclude_tail_nodes)
    for node_index, (previous, current, next_point) in enumerate(zip(positions, positions[1:], positions[2:]), start=1):
        if node_index > last_included_node_index:
            continue
        first = _sub(current, previous)
        second = _sub(next_point, current)
        first_length = _norm(first)
        second_length = _norm(second)
        if first_length <= degenerate_cutoff or second_length <= degenerate_cutoff:
            continue
        if (
            min(previous[2], current[2], next_point[2]) >= seabed_like_depth - seabed_cluster_tolerance
            and min(first_length, second_length) <= 0.5 * reference_length
        ):
            continue
        dot = max(-1.0, min(1.0, _dot(first, second) / (first_length * second_length)))
        turn = math.acos(dot)
        if turn <= 1.0e-9:
            continue
        radius = 0.5 * (first_length + second_length) / turn
        if radius < best.radius_m:
            best = _BendRadiusDiagnostic(
                radius_m=radius,
                node_index=node_index,
                left_segment_m=first_length,
                right_segment_m=second_length,
                turn_angle_deg=math.degrees(turn),
                node_depth_m=current[2],
                near_seabed=current[2] >= seabed_like_depth - seabed_cluster_tolerance,
            )
    return best if math.isfinite(best.radius_m) else _BendRadiusDiagnostic(radius_m=1.0e12)








def _synchronized_input_sample_times(dynamic_case) -> tuple[float, ...]:
    sample_fields = (
        "vessel_motion_samples",
        "plough_motion_samples",
        "payout_speed_samples",
        "plough_exit_speed_samples",
        "current_samples",
    )
    return tuple(sorted({
        float(sample.time_s)
        for field in sample_fields
        for sample in getattr(dynamic_case, field, ())
    }))


def _canonical_synchronized_time_s(
    dynamic_case,
    time_s: float,
    *,
    completed_steps: int,
) -> float:
    """将累积时钟舍入误差吸附到显式输入采样时刻。"""

    closest = float(time_s)
    closest_delta = math.inf
    for sample_time in _synchronized_input_sample_times(dynamic_case):
        scale = max(abs(time_s), abs(sample_time), 1.0)
        tolerance = max(64, completed_steps + 1) * math.ulp(scale)
        delta = abs(time_s - sample_time)
        if delta <= tolerance and delta < closest_delta:
            closest = sample_time
            closest_delta = delta
    return closest


def _next_synchronized_input_sample_time_s(dynamic_case, time_s: float) -> float | None:
    for sample_time in _synchronized_input_sample_times(dynamic_case):
        if sample_time > time_s:
            return sample_time
    return None


def _time_history_step_limit_s(dynamic_case, state: DynamicLayingState, *, base_step_s: float) -> float:
    """使用 CFL 条件和同步输入节点限制积分步长。"""

    cfl_lengths = state.rest_lengths_m
    if dynamic_case.length_boundary_source == "known_plough_trajectory" and len(cfl_lengths) > 1:
    # 最后一个分段是受控材料流出单元，其生命周期由尾部重网格处理；
    # 若在此计入，分段消耗时会形成步长与重网格阈值相互收缩的反馈。
        cfl_lengths = cfl_lengths[:-1]
    positive_lengths = [length for length in cfl_lengths if length > _MIN_LENGTH]
    if positive_lengths:
        min_length = min(positive_lengths)
        speed_scale = max(
            abs(dynamic_case.vessel_initial_speed_mps),
            abs(dynamic_case.vessel_final_speed_mps),
            _max_motion_segment_speed(getattr(dynamic_case, "vessel_motion_segments", ())),
            _max_motion_segment_speed(getattr(dynamic_case, "plough_motion_segments", ())),
            abs(_initial_payout_speed(dynamic_case)),
            abs(_final_payout_speed(dynamic_case)),
            abs(dynamic_case.current_speed_mps),
            abs(dynamic_case.plough_speed_mps or 0.0),
            _max_node_speed(state.velocities),
            _MIN_LENGTH,
        )
        cfl_step = _MAX_NODE_CFL_FRACTION * min_length / speed_scale
        step_limit = max(_MIN_INTERNAL_TIME_STEP_S, min(base_step_s, cfl_step))
    else:
        step_limit = max(_MIN_INTERNAL_TIME_STEP_S, base_step_s)
    next_sample_time = _next_synchronized_input_sample_time_s(
        dynamic_case,
        state.time_s,
    )
    if next_sample_time is not None:
        sample_delta = next_sample_time - state.time_s
        if sample_delta >= _MIN_INTERNAL_TIME_STEP_S:
            step_limit = min(step_limit, sample_delta)
    return step_limit


def _resolved_time_step_max_s(dynamic_case) -> float:
    configured = getattr(dynamic_case, "integration_time_step_max_s", None)
    if configured is not None:
        value = float(configured)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("integration_time_step_max_s must be positive and finite")
        return value
    return min(0.05, max(dynamic_case.total_duration_s / 7200.0, 0.01))


def _max_node_speed(velocities: tuple[Vector3, ...]) -> float:
    if not velocities:
        return 0.0
    return max(_norm(velocity) for velocity in velocities)


def _max_motion_segment_speed(segments) -> float:
    if not segments:
        return 0.0
    return max(max(abs(segment.start_speed_mps), abs(segment.end_speed_mps)) for segment in segments)


def _mean_positive_length(lengths: tuple[float, ...]) -> float | None:
    positive = [length for length in lengths if length > _MIN_LENGTH]
    if not positive:
        return None
    return sum(positive) / len(positive)




def _min_positive_length(lengths: tuple[float, ...]) -> float | None:
    positive = [length for length in lengths if length > _MIN_LENGTH]
    if not positive:
        return None
    return min(positive)


def _operation_case_at_time(
    dynamic_case,
    cable,
    time_s: float,
) -> StepConditions:
    """将时变海流和材料数据固化为当前求解步条件。"""

    current_x, current_y = _current_velocity_components(dynamic_case, time_s)
    current_speed = math.hypot(current_x, current_y)
    current_direction = (
        math.degrees(math.atan2(current_y, current_x)) % 360.0
        if current_speed > _MIN_LENGTH
        else dynamic_case.current_direction_deg
    )
    return StepConditions(
        cable=cable,
        water_depth_m=dynamic_case.water_depth_m,
        current_surface_mps=current_speed,
        current_bottom_mps=(
            current_speed
            if dynamic_case.current_bottom_speed_mps is None
            else dynamic_case.current_bottom_speed_mps
        ),
        current_profile_exponent=dynamic_case.current_profile_exponent,
        current_direction_deg=current_direction,
        payout_speed_mps=_payout_speed(dynamic_case, time_s),
    )


def _current_velocity_components(dynamic_case, time_s: float) -> tuple[float, float]:
    """优先采用海流采样，否则采用恒定流速和流向输入。"""

    samples = getattr(dynamic_case, "current_samples", ())
    if samples:
        start, end, fraction = _sample_bracket(samples, time_s)
        if (
            start.interpolation == "polar_unwrapped"
            and end.interpolation == "polar_unwrapped"
            and start.speed_mps is not None
            and end.speed_mps is not None
            and start.direction_unwrapped_deg is not None
            and end.direction_unwrapped_deg is not None
        ):
            speed = start.speed_mps + (end.speed_mps - start.speed_mps) * fraction
            direction = start.direction_unwrapped_deg + (
                end.direction_unwrapped_deg - start.direction_unwrapped_deg
            ) * fraction
            radians = math.radians(direction)
            return float(speed * math.cos(radians)), float(speed * math.sin(radians))
        return (
            float(start.velocity_x_mps + (end.velocity_x_mps - start.velocity_x_mps) * fraction),
            float(start.velocity_y_mps + (end.velocity_y_mps - start.velocity_y_mps) * fraction),
        )
    direction = math.radians(dynamic_case.current_direction_deg)
    return (
        float(dynamic_case.current_speed_mps * math.cos(direction)),
        float(dynamic_case.current_speed_mps * math.sin(direction)),
    )


def _sampled_scalar_value(samples, time_s: float) -> float:
    start, end, fraction = _sample_bracket(samples, time_s)
    return float(start.value + (end.value - start.value) * fraction)


def _sample_bracket(samples, time_s: float):
    if not samples:
        raise ValueError("at least one synchronized sample is required")
    if len(samples) == 1 or time_s <= samples[0].time_s:
        return samples[0], samples[0], 0.0
    for start, end in zip(samples, samples[1:]):
        if time_s <= end.time_s:
            duration = max(end.time_s - start.time_s, _MIN_LENGTH)
            fraction = max(0.0, min(1.0, (time_s - start.time_s) / duration))
            return start, end, fraction
    return samples[-1], samples[-1], 0.0




def _segment_tensions_from_length_constraints(
    length_lambdas_n_s2: tuple[float, ...],
    *,
    dt_s: float,
    expected_count: int,
) -> tuple[float, ...]:
    """将 XPBD 长度乘子转换为分段反力大小。"""

    if dt_s <= 0.0 or len(length_lambdas_n_s2) != expected_count:
        return ()
    dt2 = dt_s * dt_s
    return tuple(max(0.0, lambda_value / dt2) for lambda_value in length_lambdas_n_s2)




def _point_tensions_from_segment_tensions(
    state: DynamicLayingState,
    segment_tensions: tuple[float, ...],
) -> tuple[float, ...]:
    """将显式分段张力场映射到帧节点。"""

    _validate_state(state)
    if not segment_tensions:
        return tuple(0.0 for _ in state.positions)
    point_tensions: list[float] = []
    for index in range(len(state.positions)):
        adjacent: list[float] = []
        if index > 0 and index - 1 < len(segment_tensions):
            adjacent.append(segment_tensions[index - 1])
        if index < len(segment_tensions):
            adjacent.append(segment_tensions[index])
        if adjacent:
            point_tensions.append(max(0.0, sum(adjacent) / len(adjacent)))
        else:
            point_tensions.append(0.0)
    return tuple(point_tensions)


def _length_constraint_reactions_from_dynamic_state(state: DynamicLayingState) -> tuple[float, ...]:
    _validate_state(state)
    if state.length_constraint_reactions_n:
        return tuple(max(0.0, reaction) for reaction in state.length_constraint_reactions_n)
    return ()


def _dynamic_segment_tensions(
    dynamic_case,
    case: StepConditions,
    state: DynamicLayingState,
    time_s: float,
) -> tuple[float, ...]:
    """返回动态求解中可用口径最优先的逐段张力。"""

    _validate_state(state)
    if state.segment_tensions_n:
        return tuple(max(0.0, tension) for tension in state.segment_tensions_n)
    return _segment_tensions_from_state(case, state)


def _known_plough_output_segment_tensions(
    dynamic_case,
    case: StepConditions,
    state: DynamicLayingState,
    time_s: float,
) -> tuple[float, ...]:
    """分段分布输出采用 XPBD 缆线反力。"""

    _validate_state(state)
    natural_tensions = _dynamic_segment_tensions(dynamic_case, case, state, time_s)
    length_reactions = _length_constraint_reactions_from_dynamic_state(state)
    if len(length_reactions) != len(state.rest_lengths_m):
        return natural_tensions
    return length_reactions


# 工程输出恢复过程保持边界反力与分段张力相互独立。
def _fairlead_boundary_axial_support_reaction(
    case: StepConditions,
    state: DynamicLayingState,
    *,
    adjacent_segment_tension_n: float,
    payout_speed_mps: float,
    plough_exit_speed_mps: float | None,
    prescribed_acceleration: Vector3,
) -> float:
    """返回沿首段切向反方向的船端支反力。"""

    _validate_state(state)
    top_tangent = segment_vectors(state.positions)[0].tangent
    endpoint_external_force = compute_forces(
        case,
        state,
        seabed_depth_m=None,
        payout_speed_mps=payout_speed_mps,
        plough_exit_speed_mps=plough_exit_speed_mps,
        include_axial_tension=False,
    )[0]
    endpoint_axial_mass = _node_axial_masses(case, state)[0]
    axial_support = (
        max(0.0, adjacent_segment_tension_n)
        + _dot(endpoint_external_force, top_tangent)
        - endpoint_axial_mass * _dot(prescribed_acceleration, top_tangent)
    )
    return max(0.0, axial_support)


def _fairlead_boundary_tension_from_dynamic_state(
    dynamic_case,
    case: StepConditions,
    state: DynamicLayingState,
    time_s: float,
    *,
    adjacent_segment_tension_n: float,
) -> float:
    """根据当前给定边界恢复导缆点支承张力。"""

    plough_exit_speed, _ = _plough_exit_material_speed(
        dynamic_case,
        _vessel_velocity(dynamic_case, time_s),
        time_s=time_s,
    )
    return _fairlead_boundary_axial_support_reaction(
        case,
        state,
        adjacent_segment_tension_n=adjacent_segment_tension_n,
        payout_speed_mps=_payout_speed(dynamic_case, time_s),
        plough_exit_speed_mps=plough_exit_speed,
        prescribed_acceleration=_vessel_acceleration_vector(dynamic_case, time_s),
    )


def _plough_boundary_axial_support_reaction(
    case: StepConditions,
    state: DynamicLayingState,
    *,
    adjacent_segment_tension_n: float,
    payout_speed_mps: float,
    plough_exit_speed_mps: float | None,
    prescribed_acceleration: Vector3,
) -> float:
    """返回给定犁端节点沿尾段切向的支反力。"""

    _validate_state(state)
    tail_tangent = segment_vectors(state.positions)[-1].tangent
    endpoint_external_force = compute_forces(
        case,
        state,
        seabed_depth_m=None,
        payout_speed_mps=payout_speed_mps,
        plough_exit_speed_mps=plough_exit_speed_mps,
        include_axial_tension=False,
    )[-1]
    endpoint_axial_mass = _node_axial_masses(case, state)[-1]
    axial_support = (
        max(0.0, adjacent_segment_tension_n)
        + endpoint_axial_mass * _dot(prescribed_acceleration, tail_tangent)
        - _dot(endpoint_external_force, tail_tangent)
    )
    return max(0.0, axial_support)


def _plough_boundary_tension_from_dynamic_state(
    dynamic_case,
    case: StepConditions,
    state: DynamicLayingState,
    time_s: float,
    *,
    adjacent_segment_tension_n: float,
) -> float:
    """根据给定边界恢复犁入口支承张力。"""

    plough_exit_speed, _ = _plough_exit_material_speed(
        dynamic_case,
        _vessel_velocity(dynamic_case, time_s),
        time_s=time_s,
    )
    return _plough_boundary_axial_support_reaction(
        case,
        state,
        adjacent_segment_tension_n=adjacent_segment_tension_n,
        payout_speed_mps=_payout_speed(dynamic_case, time_s),
        plough_exit_speed_mps=plough_exit_speed,
        prescribed_acceleration=_plough_acceleration(dynamic_case, time_s),
    )


def _plough_and_contact_transition_tensions(
    *,
    segment_tensions: tuple[float, ...],
    rest_lengths_m: tuple[float, ...],
    contact_profile: SegmentContactProfile,
) -> tuple[float, float | None]:
    """从同一张力场提取犁端相邻分段及可选 TDP 张力。"""

    if not segment_tensions:
        raise ValueError("segment_tensions must contain the active cable segments")
    plough_inlet_tension = float(segment_tensions[-1])
    if not contact_profile.has_contact:
        return plough_inlet_tension, None
    contact_transition_tension = max(
        0.0,
        _segment_field_at_material_station(
            values=segment_tensions,
            rest_lengths_m=rest_lengths_m,
            material_station_m=contact_profile.tdp_arc_length_m,
        ),
    )
    return plough_inlet_tension, contact_transition_tension


def _segment_field_at_material_station(
    *,
    values: tuple[float, ...],
    rest_lengths_m: tuple[float, ...],
    material_station_m: float,
) -> float:
    """在未伸长材料弧长上插值分段中心标量。"""

    if len(values) != len(rest_lengths_m):
        raise ValueError("values and rest_lengths_m must have the same length")
    if not values:
        return 0.0
    if any(length <= 0.0 for length in rest_lengths_m):
        raise ValueError("rest_lengths_m must be positive")
    centers: list[float] = []
    cursor = 0.0
    for length in rest_lengths_m:
        centers.append(cursor + 0.5 * length)
        cursor += length
    station = max(0.0, min(float(material_station_m), cursor))
    if station <= centers[0]:
        return float(values[0])
    if station >= centers[-1]:
        return float(values[-1])
    for index in range(len(centers) - 1):
        left_station = centers[index]
        right_station = centers[index + 1]
        if station > right_station:
            continue
        fraction = (station - left_station) / max(right_station - left_station, _MIN_LENGTH)
        return float(values[index] + fraction * (values[index + 1] - values[index]))
    return float(values[-1])




def _step_dynamic_segment_tensions(
    case: StepConditions,
    *,
    positions: tuple[Vector3, ...],
    velocities: tuple[Vector3, ...],
    rest_lengths_m: tuple[float, ...],
    payout_speed_mps: float,
    plough_exit_speed_mps: float | None = None,
    terminal_tension_n: float | None = None,
) -> tuple[float, ...]:
    """返回动态步后的载荷递推分段张力估计。"""

    segments = segment_vectors(positions)
    if len(segments) != len(rest_lengths_m):
        raise ValueError("rest_lengths_m must have one entry per segment")
    segment_tensions = [0.0 for _ in rest_lengths_m]
    terminal_tension = 0.0 if terminal_tension_n is None else terminal_tension_n
    running = max(0.0, terminal_tension)
    material_speeds = _segment_material_flow_speeds(
        rest_lengths_m,
        fairlead_speed_mps=payout_speed_mps,
        plough_speed_mps=plough_exit_speed_mps,
    )
    for segment, rest_length, material_speed in reversed(
        list(zip(segments, rest_lengths_m, material_speeds))
    ):
        left = segment.index
        right = left + 1
        midpoint_depth = 0.5 * (segment.start[2] + segment.end[2])
        midpoint_velocity = _mul(_add(velocities[left], velocities[right]), 0.5)
        material_velocity = _segment_material_velocity(
            node_velocity=midpoint_velocity,
            tangent=segment.tangent,
            payout_speed_mps=material_speed,
        )
        water_velocity = current_at(
            depth_m=midpoint_depth,
            water_depth_m=case.water_depth_m,
            current_surface_mps=case.current_surface_mps,
            current_bottom_mps=case.current_bottom_mps,
            current_profile_exponent=case.current_profile_exponent,
            current_direction_deg=case.current_direction_deg,
        )
        relative_velocity = _sub(material_velocity, water_velocity)
        drag = morison_drag(
            seawater_density=_SEAWATER_DENSITY_KG_M3,
            diameter_m=case.cable.diameter_m,
            segment_length_m=segment.length_m,
            tangent=segment.tangent,
            relative_velocity=relative_velocity,
            tangential_coefficient=case.cable.tangential_drag_coefficient,
            normal_coefficient=case.cable.normal_drag_coefficient,
        )
        weight = (0.0, 0.0, case.cable.submerged_weight_n_per_m * rest_length)
        tangential_dynamic_load = _dot(_add(weight, drag), segment.tangent)
        running = max(0.0, running + tangential_dynamic_load)
        segment_tensions[segment.index] = running
    return tuple(segment_tensions)








def _state_contact_profile(state: DynamicLayingState, seabed_depth_m: float):
    return build_segment_contact_profile(
        nodes=state.positions,
        rest_lengths_m=state.rest_lengths_m,
        contact_flags=state.contact_flags,
        contact_normal_reactions_n=_padded_values(state.contact_normal_reactions_n, len(state.positions)),
        seabed_depth_m=seabed_depth_m,
    )










def _vessel_speed(dynamic_case, time_s: float) -> float:
    if getattr(dynamic_case, "vessel_motion_segments", ()):
        velocity = _motion_velocity(dynamic_case.vessel_motion_segments, time_s)
        return math.hypot(velocity[0], velocity[1])
    if time_s >= dynamic_case.transition_duration_s:
        return dynamic_case.vessel_final_speed_mps
    fraction = max(0.0, min(1.0, time_s / max(dynamic_case.transition_duration_s, 1.0e-12)))
    return dynamic_case.vessel_initial_speed_mps + (dynamic_case.vessel_final_speed_mps - dynamic_case.vessel_initial_speed_mps) * fraction






def _vessel_acceleration(dynamic_case, time_s: float) -> float:
    if time_s <= 0.0 or time_s > dynamic_case.transition_duration_s:
        return 0.0
    return (dynamic_case.vessel_final_speed_mps - dynamic_case.vessel_initial_speed_mps) / dynamic_case.transition_duration_s






def _payout_speed(dynamic_case, time_s: float) -> float:
    if getattr(dynamic_case, "payout_speed_samples", ()):
        return _sampled_scalar_value(dynamic_case.payout_speed_samples, time_s)
    if getattr(dynamic_case, "payout_speed_segments", ()):
        return _scalar_segment_speed(dynamic_case.payout_speed_segments, time_s)
    initial = _initial_payout_speed(dynamic_case)
    final = _final_payout_speed(dynamic_case)
    if time_s >= dynamic_case.transition_duration_s:
        return final
    fraction = max(0.0, min(1.0, time_s / max(dynamic_case.transition_duration_s, 1.0e-12)))
    return initial + (final - initial) * fraction


def _scalar_segment_speed(segments, time_s: float) -> float:
    remaining = max(0.0, time_s)
    last_segment = None
    for segment in segments:
        last_segment = segment
        duration = max(segment.duration_s, _MIN_LENGTH)
        if remaining <= duration:
            fraction = segment_interpolation_fraction(
                segment.interpolation,
                remaining / duration,
                duration_s=duration,
                sample_interval_s=segment.sample_interval_s,
            )
            return segment.start_speed_mps + (segment.end_speed_mps - segment.start_speed_mps) * fraction
        remaining -= duration
    return 0.0 if last_segment is None else last_segment.end_speed_mps


def _initial_payout_speed(dynamic_case) -> float:
    if getattr(dynamic_case, "payout_speed_samples", ()):
        return float(dynamic_case.payout_speed_samples[0].value)
    if getattr(dynamic_case, "payout_speed_segments", ()):
        return dynamic_case.payout_speed_segments[0].start_speed_mps
    return dynamic_case.vessel_initial_speed_mps if dynamic_case.payout_initial_speed_mps is None else dynamic_case.payout_initial_speed_mps


def _final_payout_speed(dynamic_case) -> float:
    if getattr(dynamic_case, "payout_speed_samples", ()):
        return float(dynamic_case.payout_speed_samples[-1].value)
    if getattr(dynamic_case, "payout_speed_segments", ()):
        return dynamic_case.payout_speed_segments[-1].end_speed_mps
    return dynamic_case.vessel_final_speed_mps if dynamic_case.payout_final_speed_mps is None else dynamic_case.payout_final_speed_mps


def _validate_state(state: DynamicLayingState) -> None:
    """强制检查拓扑、有限值和材料控制体不变量。

    校验同时覆盖网格数组和守恒材料单元，防止数值上看似合理的几何掩盖
    输运或重网格后的材料平衡破坏。
    """

    if len(state.positions) < 2:
        raise ValueError("at least two nodes are required")
    if len(state.velocities) != len(state.positions):
        raise ValueError("velocities must match positions")
    if len(state.rest_lengths_m) != len(state.positions) - 1:
        raise ValueError("rest_lengths_m must have one entry per segment")
    if len(state.contact_flags) != len(state.positions):
        raise ValueError("contact_flags must match positions")
    if state.length_lambdas_n_s2 and len(state.length_lambdas_n_s2) != len(state.rest_lengths_m):
        raise ValueError("length_lambdas_n_s2 must have one entry per segment")
    if state.segment_tensions_n and len(state.segment_tensions_n) != len(state.rest_lengths_m):
        raise ValueError("segment_tensions_n must have one entry per segment")
    if state.length_constraint_reactions_n and len(state.length_constraint_reactions_n) != len(state.rest_lengths_m):
        raise ValueError("length_constraint_reactions_n must have one entry per segment")
    if state.contact_lambdas_n_s2 and len(state.contact_lambdas_n_s2) != len(state.positions):
        raise ValueError("contact_lambdas_n_s2 must match positions")
    if state.contact_normal_reactions_n and len(state.contact_normal_reactions_n) != len(state.positions):
        raise ValueError("contact_normal_reactions_n must match positions")
    for name, value in (
        (
            "material remap energy error",
            state.material_remap_energy_error_per_linear_density_m3_s2,
        ),
        (
            "cumulative material remap energy error",
            state.material_remap_energy_error_cumulative_per_linear_density_m3_s2,
        ),
        (
            "material remap constraint projection numerical energy increment",
            state.material_remap_constraint_projection_numerical_energy_increment_per_linear_density_m3_s2,
        ),
    ):
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    control = state.known_plough_material_control_volume
    if control is not None:
        for name, length in (
            ("fairlead cumulative inflow", control.fairlead_cumulative_inflow_m),
            ("plough cumulative outflow", control.plough_cumulative_outflow_m),
        ):
            if not math.isfinite(length) or length < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for cut_cell in (control.fairlead_cut_cell, control.plough_cut_cell):
            _validate_material_cell_integral(
                _MaterialCellIntegral(
                    length_m=cut_cell.length_m,
                    momentum_per_linear_density_m2_s=(
                        cut_cell.momentum_per_linear_density_m2_s
                    ),
                    kinetic_energy_per_linear_density_m3_s2=(
                        cut_cell.kinetic_energy_per_linear_density_m3_s2
                    ),
                    axial_strain_integral_m=cut_cell.axial_strain_integral_m,
                    axial_strain_squared_integral_m=(
                        cut_cell.axial_strain_squared_integral_m
                    ),
                )
            )
        for cumulative in (
            control.fairlead_cumulative_integral,
            control.plough_cumulative_integral,
        ):
            _validate_material_cell_integral(cumulative)
        for endpoint, cumulative_length, cumulative_integral in (
            (
                "fairlead",
                control.fairlead_cumulative_inflow_m,
                control.fairlead_cumulative_integral,
            ),
            (
                "plough",
                control.plough_cumulative_outflow_m,
                control.plough_cumulative_integral,
            ),
        ):
            tolerance = 64.0 * math.ulp(
                max(abs(cumulative_length), abs(cumulative_integral.length_m))
            )
            if abs(cumulative_integral.length_m - cumulative_length) > tolerance:
                raise ValueError(f"{endpoint} cumulative material length is inconsistent")
        if control.material_cells:
            if len(control.material_cells) != len(state.rest_lengths_m):
                raise ValueError("material_cells must have one entry per segment")
            for cell, rest_length in zip(control.material_cells, state.rest_lengths_m):
                _validate_material_cell_integral(cell)
                tolerance = 64.0 * math.ulp(max(abs(cell.length_m), abs(rest_length)))
                if abs(cell.length_m - rest_length) > tolerance:
                    raise ValueError("material-cell length must match its grid cell")
            material_length = math.fsum(cell.length_m for cell in control.material_cells)
            reference_length = math.fsum(state.rest_lengths_m)
            tolerance = 64.0 * math.ulp(max(abs(material_length), abs(reference_length)))
            if abs(material_length - reference_length) > tolerance:
                raise ValueError("distributed material length must close on the active grid")




def _segment_tensions_from_state(case: StepConditions, state: DynamicLayingState) -> tuple[float, ...]:
    """根据相对静止长度的伸长量返回逐段张力。"""

    _validate_state(state)
    return tuple(
        _segment_tension(case, segment.length_m, rest_length)
        for segment, rest_length in zip(segment_vectors(state.positions), state.rest_lengths_m)
    )


def _segment_tension(case: StepConditions, length_m: float, rest_length_m: float) -> float:
    rest = max(rest_length_m, _MIN_LENGTH)
    strain = (length_m - rest) / rest
    return max(0.0, case.cable.axial_stiffness_n * strain)


def _node_masses(case: StepConditions, state: DynamicLayingState) -> tuple[float, ...]:
    """返回包含法向附加质量的节点横向质量。"""

    mass_per_meter = _normal_dynamic_mass_per_meter(case)
    return tuple(
        max(_node_tributary_length(index, state.rest_lengths_m) * mass_per_meter, _MIN_MASS)
        for index in range(len(state.positions))
    )


def _node_axial_masses(case: StepConditions, state: DynamicLayingState) -> tuple[float, ...]:
    mass_per_meter = _axial_dynamic_mass_per_meter(case)
    return tuple(
        max(_node_tributary_length(index, state.rest_lengths_m) * mass_per_meter, _MIN_MASS)
        for index in range(len(state.positions))
    )


def _directional_node_accelerations(
    case: StepConditions,
    state: DynamicLayingState,
    forces: tuple[Vector3, ...],
) -> tuple[Vector3, ...]:
    """在各缆线节点应用 M = m_s I + m_a(I-tt^T)。"""

    if len(forces) != len(state.positions):
        raise ValueError("forces must contain one vector per node")
    segments = segment_vectors(state.positions)
    axial_masses = _node_axial_masses(case, state)
    normal_masses = _node_masses(case, state)
    accelerations: list[Vector3] = []
    for index, (force, axial_mass, normal_mass) in enumerate(
        zip(forces, axial_masses, normal_masses)
    ):
        tangent = _node_tangent(segments, index)
        tangential_force = _mul(tangent, _dot(force, tangent))
        normal_force = _sub(force, tangential_force)
        accelerations.append(
            _add(
                _mul(tangential_force, 1.0 / max(axial_mass, _MIN_MASS)),
                _mul(normal_force, 1.0 / max(normal_mass, _MIN_MASS)),
            )
        )
    return tuple(accelerations)


def _node_inverse_mass_matrices(
    case: StepConditions,
    state: DynamicLayingState,
    *,
    fixed_indices: tuple[int, ...] = (),
):
    """返回 M=m_s I+m_a(I-tt^T) 对应的节点 M^-1。"""

    segments = segment_vectors(state.positions)
    axial_masses = _node_axial_masses(case, state)
    normal_masses = _node_masses(case, state)
    fixed = set(fixed_indices)
    matrices = []
    for index, (axial_mass, normal_mass) in enumerate(zip(axial_masses, normal_masses)):
        if index in fixed:
            matrices.append(((0.0, 0.0, 0.0),) * 3)
            continue
        tangent = _node_tangent(segments, index)
        inverse_normal = 1.0 / max(normal_mass, _MIN_MASS)
        directional_delta = 1.0 / max(axial_mass, _MIN_MASS) - inverse_normal
        matrices.append(
            tuple(
                tuple(
                    (inverse_normal if row == column else 0.0)
                    + directional_delta * tangent[row] * tangent[column]
                    for column in range(3)
                )
                for row in range(3)
            )
        )
    return tuple(matrices)


def _structural_mass_per_meter(case: StepConditions) -> float:
    return case.cable.weight_air_n_per_m / _GRAVITY_MPS2


def _displaced_water_mass_per_meter(case: StepConditions) -> float:
    return _SEAWATER_DENSITY_KG_M3 * math.pi * case.cable.diameter_m**2 / 4.0


def _axial_dynamic_mass_per_meter(case: StepConditions) -> float:
    return (
        _structural_mass_per_meter(case)
        + AXIAL_ADDED_MASS_COEFFICIENT * _displaced_water_mass_per_meter(case)
    )


def _normal_dynamic_mass_per_meter(case: StepConditions) -> float:
    return (
        _structural_mass_per_meter(case)
        + NORMAL_ADDED_MASS_COEFFICIENT * _displaced_water_mass_per_meter(case)
    )




def _node_tributary_length(index: int, rest_lengths_m: tuple[float, ...]) -> float:
    length = 0.0
    if index > 0:
        length += 0.5 * rest_lengths_m[index - 1]
    if index < len(rest_lengths_m):
        length += 0.5 * rest_lengths_m[index]
    return length


def _node_tangent(segments, index: int) -> Vector3:
    if not segments:
        return (0.0, 0.0, 0.0)
    if index <= 0:
        return segments[0].tangent
    if index >= len(segments):
        return segments[-1].tangent
    averaged = _add(segments[index - 1].tangent, segments[index].tangent)
    magnitude = _norm(averaged)
    if magnitude <= _MIN_LENGTH:
        return segments[index].tangent
    return _mul(averaged, 1.0 / magnitude)


def _segment_material_velocity(
    *,
    node_velocity: Vector3,
    tangent: Vector3,
    payout_speed_mps: float,
) -> Vector3:
    """返回 ALE 框架中用于 Morison 阻力的材料绝对速度。

    ``node_velocity`` 是运动网格速度，``payout_speed_mps`` 是材料相对网格的流动速度，
    因此两项切向贡献均需计入。
    """

    return _add(node_velocity, _mul(tangent, payout_speed_mps))


def _segment_material_flow_speeds(
    rest_lengths_m: tuple[float, ...],
    *,
    fairlead_speed_mps: float,
    plough_speed_mps: float | None,
) -> tuple[float, ...]:
    """在分段材料中点插值 ALE 材料通量速度。"""

    plough_speed = fairlead_speed_mps if plough_speed_mps is None else plough_speed_mps
    total_length = sum(rest_lengths_m)
    if total_length <= _MIN_LENGTH:
        return tuple(fairlead_speed_mps for _ in rest_lengths_m)
    coordinate = 0.0
    speeds = []
    for rest_length in rest_lengths_m:
        midpoint_fraction = (coordinate + 0.5 * rest_length) / total_length
        speeds.append(
            fairlead_speed_mps
            + midpoint_fraction * (plough_speed - fairlead_speed_mps)
        )
        coordinate += rest_length
    return tuple(speeds)


def _node_material_flow_speeds(
    rest_lengths_m: tuple[float, ...],
    *,
    fairlead_speed_mps: float,
    plough_speed_mps: float | None,
) -> tuple[float, ...]:
    plough_speed = fairlead_speed_mps if plough_speed_mps is None else plough_speed_mps
    total_length = sum(rest_lengths_m)
    if total_length <= _MIN_LENGTH:
        return tuple(fairlead_speed_mps for _ in range(len(rest_lengths_m) + 1))
    coordinate = 0.0
    speeds = [fairlead_speed_mps]
    for rest_length in rest_lengths_m:
        coordinate += rest_length
        fraction = min(1.0, max(0.0, coordinate / total_length))
        speeds.append(fairlead_speed_mps + fraction * (plough_speed - fairlead_speed_mps))
    return tuple(speeds)


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a: Vector3) -> float:
    return math.sqrt(_dot(a, a))
