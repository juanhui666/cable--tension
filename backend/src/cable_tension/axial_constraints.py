"""缆线链的全局耦合单侧轴向约束。

相邻分段约束共享节点自由度，因此受拉有效段按耦合三对角块求解，
而不是分别进行局部修正。乘子保持非负：缆线承受拉力，但不产生人为轴向压力。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence


Vector3 = tuple[float, float, float]
Matrix3 = tuple[Vector3, Vector3, Vector3]

_MIN_PIVOT = 1.0e-18
_DEFAULT_ACTIVATION_TOLERANCE_M = 1.0e-12


@dataclass(frozen=True)
class GlobalAxialConstraintStep:
    """一次非线性 XPBD 修正及其收敛诊断。"""

    positions: tuple[Vector3, ...]
    lambdas_n_s2: tuple[float, ...]
    max_residual_m: float
    active_constraint_count: int


def solve_global_axial_constraint_step(
    *,
    positions: Sequence[Vector3],
    rest_lengths_m: Sequence[float],
    inverse_masses_per_kg: Sequence[float],
    inverse_mass_matrices_per_kg: Sequence[Matrix3] | None = None,
    axial_stiffness_n: float,
    dt_s: float,
    lambdas_n_s2: Sequence[float] = (),
    activation_tolerance_m: float = _DEFAULT_ACTIVATION_TOLERANCE_M,
) -> GlobalAxialConstraintStep:
    """执行一次非线性全局耦合 XPBD 轴向修正。"""

    points = tuple(tuple(float(value) for value in point) for point in positions)
    rest_lengths = tuple(float(value) for value in rest_lengths_m)
    inverse_masses = tuple(float(value) for value in inverse_masses_per_kg)
    inverse_mass_matrices = (
        tuple(_matrix3(matrix) for matrix in inverse_mass_matrices_per_kg)
        if inverse_mass_matrices_per_kg is not None
        else tuple(_isotropic_inverse_mass(value) for value in inverse_masses)
    )
    _validate_inputs(
        positions=points,
        rest_lengths_m=rest_lengths,
        inverse_masses_per_kg=inverse_masses,
        inverse_mass_matrices_per_kg=inverse_mass_matrices,
        axial_stiffness_n=axial_stiffness_n,
        dt_s=dt_s,
        lambdas_n_s2=lambdas_n_s2,
        activation_tolerance_m=activation_tolerance_m,
    )
    if not rest_lengths:
        return GlobalAxialConstraintStep(points, (), 0.0, 0)

    lambdas = (
        [0.0 for _ in rest_lengths]
        if not lambdas_n_s2
        else [max(0.0, float(value)) for value in lambdas_n_s2]
    )
    tangents: list[Vector3] = []
    constraints: list[float] = []
    compliances: list[float] = []
    dt2 = dt_s * dt_s
    for index, rest_length in enumerate(rest_lengths):
        delta = _sub(points[index + 1], points[index])
        length = _norm(delta)
        tangent = (0.0, 0.0, 1.0) if length <= _MIN_PIVOT else _mul(delta, 1.0 / length)
        tangents.append(tangent)
        constraints.append(length - rest_length)
        compliances.append(rest_length / (axial_stiffness_n * dt2))

    # 正的上一步乘子使分段在卸载过程中保持有效；下方截断避免该历史量转为压力。
    active = tuple(
        constraint > activation_tolerance_m or lambda_value > 0.0
        for constraint, lambda_value in zip(constraints, lambdas)
    )
    applied_lambdas = [0.0 for _ in rest_lengths]
    # 无效的松弛段将缆线链分隔为相互独立的有效块。
    start = 0
    while start < len(rest_lengths):
        if not active[start]:
            start += 1
            continue
        end = start
        while end + 1 < len(rest_lengths) and active[end + 1]:
            end += 1
        increments = _solve_active_block(
            start=start,
            end=end,
            tangents=tangents,
            constraints=constraints,
            compliances=compliances,
            inverse_mass_matrices=inverse_mass_matrices,
            lambdas=lambdas,
        )
        for local_index, segment_index in enumerate(range(start, end + 1)):
            next_lambda = max(0.0, lambdas[segment_index] + increments[local_index])
            applied_lambdas[segment_index] = next_lambda - lambdas[segment_index]
            lambdas[segment_index] = next_lambda
        start = end + 1

    corrections = [(0.0, 0.0, 0.0) for _ in points]
    for index, (tangent, applied_lambda) in enumerate(zip(tangents, applied_lambdas)):
        corrections[index] = _add(
            corrections[index],
            _mul(_matvec(inverse_mass_matrices[index], tangent), applied_lambda),
        )
        corrections[index + 1] = _sub(
            corrections[index + 1],
            _mul(_matvec(inverse_mass_matrices[index + 1], tangent), applied_lambda),
        )
    corrected = tuple(_add(point, correction) for point, correction in zip(points, corrections))
    residual = axial_constraint_residual_m(
        positions=corrected,
        rest_lengths_m=rest_lengths,
        lambdas_n_s2=lambdas,
        axial_stiffness_n=axial_stiffness_n,
        dt_s=dt_s,
    )
    return GlobalAxialConstraintStep(
        positions=corrected,
        lambdas_n_s2=tuple(lambdas),
        max_residual_m=residual,
        active_constraint_count=sum(active),
    )


def axial_constraint_residual_m(
    *,
    positions: Sequence[Vector3],
    rest_lengths_m: Sequence[float],
    lambdas_n_s2: Sequence[float],
    axial_stiffness_n: float,
    dt_s: float,
) -> float:
    """返回单侧 XPBD 驻定条件的最大残差，单位为米。"""

    if len(positions) != len(rest_lengths_m) + 1:
        raise ValueError("positions must contain one more entry than rest_lengths_m")
    if len(lambdas_n_s2) != len(rest_lengths_m):
        raise ValueError("lambdas_n_s2 must contain one entry per segment")
    if not math.isfinite(axial_stiffness_n) or axial_stiffness_n <= 0.0:
        raise ValueError("axial_stiffness_n must be positive and finite")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be positive and finite")
    if any(not math.isfinite(value) or value <= 0.0 for value in rest_lengths_m):
        raise ValueError("rest_lengths_m entries must be positive and finite")
    if any(not math.isfinite(value) or value < 0.0 for value in lambdas_n_s2):
        raise ValueError("lambdas_n_s2 entries must be non-negative and finite")
    if any(len(point) != 3 or any(not math.isfinite(value) for value in point) for point in positions):
        raise ValueError("positions entries must be finite three-dimensional vectors")
    dt2 = dt_s * dt_s
    maximum = 0.0
    for index, (rest_length, lambda_value) in enumerate(zip(rest_lengths_m, lambdas_n_s2)):
        constraint = math.dist(positions[index], positions[index + 1]) - rest_length
        if lambda_value <= 0.0 and constraint <= 0.0:
            residual = 0.0
        else:
            compliance = rest_length / (axial_stiffness_n * dt2)
            residual = abs(constraint - compliance * lambda_value)
        maximum = max(maximum, residual)
    return maximum


def _solve_active_block(
    *,
    start: int,
    end: int,
    tangents: Sequence[Vector3],
    constraints: Sequence[float],
    compliances: Sequence[float],
    inverse_mass_matrices: Sequence[Matrix3],
    lambdas: Sequence[float],
) -> list[float]:
    """组装并求解一个连续的有效轴向约束块。

    非对角项表示方向逆质量矩阵作用下，相邻分段切向通过共享节点形成的耦合。
    """

    diagonal: list[float] = []
    lower: list[float] = []
    upper: list[float] = []
    right_hand_side: list[float] = []
    for segment_index in range(start, end + 1):
        diagonal.append(
            _dot(tangents[segment_index], _matvec(inverse_mass_matrices[segment_index], tangents[segment_index]))
            + _dot(tangents[segment_index], _matvec(inverse_mass_matrices[segment_index + 1], tangents[segment_index]))
            + compliances[segment_index]
        )
        right_hand_side.append(
            constraints[segment_index] - compliances[segment_index] * lambdas[segment_index]
        )
        if segment_index < end:
            coupling = -_dot(
                tangents[segment_index],
                _matvec(inverse_mass_matrices[segment_index + 1], tangents[segment_index + 1]),
            )
            lower.append(coupling)
            upper.append(coupling)
    return _solve_tridiagonal(lower, diagonal, upper, right_hand_side)


def _solve_tridiagonal(
    lower: Sequence[float],
    diagonal: Sequence[float],
    upper: Sequence[float],
    right_hand_side: Sequence[float],
) -> list[float]:
    """使用 Thomas 消元法求解非奇异三对角方程组。"""

    count = len(diagonal)
    if count == 0:
        return []
    if len(lower) != count - 1 or len(upper) != count - 1 or len(right_hand_side) != count:
        raise ValueError("invalid tridiagonal system dimensions")
    solved_diagonal = list(diagonal)
    solved_rhs = list(right_hand_side)
    solved_upper = list(upper)
    for index in range(1, count):
        pivot = solved_diagonal[index - 1]
        if abs(pivot) <= _MIN_PIVOT:
            raise RuntimeError("singular global axial constraint system")
        factor = lower[index - 1] / pivot
        solved_diagonal[index] -= factor * solved_upper[index - 1]
        solved_rhs[index] -= factor * solved_rhs[index - 1]
    if abs(solved_diagonal[-1]) <= _MIN_PIVOT:
        raise RuntimeError("singular global axial constraint system")
    solution = [0.0 for _ in range(count)]
    solution[-1] = solved_rhs[-1] / solved_diagonal[-1]
    for index in range(count - 2, -1, -1):
        pivot = solved_diagonal[index]
        if abs(pivot) <= _MIN_PIVOT:
            raise RuntimeError("singular global axial constraint system")
        solution[index] = (
            solved_rhs[index] - solved_upper[index] * solution[index + 1]
        ) / pivot
    return solution


def _validate_inputs(
    *,
    positions: Sequence[Vector3],
    rest_lengths_m: Sequence[float],
    inverse_masses_per_kg: Sequence[float],
    inverse_mass_matrices_per_kg: Sequence[Matrix3],
    axial_stiffness_n: float,
    dt_s: float,
    lambdas_n_s2: Sequence[float],
    activation_tolerance_m: float,
) -> None:
    """校验耦合 XPBD 求解所要求的维数和正值条件。"""

    if len(positions) != len(rest_lengths_m) + 1:
        raise ValueError("positions must contain one more entry than rest_lengths_m")
    if len(inverse_masses_per_kg) != len(positions):
        raise ValueError("inverse_masses_per_kg must contain one entry per position")
    if len(inverse_mass_matrices_per_kg) != len(positions):
        raise ValueError("inverse_mass_matrices_per_kg must contain one entry per position")
    if lambdas_n_s2 and len(lambdas_n_s2) != len(rest_lengths_m):
        raise ValueError("lambdas_n_s2 must contain one entry per segment")
    if not math.isfinite(axial_stiffness_n) or axial_stiffness_n <= 0.0:
        raise ValueError("axial_stiffness_n must be positive and finite")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("dt_s must be positive and finite")
    if not math.isfinite(activation_tolerance_m) or activation_tolerance_m < 0.0:
        raise ValueError("activation_tolerance_m must be non-negative and finite")
    if any(not math.isfinite(value) or value <= 0.0 for value in rest_lengths_m):
        raise ValueError("rest_lengths_m entries must be positive and finite")
    if any(not math.isfinite(value) or value < 0.0 for value in inverse_masses_per_kg):
        raise ValueError("inverse_masses_per_kg entries must be non-negative and finite")
    for matrix in inverse_mass_matrices_per_kg:
        if any(not math.isfinite(value) for row in matrix for value in row):
            raise ValueError("inverse_mass_matrices_per_kg entries must be finite")
        if any(
            not math.isclose(matrix[row][column], matrix[column][row], abs_tol=1.0e-12)
            for row in range(3)
            for column in range(3)
        ):
            raise ValueError("inverse_mass_matrices_per_kg entries must be symmetric")
    if any(not math.isfinite(value) or value < 0.0 for value in lambdas_n_s2):
        raise ValueError("lambdas_n_s2 entries must be non-negative and finite")
    if any(len(point) != 3 or any(not math.isfinite(value) for value in point) for point in positions):
        raise ValueError("positions entries must be finite three-dimensional vectors")


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


def _matrix3(matrix: Sequence[Sequence[float]]) -> Matrix3:
    if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
        raise ValueError("inverse mass matrices must be 3x3")
    return tuple(tuple(float(value) for value in row) for row in matrix)  # type: ignore[return-value]


def _isotropic_inverse_mass(value: float) -> Matrix3:
    return ((value, 0.0, 0.0), (0.0, value, 0.0), (0.0, 0.0, value))


def _matvec(matrix: Matrix3, vector: Vector3) -> Vector3:
    return tuple(sum(matrix[row][column] * vector[column] for column in range(3)) for row in range(3))  # type: ignore[return-value]
