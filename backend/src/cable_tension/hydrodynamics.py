"""统一的海流与 Morison 水动力载荷。"""

from __future__ import annotations

import math

from .geometry import Vector3


_MIN_NORM = 1.0e-12


def current_at(
    *,
    depth_m: float,
    water_depth_m: float,
    current_surface_mps: float | None = None,
    current_bottom_mps: float | None = None,
    current_profile_exponent: float = 1.0,
    current_direction_deg: float | None = None,
    current_u_mps: float = 0.0,
    current_v_mps: float = 0.0,
) -> Vector3:
    """返回指定水深处在作业坐标系中的海流速度。

    给定表层/底层流速时，剖面采用 ``Ub + (Us - Ub) * (1 - z/H)**p``，
    水深截断到 ``0..H``；``p=1`` 对应线性剖面。否则使用显式 ``u/v`` 分量。
    方向角遵循工程约定：0 deg 为 +X，90 deg 为 +Y。返回值是作业航迹坐标系中的水体速度。
    """

    if water_depth_m <= 0.0:
        raise ValueError("water_depth_m must be positive")
    if not math.isfinite(current_profile_exponent) or current_profile_exponent <= 0.0:
        raise ValueError("current_profile_exponent must be positive and finite")
    if current_surface_mps is not None and current_bottom_mps is not None:
        fraction = max(0.0, min(1.0, depth_m / water_depth_m))
        speed = current_bottom_mps + (current_surface_mps - current_bottom_mps) * (
            1.0 - fraction
        ) ** current_profile_exponent
        direction = math.radians(current_direction_deg or 0.0)
        return (speed * math.cos(direction), speed * math.sin(direction), 0.0)
    return (current_u_mps, current_v_mps, 0.0)


def morison_drag(
    *,
    seawater_density: float,
    diameter_m: float,
    segment_length_m: float,
    tangent: Vector3,
    relative_velocity: Vector3,
    tangential_coefficient: float,
    normal_coefficient: float,
) -> Vector3:
    """返回单个分段在全局坐标中的总 Morison 阻力。

    ``relative_velocity`` 是缆线相对水体的速度。返回力阻碍该相对运动，
    先分解切向与法向分量，再在全局坐标中合成。
    """

    if seawater_density <= 0.0:
        raise ValueError("seawater_density must be positive")
    if diameter_m <= 0.0:
        raise ValueError("diameter_m must be positive")
    if segment_length_m < 0.0:
        raise ValueError("segment_length_m must be non-negative")
    unit_tangent = _unit(tangent)
    relative_t_scalar = _dot(relative_velocity, unit_tangent)
    relative_t = _mul(unit_tangent, relative_t_scalar)
    relative_n = _sub(relative_velocity, relative_t)
    normal_speed = _norm(relative_n)
    tangential_force = _mul(
        unit_tangent,
        -0.5
        * math.pi
        * seawater_density
        * tangential_coefficient
        * diameter_m
        * segment_length_m
        * relative_t_scalar
        * abs(relative_t_scalar),
    )
    normal_force = (
        (0.0, 0.0, 0.0)
        if normal_speed <= _MIN_NORM
        else _mul(
            relative_n,
            -0.5
            * seawater_density
            * normal_coefficient
            * diameter_m
            * segment_length_m
            * normal_speed,
        )
    )
    return _add(tangential_force, normal_force)


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


def _unit(a: Vector3) -> Vector3:
    magnitude = _norm(a)
    if magnitude <= _MIN_NORM:
        raise ValueError("tangent length must be positive")
    return _mul(a, 1.0 / magnitude)
