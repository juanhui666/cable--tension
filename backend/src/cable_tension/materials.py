"""缆线材料目录及单步物理条件，统一采用 SI 单位。"""

from __future__ import annotations

from dataclasses import dataclass


NORMAL_ADDED_MASS_COEFFICIENT = 1.0
AXIAL_ADDED_MASS_COEFFICIENT = 0.0


@dataclass(frozen=True)
class CableParameters:
    """已知犁轨迹求解器使用的材料字段。"""

    name: str
    diameter_m: float
    weight_air_n_per_m: float
    submerged_weight_n_per_m: float
    tangential_drag_coefficient: float
    normal_drag_coefficient: float
    axial_stiffness_n: float = 1.0e9
    min_bending_radius_m: float | None = None


@dataclass(frozen=True)
class StepConditions:
    """一个求解步采用的物理条件。"""

    cable: CableParameters
    water_depth_m: float
    current_surface_mps: float
    current_bottom_mps: float
    current_profile_exponent: float = 1.0
    current_direction_deg: float = 0.0
    payout_speed_mps: float = 0.0


_MATERIALS: dict[str, CableParameters] = {
    "POWER_500KV": CableParameters(
        name="POWER_500KV",
        diameter_m=0.139,
        weight_air_n_per_m=48.0 * 9.8,
        submerged_weight_n_per_m=48.0 * 9.8 - 1025.0 * 9.8 * 3.141592653589793 * 0.139**2 / 4.0,
        tangential_drag_coefficient=0.0,
        normal_drag_coefficient=1.0,
        axial_stiffness_n=2.66e8,
        min_bending_radius_m=5.0,
    ),
}


def get_material(name: str) -> CableParameters:
    """返回指定名称的缆线材料。"""

    key = name.upper()
    if key not in _MATERIALS:
        raise KeyError(f"unknown cable material: {name}")
    return _MATERIALS[key]
