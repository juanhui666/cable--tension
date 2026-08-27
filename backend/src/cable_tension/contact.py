"""基于节点的动态铺缆海床接触约束。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .geometry import Vector3


_MIN_SPEED = 1.0e-12
SEABED_CONTACT_TOLERANCE_M = 1.0e-3


@dataclass(frozen=True)
class SegmentContactProfile:
    """平坦海床接触区间的材料弧长重构结果。"""

    has_contact: bool
    segment_contact_fractions: tuple[float, ...]
    tdp_segment_index: int
    tdp_segment_fraction: float
    tdp_point: Vector3
    tdp_arc_length_m: float
    suspended_length_m: float
    contact_length_m: float
    normal_resultant_n: float


def build_segment_contact_profile(
    *,
    nodes: Iterable[Vector3],
    rest_lengths_m: Iterable[float],
    contact_flags: Iterable[bool],
    contact_normal_reactions_n: Iterable[float],
    seabed_depth_m: float,
) -> SegmentContactProfile:
    """在材料弧长上重构首个平坦海床接触过渡点。

    穿透过渡采用分段与平面的精确交点。强制投影将首个有效节点置于平面后，
    其集中反力代表节点控制体，因此未解析的接触边界取相邻分段中点。
    """

    node_list = tuple(nodes)
    rest_lengths = tuple(float(value) for value in rest_lengths_m)
    flags = tuple(bool(value) for value in contact_flags)
    reactions = tuple(float(value) for value in contact_normal_reactions_n)
    if len(node_list) < 2:
        raise ValueError("at least two nodes are required")
    if len(rest_lengths) != len(node_list) - 1:
        raise ValueError("rest_lengths_m must have one entry per segment")
    if len(flags) != len(node_list) or len(reactions) != len(node_list):
        raise ValueError("contact fields must have one entry per node")
    if seabed_depth_m < 0.0 or not math.isfinite(seabed_depth_m):
        raise ValueError("seabed_depth_m must be finite and non-negative")
    if any(length <= 0.0 or not math.isfinite(length) for length in rest_lengths):
        raise ValueError("rest_lengths_m must be finite and positive")
    if any(not math.isfinite(reaction) for reaction in reactions):
        raise ValueError("contact_normal_reactions_n must be finite")

    total_length = sum(rest_lengths)
    active = tuple(flag or reaction > 0.0 for flag, reaction in zip(flags, reactions))
    normal_resultant = sum(max(0.0, reaction) for reaction in reactions)
    first_active = next((index for index, value in enumerate(active) if value), None)
    if first_active is None:
        return SegmentContactProfile(
            has_contact=False,
            segment_contact_fractions=tuple(0.0 for _ in rest_lengths),
            tdp_segment_index=len(rest_lengths) - 1,
            tdp_segment_fraction=1.0,
            tdp_point=node_list[-1],
            tdp_arc_length_m=total_length,
            suspended_length_m=total_length,
            contact_length_m=0.0,
            normal_resultant_n=normal_resultant,
        )

    transition_segment = max(0, min(first_active - 1, len(rest_lengths) - 1))
    if first_active == 0:
        transition_fraction = 0.0
    else:
        start_gap = node_list[transition_segment][2] - seabed_depth_m
        end_gap = node_list[transition_segment + 1][2] - seabed_depth_m
        if start_gap < 0.0 < end_gap:
            transition_fraction = -start_gap / (end_gap - start_gap)
        else:
            transition_fraction = 0.5
    transition_fraction = max(0.0, min(1.0, transition_fraction))

    fractions = [0.0 for _ in rest_lengths]
    fractions[transition_segment] = 1.0 - transition_fraction
    for segment_index in range(transition_segment + 1, len(rest_lengths)):
        left_on_bed = node_list[segment_index][2] >= seabed_depth_m - SEABED_CONTACT_TOLERANCE_M
        right_on_bed = node_list[segment_index + 1][2] >= seabed_depth_m - SEABED_CONTACT_TOLERANCE_M
        left_supported = active[segment_index] or left_on_bed
        right_supported = active[segment_index + 1] or right_on_bed
        if left_supported and right_supported and (active[segment_index] or active[segment_index + 1]):
            fractions[segment_index] = 1.0
            continue
        break

    contact_length = sum(length * fraction for length, fraction in zip(rest_lengths, fractions))
    tdp_arc_length = sum(rest_lengths[:transition_segment]) + (
        transition_fraction * rest_lengths[transition_segment]
    )
    start = node_list[transition_segment]
    end = node_list[transition_segment + 1]
    tdp_point = (
        start[0] + transition_fraction * (end[0] - start[0]),
        start[1] + transition_fraction * (end[1] - start[1]),
        seabed_depth_m,
    )
    return SegmentContactProfile(
        has_contact=True,
        segment_contact_fractions=tuple(fractions),
        tdp_segment_index=transition_segment,
        tdp_segment_fraction=transition_fraction,
        tdp_point=tdp_point,
        tdp_arc_length_m=tdp_arc_length,
        suspended_length_m=max(0.0, total_length - contact_length),
        contact_length_m=contact_length,
        normal_resultant_n=normal_resultant,
    )


def seabed_friction(
    *,
    normal_force_n: float,
    tangential_velocity: Vector3,
    friction_coefficient: float,
) -> Vector3:
    """返回阻碍水平接触运动的 Coulomb 摩擦力。"""

    if normal_force_n <= 0.0 or friction_coefficient <= 0.0:
        return (0.0, 0.0, 0.0)
    horizontal = (tangential_velocity[0], tangential_velocity[1], 0.0)
    speed = math.hypot(horizontal[0], horizontal[1])
    if speed <= _MIN_SPEED:
        return (0.0, 0.0, 0.0)
    scale = -friction_coefficient * normal_force_n / speed
    return (horizontal[0] * scale, horizontal[1] * scale, 0.0)
