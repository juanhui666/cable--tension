"""动态铺缆求解器的节点式缆线几何辅助函数。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


Vector3 = tuple[float, float, float]

_MIN_LENGTH_M = 1.0e-12


@dataclass(frozen=True)
class SegmentVector:
    """两个缆线节点之间单个分段的几何信息。"""

    index: int
    start: Vector3
    end: Vector3
    delta: Vector3
    length_m: float
    tangent: Vector3


def segment_vectors(nodes: Iterable[Vector3]) -> list[SegmentVector]:
    """根据节点坐标返回分段差向量、长度和切向。"""

    node_list = tuple(nodes)
    if len(node_list) < 2:
        raise ValueError("at least two nodes are required")
    segments: list[SegmentVector] = []
    for index, (start, end) in enumerate(zip(node_list, node_list[1:])):
        delta = _sub(end, start)
        length = _norm(delta)
        if length <= _MIN_LENGTH_M:
            raise ValueError("segment length must be positive")
        tangent = _mul(delta, 1.0 / length)
        segments.append(
            SegmentVector(
                index=index,
                start=start,
                end=end,
                delta=delta,
                length_m=length,
                tangent=tangent,
            )
        )
    return segments


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _mul(a: Vector3, scalar: float) -> Vector3:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _dot(a: Vector3, b: Vector3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a: Vector3) -> float:
    return math.sqrt(_dot(a, a))
