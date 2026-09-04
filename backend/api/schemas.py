"""最小公开 API 的 JSON 响应序列化。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ApiResponse:
    """HTTP 处理器和单元测试共用的响应对象。"""

    status: int
    body: bytes
    headers: dict[str, str]


def json_response(payload: dict[str, Any], *, status: int = 200) -> ApiResponse:
    return ApiResponse(
        status=status,
        body=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Content-Type": "application/json; charset=utf-8"},
    )


def error_response(
    error: str,
    message: str,
    *,
    status: int,
    details: dict[str, Any] | None = None,
) -> ApiResponse:
    payload: dict[str, Any] = {"error": error, "message": message}
    if details is not None:
        payload["details"] = details
    return json_response(payload, status=status)


def realtime_result_payload(
    result: Any,
    *,
    cable_name: str,
    manufacturer_limits: dict[str, float | None] | None = None,
) -> dict[str, Any]:
    """初始化和每秒更新均返回同一结果结构。"""

    point = result.point
    references = manufacturer_limits or {}
    return {
        "session_id": result.session_id,
        "sequence": int(result.sequence),
        "time_s": float(result.time_s),
        "cable": {"name": cable_name},
        "cable_shape": {
            "points": [
                {
                    "index": int(item.index),
                    "x_m": float(item.x_m),
                    "y_m": float(item.y_m),
                    "z_m": float(item.z_m),
                }
                for item in result.frame.points
            ],
            "segment_tensions_n": [
                float(value) for value in result.frame.segment_tensions_n
            ],
        },
        "tensions": {
            "top_tension_n": float(point.top_tension_n),
            "plough_inlet_tension_n": float(point.plough_inlet_tension_n),
            "measured_top_tension_n": (
                None
                if result.measured_top_tension_n is None
                else float(result.measured_top_tension_n)
            ),
            "top_tension_residual_n": (
                None
                if result.top_tension_residual_n is None
                else float(result.top_tension_residual_n)
            ),
        },
        "vessel_departure_angles": {
            "horizontal_deg": float(result.vessel_departure_horizontal_angle_deg),
            "vertical_deg": float(result.vessel_departure_vertical_angle_deg),
        },
        "minimum_bend_radius": {
            "minimum_m": result.minimum_bend_radius_m,
        },
        "bending": {
            "effective_stiffness_n_m2": float(
                result.effective_bending_stiffness_n_m2
            ),
            "maximum_curvature_per_m": result.maximum_curvature_per_m,
            "minimum_curvature_radius_m": result.minimum_bend_radius_m,
            "maximum_moment_n_m": result.maximum_bending_moment_n_m,
        },
        "manufacturer_limits": {
            field: (
                None if references.get(field) is None else float(references[field])
            )
            for field in (
                "installation_lc_mbr_m",
                "normal_operation_lc_mbr_m",
                "storage_dc_mbr_m",
                "installation_dc_mbr_m",
                "maximum_working_load_n",
                "maximum_abnormal_operation_load_n",
                "dwp_breaking_load_n",
            )
        },
        "runtime": {
            "compute_wall_s": float(result.compute_wall_s),
            "realtime_factor": (
                None if result.realtime_factor is None else float(result.realtime_factor)
            ),
        },
    }
