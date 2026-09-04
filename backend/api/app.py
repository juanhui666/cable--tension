"""海底电力缆实时张力模块的最小 HTTP 边界。

公开业务协议只包含一次会话初始化和每秒一次状态推进。本模块将
平台工程量换算为求解器规范量，但不修改力学模型或求解算法。
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit


BACKEND_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BACKEND_ROOT))
sys.path.insert(0, str(BACKEND_ROOT / "src"))

from cable_tension import __version__ as MODULE_VERSION  # noqa: E402
from cable_tension.dynamic_laying import CableGeometryInfeasibleError  # noqa: E402
from cable_tension.realtime import (  # noqa: E402
    PassivePloughDomainError,
    REALTIME_MAX_SENSOR_GAP_S,
    RealtimeSensorPacket,
    RealtimeSessionError,
    RealtimeSessionRegistry,
    SynchronizedEndpointSample,
    passive_plough_forward_speed,
    passive_plough_kinematic_samples,
)
from cable_tension.simulation import DynamicCaseInput  # noqa: E402

try:  # pragma: no cover - 同时支持包导入和直接运行。
    from .contracts import API_VERSION, API_VERSION_HEADER, json_structure_errors
    from .schemas import ApiResponse, error_response, json_response, realtime_result_payload
except ImportError:  # pragma: no cover
    from contracts import API_VERSION, API_VERSION_HEADER, json_structure_errors  # type: ignore[no-redef]
    from schemas import ApiResponse, error_response, json_response, realtime_result_payload  # type: ignore[no-redef]


_GRAVITY_MPS2 = 9.8
_REALTIME_ELEMENT_COUNT = 48
_REALTIME_INTEGRATION_TIME_STEP_MAX_S = 0.01
_REALTIME_CURRENT_BOTTOM_SPEED_MPS = 0.0
_REALTIME_CURRENT_PROFILE_EXPONENT = 2.0
_SPEED_EPSILON_MPS = 1.0e-12


@dataclass(frozen=True)
class _RealtimeSetup:
    """由公开初始化输入得到的规范静态量。"""

    cable_name: str
    diameter_m: float
    weight_air_n_per_m: float
    submerged_weight_n_per_m: float
    tangential_drag_coefficient: float
    normal_drag_coefficient: float
    axial_stiffness_n: float
    bending_stiffness_n_m2: float
    initial_suspended_length_m: float
    plough_position_mode: str
    manufacturer_limits: dict[str, float | None]


@dataclass(frozen=True)
class _PloughBoundaryState:
    """最近一次成功接受的犁边界及被动拖曳状态。"""

    horizontal_offset_x_m: float
    horizontal_offset_y_m: float
    horizontal_layback_m: float
    heading_rad: float
    depth_m: float
    position_x_m: float
    position_y_m: float
    position_z_m: float
    vessel_velocity_x_mps: float
    vessel_velocity_y_mps: float
    time_s: float
    position_mode: str


class CableApiServer:
    """可测试的最小版本化 API 路由器。"""

    def __init__(self) -> None:
        self.realtime_sessions = RealtimeSessionRegistry()
        self._plough_states: dict[str, _PloughBoundaryState] = {}
        self._cable_names: dict[str, str] = {}
        self._manufacturer_limits: dict[str, dict[str, float | None]] = {}

    def handle(
        self,
        method: str,
        raw_path: str,
        payload: dict[str, Any] | None = None,
    ) -> ApiResponse:
        """分派版本化请求；非 ``/api/v1`` 业务路径始终拒绝。"""

        method = method.upper()
        requested_path = urlsplit(raw_path).path
        versioned, path = _normalize_api_path(requested_path)
        if requested_path.startswith("/api/") and not versioned:
            return error_response("not_found", "The requested API route does not exist.", status=404)
        response = self._dispatch(method, path, payload or {}, versioned=versioned)
        if not versioned:
            return response
        return ApiResponse(
            status=response.status,
            body=response.body,
            headers={**response.headers, API_VERSION_HEADER: API_VERSION},
        )

    def _dispatch(
        self,
        method: str,
        path: str,
        payload: dict[str, Any],
        *,
        versioned: bool,
    ) -> ApiResponse:
        if method == "OPTIONS":
            return ApiResponse(status=204, body=b"", headers={"Content-Type": "text/plain"})
        if not versioned:
            return error_response("not_found", "The requested API route does not exist.", status=404)
        if method == "GET" and path == "/api/health":
            return self._health()
        if method == "POST" and path == "/api/realtime-sessions":
            return self._create_realtime_session(payload)
        match = re.fullmatch(r"/api/realtime-sessions/([^/]+)/samples", path)
        if method == "POST" and match is not None:
            return self._advance_realtime_session(match.group(1), payload)
        return error_response("not_found", "The requested API route does not exist.", status=404)

    def _health(self) -> ApiResponse:
        """返回部署存活状态，不包含业务协议。"""

        return json_response(
            {
                "status": "ok",
                "service": "cable-tension-backend",
                "module_version": MODULE_VERSION,
            }
        )

    def _create_realtime_session(self, payload: dict[str, Any]) -> ApiResponse:
        """换算静态量、构造首包并且只初始化一次求解状态。"""

        structure_errors = json_structure_errors("realtime-session-create", payload)
        if structure_errors:
            return _invalid_input("Realtime initialization does not match the public schema.", structure_errors)
        setup = _parse_realtime_setup(payload)
        if isinstance(setup, ApiResponse):
            return setup
        initial_packet = _parse_public_packet(
            payload.get("initial_packet"),
            previous_plough_state=None,
            plough_position_mode=setup.plough_position_mode,
        )
        if isinstance(initial_packet, ApiResponse):
            return initial_packet
        packet, plough_state = initial_packet
        if packet.plough.z_m < 0.0 or packet.plough.z_m > packet.water_depth_m:
            return _invalid_input(
                "Realtime initialization contains an invalid plough depth.",
                {"initial_packet.plough_position.z_m": "must be between 0 and water depth"},
            )
        base_case = _realtime_case_from_setup(setup, packet)
        try:
            session = self.realtime_sessions.create(
                base_case=base_case,
                initial_packet=packet,
                max_sensor_gap_s=REALTIME_MAX_SENSOR_GAP_S,
            )
        except RealtimeSessionError as exc:
            return _realtime_error_response(exc)
        except CableGeometryInfeasibleError as exc:
            return error_response(
                "solver_infeasible",
                "Realtime initialization has no feasible cable geometry.",
                status=422,
                details={"reason": str(exc)},
            )
        except ValueError as exc:
            return error_response(
                "invalid_input",
                "Realtime initialization values are invalid.",
                status=400,
                details={"reason": str(exc)},
            )
        except RuntimeError as exc:
            return error_response(
                "solver_failure",
                "Realtime initialization failed during internal numerical calculation.",
                status=500,
                details={"reason": str(exc)},
            )
        self._plough_states[session.session_id] = plough_state
        self._cable_names[session.session_id] = setup.cable_name
        self._manufacturer_limits[session.session_id] = setup.manufacturer_limits
        return json_response(
            realtime_result_payload(
                session.latest,
                cable_name=setup.cable_name,
                manufacturer_limits=setup.manufacturer_limits,
            ),
            status=201,
        )

    def _advance_realtime_session(
        self,
        session_id: str,
        payload: dict[str, Any],
    ) -> ApiResponse:
        """将一个公共时基工程包原子推进为新实时结果。"""

        session = self.realtime_sessions.get(session_id)
        if session is None:
            return error_response("unknown_session", "Realtime session was not found.", status=404)
        structure_errors = json_structure_errors("realtime-sensor-packet", payload)
        if structure_errors:
            return _invalid_input("Realtime update does not match the public schema.", structure_errors)
        parsed = _parse_public_packet(
            payload,
            previous_plough_state=self._plough_states[session_id],
            plough_position_mode=self._plough_states[session_id].position_mode,
        )
        if isinstance(parsed, ApiResponse):
            return parsed
        packet, candidate_plough_state = parsed
        if packet.plough.z_m < 0.0 or packet.plough.z_m > packet.water_depth_m:
            return _invalid_input(
                "Realtime update contains an invalid plough depth.",
                {"plough_position.z_m": "must be between 0 and water depth"},
            )
        try:
            result = session.advance(packet)
        except RealtimeSessionError as exc:
            return _realtime_error_response(exc)
        except CableGeometryInfeasibleError as exc:
            return error_response(
                "solver_infeasible",
                "Realtime update has no feasible cable geometry; the previous state is retained.",
                status=422,
                details={"reason": str(exc)},
            )
        except ValueError as exc:
            return error_response(
                "invalid_input",
                "Realtime update values are invalid; the previous state is retained.",
                status=400,
                details={"reason": str(exc)},
            )
        except RuntimeError as exc:
            return error_response(
                "solver_failure",
                "Realtime update failed during internal numerical calculation; the previous state is retained.",
                status=500,
                details={"reason": str(exc)},
            )
        # 只有求解状态提交后才接受位置、时刻和后拖边界。
        self._plough_states[session_id] = candidate_plough_state
        return json_response(
            realtime_result_payload(
                result,
                cable_name=self._cable_names[session_id],
                manufacturer_limits=self._manufacturer_limits[session_id],
            )
        )


def _parse_realtime_setup(payload: dict[str, Any]) -> _RealtimeSetup | ApiResponse:
    cable = payload.get("cable")
    geometry = payload.get("initial_geometry")
    if not isinstance(cable, dict) or not isinstance(geometry, dict):
        return _invalid_input("Realtime initialization groups are invalid.", {"$": "invalid object groups"})

    errors: dict[str, str] = {}
    cable_name = cable.get("name")
    if not isinstance(cable_name, str) or not cable_name.strip():
        errors["cable.name"] = "must be a non-empty string"
    elif len(cable_name) > 128:
        errors["cable.name"] = "must contain at most 128 characters"
    diameter = _positive(cable, "diameter_m", "cable", errors)
    mass_air = _positive(cable, "mass_air_kg_per_m", "cable", errors)
    submerged_weight = _positive(
        cable,
        "submerged_weight_n_per_m",
        "cable",
        errors,
    )
    axial_stiffness = _positive(cable, "axial_stiffness_n", "cable", errors)
    bending_stiffness = _optional_nonnegative(
        cable,
        "bending_stiffness_n_m2",
        "cable",
        errors,
    )
    tangential_drag = _nonnegative(cable, "tangential_drag_coefficient", "cable", errors)
    normal_drag = _nonnegative(cable, "normal_drag_coefficient", "cable", errors)
    suspended_length = _positive(geometry, "initial_suspended_length_m", "initial_geometry", errors)
    plough_position_mode = geometry.get("plough_position_mode")
    if plough_position_mode not in {"measured", "reconstructed"}:
        errors["initial_geometry.plough_position_mode"] = (
            "must be 'measured' or 'reconstructed'"
        )

    manufacturer_payload = payload.get("manufacturer_limits") or {}
    manufacturer_limits = {
        field: _optional_positive(
            manufacturer_payload,
            field,
            "manufacturer_limits",
            errors,
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
    }
    weight_air = 0.0 if mass_air is None else mass_air * _GRAVITY_MPS2
    if errors:
        return _invalid_input("Realtime initialization values are invalid.", errors)

    assert None not in (
        diameter,
        mass_air,
        submerged_weight,
        axial_stiffness,
        tangential_drag,
        normal_drag,
        suspended_length,
    )
    assert isinstance(plough_position_mode, str)
    assert isinstance(cable_name, str)
    return _RealtimeSetup(
        cable_name=cable_name,
        diameter_m=diameter,
        weight_air_n_per_m=weight_air,
        submerged_weight_n_per_m=submerged_weight,
        tangential_drag_coefficient=tangential_drag,
        normal_drag_coefficient=normal_drag,
        axial_stiffness_n=axial_stiffness,
        bending_stiffness_n_m2=(
            0.0 if bending_stiffness is None else bending_stiffness
        ),
        initial_suspended_length_m=suspended_length,
        plough_position_mode=plough_position_mode,
        manufacturer_limits=manufacturer_limits,
    )


def _parse_public_packet(
    value: Any,
    *,
    previous_plough_state: _PloughBoundaryState | None = None,
    plough_position_mode: str,
) -> tuple[RealtimeSensorPacket, _PloughBoundaryState] | ApiResponse:
    """将平台工程量适配为求解器内部同步包。"""

    if not isinstance(value, dict):
        return _invalid_input("Realtime packet must be an object.", {"$": "must be an object"})
    errors: dict[str, str] = {}
    sequence = _integer(value.get("sequence"))
    if sequence is None or sequence < 0:
        errors["sequence"] = "must be a non-negative integer"
    time_s = _number(value.get("time_s"))
    if time_s is None or time_s < 0.0:
        errors["time_s"] = "must be greater than or equal to 0"
    water_depth = _number(value.get("water_depth_m"))
    if water_depth is None or water_depth <= 0.0:
        errors["water_depth_m"] = "must be greater than 0"
    payout_speed = _number(value.get("payout_speed_mps"))
    if payout_speed is None or payout_speed < 0.0:
        errors["payout_speed_mps"] = "must be greater than or equal to 0"
    vessel = _parse_vessel(value.get("vessel"), errors)
    current_x, current_y = _parse_surface_current(value.get("surface_current"), errors)
    measured_top_tension = _optional_number(value.get("measured_top_tension_n"))
    if value.get("measured_top_tension_n") is not None and (
        measured_top_tension is None or measured_top_tension < 0.0
    ):
        errors["measured_top_tension_n"] = "must be greater than or equal to 0"

    position_value = value.get("plough_position")
    measured_position = (
        None
        if position_value is None
        else _parse_position(position_value, "plough_position", errors)
    )
    horizontal_distance = _optional_number(value.get("plough_horizontal_distance_m"))
    if value.get("plough_horizontal_distance_m") is not None and (
        horizontal_distance is None or horizontal_distance < 0.0
    ):
        errors["plough_horizontal_distance_m"] = "must be greater than or equal to 0"
    bearing_deg = _optional_number(value.get("plough_bearing_deg"))
    if value.get("plough_bearing_deg") is not None and bearing_deg is None:
        errors["plough_bearing_deg"] = "must be a finite number"
    inlet_height_above_seabed = _optional_number(
        value.get("plough_inlet_height_above_seabed_m")
    )
    if value.get("plough_inlet_height_above_seabed_m") is not None and (
        inlet_height_above_seabed is None or inlet_height_above_seabed < 0.0
    ):
        errors["plough_inlet_height_above_seabed_m"] = "must be greater than or equal to 0"
    if (
        inlet_height_above_seabed is not None
        and water_depth is not None
        and inlet_height_above_seabed > water_depth
    ):
        errors["plough_inlet_height_above_seabed_m"] = "must not exceed water_depth_m"

    if plough_position_mode == "measured" and measured_position is None:
        errors["plough_position"] = "is required in every packet in measured mode"
    if plough_position_mode == "measured":
        if horizontal_distance is not None:
            errors["plough_horizontal_distance_m"] = "is not used in measured mode"
        if bearing_deg is not None:
            errors["plough_bearing_deg"] = "is not used in measured mode"
    if plough_position_mode == "reconstructed":
        if measured_position is not None:
            errors["plough_position"] = "is not accepted in reconstructed mode"
        if horizontal_distance is None:
            errors["plough_horizontal_distance_m"] = "is required in reconstructed mode"
        if bearing_deg is None:
            errors["plough_bearing_deg"] = "is required in reconstructed mode"
        if inlet_height_above_seabed is None:
            errors["plough_inlet_height_above_seabed_m"] = "is required in reconstructed mode"
    if errors:
        return _invalid_input("Realtime packet values are invalid.", errors)
    assert sequence is not None and time_s is not None and water_depth is not None and payout_speed is not None
    assert vessel is not None and current_x is not None and current_y is not None

    try:
        if measured_position is None:
            assert horizontal_distance is not None
            assert bearing_deg is not None
            assert inlet_height_above_seabed is not None
            layback = horizontal_distance
            depth = water_depth - inlet_height_above_seabed
            bearing_rad = math.radians(bearing_deg)
            offset_x = layback * math.cos(bearing_rad)
            offset_y = layback * math.sin(bearing_rad)
            position_x = vessel.x_m + offset_x
            position_y = vessel.y_m + offset_y
            position_z = depth
            if previous_plough_state is None or time_s <= previous_plough_state.time_s:
                velocity_x = vessel.velocity_x_mps
                velocity_y = vessel.velocity_y_mps
                velocity_z = 0.0
                heading = math.atan2(velocity_y, velocity_x) if math.hypot(
                    velocity_x,
                    velocity_y,
                ) > _SPEED_EPSILON_MPS else (bearing_rad + math.pi)
            else:
                time_step_s = time_s - previous_plough_state.time_s
                velocity_x = (position_x - previous_plough_state.position_x_m) / time_step_s
                velocity_y = (position_y - previous_plough_state.position_y_m) / time_step_s
                velocity_z = (position_z - previous_plough_state.position_z_m) / time_step_s
                heading = math.atan2(velocity_y, velocity_x) if math.hypot(
                    velocity_x,
                    velocity_y,
                ) > _SPEED_EPSILON_MPS else (bearing_rad + math.pi)
        else:
            position_x, position_y, position_z = measured_position
            offset_x = position_x - vessel.x_m
            offset_y = position_y - vessel.y_m
            layback = math.hypot(offset_x, offset_y)
            depth = position_z
            if previous_plough_state is None or time_s <= previous_plough_state.time_s:
                heading = _initial_plough_heading(
                    offset_x,
                    offset_y,
                    vessel.velocity_x_mps,
                    vessel.velocity_y_mps,
                )
                plough_speed = _passive_plough_speed(
                    vessel.velocity_x_mps,
                    vessel.velocity_y_mps,
                    heading,
                )
                velocity_x = plough_speed * math.cos(heading)
                velocity_y = plough_speed * math.sin(heading)
                velocity_z = 0.0
            else:
                time_step_s = time_s - previous_plough_state.time_s
                velocity_x = (position_x - previous_plough_state.position_x_m) / time_step_s
                velocity_y = (position_y - previous_plough_state.position_y_m) / time_step_s
                velocity_z = (position_z - previous_plough_state.position_z_m) / time_step_s
                if math.hypot(velocity_x, velocity_y) > _SPEED_EPSILON_MPS:
                    heading = math.atan2(velocity_y, velocity_x)
                elif layback > _SPEED_EPSILON_MPS:
                    heading = math.atan2(-offset_y, -offset_x)
                else:
                    heading = previous_plough_state.heading_rad
    except PassivePloughDomainError as exc:
        return _invalid_input(
            "Realtime packet is outside the forward passive-tow domain.",
            {
                "vessel.velocity_x_mps": str(exc),
                "vessel.velocity_y_mps": str(exc),
            },
        )

    plough = SynchronizedEndpointSample(
        x_m=position_x,
        y_m=position_y,
        z_m=position_z,
        velocity_x_mps=velocity_x,
        velocity_y_mps=velocity_y,
        velocity_z_mps=velocity_z,
    )
    plough_state = _PloughBoundaryState(
        horizontal_offset_x_m=offset_x,
        horizontal_offset_y_m=offset_y,
        horizontal_layback_m=layback,
        heading_rad=heading,
        depth_m=depth,
        position_x_m=position_x,
        position_y_m=position_y,
        position_z_m=position_z,
        vessel_velocity_x_mps=vessel.velocity_x_mps,
        vessel_velocity_y_mps=vessel.velocity_y_mps,
        time_s=time_s,
        position_mode=plough_position_mode,
    )
    return (
        RealtimeSensorPacket(
            sequence=sequence,
            time_s=time_s,
            water_depth_m=water_depth,
            vessel=vessel,
            plough=plough,
            payout_speed_mps=payout_speed,
            plough_position_source=(
                "measured" if measured_position is not None else "reconstructed"
            ),
            plough_exit_speed_mps=None,
            current_velocity_x_mps=current_x,
            current_velocity_y_mps=current_y,
            measured_top_tension_n=measured_top_tension,
            plough_heading_rad=heading,
            plough_layback_m=layback,
        ),
        plough_state,
    )


def _realtime_case_from_setup(
    setup: _RealtimeSetup,
    initial_packet: RealtimeSensorPacket,
) -> DynamicCaseInput:
    """使用固定数值策略构造求解器初始工况。"""

    vessel_speed = math.hypot(
        initial_packet.vessel.velocity_x_mps,
        initial_packet.vessel.velocity_y_mps,
    )
    plough_speed = math.hypot(
        initial_packet.plough.velocity_x_mps,
        initial_packet.plough.velocity_y_mps,
    )
    current_speed = math.hypot(
        initial_packet.current_velocity_x_mps,
        initial_packet.current_velocity_y_mps,
    )
    vessel_heading = _heading(
        initial_packet.vessel.velocity_x_mps,
        initial_packet.vessel.velocity_y_mps,
    )
    plough_heading = (
        _heading(
            initial_packet.plough.velocity_x_mps,
            initial_packet.plough.velocity_y_mps,
        )
        if plough_speed > _SPEED_EPSILON_MPS
        else _heading(
            initial_packet.vessel.x_m - initial_packet.plough.x_m,
            initial_packet.vessel.y_m - initial_packet.plough.y_m,
        )
    )
    return DynamicCaseInput(
        case_name="realtime-session",
        diameter_m=setup.diameter_m,
        weight_air_n_per_m=setup.weight_air_n_per_m,
        submerged_weight_n_per_m=setup.submerged_weight_n_per_m,
        tangential_drag_coefficient=setup.tangential_drag_coefficient,
        normal_drag_coefficient=setup.normal_drag_coefficient,
        axial_stiffness_n=setup.axial_stiffness_n,
        bending_stiffness_n_m2=setup.bending_stiffness_n_m2,
        current_speed_mps=current_speed,
        current_bottom_speed_mps=_REALTIME_CURRENT_BOTTOM_SPEED_MPS,
        current_profile_exponent=_REALTIME_CURRENT_PROFILE_EXPONENT,
        current_direction_deg=_heading(
            initial_packet.current_velocity_x_mps,
            initial_packet.current_velocity_y_mps,
        ),
        speed_change="steady",
        vessel_initial_speed_mps=vessel_speed,
        vessel_final_speed_mps=vessel_speed,
        payout_initial_speed_mps=initial_packet.payout_speed_mps,
        payout_final_speed_mps=initial_packet.payout_speed_mps,
        transition_duration_s=1.0,
        total_duration_s=1.0,
        water_depth_m=initial_packet.water_depth_m,
        element_count=_REALTIME_ELEMENT_COUNT,
        length_boundary_source="known_plough_trajectory",
        vessel_initial_x_m=initial_packet.vessel.x_m,
        vessel_initial_y_m=initial_packet.vessel.y_m,
        vessel_heading_deg=vessel_heading,
        plough_initial_x_m=initial_packet.plough.x_m,
        plough_initial_y_m=initial_packet.plough.y_m,
        plough_initial_z_m=initial_packet.plough.z_m,
        plough_speed_mps=plough_speed,
        plough_exit_speed_mps=None,
        plough_heading_deg=plough_heading,
        initial_suspended_length_m=setup.initial_suspended_length_m,
        # 厂家 MBR 仅作为外部参考回显；实时求解器不据此投影或拒绝缆型。
        min_bending_radius_m=None,
        integration_time_step_max_s=_REALTIME_INTEGRATION_TIME_STEP_MAX_S,
    )


def _initial_plough_heading(
    offset_x_m: float,
    offset_y_m: float,
    vessel_velocity_x_mps: float,
    vessel_velocity_y_mps: float,
) -> float:
    """由后拖几何确定犁艏向；零后拖时退回船速方向。"""

    if math.hypot(offset_x_m, offset_y_m) > _SPEED_EPSILON_MPS:
        return math.atan2(-offset_y_m, -offset_x_m)
    if math.hypot(vessel_velocity_x_mps, vessel_velocity_y_mps) > _SPEED_EPSILON_MPS:
        return math.atan2(vessel_velocity_y_mps, vessel_velocity_x_mps)
    return 0.0


def _passive_plough_speed(
    vessel_velocity_x_mps: float,
    vessel_velocity_y_mps: float,
    plough_heading_rad: float,
) -> float:
    """返回前向被动拖曳适用域内的犁速。"""

    return passive_plough_forward_speed(
        vessel_velocity_x_mps,
        vessel_velocity_y_mps,
        plough_heading_rad,
    )


def _advance_passive_plough_heading(
    previous: _PloughBoundaryState,
    vessel_velocity_x_mps: float,
    vessel_velocity_y_mps: float,
    time_s: float,
) -> float:
    """按无横向滑移拖车方程推进犁艏向。"""

    samples = passive_plough_kinematic_samples(
        start_time_s=previous.time_s,
        end_time_s=time_s,
        layback_m=previous.horizontal_layback_m,
        initial_heading_rad=previous.heading_rad,
        start_vessel_velocity_x_mps=previous.vessel_velocity_x_mps,
        start_vessel_velocity_y_mps=previous.vessel_velocity_y_mps,
        end_vessel_velocity_x_mps=vessel_velocity_x_mps,
        end_vessel_velocity_y_mps=vessel_velocity_y_mps,
    )
    return samples[-1].heading_rad


def _parse_vessel(value: Any, errors: dict[str, str]) -> SynchronizedEndpointSample | None:
    if not isinstance(value, dict):
        errors["vessel"] = "must be an object"
        return None
    names = (
        "x_m",
        "y_m",
        "z_m",
        "velocity_x_mps",
        "velocity_y_mps",
        "velocity_z_mps",
    )
    parsed = {name: _number(value.get(name)) for name in names}
    for name, number in parsed.items():
        if number is None:
            errors[f"vessel.{name}"] = "must be a finite number"
    if any(number is None for number in parsed.values()):
        return None
    return SynchronizedEndpointSample(**parsed)  # type: ignore[arg-type]


def _parse_surface_current(
    value: Any,
    errors: dict[str, str],
) -> tuple[float | None, float | None]:
    if not isinstance(value, dict):
        errors["surface_current"] = "must be an object"
        return None, None
    x_mps = _number(value.get("velocity_x_mps"))
    y_mps = _number(value.get("velocity_y_mps"))
    if x_mps is None:
        errors["surface_current.velocity_x_mps"] = "must be a finite number"
    if y_mps is None:
        errors["surface_current.velocity_y_mps"] = "must be a finite number"
    return x_mps, y_mps


def _parse_position(
    value: Any,
    field: str,
    errors: dict[str, str],
) -> tuple[float, float, float] | None:
    if not isinstance(value, dict):
        errors[field] = "must be an object"
        return None
    parsed = tuple(_number(value.get(name)) for name in ("x_m", "y_m", "z_m"))
    for name, number in zip(("x_m", "y_m", "z_m"), parsed):
        if number is None:
            errors[f"{field}.{name}"] = "must be a finite number"
    return None if any(number is None for number in parsed) else parsed  # type: ignore[return-value]


def _positive(
    values: dict[str, Any],
    name: str,
    group: str,
    errors: dict[str, str],
) -> float | None:
    value = _number(values.get(name))
    if value is None or value <= 0.0:
        errors[f"{group}.{name}"] = "must be a positive finite number"
        return None
    return value


def _nonnegative(
    values: dict[str, Any],
    name: str,
    group: str,
    errors: dict[str, str],
) -> float | None:
    value = _number(values.get(name))
    if value is None or value < 0.0:
        errors[f"{group}.{name}"] = "must be a non-negative finite number"
        return None
    return value


def _optional_positive(
    values: dict[str, Any],
    name: str,
    group: str,
    errors: dict[str, str],
) -> float | None:
    raw = values.get(name)
    if raw is None:
        return None
    value = _number(raw)
    if value is None or value <= 0.0:
        errors[f"{group}.{name}"] = "must be a positive finite number"
        return None
    return value


def _optional_nonnegative(
    values: dict[str, Any],
    name: str,
    group: str,
    errors: dict[str, str],
) -> float | None:
    raw = values.get(name)
    if raw is None:
        return None
    value = _number(raw)
    if value is None or value < 0.0:
        errors[f"{group}.{name}"] = "must be a non-negative finite number"
        return None
    return value


def _number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _optional_number(value: Any) -> float | None:
    return None if value is None else _number(value)


def _integer(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return None


def _heading(x_mps: float, y_mps: float) -> float:
    return (
        math.degrees(math.atan2(y_mps, x_mps)) % 360.0
        if math.hypot(x_mps, y_mps) > 1.0e-12
        else 0.0
    )


def _invalid_input(message: str, fields: dict[str, str]) -> ApiResponse:
    return error_response(
        "invalid_input",
        message,
        status=400,
        details={"fields": fields},
    )


def _realtime_error_response(exc: RealtimeSessionError) -> ApiResponse:
    status = {
        "sequence_conflict": 409,
        "non_monotonic_time": 409,
        "session_busy": 409,
        "sensor_gap": 422,
        "invalid_packet": 400,
        "invalid_time_step": 400,
    }.get(exc.code, 400)
    return error_response(exc.code, str(exc), status=status)


def create_app() -> CableApiServer:
    """创建可测试的 API 路由器。"""

    return CableApiServer()


def run_server(
    host: str = "127.0.0.1",
    port: int = 8765,
) -> None:
    """运行本地 HTTP 服务。"""

    app = create_app()
    server = ThreadingHTTPServer((host, port), create_http_handler(app))
    print(f"cable tension API: http://{host}:{port}")
    server.serve_forever()


def create_http_handler(app: CableApiServer) -> type[BaseHTTPRequestHandler]:
    """创建带 CORS 和结构化错误的标准库 HTTP 处理器。"""

    class Handler(BaseHTTPRequestHandler):
        def __getattr__(self, name: str) -> Any:
            """将所有未公开 HTTP 方法统一交给版本化路由器拒绝。"""

            if name.startswith("do_"):
                return self._reject_unpublished_method
            raise AttributeError(name)

        def do_OPTIONS(self) -> None:  # noqa: N802
            self._send(app.handle("OPTIONS", self.path))

        def do_GET(self) -> None:  # noqa: N802
            self._send(app.handle("GET", self.path))

        def do_POST(self) -> None:  # noqa: N802
            payload = self._read_json_body()
            if payload is None:
                self._send(error_response("invalid_json", "Request body must be valid JSON.", status=400))
                return
            self._send(app.handle("POST", self.path, payload))

        def _reject_unpublished_method(self) -> None:
            self._send(app.handle(self.command, self.path))

        def send_error(  # noqa: D102
            self,
            code: int,
            message: str | None = None,
            explain: str | None = None,
        ) -> None:
            if _is_versioned_api_path(urlsplit(self.path).path):
                default_message = self.responses.get(code, ("HTTP error", ""))[0]
                self._send(error_response("http_error", message or default_message, status=code))
                return
            super().send_error(code, message, explain)

        def log_message(self, format: str, *args: Any) -> None:
            sys.stderr.write("[cable-api] " + format % args + "\n")

        def _read_json_body(self) -> dict[str, Any] | None:
            length = int(self.headers.get("Content-Length", "0"))
            if length == 0:
                return {}
            try:
                parsed = json.loads(self.rfile.read(length).decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                return None
            return parsed if isinstance(parsed, dict) else None

        def _send(self, response: ApiResponse) -> None:
            self.send_response(response.status)
            headers = {
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "GET, POST, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
                "Access-Control-Expose-Headers": API_VERSION_HEADER,
                "Content-Length": str(len(response.body)),
                **response.headers,
            }
            if _is_versioned_api_path(urlsplit(self.path).path):
                headers[API_VERSION_HEADER] = API_VERSION
            for name, value in headers.items():
                self.send_header(name, value)
            self.end_headers()
            if response.body:
                self.wfile.write(response.body)

    return Handler


def _normalize_api_path(path: str) -> tuple[bool, str]:
    if path == "/api/v1":
        return True, "/api"
    if path.startswith("/api/v1/"):
        return True, "/api/" + path.removeprefix("/api/v1/")
    return False, path


def _is_versioned_api_path(path: str) -> bool:
    return path == "/api/v1" or path.startswith("/api/v1/")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the cable tension backend API.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    run_server(host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
