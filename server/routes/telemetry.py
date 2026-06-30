"""GET /api/telemetry/latest — snapshot of all telemetry fields.

Read-only; not auth-protected so dashboards / health checks can poll cheaply.
"""
from __future__ import annotations

import math

from fastapi import APIRouter, Depends
from fastapi.responses import JSONResponse

from auth import require_token
from config import GPS_FIX_NAMES, RPP_STATE_NAMES
from models import MissionState, TelemetryData
from spray_safety import build_spray_telemetry_fields

router = APIRouter(prefix="/telemetry", tags=["telemetry"], dependencies=[Depends(require_token)])


def _sanitize(obj):
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, float) and (math.isnan(obj) or math.isinf(obj)):
        return None
    return obj


@router.get("/latest", response_model=TelemetryData)
async def telemetry_latest():
    from main import offboard_ctrl, ros_node
    if ros_node is None:
        return JSONResponse(_sanitize(TelemetryData().model_dump()))
    s = ros_node.get_state()
    code = s.get("rpp_state", 0)
    legacy_spraying = bool(s.get("spraying", False))
    spray_rt = ros_node.get_spray_runtime_status()
    mission_running = (
        offboard_ctrl is not None
        and offboard_ctrl.state == MissionState.RUNNING
        and bool(s.get("armed", False))
    )
    mission_dash = None
    if offboard_ctrl is not None and hasattr(offboard_ctrl, "loaded_path_summary"):
        summary = offboard_ctrl.loaded_path_summary()
        mission_dash = {
            "dash_feasible": summary.get("dash_feasible"),
            "dash_feasibility_reason": summary.get("dash_feasibility_reason"),
            "shortest_dash_on_run_m": summary.get("shortest_dash_on_run_m"),
            "shortest_dash_off_gap_m": summary.get("shortest_dash_off_gap_m"),
            "dash_phase_reset": summary.get("dash_phase_reset"),
            "dash_expected_speed_mps": summary.get("dash_expected_speed_mps"),
            "dash_feasibility_speed_source": summary.get(
                "dash_feasibility_speed_source"
            ),
        }
    spray_fields = build_spray_telemetry_fields(
        legacy_spraying=legacy_spraying,
        spray_rt=spray_rt,
        mission_running=mission_running,
        mission_dash=mission_dash,
    )
    data = TelemetryData(
        pos_n           = s.get("pos_n"),
        pos_e           = s.get("pos_e"),
        heading_ned_deg = s.get("heading_ned_deg"),
        xtrack_m        = s.get("xtrack_m"),
        heading_err_deg = s.get("heading_err_deg"),
        lookahead_m     = s.get("lookahead_m"),
        speed_m_s       = s.get("speed_m_s"),
        kappa           = s.get("kappa"),
        dist_to_goal_m  = s.get("dist_to_goal_m"),
        pose_age_ms     = s.get("pose_age_ms"),
        rpp_state       = code,
        rpp_state_name  = RPP_STATE_NAMES.get(code, "UNKNOWN"),
        rpp_debug_age_ms = s.get("rpp_debug_age_ms"),
        rpp_debug_fresh = s.get("rpp_debug_fresh"),
        measured_speed_m_s = s.get("measured_speed_m_s"),
        armed           = s.get("armed"),
        mode            = s.get("mode"),
        connected       = s.get("connected"),
        battery_v       = s.get("battery_v"),
        battery_pct     = s.get("battery_pct"),
        gps_fix         = s.get("gps_fix"),
        gps_fix_name    = GPS_FIX_NAMES.get(s.get("gps_fix", 0), "UNKNOWN"),
        gps_sat         = s.get("gps_sat"),
        hrms            = s.get("hrms"),
        vrms            = s.get("vrms"),
        lat             = s.get("lat"),
        lon             = s.get("lon"),
        alt             = s.get("alt"),
        **spray_fields,
    )
    return JSONResponse(_sanitize(data.model_dump()))
