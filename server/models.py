"""Pydantic request / response models."""

from __future__ import annotations

from enum import Enum
from typing import Any, Literal, Optional, Union

from pydantic import BaseModel, Field, field_validator


class VehicleMode(str, Enum):
    MANUAL = "MANUAL"
    OFFBOARD = "OFFBOARD"


class MissionState(str, Enum):
    IDLE = "idle"
    LOADING = "loading"
    ARMING = "arming"
    SWITCHING_OFFBOARD = "switching_offboard"
    RUNNING = "running"
    STOPPING = "stopping"
    DISARMING = "disarming"
    COMPLETED = "completed"
    ABORTED = "aborted"
    ERROR = "error"


# ── Request bodies ────────────────────────────────────────────────────────────


class ArmRequest(BaseModel):
    arm: bool


class ModeRequest(BaseModel):
    mode: VehicleMode


class PathPublishRequest(BaseModel):
    name: Optional[str] = None
    file: Optional[str] = None
    frame_id: str = "local_ned"


class MissionStartRequest(BaseModel):
    path_name: Optional[str] = None
    mission_file: Optional[str] = None
    mission_id: Optional[str] = None
    auto_origin: bool = False


class MissionLoadRequest(BaseModel):
    path_name: Optional[str] = None
    mission_file: Optional[str] = None


class JoystickAcquireRequest(BaseModel):
    type: Literal["joystick_acquire"] = "joystick_acquire"
    session_id: str
    client_monotonic_ms: int


class JoystickCommandRequest(BaseModel):
    type: Literal["joystick_command"] = "joystick_command"
    session_id: str
    lease_id: str
    sequence: int
    client_monotonic_ms: int
    deadman: bool = False
    throttle: float = Field(..., ge=-1.0, le=1.0)
    steering: float = Field(..., ge=-1.0, le=1.0)

    @field_validator("throttle", "steering")
    @classmethod
    def finite_axis(cls, value: float) -> float:
        import math

        if not math.isfinite(value):
            raise ValueError("axis value must be finite")
        return value


class JoystickReleaseRequest(BaseModel):
    type: Literal["joystick_release"] = "joystick_release"
    session_id: str
    lease_id: str


class SprayTestRequest(BaseModel):
    on: bool
    # Seconds to hold manual spray ON before server-side auto-off.
    # Clamped to MAX_SPRAY_TEST_DURATION_S; the node's
    # manual_override_timeout_s is the hard backstop.
    duration_s: Optional[float] = None
    # Bench-test ON requires explicit operator diagnostic authorization.
    diagnostic_authorized: bool = False


class SprayModeConfig(BaseModel):
    """Mission-bound spray configuration staged with each mission."""

    spray_mode: Literal["continuous", "dash", "point"] = "continuous"
    dash_on_distance_m: float = 0.30
    dash_off_distance_m: float = 0.30
    dash_phase_reset: Literal["per_mark_region", "continuous"] = "per_mark_region"
    point_default_dwell_s: float = 2.0
    point_max_dwell_s: float = 60.0
    point_arrival_tolerance_m: float = 0.05
    point_settle_time_s: float = 0.10
    point_leg_timeout_s: float = 120.0
    point_settle_speed_mps: float = 0.05
    point_settle_yaw_rate_rad_s: float = 0.05
    point_execution_mode: Literal["auto", "manual"] = "auto"
    point_leg_trajectory_mode: Literal["two_point", "densified"] = "two_point"
    point_leg_spacing_m: float = 0.08
    point_hold_drift_tolerance_m: float = 0.08
    point_hold_drift_policy: Literal["fail", "pause"] = "fail"
    point_mission_points: list[dict[str, Any]] = Field(default_factory=list)
    point_source_frame: Literal["LOCAL_NED", "GPS_SURVEYED", "DESIGN"] = "LOCAL_NED"
    gps_required_fix_type: int = 6
    gps_global_position_max_age_ms: float = 500.0
    gps_local_pose_max_age_ms: float = 500.0
    gps_fix_max_age_ms: float = 500.0
    gps_max_pose_global_skew_ms: float = 100.0
    gps_runtime_policy: Literal["pause", "fail"] = "pause"
    gps_resume_policy: Literal["manual", "auto"] = "manual"
    gps_recovery_stable_s: float = 2.0
    obstacle_integration_enabled: bool = False
    obstacle_signal_max_age_s: float = 2.0
    max_spray_speed_mps: float = 1.0
    unsafe_speed_behavior: Literal["BLOCK_SPRAY", "CLAMP_PWM"] = "BLOCK_SPRAY"
    max_along_track_heading_error_deg: float = 30.0
    max_cross_track_speed_mps: float = 0.10
    max_reverse_speed_tolerance_mps: float = 0.03
    max_projection_jump_m: float = 0.50
    max_backward_projection_jump_m: float = 0.10
    projection_ambiguity_distance_m: float = 0.03
    max_lead_distance_m: float = 0.50
    min_on_distance_m: float = 0.05
    min_off_distance_m: float = 0.05
    flow_mode: Literal["fixed", "mapped", "disabled"] = "mapped"
    min_target_flow: float = 0.0
    max_target_flow: float = 1.0
    max_pwm_change_per_s: float = 500.0
    low_speed_anti_puddle_behavior: Literal["block"] = "block"
    high_speed_underflow_behavior: Literal["block", "clamp"] = "block"
    calibration_profile_id: str = "factory_default"
    calibration_profile_version: int = 1
    target_paint_density: float = 1.0
    speed_pwm_table: list[dict[str, float]] = Field(
        default_factory=lambda: [
            {"speed_mps": 0.05, "pwm": 1200.0},
            {"speed_mps": 0.35, "pwm": 1800.0},
        ]
    )
    actuator_min_pwm: float = 0.0
    actuator_max_pwm: float = 2200.0
    actuator_off_pwm: float = 0.0
    actuator_min_value: float = -1.0
    actuator_max_value: float = 1.0
    actuator_off_value: float = -1.0
    timing_only_compatibility: bool = False


class ContinuousModeRequest(BaseModel):
    """Continuous mode — controller-owned timing, flow, and safety parameters."""

    solenoid_open_delay_s: float = Field(0.10, ge=0.0)
    solenoid_close_delay_s: float = Field(0.05, ge=0.0)
    on_overspray_margin_m: float = Field(0.02, ge=0.0)
    off_overspray_margin_m: float = Field(0.0, ge=0.0)
    min_spray_speed_mps: float = Field(0.05, ge=0.0)
    max_spray_speed_mps: float = Field(1.0, gt=0.0)
    unsafe_speed_behavior: Literal["BLOCK_SPRAY", "CLAMP_PWM"] = "BLOCK_SPRAY"
    max_xtrack_error_m: float = Field(0.10, gt=0.0)
    nozzle_forward_offset_m: float = 0.0
    nozzle_lateral_offset_m: float = 0.0
    max_along_track_heading_error_deg: float = Field(30.0, gt=0.0, le=180.0)
    max_cross_track_speed_mps: float = Field(0.10, ge=0.0)
    max_reverse_speed_tolerance_mps: float = Field(0.03, ge=0.0)
    max_projection_jump_m: float = Field(0.50, ge=0.0)
    max_backward_projection_jump_m: float = Field(0.10, ge=0.0)
    projection_ambiguity_distance_m: float = Field(0.03, ge=0.0)
    max_lead_distance_m: float = Field(0.50, ge=0.0)
    min_on_distance_m: float = Field(0.05, ge=0.0)
    min_off_distance_m: float = Field(0.05, ge=0.0)
    flow_mode: Literal["fixed", "mapped", "disabled"] = "mapped"
    min_target_flow: float = Field(0.0, ge=0.0)
    max_target_flow: float = Field(1.0, gt=0.0)
    max_pwm_change_per_s: float = Field(500.0, ge=0.0)
    low_speed_anti_puddle_behavior: Literal["block"] = "block"
    high_speed_underflow_behavior: Literal["block", "clamp"] = "block"
    calibration_profile_id: str = "factory_default"
    calibration_profile_version: int = Field(1, ge=1)
    target_paint_density: float = Field(1.0, gt=0.0)
    speed_pwm_table: list[dict[str, float]] = Field(
        default_factory=lambda: [
            {"speed_mps": 0.05, "pwm": 1200.0},
            {"speed_mps": 0.35, "pwm": 1800.0},
        ]
    )
    actuator_min_pwm: float = Field(0.0, ge=0.0)
    actuator_max_pwm: float = Field(2200.0, gt=0.0)
    actuator_off_pwm: float = Field(0.0, ge=0.0)
    actuator_min_value: float = -1.0
    actuator_max_value: float = 1.0
    actuator_off_value: float = -1.0
    timing_only_compatibility: bool = False


class DashModeRequest(BaseModel):
    """Dash mode — on/off distance interval parameters."""

    dash_on_distance_m: float = Field(0.30, ge=0.0)
    dash_off_distance_m: float = Field(0.30, ge=0.0)
    dash_phase_reset: Literal["per_mark_region", "continuous"] = "per_mark_region"


class PointModeRequest(BaseModel):
    """Point mode — navigation and dwell parameters."""

    point_default_dwell_s: float = Field(2.0, gt=0.0)
    point_max_dwell_s: float = Field(60.0, gt=0.0)
    point_arrival_tolerance_m: float = Field(0.05, gt=0.0)
    point_settle_time_s: float = Field(0.10, ge=0.0)
    point_leg_timeout_s: float = Field(120.0, gt=0.0)
    point_settle_speed_mps: float = Field(0.05, ge=0.0)
    point_settle_yaw_rate_rad_s: float = Field(0.05, ge=0.0)
    point_execution_mode: Literal["auto", "manual"] = "auto"
    point_leg_trajectory_mode: Literal["two_point", "densified"] = "two_point"
    point_leg_spacing_m: float = Field(0.08, gt=0.0)
    point_hold_drift_tolerance_m: float = Field(0.08, gt=0.0)
    point_hold_drift_policy: Literal["fail", "pause"] = "fail"


class SprayModeResponse(BaseModel):
    """Response for spray mode read/write endpoints."""

    name: str
    spray_mode: str
    config: dict
    has_sidecar: bool
    # Whether the saved config was pushed to the live spray controller now
    # (true only when this path's mission is the loaded, idle one). When false,
    # apply_detail explains when it will take effect (next mission load).
    applied: bool = False
    apply_detail: str = ""


class ParamSetRequest(BaseModel):
    # PX4 has int (SYS_AUTOSTART), float (RO_YAW_RATE_P), and bool params.
    value: Union[bool, int, float, str]


# ── Response / payload models ─────────────────────────────────────────────────


class TelemetryData(BaseModel):
    # Position (NED metres)
    pos_n: Optional[float] = None
    pos_e: Optional[float] = None
    heading_ned_deg: Optional[float] = None
    # RPP diagnostics
    xtrack_m: Optional[float] = None
    heading_err_deg: Optional[float] = None
    lookahead_m: Optional[float] = None
    speed_m_s: Optional[float] = None
    kappa: Optional[float] = None
    dist_to_goal_m: Optional[float] = None
    pose_age_ms: Optional[float] = None
    rpp_state: Optional[Literal[-1, 0, 1, 2, 3, 4, 5]] = None
    rpp_state_name: Optional[str] = None
    rpp_debug_age_ms: Optional[float] = None
    rpp_debug_fresh: Optional[bool] = None
    measured_speed_m_s: Optional[float] = None
    spraying: Optional[bool] = None
    marking_state: Optional[Literal["marking", "transit", "off"]] = None
    # Spray actuator truth (Task_17): `spraying` is MAVROS-command-accepted ON
    # only — not confirmed physical paint flow. Stale runtime status suppresses
    # both `spraying` and `marking_state="marking"`.
    commanded_on: Optional[bool] = None
    confirmed_off: Optional[bool] = None
    spray_state: Optional[str] = None
    desired_on: Optional[bool] = None
    pending_command: Optional[bool] = None
    accepted_command_on: Optional[bool] = None
    accepted_command_off: Optional[bool] = None
    off_acknowledged: Optional[bool] = None
    physical_feedback_supported: Optional[bool] = None
    physical_actuator_state: Optional[str] = None
    spray_runtime_status_age_s: Optional[float] = None
    spray_faulted: Optional[bool] = None
    spray_recovery_required: Optional[bool] = None
    last_spray_command_result: Optional[str] = None
    last_spray_command_reason: Optional[str] = None
    spray_safety_reason: Optional[str] = None
    along_track_speed_mps: Optional[float] = None
    projection_ambiguous: Optional[bool] = None
    flow_under_capacity: Optional[bool] = None
    geometry_hash: Optional[str] = None
    runtime_spray_geometry_hash: Optional[str] = None
    gps_safety_ok: Optional[bool] = None
    manual_resume_required: Optional[bool] = None
    pending_value: Optional[float] = None
    physical_confirmation_available: Optional[bool] = None
    physical_feedback_stale: Optional[bool] = None
    physical_feedback_age_s: Optional[float] = None
    physical_off_confirmed: Optional[bool] = None
    last_command_error: Optional[str] = None
    projection_s: Optional[float] = None
    projection_segment_index: Optional[int] = None
    projection_xtrack_error_m: Optional[float] = None
    projection_jump_m: Optional[float] = None
    ambiguity_clearance_confidence: Optional[float] = None
    cross_track_speed_mps: Optional[float] = None
    velocity_heading_error_deg: Optional[float] = None
    current_run_length_m: Optional[float] = None
    next_run_length_m: Optional[float] = None
    next_boundary_kind: Optional[str] = None
    distance_to_boundary_m: Optional[float] = None
    raw_on_lead_m: Optional[float] = None
    bounded_on_lead_m: Optional[float] = None
    raw_off_lead_m: Optional[float] = None
    bounded_off_lead_m: Optional[float] = None
    lead_clamped: Optional[bool] = None
    lead_block_reason: Optional[str] = None
    flow_mode: Optional[str] = None
    target_flow: Optional[float] = None
    target_pwm: Optional[float] = None
    command_pwm: Optional[float] = None
    pwm_ramp_limited: Optional[bool] = None
    flow_capacity_limited: Optional[bool] = None
    flow_clamp_reason: Optional[str] = None
    dash_feasible: Optional[bool] = None
    dash_feasibility_reason: Optional[str] = None
    shortest_dash_on_run_m: Optional[float] = None
    shortest_dash_off_gap_m: Optional[float] = None
    dash_phase_reset: Optional[str] = None
    dash_expected_speed_mps: Optional[float] = None
    dash_feasibility_speed_source: Optional[str] = None
    vehicle_state_age_s: Optional[float] = None
    vehicle_state_stale: Optional[bool] = None
    vehicle_state_block_reason: Optional[str] = None
    geometry_spray_request: Optional[bool] = None
    dry_run_active: Optional[bool] = None
    # FCU
    armed: Optional[bool] = None
    mode: Optional[str] = None
    connected: Optional[bool] = None
    # Battery
    battery_v: Optional[float] = None
    battery_pct: Optional[float] = None
    # GPS
    gps_fix: Optional[int] = None
    gps_fix_name: Optional[str] = None
    gps_sat: Optional[int] = None
    hrms: Optional[float] = None
    vrms: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None


class PathInfo(BaseModel):
    name: str
    description: str
    num_points: int
    source: str  # "builtin" | "file"


class PathPreviewPoint(BaseModel):
    north: float
    east: float
    spray: bool = True


class PathPreviewBounds(BaseModel):
    north_min: float
    north_max: float
    east_min: float
    east_max: float


class PathPreviewResponse(BaseModel):
    name: str
    frame: str = "local_ned"
    num_points: int
    bounds: Optional[PathPreviewBounds] = None
    waypoints: list[PathPreviewPoint]


class PointMissionStatusResponse(BaseModel):
    """Documented Point Mode runtime diagnostics."""

    point_mission_state: str = "idle"
    point_mission_id: str = ""
    point_mission_generation: int = 0
    current_point_index: int = 0
    total_points: int = 0
    point_execution_mode: str = "auto"
    waiting_for_continue: bool = False
    last_completed_point_index: Optional[int] = None
    next_point_index: Optional[int] = None
    target_north_m: Optional[float] = None
    target_east_m: Optional[float] = None
    current_distance_m: Optional[float] = None
    arrival_met: bool = False
    settle_met: bool = False
    mark_enabled: bool = True
    active_dwell: bool = False
    active_dwell_command_id: Optional[int] = None
    parent_mission_id: str = ""
    point_mission_generation: int = 0
    active_dwell_command_revision: Optional[int] = None
    active_dwell_configuration_revision: Optional[int] = None
    active_dwell_point_index: Optional[int] = None
    active_dwell_source_index: Optional[int] = None
    recovery_required: bool = False
    terminal_failure_reason: str = ""
    dwell_cancel_result: Optional[dict] = None
    spray_off_result: Optional[dict] = None
    dwell_remaining_s: float = 0.0
    last_failure_reason: str = ""
    last_transition: str = ""
    ready: bool = False
    source_frame: str = ""
    resolved_runtime_frame: str = ""
    run_active: bool = False
    obstacle_clear: bool = True
    obstacle_integration_enabled: bool = False
    obstacle_signal_state: str = "not_configured"
    obstacle_signal_age_ms: Optional[float] = None
    terminal_safety_ok: bool = True
    terminal_safety_reason: str = ""
    pause_reason: str = ""
    pre_pause_state: str = ""
    paused_point_index: Optional[int] = None
    resume_available: bool = False
    dwell_cancelled: bool = False
    dwell_ownership_invalidated: bool = False
    setpoint_source: str = "rpp"
    hold_active: bool = False
    hold_north_m: Optional[float] = None
    hold_east_m: Optional[float] = None
    hold_heading_ned_rad: Optional[float] = None
    hold_error_m: Optional[float] = None
    gps_safety_state: str = "not_applicable"
    gps_safety_ok: bool = True
    gps_required_fix_type: Optional[int] = None
    gps_current_fix_type: Optional[int] = None
    gps_global_position_age_ms: Optional[float] = None
    gps_local_pose_age_ms: Optional[float] = None
    gps_fix_age_ms: Optional[float] = None
    gps_pose_global_skew_ms: Optional[float] = None
    gps_anchor_valid: Optional[bool] = None
    gps_last_safety_reason: str = ""
    gps_fault_count: int = 0
    gps_last_fault_time_s: Optional[float] = None
    gps_recovery_ready: bool = False
    gps_runtime_policy: Optional[str] = None
    gps_resume_policy: Optional[str] = None
    point_leg_trajectory_mode: str = "two_point"
    point_leg_spacing_m: float = 0.08
    point_leg_published_count: Optional[int] = None
    point_leg_conditioned_count: Optional[int] = None
    active_trajectory_mode: Optional[str] = None
    point_leg_length_m: Optional[float] = None
    last_skipped_point_index: Optional[int] = None
    skipped_point_indices: list[int] = Field(default_factory=list)
    skip_pending: bool = False


class MissionResumeRequest(BaseModel):
    expected_generation: Optional[int] = None


class ObstacleStatusRequest(BaseModel):
    clear: bool


class PointPauseResponse(BaseModel):
    paused: bool
    message: str
    status: PointMissionStatusResponse


class PointResumeResponse(BaseModel):
    resumed: bool
    message: str
    status: PointMissionStatusResponse


class PointContinueResponse(BaseModel):
    continued: bool
    message: str
    status: PointMissionStatusResponse


class ObstacleStatusResponse(BaseModel):
    obstacle_clear: bool
    status: PointMissionStatusResponse


class PointSkipRequest(BaseModel):
    point_index: int
    expected_generation: Optional[int] = None
    reason: str = "operator_skip"


class PointSkipResponse(BaseModel):
    skipped: bool
    message: str
    status: PointMissionStatusResponse


class MissionRestartRequest(BaseModel):
    mission_id: str
    stop_first: bool = False
    start_after_reset: bool = False
    auto_origin: bool = False


class MissionRestartResponse(BaseModel):
    success: bool
    restarted: bool
    reset: bool
    started: bool
    state: str
    mission_id: str
    point_mission_generation: Optional[int] = None
    message: str
    stop_result: Optional[dict] = None
    start_result: Optional[dict] = None


class PointMissionEvent(BaseModel):
    event_id: int = 0
    ts: str
    timestamp: str = ""           # mobile alias for ts (set equal to ts on build)
    event_type: Literal[
        "point_leg_started",
        "point_arrived",
        "point_dwell_started",
        "point_marked",
        "point_waiting_for_continue",
        "point_paused",
        "point_resumed",
        "point_skipped",
        "point_completed",
        "point_failed",
        "point_aborted",
    ]
    mission_id: str
    parent_mission_id: str
    point_mission_generation: int
    generation: int = 0           # mobile alias for point_mission_generation
    point_index: int = 0          # always set (build falls back to current_point_index)
    source_index: Optional[int] = None
    point_mission_state: str
    mark: Optional[bool] = None
    dwell_command_id: Optional[int] = None
    dwell_remaining_s: Optional[float] = None
    hold_active: bool
    obstacle_signal_state: str
    gps_safety_state: str
    terminal: bool
    reason: str
    message: str = ""             # human-readable summary (mobile displays this)
    lat: Optional[float] = None   # target point geodetic (optional; None when no anchor)
    lon: Optional[float] = None
    status: dict


class PointEventHistoryResponse(BaseModel):
    events: list[PointMissionEvent]
    latest_event_id: int
    last_event_id: int = 0        # mobile alias for latest_event_id
    history_evicted: bool
    oldest_available_event_id: Optional[int] = None


# ── Verified GPS mission models ───────────────────────────────────────────────


class VerifiedWaypointInput(BaseModel):
    index: int = Field(..., ge=0)
    lat: float
    lon: float
    alt: float = 0.0
    mark: bool
    dwell_s: Optional[float] = None
    block: Optional[str] = None
    row: Optional[str] = None
    pile: Optional[str] = None
    label: Optional[str] = None

    @field_validator("lat", "lon", "alt", "dwell_s")
    @classmethod
    def finite_coord(cls, value: Optional[float]) -> Optional[float]:
        if value is None:
            return value
        import math

        if not math.isfinite(value):
            raise ValueError("coordinate must be finite")
        return value


class UploadVerifiedMissionRequest(BaseModel):
    mission_name: str
    waypoints: list[VerifiedWaypointInput]
    settings: Optional[dict[str, Any]] = None


class UploadVerifiedMissionResponse(BaseModel):
    success: bool
    mission_id: str
    total_targets: int
    mission_name: Optional[str] = None
    message: Optional[str] = None


class GetVerifiedMissionResponse(BaseModel):
    mission_id: str
    mission_name: str
    total_targets: int
    waypoints: Optional[list[dict[str, Any]]] = None
    created_at: Optional[str] = None
    state: Literal["stored", "loaded", "running", "completed", "failed"]


class VerifiedTargetEvent(BaseModel):
    event_id: int = 0
    target_index: int
    mission_id: str
    event_type: Literal[
        "target_active",
        "target_arrived",
        "target_settling",
        "target_marking",
        "target_completed",
        "target_failed",
        "target_skipped",
        "target_stopped",
        "target_aborted",
    ]
    timestamp: str
    terminal: bool = False
    mission_outcome: Optional[Literal["completed", "failed", "stopped", "aborted"]] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    reason: Optional[str] = None
    message: Optional[str] = None


class VerifiedTargetEventHistoryResponse(BaseModel):
    events: list[VerifiedTargetEvent]
    latest_event_id: int
    history_evicted: bool
    oldest_available_event_id: Optional[int] = None


class PointTerminalCleanupResult(BaseModel):
    success: bool
    idempotent: bool
    reason: str
    point_mission_state: str
    dwell_cancel_result: Optional[dict] = None
    spray_off_result: Optional[dict] = None
    hold_deactivated: bool
    terminal_event_emitted: bool
    recovery_required: bool
    message: str


class MissionStatus(BaseModel):
    state: MissionState
    rpp_state: Optional[int] = None
    rpp_state_name: Optional[str] = None
    dist_to_goal: Optional[float] = None
    speed: Optional[float] = None
    xtrack: Optional[float] = None
    pose_age_ms: Optional[float] = None
    rpp_debug_age_ms: Optional[float] = None
    rpp_debug_fresh: Optional[bool] = None
    measured_speed_m_s: Optional[float] = None
    fcu_connected: Optional[bool] = None
    last_path_loaded: Optional[str] = None
    loaded_mission_id: Optional[str] = None
    running_mission_id: Optional[str] = None
    point: Optional[PointMissionStatusResponse] = None


class ActivityEntry(BaseModel):
    timestamp: str
    level: str
    message: str


class RosNodeInfo(BaseModel):
    name: str
    namespace: str = "/"
    fully_qualified_name: str
    expected: bool = False
    status: Literal["present", "missing", "unknown"] = "present"


class NodesStatusResponse(BaseModel):
    ok: bool
    source: Literal["ros_graph", "ros2_cli", "unavailable"]
    nodes: list[RosNodeInfo] = Field(default_factory=list)
    expected_nodes: list[RosNodeInfo] = Field(default_factory=list)
    errors: list[str] = Field(default_factory=list)


class NetworkAddress(BaseModel):
    family: str
    address: str
    prefixlen: Optional[int] = None


class NetworkInterfaceInfo(BaseModel):
    name: str
    operstate: Optional[str] = None
    mac: Optional[str] = None
    addresses: list[NetworkAddress] = Field(default_factory=list)


class NetworkRouteInfo(BaseModel):
    interface: Optional[str] = None
    gateway: Optional[str] = None
    destination: str = "default"


class WifiLinkInfo(BaseModel):
    interface: str
    connected: Optional[bool] = None
    ssid: Optional[str] = None
    signal_dbm: Optional[float] = None
    frequency_mhz: Optional[int] = None
    tx_bitrate: Optional[str] = None


class WifiTelemetry(BaseModel):
    available: bool
    interfaces: list[WifiLinkInfo] = Field(default_factory=list)
    errors: list[str] = Field(default_factory=list)


class NetworkTelemetryResponse(BaseModel):
    timestamp: str
    hostname: str
    source: Literal["iproute2", "sysfs", "unavailable"]
    interfaces: list[NetworkInterfaceInfo] = Field(default_factory=list)
    default_routes: list[NetworkRouteInfo] = Field(default_factory=list)
    wifi: WifiTelemetry
    errors: list[str] = Field(default_factory=list)


class RppParamSetRequest(BaseModel):
    """Set a single RPP controller parameter."""

    value: Union[bool, int, float, str]


class RppParamSetBulkRequest(BaseModel):
    """Set multiple RPP controller parameters atomically."""

    parameters: dict[str, Union[bool, int, float, str]]


class RppParamInfo(BaseModel):
    """RPP parameter schema entry with current value."""

    name: str
    type: str  # "float" | "int" | "bool" | "string"
    default: Union[float, int, bool, str, None] = None
    current: Union[float, int, bool, str, None] = None
    group: str  # category for UI grouping
    description: str  # human-readable purpose
    min: Union[float, int, None] = None
    max: Union[float, int, None] = None


class RppParamListResponse(BaseModel):
    """Response for listing all RPP params with current values."""

    parameters: list[RppParamInfo]
    count: int


class RppParamGetResponse(BaseModel):
    """Response for a single RPP param value."""

    name: str
    value: Union[bool, int, float, str, None]


class RppParamSetResponse(BaseModel):
    """Response after setting an RPP param."""

    name: str
    value: Union[bool, int, float, str]
    ok: bool = True


class RppParamSetBulkResponse(BaseModel):
    """Response after bulk-setting RPP params."""

    parameters: dict[str, bool]  # {name: success}
    ok: bool


class EstopResponse(BaseModel):
    success: bool
    message: str


class PingResponse(BaseModel):
    status: str
    timestamp: float


class ArmResponse(BaseModel):
    success: bool
    message: str
    spray_off_confirmed: Optional[bool] = None
    spray_off_result: Optional[dict] = None
    disarmed: Optional[bool] = None


class ModeResponse(BaseModel):
    success: bool
    message: str
    spray_off_confirmed: Optional[bool] = None
    spray_off_result: Optional[dict] = None


# ── Path planning request / response models ────────────────────────────────────


class DXFEntityInfo(BaseModel):
    """Parsed DXF entity summary for API responses."""

    entity_type: str  # LINE, ARC, CIRCLE, LWPOLYLINE, POINT, etc.
    layer: str  # DXF layer name
    color: int = 7  # AutoCAD color index
    entity_id: str = ""  # ezdxf handle
    is_mark: bool = True  # True = spray ON, False = TRANSIT
    length_m: float = 0.0  # Approximate arc length in metres


class EntityPreviewPoint(BaseModel):
    """Lightweight local-NED point used to render/select a DXF entity."""

    north: float
    east: float


class DXFEntityPreview(BaseModel):
    """Entity-level DXF preview geometry for canvas rendering and hit-testing."""

    entity_id: str
    entity_type: str
    layer: str
    color: int = 7
    default_is_mark: bool = True
    is_mark: bool = True
    order_index: int = 0
    length_m: float = 0.0
    geometry: dict[str, Any] = Field(default_factory=dict)
    preview_points: list[EntityPreviewPoint]
    extension_preview: Optional["EntityExtensionPreview"] = None


class DXFEntitiesResponse(BaseModel):
    """Response from /api/path/{name}/entities."""

    name: str
    frame: str = "local_ned"
    num_entities: int
    bounds: Optional[PathPreviewBounds] = None
    extension_config: Optional["PathExtensionConfig"] = None
    transit_preview: list["EntityTransitPreview"] = Field(default_factory=list)
    entities: list[DXFEntityPreview]


class EntityTransitPreview(BaseModel):
    """Lightweight no-spray connector between consecutive MARK entities."""

    from_entity_id: str
    to_entity_id: str
    length_m: float = 0.0
    points: list[EntityPreviewPoint]


class EntityExtensionPreview(BaseModel):
    """Lightweight PRE/AFT extension geometry for an entity preview."""

    enabled: bool = False
    pre_length_m: float = 0.0
    aft_length_m: float = 0.0
    pre_points: list[EntityPreviewPoint] = Field(default_factory=list)
    aft_points: list[EntityPreviewPoint] = Field(default_factory=list)


class EntityMarkOverride(BaseModel):
    """User-editable spray classification for a single DXF entity."""

    entity_id: str
    is_mark: bool


class DXFEntityOverridesRequest(BaseModel):
    """Persist per-entity spray overrides for a DXF file."""

    overrides: list[EntityMarkOverride]


class DXFEntityOverridesResponse(BaseModel):
    """Response from POST /api/path/{name}/entities."""

    name: str
    saved: bool = True
    num_overrides: int


class EntityOrderUpdateRequest(BaseModel):
    """Persist entity execution order for a DXF file."""

    entity_order: list[str]


class EntityOrderUpdateResponse(BaseModel):
    """Response from POST /api/path/{name}/entities/order."""

    name: str
    num_entities: int
    entity_order: list[str]


class PathExtensionConfig(BaseModel):
    """Per-file path extension settings."""

    enabled: bool = False
    pre_extension_m: float = Field(0.5, ge=0.0)
    aft_extension_m: float = Field(0.5, ge=0.0)
    # When True, every CAD line is an independent PRE→MARK→AFT pass (each side of
    # a square/rectangle/polygon gets its own run-up/run-out, closed loops are no
    # longer suppressed). When False, the connectivity-aware policy extends only a
    # chain's true open ends.
    #
    # None = "leave unchanged" on save: a client that omits per_line (e.g. an older
    # frontend that predates this field) preserves the saved value instead of
    # silently resetting it to False. Reads always resolve to a concrete bool.
    per_line: Optional[bool] = None


class PathExtensionConfigResponse(PathExtensionConfig):
    """Response from GET/POST /api/path/{name}/extensions."""

    name: str
    saved: bool = True


class DXFParseResponse(BaseModel):
    """Response from /api/path/parse-dxf."""

    filename: str
    num_entities: int
    entities: list[DXFEntityInfo]
    unit_scale: float  # metres per DXF unit
    layer_names: list[str]  # unique layer names found


class RefPoint(BaseModel):
    """A reference point mapping DXF coordinates to real-world lat/lon."""

    dxf_x: float  # DXF x coordinate
    dxf_y: float  # DXF y coordinate
    lat: float  # WGS84 latitude
    lon: float  # WGS84 longitude


class PathPlanRequest(BaseModel):
    """Request for /api/path/plan."""

    source: str  # filename or "builtin:square_2x2"
    selected_entities: Optional[list[str]] = None  # entity IDs to include (None = all)
    overrides: Optional[dict[str, dict]] = (
        None  # {entity_id: {scale, offsetX, offsetY, traverse}}
    )
    order: Optional[list[str]] = None  # entity IDs in execution order
    layer_mapping: Optional[dict[str, str]] = (
        None  # {layer_pattern: "mark" | "transit" | "ignore"}
    )
    origin: Optional[list[float]] = None  # [north, east] NED offset
    start_position: Optional[list[float]] = None  # [north, east] rover position for TSP
    ref_points: Optional[list[RefPoint]] = None  # reference points for DXF→NED affine
    origin_gps: Optional[list[float]] = None  # [latitude, longitude] WGS84 reference
    rotation_deg: float = 0.0  # DXF rotation relative to true north
    close_loop: bool = False  # True to close open loop paths
    line_spacing: float = 0.05  # MARK waypoint spacing (m)
    transit_spacing: float = 0.15  # TRANSIT waypoint spacing (m)
    marking_speed: float = 0.35  # MARK speed (m/s)
    transit_speed: float = 0.50  # TRANSIT speed (m/s)
    optimize: bool = True  # Reorder segments for minimal dead-heading
    # Planner preserves CAD geometry; runtime spray_controller owns latency
    # anticipation (use_distance_aware_spray). Strict: True is rejected by the
    # validator below — geometric pre-compensation is offline-only (PathEngine).
    compensate_spray: bool = False
    # Deprecated trio: ignored by /api/path/plan (a warning is logged and
    # returned in the response `warnings` when set explicitly). Configure
    # extensions via GET/POST /api/path/{name}/extensions instead.
    enable_path_extensions: bool = False
    pre_extension_m: float = Field(0.5, ge=0.0)
    aft_extension_m: float = Field(0.5, ge=0.0)
    corner_smooth_radius_m: float = Field(0.0, ge=0.0)  # Planner-side corner radius; 0 disables
    corner_smooth_arc_pts: int = Field(6, ge=2)  # Points per smoothed corner arc
    use_two_opt: bool = True  # Improve greedy segment order with 2-opt
    max_two_opt_segments: int = Field(80, ge=0, le=1000)  # Skip 2-opt above this MARK count
    max_waypoints: int = Field(10000, ge=100, le=500000)  # Hard publication guard
    max_segments: int = Field(2000, ge=1, le=100000)  # Hard segment-count guard
    include_waypoints: bool = True  # If False, return summary only (no waypoint arrays)
    spray_mode: Optional[Literal["continuous", "dash", "point"]] = None
    dash_on_distance_m: float = Field(0.30, ge=0.0)
    dash_off_distance_m: float = Field(0.30, ge=0.0)
    dash_phase_reset: Literal["per_mark_region", "continuous"] = "per_mark_region"
    point_default_dwell_s: float = Field(2.0, gt=0.0)
    point_max_dwell_s: float = Field(60.0, gt=0.0)
    point_arrival_tolerance_m: float = Field(0.05, gt=0.0)
    point_settle_time_s: float = Field(0.10, ge=0.0)
    point_leg_timeout_s: float = Field(120.0, gt=0.0)
    point_settle_speed_mps: float = Field(0.05, ge=0.0)
    point_settle_yaw_rate_rad_s: float = Field(0.05, ge=0.0)
    point_execution_mode: Literal["auto", "manual"] = "auto"
    point_leg_trajectory_mode: Literal["two_point", "densified"] = "two_point"
    point_leg_spacing_m: float = Field(0.08, gt=0.0)
    point_hold_drift_tolerance_m: float = Field(0.08, gt=0.0)
    point_hold_drift_policy: Literal["fail", "pause"] = "fail"
    point_mission_points: list[dict[str, Any]] = Field(default_factory=list)
    point_source_frame: Literal["LOCAL_NED", "GPS_SURVEYED", "DESIGN"] = "LOCAL_NED"
    gps_required_fix_type: int = Field(6, ge=0, le=8)
    gps_global_position_max_age_ms: float = Field(500.0, gt=0.0)
    gps_local_pose_max_age_ms: float = Field(500.0, gt=0.0)
    gps_fix_max_age_ms: float = Field(500.0, gt=0.0)
    gps_max_pose_global_skew_ms: float = Field(100.0, gt=0.0)
    gps_runtime_policy: Literal["pause", "fail"] = "pause"
    gps_resume_policy: Literal["manual", "auto"] = "manual"
    gps_recovery_stable_s: float = Field(2.0, ge=0.0)
    obstacle_integration_enabled: bool = False
    obstacle_signal_max_age_s: float = Field(2.0, gt=0.0)

    @field_validator("compensate_spray")
    @classmethod
    def _reject_geometric_compensation(cls, v: bool) -> bool:
        # STRICT controller-only ownership: production planning routes preserve
        # exact CAD MARK geometry; the runtime spray_controller owns latency
        # anticipation (use_distance_aware_spray). Reject geometric pre-shift at
        # the API edge so a client cannot resurrect 2.0315 m MARK segments.
        # Offline/diagnostic geometric compensation remains available by calling
        # PathEngine(compensate_spray=True) directly, off the API.
        if v:
            raise ValueError(
                "compensate_spray=true is not accepted on production planning "
                "routes: the planner preserves exact CAD geometry and the "
                "spray_controller owns latency anticipation. For offline "
                "geometric pre-compensation, call PathEngine directly."
            )
        return v


class PathPlanResponse(BaseModel):
    """Response from /api/path/plan."""

    source: str
    num_waypoints: int
    num_segments: int
    mark_length_m: float
    transit_length_m: float
    total_length_m: float
    segments: list[dict]  # [{type, points, speed, source}]
    merged_waypoints: list[list[float]]  # [[north, east], ...]
    spray_flags: list[bool]  # True = MARK
    alignment_metadata: Optional[dict] = None  # alignment stats/residuals
    planning_metadata: Optional[dict] = None  # counts/timings/bbox/unit metadata
    warnings: Optional[list[str]] = None  # geometry/safety warnings
    mission_summary: Optional["MissionSummary"] = None  # staged-mission handoff summary


class AnchorBlock(BaseModel):
    """Definitive global anchor for the aligned mission (Gap E).

    Written as the first object of a staged mission so the controller can
    re-project NED waypoints back to WGS84 if it needs to recompute a
    deviation mid-run.
    """

    frame: str = "local_ned"
    lat: float
    lon: float
    rotation_deg: float = 0.0
    scale: float = 1.0


class MissionSummary(BaseModel):
    """High-level summary returned for operator confirmation (Gap C)."""

    mission_id: str
    num_waypoints: int
    total_length_m: float
    estimated_paint_l: float
    estimated_runtime_s: float
    rmse_m: float


# ── Staged workflow: stage-specific endpoints ─────────────────────────────────
# These split the monolithic /api/path/plan into composable stages. /plan stays
# unchanged; each stage below is additive.

class AlignRequest(BaseModel):
    """Stage 6/7 — alignment only. No optimize / extend / stage / load."""

    ref_points: Optional[list[RefPoint]] = None       # DXF→NED affine fit
    origin_gps: Optional[list[float]] = None          # [lat, lon]
    rotation_deg: float = 0.0
    origin: Optional[list[float]] = None              # [north, east] NED offset
    auto_origin: bool = False
    sample_points: int = Field(20, ge=0, le=2000)     # transformed coords to return


class RefPointResidual(BaseModel):
    """Per-reference-point alignment residual (least-squares mode only)."""

    dxf_x: float
    dxf_y: float
    lat: float
    lon: float
    residual_m: float


class AlignResponse(BaseModel):
    """Response for POST /api/path/{name}/align."""

    source: str
    method: Optional[str] = None          # least_squares | single_point_heading | gps_origin
    rmse_m: float = 0.0
    scale: float = 1.0
    rotation_deg: float = 0.0
    offset_n: float = 0.0
    offset_e: float = 0.0
    origin_gps: Optional[list[float]] = None
    num_waypoints: int = 0
    sample_coords: list[list[float]] = Field(default_factory=list)   # [[n, e], ...]
    residuals: list[RefPointResidual] = Field(default_factory=list)
    warnings: Optional[list[str]] = None


class SegmentInfo(BaseModel):
    """One verification segment (stage 8)."""

    index: int
    sequence: int
    type: str                       # MARK | TRANSIT
    segment_role: Optional[str] = None   # mark | pre_transit | aft_transit | transit
    source_entity: str = ""
    is_extension: bool = False
    spray_on: bool = False
    speed: float = 0.0
    length_m: float = 0.0
    points: list[list[float]] = Field(default_factory=list)   # [[n, e], ...]


class PathSegmentsResponse(BaseModel):
    """Response for GET /api/path/{name}/segments."""

    name: str
    num_segments: int
    num_waypoints: int
    mark_length_m: float
    transit_length_m: float
    total_length_m: float
    extension_config: Optional["PathExtensionConfig"] = None
    segments: list[SegmentInfo] = Field(default_factory=list)
    warnings: Optional[list[str]] = None


class StagedMissionResponse(BaseModel):
    """Response for GET /api/path/staged/{mission_id}. Exact staged content."""

    mission_id: str
    created_at: Optional[float] = None
    anchor: Optional[dict] = None
    num_waypoints: int = 0
    waypoints: list[list[float]] = Field(default_factory=list)
    spray_flags: list[bool] = Field(default_factory=list)
    segment_runs: list[dict] = Field(default_factory=list)  # derived spray on/off runs
    spray_mode: str = "continuous"
    dash_on_distance_m: float = 0.30
    dash_off_distance_m: float = 0.30
    dash_phase_reset: str = "per_mark_region"
    point_default_dwell_s: float = 2.0
    point_max_dwell_s: float = 60.0
    point_arrival_tolerance_m: float = 0.05
    point_settle_time_s: float = 0.10
    point_leg_timeout_s: float = 120.0
    point_settle_speed_mps: float = 0.05
    point_settle_yaw_rate_rad_s: float = 0.05
    point_execution_mode: str = "auto"
    point_leg_trajectory_mode: str = "two_point"
    point_leg_spacing_m: float = 0.08
    point_hold_drift_tolerance_m: float = 0.08
    point_hold_drift_policy: str = "fail"
    point_mission_points: list[dict[str, Any]] = Field(default_factory=list)
    point_source_frame: str = ""
    point_mission_points_original: list[dict[str, Any]] = Field(default_factory=list)
    gps_required_fix_type: int = 6
    gps_global_position_max_age_ms: float = 500.0
    gps_local_pose_max_age_ms: float = 500.0
    gps_fix_max_age_ms: float = 500.0
    gps_max_pose_global_skew_ms: float = 100.0
    gps_runtime_policy: str = "pause"
    gps_resume_policy: str = "manual"
    gps_recovery_stable_s: float = 2.0
    obstacle_integration_enabled: bool = False
    obstacle_signal_max_age_s: float = 2.0
    path_fingerprint: str = ""
    max_spray_speed_mps: float = 1.0
    unsafe_speed_behavior: str = "BLOCK_SPRAY"
    calibration_profile_id: str = "factory_default"
    calibration_profile_version: int = 1
    target_paint_density: float = 1.0
    speed_pwm_table: list[dict[str, float]] = Field(default_factory=list)
    actuator_min_pwm: float = 0.0
    actuator_max_pwm: float = 2200.0
    actuator_off_pwm: float = 0.0
    actuator_min_value: float = -1.0
    actuator_max_value: float = 1.0
    actuator_off_value: float = -1.0
    timing_only_compatibility: bool = False
    configuration_revision: int = 0
    alignment_metadata: Optional[dict] = None
    metadata: Optional[dict] = None


class LoadedPathResponse(BaseModel):
    """Response for GET /api/mission/loaded-path (stage 10)."""

    loaded: bool = False
    name: Optional[str] = None
    mission_id: Optional[str] = None
    running_mission_id: Optional[str] = None
    source_name: Optional[str] = None
    placement_mode: str = "LOCAL_NED"
    origin_gps: Optional[list[float]] = None
    is_staged: bool = False
    protected: bool = False
    state: str = "idle"
    num_waypoints: int = 0
    num_mark: int = 0
    num_transit: int = 0
    has_spray_flags: bool = False
    sample_coords: list[list[float]] = Field(default_factory=list)
    sample_truncated: bool = False


class MissionClearResponse(BaseModel):
    """Confirmation and post-clear controller snapshot."""

    cleared: bool
    status: LoadedPathResponse


class LoadMissionRequest(BaseModel):
    """Payload for committing a staged mission to the controller."""

    mission_id: str
