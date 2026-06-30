"""OFFBOARD mission lifecycle state machine (async).

States:
  IDLE → ARMING → SWITCHING_OFFBOARD → RUNNING → STOPPING → IDLE
                                         ↓ (estop / abort / safety)
                                       ABORTED
                          RUNNING ─→ COMPLETED  (auto, when RPP DONE settled)

All public methods that touch ROS services are async — they delegate to
`RosBridgeNode.arm_async() / set_mode_async()` so the FastAPI event loop
is never blocked.
"""
from __future__ import annotations

import asyncio
import datetime
import inspect
import math
from collections import deque
from typing import Any, Callable, Literal, Optional

MissionKind = Literal["none", "path", "point_staged", "verified_gps", "legacy"]

from config import (
    MISSION_COMPLETE_DISARM,
    MISSION_COMPLETE_REST_SPEED_M_S,
    MISSION_COMPLETE_REST_TIMEOUT_S,
    MISSION_COMPLETE_SET_MANUAL,
    MISSION_COMPLETE_SPRAY_TIMEOUT_S,
    RPP_IDLE,
    RPP_STALE,
    RPP_UNHEALTHY_CODES,
    SETPOINT_STREAM_GRACE_S,
)
from logging_setup import get_logger
from mission_loading import MissionLoadConflict, load_block_reason, pose_origin_or_error
from mission_placement import (
    GPS_SURVEYED,
    LOCAL_NED,
    PlacementError,
    resolve_surveyed_points,
)
from models import MissionState
from path_validation import (
    normalize_path_points,
    normalize_spray_flags,
    verified_path_fingerprint,
)
from spray_safety import force_spray_off_confirmed

log = get_logger("server.offboard")

STOP_ALLOWED_STATES = {
    MissionState.RUNNING,
    MissionState.ARMING,
    MissionState.SWITCHING_OFFBOARD,
}
ABORT_NOOP_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
}
STOP_SETTLE_S = 0.1
ENTRY_COINCIDENT_TOLERANCE_M = 1e-6
CLEAR_ALLOWED_STATES = {
    MissionState.IDLE,
    MissionState.COMPLETED,
    MissionState.ABORTED,
    MissionState.ERROR,
}


class MissionClearConflict(Exception):
    """Raised when resident mission state cannot be cleared safely."""


def _build_runtime_entry_path(
    points: list[tuple[float, float]],
    spray_flags: list[bool] | None,
    rover_ned: tuple[float, float],
) -> tuple[list[tuple[float, float]], list[bool] | None, dict[str, Any]]:
    """Prepend a spray-OFF acquisition leg without changing mission data."""
    original_first = points[0]
    distance_m = math.hypot(
        rover_ned[0] - original_first[0], rover_ned[1] - original_first[1]
    )
    evidence = {
        "entry_transit_added": distance_m > ENTRY_COINCIDENT_TOLERANCE_M,
        "entry_start_ned": list(rover_ned),
        "entry_target_ned": list(original_first),
        "entry_distance_m": distance_m,
        "entry_target_original_spray_on": (
            bool(spray_flags[0]) if spray_flags is not None else None
        ),
    }
    if not evidence["entry_transit_added"]:
        return list(points), list(spray_flags) if spray_flags is not None else None, evidence

    if spray_flags is None:
        return [rover_ned, original_first, *points], None, evidence
    # The duplicate marks the end of the runtime-only entry run. For MARK-first
    # it is also the zero-distance OFF->ON boundary; PRE-first stays OFF->OFF.
    return (
        [rover_ned, original_first, *points],
        [False, False, *spray_flags],
        evidence,
    )


class OffboardController:
    def __init__(self, ros_node, activity_log: deque) -> None:
        self._node       = ros_node
        self._log        = activity_log
        self._state      = MissionState.IDLE
        self._loaded_source_pts: tuple[tuple[float, float], ...] = ()
        self._loaded_spray_flags: tuple[bool, ...] | None = None
        self._path_name: str | None = None
        self._loaded_mission_id: str | None = None
        self._running_mission_id: str | None = None
        self._source_name: str | None = None
        self._placement_mode = LOCAL_NED
        self._origin_gps: tuple[float, float] | None = None
        self._is_staged_mission = False
        self._spray_mode = "continuous"
        self._mission_kind: MissionKind = "none"
        self._verified_metadata: dict[str, Any] | None = None
        self._path_fingerprint = ""
        self._configuration_revision = 0
        self._dash_feasibility: dict[str, Any] | None = None
        # Serialises lifecycle calls. Created lazily on first use: on
        # Python 3.9 asyncio.Lock() binds an event loop at construction,
        # and the controller is built at server startup outside any loop.
        self._lock: asyncio.Lock | None = None

    def _lifecycle_lock(self) -> asyncio.Lock:
        # Only ever called from coroutines on the server's single event
        # loop, so the check-then-create is race-free.
        if self._lock is None:
            self._lock = asyncio.Lock()
        return self._lock

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def state(self) -> MissionState:
        return self._state

    @state.setter
    def state(self, value: MissionState) -> None:
        self._state = value

    @property
    def loaded_path_name(self) -> Optional[str]:
        return self._path_name

    @property
    def loaded_mission_id(self) -> Optional[str]:
        return self._loaded_mission_id

    @property
    def running_mission_id(self) -> Optional[str]:
        return self._running_mission_id

    @property
    def placement_mode(self) -> str:
        return self._placement_mode

    @property
    def has_protected_mission(self) -> bool:
        return bool(self._loaded_source_pts) and (
            self._placement_mode == GPS_SURVEYED
            or self._is_staged_mission
        )

    @property
    def spray_mode(self) -> str:
        return self._spray_mode

    @property
    def loaded_mission_kind(self) -> MissionKind:
        return self._mission_kind

    @property
    def verified_metadata(self) -> dict[str, Any] | None:
        return self._verified_metadata

    @property
    def uses_global_rpp_completion(self) -> bool:
        return self._spray_mode in {"continuous", "dash"}

    @property
    def completion_owner(self) -> str:
        return "telemetry_rpp" if self.uses_global_rpp_completion else "point_orchestrator"

    def gps_surveyed_runtime_context(self) -> dict[str, Any] | None:
        """Context for the continuous/dash GPS_SURVEYED runtime watchdog (F-02).

        Returns None unless a GPS_SURVEYED **continuous or dash** mission is
        RUNNING — point missions carry their own runtime GPS gate in the point
        orchestrator, and LOCAL_NED missions are never RTK-gated. When non-None,
        the server watchdog evaluates GPS safety each tick and feeds the result
        to the spray node's independent gate."""
        if self._state != MissionState.RUNNING:
            return None
        if self._placement_mode != GPS_SURVEYED:
            return None
        if self._spray_mode not in ("continuous", "dash"):
            return None
        return {
            "origin_gps": self._origin_gps,
            "source_points": tuple(self._loaded_source_pts),
            "spray_mode": self._spray_mode,
            "mission_id": self._running_mission_id or self._loaded_mission_id,
        }

    def loaded_path_summary(self, sample: int = 20) -> dict:
        """Read-only snapshot of the path currently resident in the controller.

        Used by GET /api/mission/loaded-path to confirm what coordinates were
        actually committed (stage 10). Returns counts + a head/tail coordinate
        sample so the operator can verify without shipping the full array.
        """
        pts = self._loaded_source_pts
        flags = self._loaded_spray_flags
        num_mark = sum(1 for f in flags) if flags else 0
        if flags is not None:
            num_mark = sum(1 for f in flags if f)
        sample = max(0, int(sample))
        if sample and len(pts) > 2 * sample:
            sample_coords = [list(p) for p in pts[:sample]] + [list(p) for p in pts[-sample:]]
            sample_truncated = True
        else:
            sample_coords = [list(p) for p in pts]
            sample_truncated = False
        return {
            "loaded": bool(pts),
            "name": self._path_name,
            "mission_id": self._loaded_mission_id,
            "running_mission_id": self._running_mission_id,
            "source_name": self._source_name,
            "placement_mode": self._placement_mode,
            "origin_gps": list(self._origin_gps) if self._origin_gps else None,
            "is_staged": self._is_staged_mission,
            "protected": self.has_protected_mission,
            "state": self._state.value,
            "num_waypoints": len(pts),
            "num_mark": num_mark,
            "num_transit": (len(flags) - num_mark) if flags else 0,
            "has_spray_flags": flags is not None,
            "sample_coords": sample_coords,
            "sample_truncated": sample_truncated,
            "dash": self._dash_feasibility,
            "dash_feasible": (
                bool(self._dash_feasibility.get("dash_feasible"))
                if self._dash_feasibility is not None
                else None
            ),
            "dash_feasibility_reason": (
                self._dash_feasibility.get("dash_feasibility_reason", "")
                if self._dash_feasibility is not None
                else ""
            ),
            "shortest_dash_on_run_m": (
                self._dash_feasibility.get("shortest_dash_on_run_m")
                if self._dash_feasibility is not None
                else None
            ),
            "shortest_dash_off_gap_m": (
                self._dash_feasibility.get("shortest_dash_off_gap_m")
                if self._dash_feasibility is not None
                else None
            ),
            "expected_on_lead_m": (
                self._dash_feasibility.get("expected_on_lead_m")
                if self._dash_feasibility is not None
                else None
            ),
            "expected_off_lead_m": (
                self._dash_feasibility.get("expected_off_lead_m")
                if self._dash_feasibility is not None
                else None
            ),
            "dash_phase_reset": (
                self._dash_feasibility.get("dash_phase_reset")
                if self._dash_feasibility is not None
                else None
            ),
            "dash_expected_speed_mps": (
                self._dash_feasibility.get("dash_expected_speed_mps")
                if self._dash_feasibility is not None
                else None
            ),
            "dash_feasibility_speed_source": (
                self._dash_feasibility.get("dash_feasibility_speed_source")
                if self._dash_feasibility is not None
                else None
            ),
        }

    # ── Path management ───────────────────────────────────────────────────────

    def load_path(
        self,
        points: list[tuple[float, float]],
        name: Optional[str] = None,
        spray_flags: Optional[list[bool]] = None,
        *,
        placement_mode: str = LOCAL_NED,
        origin_gps: tuple[float, float] | None = None,
        mission_id: str | None = None,
        source_name: str | None = None,
        is_staged: bool = False,
        allow_replace_protected: bool = False,
        spray_mode: str = "continuous",
        mission_kind: MissionKind = "legacy",
        metadata: dict[str, Any] | None = None,
        path_fingerprint: str = "",
        configuration_revision: int = 0,
        dash_feasibility: dict[str, Any] | None = None,
    ) -> None:
        reason = load_block_reason(self._state)
        if reason:
            raise MissionLoadConflict(reason)
        if placement_mode not in (LOCAL_NED, GPS_SURVEYED):
            raise ValueError(f"unsupported placement mode: {placement_mode!r}")

        new_mission_id = mission_id or name or "unknown"
        if (
            self.has_protected_mission
            and not allow_replace_protected
        ):
            raise MissionLoadConflict(
                f"Loaded mission {self._loaded_mission_id!r} is staged/surveyed; "
                "use the staged mission workflow to replace it"
            )

        normalized_points = normalize_path_points(points, label="mission path")
        if spray_flags is not None and len(spray_flags) == len(normalized_points):
            normalized_flags = normalize_spray_flags(
                spray_flags, len(normalized_points), default=False
            )
            self._loaded_spray_flags = tuple(normalized_flags)
        elif spray_flags is not None:
            normalized_flags = [False] * len(normalized_points)
            self._loaded_spray_flags = None
            self._log_entry(
                "warning",
                f"spray_flags length mismatch for {name or 'unknown'} — loading path with spray OFF",
            )
        else:
            normalized_flags = [False] * len(normalized_points)
            self._loaded_spray_flags = None
        verified_fingerprint = verified_path_fingerprint(
            normalized_points,
            normalized_flags,
            path_fingerprint,
        )
        self._loaded_source_pts = tuple(normalized_points)
        self._path_name = name or source_name or new_mission_id
        self._loaded_mission_id = new_mission_id
        self._running_mission_id = None
        self._source_name = source_name or name or new_mission_id
        self._placement_mode = placement_mode
        self._is_staged_mission = bool(is_staged)
        self._spray_mode = str(spray_mode or "continuous")
        if mission_kind == "verified_gps":
            self._mission_kind = "verified_gps"
            self._verified_metadata = dict(metadata) if metadata else None
            self._placement_mode = GPS_SURVEYED
            self._is_staged_mission = True
        elif is_staged:
            self._mission_kind = "point_staged"
            self._verified_metadata = None
        elif placement_mode == GPS_SURVEYED:
            self._mission_kind = "path"
            self._verified_metadata = None
        else:
            self._mission_kind = "legacy" if mission_kind == "legacy" else mission_kind
            self._verified_metadata = None
        self._path_fingerprint = verified_fingerprint
        self._configuration_revision = int(configuration_revision or 0)
        self._dash_feasibility = dict(dash_feasibility) if dash_feasibility else None
        if origin_gps is not None:
            lat = float(origin_gps[0])
            lon = float(origin_gps[1])
            if not math.isfinite(lat) or not math.isfinite(lon):
                raise ValueError("origin_gps must contain finite latitude/longitude")
            self._origin_gps = (lat, lon)
        else:
            self._origin_gps = None
        if self._state in (MissionState.COMPLETED, MissionState.ABORTED, MissionState.ERROR):
            self._state = MissionState.IDLE
        # Reset RPP done-settle timer so a leftover DONE from the previous
        # mission does not trigger instant auto-completion of the new one.
        if self._node is not None:
            try:
                self._node.get_rpp_monitor().reset()
            except Exception:
                pass
        self._log_entry(
            "info",
            f"Path loaded: {self._path_name} ({len(points)} pts, "
            f"id={self._loaded_mission_id}, placement={self._placement_mode})",
        )

    async def clear_mission_async(self) -> dict[str, Any]:
        """Clear the resident mission without deleting its source artifact."""
        async with self._lifecycle_lock():
            if self._state not in CLEAR_ALLOWED_STATES:
                raise MissionClearConflict(
                    f"Cannot clear mission while controller state is {self._state.value}; "
                    "stop or abort the mission first"
                )

            cleared_name = self._path_name
            self._loaded_source_pts = ()
            self._loaded_spray_flags = None
            self._path_name = None
            self._loaded_mission_id = None
            self._running_mission_id = None
            self._source_name = None
            self._placement_mode = LOCAL_NED
            self._origin_gps = None
            self._is_staged_mission = False
            self._spray_mode = "continuous"
            self._mission_kind = "none"
            self._verified_metadata = None
            self._path_fingerprint = ""
            self._configuration_revision = 0
            self._dash_feasibility = None
            self._state = MissionState.IDLE
            if self._node is not None and hasattr(self._node, "publish_path_clear"):
                self._node.publish_path_clear()
            self._log_entry(
                "info",
                f"Resident mission cleared: {cleared_name or 'none'}",
            )
            summary = self.loaded_path_summary()
        # F-04: reset the live spray-controller config outside the lifecycle lock
        # (it issues node service calls). Clears mission_id/fingerprint/revision +
        # dash transform + dwell state on the node so the next load cannot inherit
        # a stale mission-bound identity. Best-effort: a node that is down is logged
        # but does not fail the clear.
        await self._reset_spray_config_on_clear()
        return summary

    async def _reset_spray_config_on_clear(self) -> None:
        if self._node is None:
            return
        try:
            from spray_mission_config import (
                apply_spray_mission_config,
                default_backward_compatible_staged_fields,
            )

            defaults = default_backward_compatible_staged_fields()
            defaults["spray_mode"] = "continuous"
            defaults["mission_id"] = ""
            defaults["path_fingerprint"] = ""
            ok, why, _ = await apply_spray_mission_config(
                self._node, defaults, revision=0
            )
            if not ok:
                self._log_entry(
                    "warning", f"spray config reset on clear not applied: {why}"
                )
        except Exception as exc:  # noqa: BLE001 — best-effort cleanup
            self._log_entry("warning", f"spray config reset on clear raised: {exc}")

    # ── Lifecycle (async) ─────────────────────────────────────────────────────

    def _rpp_unhealthy_start_message(self, rpp_code: int) -> str:
        if rpp_code == RPP_STALE:
            return "start: RPP STALE — is twist_to_setpoint_node running?"
        if rpp_code == 4:  # RPP_RTK_WAIT
            return (
                "start: RPP RTK_WAIT — GPS fix < RTK_FIXED. "
                "Wait for fix or set require_rtk_fix:=false on the controller."
            )
        if rpp_code == 5:  # RPP_JUMP_SKIP
            return (
                "start: RPP JUMP_SKIP — EKF position jump in progress; "
                "retry in ~1 s once the estimator settles."
            )
        return f"start: RPP unhealthy (code={rpp_code})"

    def _publish_path_to_node(self, points, **kwargs) -> None:
        """Publish path while tolerating older test/helper node signatures."""
        publish = self._node.publish_path
        try:
            sig = inspect.signature(publish)
        except (TypeError, ValueError):
            publish(points, **kwargs)
            return
        params = sig.parameters
        if not any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values()):
            kwargs = {k: v for k, v in kwargs.items() if k in params}
        publish(points, **kwargs)

    async def start_async(
        self,
        auto_origin: bool = False,
        expected_mission_id: str | None = None,
        pre_publish_hook: Callable[[dict[str, Any]], None] | None = None,
    ) -> tuple[bool, str]:
        from control_arbiter import ControlArbiterError, get_control_arbiter

        try:
            # RPP/RTK_WAIT start safety remains in _start_async_locked where
            # rpp_code is checked immediately before OFFBOARD transition.
            async with get_control_arbiter().mission_start(self):
                return await self._start_async_locked(
                    auto_origin=auto_origin,
                    expected_mission_id=expected_mission_id,
                    pre_publish_hook=pre_publish_hook,
                )
        except ControlArbiterError as exc:
            self._log_entry("warning", exc.message)
            return False, exc.message

    async def _start_async_locked(
        self,
        auto_origin: bool = False,
        expected_mission_id: str | None = None,
        pre_publish_hook: Callable[[dict[str, Any]], None] | None = None,
    ) -> tuple[bool, str]:
        async with self._lifecycle_lock():
            if self._node is None:
                return False, "ROS node not available"

            # Guard: re-starting while already running re-arms and re-switches
            # OFFBOARD, which is wrong. Operator must stop first.
            if self._state == MissionState.RUNNING:
                msg = "start: mission already running — call stop first"
                self._log_entry("warning", msg)
                return False, msg

            if self._state in (
                MissionState.LOADING,
                MissionState.ARMING,
                MissionState.SWITCHING_OFFBOARD,
                MissionState.STOPPING,
                MissionState.DISARMING,
            ):
                msg = f"start: controller state is {self._state.value} — wait until idle"
                self._log_entry("warning", msg)
                return False, msg

            if not self._loaded_source_pts:
                self._state = MissionState.ERROR
                msg = "start: no path loaded"
                self._log_entry("error", msg)
                return False, msg

            if (
                expected_mission_id is not None
                and expected_mission_id != self._loaded_mission_id
            ):
                msg = (
                    f"start: mission identity mismatch: expected {expected_mission_id!r}, "
                    f"loaded {self._loaded_mission_id!r}"
                )
                self._log_entry("error", msg)
                return False, msg

            if self._placement_mode == GPS_SURVEYED and auto_origin:
                msg = "start: GPS_SURVEYED missions are incompatible with auto_origin"
                self._log_entry("error", msg)
                self._running_mission_id = None
                raise PlacementError(msg)

            fcu = self._node.get_state()
            if not fcu.get("connected", False):
                self._state = MissionState.ERROR
                msg = "start: FCU not connected"
                self._log_entry("error", msg)
                return False, msg

            if self._spray_mode == "point":
                return await self._start_point_shell_async(
                    expected_mission_id=expected_mission_id,
                    pre_publish_hook=pre_publish_hook,
                )

            pts_to_publish = list(self._loaded_source_pts)
            resolved_mission_pts = list(pts_to_publish)
            rover_local_ned = None
            if fcu.get("pose_received", False):
                rover_local_ned = [float(fcu.get("pos_n", 0.0)), float(fcu.get("pos_e", 0.0))]
            survey_translation_ned = None
            spray_flags_to_publish = (
                list(self._loaded_spray_flags)
                if self._loaded_spray_flags is not None else None
            )
            if self._placement_mode == GPS_SURVEYED:
                try:
                    pts_to_publish, translation = resolve_surveyed_points(
                        self._loaded_source_pts,
                        self._origin_gps,
                        fcu,
                    )
                    survey_translation_ned = [float(translation[0]), float(translation[1])]
                    resolved_mission_pts = list(pts_to_publish)
                except (PlacementError, ImportError) as exc:
                    self._state = MissionState.ERROR
                    msg = f"start: surveyed placement failed: {exc}"
                    self._log_entry("error", msg)
                    self._running_mission_id = None
                    raise PlacementError(msg) from exc
                self._log_entry(
                    "info",
                    "survey placement offset: "
                    f"{translation[0]:+.3f}N {translation[1]:+.3f}E",
                )

            # Surveyed placement is validated first so stale pose/GPS and RTK
            # quality failures retain their typed 422 contract instead of being
            # masked by the RPP STALE/RTK_WAIT state.
            rpp_code = fcu.get("rpp_state", RPP_STALE)
            if fcu.get("rpp_debug_fresh") is not True:
                self._state = MissionState.ERROR
                msg = "start: RPP debug stale — setpoint chain not healthy"
                self._log_entry("error", msg)
                return False, msg
            if rpp_code in RPP_UNHEALTHY_CODES:
                self._state = MissionState.ERROR
                msg = self._rpp_unhealthy_start_message(rpp_code)
                self._log_entry("error", msg)
                return False, msg

            if self._placement_mode != GPS_SURVEYED and auto_origin:
                pose_origin = pose_origin_or_error(self._node.get_state())
                if isinstance(pose_origin, str):
                    self._state = MissionState.ERROR
                    msg = f"start: {pose_origin}"
                    self._log_entry("error", msg)
                    return False, msg
                off_n, off_e = pose_origin
                rover_local_ned = [float(off_n), float(off_e)]
                pts_to_publish = [
                    (n + off_n, e + off_e) for n, e in self._loaded_source_pts
                ]
                self._log_entry(
                    "info", f"auto_origin offset: +{off_n:.3f}N +{off_e:.3f}E"
                )

            entry_evidence = {
                "entry_transit_added": False,
                "entry_start_ned": None,
                "entry_target_ned": list(resolved_mission_pts[0]),
                "entry_distance_m": None,
                "entry_target_original_spray_on": None,
            }
            final_local_pose_age_ms = fcu.get("local_pose_age_ms")
            if self._placement_mode == GPS_SURVEYED:
                # Resolve again from one fresh, internally coherent snapshot. This
                # snapshot owns both final GPS placement and the injected entry leg.
                final_fcu = self._node.get_state()
                try:
                    resolved_mission_pts, translation = resolve_surveyed_points(
                        self._loaded_source_pts,
                        self._origin_gps,
                        final_fcu,
                    )
                except (PlacementError, ImportError) as exc:
                    self._state = MissionState.ERROR
                    msg = f"start: surveyed placement failed: {exc}"
                    self._log_entry("error", msg)
                    self._running_mission_id = None
                    raise PlacementError(msg) from exc
                rover_ned = (
                    float(final_fcu["pos_n"]),
                    float(final_fcu["pos_e"]),
                )
                rover_local_ned = list(rover_ned)
                survey_translation_ned = [float(translation[0]), float(translation[1])]
                final_local_pose_age_ms = final_fcu.get("local_pose_age_ms")
                pts_to_publish, spray_flags_to_publish, entry_evidence = (
                    _build_runtime_entry_path(
                        list(resolved_mission_pts),
                        spray_flags_to_publish,
                        rover_ned,
                    )
                )
                self._log_entry(
                    "info",
                    "runtime entry transit: "
                    f"{entry_evidence['entry_distance_m']:.3f} m "
                    f"({'added' if entry_evidence['entry_transit_added'] else 'coincident; unchanged'})",
                )

            armed_here = False
            try:
                if pre_publish_hook is not None:
                    # Debug-capture placement snapshot. This is best-effort
                    # telemetry only — a failure here (e.g. full disk writing the
                    # sidecar) must never abort a mission start, so it is isolated
                    # from the control path below.
                    try:
                        pre_publish_hook({
                            "placement_mode": self._placement_mode,
                            "origin_gps": list(self._origin_gps) if self._origin_gps else None,
                            "rover_local_ned_at_resolution": rover_local_ned,
                            "survey_translation_ned": survey_translation_ned,
                            "resolved_first_waypoint_ned": list(resolved_mission_pts[0]),
                            "published_first_waypoint_ned": list(pts_to_publish[0]),
                            "source_point_count": len(self._loaded_source_pts),
                            "published_point_count": len(pts_to_publish),
                            "final_local_pose_age_ms": final_local_pose_age_ms,
                            **entry_evidence,
                            "point_count": len(pts_to_publish),
                            "spray_on_count": (
                                sum(1 for flag in spray_flags_to_publish if flag)
                                if spray_flags_to_publish is not None else 0
                            ),
                            "spray_off_count": (
                                len(spray_flags_to_publish)
                                - sum(1 for flag in spray_flags_to_publish if flag)
                                if spray_flags_to_publish is not None else len(pts_to_publish)
                            ),
                        })
                    except Exception as exc:
                        self._log_entry(
                            "warning", f"pre-publish capture hook failed (ignored): {exc}"
                        )
                # Publish the mission path before the OFFBOARD request so the
                # 50 Hz setpoint stream carries mission setpoints, not just the
                # streamer's zero-velocity bootstrap, when PX4 evaluates entry.
                publish_kwargs = {
                    "spray_flags": spray_flags_to_publish,
                    "mission_id": self._loaded_mission_id or "",
                    "configuration_revision": self._configuration_revision,
                    "path_fingerprint": self._path_fingerprint,
                    "verify_supplied_fingerprint": False,
                }
                if entry_evidence["entry_transit_added"]:
                    publish_kwargs["runtime_entry"] = True
                self._publish_path_to_node(pts_to_publish, **publish_kwargs)

                # ── Arm ───────────────────────────────────────────────────────
                self._state = MissionState.ARMING
                self._log_entry("info", "arming…")
                ok, why = await self._node.arm_async(True)
                if not ok:
                    self._state = MissionState.ERROR
                    self._log_entry("error", f"arming failed: {why}")
                    return False, f"arm failed: {why}"
                armed_here = True

                # ── Switch to OFFBOARD ────────────────────────────────────────
                self._state = MissionState.SWITCHING_OFFBOARD
                self._log_entry("info", "switching to OFFBOARD…")
                await asyncio.sleep(SETPOINT_STREAM_GRACE_S)
                fcu = self._node.get_state()
                rpp_code = fcu.get("rpp_state", RPP_STALE)
                if fcu.get("rpp_debug_fresh") is not True:
                    self._state = MissionState.ERROR
                    msg = "start: RPP debug stale after path publish — setpoint chain not ready"
                    self._log_entry("error", msg)
                    await self._node.arm_async(False)
                    return False, msg
                if rpp_code in RPP_UNHEALTHY_CODES:
                    self._state = MissionState.ERROR
                    msg = self._rpp_unhealthy_start_message(rpp_code)
                    self._log_entry("error", msg)
                    await self._node.arm_async(False)
                    return False, msg
                if rpp_code == RPP_IDLE:
                    self._state = MissionState.ERROR
                    msg = (
                        "start: RPP IDLE after path publish — "
                        "setpoint chain not ready"
                    )
                    self._log_entry("error", msg)
                    await self._node.arm_async(False)
                    return False, msg
                ok, why = await self._node.set_mode_async("OFFBOARD")
                if not ok:
                    self._state = MissionState.ERROR
                    self._log_entry("error", f"OFFBOARD switch failed: {why}")
                    # Best-effort disarm; ignore result
                    await self._node.arm_async(False)
                    return False, f"OFFBOARD failed: {why}"

                self._state = MissionState.RUNNING
                self._running_mission_id = self._loaded_mission_id
                self._log_entry("info", f"mission running: {self._path_name}")
                return True, "running"
            except Exception as exc:
                self._state = MissionState.ERROR
                self._running_mission_id = None
                self._log_entry("error", f"unexpected start failure: {exc}")
                if armed_here:
                    try:
                        await self._node.arm_async(False)
                    except Exception:
                        pass
                return False, f"unexpected start failure: {exc}"

    async def _start_point_shell_async(
        self,
        *,
        expected_mission_id: str | None,
        pre_publish_hook: Callable[[dict[str, Any]], None] | None,
    ) -> tuple[bool, str]:
        """Arm and enter OFFBOARD for point missions without publishing a full path."""
        if self._node is None:
            return False, "ROS node not available"
        if expected_mission_id is not None and expected_mission_id != self._loaded_mission_id:
            msg = (
                f"start: mission identity mismatch: expected {expected_mission_id!r}, "
                f"loaded {self._loaded_mission_id!r}"
            )
            self._log_entry("error", msg)
            return False, msg

        fcu = self._node.get_state()
        if not fcu.get("connected", False):
            self._state = MissionState.ERROR
            return False, "start: FCU not connected"
        if fcu.get("rpp_debug_fresh") is not True:
            self._state = MissionState.ERROR
            return False, "start: RPP debug stale — setpoint chain not healthy"

        if pre_publish_hook is not None:
            try:
                pre_publish_hook(
                    {
                        "placement_mode": self._placement_mode,
                        "spray_mode": self._spray_mode,
                        "point_count": len(self._loaded_source_pts),
                        "published_point_count": 0,
                    }
                )
            except Exception as exc:
                self._log_entry("warning", f"pre-publish capture hook failed (ignored): {exc}")

        armed_here = False
        try:
            self._state = MissionState.ARMING
            ok, why = await self._node.arm_async(True)
            if not ok:
                self._state = MissionState.ERROR
                return False, f"arm failed: {why}"
            armed_here = True

            self._state = MissionState.SWITCHING_OFFBOARD
            await asyncio.sleep(SETPOINT_STREAM_GRACE_S)
            fcu = self._node.get_state()
            if fcu.get("rpp_debug_fresh") is not True:
                self._state = MissionState.ERROR
                await self._node.arm_async(False)
                return False, "start: RPP debug stale after point shell arm"
            ok, why = await self._node.set_mode_async("OFFBOARD")
            if not ok:
                self._state = MissionState.ERROR
                await self._node.arm_async(False)
                return False, f"OFFBOARD failed: {why}"

            self._state = MissionState.RUNNING
            self._running_mission_id = self._loaded_mission_id
            self._log_entry("info", f"point mission shell running: {self._path_name}")
            return True, "point shell running"
        except Exception as exc:
            self._state = MissionState.ERROR
            self._running_mission_id = None
            if armed_here:
                try:
                    await self._node.arm_async(False)
                except Exception:
                    pass
            return False, f"unexpected point start failure: {exc}"

    async def stop_async(self) -> dict[str, Any]:
        """Soft stop: publish a single-point stop-path → RPP zeroes velocity.

        Empty Path is **ignored** by upstream RPP (early-return), so we
        publish a stop-path at the rover's current position. RPP treats it
        as DONE immediately and outputs zero velocity. Vehicle stays armed.
        """
        async with self._lifecycle_lock():
            if self._node is None:
                msg = "stop: ROS node not available"
                self._log_entry("warning", msg)
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_node",
                    "armed": None,
                    "message": msg,
                }

            if self._state not in STOP_ALLOWED_STATES:
                msg = f"stop called from {self._state.value} — no active mission to stop"
                self._log_entry("info", msg)
                s = self._node.get_state()
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_op",
                    "armed": s.get("armed"),
                    "message": msg,
                }

            try:
                self._state = MissionState.STOPPING
                stop_position = self._node.publish_stop_path()
                if stop_position is None:
                    self._state = MissionState.ERROR
                    s = self._node.get_state()
                    msg = "stop: no local pose available; stop-path not published"
                    self._log_entry("error", msg)
                    return {
                        "success": False,
                        "state": self._state.value,
                        "action": "no_pose",
                        "armed": s.get("armed"),
                        "message": msg,
                    }

                await asyncio.sleep(STOP_SETTLE_S)
                self._state = MissionState.IDLE
                self._running_mission_id = None
                self._mark_control_idle()
                s = self._node.get_state()
                n, e = stop_position
                msg = f"mission stopped at N={n:.3f}, E={e:.3f}"
                self._log_entry("info", msg)
                return {
                    "success": True,
                    "state": self._state.value,
                    "action": "hold_position",
                    "armed": s.get("armed"),
                    "message": msg,
                    "stop_position": {"n": n, "e": e},
                }
            except Exception as exc:
                self._state = MissionState.ERROR
                msg = f"stop failed: {exc}"
                self._log_entry("error", msg)
                try:
                    s = self._node.get_state()
                    armed = s.get("armed")
                except Exception:
                    armed = None
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "error",
                    "armed": armed,
                    "message": msg,
                }

    async def abort_async(self) -> dict[str, Any]:
        """Hard abort: stop-path + MANUAL + disarm."""
        async with self._lifecycle_lock():
            if self._node is None:
                msg = "abort: ROS node not available"
                self._log_entry("warning", msg)
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_node",
                    "message": msg,
                    "errors": [msg],
                    "stop_path_sent": False,
                    "manual_mode": False,
                    "disarmed": False,
                    "armed": None,
                }

            if self._state in ABORT_NOOP_STATES:
                msg = f"abort called from {self._state.value} — no active mission to abort"
                self._log_entry("info", msg)
                s = self._node.get_state()
                return {
                    "success": True,
                    "state": self._state.value,
                    "action": "no_op",
                    "message": msg,
                    "errors": [],
                    "stop_path_sent": False,
                    "manual_mode": s.get("mode") == "MANUAL",
                    "disarmed": not bool(s.get("armed")),
                    "armed": s.get("armed"),
                }

            errors: list[str] = []
            stop_position: tuple[float, float] | None = None
            manual_mode = False
            disarmed = False

            try:
                stop_position = self._node.publish_stop_path()
                if stop_position is None:
                    errors.append("publish_stop_path: no local pose available")
            except Exception as exc:
                errors.append(f"publish_stop_path raised: {exc}")
                log.exception("abort publish_stop_path raised")

            spray_off = await force_spray_off_confirmed(
                self._node, timeout_s=MISSION_COMPLETE_SPRAY_TIMEOUT_S
            )
            if not spray_off.success and spray_off.live:
                errors.append(f"spray OFF: {spray_off.message}")

            try:
                ok, why = await self._node.set_mode_async("MANUAL")
                manual_mode = bool(ok)
                if not ok:
                    errors.append(f"set_mode(MANUAL): {why}")
            except Exception as exc:
                errors.append(f"set_mode(MANUAL) raised: {exc}")
                log.exception("abort set_mode(MANUAL) raised")

            self._state = MissionState.DISARMING
            try:
                ok, why = await self._node.arm_async(False)
                disarmed = bool(ok)
                if not ok:
                    errors.append(f"disarm: {why}")
            except Exception as exc:
                errors.append(f"disarm raised: {exc}")
                log.exception("abort disarm raised")

            self._state = MissionState.ABORTED
            self._running_mission_id = None
            self._mark_control_idle()
            try:
                s = self._node.get_state()
                armed = s.get("armed")
                if s.get("mode") == "MANUAL":
                    manual_mode = True
                if armed is False:
                    disarmed = True
            except Exception:
                armed = None

            msg = "mission ABORTED — MANUAL + disarm"
            if errors:
                msg += " (with errors: " + "; ".join(errors) + ")"
            self._log_entry("warning" if errors else "error", msg)

            result: dict[str, Any] = {
                "success": not errors,
                "state": self._state.value,
                "action": "abort",
                "message": msg,
                "errors": errors,
                "stop_path_sent": stop_position is not None,
                "manual_mode": manual_mode,
                "disarmed": disarmed,
                "armed": armed,
            }
            if stop_position is not None:
                n, e = stop_position
                result["stop_position"] = {"n": n, "e": e}
            result["spray_off_result"] = spray_off.as_dict()
            return result

    async def disarm_async(self) -> bool:
        async with self._lifecycle_lock():
            if self._node is None:
                self._log_entry("warning", "disarm: ROS node not available")
                return False

            ok, why = await self._node.arm_async(False)
            self._state = MissionState.IDLE
            self._running_mission_id = None
            self._mark_control_idle()
            self._log_entry(
                "info" if ok else "error",
                f"disarm {'ok' if ok else f'failed: {why}'}",
            )
            return ok

    async def complete_async(self) -> dict[str, Any]:
        """Terminalize a successfully tracked mission before marking complete."""
        async with self._lifecycle_lock():
            if self._state != MissionState.RUNNING:
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_op",
                    "message": f"complete called from {self._state.value}",
                    "warnings": [],
                }
            if self._node is None:
                self._state = MissionState.ERROR
                return {
                    "success": False,
                    "state": self._state.value,
                    "action": "no_node",
                    "message": "complete: ROS node not available",
                    "warnings": ["ROS node not available"],
                }

            warnings: list[str] = []
            fresh_done = False
            try:
                fresh_done = bool(self._node.get_rpp_monitor().is_done())
            except Exception as exc:
                warnings.append(f"fresh DONE check failed: {exc}")
            if not fresh_done:
                warnings.append("RPP DONE was not fresh/settled at terminalization")

            self._state = MissionState.STOPPING
            spray_off_confirmed = await self._terminalize_spray(warnings)

            stop_position: tuple[float, float] | None = None
            try:
                stop_position = self._node.publish_stop_path()
                if stop_position is None:
                    warnings.append("stop-path not published: no local pose")
            except Exception as exc:
                warnings.append(f"publish_stop_path raised: {exc}")
                log.exception("completion publish_stop_path raised")

            rest_confirmed = await self._wait_until_rest(warnings)

            manual_mode = True
            if MISSION_COMPLETE_SET_MANUAL:
                try:
                    ok, why = await self._node.set_mode_async("MANUAL")
                    manual_mode = bool(ok)
                    if not ok:
                        warnings.append(f"set_mode(MANUAL): {why}")
                except Exception as exc:
                    manual_mode = False
                    warnings.append(f"set_mode(MANUAL) raised: {exc}")
                    log.exception("completion set_mode(MANUAL) raised")

            disarmed = True
            if MISSION_COMPLETE_DISARM:
                self._state = MissionState.DISARMING
                try:
                    ok, why = await self._node.arm_async(False)
                    disarmed = bool(ok)
                    if not ok:
                        warnings.append(f"disarm: {why}")
                except Exception as exc:
                    disarmed = False
                    warnings.append(f"disarm raised: {exc}")
                    log.exception("completion disarm raised")

            all_confirmed = (
                fresh_done
                and spray_off_confirmed
                and rest_confirmed
                and manual_mode
                and disarmed
            )
            if all_confirmed:
                self.mark_completed()
                self._log_entry("info", f"mission terminalized safely: {self._path_name}")
                action = "complete_terminalized"
                message = "mission completed"
            else:
                self._state = MissionState.ERROR
                self._running_mission_id = None
                self._mark_control_idle()
                self._log_entry(
                    "warning",
                    "mission terminalization degraded: "
                    + "; ".join(warnings),
                )
                action = "completion_degraded"
                message = "mission terminalization degraded"
            return {
                "success": all_confirmed,
                "state": self._state.value,
                "action": action,
                "message": message,
                "warnings": warnings,
                "fresh_done": fresh_done,
                "spray_off_confirmed": spray_off_confirmed,
                "rest_confirmed": rest_confirmed,
                "manual_mode": manual_mode,
                "disarmed": disarmed,
                "stop_position": (
                    {"n": stop_position[0], "e": stop_position[1]}
                    if stop_position is not None else None
                ),
            }

    async def _terminalize_spray(self, warnings: list[str]) -> bool:
        result = await force_spray_off_confirmed(
            self._node, timeout_s=MISSION_COMPLETE_SPRAY_TIMEOUT_S
        )
        try:
            ok, why = await self._node.set_spray_param_async("spray_enabled", False)
            if not ok:
                warnings.append(f"spray_enabled=False: {why}")
        except AttributeError:
            warnings.append("spray param API unavailable")
        except Exception as exc:
            warnings.append(f"spray_enabled=False raised: {exc}")

        if result.success:
            return True
        if not result.live:
            s = self._node.get_state()
            if not bool(s.get("spraying", False)):
                warnings.append("spray runtime status unavailable; used /spray/state fallback")
                return False
        warnings.append(f"spray OFF not confirmed: {result.as_dict()}")
        return False

    async def _wait_until_rest(self, warnings: list[str]) -> bool:
        deadline = asyncio.get_running_loop().time() + MISSION_COMPLETE_REST_TIMEOUT_S
        last_speed: float | None = None
        while asyncio.get_running_loop().time() <= deadline:
            s = self._node.get_state()
            value = s.get("measured_speed_m_s")
            if value is not None:
                try:
                    last_speed = float(value)
                except (TypeError, ValueError):
                    last_speed = None
                if last_speed is not None and math.isfinite(last_speed):
                    if last_speed <= MISSION_COMPLETE_REST_SPEED_M_S:
                        return True
            await asyncio.sleep(0.05)
        warnings.append(
            "measured rest not confirmed"
            + (f" (last speed {last_speed:.3f} m/s)" if last_speed is not None else "")
        )
        return False

    def reset_progress_for_restart(self, expected_mission_id: str) -> None:
        """Clear runtime progress for continuous/dash restart without mutating geometry."""
        if expected_mission_id != self._loaded_mission_id:
            raise ValueError(
                f"restart rejected: mission identity mismatch "
                f"(expected {expected_mission_id!r}, loaded {self._loaded_mission_id!r})"
            )
        self._running_mission_id = None
        if self._state in {
            MissionState.COMPLETED,
            MissionState.ABORTED,
            MissionState.ERROR,
        }:
            self._state = MissionState.IDLE

    # Called from telemetry loop or complete_async. State-only, no service calls.
    def mark_completed(self) -> None:
        if self._state == MissionState.RUNNING:
            self._state = MissionState.COMPLETED
            self._running_mission_id = None
            self._mark_control_idle()
            self._log_entry("info", f"mission completed: {self._path_name}")
        elif self._state in {MissionState.STOPPING, MissionState.DISARMING}:
            self._state = MissionState.COMPLETED
            self._running_mission_id = None
            self._mark_control_idle()
            self._log_entry("info", f"mission completed: {self._path_name}")

    # ── Internal ──────────────────────────────────────────────────────────────

    def _mark_control_idle(self) -> None:
        try:
            from control_arbiter import get_control_arbiter

            get_control_arbiter().mark_idle_if_not_joystick()
        except Exception:
            pass

    def _log_entry(self, level: str, message: str) -> None:
        ts = datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z"
        self._log.append({"timestamp": ts, "level": level, "message": message})
        getattr(log, level if level in ("info", "warning", "error") else "info")(message)
