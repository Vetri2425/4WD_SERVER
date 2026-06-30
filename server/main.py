"""Drawing Rover FastAPI backend.

Lifespan order (startup → ready → shutdown):
  1. Configure logging
  2. Initialise auth (load local password hash + machine token records)
  3. rclpy.init() + RosBridgeNode + MultiThreadedExecutor in daemon thread
  4. Build shared singletons (PathManager, OffboardController, EmergencyHandler)
  5. Register Socket.IO handlers
  6. Start telemetry push loop (10 Hz) — also runs:
       · auto-completion (RUNNING → COMPLETED on RPP DONE settle)
       · pose-stale watchdog (RUNNING + STALE > grace → estop)
       · disconnect notification
  7. Start UDP discovery beacon

Shutdown reverses the order. Telemetry loop catches and logs every exception
without dying. Beacon and rclpy threads use Event-based stop signals so
shutdown completes within ~1 s.
"""

from __future__ import annotations

import asyncio
import datetime
import math
import time
from collections import deque
from contextlib import asynccontextmanager
from typing import Optional

import socketio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.openapi.utils import get_openapi
from starlette.middleware.gzip import GZipMiddleware

from auth import authenticated_sids, init_auth
from config import (
    BEACON_INTERVAL,
    BEACON_PORT,
    CORS_ALLOW_CREDENTIALS,
    CORS_ALLOW_ORIGINS,
    DEFAULT_PORT,
    GPS_FIX_NAMES,
    MAX_ACTIVITY_LOG,
    MISSION_DIR,
    POSE_STALE_MS,
    ROVER_ID,
    RPP_STATE_NAMES,
    RPP_UNHEALTHY_CODES,
    SAFETY_STALE_GRACE_S,
    TELEMETRY_HZ,
)
from gps_safety import GpsSurveyedSafetyParams, evaluate_gps_surveyed_safety
from logging_setup import configure_logging, get_logger
from models import MissionState

# ── sd_notify for systemd watchdog ────────────────────────────────────────────
_sd_notifier = None
try:
    import sdnotify

    _sd_notifier = sdnotify.SystemdNotifier()
except ImportError:
    pass

# ── Module-level singletons (populated in lifespan) ───────────────────────────
ros_node: Optional["object"] = None
offboard_ctrl: Optional["object"] = None
path_mgr: Optional["object"] = None
emergency_handler: Optional["object"] = None
_executor: Optional["object"] = None
_beacon: Optional["object"] = None
_listener: Optional["object"] = None
_telemetry_task: Optional[asyncio.Task] = None
bridge_health: Optional["object"] = None
rtk_manager: Optional["object"] = None
mission_capture: Optional["object"] = None
point_mission: Optional["object"] = None
verified_mission: Optional["object"] = None
verified_mission_store: Optional["object"] = None
hold_owner: Optional["object"] = None
operation_coordinator: Optional["object"] = None
manual_gateway: Optional["object"] = None
joystick_ctrl: Optional["object"] = None
spray_startup_reconciliation: Optional["object"] = None

# Bounded, thread-safe ring buffer (deque maxlen). All log appends are atomic
# under the GIL; bounded eviction is built in. Replaces the racy list+trim.
activity_log: deque = deque(maxlen=MAX_ACTIVITY_LOG)

log = get_logger("server.main")


# ── Socket.IO ASGI app ────────────────────────────────────────────────────────
# cors_allowed_origins must match the REST CORS policy — they are independent
# implementations and both must agree.
sio = socketio.AsyncServer(
    async_mode="asgi",
    cors_allowed_origins="*" if "*" in CORS_ALLOW_ORIGINS else CORS_ALLOW_ORIGINS,
)
socket_app = socketio.ASGIApp(sio)


# ── Lifespan ──────────────────────────────────────────────────────────────────


@asynccontextmanager
async def lifespan(app: FastAPI):
    global ros_node, offboard_ctrl, path_mgr, emergency_handler
    global _executor, _beacon, _listener, _telemetry_task, bridge_health, rtk_manager
    global mission_capture, point_mission, verified_mission, verified_mission_store
    global hold_owner, manual_gateway, joystick_ctrl
    global spray_startup_reconciliation, operation_coordinator

    configure_logging()
    init_auth()

    # ── Start ROS2 ────────────────────────────────────────────────────────────
    try:
        import rclpy
        from ros_node import RosBridgeNode, RosExecutorThread

        if not rclpy.ok():
            rclpy.init()
        ros_node = RosBridgeNode()
        _executor = RosExecutorThread(num_threads=4)
        _executor.add_node(ros_node)
        _executor.start()
        _record("info", "ROS2 bridge started")
        # Reset operator authorization after a server-only restart too; the
        # spray node may have survived with its prior parameter value.
        disabled = False
        disable_reason = "spray_controller not discovered"
        for _ in range(5):
            disabled, disable_reason = await ros_node.set_spray_param_async(
                "spray_enabled", False, timeout=1.0
            )
            if disabled:
                break
            await asyncio.sleep(0.1)
        if not disabled:
            _record("warning", f"Could not reset spray authorization: {disable_reason}")
    except Exception as exc:
        log.exception("ROS2 startup failed — continuing without MAVROS")
        _record("warning", f"ROS2 unavailable — server running without MAVROS: {exc}")

    # ── Build shared objects ──────────────────────────────────────────────────
    from beacon import RoverBeacon, BeaconListener
    from emergency import EmergencyHandler
    from offboard_controller import OffboardController
    from path_manager import PathManager
    from rtk_manager import AsyncRTKManager
    from mission_debug_capture import MissionDebugCoordinator
    from manual_control_gateway import ManualControlGateway, build_manual_transport
    from mission_ops import MissionOperationCoordinator
    from point_events import get_point_event_journal
    from point_mission import PointMissionOrchestrator
    from setpoint_hold import SetpointHoldOwner
    from verified_mission.adapter import VerifiedMissionOrchestrator
    from verified_mission.events import get_target_event_journal
    from verified_mission.store import VerifiedMissionStore

    operation_coordinator = MissionOperationCoordinator()
    path_mgr = PathManager(MISSION_DIR)
    offboard_ctrl = OffboardController(ros_node, activity_log)
    manual_gateway = ManualControlGateway(build_manual_transport(ros_node))
    manual_gateway.start()
    from joystick_controller import JoystickController

    joystick_ctrl = JoystickController(ros_node, offboard_ctrl, manual_gateway)
    hold_owner = SetpointHoldOwner()
    point_mission = PointMissionOrchestrator()
    point_mission.set_logger(_record)
    verified_mission = VerifiedMissionOrchestrator()
    verified_mission.set_logger(_record)
    verified_mission_store = VerifiedMissionStore()

    def _verified_is_resident(mission_id: str) -> bool:
        if offboard_ctrl is None:
            return False
        loaded_id = getattr(offboard_ctrl, "loaded_mission_id", None)
        running_id = getattr(offboard_ctrl, "running_mission_id", None)
        loaded_kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
        if loaded_kind != "verified_gps":
            return False
        if mission_id in {loaded_id, running_id}:
            return True
        if verified_mission is not None and verified_mission.is_active():
            if getattr(verified_mission.status, "mission_id", None) == mission_id:
                return True
        return False

    verified_mission_store.prune_expired(is_resident=_verified_is_resident)

    loop = asyncio.get_running_loop()
    get_point_event_journal().configure_emit(loop, _emit_authenticated)
    get_target_event_journal().configure_emit(loop, _emit_authenticated)

    def _obstacle_callback(clear: bool) -> None:
        if offboard_ctrl is None:
            return
        kind = getattr(offboard_ctrl, "loaded_mission_kind", "none")
        if kind == "verified_gps" and verified_mission is not None:
            verified_mission.set_obstacle_clear(clear)
        elif point_mission is not None:
            point_mission.set_obstacle_clear(clear)

    if ros_node is not None:
        ros_node.set_obstacle_callback(_obstacle_callback)

    from spray_startup_reconciliation import SprayStartupReconciliation

    spray_startup_reconciliation = SprayStartupReconciliation()
    if ros_node is not None:
        await spray_startup_reconciliation.start(ros_node, record=_record)
    else:
        spray_startup_reconciliation.mark_bridge_unavailable()
    mission_capture = MissionDebugCoordinator()
    emergency_handler = EmergencyHandler(
        ros_node, offboard_ctrl, activity_log, mission_capture
    )
    def _rtk_navigation_state() -> dict:
        if ros_node is None:
            return {}
        return ros_node.get_state()

    rtk_manager = AsyncRTKManager(navigation_provider=_rtk_navigation_state)

    # ── Register Socket.IO handlers ───────────────────────────────────────────
    from sockets.events import register_handlers

    register_handlers(sio)

    # ── Start telemetry + watchdog loop ───────────────────────────────────────
    _telemetry_task = asyncio.create_task(_telemetry_loop(), name="telemetry-loop")

    # ── Start bridge-health watchdog (Phase 3A: observe-only by default) ───────
    try:
        from bridge_health import BridgeHealthManager

        bridge_health = BridgeHealthManager(
            ros_node, offboard_ctrl, _record, _emit_authenticated
        )
        bridge_health.start()
    except Exception as exc:
        log.exception("BridgeHealthManager failed to start")
        _record("warning", f"bridge-health watchdog unavailable: {exc}")

    # ── Start UDP discovery beacon ────────────────────────────────────────────
    _beacon = RoverBeacon(
        port=BEACON_PORT,
        interval=BEACON_INTERVAL,
        rover_id=ROVER_ID,
        server_port=DEFAULT_PORT,
    )
    _beacon.start()
    _listener = BeaconListener(port=BEACON_PORT)
    _listener.start()

    _record("info", f"Server ready on port {DEFAULT_PORT}")
    log.info("server ready: port=%d telemetry=%dHz", DEFAULT_PORT, TELEMETRY_HZ)

    # Notify systemd that we're ready (Type=notify)
    if _sd_notifier:
        _sd_notifier.notify("READY=1")

    yield  # ─── Running ───────────────────────────────────────────────────────

    # ── Shutdown ──────────────────────────────────────────────────────────────
    log.info("shutting down…")

    if joystick_ctrl is not None:
        try:
            await joystick_ctrl.shutdown()
        except Exception:
            log.exception("joystick shutdown raised")

    if rtk_manager is not None:
        try:
            await rtk_manager.shutdown()
        except Exception:
            log.exception("RTK manager stop raised")

    if bridge_health is not None:
        try:
            await bridge_health.stop()
        except Exception:
            log.exception("bridge-health stop raised")

    if _telemetry_task:
        _telemetry_task.cancel()
        try:
            await _telemetry_task
        except (asyncio.CancelledError, Exception):
            pass

    if _listener:
        _listener.stop()
    if _beacon:
        _beacon.stop()

    if _executor:
        _executor.stop()

    if ros_node:
        try:
            ros_node.destroy_node()
        except Exception:
            log.exception("destroy_node raised")
    try:
        import rclpy

        rclpy.try_shutdown()
    except Exception:
        pass

    _record("info", "Server stopped")


# ── FastAPI app factory ───────────────────────────────────────────────────────


def create_app() -> FastAPI:
    app = FastAPI(
        title="Drawing Rover API",
        version="1.0.0",
        lifespan=lifespan,
    )

    def custom_openapi():
        if app.openapi_schema:
            return app.openapi_schema
        schema = get_openapi(
            title=app.title,
            version=app.version,
            routes=app.routes,
        )
        schema.setdefault("components", {}).setdefault("securitySchemes", {})[
            "RoverToken"
        ] = {
            "type": "apiKey",
            "in": "header",
            "name": "X-Rover-Token",
        }
        schema["security"] = [{"RoverToken": []}]
        app.openapi_schema = schema
        return schema

    app.openapi = custom_openapi
    app.add_middleware(GZipMiddleware, minimum_size=1024)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=CORS_ALLOW_ORIGINS,
        allow_credentials=CORS_ALLOW_CREDENTIALS,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # REST routers
    from routes.system import router as sys_router
    from routes.auth import router as auth_router
    from routes.vehicle import router as veh_router
    from routes.mission import router as mis_router
    from routes.verified_mission import router as verified_mission_router
    from routes.path import paths_router, path_router
    from routes.params import router as par_router
    from routes.rpp_params import router as rpp_par_router
    from routes.telemetry import router as tel_router
    from routes.rtk import router as rtk_router
    from routes.spray import router as spray_router
    from routes.spray_params import router as spray_par_router
    from routes.spray_mode import router as spray_mode_router

    app.include_router(sys_router, prefix="/api")
    app.include_router(auth_router, prefix="/api")
    app.include_router(veh_router, prefix="/api")
    app.include_router(mis_router, prefix="/api")
    app.include_router(verified_mission_router, prefix="/api")
    app.include_router(paths_router, prefix="/api")  # → /api/paths
    app.include_router(path_router, prefix="/api")  # → /api/path/*
    app.include_router(spray_mode_router, prefix="/api")  # → /api/path/{name}/spray-mode/*
    app.include_router(par_router, prefix="/api")
    app.include_router(rpp_par_router, prefix="/api")
    app.include_router(tel_router, prefix="/api")
    app.include_router(rtk_router, prefix="/api")
    app.include_router(spray_router, prefix="/api")   # → /api/spray/*
    app.include_router(spray_par_router, prefix="/api")  # → /api/spray/params/*

    # Socket.IO
    app.mount("/socket.io", socket_app)
    return app


app = create_app()


async def _emit_authenticated(event: str, data: dict) -> None:
    """Emit a Socket.IO event only to authenticated operator sessions."""
    for sid in authenticated_sids():
        await sio.emit(event, data, to=sid)


def _should_global_rpp_complete(offboard_ctrl, point_mission, ros_node) -> bool:
    if offboard_ctrl is None or ros_node is None:
        return False
    if offboard_ctrl.state != MissionState.RUNNING:
        return False
    if not getattr(offboard_ctrl, "uses_global_rpp_completion", False):
        return False
    return bool(ros_node.get_rpp_monitor().is_done())


# ── Telemetry loop with watchdog and auto-completion ──────────────────────────


def _sanitize(d: dict) -> dict:
    """Replace float NaN/Inf with None so Socket.IO emits valid JSON.

    Python's json encoder writes the bare token NaN for float('nan'), which is
    illegal JSON and causes JS JSON.parse() to throw, disconnecting the client.
    """
    return {
        k: (None if isinstance(v, float) and not math.isfinite(v) else v)
        for k, v in d.items()
    }


async def _telemetry_loop() -> None:
    interval = 1.0 / TELEMETRY_HZ
    prev_connected: Optional[bool] = None
    stale_since: Optional[float] = None
    consecutive_errors = 0
    _watchdog_counter = 0
    # F-02: GPS_SURVEYED continuous/dash runtime gate state
    gps_gate_seq = 0
    gps_gate_fault_count = 0
    gps_gate_last_fault_time: Optional[float] = None
    _WATCHDOG_EVERY_N = TELEMETRY_HZ * 3  # ping systemd every ~3s

    log.info("telemetry loop started @ %d Hz", TELEMETRY_HZ)
    try:
        while True:
            try:
                await asyncio.sleep(interval)
                if ros_node is None:
                    continue

                s = ros_node.get_state()
                code = s.get("rpp_state", 0)
                now = time.time()
                legacy_spraying = bool(s.get("spraying", False))
                spray_rt = ros_node.get_spray_runtime_status()
                mission_running = (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.RUNNING
                    and bool(s.get("armed", False))
                )
                from spray_safety import build_spray_telemetry_fields

                mission_dash = None
                if offboard_ctrl is not None and hasattr(
                    offboard_ctrl, "loaded_path_summary"
                ):
                    summary = offboard_ctrl.loaded_path_summary()
                    mission_dash = {
                        "dash_feasible": summary.get("dash_feasible"),
                        "dash_feasibility_reason": summary.get(
                            "dash_feasibility_reason"
                        ),
                        "shortest_dash_on_run_m": summary.get(
                            "shortest_dash_on_run_m"
                        ),
                        "shortest_dash_off_gap_m": summary.get(
                            "shortest_dash_off_gap_m"
                        ),
                        "dash_phase_reset": summary.get("dash_phase_reset"),
                        "dash_expected_speed_mps": summary.get(
                            "dash_expected_speed_mps"
                        ),
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

                # ── 1. Push telemetry ──────────────────────────────────────────
                telem = {
                    "pos_n": s.get("pos_n"),
                    "pos_e": s.get("pos_e"),
                    "heading_ned_deg": s.get("heading_ned_deg"),
                    "xtrack_m": s.get("xtrack_m"),
                    "heading_err_deg": s.get("heading_err_deg"),
                    "lookahead_m": s.get("lookahead_m"),
                    "speed_m_s": s.get("speed_m_s"),
                    "measured_speed_m_s": s.get("measured_speed_m_s"),
                    "kappa": s.get("kappa"),
                    "dist_to_goal_m": s.get("dist_to_goal_m"),
                    "pose_age_ms": s.get("pose_age_ms"),
                    "rpp_state": code,
                    "rpp_state_name": RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    **spray_fields,
                    "armed": s.get("armed"),
                    "mode": s.get("mode"),
                    "connected": s.get("connected"),
                    "battery_v": s.get("battery_v"),
                    "battery_pct": s.get("battery_pct"),
                    "gps_fix": s.get("gps_fix"),
                    "gps_fix_name": GPS_FIX_NAMES.get(s.get("gps_fix", 0), "UNKNOWN"),
                    "gps_sat": s.get("gps_sat"),
                    "hrms": s.get("hrms"),
                    "vrms": s.get("vrms"),
                    "lat": s.get("lat"),
                    "lon": s.get("lon"),
                    "alt": s.get("alt"),
                }
                if joystick_ctrl is not None:
                    telem.update(joystick_ctrl.snapshot())
                await _emit_authenticated("telemetry", _sanitize(telem))

                mission_status = {
                    "state": (offboard_ctrl.state.value if offboard_ctrl else "idle"),
                    "rpp_state": code,
                    "rpp_state_name": RPP_STATE_NAMES.get(code, "UNKNOWN"),
                    "dist_to_goal": s.get("dist_to_goal_m"),
                    "speed": s.get("speed_m_s"),
                    "xtrack": s.get("xtrack_m"),
                    "rpp_debug_age_ms": s.get("rpp_debug_age_ms"),
                    "rpp_debug_fresh": s.get("rpp_debug_fresh"),
                    "measured_speed_m_s": s.get("measured_speed_m_s"),
                }
                await _emit_authenticated("mission_status", _sanitize(mission_status))

                # ── 2. Auto-completion: continuous/dash only (Point owns completion) ─
                if _should_global_rpp_complete(offboard_ctrl, point_mission, ros_node):
                    from mission_ops import MissionOperation, MissionOperationConflict

                    coordinator = operation_coordinator
                    if coordinator is None:
                        from mission_ops import MissionOperationCoordinator

                        coordinator = MissionOperationCoordinator()
                    try:
                        token = await coordinator.begin(
                            MissionOperation.COMPLETION, timeout_s=0.25
                        )
                    except MissionOperationConflict:
                        token = None
                    if token is not None and not token.is_preempted():
                        try:
                            if coordinator.is_current(token):
                                completion = await offboard_ctrl.complete_async()
                                completion_ok = bool(completion.get("success"))
                                terminal_reason = (
                                    "mission_completed"
                                    if completion_ok
                                    else "mission_completion_degraded"
                                )
                                if mission_capture is not None:
                                    mission_capture.record_terminal(
                                        None,
                                        terminal_reason,
                                        state=offboard_ctrl.state.value,
                                        details=completion,
                                    )
                                await _emit_authenticated(
                                    terminal_reason,
                                    {
                                        "state": offboard_ctrl.state.value,
                                        "name": offboard_ctrl.loaded_path_name,
                                        "terminal": completion,
                                    },
                                )
                        finally:
                            await coordinator.finish(token)

                # ── 3. Watchdog: RUNNING + unhealthy/disconnected → estop ──────
                # B2: RPP_UNHEALTHY_CODES covers STALE (-1), RTK_WAIT (4),
                # JUMP_SKIP (5). All three mean "controller is publishing
                # zero velocity for a safety reason" — same response.
                pose_age = s.get("pose_age_ms") or 0.0
                running = (
                    offboard_ctrl is not None
                    and offboard_ctrl.state == MissionState.RUNNING
                )
                unhealthy = (
                    code in RPP_UNHEALTHY_CODES
                    or pose_age > POSE_STALE_MS
                    or s.get("connected") is False
                    or s.get("rpp_debug_fresh") is not True
                )
                if running and unhealthy:
                    if stale_since is None:
                        stale_since = now
                    elif now - stale_since > SAFETY_STALE_GRACE_S:
                        if emergency_handler is not None:
                            rpp_name = RPP_STATE_NAMES.get(code, f"?{code}")
                            log.warning(
                                "safety abort: stale=%.0fms rpp=%s(%s) connected=%s",
                                pose_age,
                                code,
                                rpp_name,
                                s.get("connected"),
                            )
                            await emergency_handler.estop_async()
                            await _emit_authenticated(
                                "safety_abort",
                                {
                                    "reason": "pose stale or FCU disconnected",
                                    "pose_age_ms": pose_age,
                                    "rpp_state": code,
                                    "rpp_state_name": RPP_STATE_NAMES.get(
                                        code, "UNKNOWN"
                                    ),
                                    "connected": s.get("connected"),
                                },
                            )
                        stale_since = None
                else:
                    stale_since = None

                # Joystick MANUAL mode has a 500 ms PX4 RC_LOSS fallback. On
                # confirmed FCU/MAVROS disconnect, revoke ownership and run the
                # established hard e-stop path.
                if (
                    joystick_ctrl is not None
                    and joystick_ctrl.is_active
                    and s.get("connected") is False
                ):
                    await joystick_ctrl.force_release(reason="fcu_disconnected")
                    if emergency_handler is not None:
                        await emergency_handler.estop_async()

                # ── 3b. GPS_SURVEYED continuous/dash runtime gate (F-01/F-02) ──
                # Point missions self-gate in the orchestrator; LOCAL_NED is
                # never RTK-gated. For a RUNNING surveyed line/dash mission we
                # re-evaluate GPS safety each tick, feed the result to the spray
                # node's independent gate, and on fault force spray OFF + e-stop
                # (the rover must not keep driving on degraded localization).
                gps_gate_seq += 1
                gctx = (
                    offboard_ctrl.gps_surveyed_runtime_context()
                    if offboard_ctrl is not None else None
                )
                if gctx is not None:
                    verdict = evaluate_gps_surveyed_safety(
                        s,
                        gctx["origin_gps"],
                        gctx["source_points"],
                        GpsSurveyedSafetyParams(),
                        recovery_since=None,
                        fault_count=gps_gate_fault_count,
                        last_fault_time_s=gps_gate_last_fault_time,
                    )
                    if not verdict.ok:
                        gps_gate_fault_count += 1
                        gps_gate_last_fault_time = now
                        ros_node.publish_gps_gate(
                            active=True, ok=False,
                            reason=verdict.reason, seq=gps_gate_seq,
                        )
                        log.warning(
                            "GPS_SURVEYED runtime fault (%s mission): %s",
                            gctx["spray_mode"], verdict.reason,
                        )
                        try:
                            from spray_safety import force_spray_off_confirmed

                            spray_off = await force_spray_off_confirmed(
                                ros_node, timeout_s=2.0
                            )
                            if not spray_off.success and spray_off.live:
                                log.warning(
                                    "GPS fault spray OFF not confirmed: %s",
                                    spray_off.as_dict(),
                                )
                        except Exception:
                            log.exception("force spray OFF during GPS fault failed")
                        if emergency_handler is not None:
                            await emergency_handler.estop_async()
                        if mission_capture is not None:
                            mission_capture.record_terminal(
                                None, "gps_safety_abort",
                                state=offboard_ctrl.state.value,
                                details={
                                    "reason": verdict.reason,
                                    "spray_mode": gctx["spray_mode"],
                                },
                            )
                        await _emit_authenticated(
                            "gps_safety_abort",
                            {
                                "reason": verdict.reason,
                                "spray_mode": gctx["spray_mode"],
                                "gps_fix": verdict.current_fix_type,
                                "manual_resume_required": True,
                            },
                        )
                    else:
                        gps_gate_fault_count = 0
                        ros_node.publish_gps_gate(
                            active=True, ok=True, reason="", seq=gps_gate_seq,
                        )
                else:
                    gps_gate_fault_count = 0
                    ros_node.publish_gps_gate(
                        active=False, ok=True, reason="", seq=gps_gate_seq,
                    )

                # ── 4. Disconnect notification (transition: was connected) ─────
                connected = bool(s.get("connected", False))
                if prev_connected is True and not connected:
                    await _emit_authenticated("rover_disconnected", {})
                    _record("warning", "FCU disconnected")
                prev_connected = connected

                consecutive_errors = 0

                # ── 5. Systemd watchdog heartbeat ──────────────────────────────
                _watchdog_counter += 1
                if _sd_notifier and _watchdog_counter >= _WATCHDOG_EVERY_N:
                    _sd_notifier.notify("WATCHDOG=1")
                    _watchdog_counter = 0

            except asyncio.CancelledError:
                raise
            except Exception:
                consecutive_errors += 1
                log.exception(
                    "telemetry loop iteration failed (n=%d)", consecutive_errors
                )
                # Exponential back-off on repeated failures, capped at 1 s
                await asyncio.sleep(min(1.0, 0.05 * consecutive_errors))
    finally:
        log.info("telemetry loop exited")


# ── Internal helper ───────────────────────────────────────────────────────────


def _record(level: str, message: str) -> None:
    activity_log.append(
        {
            "timestamp": datetime.datetime.utcnow().isoformat(timespec="seconds") + "Z",
            "level": level,
            "message": message,
        }
    )
    getattr(log, level if level in ("info", "warning", "error", "debug") else "info")(
        message
    )
