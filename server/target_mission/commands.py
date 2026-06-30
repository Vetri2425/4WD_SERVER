"""Shared target-mission operator command API (pause/skip/continue/clear)."""

from __future__ import annotations

import asyncio
from typing import Any

from mission_placement import PlacementError
from target_mission.types import (
    PAUSED_STATES,
    TargetMissionRun,
    TargetMissionState,
    _SKIP_ACCEPTED_STATES,
)

try:
    from mission_ops import MissionOperationToken
except ImportError:  # pragma: no cover
    MissionOperationToken = Any  # type: ignore[misc,assignment]


class TargetMissionCommandsMixin:
    """Operator commands shared by point and verified adapters."""

    _DRAIN_TIMEOUT_S = 6.0

    def _empty_status(self, *, last_transition: str, ready: bool = False):
        from point_mission import PointMissionStatus

        return PointMissionStatus(
            state=TargetMissionState.IDLE,
            last_transition=last_transition,
            ready=ready,
        )

    def _refresh_resolved_points(self, state: dict[str, Any]) -> None:
        """Override in adapters: point uses _resolve_points, verified uses prepare."""
        raise NotImplementedError

    async def clear_mission(
        self, ros_node, *, reason: str = "cleared", offboard_ctrl=None
    ) -> None:
        await self.cancel_and_drain(
            ros_node, reason=reason, offboard_ctrl=offboard_ctrl
        )
        self._points = []
        self._resolved_points = []
        self._config = None
        self._run_token = None
        self._task = None
        self._status = self._empty_status(last_transition=reason)

    async def cancel_and_drain(
        self,
        ros_node,
        *,
        reason: str = "cancelled",
        offboard_ctrl=None,
    ) -> None:
        run, task = self._run_token, self._task
        dwell_cancel_result: dict[str, Any] | None = None
        spray_off_result: dict[str, Any] | None = None
        recovery_required = False
        terminal_failure_reason = ""
        try:
            if run is not None:
                run.cancel_event.set()
                run.pause_requested = False
                if run.continue_gate is not None and not run.continue_gate.done():
                    run.continue_gate.cancel()
                if run.resume_gate is not None and not run.resume_gate.done():
                    run.resume_gate.cancel()
                self._write(
                    run,
                    state=TargetMissionState.ABORTING,
                    last_transition=reason,
                    ready=False,
                    waiting_for_continue=False,
                    run_active=False,
                )
            self._invalidate_dwell_identity(run)
            if task is not None and not task.done():
                task.cancel()
                try:
                    await asyncio.wait_for(
                        asyncio.shield(task), timeout=self._DRAIN_TIMEOUT_S
                    )
                except (asyncio.CancelledError, asyncio.TimeoutError, Exception):
                    pass
        finally:
            if ros_node is not None:
                try:
                    dwell_cancel_result = await self._cancel_dwell_service(ros_node)
                except Exception as exc:
                    dwell_cancel_result = {"success": False, "message": str(exc)}
                try:
                    spray_off_result = await self._force_spray_off_with_result(
                        ros_node, require_confirm=True
                    )
                except Exception as exc:
                    spray_off_result = {
                        "success": False,
                        "message": str(exc),
                        "recovery_required": True,
                    }
            if dwell_cancel_result is not None and not dwell_cancel_result.get(
                "success", False
            ):
                recovery_required = True
                terminal_failure_reason = (
                    dwell_cancel_result.get("message")
                    or "dwell cancel failed during cancellation"
                )
            if spray_off_result is not None and (
                spray_off_result.get("recovery_required")
                or not spray_off_result.get("success", False)
            ):
                recovery_required = True
                terminal_failure_reason = (
                    spray_off_result.get("message")
                    or terminal_failure_reason
                    or "spray OFF not confirmed during cancellation"
                )
            if self._run_token is run:
                self._write(
                    run,
                    run_active=False,
                    waiting_for_continue=False,
                    dwell_cancel_result=dwell_cancel_result,
                    spray_off_result=spray_off_result,
                    recovery_required=recovery_required,
                    terminal_safety_ok=not recovery_required,
                    terminal_failure_reason=terminal_failure_reason,
                )
                self._task = None
                self._run_token = None

    async def abort(self, ros_node, offboard_ctrl=None) -> None:
        await self.cancel_and_drain(
            ros_node, reason="abort", offboard_ctrl=offboard_ctrl
        )

    def reset_for_restart(self, expected_mission_id: str) -> int:
        from point_mission import PointMissionStatus

        if self._status.mission_id and self._status.mission_id != expected_mission_id:
            raise ValueError("restart rejected: mission identity mismatch")
        self._generation += 1
        self._terminal_cleanup_reason = None
        self._resolved_points = []
        self._spray_ever_on = False
        self._run_token = None
        self._task = None
        self._status = PointMissionStatus(
            state=TargetMissionState.IDLE,
            mission_id=expected_mission_id or self._status.mission_id,
            generation=self._generation,
            total_points=len(self._points),
            ready=bool(self._points),
            last_transition="restart_reset",
            source_frame=self._source_frame,
            point_execution_mode=self._execution_mode.value,
            parent_mission_id=expected_mission_id or self._status.mission_id,
            point_mission_generation=self._generation,
        )
        return self._generation

    async def skip_point(
        self,
        ros_node,
        hold_owner,
        *,
        point_index: int,
        expected_generation: int | None,
        reason: str,
        operation_token: MissionOperationToken,
    ) -> tuple[bool, str, int]:
        coordinator = self._operation_coordinator()
        if coordinator is not None:
            try:
                operation_token.raise_if_stale(coordinator.current_generation())
            except Exception:
                return False, "mission operation token is stale", 409
            if operation_token.is_preempted():
                return False, "skip preempted by higher-priority operation", 409
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no target mission loaded", 409
        if not self.is_active():
            return False, "target mission is not active", 409
        if expected_generation is not None and expected_generation != self._generation:
            return False, "stale target mission generation", 409
        if point_index != self._status.current_point_index:
            return False, "skip point_index does not match active point", 409
        if self._status.state == TargetMissionState.WAITING_FOR_CONTINUE:
            return False, "target already completed; use continue", 409
        if self._status.state == TargetMissionState.COMPLETED:
            return False, "target mission already completed", 409
        if self._status.state == TargetMissionState.PAUSED_GPS_SAFETY:
            return False, "GPS safety blocks skip until recovery", 409
        if self._status.state not in _SKIP_ACCEPTED_STATES:
            return (
                False,
                f"target mission is {self._status.state.value}, skip not accepted",
                409,
            )
        if self._status.skip_pending:
            return False, "skip already pending", 409
        run = self._run_token
        if run is None:
            return False, "target mission is not active", 409
        if coordinator is not None:
            run.operation_generation = coordinator.current_generation()
        async with self._command_lock:
            run.skip_requested = True
            run.skip_request_id = point_index
            run.skip_reason = reason
            self._write(run, skip_pending=True)
            if self._status.state in PAUSED_STATES:
                is_last = self._status.current_point_index >= len(self._resolved_points) - 1
                ok, msg = await self._skip_cycle(
                    run, ros_node, hold_owner, self._status.current_point_index, is_last
                )
                if not ok:
                    return False, msg, 503
        return True, "skip accepted", 200

    async def pause_mission(self, ros_node, hold_owner) -> tuple[bool, str, int]:
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no target mission loaded", 409
        if self._status.state in {
            TargetMissionState.COMPLETED,
            TargetMissionState.FAILED,
            TargetMissionState.ABORTING,
        }:
            return False, f"target mission is terminal: {self._status.state.value}", 409
        if self.is_paused():
            return False, "target mission already paused", 409
        if not self.is_active():
            return False, "target mission is not active", 409
        run = self._run_token
        if run is None:
            return False, "target mission is not active", 409
        run.pause_requested = True
        await asyncio.sleep(0)
        return True, "pause requested", 200

    async def resume_mission(
        self,
        ros_node,
        hold_owner,
        *,
        expected_generation: int | None = None,
    ) -> tuple[bool, str, int]:
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no target mission loaded", 409
        if expected_generation is not None and expected_generation != self._generation:
            return False, "stale target mission generation", 409
        if not self.is_paused():
            return False, f"target mission is {self._status.state.value}, not paused", 409
        obstacle_blocked, obstacle_state = self._write_obstacle_status(self._run_token)
        if obstacle_blocked:
            return False, f"obstacle {obstacle_state} — cannot resume", 409
        run = self._run_token
        if self._status.state == TargetMissionState.PAUSED_GPS_SAFETY:
            verdict = self._evaluate_gps_safety(
                ros_node.get_state(),
                recovery_since=self._gps_recovery_since,
                paused=True,
            )
            self._write_gps_verdict(run, verdict)
            if not verdict.recovery_ready:
                return False, "GPS placement not stable for recovery", 409
            try:
                self._refresh_resolved_points(ros_node.get_state())
            except PlacementError as exc:
                return False, str(exc), 409
        if hold_owner is None or not hold_owner.active:
            return False, "hold is not active", 409
        if not self._resume_health_ok(ros_node):
            return False, "pose or FCU telemetry not healthy for resume", 409
        if run is None or not self.is_active():
            return False, "target mission is not active", 409
        gate = run.resume_gate
        if gate is None or gate.done():
            return False, "target mission is not awaiting resume", 409
        gate.set_result(True)
        await asyncio.sleep(0)
        return True, "resume accepted", 200

    def _resume_health_ok(self, ros_node) -> bool:
        if ros_node is None:
            return False
        state = ros_node.get_state()
        if not state.get("pose_received", False):
            return False
        if not state.get("connected", False):
            return False
        if self._telemetry_stale(state):
            return False
        return True

    async def continue_point(self, ros_node=None) -> tuple[bool, str, int]:
        reject = self._terminal_reject()
        if reject is not None:
            return reject
        if self._config is None or not self._points:
            return False, "no target mission loaded", 409
        if self.is_paused():
            return False, "target mission is paused", 409
        obstacle_blocked, obstacle_state = self._write_obstacle_status(self._run_token)
        if obstacle_blocked:
            return False, f"obstacle {obstacle_state} — cannot continue", 409
        if self._status.state == TargetMissionState.COMPLETED:
            return False, "target mission already completed", 409
        if self._status.state in {TargetMissionState.FAILED, TargetMissionState.ABORTING}:
            return False, f"target mission is terminal: {self._status.state.value}", 409
        if self._status.state != TargetMissionState.WAITING_FOR_CONTINUE:
            return (
                False,
                f"target mission is {self._status.state.value}, not waiting for continue",
                409,
            )
        run = self._run_token
        if run is None or not self.is_active():
            return False, "target mission is not active", 409
        if self._gps_applies():
            if ros_node is None:
                try:
                    from main import ros_node as main_ros
                except Exception:
                    main_ros = None
                ros_node = main_ros
            if ros_node is None:
                return False, "GPS safety blocks continue: telemetry unavailable", 409
            verdict = self._evaluate_gps_safety(ros_node.get_state())
            self._write_gps_verdict(run, verdict)
            if not verdict.ok:
                return False, f"GPS safety blocks continue: {verdict.reason}", 409
        gate = run.continue_gate
        if gate is None or gate.done():
            return False, "target mission is not awaiting continue", 409
        gate.set_result(True)
        await asyncio.sleep(0)
        return True, "continue accepted", 200