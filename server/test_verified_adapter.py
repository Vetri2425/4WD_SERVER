#!/usr/bin/env python3
"""Verified mission adapter tests (mocked ROS/offboard)."""

from __future__ import annotations

import asyncio
import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from models import MissionState
from target_mission.types import TargetMissionState
from verified_mission.adapter import VerifiedMissionOrchestrator
from verified_mission.artifact import VerifiedMissionArtifact
from verified_mission.events import reset_target_event_journal_for_tests


class FakeRos:
    def __init__(self):
        self.state = {
            "pose_received": True,
            "global_position_received": True,
            "gps_fix_received": True,
            "connected": True,
            "pos_n": 0.0,
            "pos_e": 0.0,
            "lat": 37.0,
            "lon": -122.0,
            "speed_m_s": 0.0,
            "measured_speed_m_s": 0.0,
            "yaw_rate_rad_s": 0.0,
            "pose_age_ms": 10.0,
            "velocity_age_ms": 10.0,
            "local_pose_age_ms": 10.0,
            "global_position_age_ms": 10.0,
            "gps_fix_age_ms": 10.0,
            "pose_global_skew_ms": 5.0,
            "gps_fix": 6,
            "rpp_state": 3,
        }
        self.paths = []
        self.dwells = []
        self.auto_arrive = True
        self.live_dwell = None

    def get_state(self):
        return dict(self.state)

    async def cancel_spray_dwell_async(self):
        self.live_dwell = None
        return True, "ok"

    def publish_path(self, points, spray_flags=None, runtime_entry=False):
        self.paths.append((list(points), spray_flags, runtime_entry))
        if self.auto_arrive and points:
            self.state["pos_n"], self.state["pos_e"] = points[-1]

    async def start_spray_dwell_async(self, **kwargs):
        self.dwells.append(kwargs)
        self.live_dwell = {**kwargs, "deadline": time.monotonic() + kwargs["duration_s"]}
        return True, "ok"

    def get_spray_runtime_status(self):
        base = {
            "configuration_revision": 1,
            "model_revision": 0,
            "timestamp_monotonic_s": time.monotonic(),
            "dwell_mission_id": "",
            "dwell_point_index": None,
            "off_acknowledged": True,
            "commanded_on": False,
            "confirmed_off": True,
            "status_stale": False,
            "ready": True,
            "last_error": "",
        }
        if self.live_dwell is not None:
            remaining = self.live_dwell["deadline"] - time.monotonic()
            active = remaining > 0.0
            return {
                **base,
                "active_dwell": active,
                "dwell_remaining_s": max(0.0, remaining),
                "commanded_on": active,
                "confirmed_off": not active,
                "off_acknowledged": not active,
                "dwell_command_id": self.live_dwell.get("command_id"),
                "dwell_mission_id": self.live_dwell.get("mission_id", ""),
                "dwell_point_index": self.live_dwell.get("point_index"),
            }
        return {**base, "active_dwell": False, "dwell_remaining_s": 0.0}


class FakeOffboard:
    state = MissionState.RUNNING
    complete_calls = 0

    async def complete_async(self):
        self.complete_calls += 1
        self.state = MissionState.COMPLETED
        return {"success": True, "message": "ok", "warnings": []}


def _artifact(*, mark_last=True):
    return VerifiedMissionArtifact(
        mission_id="vwm_test123",
        mission_name="test",
        waypoints=[
            {
                "index": 0,
                "lat": 37.0,
                "lon": -122.0,
                "alt": 0.0,
                "mark": True,
                "dwell_s": 0.05,
            },
            {
                "index": 1,
                "lat": 37.0002,
                "lon": -122.0002,
                "alt": 0.0,
                "mark": mark_last,
                "dwell_s": 0.05 if mark_last else 0.0,
            },
        ],
        total_targets=2,
        settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
    )


@pytest.fixture(autouse=True)
def _journal():
    reset_target_event_journal_for_tests()


@pytest.mark.anyio
async def test_verified_adapter_prepare_converts_gps_once(monkeypatch):
    ros = FakeRos()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(_artifact())
    orch.prepare(ros.get_state())
    assert len(orch._resolved_points) == 2
    assert orch._resolved_points[0].north_m == pytest.approx(0.0, abs=1e-6)
    assert orch._resolved_points[1].north_m > 0.0


@pytest.mark.anyio
async def test_verified_adapter_mark_false_skips_dwell(monkeypatch):
    ros = FakeRos()
    offboard = FakeOffboard()
    orch = VerifiedMissionOrchestrator()
    artifact = _artifact(mark_last=False)
    artifact.waypoints[0]["mark"] = False
    artifact.waypoints[0]["dwell_s"] = 0.0
    orch.load_artifact(artifact)
    await orch.start(ros, offboard)
    await asyncio.sleep(0.4)
    assert ros.dwells == []
    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    types = [e.event_type for e in events]
    assert "target_marking" not in types
    # Intermediate mark=false target must still emit exactly one (non-terminal)
    # completion so the frontend progress map does not stall on that index.
    target0_completions = [
        e
        for e in events
        if e.event_type == "target_completed" and e.target_index == 0
    ]
    assert len(target0_completions) == 1
    assert target0_completions[0].terminal is False
    # Final mark=false target completes via the single terminal event only.
    terminal = [e for e in events if e.terminal]
    assert len(terminal) == 1
    assert terminal[0].event_type == "target_completed"
    assert terminal[0].target_index == 1


@pytest.mark.anyio
async def test_verified_adapter_final_mark_true_completion_is_terminal_only():
    ros = FakeRos()
    offboard = FakeOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_final_mark",
            mission_name="final-mark",
            waypoints=[
                {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": True, "dwell_s": 0.05},
            ],
            total_targets=1,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    await orch.start(ros, offboard)
    await asyncio.sleep(0.4)

    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    completions = [e for e in events if e.event_type == "target_completed"]
    assert len(completions) == 1
    assert completions[0].terminal is True
    assert completions[0].mission_outcome == "completed"


@pytest.mark.anyio
async def test_verified_adapter_final_mark_true_no_success_when_completion_fails():
    ros = FakeRos()

    class FailingOffboard(FakeOffboard):
        async def complete_async(self):
            self.complete_calls += 1
            return {"success": False, "message": "degraded", "warnings": []}

    offboard = FailingOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_final_mark_fail",
            mission_name="final-mark-fail",
            waypoints=[
                {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": True, "dwell_s": 0.05},
            ],
            total_targets=1,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    await orch.start(ros, offboard)
    await asyncio.sleep(0.4)

    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    assert not [e for e in events if e.event_type == "target_completed"]
    terminal = [e for e in events if e.terminal]
    assert len(terminal) == 1
    assert terminal[0].event_type == "target_failed"
    assert terminal[0].mission_outcome == "failed"


@pytest.mark.anyio
async def test_verified_adapter_final_skipped_target_terminal_is_not_completed():
    ros = FakeRos()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_final_skip",
            mission_name="final-skip",
            waypoints=[
                {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": False, "dwell_s": 0.0},
            ],
            total_targets=1,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    orch.prepare(ros.get_state())
    orch.status.last_completed_point_index = 0
    orch.status.last_skipped_point_index = 0
    orch.status.skipped_point_indices = [0]
    await orch.terminal_cleanup(
        ros,
        None,
        reason="normal_completion",
        terminal_state=TargetMissionState.COMPLETED,
        operation_token=object(),
        require_spray_confirm=True,
    )

    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    terminal = [e for e in events if e.terminal]
    assert len(terminal) == 1
    assert terminal[0].event_type == "target_skipped"
    assert terminal[0].mission_outcome == "completed"
    assert not [e for e in events if e.event_type == "target_completed"]


@pytest.mark.anyio
async def test_verified_adapter_emits_target_active_for_original_indices():
    ros = FakeRos()
    offboard = FakeOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_one",
            mission_name="one",
            waypoints=[
                {
                    "index": 0,
                    "lat": 37.0,
                    "lon": -122.0,
                    "alt": 0.0,
                    "mark": False,
                    "dwell_s": 0.0,
                }
            ],
            total_targets=1,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    await orch.start(ros, offboard)
    await asyncio.sleep(0.3)
    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    assert events[0].event_type == "target_active"
    assert events[0].target_index == 0


@pytest.mark.anyio
async def test_verified_adapter_per_target_completion_mixed_mark():
    """Every original target emits exactly one completion event (mixed mark)."""
    ros = FakeRos()
    offboard = FakeOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_mixed",
            mission_name="mixed",
            waypoints=[
                {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": False, "dwell_s": 0.0},
                {"index": 1, "lat": 37.0002, "lon": -122.0002, "alt": 0.0, "mark": True, "dwell_s": 0.05},
                {"index": 2, "lat": 37.0004, "lon": -122.0004, "alt": 0.0, "mark": False, "dwell_s": 0.0},
            ],
            total_targets=3,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    await orch.start(ros, offboard)
    await asyncio.sleep(0.6)

    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]

    # Exactly one completion lifecycle event per original target index.
    completions_by_index: dict[int, list] = {0: [], 1: [], 2: []}
    for e in events:
        if e.event_type == "target_completed":
            completions_by_index[e.target_index].append(e)
    assert len(completions_by_index[0]) == 1
    assert len(completions_by_index[1]) == 1
    assert len(completions_by_index[2]) == 1

    # target 0 (mark=false, intermediate): single non-terminal target_completed.
    assert completions_by_index[0][0].terminal is False

    # target 1 (mark=true, intermediate): emits target_marking + target_completed.
    t1_types = [e.event_type for e in events if e.target_index == 1]
    assert "target_marking" in t1_types
    assert completions_by_index[1][0].terminal is False

    # final target (mark=false): exactly one terminal completion, no duplicate.
    final_completion = completions_by_index[2][0]
    assert final_completion.terminal is True
    assert final_completion.mission_outcome == "completed"
    terminal = [e for e in events if e.terminal]
    assert len(terminal) == 1
    assert terminal[0].target_index == 2

    # No densified intermediate travel point ever emits a target_event: all
    # events carry an original target index in range.
    assert all(0 <= e.target_index <= 2 for e in events)


@pytest.mark.anyio
async def test_verified_adapter_no_completion_on_failed_target():
    """A failed run must not emit a successful per-target completion."""
    ros = FakeRos()

    class FailingOffboard(FakeOffboard):
        async def complete_async(self):
            self.complete_calls += 1
            return {"success": False, "message": "degraded", "warnings": []}

    offboard = FailingOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_failnav",
            mission_name="failnav",
            waypoints=[
                {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": False, "dwell_s": 0.0},
            ],
            total_targets=1,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    await orch.start(ros, offboard)
    await asyncio.sleep(0.35)

    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    terminal = [e for e in events if e.terminal]
    assert len(terminal) == 1
    assert terminal[0].event_type == "target_failed"
    assert terminal[0].mission_outcome == "failed"
    # No successful completion event for the only (final) target.
    assert not [e for e in events if e.event_type == "target_completed"]
