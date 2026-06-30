#!/usr/bin/env python3
"""Verified terminal event ordering vs complete_async."""

from __future__ import annotations

import asyncio
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from models import MissionState
from verified_mission.adapter import VerifiedMissionOrchestrator
from verified_mission.artifact import VerifiedMissionArtifact
from verified_mission.events import reset_target_event_journal_for_tests
from test_verified_adapter import FakeOffboard, FakeRos


@pytest.fixture(autouse=True)
def _journal():
    reset_target_event_journal_for_tests()


@pytest.mark.anyio
async def test_terminal_success_after_complete_async():
    ros = FakeRos()
    offboard = FakeOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_term",
            mission_name="term",
            waypoints=[
                {
                    "index": 0,
                    "lat": 37.0,
                    "lon": -122.0,
                    "alt": 0.0,
                    "mark": True,
                    "dwell_s": 0.05,
                }
            ],
            total_targets=1,
            settings={"settle_time_s": 0.0, "leg_timeout_s": 2.0, "arrival_tolerance_m": 0.5},
        )
    )
    await orch.start(ros, offboard)
    await asyncio.sleep(0.4)

    assert offboard.complete_calls == 1
    from verified_mission.events import get_target_event_journal

    events = get_target_event_journal().history()["events"]
    terminal = [e for e in events if e.terminal]
    assert len(terminal) == 1
    assert terminal[0].event_type == "target_completed"
    assert terminal[0].mission_outcome == "completed"
    assert terminal[-1].event_id >= events[-2].event_id


@pytest.mark.anyio
async def test_terminal_failure_when_complete_async_fails():
    ros = FakeRos()

    class FailingOffboard(FakeOffboard):
        async def complete_async(self):
            self.complete_calls += 1
            return {"success": False, "message": "degraded", "warnings": ["hold"]}

    offboard = FailingOffboard()
    orch = VerifiedMissionOrchestrator()
    orch.load_artifact(
        VerifiedMissionArtifact(
            mission_id="vwm_fail",
            mission_name="fail",
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
    await asyncio.sleep(0.35)

    from verified_mission.events import get_target_event_journal

    terminal = [e for e in get_target_event_journal().history()["events"] if e.terminal]
    assert terminal[-1].mission_outcome == "failed"
    assert terminal[-1].event_type == "target_failed"