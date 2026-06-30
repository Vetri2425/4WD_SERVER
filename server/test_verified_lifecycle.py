#!/usr/bin/env python3
"""Verified mission lifecycle / loader state tests."""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from models import MissionState
from verified_mission.artifact import VerifiedMissionArtifact
from verified_mission.loader import derive_verified_mission_state, load_verified_artifact_into_controller
from verified_mission.store import VerifiedMissionStore


class FakeOffboard:
    def __init__(self):
        self.loaded_mission_id = None
        self.running_mission_id = None
        self.loaded_mission_kind = "none"
        self._mission_kind = "none"
        self.state = MissionState.IDLE
        self.loaded = []

    def load_path(self, shell, **kwargs):
        self.loaded.append((shell, kwargs))
        self.loaded_mission_id = kwargs.get("mission_id")
        self.loaded_mission_kind = kwargs.get("mission_kind", "none")
        self._mission_kind = self.loaded_mission_kind


class FakeOrchestrator:
    def __init__(self, active=False, terminal_outcome=None):
        self._active = active
        self.status = type("S", (), {"terminal_outcome": terminal_outcome})()

    def is_active(self):
        return self._active

    def load_artifact(self, _artifact):
        pass


def _artifact(mid="vwm_life1"):
    return VerifiedMissionArtifact(
        mission_id=mid,
        mission_name="life",
        waypoints=[
            {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": True, "dwell_s": 2.0}
        ],
        total_targets=1,
    )


def test_derive_state_stored_when_not_resident():
    assert derive_verified_mission_state(_artifact(), offboard_ctrl=None) == "stored"


def test_derive_state_loaded_when_resident_idle():
    ctrl = FakeOffboard()
    load_verified_artifact_into_controller(_artifact(), offboard_ctrl=ctrl)
    assert derive_verified_mission_state(_artifact(), offboard_ctrl=ctrl) == "loaded"


def test_derive_state_running_when_orchestrator_active():
    ctrl = FakeOffboard()
    load_verified_artifact_into_controller(_artifact(), offboard_ctrl=ctrl)
    ctrl.state = MissionState.RUNNING
    ctrl.running_mission_id = "vwm_life1"
    orch = FakeOrchestrator(active=True)
    assert (
        derive_verified_mission_state(_artifact(), offboard_ctrl=ctrl, orchestrator=orch)
        == "running"
    )


def test_load_uses_point_spray_mode_and_verified_kind():
    ctrl = FakeOffboard()
    load_verified_artifact_into_controller(_artifact(), offboard_ctrl=ctrl)
    _, kwargs = ctrl.loaded[0]
    assert kwargs["mission_kind"] == "verified_gps"
    assert kwargs["spray_mode"] == "point"


def test_ensure_load_from_store(tmp_path):
    store = VerifiedMissionStore(str(tmp_path))
    saved = store.save(mission_name="x", waypoints=_artifact().waypoints)
    ctrl = FakeOffboard()
    from verified_mission.loader import ensure_verified_mission_loaded

    artifact = ensure_verified_mission_loaded(
        saved.mission_id,
        offboard_ctrl=ctrl,
        store=store,
    )
    assert artifact.mission_id == saved.mission_id
    assert ctrl.loaded_mission_kind == "verified_gps"