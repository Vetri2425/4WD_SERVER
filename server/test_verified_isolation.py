#!/usr/bin/env python3
"""Verified mission isolation: no path_mgr / STAGING_DIR writes."""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from config import STAGING_DIR
from verified_gps_ingest import waypoints_to_dict
from verified_mission.ingest import ingest_verified_waypoints
from verified_mission.store import VerifiedMissionStore


def test_verified_ingest_store_never_touches_staging(tmp_path, monkeypatch):
    verified_dir = tmp_path / "verified"
    monkeypatch.setattr("verified_mission.store.VERIFIED_MISSION_DIR", str(verified_dir))

    waypoints = [
        {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": True, "dwell_s": 2.0},
        {"index": 1, "lat": 37.0003, "lon": -122.0003, "alt": 0.0, "mark": False, "dwell_s": 0.0},
    ]
    validated = ingest_verified_waypoints(waypoints)
    store = VerifiedMissionStore(str(verified_dir))
    saved = store.save(
        mission_name="iso",
        waypoints=waypoints_to_dict(validated),
    )

    assert saved.mission_id.startswith("vwm_")
    assert (verified_dir / f"{saved.mission_id}.json").is_file()
    if os.path.isdir(STAGING_DIR):
        staging_before = set(os.listdir(STAGING_DIR))
    else:
        staging_before = set()
    # Re-save should still not create staging artifacts.
    store.save(
        mission_name="iso2",
        waypoints=waypoints_to_dict(validated),
    )
    if os.path.isdir(STAGING_DIR):
        assert set(os.listdir(STAGING_DIR)) == staging_before


def test_verified_modules_do_not_import_path_manager():
    import verified_mission.adapter as adapter
    import verified_mission.loader as loader
    import verified_mission.placement as placement
    import verified_mission.store as store_mod

    for mod in (adapter, loader, placement, store_mod):
        source = open(mod.__file__, encoding="utf-8").read()
        assert "path_manager" not in source
        assert "path_engine" not in source
        assert "PathEngine" not in source
        assert "STAGING_DIR" not in source
