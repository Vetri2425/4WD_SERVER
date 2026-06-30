#!/usr/bin/env python3
"""Verified mission store tests."""

from __future__ import annotations

import os
import sys
import time

import pytest

sys.path.insert(0, os.path.dirname(__file__))

from verified_mission.store import VerifiedMissionStore


def _waypoints():
    return [
        {"index": 0, "lat": 37.0, "lon": -122.0, "alt": 0.0, "mark": True, "dwell_s": 2.0},
        {"index": 1, "lat": 37.0002, "lon": -122.0002, "alt": 0.0, "mark": False, "dwell_s": 0.0},
    ]


def test_store_save_load_round_trip(tmp_path):
    store = VerifiedMissionStore(str(tmp_path))
    saved = store.save(mission_name="field-a", waypoints=_waypoints(), settings={"mode": "auto"})
    assert saved.mission_id.startswith("vwm_")
    loaded = store.load(saved.mission_id)
    assert loaded.mission_name == "field-a"
    assert loaded.total_targets == 2
    assert loaded.waypoints[1]["mark"] is False


def test_store_rejects_bad_mission_id_prefix(tmp_path):
    store = VerifiedMissionStore(str(tmp_path))
    with pytest.raises(ValueError, match="vwm_"):
        store.save(mission_name="x", waypoints=_waypoints(), mission_id="bad_id")


def test_store_prune_expired_skips_resident(tmp_path):
    store = VerifiedMissionStore(str(tmp_path))
    saved = store.save(mission_name="ttl", waypoints=_waypoints())
    path = store._path_for(saved.mission_id)
    old = time.time() - 10_000
    os.utime(path, (old, old))

    removed = store.prune_expired(ttl_s=60.0, is_resident=lambda mid: mid == saved.mission_id)
    assert removed == []
    assert store.exists(saved.mission_id)

    removed2 = store.prune_expired(ttl_s=60.0, is_resident=lambda _mid: False)
    assert saved.mission_id in removed2
    assert not store.exists(saved.mission_id)