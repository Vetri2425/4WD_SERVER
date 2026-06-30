#!/usr/bin/env python3
"""Verified GPS ingest validation tests."""

from __future__ import annotations

import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from verified_gps_ingest import DUPLICATE_TOLERANCE_M_DEFAULT, validate_all
from verified_mission.ingest import ingest_verified_waypoints


def _wp(index: int, lat: float, lon: float, *, mark: bool = True, dwell_s=None):
    payload = {
        "index": index,
        "lat": lat,
        "lon": lon,
        "alt": 0.0,
        "mark": mark,
    }
    if dwell_s is not None:
        payload["dwell_s"] = dwell_s
    return payload


def test_validate_all_accepts_contiguous_indices_and_mark():
    waypoints = [_wp(0, 37.0, -122.0), _wp(1, 37.0001, -122.0001, mark=False, dwell_s=0.0)]
    result = validate_all(waypoints)
    assert len(result) == 2
    assert result[1].mark is False
    assert result[1].dwell_s == 0.0


def test_validate_all_rejects_missing_mark():
    with pytest.raises(ValueError, match="mark is required"):
        validate_all([{"index": 0, "lat": 1.0, "lon": 2.0, "alt": 0.0}])


def test_validate_all_rejects_non_contiguous_indices():
    with pytest.raises(ValueError, match="contiguous"):
        validate_all([_wp(0, 37.0, -122.0), _wp(2, 37.0001, -122.0001)])


def test_validate_all_rejects_near_duplicate_coords():
    base = 37.0, -122.0
    near = base[0], base[1] + (DUPLICATE_TOLERANCE_M_DEFAULT / 111_000.0)
    with pytest.raises(ValueError, match="duplicate"):
        validate_all([_wp(0, *base), _wp(1, *near)])


def test_validate_all_does_not_apply_hard_half_meter_rule_by_default():
    # ~0.1 m apart in latitude — must pass without min_sep_m.
    waypoints = [
        _wp(0, 37.0, -122.0),
        _wp(1, 37.0 + (0.1 / 111_000.0), -122.0),
    ]
    assert len(validate_all(waypoints)) == 2


def test_validate_all_applies_min_sep_only_when_provided():
    waypoints = [
        _wp(0, 37.0, -122.0),
        _wp(1, 37.0 + (0.1 / 111_000.0), -122.0),
    ]
    with pytest.raises(ValueError, match="min_sep_m"):
        validate_all(waypoints, min_sep_m=0.5)
    assert len(validate_all(waypoints, min_sep_m=0.05)) == 2


def test_validate_all_rejects_whole_mission_atomically():
    bad = [
        _wp(0, 37.0, -122.0),
        {"index": 1, "lat": "bad", "lon": -122.0, "alt": 0.0, "mark": True},
    ]
    with pytest.raises(ValueError):
        validate_all(bad)


def test_server_ingest_wrapper_honors_settings_min_sep():
    waypoints = [
        _wp(0, 37.0, -122.0),
        _wp(1, 37.0 + (0.1 / 111_000.0), -122.0),
    ]
    with pytest.raises(ValueError, match="min_sep_m"):
        ingest_verified_waypoints(waypoints, settings={"min_sep_m": 0.5})