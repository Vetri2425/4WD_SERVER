"""Server wrapper around pure verified GPS ingest validation."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

from config import DUPLICATE_TOLERANCE_M, VERIFIED_MISSION_MIN_SEP_M

_SRC = Path(__file__).resolve().parents[2] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from verified_gps_ingest import (  # noqa: E402
    VerifiedWaypoint,
    validate_all,
    waypoints_to_dict,
)


def ingest_verified_waypoints(
    waypoints: list[dict[str, Any]],
    *,
    settings: dict[str, Any] | None = None,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = DUPLICATE_TOLERANCE_M,
    min_sep_m: float | None = None,
) -> list[VerifiedWaypoint]:
    """Validate upload payload atomically; reject entire mission on any error."""
    cfg = settings or {}
    effective_default = float(cfg.get("default_dwell_s", default_dwell_s))
    effective_max = float(cfg.get("max_dwell_s", max_dwell_s))
    effective_dup = float(cfg.get("duplicate_tolerance_m", duplicate_tolerance_m))

    effective_min_sep = min_sep_m
    if effective_min_sep is None and cfg.get("min_sep_m") is not None:
        effective_min_sep = float(cfg["min_sep_m"])
    if effective_min_sep is None and VERIFIED_MISSION_MIN_SEP_M is not None:
        effective_min_sep = float(VERIFIED_MISSION_MIN_SEP_M)

    return validate_all(
        waypoints,
        default_dwell_s=effective_default,
        max_dwell_s=effective_max,
        duplicate_tolerance_m=effective_dup,
        min_sep_m=effective_min_sep,
    )


def validated_waypoints_as_dict(waypoints: list[VerifiedWaypoint]) -> list[dict[str, Any]]:
    return waypoints_to_dict(waypoints)