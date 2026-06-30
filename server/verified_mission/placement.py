"""GPS placement for verified missions (conversion at prepare/start only)."""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

from mission_placement import PlacementError, resolve_surveyed_points
from spray_config import GpsSurveyedSafetyParams

_ROOT = Path(__file__).resolve().parents[2]
_SRC = _ROOT / "src"
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

try:
    from geographiclib.geodesic import Geodesic
except ImportError:  # pragma: no cover - deployment dependency
    Geodesic = None


def _latlon_to_ned(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    if Geodesic is None:
        raise ImportError("geographiclib is required for verified GPS conversion")
    result = Geodesic.WGS84.Inverse(origin_lat, origin_lon, lat, lon)
    dist = result["s12"]
    bearing_rad = math.radians(result["azi1"])
    return dist * math.cos(bearing_rad), dist * math.sin(bearing_rad)


def resolve_verified_targets(
    waypoints: list[dict[str, Any]],
    *,
    live_state: dict[str, Any],
    safety: GpsSurveyedSafetyParams | None = None,
) -> tuple[list[dict[str, Any]], tuple[float, float]]:
    """Convert WGS84 waypoints to live local NED using first waypoint as anchor.

    Returns resolved targets (north_m/east_m + original metadata) and anchor GPS.
    """
    if not waypoints:
        raise PlacementError("verified mission has no waypoints")

    anchor = waypoints[0]
    anchor_lat = float(anchor["lat"])
    anchor_lon = float(anchor["lon"])

    anchor_relative: list[tuple[float, float]] = []
    for wp in waypoints:
        n, e = _latlon_to_ned(
            float(wp["lat"]),
            float(wp["lon"]),
            anchor_lat,
            anchor_lon,
        )
        anchor_relative.append((n, e))

    resolved_ned, _translation = resolve_surveyed_points(
        anchor_relative,
        (anchor_lat, anchor_lon),
        live_state,
        safety=safety,
    )

    resolved_targets: list[dict[str, Any]] = []
    for wp, (north_m, east_m) in zip(waypoints, resolved_ned, strict=True):
        resolved_targets.append(
            {
                **wp,
                "north_m": north_m,
                "east_m": east_m,
                "anchor_lat": anchor_lat,
                "anchor_lon": anchor_lon,
            }
        )
    return resolved_targets, (anchor_lat, anchor_lon)
