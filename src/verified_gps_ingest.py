"""Verified GPS mission waypoint validation (pure, fail-closed)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

DUPLICATE_TOLERANCE_M_DEFAULT = 1e-3


@dataclass(frozen=True)
class VerifiedWaypoint:
    index: int
    lat: float
    lon: float
    alt: float
    mark: bool
    dwell_s: float | None = None
    block: str | None = None
    row: str | None = None
    pile: str | None = None
    label: str | None = None


def _finite_coord(name: str, value: Any) -> float:
    try:
        num = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(num):
        raise ValueError(f"{name} must be finite")
    return num


def _require_bool(name: str, value: Any) -> bool:
    if isinstance(value, bool):
        return value
    raise ValueError(f"{name} is required and must be a boolean")


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Approximate geodesic distance in metres (WGS84 sphere)."""
    r = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2.0 * r * math.asin(min(1.0, math.sqrt(a)))


def _validate_effective_dwell(
    dwell_s: float,
    *,
    mark: bool,
    index: int,
    max_dwell_s: float,
) -> None:
    if not math.isfinite(dwell_s):
        raise ValueError(f"waypoint {index}: dwell_s must be finite")
    if mark and dwell_s <= 0.0:
        raise ValueError(f"waypoint {index}: dwell_s must be > 0 when mark=true")
    if not mark and dwell_s < 0.0:
        raise ValueError(f"waypoint {index}: dwell_s must be >= 0 when mark=false")
    if dwell_s > max_dwell_s:
        raise ValueError(
            f"waypoint {index}: dwell_s {dwell_s} exceeds maximum {max_dwell_s}"
        )


def _parse_waypoint(raw: dict[str, Any], *, max_dwell_s: float) -> VerifiedWaypoint:
    if not isinstance(raw, dict):
        raise ValueError("each waypoint must be an object")

    if "index" not in raw:
        raise ValueError("waypoint index is required")
    try:
        index = int(raw["index"])
    except (TypeError, ValueError) as exc:
        raise ValueError("waypoint index must be an integer") from exc
    if index < 0:
        raise ValueError(f"waypoint {index}: index must be >= 0")

    lat = _finite_coord(f"waypoint {index} lat", raw.get("lat"))
    lon = _finite_coord(f"waypoint {index} lon", raw.get("lon"))
    if not (-90.0 <= lat <= 90.0):
        raise ValueError(f"waypoint {index}: lat must be in [-90, 90]")
    if not (-180.0 <= lon <= 180.0):
        raise ValueError(f"waypoint {index}: lon must be in [-180, 180]")

    alt = _finite_coord(f"waypoint {index} alt", raw.get("alt", 0.0))

    if "mark" not in raw:
        raise ValueError(f"waypoint {index}: mark is required")
    mark = _require_bool(f"waypoint {index} mark", raw["mark"])

    dwell: float | None = None
    if raw.get("dwell_s") is not None:
        dwell = _finite_coord(f"waypoint {index} dwell_s", raw["dwell_s"])

    optional_str = ("block", "row", "pile", "label")
    extras: dict[str, str | None] = {}
    for key in optional_str:
        val = raw.get(key)
        if val is None:
            extras[key] = None
        elif isinstance(val, str):
            extras[key] = val.strip() or None
        else:
            raise ValueError(f"waypoint {index}: {key} must be a string when provided")

    if dwell is not None:
        _validate_effective_dwell(dwell, mark=mark, index=index, max_dwell_s=max_dwell_s)

    return VerifiedWaypoint(
        index=index,
        lat=lat,
        lon=lon,
        alt=alt,
        mark=mark,
        dwell_s=dwell,
        block=extras["block"],
        row=extras["row"],
        pile=extras["pile"],
        label=extras["label"],
    )


def validate_all(
    waypoints: list[dict[str, Any]],
    *,
    default_dwell_s: float = 2.0,
    max_dwell_s: float = 60.0,
    duplicate_tolerance_m: float = DUPLICATE_TOLERANCE_M_DEFAULT,
    min_sep_m: float | None = None,
) -> list[VerifiedWaypoint]:
    """Validate a verified GPS mission atomically; reject the whole mission on any error."""
    if default_dwell_s <= 0.0 or not math.isfinite(default_dwell_s):
        raise ValueError("default_dwell_s must be finite and > 0")
    if max_dwell_s <= 0.0 or not math.isfinite(max_dwell_s):
        raise ValueError("max_dwell_s must be finite and > 0")
    if default_dwell_s > max_dwell_s:
        raise ValueError("default_dwell_s exceeds max_dwell_s")
    if duplicate_tolerance_m < 0.0 or not math.isfinite(duplicate_tolerance_m):
        raise ValueError("duplicate_tolerance_m must be finite and >= 0")
    if min_sep_m is not None and (min_sep_m <= 0.0 or not math.isfinite(min_sep_m)):
        raise ValueError("min_sep_m must be finite and > 0 when provided")
    if not waypoints:
        raise ValueError("mission must contain at least one waypoint")

    parsed: list[VerifiedWaypoint] = []
    errors: list[str] = []

    for raw in waypoints:
        try:
            parsed.append(_parse_waypoint(raw, max_dwell_s=max_dwell_s))
        except ValueError as exc:
            errors.append(str(exc))

    if errors:
        raise ValueError("; ".join(errors))

    parsed.sort(key=lambda wp: wp.index)
    expected = list(range(len(parsed)))
    actual = [wp.index for wp in parsed]
    if actual != expected:
        raise ValueError(
            f"waypoint indices must be contiguous starting at 0; got {actual}"
        )

    for i, wp in enumerate(parsed):
        effective = wp.dwell_s if wp.dwell_s is not None else default_dwell_s
        try:
            _validate_effective_dwell(
                effective,
                mark=wp.mark,
                index=wp.index,
                max_dwell_s=max_dwell_s,
            )
        except ValueError as exc:
            errors.append(str(exc))

    for i in range(len(parsed)):
        for j in range(i + 1, len(parsed)):
            a, b = parsed[i], parsed[j]
            dist = _haversine_m(a.lat, a.lon, b.lat, b.lon)
            if dist <= duplicate_tolerance_m:
                errors.append(
                    f"duplicate waypoint at indices {a.index} and {b.index} "
                    f"({dist:.6f} m <= {duplicate_tolerance_m} m)"
                )

    if min_sep_m is not None:
        for i in range(len(parsed) - 1):
            a, b = parsed[i], parsed[i + 1]
            dist = _haversine_m(a.lat, a.lon, b.lat, b.lon)
            if dist < min_sep_m:
                errors.append(
                    f"waypoints {a.index} and {b.index} are {dist:.4f} m apart "
                    f"(< min_sep_m {min_sep_m})"
                )

    if errors:
        raise ValueError("; ".join(errors))

    finalized: list[VerifiedWaypoint] = []
    for wp in parsed:
        effective = wp.dwell_s if wp.dwell_s is not None else default_dwell_s
        finalized.append(
            VerifiedWaypoint(
                index=wp.index,
                lat=wp.lat,
                lon=wp.lon,
                alt=wp.alt,
                mark=wp.mark,
                dwell_s=effective,
                block=wp.block,
                row=wp.row,
                pile=wp.pile,
                label=wp.label,
            )
        )
    return finalized


def waypoints_to_dict(waypoints: list[VerifiedWaypoint]) -> list[dict[str, Any]]:
    return [
        {
            "index": wp.index,
            "lat": wp.lat,
            "lon": wp.lon,
            "alt": wp.alt,
            "mark": wp.mark,
            "dwell_s": wp.dwell_s,
            "block": wp.block,
            "row": wp.row,
            "pile": wp.pile,
            "label": wp.label,
        }
        for wp in waypoints
    ]