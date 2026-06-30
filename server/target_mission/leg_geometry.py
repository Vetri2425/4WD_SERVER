"""Target leg path construction shared by point and future target missions."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2] / "src"
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from point_leg_trajectory import (  # noqa: E402
    PointLegTrajectoryMode,
    build_point_leg_path,
    leg_length_m,
    predict_rpp_conditioning,
)
from spray_config import PointSprayParams  # noqa: E402


def build_target_leg(
    state: dict[str, Any],
    target_north: float,
    target_east: float,
    params: PointSprayParams,
) -> tuple[list[tuple[float, float]], dict[str, Any]]:
    """Build a published leg path and diagnostics for a single target."""
    start = (float(state["pos_n"]), float(state["pos_e"]))
    end = (target_north, target_east)
    mode = PointLegTrajectoryMode.parse(params.leg_trajectory_mode)
    published = build_point_leg_path(
        start,
        end,
        mode=mode,
        spacing_m=params.leg_spacing_m,
    )
    profile, conditioned = predict_rpp_conditioning(
        published,
        runtime_entry=True,
        resample_spacing_m=params.leg_spacing_m,
    )
    return published, {
        "point_leg_trajectory_mode": mode.value,
        "point_leg_spacing_m": params.leg_spacing_m,
        "point_leg_published_count": len(published),
        "point_leg_conditioned_count": len(conditioned),
        "active_trajectory_mode": profile,
        "point_leg_length_m": leg_length_m(start, end),
    }