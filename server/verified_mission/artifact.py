"""Verified GPS mission artifact persisted on disk."""

from __future__ import annotations

import datetime
from dataclasses import dataclass, field
from typing import Any


def _utc_now_iso() -> str:
    return datetime.datetime.utcnow().isoformat(timespec="milliseconds") + "Z"


@dataclass
class VerifiedMissionArtifact:
    mission_id: str
    mission_name: str
    waypoints: list[dict[str, Any]]
    total_targets: int
    created_at: str = field(default_factory=_utc_now_iso)
    settings: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "mission_id": self.mission_id,
            "mission_name": self.mission_name,
            "waypoints": list(self.waypoints),
            "total_targets": self.total_targets,
            "created_at": self.created_at,
            "settings": dict(self.settings),
        }

    @classmethod
    def from_dict(cls, payload: dict[str, Any]) -> VerifiedMissionArtifact:
        mission_id = str(payload.get("mission_id") or "")
        if not mission_id:
            raise ValueError("mission_id is required")
        waypoints = payload.get("waypoints")
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError("waypoints must be a non-empty list")
        total_targets = int(payload.get("total_targets") or len(waypoints))
        return cls(
            mission_id=mission_id,
            mission_name=str(payload.get("mission_name") or mission_id),
            waypoints=waypoints,
            total_targets=total_targets,
            created_at=str(payload.get("created_at") or _utc_now_iso()),
            settings=dict(payload.get("settings") or {}),
        )