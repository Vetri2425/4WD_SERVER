"""Verified GPS mission artifact store with TTL pruning."""

from __future__ import annotations

import json
import os
import time
import uuid
from typing import Any, Callable

from config import (
    VERIFIED_MISSION_DIR,
    VERIFIED_MISSION_ID_PREFIX,
    VERIFIED_MISSION_TTL_S,
)
from logging_setup import get_logger
from verified_mission.artifact import VerifiedMissionArtifact

log = get_logger("server.verified_mission.store")


class VerifiedMissionStore:
    def __init__(self, root_dir: str | None = None) -> None:
        self._root = root_dir or VERIFIED_MISSION_DIR
        os.makedirs(self._root, exist_ok=True)

    @property
    def root_dir(self) -> str:
        return self._root

    def _path_for(self, mission_id: str) -> str:
        safe = mission_id.replace("/", "_").replace("\\", "_")
        return os.path.join(self._root, f"{safe}.json")

    def _new_mission_id(self) -> str:
        return f"{VERIFIED_MISSION_ID_PREFIX}{uuid.uuid4().hex[:12]}"

    def save(
        self,
        *,
        mission_name: str,
        waypoints: list[dict[str, Any]],
        settings: dict[str, Any] | None = None,
        mission_id: str | None = None,
    ) -> VerifiedMissionArtifact:
        mid = mission_id or self._new_mission_id()
        if not mid.startswith(VERIFIED_MISSION_ID_PREFIX):
            raise ValueError(
                f"mission_id must start with {VERIFIED_MISSION_ID_PREFIX!r}"
            )
        artifact = VerifiedMissionArtifact(
            mission_id=mid,
            mission_name=mission_name or mid,
            waypoints=list(waypoints),
            total_targets=len(waypoints),
            settings=dict(settings or {}),
        )
        path = self._path_for(mid)
        tmp = path + ".tmp"
        payload = artifact.to_dict()
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
        return artifact

    def load(self, mission_id: str) -> VerifiedMissionArtifact:
        path = self._path_for(mission_id)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"verified mission not found: {mission_id}")
        with open(path, encoding="utf-8") as handle:
            payload = json.load(handle)
        return VerifiedMissionArtifact.from_dict(payload)

    def exists(self, mission_id: str) -> bool:
        return os.path.isfile(self._path_for(mission_id))

    def list_ids(self) -> list[str]:
        ids: list[str] = []
        for name in os.listdir(self._root):
            if not name.endswith(".json") or name.endswith(".tmp"):
                continue
            ids.append(name[:-5])
        return sorted(ids)

    def prune_expired(
        self,
        *,
        ttl_s: float | None = None,
        is_resident: Callable[[str], bool] | None = None,
        now: float | None = None,
    ) -> list[str]:
        """Remove orphan artifacts older than TTL; skip loaded/running missions."""
        limit = float(ttl_s if ttl_s is not None else VERIFIED_MISSION_TTL_S)
        clock = time.time() if now is None else now
        removed: list[str] = []
        resident = is_resident or (lambda _mid: False)

        for mission_id in self.list_ids():
            if resident(mission_id):
                continue
            path = self._path_for(mission_id)
            try:
                age = clock - os.path.getmtime(path)
            except OSError:
                continue
            if age <= limit:
                continue
            try:
                os.remove(path)
                removed.append(mission_id)
            except OSError as exc:
                log.warning("failed to prune verified mission %s: %s", mission_id, exc)
        return removed