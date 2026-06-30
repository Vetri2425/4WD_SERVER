#!/usr/bin/env python3
"""Verified target event journal tests."""

from __future__ import annotations

import asyncio
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from models import VerifiedTargetEvent
from verified_mission.events import TargetEventJournal, reset_target_event_journal_for_tests


def _event(**kwargs) -> VerifiedTargetEvent:
    base = dict(
        timestamp="2026-06-30T00:00:00.000Z",
        event_type="target_active",
        mission_id="vwm_abc",
        target_index=0,
        terminal=False,
    )
    base.update(kwargs)
    return VerifiedTargetEvent(**base)


def test_target_event_journal_capacity_512():
    journal = TargetEventJournal()
    for i in range(600):
        journal.append(_event(target_index=i % 5))
    hist = journal.history()
    assert len(hist["events"]) == 512
    assert hist["latest_event_id"] == 600


def test_target_event_history_evicted_flag():
    journal = TargetEventJournal()
    for _ in range(520):
        journal.append(_event())
    hist = journal.history(since_event_id=500)
    assert hist["history_evicted"] is False
    hist2 = journal.history(since_event_id=1)
    assert hist2["history_evicted"] is True


def test_target_event_includes_mission_outcome_on_terminal():
    journal = reset_target_event_journal_for_tests()
    journal.append(
        _event(
            event_type="target_completed",
            terminal=True,
            mission_outcome="completed",
        )
    )
    event = journal.history()["events"][-1]
    assert event.terminal is True
    assert event.mission_outcome == "completed"


def test_target_events_emit_nonblocking_when_socket_emit_fails():
    async def run():
        journal = TargetEventJournal()
        loop = asyncio.get_running_loop()

        async def bad_emit(_event, _payload):
            raise RuntimeError("socket down")

        journal.configure_emit(loop, bad_emit)
        journal.append(_event())
        await asyncio.sleep(0.01)

    asyncio.run(run())