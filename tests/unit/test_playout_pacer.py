# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Playout pacer core — schedule properties on synthetic arrival traces."""

from __future__ import annotations

import random
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
WS_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py"
sys.path.insert(0, str(WS_PY))

from com_py.playout_pacer_core import PacerConfig, PlayoutSchedule  # noqa: E402


def _run(schedule: PlayoutSchedule, stamps, delays):
    releases = []
    for stamp, d in zip(stamps, delays, strict=True):
        releases.append(schedule.on_message(stamp, stamp + d))
    return releases


def test_jittered_arrivals_release_on_a_regular_grid() -> None:
    # 10 Hz stream, delay jitter far above the frame period: released cadence
    # must be the sender cadence, not the arrival cadence.
    rng = random.Random(1)
    stamps = [i / 10 for i in range(300)]
    delays = [0.030 + rng.random() * 0.120 for _ in stamps]  # 30-150 ms
    schedule = PlayoutSchedule(PacerConfig(adaptive=True, min_ms=100.0, max_ms=800.0))
    releases = _run(schedule, stamps, delays)
    gaps = [b - a for a, b in zip(releases[50:], releases[51:], strict=False)]
    assert max(gaps) < 0.150  # no visible stutter (>150 ms) left
    assert sum(1 for g in gaps if abs(g - 0.1) < 0.01) / len(gaps) > 0.95


def test_late_message_passes_through_immediately_and_order_is_kept() -> None:
    schedule = PlayoutSchedule(PacerConfig(adaptive=False, target_ms=200.0))
    stamps = [i / 10 for i in range(50)]
    delays = [0.030] * 50
    delays[25] = 0.700  # one message far beyond any budget
    releases = _run(schedule, stamps, delays)
    # Late message is released at its arrival, not held further...
    assert releases[25] == pytest.approx(stamps[25] + 0.700)
    # ...and ordering is never violated.
    assert all(b >= a for a, b in zip(releases, releases[1:], strict=False))


def test_budget_is_clock_offset_proof() -> None:
    # Same jitter, sender clock shifted by -3 s: the budget must not explode.
    rng = random.Random(2)
    delays = [0.030 + rng.random() * 0.060 for _ in range(200)]
    plain = PlayoutSchedule()
    shifted = PlayoutSchedule()
    for i, d in enumerate(delays):
        plain.on_message(i / 10, i / 10 + d)
        shifted.on_message(i / 10 - 3.0, i / 10 + d)  # stamp 3 s in the "past"
    assert shifted.budget_ms == pytest.approx(plain.budget_ms, rel=0.05)
    assert shifted.budget_ms < 800.0


def test_adaptive_budget_tracks_jitter_and_respects_clamps() -> None:
    calm = PlayoutSchedule(PacerConfig(min_ms=100.0, max_ms=800.0, margin_ms=40.0))
    rough = PlayoutSchedule(PacerConfig(min_ms=100.0, max_ms=800.0, margin_ms=40.0))
    rng = random.Random(3)
    for i in range(300):
        calm.on_message(i / 10, i / 10 + 0.030 + rng.random() * 0.010)
        rough.on_message(i / 10, i / 10 + 0.030 + rng.random() * 0.400)
    assert calm.budget_ms == pytest.approx(100.0)  # clamped at min
    assert rough.budget_ms > 300.0
    assert rough.budget_ms <= 800.0


def test_config_validation() -> None:
    with pytest.raises(ValueError):
        PacerConfig(percentile=1.5)
    with pytest.raises(ValueError):
        PacerConfig(min_ms=500.0, max_ms=100.0)
    with pytest.raises(ValueError):
        PacerConfig(window=3)


# The hold the node reports (#301).
#
# `.../paced/hold_ms` carries what this node actually added, so the status
# overview can take it back out of a decoded stage's age and judge what is left
# -- the link. The node computes `max(0, release - arrival)`; both halves of
# that clamp are properties of the schedule, so they are asserted here rather
# than in three lines of publishing glue.


def _hold_ms(schedule: PlayoutSchedule, stamp: float, arrival: float) -> float:
    """What the node publishes for a message: the delay it applied, never negative."""
    return max(0.0, (schedule.on_message(stamp, arrival) - arrival) * 1000.0)


def test_the_reported_hold_is_what_was_added_not_what_was_allowed() -> None:
    schedule = PlayoutSchedule(PacerConfig(adaptive=False, target_ms=200.0))
    # First message sets the floor at its own delay, so it is released at
    # floor + 200 ms and waits exactly the budget.
    assert _hold_ms(schedule, 0.0, 0.030) == pytest.approx(200.0, abs=1.0)
    # One that arrives 150 ms later than the floor has already spent 150 of the
    # budget on the wire and waits only the remainder.
    assert _hold_ms(schedule, 1.0, 1.180) == pytest.approx(50.0, abs=1.0)


def test_a_late_message_is_reported_as_held_for_nothing() -> None:
    """The clamp: a pass-through must not be credited with a buffer it skipped."""
    schedule = PlayoutSchedule(PacerConfig(adaptive=False, target_ms=200.0))
    _hold_ms(schedule, 0.0, 0.030)

    assert _hold_ms(schedule, 1.0, 1.400) == 0.0


# The floor has to be able to rise again (#312).
#
# Every budget is anchored to `d_floor`, the fastest path observed. Taking that
# over all time rather than over the window means it can only fall, so one
# upward shift in `arrival - stamp` puts every later message past its release
# time and the pacer becomes a pass-through it never recovers from. A clock step
# on either peer is enough; the fleet e2e's looping bag replay does it every
# lap, and until #301 the symptom was invisible -- node up, topic flowing,
# nothing re-timed.


def _hold_p50(schedule: PlayoutSchedule, count: int, stamp0: float, offset: float) -> float:
    holds = []
    for index in range(count):
        stamp = stamp0 + index * 0.1
        # 10 ms most of the time, 50 ms every third message: enough jitter for
        # an adaptive budget to have something to track.
        arrival = stamp + offset + (0.05 if index % 3 == 0 else 0.01)
        holds.append(max(0.0, (schedule.on_message(stamp, arrival) - arrival) * 1000.0))
    return sorted(holds)[len(holds) // 2]


def test_a_delay_step_up_is_re_anchored_within_one_window() -> None:
    schedule = PlayoutSchedule(PacerConfig(adaptive=True, min_ms=100.0, max_ms=300.0))
    window = schedule.config.window

    assert _hold_p50(schedule, 500, 0.0, 0.20) == pytest.approx(100.0, abs=5.0)
    # The step itself is absorbed, not smoothed: those messages are genuinely
    # late relative to what was known, and pass through.
    _hold_p50(schedule, window, 1000.0, 161.20)
    # One window later the floor describes the new path again.
    assert _hold_p50(schedule, 500, 2000.0, 161.20) == pytest.approx(100.0, abs=5.0)


def test_a_constant_offset_is_still_absorbed_entirely() -> None:
    """The property the floor exists for: no clock synchronization needed."""
    near = PlayoutSchedule(PacerConfig(adaptive=True, min_ms=100.0, max_ms=300.0))
    far = PlayoutSchedule(PacerConfig(adaptive=True, min_ms=100.0, max_ms=300.0))

    assert _hold_p50(near, 500, 0.0, 0.20) == pytest.approx(_hold_p50(far, 500, 0.0, 3600.20), abs=1.0)


def test_the_step_neither_drops_nor_reorders() -> None:
    schedule = PlayoutSchedule(PacerConfig(adaptive=True, min_ms=100.0, max_ms=300.0))
    releases = []
    for index in range(600):
        stamp = index * 0.1
        offset = 0.20 if index < 300 else 161.20
        releases.append(schedule.on_message(stamp, stamp + offset + 0.01))

    assert len(releases) == 600
    assert releases == sorted(releases)
