# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Cross-file contracts of the benched set (performance bands reference).

The registry is only a gate if its references hold: every row names a committed
profile, every gated metric has a calibrated band, random elements are seeded,
and the CI lanes actually consume the registry instead of hardcoding rows.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from rosotacom.benched_set import GateRow, load_registry, profiles_for_row
from rosotacom.benchmark import UNCALIBRATED_FINGERPRINT, load_bands
from rosotacom.network_profiles import Profile, load_profiles_file

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
BUDGETS_PATH = PACKAGE_ROOT / "budgets.jsonl"
PROFILES_PATH = PACKAGE_ROOT / "src" / "rosotacom" / "resources" / "examples" / "profiles.yaml"
WORKFLOWS_DIR = PACKAGE_ROOT / ".github" / "workflows"
GATE_WORKFLOW = WORKFLOWS_DIR / "benchmark-gate.yml"
CALIBRATE_WORKFLOW = WORKFLOWS_DIR / "benchmark-calibrate.yml"
JUSTFILE = PACKAGE_ROOT / "justfile"


def _e2e_slices() -> dict[str, dict[str, float]]:
    """E2E_SLICES, loaded from tests/e2e/conftest.py by path (not importable)."""
    import importlib.util

    path = PACKAGE_ROOT / "tests" / "e2e" / "conftest.py"
    spec = importlib.util.spec_from_file_location("rosotacom_e2e_conftest", path)
    assert spec and spec.loader, f"cannot load {path}"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    slices: dict[str, dict[str, float]] = module.E2E_SLICES
    return slices


@pytest.fixture(scope="module")
def rows() -> list[GateRow]:
    return load_registry()


@pytest.fixture(scope="module")
def profiles() -> dict[str, Profile]:
    return load_profiles_file(PROFILES_PATH)


def test_every_row_names_a_committed_public_profile(rows: list[GateRow], profiles: dict[str, Profile]) -> None:
    referenced = {profile for row in rows for profile in profiles_for_row(row)}
    missing = sorted(referenced - set(profiles))
    assert not missing, (
        f"registry rows reference profiles {missing} that are not committed in {PROFILES_PATH.name}; "
        "gate profiles are public, plain-netem, and live next to the other example profiles"
    )


def _random_netem_directions(profile: Profile) -> list[str]:
    """Directions whose netem uses randomness (jitter/loss/reorder/duplicate)."""

    def shaping_is_random(shaping: object) -> bool:
        return shaping is not None and any(
            getattr(shaping, name, None) is not None
            for name in ("jitter_ms", "loss_pct", "reorder_pct", "duplicate_pct")
        )

    random_dirs: list[str] = []
    segments = profile.timeline if profile.is_timeline else [profile]
    for index, segment in enumerate(segments):
        for direction in ("uplink", "downlink"):
            shaping = getattr(segment, direction, None)
            if shaping_is_random(shaping):
                random_dirs.append(f"{profile.name}[{index}].{direction}")
    return random_dirs


def test_public_gate_profiles_use_deterministic_netem(rows: list[GateRow], profiles: dict[str, Profile]) -> None:
    """Public gate profiles must shape with *deterministic* netem only —
    rate/delay bottlenecks, no random loss/jitter/reorder/duplicate.

    The vision's "seed policy is explicit" is realizable for the seeded *load*
    (application-level RNG in sized_publisher, committed in the registry) but
    NOT for random *netem* on the public runner class: the tc on GitHub-hosted
    runners rejects the netem `seed` option (observed: `netem … loss 8% seed 1
    -> What is "seed"?`), so a seeded-random public profile cannot even be
    installed there, and an unseeded one is non-deterministic — either way it
    must not gate. Determinism for public rows therefore comes from the pinned
    bottleneck (rate/delay), which needs no seed. Seeded-random netem stays in
    the operator harness lanes on a tc that supports it (performance bands reference, honest
    limits)."""
    problems: list[str] = []
    for profile_name in sorted({profile for row in rows for profile in profiles_for_row(row)}):
        problems += _random_netem_directions(profiles[profile_name])
    assert not problems, (
        "random netem in public gate profiles (github-hosted tc cannot seed it, so it cannot gate "
        f"deterministically): {problems}. Use a rate/delay bottleneck instead."
    )


def test_every_gated_metric_has_a_calibrated_band(rows: list[GateRow]) -> None:
    """The registry without calibrated bands is a monitor pretending to gate:
    every (row, profile, gated metric) needs a band whose provenance names a
    real runner class (mint them: the benchmark-calibrate workflow +
    `rosotacom benchmark calibrate`)."""
    bands = {band.key: band for band in load_bands(BUDGETS_PATH)}
    problems: list[str] = []
    for row in rows:
        for metric in row.metrics:
            band = bands.get((row.id, row.profile, metric))
            if band is None:
                problems.append(f"({row.id}, {row.profile}, {metric}): no band committed")
            elif band.provenance.fingerprint == UNCALIBRATED_FINGERPRINT:
                problems.append(f"({row.id}, {row.profile}, {metric}): band is uncalibrated")
    assert not problems, "budgets.jsonl is missing calibrated bands:\n" + "\n".join(problems)


def test_public_s2_replay_rows_carry_whole_bag_expect_metadata(rows: list[GateRow]) -> None:
    """Issue #185 / harness #34: the public nightly gate must keep the S2
    anonymized costmap and camera replay shapes under both nominal and tight
    profiles, with the generated whole-bag expect thresholds recorded next to
    the row and the delivery metrics kept in the verdict."""
    expected = {
        ("replay-costmap-nominal-cyclone", "gate-nominal", "15_remote_assist_anonymized_costmap"),
        ("replay-costmap-tight-cyclone", "gate-tight", "15_remote_assist_anonymized_costmap"),
        ("replay-camera-nominal-cyclone", "gate-nominal", "16_remote_assist_anonymized_camera"),
        ("replay-camera-tight-cyclone", "gate-tight", "16_remote_assist_anonymized_camera"),
    }
    by_id = {row.id: row for row in rows}
    missing = sorted(row_id for row_id, _profile, _example in expected if row_id not in by_id)
    assert not missing, f"missing public S2 replay nightly rows: {missing}"

    for row_id, profile, public_example in expected:
        row = by_id[row_id]
        assert row.lane == "nightly"
        assert row.genre == "replay", "the S2 rows are bag-derived; the genre is what makes that checkable"
        assert row.profile == profile
        assert {"loss_pct", "completeness_pct", "delivered_count", "delivered_hz"} <= set(row.monitor)
        assert row.replay["public_example"] == public_example
        assert row.replay["bag_topic"] == "/topic1"
        assert row.replay["native_count"] == 1396
        assert row.replay["expect"] == {
            "min_count": 1256,
            "completeness_min_ratio": 0.9,
            "vs_bag_ratio": 0.9,
        }
    assert by_id["replay-camera-tight-cyclone"].replay["keyframes"] == 285
    assert by_id["replay-camera-tight-cyclone"].replay["gop_spacing_median_frames"] == 5


def test_public_s2_replay_rows_await_their_calibration(rows: list[GateRow]) -> None:
    """Step 2 of issue #185 is a calibration run on the runner class, so these
    rows land unbanded on purpose. This is the reminder that the step exists: it
    fails the moment a band appears, which is exactly when the row should also
    move its gated metric out of `monitor`."""
    replay_rows = [row for row in rows if row.genre == "replay"]
    assert replay_rows, "the registry lost its replay rows"
    banded = sorted(row.id for row in replay_rows if row.metrics)
    if banded:
        pytest.fail(
            f"replay rows {banded} carry banded metrics now — promote them in the registry and "
            "commit the calibrated bands in the same change (issue #185, step 2)"
        )
    bands = {band.key for band in load_bands(BUDGETS_PATH)}
    stale = sorted(key for key in bands if key[0] in {row.id for row in replay_rows})
    assert not stale, f"budgets.jsonl carries bands for unbanded replay rows: {stale}"


def test_committed_bands_belong_to_registry_rows(rows: list[GateRow]) -> None:
    """No orphan bands: every committed band is one a benched row still gates,
    so a renamed/removed row cannot leave stale envelopes behind."""
    expected = {(row.id, row.profile, metric) for row in rows for metric in row.metrics}
    orphans = [band.key for band in load_bands(BUDGETS_PATH) if band.key not in expected]
    assert not orphans, f"budgets.jsonl carries bands no registry row gates: {orphans}"


def test_merge_gate_lane_actually_selects_the_benchmark_e2e() -> None:
    """Regression contract for the silent deselection this work found: a
    `-k` filter meant for one file used to deselect every test in another, so
    the benchmark E2E could vanish from the gate without failing anything.

    The slice runner carries no `-k` at all now — it deselects by node id
    against E2E_SLICES — so the contract is that both benchmark E2E files are
    owned by a slice. Which diagnostic slice owns them is a partition decision
    (#226 split the capacity probes out of `runtime-tools`, #253 rejects
    cross-theme cost packing); that they run at all is not.
    """
    slice_recipe = re.search(
        r"^test-e2e-slice[^\n:]*:\n((?:\t.*\n?)+)",
        JUSTFILE.read_text(encoding="utf-8"),
        re.MULTILINE,
    )
    assert slice_recipe, "justfile recipe test-e2e-slice is missing"
    poisoned = [line for line in slice_recipe.group(1).splitlines() if re.search(r"\s-k\s", line)]
    assert not poisoned, f"a -k filter would silently deselect tests from a slice: {poisoned}"

    owned = {nodeid for tests in _e2e_slices().values() for nodeid in tests}
    for e2e_file in ("test_benchmark_capacity.py", "test_benchmark_ab.py"):
        assert any(nodeid.startswith(f"tests/e2e/{e2e_file}::") for nodeid in owned), (
            f"no e2e slice owns tests/e2e/{e2e_file}, so no merge-gate lane runs it"
        )
