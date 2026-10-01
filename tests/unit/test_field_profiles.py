# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The packaged field-fitted profiles must stay loadable and self-contained."""

from __future__ import annotations

from pathlib import Path

from rosotacom.network_profiles import load_profiles_file, required_dist_installs

PROFILES = Path(__file__).resolve().parents[2] / "src" / "rosotacom" / "resources" / "examples" / "profiles.yaml"


def test_field_profiles_parse_and_their_dist_table_ships() -> None:
    profiles = load_profiles_file(PROFILES)
    for name in (
        "field-20260817-steady",
        "field-20260817-events",
        "field-20260817-soggy",
        "field-20260817-case2-weak-zone",
        "field-20260817-case3-delay-jam",
        "field-20260817-case6-source-gap",
    ):
        assert name in profiles, name

    events = profiles["field-20260817-events"]
    assert events.uplink is not None and events.uplink.loss_gemodel is not None
    assert events.uplink.loss_gemodel.loss_bad_pct == 2.7

    # Every custom distribution resolves to a file that actually ships.
    for profile in profiles.values():
        for table_name, file_path in required_dist_installs(profile):
            assert table_name == "field-20260817"
            assert Path(file_path).is_file(), file_path

    jam = profiles["field-20260817-case3-delay-jam"]
    assert jam.is_timeline and jam.timeline[1].uplink is not None
    assert jam.timeline[1].uplink.rate_bps == 900_000

    # The source-side case is deliberately unshaped.
    assert profiles["field-20260817-case6-source-gap"].uplink is None


def test_field_dist_table_is_a_valid_netem_table() -> None:
    dist = PROFILES.parent / "dist" / "field-20260817.dist"
    values = [
        int(v) for line in dist.read_text().splitlines() if line and not line.startswith("#") for v in line.split()
    ]
    assert len(values) == 4096
    assert values == sorted(values)
    # Field delay is right-skewed: the upper tail reaches further than the lower.
    p50 = values[2048]
    assert (values[-5] - p50) > 1.5 * (p50 - values[4])
