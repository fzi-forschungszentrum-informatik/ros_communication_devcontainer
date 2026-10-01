# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""netem distribution-table generation (rosotacom.netem_dist) — host tests."""

from __future__ import annotations

import math
import random

import pytest

from rosotacom.netem_dist import (
    NETEM_DIST_SCALE,
    TABLE_SIZE,
    build_dist_table,
    format_dist_file,
    read_samples_csv,
)


def test_table_shape_and_normalization() -> None:
    rng = random.Random(1)
    samples = [rng.gauss(30.0, 10.0) for _ in range(20000)]
    table = build_dist_table(samples)
    assert len(table) == TABLE_SIZE
    assert table == sorted(table)  # inverse CDF is nondecreasing
    mean = sum(table) / len(table)
    std = math.sqrt(sum((v - mean) ** 2 for v in table) / len(table))
    # normalized to mean 0 / std 1 in units of NETEM_DIST_SCALE
    assert abs(mean) < 0.05 * NETEM_DIST_SCALE
    assert abs(std - NETEM_DIST_SCALE) < 0.05 * NETEM_DIST_SCALE


def test_table_preserves_a_heavy_tail() -> None:
    # A lognormal delay (field cellular links) must keep its asymmetry: the
    # upper tail reaches much further from the median than the lower tail.
    rng = random.Random(2)
    samples = [math.exp(rng.gauss(3.3, 0.6)) for _ in range(20000)]
    table = build_dist_table(samples)
    p50 = table[TABLE_SIZE // 2]
    upper = table[int(TABLE_SIZE * 0.999)] - p50
    lower = p50 - table[int(TABLE_SIZE * 0.001)]
    assert upper > 1.8 * lower


def test_rejects_degenerate_input() -> None:
    with pytest.raises(ValueError):
        build_dist_table([1.0] * 500)  # zero spread
    with pytest.raises(ValueError):
        build_dist_table([1.0, 2.0])  # too few samples


def test_format_and_csv_roundtrip(tmp_path) -> None:
    csv_path = tmp_path / "samples.csv"
    csv_path.write_text("delay_ms\n" + "\n".join(str(10 + (i % 50)) for i in range(500)) + "\n", encoding="utf-8")
    samples = read_samples_csv(csv_path, "delay_ms")
    assert len(samples) == 500
    text = format_dist_file(build_dist_table(samples), comment="prov line")
    lines = text.strip().splitlines()
    assert lines[0] == "# prov line"
    values = [int(v) for line in lines[1:] for v in line.split()]
    assert len(values) == TABLE_SIZE
    with pytest.raises(ValueError):
        read_samples_csv(csv_path, "missing_column")
