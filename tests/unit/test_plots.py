# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Smoke tests for rosotacom.plots — verify each function writes a non-empty PNG."""

from __future__ import annotations

from pathlib import Path
from unittest.mock import patch

import pytest

try:
    import matplotlib  # noqa: F401

    has_matplotlib = True
except ImportError:
    has_matplotlib = False

pytestmark = pytest.mark.skipif(not has_matplotlib, reason="matplotlib is required for benchmark plots")

from rosotacom.plots import (  # noqa: E402, I001
    _require_matplotlib,
    plot_capacity_frontier,
    plot_forensics_stream,
    plot_offered_bw,
    plot_probe_timeseries,
    plot_probe_raw,
    plot_ramp,
    plot_recovery_timeline,
    plot_topic_heatmap,
)


# -- fixtures --------------------------------------------------------------- #

CAPACITY_RESULTS: list[dict] = [
    {"bandwidth_bps": 1e6, "loss_pct": 0.1, "latency_p95_ms": 50},
    {"bandwidth_bps": 2e6, "loss_pct": 0.5, "latency_p95_ms": 80},
    {"bandwidth_bps": 4e6, "loss_pct": 2.0, "latency_p95_ms": 150},
]

OFFERED_BW_RESULTS: list[dict] = [
    {"offered_bw_bps": 1e6, "latency_p95_ms": 10, "size": 1000, "rate_hz": 10, "streams": 1},
    {"offered_bw_bps": 2e6, "latency_p95_ms": 25, "size": 1000, "rate_hz": 20, "streams": 1},
    {"offered_bw_bps": 4e6, "latency_p95_ms": 80, "size": 2000, "rate_hz": 10, "streams": 2},
]

RAMP_CURVE: list[dict] = [
    {"value": 1.0, "metric": 5.0},
    {"value": 2.0, "metric": 6.0},
    {"value": 3.0, "metric": 8.0},
    {"value": 4.0, "metric": 15.0},
    {"value": 5.0, "metric": 40.0},
]

RECOVERY_RECORDS: list[dict] = [
    {"arrival_s": 0.5},
    {"arrival_s": 1.0},
    {"arrival_s": 3.5},
    {"arrival_s": 4.0},
    {"arrival_s": 4.1},
    {"arrival_s": 4.2},
    {"arrival_s": 5.0},
]

TOPIC_HEATMAP: dict[str, dict[str, float]] = {
    "/camera": {"wifi_good": 0.1, "wifi_bad": 5.0, "lte": 1.2},
    "/lidar": {"wifi_good": 0.0, "wifi_bad": 3.5, "lte": 0.8},
    "/cmd_vel": {"wifi_good": 0.0, "wifi_bad": 0.2, "lte": 0.0},
}

PROBE_BINS: list[dict] = [
    {
        "attempt": 1,
        "topic": "/bench_capacity",
        "bin_start_s": 0.0,
        "loss_pct": 0.0,
        "latency_p95_ms": 50.0,
        "delivered_hz": 20.0,
        "payload_bandwidth_bps": 2_880_000.0,
    },
    {
        "attempt": 1,
        "topic": "/bench_capacity",
        "bin_start_s": 1.0,
        "loss_pct": 10.0,
        "latency_p95_ms": 120.0,
        "delivered_hz": 18.0,
        "payload_bandwidth_bps": 2_592_000.0,
    },
]

RAW_RECORDS: list[dict] = [
    {
        "kind": "transit",
        "peer": "b",
        "source": "a",
        "target": "b",
        "topic": "/bench_capacity",
        "seq": 0,
        "status": "delivered",
        "t_wrap": 10.0,
        "sections": {"ota_hop_ms": 10.0},
        "size_bytes": 100,
    },
    {
        "kind": "transit",
        "peer": "b",
        "source": "a",
        "target": "b",
        "topic": "/bench_capacity",
        "seq": 1,
        "status": "lost",
    },
    {
        "kind": "transit",
        "peer": "b",
        "source": "a",
        "target": "b",
        "topic": "/bench_capacity",
        "seq": 2,
        "status": "delivered",
        "t_wrap": 11.0,
        "sections": {"ota_hop_ms": 20.0},
        "size_bytes": 200,
    },
]


# -- smoke tests ------------------------------------------------------------ #


def test_capacity_frontier_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "cap.png"
    result = plot_capacity_frontier(CAPACITY_RESULTS, out=out)
    assert result == out
    assert out.stat().st_size > 0


def test_offered_bw_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "offered.png"
    result = plot_offered_bw(OFFERED_BW_RESULTS, out=out)
    assert result == out
    assert out.stat().st_size > 0


def test_probe_timeseries_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "probe.png"
    result = plot_probe_timeseries(PROBE_BINS, out=out)
    assert result == out
    assert out.stat().st_size > 0


def test_probe_raw_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "probe_raw.png"
    result = plot_probe_raw(RAW_RECORDS, out=out)
    assert result == out
    assert out.stat().st_size > 0


def test_ramp_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "ramp.png"
    result = plot_ramp(RAMP_CURVE, out=out)
    assert result == out
    assert out.stat().st_size > 0


def test_recovery_timeline_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "recovery.png"
    result = plot_recovery_timeline(RECOVERY_RECORDS, outage_start=2.0, outage_end=3.5, out=out)
    assert result == out
    assert out.stat().st_size > 0


def test_topic_heatmap_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "heatmap.png"
    result = plot_topic_heatmap(TOPIC_HEATMAP, out=out)
    assert result == out
    assert out.stat().st_size > 0


FORENSICS_BINS: list[dict] = [
    {
        "bin_start_s": float(index),
        "bin_end_s": float(index + 1),
        "expected": 20,
        "delivered": 0 if index == 2 else 20,
        "lost": 20 if index == 2 else 0,
        "delivered_hz": 0.0 if index == 2 else 20.0,
        "latency_p50_ms": 10.0,
        "latency_p95_ms": 12.0,
        "latency_max_ms": 15.0,
        "mean_size_bytes": 5000.0,
        "max_size_bytes": 40000.0 if index % 2 == 0 else 6000.0,
        "keyframes": 1 if index % 2 == 0 else 0,
    }
    for index in range(6)
]

FORENSICS_EVENTS: list[dict] = [
    {"kind": "loss_burst", "start_s": 2.0, "end_s": 3.0},
    {"kind": "latency_excursion", "start_s": 4.0, "end_s": 4.5},
]

FORENSICS_STEPS: list[dict] = [
    {"start_s": 0.0, "end_s": 3.0, "label": "step 0"},
    {"start_s": 3.0, "end_s": 6.0, "label": "step 1 catchup"},
]


def test_forensics_stream_writes_figure(tmp_path: Path) -> None:
    out = tmp_path / "forensics.png"
    result = plot_forensics_stream(
        FORENSICS_BINS,
        FORENSICS_EVENTS,
        out=out,
        nominal_hz=20.0,
        timeline_steps=FORENSICS_STEPS,
    )
    assert result == out
    assert out.stat().st_size > 0


def test_require_matplotlib_error_message() -> None:
    """Verify the import guard gives a helpful install hint."""
    with patch.dict("sys.modules", {"matplotlib": None}):
        with pytest.raises(ImportError, match=r"pip install rosotacom\[plots\]"):
            _require_matplotlib()
