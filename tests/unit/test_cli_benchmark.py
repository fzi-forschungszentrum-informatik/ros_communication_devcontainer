# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Host-deterministic tests for the benchmark CLI glue (benchmark commands reference § 1d).

These test the *wiring* — the driver functions that connect the pure
``benchmark.py`` logic to a ``run_point`` probe — with a **stubbed probe**
that returns canned transit summaries. The pure logic itself is already
covered by ``test_benchmark.py``; these verify the CLI-level orchestration.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path
from typing import Any

import pytest
import yaml

import rosotacom.cli as cli
import rosotacom.cli_benchmark as benchmark_cli
from rosotacom.benchmark import AbTolerance, default_ab_tolerance
from rosotacom.cli_benchmark import (
    BENCHMARK_RESULT_FILE,
    DEFAULT_BENCHMARK_DRAIN_S,
    DEFAULT_BENCHMARK_RMW,
    _benchmark_child_command,
    _benchmark_ota_target,
    _benchmark_profiles_file,
    _build_matrix_profiles,
    _build_sensitivity_profiles,
    _initialize_interactive_log,
    _is_ota_benchmark,
    _parse_loss_boundary_axes,
    _parse_requirements_axes,
    _parse_values,
    _peer_catmux_attach_script,
    _prepare_benchmark_session_config,
    _probe_load,
    _probe_spdp_diagnostics,
    _profile_requires_netem_seed,
    _requirements_jitter_loss_pct,
    _requirements_target_quality,
    _result_once_script,
    _start_interactive_benchmark,
    benchmark_ab,
    collect_transit_summary,
    drive_ab,
    drive_capacity,
    drive_loss_boundaries,
    drive_matrix,
    drive_probe,
    drive_ramp,
    drive_requirements,
    drive_sensitivity,
    drive_sweep,
    register_benchmark_parser,
)

# --------------------------------------------------------------------------- #
# Stubbed run_point
# --------------------------------------------------------------------------- #


def _make_stub_probe(
    *,
    loss_pct: float = 0.0,
    latency_p95: float = 50.0,
    breakpoint_size: int | None = None,
) -> Any:
    """A stubbed run_point that returns canned transit summaries.

    When ``breakpoint_size`` is set, the probe fails (100 % loss) above that
    size — simulating a capacity breakpoint for the binary-search tests.
    """

    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        size = load.get("size_a", 1000)
        if breakpoint_size is not None and size > breakpoint_size:
            return {
                "topics": {
                    "/test": {
                        "expected": 100,
                        "delivered": 0,
                        "lost": 100,
                        "loss_pct": 100.0,
                        "reordered": 0,
                        "ota_hop_ms": {"p50": None, "p95": None},
                        "jitter_ms": {"p50": None, "p95": None},
                    }
                }
            }
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": 100 - int(loss_pct),
                    "lost": int(loss_pct),
                    "loss_pct": loss_pct,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": 2.0, "p95": 5.0},
                }
            }
        }

    return stub


# --------------------------------------------------------------------------- #
# Fixed probe driver
# --------------------------------------------------------------------------- #


def test_probe_driver_writes_time_bins_from_transit_records(tmp_path: Path) -> None:
    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        events_dir = out_dir / "logs" / "b" / "status"
        events_dir.mkdir(parents=True)
        records = [
            {
                "kind": "transit",
                "source": "a",
                "target": "b",
                "topic": "/test",
                "seq": 0,
                "status": "delivered",
                "t_wrap": 1.0,
                "sections": {"ota_hop_ms": 10.0},
                "size_bytes": 100,
            },
            {
                "kind": "transit",
                "source": "a",
                "target": "b",
                "topic": "/test",
                "seq": 1,
                "status": "lost",
                "t_wrap": None,
                "sections": {"ota_hop_ms": None},
                "size_bytes": None,
            },
            {
                "kind": "transit",
                "source": "a",
                "target": "b",
                "topic": "/test",
                "seq": 2,
                "status": "delivered",
                "t_wrap": 2.0,
                "sections": {"ota_hop_ms": 20.0},
                "size_bytes": 200,
            },
        ]
        (events_dir / "events.jsonl").write_text(
            "\n".join(json.dumps(record) for record in records) + "\n",
            encoding="utf-8",
        )
        return {
            "topics": {
                "/test": {
                    "expected": 3,
                    "delivered": 2,
                    "lost": 1,
                    "loss_pct": 33.333,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 10.0, "p95": 20.0},
                    "jitter_ms": {"p50": None, "p95": None},
                }
            }
        }

    result = drive_probe(
        stub,
        profile="losssless-typical",
        size=18_000,
        rate_hz=2.0,
        repeats=1,
        duration_s=2.0,
        bin_s=1.0,
        render_plot=False,
        out_dir=tmp_path,
    )

    bins = [json.loads(line) for line in (tmp_path / "time-bins.jsonl").read_text(encoding="utf-8").splitlines()]
    assert result["profile"] == "losssless-typical"
    assert result["time_bin_count"] == 2
    assert bins[0]["topic"] == "a->b:/test"
    assert bins[0]["expected"] == 2
    assert bins[0]["delivered"] == 1
    assert bins[0]["lost"] == 1
    assert bins[0]["loss_pct"] == 50.0
    assert bins[0]["delivered_hz"] == 1.0
    assert bins[0]["payload_bandwidth_bps"] == 800.0
    assert bins[1]["latency_p95_ms"] == 20.0
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["genre"] == "probe"
    assert result_doc["configuration"]["bin_s"] == 1.0
    assert result_doc["artifacts"]["time_bins"] == "time-bins.jsonl"


def test_probe_driver_accepts_size_pattern_load(tmp_path: Path) -> None:
    seen_loads: list[dict[str, Any]] = []

    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        seen_loads.append(dict(load))
        return {
            "topics": {
                "/test": {
                    "expected": 2,
                    "delivered": 2,
                    "lost": 0,
                    "loss_pct": 0.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 10.0, "p95": 10.0},
                    "jitter_ms": {"p50": 1.0, "p95": 1.0},
                }
            }
        }

    result = drive_probe(
        stub,
        profile="lossless-typical",
        size_pattern="1x20KB+1x0KB",
        rate_hz=10.0,
        repeats=1,
        duration_s=1.0,
        bin_s=1.0,
        render_plot=False,
        out_dir=tmp_path,
    )

    assert seen_loads == [
        {
            "size_a": 20_000,
            "size_b": 0,
            "pattern": "a*1,b*1",
            "size_pattern": "1x20KB+1x0KB",
            "rate": 10.0,
            "sizes": [20_000, 0],
        }
    ]
    assert result["load"]["mean_payload_bytes"] == 10_000.0
    assert result["load"]["offered_bandwidth_bps"] == 800_000.0
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["configuration"]["load"]["parameters"]["size_pattern"] == "1x20KB+1x0KB"


def test_sized_publisher_args_include_size_pattern_and_zero_payload() -> None:
    assert benchmark_cli._sized_publisher_param_args(
        "/bench_capacity",
        {
            "size_a": 20_000,
            "size_b": 0,
            "pattern": "a*1,b*1",
            "rate": 10.0,
            "streams": 1,
        },
    ) == [
        "-p",
        "topic:=/bench_capacity",
        "-p",
        "rate:=10.0",
        "-p",
        "streams:=1",
        "-p",
        "size_a:=20000",
        "-p",
        "pattern:=a*1,b*1",
        "-p",
        "size_b:=0",
    ]
    assert benchmark_cli._sized_publisher_param_args("/bench_capacity", {"size_a": 0})[-2:] == [
        "-p",
        "size:=0",
    ]
    assert benchmark_cli._sized_publisher_param_args(
        "/bench_capacity_0",
        {"size_a": 9000, "rate": 20.0, "streams": 2},
        streams=1,
    ) == [
        "-p",
        "topic:=/bench_capacity_0",
        "-p",
        "rate:=20.0",
        "-p",
        "streams:=1",
        "-p",
        "size:=9000",
    ]


def test_sized_publisher_args_include_interval_jitter() -> None:
    assert benchmark_cli._sized_publisher_param_args(
        "/bench_capacity",
        {
            "size_a": 20_000,
            "rate": 10.0,
            "streams": 1,
            "interval_jitter_ms": 20.0,
            "interval_jitter_seed": 42,
        },
    ) == [
        "-p",
        "topic:=/bench_capacity",
        "-p",
        "rate:=10.0",
        "-p",
        "streams:=1",
        "-p",
        "interval_jitter_ms:=20.0",
        "-p",
        "interval_jitter_seed:=42",
        "-p",
        "size:=20000",
    ]


def test_sized_publisher_args_include_dynamic_sizes() -> None:
    assert benchmark_cli._sized_publisher_param_args(
        "/bench_capacity",
        {
            "sizes": [43000, 3000, 4000, 4000, 4000],
            "rate": 10.0,
            "streams": 1,
        },
    ) == [
        "-p",
        "topic:=/bench_capacity",
        "-p",
        "rate:=10.0",
        "-p",
        "streams:=1",
        "-p",
        "sizes:=[43000,3000,4000,4000,4000]",
    ]


def test_publisher_streams_collapse_for_expanded_stream_topics() -> None:
    topics = [{"topic": "/bench_capacity_0"}, {"topic": "/bench_capacity_1"}]

    assert benchmark_cli._publisher_streams_for_topic_specs(topics, {"streams": 2}) == 1
    assert benchmark_cli._publisher_streams_for_topic_specs([{"topic": "/bench_capacity"}], {"streams": 2}) == 2


# --------------------------------------------------------------------------- #
# Capacity driver
# --------------------------------------------------------------------------- #


def test_capacity_driver_finds_breakpoint_with_stubbed_probe(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The capacity driver finds the binary-search breakpoint against a stubbed
    metric source that fails above 8000 B."""
    probe = _make_stub_probe(breakpoint_size=8000)
    result = drive_capacity(
        probe,
        profile="test-profile",
        knob="size",
        low=1000,
        high=70000,
        max_loss_pct=5.0,
        max_latency_ms=200.0,
        duration_s=1.0,
        out_dir=tmp_path,
    )
    assert result["capacity"] == 8000
    assert result["slice"]["knob"] == "size"
    assert result["slice"]["profile"] == "test-profile"
    # No per-run budget file anymore: bands change only via `benchmark ratchet`.
    assert not (tmp_path / "budgets.jsonl").exists()
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    # The self-contained result carries the provenance the band gate needs.
    assert result_doc["runner"]["fingerprint"]
    assert result_doc["sha"]
    assert result_doc["configuration"]["thresholds"]["max_loss_pct"] == 5.0
    assert result_doc["result"]["capacity"] == 8000
    assert result_doc["verdict"]["passed"] is True
    assert result_doc["measurements"]["probes"]
    assert result_doc["measurements"]["probes"][0]["topics"][0]["loss_pct"] in (0.0, 100.0)
    assert "offered_bw=" in capsys.readouterr().out


def test_capacity_driver_returns_none_when_low_fails(tmp_path: Path) -> None:
    """If even the lowest probe value fails, capacity is None."""
    probe = _make_stub_probe(breakpoint_size=0)  # everything fails
    result = drive_capacity(
        probe,
        profile="p",
        knob="size",
        low=1000,
        high=10000,
        max_loss_pct=1.0,
        max_latency_ms=100.0,
        duration_s=1.0,
        out_dir=tmp_path,
    )
    assert result["capacity"] is None


# --------------------------------------------------------------------------- #
# Ramp driver
# --------------------------------------------------------------------------- #


def test_ramp_driver_builds_curve_with_stubbed_probe(tmp_path: Path) -> None:
    """The ramp driver builds the latency-vs-load curve from stubbed points."""
    probe = _make_stub_probe(latency_p95=50.0)
    curve = drive_ramp(
        probe,
        profile="p",
        values=[1000, 2000, 4000],
        duration_s=1.0,
        out_dir=tmp_path,
    )
    assert len(curve) == 3
    assert all("value" in point and "metric" in point for point in curve)
    assert curve[0]["value"] == 1000.0
    # Curve file was written.
    assert (tmp_path / "curve.jsonl").exists()
    assert (tmp_path / BENCHMARK_RESULT_FILE).exists()


# --------------------------------------------------------------------------- #
# Recovery driver
# --------------------------------------------------------------------------- #


def test_recovery_driver_extracts_metrics_from_synthetic_timeline(tmp_path: Path) -> None:
    """The recovery driver feeds transit records through recovery_metrics."""
    from rosotacom.benchmark import OutageWindow, recovery_metrics

    # Build a synthetic events.jsonl under the expected path.
    events_dir = tmp_path / "logs" / "a" / "status"
    events_dir.mkdir(parents=True)
    events_path = events_dir / "events.jsonl"

    records = [
        {
            "kind": "transit",
            "topic": "/x",
            "seq": 0,
            "status": "delivered",
            "t_wrap": 0.0,
            "sections": {"ota_hop_ms": 100.0},
        },
        {
            "kind": "transit",
            "topic": "/x",
            "seq": 1,
            "status": "delivered",
            "t_wrap": 1.0,
            "sections": {"ota_hop_ms": 100.0},
        },
        {
            "kind": "transit",
            "topic": "/x",
            "seq": 2,
            "status": "lost",
            "t_wrap": None,
            "sections": {"ota_hop_ms": None},
        },
        {
            "kind": "transit",
            "topic": "/x",
            "seq": 3,
            "status": "delivered",
            "t_wrap": 3.0,
            "sections": {"ota_hop_ms": 100.0},
        },
        {
            "kind": "transit",
            "topic": "/x",
            "seq": 4,
            "status": "delivered",
            "t_wrap": 4.0,
            "sections": {"ota_hop_ms": 100.0},
        },
    ]
    events_path.write_text(
        "\n".join(json.dumps(r) for r in records) + "\n",
        encoding="utf-8",
    )

    # Direct recovery_metrics call (the driver's core logic).
    from rosotacom.transit import join_transit_records, load_transit_records

    loaded = join_transit_records(load_transit_records([events_path]))
    metrics = recovery_metrics(
        loaded,
        OutageWindow(start=1.5, end=2.5),
        nominal_period_s=1.0,
        latched_topics=["/x"],
    )
    assert metrics.lost_during_outage == {"/x": 1}
    assert metrics.latched_rearrival == {"/x": True}


# --------------------------------------------------------------------------- #
# Sweep driver
# --------------------------------------------------------------------------- #


def test_sweep_driver_runs_grid_with_stubbed_probe(tmp_path: Path) -> None:
    """The sweep driver runs one point per profile and reports oracle results."""
    probe = _make_stub_probe(loss_pct=1.0, latency_p95=80.0)
    frontier = drive_sweep(
        probe,
        profile_grid=["profile-a", "profile-b"],
        max_loss_pct=5.0,
        max_latency_ms=200.0,
        duration_s=1.0,
        out_dir=tmp_path,
    )
    assert len(frontier) == 2
    assert frontier[0]["profile"] == "profile-a"
    assert frontier[1]["profile"] == "profile-b"
    # Both should pass with our lenient thresholds.
    assert all(row["passes"] for row in frontier)
    # Frontier file was written.
    assert (tmp_path / "frontier.jsonl").exists()
    assert (tmp_path / BENCHMARK_RESULT_FILE).exists()


def test_sweep_driver_supports_unshaped_profile(tmp_path: Path) -> None:
    seen_profiles: list[str | None] = []

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        seen_profiles.append(profile)
        return _make_stub_probe(loss_pct=0.0, latency_p95=10.0)(
            profile=profile,
            load=load,
            duration_s=duration_s,
            out_dir=out_dir,
        )

    frontier = drive_sweep(
        probe,
        profile_grid=["none", "profile-a"],
        max_loss_pct=5.0,
        max_latency_ms=200.0,
        duration_s=1.0,
        out_dir=tmp_path,
    )

    assert seen_profiles == [None, "profile-a"]
    assert [row["profile"] for row in frontier] == ["none", "profile-a"]
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["configuration"]["profile_grid"] == ["none", "profile-a"]


# --------------------------------------------------------------------------- #
# Sensitivity driver
# --------------------------------------------------------------------------- #


def test_sensitivity_profile_generation_and_driver(tmp_path: Path) -> None:
    profiles_file = tmp_path / "profiles.yaml"
    profiles_file.write_text(
        "profiles:\n"
        "  cellular-4g-degraded:\n"
        "    uplink:\n"
        "      { rate: 1mbit, delay: 180ms, jitter: 50ms, distribution: normal, loss: 3%, loss_correlation: 25% }\n"
        "    downlink: { rate: 10mbit, delay: 100ms, jitter: 30ms, loss: 1% }\n",
        encoding="utf-8",
    )
    generated, cases = _build_sensitivity_profiles(
        base_profile="cellular-4g-degraded",
        profiles_file=profiles_file,
        ideal_rate="1gbit",
        loss_values=[0.0, 3.0],
        delay_values=[0.0, 180.0],
        jitter_values=[0.0, 50.0],
        rate_values=["1gbit", "1mbit"],
        correlation_values=[0.0, 25.0],
        axes=["jitter"],
    )
    generated_file = tmp_path / "generated-profiles.yaml"
    generated_file.write_text(yaml.safe_dump(generated, sort_keys=False), encoding="utf-8")

    seen_profiles: list[str | None] = []

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        seen_profiles.append(profile)
        latency = 10.0 if profile is None or profile == "lab_ideal_rate_only" else 25.0
        return _make_stub_probe(loss_pct=0.0, latency_p95=latency)(
            profile=profile,
            load=load,
            duration_s=duration_s,
            out_dir=out_dir,
        )

    rows = drive_sensitivity(
        probe,
        cases=cases,
        max_loss_pct=5.0,
        max_latency_ms=200.0,
        rate_hz=20.0,
        size=1,
        topic="",
        duration_s=1.0,
        out_dir=tmp_path,
        generated_profiles_file=generated_file,
    )

    assert seen_profiles[0] is None
    assert any(row["axis"] == "jitter" and row["value"] == "50ms" for row in rows)
    assert not any(row["axis"] == "loss" for row in rows)
    assert generated["metadata"]["axes"] == ["jitter"]
    assert all(row["expected"] == 100 for row in rows)
    assert all(row["delivered"] == 100 for row in rows)
    assert all(row["lost"] == 0 for row in rows)
    assert (tmp_path / "sensitivity.jsonl").is_file()
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["genre"] == "sensitivity"
    assert result_doc["artifacts"]["generated_profiles"] == "generated-profiles.yaml"
    assert result_doc["result"]["analysis"]["axes"]["jitter"]["points"] == 2


# --------------------------------------------------------------------------- #
# Matrix driver
# --------------------------------------------------------------------------- #


def test_matrix_profile_generation_and_driver(tmp_path: Path) -> None:
    profiles_file = tmp_path / "profiles.yaml"
    profiles_file.write_text(
        "profiles:\n"
        "  cellular-4g-degraded:\n"
        "    uplink:\n"
        "      { rate: 1mbit, delay: 180ms, jitter: 50ms, distribution: normal, loss: 3% }\n"
        "    downlink: { rate: 10mbit, delay: 100ms, jitter: 30ms, loss: 1% }\n",
        encoding="utf-8",
    )
    generated, cases = _build_matrix_profiles(
        base_profile="cellular-4g-degraded",
        profiles_file=profiles_file,
        ideal_rate="1gbit",
        jitter_ms=30.0,
        latency_values=[180.0],
        rate_hz_values=[20.0, 1.0],
        qos_cases=[{"reliability": "reliable", "depth": 1}, {"reliability": "best_effort", "depth": 10}],
        axes=["latency", "hz", "qos"],
        size=1,
        fixed_rate_hz=20.0,
        min_duration_s=20.0,
        min_messages=100,
    )
    generated_file = tmp_path / "generated-profiles.yaml"
    generated_file.write_text(yaml.safe_dump(generated, sort_keys=False), encoding="utf-8")

    seen: list[dict[str, Any]] = []

    def probe(
        *,
        profile: str | None,
        load: dict[str, Any],
        duration_s: float,
        out_dir: Path,
        session_options: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        seen.append(
            {
                "profile": profile,
                "rate": load["rate"],
                "duration_s": duration_s,
                "session_options": session_options,
            }
        )
        return _make_stub_probe(loss_pct=0.0, latency_p95=25.0)(
            profile=profile,
            load=load,
            duration_s=duration_s,
            out_dir=out_dir,
        )

    rows = drive_matrix(
        probe,
        cases=cases,
        max_loss_pct=5.0,
        max_latency_ms=300.0,
        topic="",
        out_dir=tmp_path,
        generated_profiles_file=generated_file,
    )

    assert any(row["axis"] == "reference" for row in rows)
    assert any(row["axis"] == "qos" and row["qos"] == {"reliability": "best_effort", "depth": 10} for row in rows)
    assert any(item["rate"] == 1.0 and item["duration_s"] == 100.0 for item in seen)
    assert any((item["session_options"] or {}).get("qos_reliability") == "reliable" for item in seen)
    assert generated["metadata"]["latency_model"]["downlink_ratio"] == pytest.approx(100.0 / 180.0)
    assert (tmp_path / "matrix.jsonl").is_file()
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["genre"] == "matrix"
    assert result_doc["result"]["analysis"]["reference_profile"].startswith("matrix_jitter")
    assert result_doc["result"]["analysis"]["axes"]["qos"]["points"] == 2


# --------------------------------------------------------------------------- #
# Requirements driver
# --------------------------------------------------------------------------- #


def test_requirements_driver_finds_tight_profile_with_stubbed_probe(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_ms, parse_pct, parse_rate_bps

    generated_file = tmp_path / "generated-profiles.yaml"

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        spec = generated["profiles"][profile]["uplink"]
        bandwidth = parse_rate_bps(spec["rate"])
        latency = parse_ms(spec.get("delay", 0), "delay")
        jitter = parse_ms(spec.get("jitter", 0), "jitter")
        loss = parse_pct(spec.get("loss", 0), "loss")

        # Deterministic synthetic quality model:
        # - bandwidth below 4 Mbit/s causes loss,
        # - jitter above 20 ms causes loss,
        # - network loss maps 1:1.5 into ROS-level loss,
        # - latency and jitter combine into p95 latency.
        ros_loss = max(0.0, (4_000_000.0 - bandwidth) / 1_000_000.0 * 3.0) + max(0.0, jitter - 20.0) * 0.4
        ros_loss += loss * 1.5
        latency_p95 = 40.0 + latency + jitter * 2.0
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": max(0, 100 - int(round(ros_loss))),
                    "lost": int(round(ros_loss)),
                    "loss_pct": ros_loss,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": jitter * 0.5, "p95": jitter},
                }
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=5.0,
        max_latency_ms=250.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        bandwidth_high_bps=10_000_000.0,
        bandwidth_low_bps=1_000_000.0,
        latency_base_ms=0.0,
        latency_high_ms=300.0,
        jitter_high_ms=60.0,
        loss_high_pct=10.0,
        axes=_parse_requirements_axes("bandwidth,latency,jitter,loss"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=6,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=0,
        loss_coupling="independent",
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    profile = result["profile"]["candidate"]
    assert result["analysis"]["final_passes"] is True
    assert result["analysis"]["tight"] is True
    assert 2_200_000.0 <= profile["bandwidth_bps"] <= 2_500_000.0
    assert 200.0 <= profile["uplink_latency_ms"] <= 210.0
    assert profile["jitter_ms"] <= 2.0
    assert profile["loss_pct"] == 0.0
    assert all(result["bounds"][axis]["last_fail"] for axis in ("bandwidth", "latency", "jitter", "loss"))
    assert (tmp_path / "requirements.jsonl").is_file()
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["genre"] == "requirements"
    assert result_doc["result"]["stream"]["load"]["mean_payload_bytes"] == 18_000.0
    assert result_doc["artifacts"]["generated_profiles"] == "generated-profiles.yaml"
    assert "requirements_final" in yaml.safe_load(generated_file.read_text(encoding="utf-8"))["profiles"]


def test_requirements_driver_can_search_coupled_jitter_loss(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_ms, parse_pct

    generated_file = tmp_path / "generated-profiles.yaml"

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        spec = generated["profiles"][profile]["uplink"]
        jitter = parse_ms(spec.get("jitter", 0), "jitter")
        loss = parse_pct(spec.get("loss", 0), "loss")
        latency_p95 = 40.0 + jitter * 2.0
        ros_loss = loss + max(0.0, jitter - 20.0) * 0.5
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": max(0, 100 - int(round(ros_loss))),
                    "lost": int(round(ros_loss)),
                    "loss_pct": ros_loss,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": jitter * 0.5, "p95": jitter},
                }
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=5.0,
        max_latency_ms=100.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        bandwidth_high_bps=100_000_000.0,
        bandwidth_low_bps=10_000_000.0,
        latency_base_ms=0.0,
        latency_high_ms=0.0,
        jitter_high_ms=50.0,
        loss_high_pct=10.0,
        axes=_parse_requirements_axes("jitter,loss"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=5,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=2,
        loss_coupling="jitter",
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    profile = result["profile"]["candidate"]
    assert "jitter_loss" in result["bounds"]
    assert "loss" not in result["bounds"]
    assert profile["jitter_ms"] == pytest.approx(28.125)
    assert profile["loss_pct"] == pytest.approx(_requirements_jitter_loss_pct(profile["jitter_ms"]))
    assert result["analysis"]["loss_coupling"] == "jitter"
    assert result["analysis"]["final_target_quality"]["limiting_metric"] == "loss"
    assert result["analysis"]["final_target_quality"]["utilization"] == pytest.approx(0.99375)


def test_requirements_zero_loss_target_clamps_network_loss(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_ms, parse_pct

    generated_file = tmp_path / "generated-profiles.yaml"
    seen_losses: list[float] = []

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        spec = generated["profiles"][profile]["uplink"]
        jitter = parse_ms(spec.get("jitter", 0), "jitter")
        loss = parse_pct(spec.get("loss", 0), "loss")
        seen_losses.append(loss)
        latency_p95 = 40.0 + jitter * 3.0
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": 100,
                    "lost": 0,
                    "loss_pct": 0.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": jitter * 0.5, "p95": jitter},
                }
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=0.0,
        max_latency_ms=100.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        bandwidth_high_bps=100_000_000.0,
        bandwidth_low_bps=10_000_000.0,
        latency_base_ms=0.0,
        latency_high_ms=0.0,
        jitter_high_ms=40.0,
        loss_high_pct=10.0,
        axes=_parse_requirements_axes("jitter,loss"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=3,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=0,
        loss_coupling="jitter",
        downlink_mode="lan",
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    assert set(seen_losses) == {0.0}
    assert result["profile"]["candidate"]["loss_pct"] == 0.0
    assert "jitter" in result["bounds"]
    assert "jitter_loss" not in result["bounds"]
    assert "loss" not in result["bounds"]
    assert result["analysis"]["loss_coupling"] == "jitter"
    assert result["analysis"]["effective_loss_coupling"] == "zero_loss"
    assert result["analysis"]["strict_zero_loss_target"] is True
    assert result["analysis"]["skipped_axes"] == ["loss"]
    assert result["profile"]["candidate"]["downlink_mode"] == "lan"
    generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
    assert generated["metadata"]["search"]["effective_loss_coupling"] == "zero_loss"
    assert generated["profiles"]["requirements_final"]["downlink"] == {}


def test_requirements_repeat_policy_finds_good_and_bad_zero_loss_cases(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_ms

    generated_file = tmp_path / "generated-profiles.yaml"
    profile_counts: dict[str, int] = {}

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        jitter = parse_ms(generated["profiles"][profile]["uplink"].get("jitter", 0), "jitter")
        repeat_index = profile_counts.get(profile, 0) + 1
        profile_counts[profile] = repeat_index
        lossy = jitter > 10.0 or repeat_index == 10
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": 99 if lossy else 100,
                    "lost": 1 if lossy else 0,
                    "loss_pct": 1.0 if lossy else 0.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 20.0, "p95": 40.0 + jitter},
                    "jitter_ms": {"p50": jitter * 0.5, "p95": jitter},
                }
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=0.0,
        max_latency_ms=100.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        bandwidth_high_bps=100_000_000.0,
        bandwidth_low_bps=10_000_000.0,
        latency_base_ms=30.0,
        latency_high_ms=30.0,
        jitter_high_ms=20.0,
        loss_high_pct=0.0,
        axes=_parse_requirements_axes("jitter"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=1,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=0,
        loss_coupling="jitter",
        probe_repeats=10,
        probe_min_passes=9,
        bad_lossy_count=10,
        netem_seed=12345,
        jitter_guard_ratio=0.1,
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    assert result["analysis"]["final_passes"] is True
    assert result["analysis"]["probe_repeats"] == 10
    assert result["analysis"]["probe_min_passes"] == 9
    assert result["rows"][0]["repeat"]["pass_count"] == 9
    assert result["rows"][0]["repeat"]["loss_free_count"] == 9
    assert result["bounds"]["jitter"]["last_bad"]["repeat"]["lossy_count"] == 10
    assert result["analysis"]["bad_cases_observed"]["jitter"]["repeat"]["bad_case"] is True
    assert result["analysis"]["final_target_quality"]["repeat_pass_count"] == 9
    assert result["analysis"]["netem_seed"] == 12345
    assert result["analysis"]["jitter_guard_ratio"] == 0.1
    assert result["analysis"]["guarded_axes"]["jitter"]["selected"]["jitter_ms"] == 10.0
    assert result["analysis"]["guarded_axes"]["jitter"]["guarded"]["jitter_ms"] == 9.0
    assert result["profile"]["candidate"]["uplink_latency_ms"] == 30.0
    assert result["profile"]["candidate"]["jitter_ms"] == 9.0
    generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
    assert generated["metadata"]["search"]["netem_seed"] == 12345
    assert generated["metadata"]["search"]["jitter_guard_ratio"] == 0.1
    assert generated["metadata"]["search"]["latency_base_ms"] == 30.0
    assert generated["profiles"]["requirements_final"]["uplink"]["delay"] == "30ms"
    assert generated["profiles"]["requirements_final"]["uplink"]["jitter"] == "9ms"
    assert generated["profiles"]["requirements_final"]["uplink"]["seed"] == 12345


def test_requirements_bandwidth_refine_recovers_when_previous_fail_becomes_pass(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_rate_bps

    generated_file = tmp_path / "generated-profiles.yaml"
    four_mbit_seen = 0

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        nonlocal four_mbit_seen
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        rate_bps = parse_rate_bps(generated["profiles"][profile]["uplink"]["rate"])
        if 3_900_000.0 <= rate_bps <= 4_100_000.0:
            four_mbit_seen += 1
            passes = four_mbit_seen >= 2
        else:
            passes = rate_bps > 4_100_000.0
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": 100 if passes else 0,
                    "lost": 0 if passes else 100,
                    "loss_pct": 0.0 if passes else 100.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 5.0, "p95": 5.0},
                    "jitter_ms": {"p50": 1.0, "p95": 1.0},
                }
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=5.0,
        max_latency_ms=250.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        bandwidth_high_bps=16_000_000.0,
        bandwidth_low_bps=1_000_000.0,
        latency_base_ms=0.0,
        latency_high_ms=0.0,
        jitter_high_ms=0.0,
        loss_high_pct=0.0,
        axes=_parse_requirements_axes("bandwidth"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=1,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=1,
        loss_coupling="jitter",
        bandwidth_guard_ratio=0.1,
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    assert result["bounds"]["bandwidth"]["status"] == "bounded_after_floor_reset"
    assert result["bounds"]["bandwidth"]["tight"] is True
    assert result["bounds"]["bandwidth"]["last_fail"]
    selected_bw = result["bounds"]["bandwidth"]["selected"]["bandwidth_bps"]
    assert result["profile"]["candidate"]["bandwidth_bps"] == pytest.approx(selected_bw * 1.1)
    assert result["analysis"]["guarded_axes"]["bandwidth"]["selected"]["bandwidth_bps"] == pytest.approx(selected_bw)


def test_requirements_bandwidth_probe_repeats_can_be_cheaper_than_final(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_rate_bps

    generated_file = tmp_path / "generated-profiles.yaml"

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        rate_bps = parse_rate_bps(generated["profiles"][profile]["uplink"]["rate"])
        passes = rate_bps >= 4_000_000.0
        return {
            "topics": {
                "/test": {
                    "expected": 100,
                    "delivered": 100,
                    "lost": 0,
                    "loss_pct": 0.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 50.0, "p95": 90.0 if passes else 300.0},
                    "jitter_ms": {"p50": 1.0, "p95": 1.0},
                }
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=0.0,
        max_latency_ms=250.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        bandwidth_high_bps=16_000_000.0,
        bandwidth_low_bps=1_000_000.0,
        latency_base_ms=30.0,
        latency_high_ms=30.0,
        jitter_high_ms=0.0,
        loss_high_pct=0.0,
        axes=_parse_requirements_axes("bandwidth"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=1,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=0,
        loss_coupling="jitter",
        probe_repeats=3,
        probe_min_passes=3,
        bad_lossy_count=1,
        bandwidth_probe_repeats=1,
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    bandwidth_rows = [row for row in result["rows"] if row["axis"] == "bandwidth"]
    assert bandwidth_rows
    assert all(row["repeat"]["configured_repeats"] == 1 for row in bandwidth_rows)
    assert result["rows"][0]["axis"] == "baseline"
    assert result["rows"][0]["repeat"]["configured_repeats"] == 3
    assert result["rows"][-1]["axis"] == "combined"
    assert result["rows"][-1]["repeat"]["configured_repeats"] == 3
    assert result["analysis"]["bandwidth_probe_repeats"] == 1
    assert result["analysis"]["final_passes"] is True


def test_loss_boundaries_driver_finds_discrete_good_and_bad_cases(tmp_path: Path) -> None:
    from rosotacom.network_profiles import parse_ms, parse_rate_bps

    generated_file = tmp_path / "generated-profiles.yaml"
    candidate_counts: dict[tuple[int, int], int] = {}

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        generated = yaml.safe_load(generated_file.read_text(encoding="utf-8"))
        spec = generated["profiles"][profile]["uplink"]
        bandwidth = parse_rate_bps(spec["rate"])
        jitter = parse_ms(spec.get("jitter", 0), "jitter")
        key = (int(round(bandwidth / 100_000.0)), int(round(jitter)))
        sample_index = candidate_counts.get(key, 0) + 1
        candidate_counts[key] = sample_index

        if bandwidth < 3_200_000.0:
            lossy = True
        elif jitter <= 18.0:
            lossy = False
        elif jitter >= 27.0:
            lossy = True
        else:
            lossy = sample_index % 2 == 0

        lost = 3 if lossy else 0
        expected = 400
        latency_p95 = 35.0 + jitter
        return {
            "topics": {
                "/test": {
                    "expected": expected,
                    "delivered": expected - lost,
                    "lost": lost,
                    "loss_pct": lost / expected * 100.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": jitter * 0.5, "p95": jitter},
                }
            }
        }

    result = drive_loss_boundaries(
        probe,
        max_latency_ms=250.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="/test",
        out_dir=tmp_path,
        axes=_parse_loss_boundary_axes("bandwidth,jitter"),
        bandwidth_low_bps=3_000_000.0,
        bandwidth_high_bps=4_000_000.0,
        bandwidth_step_bps=100_000.0,
        latency_base_ms=30.0,
        jitter_low_ms=0.0,
        jitter_high_ms=30.0,
        jitter_step_ms=1.0,
        min_duration_s=20.0,
        min_messages=100,
        distribution="normal",
        downlink_mode="lan",
        probe_repeats=10,
        good_clean_count=10,
        bad_lossy_count=10,
        result_context={"test": True},
        generated_profiles_file=generated_file,
    )

    bandwidth_boundary = result["boundaries"]["bandwidth"]
    assert bandwidth_boundary["good_boundary"]["candidate"]["bandwidth_bps"] == pytest.approx(3_200_000.0)
    assert bandwidth_boundary["bad_boundary"]["candidate"]["bandwidth_bps"] == pytest.approx(3_100_000.0)
    assert bandwidth_boundary["tight"] is True

    jitter_boundary = result["boundaries"]["jitter"]
    assert jitter_boundary["good_boundary"]["candidate"]["jitter_ms"] == pytest.approx(18.0)
    assert jitter_boundary["first_not_good"]["candidate"]["jitter_ms"] == pytest.approx(19.0)
    assert jitter_boundary["bad_boundary"]["candidate"]["jitter_ms"] == pytest.approx(27.0)
    assert jitter_boundary["last_not_bad"]["candidate"]["jitter_ms"] == pytest.approx(26.0)
    assert jitter_boundary["mixed_zone"] == {"low_ms": 19.0, "high_ms": 26.0}

    assert result["combined_good"]["classification"] == "good"
    assert (tmp_path / "loss-boundaries.json").is_file()
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["genre"] == "loss-boundaries"
    assert result_doc["result"]["analysis"]["seed_policy"] == "seedless"


def test_loss_boundaries_driver_consumes_replay_verdicts_across_topics(tmp_path: Path) -> None:
    # Bag-as-load: the probe returns a whole replay contract (two topics). The
    # boundary is driven by the hard stream /topic5; the light /topic9 always
    # clears. The search logic is unchanged — it consumes the aggregated verdict —
    # and the failing topic is named on the bad rows.
    from rosotacom.network_profiles import parse_ms, parse_rate_bps

    generated_file = tmp_path / "generated-profiles.yaml"
    candidate_counts: dict[tuple[int, int], int] = {}

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        spec = yaml.safe_load(generated_file.read_text(encoding="utf-8"))["profiles"][profile]["uplink"]
        bandwidth = parse_rate_bps(spec["rate"])
        jitter = parse_ms(spec.get("jitter", 0), "jitter")
        key = (int(round(bandwidth / 100_000.0)), int(round(jitter)))
        sample_index = candidate_counts.get(key, 0) + 1
        candidate_counts[key] = sample_index
        if bandwidth < 3_200_000.0:
            lossy = True
        elif jitter <= 18.0:
            lossy = False
        elif jitter >= 27.0:
            lossy = True
        else:
            lossy = sample_index % 2 == 0
        lost = 3 if lossy else 0
        latency_p95 = 35.0 + jitter
        return {
            "topics": {
                "a->b:/topic5": {
                    "expected": 400,
                    "delivered": 400 - lost,
                    "lost": lost,
                    "loss_pct": lost / 400 * 100.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": jitter * 0.5, "p95": jitter},
                },
                "b->a:/topic9": {
                    "expected": 200,
                    "delivered": 200,
                    "lost": 0,
                    "loss_pct": 0.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 20.0, "p95": 40.0},
                    "jitter_ms": {"p50": 1.0, "p95": 2.0},
                },
            }
        }

    result = drive_loss_boundaries(
        probe,
        max_latency_ms=250.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="",
        out_dir=tmp_path,
        axes=_parse_loss_boundary_axes("bandwidth"),
        bandwidth_low_bps=3_000_000.0,
        bandwidth_high_bps=4_000_000.0,
        bandwidth_step_bps=100_000.0,
        latency_base_ms=30.0,
        jitter_low_ms=0.0,
        jitter_high_ms=30.0,
        jitter_step_ms=1.0,
        min_duration_s=20.0,
        min_messages=100,
        distribution="normal",
        downlink_mode="lan",
        probe_repeats=10,
        good_clean_count=10,
        bad_lossy_count=10,
        result_context={"test": True},
        generated_profiles_file=generated_file,
        ground_truth={"/topic5": {"count": 400}, "/topic9": {"count": 200}},
        required_topics=["/topic5", "/topic9"],
        target={"name": "15_remote_assist_anonymized_costmap", "type": "session"},
        bag={"metadata": "bags/costmap/metadata.yaml"},
    )

    bandwidth_boundary = result["boundaries"]["bandwidth"]
    assert bandwidth_boundary["good_boundary"]["candidate"]["bandwidth_bps"] == pytest.approx(3_200_000.0)
    assert bandwidth_boundary["bad_boundary"]["candidate"]["bandwidth_bps"] == pytest.approx(3_100_000.0)
    # The whole-contract verdict is driven by the hard topic, named on the bad row.
    bad_row = next(row for row in result["rows"] if row["classification"] == "bad")
    assert bad_row["failing_topics"] == ["/topic5"]
    assert bad_row["samples"][0]["per_topic"][0]["topic"] in {"/topic5", "/topic9"}
    # Replay identity is recorded in the result.
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["result"]["replay"]["target"]["name"] == "15_remote_assist_anonymized_costmap"
    assert result_doc["result"]["replay"]["bag"]["metadata"].endswith("metadata.yaml")


def test_requirements_driver_consumes_replay_verdicts_with_completeness_floor(tmp_path: Path) -> None:
    # A tight profile must carry the whole contract: /topic5 needs bandwidth, and
    # /topic9 is incomplete below 2 Mbit/s. The search reuses the requirements
    # logic against the aggregated per-topic verdict.
    from rosotacom.network_profiles import parse_rate_bps

    generated_file = tmp_path / "generated-profiles.yaml"

    def probe(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        assert profile is not None
        spec = yaml.safe_load(generated_file.read_text(encoding="utf-8"))["profiles"][profile]["uplink"]
        bandwidth = parse_rate_bps(spec["rate"])
        topic5_lost = max(0, int(round((4_000_000.0 - bandwidth) / 1_000_000.0 * 4.0)))
        topic9_delivered = 200 if bandwidth >= 2_000_000.0 else 150
        return {
            "topics": {
                "a->b:/topic5": {
                    "expected": 400,
                    "delivered": 400 - topic5_lost,
                    "lost": topic5_lost,
                    "loss_pct": topic5_lost / 400 * 100.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 40.0, "p95": 80.0},
                    "jitter_ms": {"p50": 1.0, "p95": 2.0},
                },
                "b->a:/topic9": {
                    "expected": topic9_delivered,
                    "delivered": topic9_delivered,
                    "lost": 0,
                    "loss_pct": 0.0,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": 20.0, "p95": 40.0},
                    "jitter_ms": {"p50": 1.0, "p95": 2.0},
                },
            }
        }

    result = drive_requirements(
        probe,
        max_loss_pct=5.0,
        max_latency_ms=250.0,
        rate_hz=20.0,
        size=18_000,
        streams=1,
        qos_reliability="best_effort",
        qos_depth=1,
        topic="",
        out_dir=tmp_path,
        bandwidth_high_bps=10_000_000.0,
        bandwidth_low_bps=1_000_000.0,
        latency_base_ms=0.0,
        latency_high_ms=300.0,
        jitter_high_ms=60.0,
        loss_high_pct=10.0,
        axes=_parse_requirements_axes("bandwidth"),
        min_duration_s=20.0,
        min_messages=100,
        search_iterations=6,
        search_rounds=1,
        distribution="normal",
        final_refine_iterations=0,
        loss_coupling="independent",
        result_context={"test": True},
        generated_profiles_file=generated_file,
        ground_truth={"/topic5": {"count": 400}, "/topic9": {"count": 200}},
        required_topics=["/topic5", "/topic9"],
        oracle_min_completeness=0.95,
        target={"name": "15_remote_assist_anonymized_costmap", "type": "session"},
        bag={"metadata": "bags/costmap/metadata.yaml"},
    )

    # Passing requires both topics: /topic5 loss-bounded AND /topic9 complete
    # (>=190/200), so the tight bandwidth sits at/above the 2 Mbit/s completeness knee.
    assert result["analysis"]["final_passes"] is True
    assert result["profile"]["candidate"]["bandwidth_bps"] >= 2_000_000.0
    result_doc = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text(encoding="utf-8"))
    assert result_doc["result"]["replay"]["required_topics"] == ["/topic5", "/topic9"]
    assert result_doc["result"]["replay"]["oracle_min_completeness"] == 0.95


def test_requirements_zero_loss_target_reports_loss_as_limiter() -> None:
    quality = _requirements_target_quality(
        {"loss_pct": 0.25, "latency_p95_ms": 10.0},
        max_loss_pct=0.0,
        max_latency_ms=250.0,
    )

    assert quality["limiting_metric"] == "loss"
    assert quality["utilization"] == math.inf


def test_profile_requires_netem_seed_only_when_seeded_netem_is_generated() -> None:
    from rosotacom.network_profiles import DirectionShaping, Profile, TimelineSegment

    rate_only = Profile("rate-only", "static", uplink=DirectionShaping(rate_bps=1_000_000.0, seed=123))
    seeded_static = Profile(
        "seeded-static",
        "static",
        uplink=DirectionShaping(delay_ms=30.0, jitter_ms=10.0, seed=123),
    )
    seeded_timeline = Profile(
        "seeded-timeline",
        "timeline",
        timeline=(
            TimelineSegment(for_s=1.0, uplink=DirectionShaping(rate_bps=1_000_000.0)),
            TimelineSegment(for_s=1.0, downlink=DirectionShaping(delay_ms=30.0, jitter_ms=10.0, seed=123)),
        ),
    )

    assert _profile_requires_netem_seed(None) is False
    assert _profile_requires_netem_seed(rate_only) is False
    assert _profile_requires_netem_seed(seeded_static) is True
    assert _profile_requires_netem_seed(seeded_timeline) is True


# --------------------------------------------------------------------------- #
# Band gate: compare + ratchet roundtrip (CLI-level, performance bands reference)
# --------------------------------------------------------------------------- #


def _capacity_run(tmp_path: Path, name: str, breakpoint_size: int) -> Path:
    """One stubbed capacity run; returns the run directory holding result.json."""
    run_dir = tmp_path / name
    run_dir.mkdir()
    drive_capacity(
        _make_stub_probe(breakpoint_size=breakpoint_size),
        profile="p",
        knob="size",
        low=1000,
        high=10000,
        max_loss_pct=5.0,
        max_latency_ms=200.0,
        duration_s=1.0,
        out_dir=run_dir,
    )
    return run_dir


def _band_args(results: list[Path], budgets: Path, **overrides: Any) -> argparse.Namespace:
    args: dict[str, Any] = {
        "results": [str(result) for result in results],
        "budgets": str(budgets),
        "row": None,
        "profile": None,
        "metric": None,
        "note": "",
        "recalibrate": False,
        "better": None,
        "k": 3.0,
        "floor": 0.0,
        "floor_frac": 0.02,
        "monitor": False,
    }
    args.update(overrides)
    return argparse.Namespace(**args)


def test_capacity_run_ratchet_compare_is_within(tmp_path: Path) -> None:
    """The performance bands reference roundtrip: run → ratchet --recalibrate → compare is WITHIN."""
    run = _capacity_run(tmp_path, "calibration", breakpoint_size=5000)
    budgets = tmp_path / "budgets.jsonl"
    assert benchmark_cli.benchmark_ratchet(_band_args([run], budgets, recalibrate=True, note="initial")) == 0
    # A run directory or its result.json both address the same run.
    assert benchmark_cli.benchmark_compare(_band_args([run / BENCHMARK_RESULT_FILE], budgets)) == 0


def test_compare_gates_both_sides_and_banks_improvements(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """REGRESSED exits 1; IMPROVED exits 2 printing the exact ratchet command;
    running that command banks the improvement and compare returns to WITHIN."""
    from rosotacom.benchmark import WideningRefused, load_bands

    calibration = _capacity_run(tmp_path, "calibration", breakpoint_size=5000)
    regressed = _capacity_run(tmp_path, "regressed", breakpoint_size=4000)
    improved = _capacity_run(tmp_path, "improved", breakpoint_size=6000)
    budgets = tmp_path / "budgets.jsonl"
    assert benchmark_cli.benchmark_ratchet(_band_args([calibration], budgets, recalibrate=True)) == 0
    capsys.readouterr()

    # Worse side: gate-red, and a plain ratchet refuses to loosen the band toward it.
    assert benchmark_cli.benchmark_compare(_band_args([regressed], budgets)) == 1
    assert "REGRESSED" in capsys.readouterr().out
    with pytest.raises(WideningRefused, match="--recalibrate"):
        benchmark_cli.benchmark_ratchet(_band_args([regressed], budgets))

    # Better side: red too, with the exact ratchet command in the happy message.
    improved_result = improved / BENCHMARK_RESULT_FILE
    assert benchmark_cli.benchmark_compare(_band_args([improved_result], budgets)) == 2
    out = capsys.readouterr().out
    assert "IMPROVED" in out
    assert f"rosotacom benchmark ratchet {improved_result} --budgets {budgets}" in out
    # Monitor lanes report without blocking.
    assert benchmark_cli.benchmark_compare(_band_args([improved_result], budgets, monitor=True)) == 0
    capsys.readouterr()

    # Run the printed command: the band re-centers, calibration provenance survives.
    before = load_bands(budgets)[0]
    assert benchmark_cli.benchmark_ratchet(_band_args([improved_result], budgets, note="stub got faster")) == 0
    after = load_bands(budgets)[0]
    assert after.center == 6000.0
    assert after.half_width == before.half_width
    assert after.provenance.sigma == before.provenance.sigma
    assert after.provenance.floor == before.provenance.floor
    assert after.provenance.note == "stub got faster"
    assert benchmark_cli.benchmark_compare(_band_args([improved_result], budgets)) == 0


def test_compare_refuses_bands_from_another_runner_class(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Uncalibrated or foreign-runner bands refuse to gate (exit 1), naming the fix;
    --monitor reports the refusal without blocking."""
    import dataclasses

    from rosotacom.benchmark import UNCALIBRATED_FINGERPRINT, load_bands, save_bands

    run = _capacity_run(tmp_path, "run", breakpoint_size=5000)
    budgets = tmp_path / "budgets.jsonl"
    assert benchmark_cli.benchmark_ratchet(_band_args([run], budgets, recalibrate=True)) == 0
    doctored = [
        dataclasses.replace(band, provenance=dataclasses.replace(band.provenance, fingerprint=UNCALIBRATED_FINGERPRINT))
        for band in load_bands(budgets)
    ]
    save_bands(budgets, doctored)
    capsys.readouterr()

    assert benchmark_cli.benchmark_compare(_band_args([run], budgets)) == 1
    out = capsys.readouterr().out
    assert "REFUSED" in out
    assert "--recalibrate" in out

    assert benchmark_cli.benchmark_compare(_band_args([run], budgets, monitor=True)) == 0
    out = capsys.readouterr().out
    assert "REFUSED" in out


# --------------------------------------------------------------------------- #
# Benched set: row / calibrate / gate-summary (performance bands reference)
# --------------------------------------------------------------------------- #


@pytest.fixture()
def examples_project(tmp_path: Path) -> Path:
    project = tmp_path / "examples"
    assert cli.main(["examples", "create", str(project)]) == 0
    return project


def _run_row(
    monkeypatch: pytest.MonkeyPatch,
    project: Path,
    out_root: Path,
    row_id: str,
    *,
    stub: Any,
    budgets: Path,
    verdict: Path,
    extra: list[str] | None = None,
) -> int:
    monkeypatch.setattr(benchmark_cli, "_make_live_run_point", lambda args, session_name: stub)
    return cli.main(
        [
            "benchmark",
            "row",
            row_id,
            "--rosotacom-config",
            str(project / "rosotacom.yaml"),
            "--artifacts-dir",
            str(out_root),
            "--budgets",
            str(budgets),
            "--verdict-file",
            str(verdict),
            *(extra or []),
        ]
    )


def test_row_lifecycle_calibrate_gate_regress_improve(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The full performance bands reference row lifecycle against the packaged registry: an ungated
    calibration run, `calibrate` minting the banded metrics with the committed
    floor, then WITHIN / REGRESSED / IMPROVED / monitor verdicts end-to-end."""
    from rosotacom.benchmark import load_bands

    budgets = tmp_path / "budgets.jsonl"
    row_id = "probe-loss-tight-cyclone"

    # Calibration repeats run ungated and record a RAN verdict.
    verdict_path = tmp_path / "calib-verdict.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "calib",
        row_id,
        stub=_make_stub_probe(loss_pct=48.0),
        budgets=budgets,
        verdict=verdict_path,
        extra=["--no-compare"],
    )
    assert rc == 0
    calib_verdict = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert calib_verdict["verdict"] == "RAN"
    assert calib_verdict["gate"] is False
    calib_run = Path(calib_verdict["result"]).parent

    # calibrate mints bands only for the gated metrics, with the committed floor.
    report = tmp_path / "report.json"
    rc = cli.main(
        [
            "benchmark",
            "calibrate",
            row_id,
            str(calib_run),
            "--budgets",
            str(budgets),
            "--report",
            str(report),
        ]
    )
    assert rc == 0
    bands = load_bands(budgets)
    assert [band.metric for band in bands] == ["loss_pct"]
    assert bands[0].row == row_id
    assert bands[0].profile == "gate-tight"
    # One repeat means sigma 0: the width is pure floor — the larger of the
    # registry's committed floor and the default floor_frac of the center.
    assert bands[0].half_width == pytest.approx(max(0.5, 0.02 * 48.0))
    assert bands[0].provenance.floor >= 0.5, "the registry's committed floor is respected"
    report_doc = json.loads(report.read_text(encoding="utf-8"))
    assert report_doc["metrics"]["loss_pct"]["banded"] is True
    assert report_doc["metrics"]["latency_p95_ms"]["banded"] is False, "monitor metrics keep their spread evidence"

    # WITHIN gates green and the verdict is machine-readable.
    within_verdict = tmp_path / "within.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "within",
        row_id,
        stub=_make_stub_probe(loss_pct=48.0),
        budgets=budgets,
        verdict=within_verdict,
    )
    assert rc == 0
    within_doc = json.loads(within_verdict.read_text(encoding="utf-8"))
    assert within_doc["verdict"] == "WITHIN"
    assert within_doc["metrics"] == {"loss_pct": 48.0}
    assert within_doc["monitor_metrics"]["latency_p95_ms"] == 50.0
    assert within_doc["bands"]["loss_pct"]["better"] == "lower"

    # REGRESSED is red (exit 1).
    regressed_verdict = tmp_path / "regressed.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "regressed",
        row_id,
        stub=_make_stub_probe(loss_pct=60.0),
        budgets=budgets,
        verdict=regressed_verdict,
    )
    assert rc == 1
    assert json.loads(regressed_verdict.read_text(encoding="utf-8"))["verdict"] == "REGRESSED"

    # IMPROVED is red too (exit 2) and the verdict carries the exact ratchet command.
    improved_verdict = tmp_path / "improved.json"
    capsys.readouterr()
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "improved",
        row_id,
        stub=_make_stub_probe(loss_pct=40.0),
        budgets=budgets,
        verdict=improved_verdict,
    )
    assert rc == 2
    improved_doc = json.loads(improved_verdict.read_text(encoding="utf-8"))
    assert improved_doc["verdict"] == "IMPROVED"
    assert "rosotacom benchmark ratchet" in improved_doc["ratchet_command"]
    assert "IMPROVED" in capsys.readouterr().out

    # --monitor reports the same verdict without blocking.
    monitor_verdict = tmp_path / "monitor.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "monitor",
        row_id,
        stub=_make_stub_probe(loss_pct=60.0),
        budgets=budgets,
        verdict=monitor_verdict,
        extra=["--monitor"],
    )
    assert rc == 0
    monitor_doc = json.loads(monitor_verdict.read_text(encoding="utf-8"))
    assert monitor_doc["verdict"] == "REGRESSED"
    assert monitor_doc["gate"] is False


def test_row_refuses_without_bands_and_when_a_gated_band_is_missing(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    row_id = "probe-loss-tight-cyclone"
    budgets = tmp_path / "budgets.jsonl"
    budgets.write_text("", encoding="utf-8")

    verdict_path = tmp_path / "verdict.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "runs",
        row_id,
        stub=_make_stub_probe(loss_pct=48.0),
        budgets=budgets,
        verdict=verdict_path,
    )
    assert rc == 1
    doc = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert doc["verdict"] == "REFUSED"
    assert "calibrate" in doc["refusal"]
    run_dir = Path(doc["result"]).parent

    # A band store that lost the gated metric refuses instead of silently not gating.
    ratchet_args = _band_args(
        [run_dir], budgets, row=row_id, profile="gate-tight", recalibrate=True, metric=["latency_p95_ms"]
    )
    assert benchmark_cli.benchmark_ratchet(ratchet_args) == 0
    capsys.readouterr()
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "runs2",
        row_id,
        stub=_make_stub_probe(loss_pct=48.0),
        budgets=budgets,
        verdict=verdict_path,
    )
    assert rc == 1
    assert "loss_pct" in json.loads(verdict_path.read_text(encoding="utf-8"))["refusal"]


def _make_stub_replay(*, expected: int, delivered: int, latency_p95: float = 50.0) -> Any:
    """A run_point returning one replay-sized topic summary."""
    lost = expected - delivered

    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        return {
            "topics": {
                "/topic1": {
                    "expected": expected,
                    "delivered": delivered,
                    "lost": lost,
                    "loss_pct": 100.0 * lost / expected,
                    "reordered": 0,
                    "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                    "jitter_ms": {"p50": 2.0, "p95": 5.0},
                }
            }
        }

    return stub


def test_replay_row_gates_on_the_whole_bag_expect_without_any_band(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The point of the replay genre landing before its calibration: the row
    already asserts something. With an empty band store a banded row refuses,
    while a replay row runs, judges the bag's whole-bag expect fragment, and
    keeps the delivery metrics as monitor evidence for the calibration."""
    row_id = "replay-costmap-nominal-cyclone"
    budgets = tmp_path / "budgets.jsonl"
    budgets.write_text("", encoding="utf-8")

    # 1400 messages over the row's 140 s window: the whole bag arrived.
    verdict_path = tmp_path / "within.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "within",
        row_id,
        stub=_make_stub_replay(expected=1400, delivered=1400),
        budgets=budgets,
        verdict=verdict_path,
    )
    assert rc == 0, "an unbanded replay row must run, not refuse for want of bands"
    doc = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert doc["verdict"] == "WITHIN"
    assert doc["genre"] == "replay"
    assert doc["metrics"] == {}
    assert doc["replay_expect"]["passed"] is True
    assert doc["replay_expect"]["bag"]["public_example"] == "15_remote_assist_anonymized_costmap"
    assert doc["monitor_metrics"]["delivered_count"] == 1400.0
    assert doc["monitor_metrics"]["delivered_hz"] == pytest.approx(10.0)
    assert doc["monitor_metrics"]["loss_pct"] == 0.0

    # Half the bag: min_count, completeness and the bag-relative rate all fail.
    capsys.readouterr()
    failed_path = tmp_path / "failed.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "failed",
        row_id,
        stub=_make_stub_replay(expected=1400, delivered=700),
        budgets=budgets,
        verdict=failed_path,
    )
    assert rc == 1
    failed = json.loads(failed_path.read_text(encoding="utf-8"))
    assert failed["verdict"] == "EXPECT_FAILED"
    assert len(failed["replay_expect"]["failures"]) == 3
    out = capsys.readouterr().out
    assert "Replay expect FAILED" in out
    assert "min_count 1256" in out

    # --monitor reports the same verdict without blocking.
    monitor_path = tmp_path / "monitor.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "monitor",
        row_id,
        stub=_make_stub_replay(expected=1400, delivered=700),
        budgets=budgets,
        verdict=monitor_path,
        extra=["--monitor"],
    )
    assert rc == 0
    monitored = json.loads(monitor_path.read_text(encoding="utf-8"))
    assert monitored["verdict"] == "EXPECT_FAILED"
    assert monitored["gate"] is False


def test_replay_calibration_run_reports_spread_for_the_metrics_a_band_would_take(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path
) -> None:
    """Step 2 of issue #185 needs evidence, not bands: `--no-compare` runs the
    row ungated and `calibrate` reports the per-metric spread even though the
    row bands nothing yet."""
    row_id = "replay-camera-tight-cyclone"
    budgets = tmp_path / "budgets.jsonl"
    verdict_path = tmp_path / "calib.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "calib",
        row_id,
        stub=_make_stub_replay(expected=1400, delivered=1390),
        budgets=budgets,
        verdict=verdict_path,
        extra=["--no-compare"],
    )
    assert rc == 0
    calib_verdict = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert calib_verdict["verdict"] == "RAN"
    assert calib_verdict["gate"] is False
    calib_run = Path(calib_verdict["result"]).parent

    report = tmp_path / "report.json"
    rc = cli.main(
        ["benchmark", "calibrate", row_id, str(calib_run), "--budgets", str(budgets), "--report", str(report)]
    )
    assert rc == 0
    assert not budgets.exists() or budgets.read_text(encoding="utf-8").strip() == "", (
        "a row that bands nothing must not mint bands"
    )
    metrics = json.loads(report.read_text(encoding="utf-8"))["metrics"]
    assert metrics["loss_pct"]["banded"] is False
    assert metrics["delivered_count"]["median"] == 1390.0


def test_capacity_row_gates_the_breakpoint(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path
) -> None:
    row_id = "capacity-size-tight-cyclone"
    budgets = tmp_path / "budgets.jsonl"

    verdict_path = tmp_path / "calib.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "calib",
        row_id,
        stub=_make_stub_probe(breakpoint_size=5000),
        budgets=budgets,
        verdict=verdict_path,
        extra=["--no-compare"],
    )
    assert rc == 0
    calib_run = Path(json.loads(verdict_path.read_text(encoding="utf-8"))["result"]).parent
    assert cli.main(["benchmark", "calibrate", row_id, str(calib_run), "--budgets", str(budgets)]) == 0

    gate_verdict = tmp_path / "gate.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "gate",
        row_id,
        stub=_make_stub_probe(breakpoint_size=5000),
        budgets=budgets,
        verdict=gate_verdict,
    )
    assert rc == 0
    doc = json.loads(gate_verdict.read_text(encoding="utf-8"))
    assert doc["verdict"] == "WITHIN"
    assert doc["metrics"] == {"capacity_size": 5000.0}

    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "worse",
        row_id,
        stub=_make_stub_probe(breakpoint_size=4000),
        budgets=budgets,
        verdict=gate_verdict,
    )
    assert rc == 1
    assert json.loads(gate_verdict.read_text(encoding="utf-8"))["verdict"] == "REGRESSED"


def _write_boundary_band(budgets: Path, row_id: str) -> None:
    from rosotacom.benchmark import Band, BandProvenance, Better, runner_fingerprint, save_bands

    save_bands(
        budgets,
        [
            Band(
                row=row_id,
                profile="finding-3.2mbit",
                metric="loss_pct",
                lo=-0.5,
                hi=0.5,
                better=Better.LOWER,
                provenance=BandProvenance(
                    fingerprint=runner_fingerprint(),
                    window_s=60.0,
                    repeats=1,
                    sigma=0.0,
                    floor=0.5,
                    k=3.0,
                    source_sha="test",
                    ratcheted_at="2026-07-09T00:00:00",
                    note="test boundary band",
                ),
            )
        ],
    )


def test_boundary_row_requires_the_bad_side_to_stay_bad(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path
) -> None:
    row_id = "boundary-loss-18kb20hz-bandwidth-cyclone"
    budgets = tmp_path / "budgets.jsonl"
    _write_boundary_band(budgets, row_id)

    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        loss = 3.0 if profile == "finding-3.1mbit" else 0.0
        return _make_stub_probe(loss_pct=loss)(profile=profile, load=load, duration_s=duration_s, out_dir=out_dir)

    verdict_path = tmp_path / "boundary.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "boundary",
        row_id,
        stub=stub,
        budgets=budgets,
        verdict=verdict_path,
    )

    assert rc == 0
    doc = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert doc["verdict"] == "WITHIN"
    assert doc["boundary"]["sides"]["bad"]["metrics"]["loss_pct"] == pytest.approx(3.0)


def test_boundary_row_happy_reds_when_bad_side_meets_the_good_oracle(
    monkeypatch: pytest.MonkeyPatch,
    examples_project: Path,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    row_id = "boundary-loss-18kb20hz-bandwidth-cyclone"
    budgets = tmp_path / "budgets.jsonl"
    _write_boundary_band(budgets, row_id)

    verdict_path = tmp_path / "boundary-widened.json"
    rc = _run_row(
        monkeypatch,
        examples_project,
        tmp_path / "boundary-widened",
        row_id,
        stub=_make_stub_probe(loss_pct=0.0),
        budgets=budgets,
        verdict=verdict_path,
    )

    assert rc == 2
    out = capsys.readouterr().out
    assert "BOUNDARY_WIDENED" in out
    assert "docs/findings/zero-loss-boundaries-18kb-20hz.md" in out
    assert "good=finding-3.2mbit, bad=finding-3.1mbit" in out
    assert "rosotacom benchmark ratchet" in out
    doc = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert doc["verdict"] == "BOUNDARY_WIDENED"
    assert doc["boundary"]["finding"] == "docs/findings/zero-loss-boundaries-18kb-20hz.md"
    assert doc["boundary"]["bad_profile"] == "finding-3.1mbit"
    assert "rosotacom benchmark ratchet" in doc["ratchet_command"]


def test_gate_summary_aggregates_rows_and_reds_on_missing_or_regressed(tmp_path: Path) -> None:
    from rosotacom.benched_set import find_row, load_registry, rows_for_lane, verdict_document, write_verdict

    rows = rows_for_lane(load_registry(), "nightly")
    verdicts_dir = tmp_path / "verdicts"

    def summary(out_name: str) -> tuple[int, dict[str, Any]]:
        out = tmp_path / out_name
        rc = cli.main(
            ["benchmark", "gate-summary", "--verdicts", str(verdicts_dir), "--lane", "nightly", "--out", str(out)]
        )
        return rc, json.loads(out.read_text(encoding="utf-8"))

    verdicts_dir.mkdir()
    rc, doc = summary("empty.json")
    assert rc == 1
    assert doc["overall"] == "red"
    assert doc["red_rows"] == [row.id for row in rows]

    def write(row_id: str, verdict: str, exit_code: int) -> None:
        row = find_row(rows, row_id)
        write_verdict(
            verdicts_dir / f"{row_id}.json",
            verdict_document(
                row,
                verdict=verdict,
                exit_code=exit_code,
                gate=True,
                sha="abc1234",
                fingerprint="github-hosted-linux-x86_64",
                created_at="2026-07-08T12:00:00",
                metrics={},
                monitor_metrics={},
                bands={},
                result_path="run/result.json",
            ),
        )

    for row in rows:
        write(row.id, "WITHIN", 0)
    rc, doc = summary("green.json")
    assert rc == 0
    assert doc["overall"] == "green"

    write(rows[0].id, "IMPROVED", 2)
    rc, doc = summary("improved.json")
    assert rc == 1
    assert doc["red_rows"] == [rows[0].id], "IMPROVED blocks: bank the ratchet, don't revert"


def test_row_accepts_a_relative_artifacts_dir(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path
) -> None:
    """The CI lanes pass cwd-relative --artifacts-dir; the run dir must resolve
    absolute before it becomes a session-configs source, or session resolution
    later re-anchors it under the project config's directory (the first
    calibration run failed exactly this way)."""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(
        benchmark_cli, "_make_live_run_point", lambda args, session_name: _make_stub_probe(loss_pct=48.0)
    )
    rc = cli.main(
        [
            "benchmark",
            "row",
            "probe-loss-tight-cyclone",
            "--rosotacom-config",
            str(examples_project / "rosotacom.yaml"),
            "--artifacts-dir",
            "rel-artifacts",
            "--no-compare",
            "--verdict-file",
            "rel-verdicts/row.json",
        ]
    )
    assert rc == 0
    verdict = json.loads((tmp_path / "rel-verdicts" / "row.json").read_text(encoding="utf-8"))
    assert verdict["verdict"] == "RAN"
    assert Path(verdict["result"]).is_absolute()
    assert Path(verdict["result"]).is_file()


def test_ota_row_with_explicit_target_skips_benchmark_session_copy(
    monkeypatch: pytest.MonkeyPatch, examples_project: Path, tmp_path: Path
) -> None:
    def fail_prepare(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise AssertionError("explicit OTA target rows must not prepare the benchmark session")

    monkeypatch.setattr(benchmark_cli, "_prepare_benchmark_session_config", fail_prepare)
    monkeypatch.setattr(
        benchmark_cli, "_make_live_run_point", lambda args, session_name: _make_stub_probe(loss_pct=0.0)
    )

    verdict_path = tmp_path / "verdict.json"
    rc = cli.main(
        [
            "ota-benchmark",
            "row",
            "probe-loss-tight-cyclone",
            "--rosotacom-config",
            str(examples_project / "rosotacom.yaml"),
            "--target",
            "remote_assist",
            "--target-type",
            "scenario",
            "--artifacts-dir",
            str(tmp_path / "runs"),
            "--no-compare",
            "--verdict-file",
            str(verdict_path),
        ]
    )

    assert rc == 0
    verdict = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert verdict["verdict"] == "RAN"
    result = json.loads(Path(verdict["result"]).read_text(encoding="utf-8"))
    assert result["context"]["session"]["prepared"] is False
    assert result["context"]["session"]["target_override"] == {
        "target": "remote_assist",
        "target_type": "scenario",
    }


def test_probe_point_dirnames_are_artifact_safe() -> None:
    """Pattern loads carry `a*1,b*1` and bracketed size lists; the copied probe
    directory name must stay shell- and upload-artifact-safe (no `*?"<>|[] `)."""
    from rosotacom.benchmark import parse_size_pattern_load
    from rosotacom.cli_benchmark import _probe_point_dirname

    load = parse_size_pattern_load("1x12KB+1x2KB+3x3KB")
    load["rate"] = 20.0
    load["interval_jitter_ms"] = 20.0
    name = _probe_point_dirname("deadbeef", load)

    assert not set(name) & set('*?"<>|[] ,:'), name
    assert name.startswith("probe_deadbeef")


def test_rows_ids_format_is_a_workflow_matrix(capsys: pytest.CaptureFixture[str]) -> None:
    assert cli.main(["benchmark", "rows", "--lane", "merge-gate", "--format", "ids"]) == 0
    ids = json.loads(capsys.readouterr().out)
    assert isinstance(ids, list) and ids
    assert "probe-loss-gop-tight-cyclone" in ids


# --------------------------------------------------------------------------- #
# Value parser
# --------------------------------------------------------------------------- #


def test_parse_values_comma_separated() -> None:
    assert _parse_values("1000,2000,4000") == [1000.0, 2000.0, 4000.0]


def test_parse_values_range_syntax() -> None:
    values = _parse_values("1000:5000:1000")
    assert values == [1000.0, 2000.0, 3000.0, 4000.0, 5000.0]


def test_parse_values_linspace_syntax() -> None:
    values = _parse_values("0..100/5")
    assert len(values) == 5
    assert values[0] == 0.0
    assert values[-1] == pytest.approx(100.0)


# --------------------------------------------------------------------------- #
# Transit-record collection
# --------------------------------------------------------------------------- #


def test_collect_transit_summary_from_fixture(tmp_path: Path) -> None:
    """Transit records from a fixture directory structure are summarized correctly."""
    events_dir = tmp_path / "logs" / "a" / "status"
    events_dir.mkdir(parents=True)
    events_path = events_dir / "events.jsonl"
    records = [
        {
            "kind": "transit",
            "topic": "/t",
            "seq": 0,
            "status": "delivered",
            "t_wrap": 0.0,
            "sections": {"ota_hop_ms": 50.0},
        },
        {
            "kind": "transit",
            "topic": "/t",
            "seq": 1,
            "status": "delivered",
            "t_wrap": 0.1,
            "sections": {"ota_hop_ms": 55.0},
        },
        {
            "kind": "transit",
            "topic": "/t",
            "seq": 2,
            "status": "lost",
            "t_wrap": None,
            "sections": {"ota_hop_ms": None},
        },
    ]
    events_path.write_text("\n".join(json.dumps(r) for r in records) + "\n", encoding="utf-8")

    summary = collect_transit_summary(tmp_path)
    topics = summary.get("topics", {})
    # There should be one topic entry.
    assert len(topics) >= 1
    topic_key = next(iter(topics))
    assert topics[topic_key]["expected"] == 3
    assert topics[topic_key]["delivered"] == 2
    assert topics[topic_key]["lost"] == 1


def test_live_lab_run_point_uses_instance_scoped_smoke_network(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    profiles_file = tmp_path / "profiles" / "benchmark-profiles.yaml"
    profiles_file.parent.mkdir()
    profiles_file.write_text(
        "profiles:\n"
        "  cellular-4g-degraded:\n"
        "    uplink: { rate: 1mbit, delay: 180ms, jitter: 50ms, loss: 3% }\n"
        "    downlink: { rate: 10mbit, delay: 100ms, jitter: 30ms, loss: 1% }\n",
        encoding="utf-8",
    )
    monkeypatch.chdir(tmp_path)

    sessions_root = tmp_path / "sessions"
    session_dir = sessions_root / "bench_1_1_capacity"
    session_dir.mkdir(parents=True)
    instance = cli.SessionInstance(
        instance_id="bench",
        host_dir=tmp_path / "instances" / "2026-01-01" / "bench_1_1_capacity_bench",
        container_dir="/session/instances/2026-01-01/bench_1_1_capacity_bench",
        config_host_dir=tmp_path / "instances" / "2026-01-01" / "bench_1_1_capacity_bench" / "config",
        config_container_dir="/session/instances/2026-01-01/bench_1_1_capacity_bench/config",
        logs_host_dir=tmp_path / "instances" / "2026-01-01" / "bench_1_1_capacity_bench" / "logs",
        logs_container_dir="/session/instances/2026-01-01/bench_1_1_capacity_bench/logs",
        rosbags_host_dir=tmp_path / "instances" / "2026-01-01" / "bench_1_1_capacity_bench" / "rosbags",
        rosbags_container_dir="/session/instances/2026-01-01/bench_1_1_capacity_bench/rosbags",
    )
    status_dir = instance.host_dir / "logs" / "b" / "status"
    status_dir.mkdir(parents=True)
    (status_dir / "status.json").write_text(
        json.dumps({"topics": [{"base": "/bench_capacity", "stages": [{"messages_total": 1}]}]}),
        encoding="utf-8",
    )
    runtime = cli.RuntimeConfig(
        None,
        tmp_path / "ros2docker.json",
        (sessions_root,),
        None,
        "id",
        tmp_path / "instances",
    )
    session = cli.ResolvedSession(session_dir, "/session/definitions/bench_1_1_capacity", "session_configs")
    smoke_network = cli._noninteractive_smoke_network_config(runtime, session, instance.instance_id)
    started: list[argparse.Namespace] = []
    networks_created: list[tuple[str, str]] = []
    networks_removed: list[str] = []
    stopped: list[str] = []

    monkeypatch.setattr(cli, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(cli, "_resolve_session", lambda session_name, runtime: session)
    monkeypatch.setattr(cli, "_resolve_session_instance", lambda runtime, session, instance_id=None: instance)
    # One a->b topic, as every benchmark session has: which container meets the
    # profile's `uplink` leg follows the session's own loaded direction, so a
    # fixture with no topics would be asserting an arbitrary default rather than
    # the rule.
    monkeypatch.setattr(
        cli,
        "_effective_session_config",
        lambda *args, **kwargs: {"topics": {"a_to_b": [{"topic": "/bench_capacity"}], "b_to_a": []}},
    )
    monkeypatch.setattr(cli, "_list_docker_containers", lambda all_states=False: [])
    monkeypatch.setattr(
        cli, "_ensure_smoke_network", lambda name, subnet, labels=None: networks_created.append((name, subnet))
    )
    monkeypatch.setattr(cli, "start_session", lambda args: started.append(args) or f"container_{args.identity}")
    monkeypatch.setattr(cli, "_smoke_ros_setup", lambda *args: "source /ros")
    monkeypatch.setattr(cli, "_write_docker_log", lambda *args: None)
    monkeypatch.setattr(cli, "_stop_container_name", lambda name, runtime: stopped.append(name) or True)
    monkeypatch.setattr(cli, "_remove_smoke_network", lambda name: networks_removed.append(name))
    publish_windows: list[tuple[float, float] | None] = []

    def fake_collect_transit_summary(
        instance_dir: Path,
        *,
        publish_window: tuple[float, float] | None = None,
    ) -> dict[str, Any]:
        publish_windows.append(publish_window)
        return {"topics": {}}

    monkeypatch.setattr(benchmark_cli, "collect_transit_summary", fake_collect_transit_summary)
    sleep_calls: list[float] = []
    monkeypatch.setattr(benchmark_cli.time, "sleep", lambda seconds: sleep_calls.append(float(seconds)))
    commands: list[list[str]] = []

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        commands.append(command)
        return subprocess.CompletedProcess(command, 0, "Subscription count: 1\n", "")

    monkeypatch.setattr(subprocess, "run", fake_run)
    out_dir = tmp_path / "out"
    out_dir.mkdir()
    args = argparse.Namespace(
        rosotacom_config=None,
        ros2docker_config=None,
        session_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        dry_run=False,
        instance_id=instance.instance_id,
        profile=None,
    )

    run_point = benchmark_cli._make_live_run_point(args, "bench_1_1_capacity")

    assert run_point(profile="cellular-4g-degraded", load={"size": 1}, duration_s=0.1, out_dir=out_dir) == {
        "topics": {}
    }
    assert networks_created == [(smoke_network.name, smoke_network.subnet)]
    assert [(args.identity, args.network_name, args.network_ip) for args in started] == [
        ("a", smoke_network.name, smoke_network.peer_ips["a"]),
        ("b", smoke_network.name, smoke_network.peer_ips["b"]),
    ]
    assert all(args.peer_address == cli._smoke_peer_address_args(smoke_network.peer_ips) for args in started)
    assert stopped == ["container_a", "container_b"]
    assert networks_removed == [smoke_network.name]
    assert publish_windows and publish_windows[0] is not None
    assert any(
        command[:6] == ["docker", "exec", "-u", "root", "container_a", "tc"] and "3%" in command for command in commands
    )
    assert any(
        command[:6] == ["docker", "exec", "-u", "root", "container_b", "tc"] and "1%" in command for command in commands
    )
    assert sleep_calls[-2:] == [0.1, DEFAULT_BENCHMARK_DRAIN_S]
    uplink_arm = next(
        i
        for i, command in enumerate(commands)
        if command[:8] == ["docker", "exec", "-u", "root", "container_a", "tc", "qdisc", "replace"]
    )
    publisher_stop = next(
        i
        for i, command in enumerate(commands)
        if i > uplink_arm and command == ["docker", "exec", "container_a", "pkill", "-f", "sized_publisher"]
    )
    final_uplink_teardown = max(
        i
        for i, command in enumerate(commands)
        if command[:8] == ["docker", "exec", "-u", "root", "container_a", "tc", "qdisc", "del"]
    )
    assert publisher_stop < final_uplink_teardown


# --------------------------------------------------------------------------- #
# Argparse wiring
# --------------------------------------------------------------------------- #


def test_benchmark_subcommand_arg_parsing() -> None:
    """The benchmark subcommand and its sub-subcommands parse without error."""
    import argparse

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command")
    register_benchmark_parser(subparsers)

    # Probe.
    args = parser.parse_args(["benchmark", "probe", "--profile", "losssless-typical"])
    assert args.benchmark_command == "probe"
    assert args.size == 18_000
    assert args.rate_hz == 20.0
    assert args.bin_s == 1.0
    assert args.plot is True
    assert args.size_pattern is None
    assert args.cyclone_spdp_interval is None

    args = parser.parse_args(["benchmark", "probe", "--profile", "losssless-typical", "--no-plot"])
    assert args.plot is False

    args = parser.parse_args(
        [
            "benchmark",
            "probe",
            "--profile",
            "lossless-typical",
            "--size-pattern",
            "1x20KB+1x0KB",
            "--rate-hz",
            "10",
            "--cyclone-spdp-interval",
            "150s",
        ]
    )
    assert args.size_pattern == "1x20KB+1x0KB"
    assert args.rate_hz == 10.0
    assert args.cyclone_spdp_interval == "150s"

    # Capacity.
    args = parser.parse_args(
        [
            "benchmark",
            "capacity",
            "--profile",
            "p",
            "--knob",
            "size",
            "--low",
            "1000",
            "--high",
            "10000",
            "--max-loss",
            "5",
            "--max-latency-ms",
            "200",
        ]
    )
    assert args.benchmark_command == "capacity"
    assert args.knob == "size"
    assert args.rmw == DEFAULT_BENCHMARK_RMW
    assert args.drain_s == DEFAULT_BENCHMARK_DRAIN_S
    assert args.ota_benchmark is False
    assert args.sudo_mode == "passwordless"
    assert _is_ota_benchmark(args) is False

    args = parser.parse_args(
        [
            "benchmark",
            "capacity",
            "--profile",
            "p",
            "--knob",
            "size",
            "--low",
            "1000",
            "--high",
            "10000",
            "--max-loss",
            "5",
            "--max-latency-ms",
            "200",
            "--rmw",
            "fastdds",
        ]
    )
    assert args.rmw == "fastdds"

    args = parser.parse_args(
        [
            "benchmark",
            "capacity",
            "--profile",
            "p",
            "--knob",
            "size",
            "--low",
            "1000",
            "--high",
            "10000",
            "--max-loss",
            "5",
            "--max-latency-ms",
            "200",
            "--target",
            "remote_assist",
            "--target-type",
            "scenario",
            "--workdir",
            "/tmp/bench",
            "--reuse",
            "--interactive",
            "--no-attach",
        ]
    )
    assert args.target == "remote_assist"
    assert args.target_type == "scenario"
    assert args.workdir == "/tmp/bench"
    assert args.reuse is True
    assert args.interactive is True
    assert args.no_attach is True
    assert args.ota_benchmark is False

    # Ramp.
    args = parser.parse_args(["benchmark", "ramp", "--profile", "p", "--values", "1000,2000"])
    assert args.benchmark_command == "ramp"

    # Recovery.
    args = parser.parse_args(["benchmark", "recovery", "--profile", "timeline-p"])
    assert args.benchmark_command == "recovery"

    # Sweep.
    args = parser.parse_args(["benchmark", "sweep", "--profile-grid", "p1,p2"])
    assert args.benchmark_command == "sweep"

    # Sensitivity.
    args = parser.parse_args(["benchmark", "sensitivity", "--profile", "cellular-4g-degraded"])
    assert args.benchmark_command == "sensitivity"
    assert args.size == 1
    assert args.loss_values == "0,0.5,1,3,5"
    assert args.axes == "all"

    # Matrix.
    args = parser.parse_args(["benchmark", "matrix", "--profile", "cellular-4g-degraded"])
    assert args.benchmark_command == "matrix"
    assert args.jitter_ms == 30.0
    assert args.latency_values == "30,60,100,140,180,220"
    assert args.rate_hz_values == "20,15,10,5,1"
    assert args.qos_cases == "best_effort:1,reliable:1,best_effort:10,reliable:10"
    assert args.min_duration == 20.0
    assert args.min_messages == 100

    # Requirements.
    args = parser.parse_args(["benchmark", "requirements"])
    assert args.benchmark_command == "requirements"
    assert args.rate_hz == 20.0
    assert args.size == 18_000
    assert args.max_loss == 5.0
    assert args.max_latency_ms == 250.0
    assert args.axes == "all"
    assert args.bandwidth_high == "auto"
    assert args.bandwidth_high_factor == 8.0
    assert args.bandwidth_low_factor == 1.0
    assert args.latency_base_ms == 0.0
    assert args.latency_high_ms is None
    assert args.search_iterations == 6
    assert args.search_rounds == 1
    assert args.final_refine_iterations == 3
    assert args.loss_coupling == "jitter"
    assert args.downlink_mode == "mirror"
    assert args.probe_repeats == 1
    assert args.probe_min_passes is None
    assert args.bad_lossy_count is None
    assert args.bandwidth_probe_repeats is None
    assert args.search_order == "auto"
    assert args.netem_seed is None
    assert args.jitter_guard_ratio == 0.0
    assert args.bandwidth_guard_ratio == 0.0

    # Loss boundaries.
    args = parser.parse_args(["benchmark", "loss-boundaries"])
    assert args.benchmark_command == "loss-boundaries"
    assert args.max_latency_ms == 250.0
    assert args.rate_hz == 20.0
    assert args.size == 18_000
    assert args.axes == "all"
    assert args.bandwidth_high == "auto"
    assert args.bandwidth_step == "0.1mbit"
    assert args.latency_base_ms == 30.0
    assert args.jitter_high_ms == 40.0
    assert args.jitter_step_ms == 1.0
    assert args.downlink_mode == "lan"
    assert args.probe_repeats == 10
    assert args.netem_seed is None
    assert args.netem_seeds is None

    # Plot.
    args = parser.parse_args(["benchmark", "plot", "results.jsonl"])
    assert args.benchmark_command == "plot"
    assert args.input == "results.jsonl"

    args = parser.parse_args(
        [
            "ota-benchmark",
            "capacity",
            "--profile",
            "cellular-4g-degraded",
            "--knob",
            "size",
            "--low",
            "1",
            "--high",
            "1",
            "--max-loss",
            "30",
            "--max-latency-ms",
            "1000",
            "--duration",
            "10",
            "--repeats",
            "1",
            "--peer",
            "a=vehicle",
            "--peer",
            "b=center",
            "--sudo-mode",
            "askpass",
        ]
    )
    assert args.command == "ota-benchmark"
    assert args.benchmark_command == "capacity"
    assert args.ota_benchmark is True
    assert args.target is None
    assert args.target_type is None
    assert args.peer == ["a=vehicle", "b=center"]
    assert args.sudo_mode == "askpass"
    assert _is_ota_benchmark(args) is True

    args = parser.parse_args(
        [
            "ota-benchmark",
            "row",
            "private-s3-full-tight-cyclone",
            "--registry",
            "fzi_projects/remote_assist/benchmark-rows.yaml",
            "--budgets",
            "fzi_projects/remote_assist/budgets.jsonl",
            "--monitor",
            "--peer",
            "a=vehicle",
            "--peer",
            "b=center",
        ]
    )
    assert args.command == "ota-benchmark"
    assert args.benchmark_command == "row"
    assert args.row_id == "private-s3-full-tight-cyclone"
    assert args.ota_benchmark is True
    assert args.registry == "fzi_projects/remote_assist/benchmark-rows.yaml"
    assert args.budgets == "fzi_projects/remote_assist/budgets.jsonl"
    assert args.monitor is True
    assert args.peer == ["a=vehicle", "b=center"]
    assert _is_ota_benchmark(args) is True


def test_main_routes_ota_benchmark_as_top_level_command(monkeypatch: pytest.MonkeyPatch) -> None:
    seen: dict[str, argparse.Namespace] = {}

    def fake_capacity(args: argparse.Namespace) -> int:
        seen["args"] = args
        return 0

    monkeypatch.setattr(benchmark_cli, "benchmark_capacity", fake_capacity)

    rc = cli.main(
        [
            "ota-benchmark",
            "capacity",
            "--profile",
            "cellular-4g-degraded",
            "--knob",
            "size",
            "--low",
            "1",
            "--high",
            "1",
            "--max-loss",
            "30",
            "--max-latency-ms",
            "1000",
            "--duration",
            "10",
            "--repeats",
            "1",
            "--peer",
            "a=vehicle",
            "--peer",
            "b=center",
        ]
    )

    assert rc == 0
    assert seen["args"].command == "ota-benchmark"
    assert seen["args"].benchmark_command == "capacity"
    assert seen["args"].ota_benchmark is True
    assert seen["args"].peer == ["a=vehicle", "b=center"]


def test_runtime_config_parses_benchmarks_dir_and_profiles(tmp_path: Path) -> None:
    import yaml

    from rosotacom.cli import _load_runtime_config

    config_file = tmp_path / "rosotacom.yaml"
    ros2docker = tmp_path / "ros2docker.json"
    ros2docker.write_text("{}", encoding="utf-8")

    config_data = {
        "ros2docker_config": str(ros2docker),
        "session_configs_dir": [],
        "scenario_configs_dir": [],
        "session_instances_dir": "session-instances",
        "profiles": "profiles.yaml",
        "benchmarks_dir": "benchmarks",
    }
    config_file.write_text(yaml.safe_dump(config_data), encoding="utf-8")

    # Write a dummy profiles file
    profiles_file = tmp_path / "profiles.yaml"
    profiles_file.write_text("profiles: {}", encoding="utf-8")

    import argparse

    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
    )
    runtime = _load_runtime_config(args)

    assert runtime.profiles_file == profiles_file
    assert runtime.benchmarks_dir == tmp_path / "benchmarks"


def test_benchmark_profiles_file_falls_back_to_cwd(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    profiles_file = tmp_path / "profiles" / "benchmark-profiles.yaml"
    profiles_file.parent.mkdir()
    profiles_file.write_text("profiles: {}\n", encoding="utf-8")
    monkeypatch.chdir(tmp_path)

    assert _benchmark_profiles_file("cellular-4g-degraded", None) == profiles_file


def test_benchmark_session_copy_pins_requested_rmw(tmp_path: Path) -> None:
    import argparse

    import yaml

    project = tmp_path / "project"
    session_dir = project / "sessions" / "bench_1_1_capacity"
    session_dir.mkdir(parents=True)
    (session_dir / "session-definition.yaml").write_text(
        "peers:\n"
        "  a: {}\n"
        "  b: {}\n"
        "shared:\n"
        "  rmw: fastdds\n"
        "  qos:\n"
        "    defaults:\n"
        "      depth: 1\n"
        "    for_role:\n"
        "      ota_sub: { reliability: best_effort }\n"
        "      ota_pub: { reliability: best_effort }\n",
        encoding="utf-8",
    )
    ros2docker = project / "ros2docker.json"
    ros2docker.write_text("{}", encoding="utf-8")
    config_file = project / "rosotacom.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "ros2docker_config": str(ros2docker),
                "session_configs_dir": ["sessions"],
                "session_instances_dir": "session-instances",
            }
        ),
        encoding="utf-8",
    )
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
        artifacts_dir=None,
        rmw="cyclone",
        qos_reliability="reliable",
        qos_depth=10,
    )

    context = _prepare_benchmark_session_config(args, "bench_1_1_capacity", run_dir)

    copied_config = Path(context["config_path"])
    copied = yaml.safe_load(copied_config.read_text(encoding="utf-8"))
    assert copied["shared"]["rmw"] == "cyclone"
    assert copied["shared"]["qos"]["defaults"]["depth"] == 10
    assert copied["shared"]["qos"]["for_role"]["ota_sub"]["reliability"] == "reliable"
    assert copied["shared"]["qos"]["for_role"]["ota_pub"]["reliability"] == "reliable"
    assert args.session_configs_dir[0] == str(run_dir / "session-configs")
    assert context["runtime_implementation"] == "rmw_cyclonedds_cpp"
    assert context["cyclone_spdp_interval"] is None
    assert context["qos"] == {"reliability": "reliable", "depth": 10}


def test_benchmark_session_copy_applies_cyclone_spdp_override(tmp_path: Path) -> None:
    import argparse

    project = tmp_path / "project"
    session_dir = project / "sessions" / "bench_1_1_capacity"
    session_dir.mkdir(parents=True)
    (session_dir / "session-definition.yaml").write_text(
        "peers:\n  a: {}\n  b: {}\nshared:\n  rmw: cyclone\n",
        encoding="utf-8",
    )
    ros2docker = project / "ros2docker.json"
    ros2docker.write_text("{}", encoding="utf-8")
    config_file = project / "rosotacom.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "ros2docker_config": str(ros2docker),
                "session_configs_dir": ["sessions"],
                "session_instances_dir": "session-instances",
            }
        ),
        encoding="utf-8",
    )
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
        artifacts_dir=None,
        rmw="cyclone",
        qos_reliability=None,
        qos_depth=None,
        cyclone_spdp_interval="150s",
    )

    context = _prepare_benchmark_session_config(args, "bench_1_1_capacity", run_dir)

    copied = yaml.safe_load(Path(context["config_path"]).read_text(encoding="utf-8"))
    assert copied["shared"]["rmw"] == {
        "local": "cyclone",
        "ota": {"cyclone": {"spdp_interval": "150s"}},
    }
    assert context["cyclone_spdp_interval"] == "150s"


def test_benchmark_session_copy_rejects_spdp_override_for_non_cyclone(tmp_path: Path) -> None:
    import argparse

    project = tmp_path / "project"
    session_dir = project / "sessions" / "bench_1_1_capacity"
    session_dir.mkdir(parents=True)
    (session_dir / "session-definition.yaml").write_text(
        "peers:\n  a: {}\n  b: {}\nshared:\n  rmw: fastdds\n",
        encoding="utf-8",
    )
    (project / "ros2docker.json").write_text("{}", encoding="utf-8")
    config_file = project / "rosotacom.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "ros2docker_config": "ros2docker.json",
                "session_configs_dir": ["sessions"],
                "session_instances_dir": "session-instances",
            }
        ),
        encoding="utf-8",
    )
    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
        artifacts_dir=None,
        rmw="fastdds",
        cyclone_spdp_interval="150s",
    )
    run_dir = tmp_path / "run"
    run_dir.mkdir()

    with pytest.raises(ValueError, match="--rmw cyclone"):
        _prepare_benchmark_session_config(args, "bench_1_1_capacity", run_dir)


def test_probe_spdp_diagnostics_warn_on_tight_cyclone_profile(tmp_path: Path) -> None:
    import argparse

    project = tmp_path / "project"
    project.mkdir()
    (project / "ros2docker.json").write_text("{}", encoding="utf-8")
    (project / "profiles.yaml").write_text(
        "profiles:\n  tight:\n    uplink: { rate: 3.2mbit }\n",
        encoding="utf-8",
    )
    config_file = project / "rosotacom.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "ros2docker_config": "ros2docker.json",
                "profiles": "profiles.yaml",
                "session_instances_dir": "session-instances",
            }
        ),
        encoding="utf-8",
    )
    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
        artifacts_dir=None,
        rmw="cyclone",
        cyclone_spdp_interval=None,
    )
    load_info = benchmark_cli._load_context(_probe_load(size=18_000, size_pattern=None, rate_hz=20.0, streams=1))

    diagnostic = _probe_spdp_diagnostics(args=args, profile="tight", duration_s=25.0, load_info=load_info)

    assert diagnostic is not None
    assert diagnostic["interval"] == "30s"
    assert diagnostic["risk"] == "possible"
    assert diagnostic["offered_to_minimum_profile_rate"] == pytest.approx(0.9)
    assert "SPDPInterval=30s" in diagnostic["warnings"][0]


def test_benchmark_session_copy_expands_capacity_stream_topics(tmp_path: Path) -> None:
    import argparse

    import yaml

    project = tmp_path / "project"
    session_dir = project / "sessions" / "bench_1_1_capacity"
    session_dir.mkdir(parents=True)
    (session_dir / "session-definition.yaml").write_text(
        "peers:\n"
        "  a: {}\n"
        "  b: {}\n"
        "shared:\n"
        "  rmw: cyclone\n"
        "topics:\n"
        "  a_to_b:\n"
        "    - topic: /bench_capacity\n"
        "      type: com_msgs/msg/SizedPayload\n"
        "      processing:\n"
        "        use_ota_wrapper: true\n",
        encoding="utf-8",
    )
    ros2docker = project / "ros2docker.json"
    ros2docker.write_text("{}", encoding="utf-8")
    config_file = project / "rosotacom.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "ros2docker_config": str(ros2docker),
                "session_configs_dir": ["sessions"],
                "session_instances_dir": "session-instances",
            }
        ),
        encoding="utf-8",
    )
    run_dir = tmp_path / "run"
    run_dir.mkdir()
    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
        artifacts_dir=None,
        rmw="cyclone",
        qos_reliability=None,
        qos_depth=None,
        streams=2,
    )

    context = _prepare_benchmark_session_config(args, "bench_1_1_capacity", run_dir)

    copied = yaml.safe_load(Path(context["config_path"]).read_text(encoding="utf-8"))
    topics = copied["topics"]["a_to_b"]
    assert [topic["topic"] for topic in topics] == ["/bench_capacity_0", "/bench_capacity_1"]
    assert all(topic["type"] == "com_msgs/msg/SizedPayload" for topic in topics)
    assert all(topic["processing"] == {"use_ota_wrapper": True} for topic in topics)
    assert context["streams"] == {
        "count": 2,
        "base_topic": "/bench_capacity",
        "topics": ["/bench_capacity_0", "/bench_capacity_1"],
    }


def test_benchmark_ota_target_defaults_to_benchmark_session() -> None:
    args = argparse.Namespace(target=None, target_type=None)
    assert _benchmark_ota_target(args, "bench_1_1_capacity") == ("bench_1_1_capacity", "session")

    args = argparse.Namespace(target="remote_assist", target_type=None)
    assert _benchmark_ota_target(args, "bench_1_1_capacity") == ("remote_assist", "auto")

    args = argparse.Namespace(target="remote_assist", target_type="scenario")
    assert _benchmark_ota_target(args, "bench_1_1_capacity") == ("remote_assist", "scenario")

    args = argparse.Namespace(target=None, target_type="scenario")
    with pytest.raises(ValueError, match="--target-type requires --target"):
        _benchmark_ota_target(args, "bench_1_1_capacity")


def test_peer_catmux_attach_script_waits_for_container_and_tmux() -> None:
    script = _peer_catmux_attach_script("a", "rosotacom_id_com_to_b", Path("/tmp/orchestrator.full.log"))

    assert "waiting for benchmark peer $identity container" in script
    assert "orchestrator.full.log" in script
    assert 'docker exec "$container" tmux -L catmux list-sessions' in script
    assert 'docker exec -it "$container" tmux -L catmux attach-session -t "$session"' in script
    assert "split-window" not in script


def test_ota_profile_shaping_uses_noninteractive_sudo(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, dict[str, object]]] = []
    peer = cli.OtaSmokePeer(name="a", ssh="robot-a", address="10.0.0.10")

    def fake_ota_run(_peer: cli.OtaSmokePeer, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append((script, kwargs))
        return subprocess.CompletedProcess(["ssh"], 0, "", "")

    monkeypatch.setattr(cli, "_ota_run", fake_ota_run)

    cli._peer_command_runner(peer, dry_run=False)(["tc", "qdisc", "show", "dev", "tun0"])
    cli._peer_watchdog_launcher(peer, dry_run=False)(
        ["sh", "-c", "sleep 1; tc qdisc del dev tun0 root; ip link set dev tun0 up"]
    )

    assert calls[0][0] == "sudo -n tc qdisc show dev tun0"
    assert calls[0][1]["label"] == "a: tc/netem"
    assert calls[1][0].startswith("nohup sh -c ")
    assert "sudo -n tc qdisc del dev tun0 root" in calls[1][0]
    assert "sudo -n ip link set dev tun0 up" in calls[1][0]
    assert calls[1][1]["label"] == "a: profile safety watchdog"


def test_ota_profile_shaping_askpass_uses_stdin(monkeypatch: pytest.MonkeyPatch) -> None:
    action_calls: list[tuple[str, str]] = []
    secret_calls: list[tuple[str, dict[str, object]]] = []
    peer = cli.OtaSmokePeer(name="a", ssh="robot-a", address="10.0.0.10")

    def fake_log_action(label: str, detail: str) -> None:
        action_calls.append((label, detail))

    def fake_ota_run_with_secret_stdin(
        _peer: cli.OtaSmokePeer, script: str, **kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        secret_calls.append((script, kwargs))
        return subprocess.CompletedProcess(["ssh"], 0, "", "")

    monkeypatch.setattr(cli, "_ota_log_action", fake_log_action)
    monkeypatch.setattr(cli, "_ota_run_with_secret_stdin", fake_ota_run_with_secret_stdin)

    cli._peer_command_runner(peer, dry_run=False, sudo_password="secret")(["tc", "qdisc", "show", "dev", "tun0"])
    cli._peer_watchdog_launcher(peer, dry_run=False, sudo_password="secret")(
        ["sh", "-c", "sleep 1; tc qdisc del dev tun0 root; ip link set dev tun0 up"]
    )

    assert action_calls[0] == ("a: tc/netem", "running remote sudo command via stdin")
    assert secret_calls[0][0] == "sudo -S -p '' tc qdisc show dev tun0"
    assert secret_calls[0][1]["secret_stdin"] == "secret\n"
    assert "secret" not in secret_calls[0][0]
    assert action_calls[1] == ("a: profile safety watchdog", "arming remote sudo watchdog via stdin")
    assert secret_calls[1][0].startswith("sudo -S -p '' sh -c ")
    assert secret_calls[1][1]["secret_stdin"] == "secret\n"
    assert "secret" not in secret_calls[1][0]


def test_ota_preflight_can_require_network_shaping_sudo(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str]] = []
    plan = cli.OtaSmokePlan(
        state_path=None,
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="project/rosotacom.yaml",
        peers={"a": cli.OtaSmokePeer(name="a", ssh="robot-a", address="10.0.0.10")},
    )

    def fake_ota_run(
        _peer: cli.OtaSmokePeer,
        script: str,
        *,
        label: str,
        **_kwargs: object,
    ) -> subprocess.CompletedProcess[str]:
        calls.append((label, script))
        return subprocess.CompletedProcess(["ssh"], 0, "", "")

    monkeypatch.setattr(cli, "_ota_run", fake_ota_run)

    cli._ota_preflight(
        plan,
        require_tmux=False,
        check_peer_reachability=False,
        dry_run=False,
        require_network_shaping_sudo=True,
    )

    assert (
        "a: required commands for network shaping",
        "command -v tc >/dev/null 2>&1 && command -v ip >/dev/null 2>&1",
    ) in calls
    assert (
        "a: passwordless sudo for network shaping",
        "sudo -n tc qdisc show >/dev/null && sudo -n ip link show >/dev/null",
    ) in calls


def test_ota_preflight_askpass_authenticates_via_stdin(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[tuple[str, str]] = []
    action_calls: list[tuple[str, str]] = []
    secret_calls: list[tuple[str, str, str]] = []
    plan = cli.OtaSmokePlan(
        state_path=None,
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="project/rosotacom.yaml",
        peers={"a": cli.OtaSmokePeer(name="a", ssh="robot-a", address="10.0.0.10")},
    )

    def fake_ota_run(
        _peer: cli.OtaSmokePeer,
        script: str,
        *,
        label: str,
        **_kwargs: object,
    ) -> subprocess.CompletedProcess[str]:
        calls.append((label, script))
        return subprocess.CompletedProcess(["ssh"], 0, "", "")

    def fake_log_action(label: str, detail: str) -> None:
        action_calls.append((label, detail))

    def fake_ota_run_with_secret_stdin(
        _peer: cli.OtaSmokePeer,
        script: str,
        *,
        label: str,
        secret_stdin: str,
        **_kwargs: object,
    ) -> subprocess.CompletedProcess[str]:
        secret_calls.append((label, script, secret_stdin))
        return subprocess.CompletedProcess(["ssh"], 0, "", "")

    monkeypatch.setattr(cli, "_ota_run", fake_ota_run)
    monkeypatch.setattr(cli, "_ota_log_action", fake_log_action)
    monkeypatch.setattr(cli, "_ota_run_with_secret_stdin", fake_ota_run_with_secret_stdin)

    cli._ota_preflight(
        plan,
        require_tmux=False,
        check_peer_reachability=False,
        dry_run=False,
        require_network_shaping_sudo=True,
        sudo_mode="askpass",
        sudo_passwords={"a": "secret"},
    )

    assert (
        "a: sudo authentication for network shaping",
        "authenticating sudo via stdin",
    ) in action_calls
    assert ("a: sudo authentication for network shaping", "sudo -S -p '' true", "secret\n") in secret_calls
    assert all("secret" not in script for _label, script in calls)
    assert all("secret" not in detail for _label, detail in action_calls)
    assert all("secret" not in script for _label, script, _stdin in secret_calls)


def test_ota_askpass_prompts_once_per_peer(monkeypatch: pytest.MonkeyPatch) -> None:
    prompts: list[str] = []
    plan = cli.OtaSmokePlan(
        state_path=None,
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="project/rosotacom.yaml",
        peers={
            "a": cli.OtaSmokePeer(name="a", ssh="robot-a", address="10.0.0.10"),
            "b": cli.OtaSmokePeer(name="b", ssh="robot-b", address="10.0.0.11"),
        },
    )

    def fake_getpass(prompt: str) -> str:
        prompts.append(prompt)
        return f"pw-{len(prompts)}"

    monkeypatch.setattr(cli.getpass, "getpass", fake_getpass)

    passwords = cli._ota_network_sudo_passwords(
        plan,
        sudo_mode="askpass",
        require_network_shaping_sudo=True,
        dry_run=False,
    )

    assert passwords == {"a": "pw-1", "b": "pw-2"}
    assert prompts == [
        "sudo password for a (robot-a) [network shaping only]: ",
        "sudo password for b (robot-b) [network shaping only]: ",
    ]

    dry_run_passwords = cli._ota_network_sudo_passwords(
        plan,
        sudo_mode="askpass",
        require_network_shaping_sudo=True,
        dry_run=True,
    )
    assert dry_run_passwords == {"a": "", "b": ""}
    assert len(prompts) == 2


def test_interactive_result_log_is_reset_between_runs(tmp_path: Path) -> None:
    full_log = tmp_path / "interactive" / "benchmark-capacity-p" / "orchestrator.full.log"
    full_log.parent.mkdir(parents=True)
    full_log.write_text("Benchmark result saved to /old/result.json\n", encoding="utf-8")

    _initialize_interactive_log(full_log, ["rosotacom", "benchmark", "capacity"])

    text = full_log.read_text(encoding="utf-8")
    assert "Benchmark result saved" not in text
    assert "benchmark operator log initialized" in text
    assert "command: rosotacom benchmark capacity" in text

    script = _result_once_script(full_log)
    assert "result printed once; pane stays alive" in script
    assert "sys.exit" not in script


def test_interactive_benchmark_dry_run_prints_operator_view(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    import yaml

    ros2docker = tmp_path / "ros2docker.json"
    ros2docker.write_text("{}", encoding="utf-8")
    config_file = tmp_path / "rosotacom.yaml"
    config_file.write_text(
        yaml.safe_dump(
            {
                "ros2docker_config": str(ros2docker),
                "session_configs_dir": [
                    str(Path(benchmark_cli.__file__).resolve().parent / "resources" / "examples" / "sessions")
                ],
                "scenario_configs_dir": [],
                "session_instances_dir": "session-instances",
                "benchmarks_dir": "benchmarks",
            }
        ),
        encoding="utf-8",
    )
    monkeypatch.setattr(
        benchmark_cli.sys,
        "argv",
        [
            "rosotacom",
            "benchmark",
            "capacity",
            "--profile",
            "p",
            "--knob",
            "size",
            "--low",
            "1",
            "--high",
            "10",
            "--max-loss",
            "5",
            "--max-latency-ms",
            "200",
            "--interactive",
            "--no-attach",
        ],
    )
    args = argparse.Namespace(
        rosotacom_config=str(config_file),
        ros2docker_config=None,
        session_configs_dir=None,
        scenario_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        profiles_file=None,
        artifacts_dir=None,
        dry_run=True,
        no_attach=True,
        profile="p",
    )

    assert _start_interactive_benchmark(args, "capacity") == 0

    output = capsys.readouterr().out
    assert "Would create benchmark tmux session: benchmark-capacity-p" in output
    assert "Run window: high-level orchestrator" in output
    assert "Peer window a: a_catmux fullscreen catmux attach" in output
    assert "Peer window b: b_catmux fullscreen catmux attach" in output
    assert "Network window: qdisc monitor + tc command log" in output
    assert "Results window: final result printed once" in output
    assert "launch log" not in output
    child = " ".join(_benchmark_child_command())
    assert "--interactive" not in child
    assert "--no-attach" not in child


def test_tee_stream_and_stdout_redirection(tmp_path: Path) -> None:
    import subprocess
    import sys

    from rosotacom.cli_benchmark import log_stdout_stderr_to_file

    log_file = tmp_path / "test_log.txt"

    with log_stdout_stderr_to_file(log_file):
        print("Hello from test stdout")
        print("Hello from test stderr", file=sys.stderr)

        # Test hooked subprocess.run
        res = subprocess.run(
            [
                sys.executable,
                "-c",
                "import sys; sys.stdout.write('sub stdout\\n'); sys.stderr.write('sub stderr\\n')",
            ],
            capture_output=False,
        )
        assert res.returncode == 0

    log_content = log_file.read_text(encoding="utf-8")
    assert "Hello from test stdout" in log_content
    assert "Hello from test stderr" in log_content
    assert "sub stdout" in log_content
    assert "sub stderr" in log_content


def test_probe_driver_prints_latency_and_arrival_spacing_details(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """The fixed probe prints the full latency distribution, the first/last-third
    p50 trend, and the arrival-spacing bunching stats (benchmark commands reference characterization)."""
    from rosotacom.transit import summarize_transit_records

    # 12 messages at 100 ms cadence; latency ramps 10 -> 65 ms; one stalled
    # arrival gap (300 ms) immediately followed by a bunched one (5 ms).
    gaps = [None, 100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 5.0, 100.0, 100.0, 100.0, 100.0]
    records = [
        {
            "kind": "transit",
            "source": "a",
            "target": "b",
            "topic": "/test",
            "seq": seq,
            "status": "delivered",
            "t_wrap": 10.0 + 0.1 * seq,
            "sections": {"ota_hop_ms": 10.0 + 5.0 * seq},
            "jitter_ms": 2.0,
            "inter_arrival_ms": gaps[seq],
            "size_bytes": 100,
        }
        for seq in range(12)
    ]

    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        events_dir = out_dir / "logs" / "b" / "status"
        events_dir.mkdir(parents=True)
        (events_dir / "events.jsonl").write_text(
            "\n".join(json.dumps(record) for record in records) + "\n", encoding="utf-8"
        )
        return summarize_transit_records(records)

    result = drive_probe(
        stub,
        profile="lossless-typical",
        size=100,
        rate_hz=10.0,
        repeats=1,
        duration_s=1.2,
        bin_s=1.0,
        render_plot=False,
        out_dir=tmp_path,
    )

    out = capsys.readouterr().out
    assert "p50=35.0ms p95=65.0ms" in out
    assert "latency_ms: min=10.0 p50=35.0 mean=37.5 p95=65.0 p99=65.0 max=65.0 std=" in out
    assert "latency_p50_trend_ms: first_third=15.0 last_third=55.0 delta=+40.0" in out
    assert "inter_arrival_ms: min=5.0 p05=5.0 p50=100.0" in out
    assert (
        "arrival_spacing vs nominal=100.0ms: bunched(<50.0ms)=9.091% stalled(>150.0ms)=9.091% stall_then_bunch=100.0%"
        in out
    )
    row = result["attempts"][0]["topics"][0]
    assert row["latency_trend_ms"]["delta"] == 40.0
    assert row["inter_arrival_ms"]["stall_then_bunch_pct"] == 100.0


# --------------------------------------------------------------------------- #
# A/B tuning experiments (#22): candidate configs vs baseline, same load
# --------------------------------------------------------------------------- #


def _ab_topic_summary(*, delivered: int, expected: int = 100, latency_p95: float = 100.0) -> dict[str, Any]:
    lost = expected - delivered
    return {
        "topics": {
            "a->b:/bench_capacity": {
                "expected": expected,
                "delivered": delivered,
                "lost": lost,
                "loss_pct": round(100.0 * lost / expected, 3),
                "ota_hop_ms": {"p50": latency_p95 * 0.6, "p95": latency_p95},
                "jitter_ms": {"p50": 2.0, "p95": 5.0},
            }
        }
    }


def _ab_configs(base: Path, labels: list[str]) -> list[dict[str, Any]]:
    configs: list[dict[str, Any]] = []
    for index, label in enumerate(labels):
        root = base / "configs" / label
        (root / benchmark_cli.AB_SESSION_NAME).mkdir(parents=True)
        configs.append(
            {"label": label, "is_baseline": index == 0, "source": base / f"{label}.yaml", "configs_root": root}
        )
    return configs


def _ab_tols(metrics: list[str]) -> dict[str, AbTolerance]:
    return {metric: default_ab_tolerance(metric) for metric in metrics}


def test_drive_ab_interleaves_aggregates_and_writes_result(tmp_path: Path) -> None:
    canned = {
        "baseline": _ab_topic_summary(delivered=70, latency_p95=150.0),
        "fast": _ab_topic_summary(delivered=98, latency_p95=95.0),
        "worse": _ab_topic_summary(delivered=40, latency_p95=250.0),
    }
    order: list[str] = []

    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path, config: dict[str, Any]):
        order.append(str(config["label"]))
        return canned[str(config["label"])]

    metrics = ["completeness_pct", "loss_pct", "latency_p95_ms"]
    report = drive_ab(
        stub,
        configs=_ab_configs(tmp_path, ["baseline", "fast", "worse"]),
        baseline_label="baseline",
        profile="ab-ci",
        load={"size_a": 18_000, "rate": 20},
        duration_s=1.0,
        repeats=2,
        metrics=metrics,
        tolerances=_ab_tols(metrics),
        topic="",
        out_dir=tmp_path,
    )

    # Interleaved with a rotating start (ab_schedule), one run per (config, repeat).
    assert order == ["baseline", "fast", "worse", "fast", "worse", "baseline"]

    result = json.loads((tmp_path / BENCHMARK_RESULT_FILE).read_text())
    assert result["genre"] == "ab"
    assert result["verdict"]["passed"] is False  # "worse" regresses; "fast" does not
    assert "worse" in result["verdict"]["regressed"]
    assert "fast" not in result["verdict"]["regressed"]
    assert [(s["order"], s["config"]) for s in result["configuration"]["schedule"][:3]] == [
        (1, "baseline"),
        (2, "fast"),
        (3, "worse"),
    ]
    rows = [json.loads(line) for line in (tmp_path / "ab.jsonl").read_text().splitlines()]
    assert len(rows) == 6
    assert (tmp_path / "ab.md").read_text().count("vs `baseline`") == 2
    assert report["passed"] is False


def test_drive_ab_verdict_is_deterministic(tmp_path: Path) -> None:
    def stub(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path, config: dict[str, Any]):
        return _ab_topic_summary(delivered=80 if config["is_baseline"] else 96)

    def once(name: str) -> dict[str, Any]:
        out = tmp_path / name
        out.mkdir()
        metrics = ["completeness_pct", "loss_pct"]
        return drive_ab(
            stub,
            configs=_ab_configs(out, ["baseline", "cand"]),
            baseline_label="baseline",
            profile="p",
            load={"size_a": 1, "rate": 20},
            duration_s=1.0,
            repeats=3,
            metrics=metrics,
            tolerances=_ab_tols(metrics),
            topic="",
            out_dir=out,
        )

    assert once("run_a") == once("run_b")


def _write_ab_config(path: Path, throttle_hz: int) -> Path:
    path.mkdir(parents=True, exist_ok=True)
    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 50}, "b": {"domain_id": 51}},
        "shared": {"ota_domain_id": 52, "rmw": "cyclone", "use_status_overview": True},
        "topics": {
            "a_to_b": [
                {
                    "topic": "/bench_capacity",
                    "type": "com_msgs/msg/SizedPayload",
                    "processing": {"use_ota_wrapper": True, "throttle_hz": throttle_hz},
                }
            ]
        },
    }
    (path / "session-definition.yaml").write_text(yaml.safe_dump(cfg), encoding="utf-8")
    return path


def _ab_handler_args(tmp_path: Path, *, baseline: Path, candidate: Path, **overrides: Any) -> argparse.Namespace:
    project = tmp_path / "proj"
    project.mkdir(parents=True, exist_ok=True)
    (project / "rosotacom.yaml").write_text("profiles: profiles.yaml\n", encoding="utf-8")
    (project / "profiles.yaml").write_text(
        "profiles:\n  ab-ci:\n    uplink: { rate: 1mbit }\n    downlink: { rate: 10mbit }\n", encoding="utf-8"
    )
    args: dict[str, Any] = {
        "rosotacom_config": str(project / "rosotacom.yaml"),
        "ros2docker_config": None,
        "deployment": None,
        "session_configs_dir": None,
        "scenario_configs_dir": None,
        "session_instances_dir": None,
        "profiles_file": str(project / "profiles.yaml"),
        "artifacts_dir": str(tmp_path / "artifacts"),
        "profile": "ab-ci",
        "baseline": str(baseline),
        "candidate": [f"unthrottled={candidate}"],
        "repeats": 2,
        "size": 18_000,
        "size_pattern": None,
        "rate_hz": 20.0,
        "streams": 1,
        "interval_jitter_ms": 0.0,
        "interval_jitter_seed": 42,
        "duration": 1.0,
        "topic": "",
        "metrics": None,
        "rel_tolerance": None,
        "abs_tolerance": None,
        "fail_on_regression": False,
        "out": "ab.jsonl",
        "rmw": "cyclone",
        "qos_reliability": None,
        "qos_depth": None,
        "cyclone_spdp_interval": None,
    }
    args.update(overrides)
    namespace = argparse.Namespace(**args)

    # The stub reads throttle_hz from the config the wrapper just activated via
    # session_configs_dir — so the test exercises the real materialize+swap path.
    def run_point(*, profile: str | None, load: dict[str, Any], duration_s: float, out_dir: Path) -> dict[str, Any]:
        root = Path(namespace.session_configs_dir[0])
        cfg = yaml.safe_load((root / benchmark_cli.AB_SESSION_NAME / "session-definition.yaml").read_text())
        throttle = cfg["topics"]["a_to_b"][0]["processing"]["throttle_hz"]
        # Heavier throttle => more offered load => more loss under the tight profile.
        return _ab_topic_summary(delivered=96 if throttle <= 5 else 55)

    namespace._test_run_point = run_point
    return namespace


def test_benchmark_ab_handler_materializes_configs_and_gates_exit_code(tmp_path: Path) -> None:
    baseline = _write_ab_config(tmp_path / "base", throttle_hz=5)
    candidate = _write_ab_config(tmp_path / "cand", throttle_hz=20)

    # Default: verdict lives in result.json, exit 0 even on a regression.
    args = _ab_handler_args(tmp_path, baseline=baseline, candidate=candidate)
    assert benchmark_ab(args) == 0

    run_dir = next((tmp_path / "artifacts").rglob("result.json")).parent
    result = json.loads((run_dir / BENCHMARK_RESULT_FILE).read_text())
    assert result["genre"] == "ab"
    assert result["verdict"]["passed"] is False
    assert "unthrottled" in result["verdict"]["regressed"]
    # The knob difference is captured in the self-describing diff.
    assert "throttle_hz" in (run_dir / "configs" / "unthrottled.diff").read_text()

    # --fail-on-regression makes the process exit reflect the verdict.
    args2 = _ab_handler_args(tmp_path / "again", baseline=baseline, candidate=candidate, fail_on_regression=True)
    assert benchmark_ab(args2) == 1


def test_prepare_ab_config_requires_an_a_to_b_load_topic(tmp_path: Path) -> None:
    source = tmp_path / "no_load"
    source.mkdir()
    (source / "session-definition.yaml").write_text(
        yaml.safe_dump(
            {"peers": {"a": {}, "b": {}}, "topics": {"b_to_a": [{"topic": "/x", "type": "std_msgs/msg/String"}]}}
        ),
        encoding="utf-8",
    )
    args = argparse.Namespace(rmw="cyclone", cyclone_spdp_interval=None, qos_reliability=None, qos_depth=None)
    with pytest.raises(ValueError, match="a_to_b"):
        benchmark_cli._prepare_ab_config(
            args,
            label="baseline",
            source_yaml=source / "session-definition.yaml",
            is_baseline=True,
            run_dir=tmp_path / "run",
        )


def test_explicit_ota_target_is_recorded_without_preparing_a_genre_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """`--target` means the load is somebody else's session, run from each peer's project.

    Nothing is copied and no `shared.rmw` is rewritten, so a genre must not
    report a prepared session it never used — and must not touch the disk
    preparing one.
    """

    def _fail(*_args: Any, **_kwargs: Any) -> dict[str, Any]:
        raise AssertionError("an explicit OTA target must not prepare a genre session")

    monkeypatch.setattr(benchmark_cli, "_prepare_benchmark_session_config", _fail)
    args = argparse.Namespace(
        ota_benchmark=True,
        target="mwc_fastdds_12k",
        target_type="session",
    )

    context = benchmark_cli._benchmark_session_context(args, "bench_1_1_capacity", tmp_path)

    assert context["prepared"] is False
    assert context["target_override"] == {"target": "mwc_fastdds_12k", "target_type": "session"}


def test_without_an_explicit_target_a_genre_still_prepares_its_own_session(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    prepared: list[str] = []

    def _prepare(_args: Any, session_name: str, _run_dir: Path) -> dict[str, Any]:
        prepared.append(session_name)
        return {"name": session_name, "prepared": True}

    monkeypatch.setattr(benchmark_cli, "_prepare_benchmark_session_config", _prepare)
    args = argparse.Namespace(ota_benchmark=True, target=None, target_type=None)

    context = benchmark_cli._benchmark_session_context(args, "bench_1_1_capacity", tmp_path)

    assert prepared == ["bench_1_1_capacity"]
    assert context["prepared"] is True


def test_a_target_driven_probe_records_the_target_instead_of_unused_load_parameters() -> None:
    """`--size` / `--rate-hz` describe nothing when the target brings its own publishers.

    Recording them anyway is worse than recording nothing: a figure would carry
    an offered bandwidth the link never saw.
    """
    args = argparse.Namespace(
        ota_benchmark=True,
        target="mwc_fastdds_12k",
        target_type="session",
        size=18_000,
        rate_hz=20.0,
    )

    context = benchmark_cli._load_context(benchmark_cli._benchmark_probe_load(args))

    assert context["driven_by"] == {"target": "mwc_fastdds_12k", "target_type": "session"}
    assert context["offered_bandwidth_bps"] is None
    assert context["parameters"] is None


def test_a_synthetic_probe_still_records_its_offered_bandwidth() -> None:
    args = argparse.Namespace(ota_benchmark=False, target=None, size=12_000, rate_hz=10.0, streams=1)

    context = benchmark_cli._load_context(benchmark_cli._benchmark_probe_load(args))

    assert context["mean_payload_bytes"] == 12_000
    assert context["offered_bandwidth_bps"] == 12_000 * 8 * 10
