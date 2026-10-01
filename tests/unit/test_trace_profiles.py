# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import json
from pathlib import Path

import yaml

from rosotacom.cli import main
from rosotacom.network_profiles import (
    OUTAGE_CATCHUP,
    OUTAGE_RECONNECT,
    expand_timeline,
    load_profiles_file,
    parse_rate_bps,
)
from rosotacom.network_shaper import ProfileShaper
from rosotacom.trace_profiles import TraceProfileConfig, convert_trace_to_profile_yaml


def _write_trace(path: Path, rows: list[dict[str, object]]) -> Path:
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
    return path


def _row(
    time_s: float,
    *,
    rtt_ms: float,
    loss_pct: float,
    tx_kbps: float = 0.0,
    rx_kbps: float = 0.0,
    tx_marked: bool = False,
    rx_marked: bool = False,
) -> dict[str, object]:
    tx: dict[str, object] = {"bytes_delta": 0, "packets_delta": 0, "observed_kbps": tx_kbps}
    rx: dict[str, object] = {"bytes_delta": 0, "packets_delta": 0, "observed_kbps": rx_kbps}
    if tx_marked:
        tx["saturated"] = True
    if rx_marked:
        rx["capacity_probe"] = True
    return {
        "schema_version": 1,
        "kind": "link_trace",
        "monotonic_s": time_s,
        "peer": "a",
        "remote": "b",
        "passive_counter_delta": {
            "available": True,
            "window_s": 1.0,
            "observed_not_available_bandwidth": True,
            "tx": tx,
            "rx": rx,
        },
        "peer_probe": {
            "available": True,
            "rtt_ms": rtt_ms,
            "loss_pct": loss_pct,
            "assumption": "symmetric_path",
        },
    }


def test_timeline_recovers_known_segments_and_arms_in_dry_run(tmp_path: Path) -> None:
    trace = _write_trace(
        tmp_path / "link_trace.jsonl",
        [
            *[_row(float(second), rtt_ms=20.0, loss_pct=0.0, tx_kbps=2_000.0, tx_marked=True) for second in range(5)],
            *[
                _row(float(second), rtt_ms=100.0, loss_pct=5.0, tx_kbps=1_000.0, tx_marked=True)
                for second in range(5, 10)
            ],
        ],
    )

    text = convert_trace_to_profile_yaml(
        trace,
        mode="timeline",
        config=TraceProfileConfig(
            name="drive_replay",
            directions=("uplink",),
            min_segment_s=3.0,
            change_sensitivity=0.2,
        ),
    )
    out = tmp_path / "profiles.yaml"
    out.write_text(text, encoding="utf-8")

    doc = yaml.safe_load(text)
    timeline = doc["profiles"]["drive_replay"]["timeline"]
    assert timeline == [
        {"for": "5s", "uplink": {"rate": "2000000bit", "delay": "10ms"}},
        {"for": "5s", "uplink": {"rate": "1000000bit", "delay": "50ms", "loss": "5%"}},
    ]

    profile = load_profiles_file(out)["drive_replay"]
    steps = expand_timeline(profile, "tun0", direction="uplink")
    assert len(steps) == 2
    calls: list[list[str]] = []
    ProfileShaper("tun0", lambda argv: calls.append(list(argv))).arm(steps[0].commands)
    assert ["tc", "qdisc", "replace", "dev", "tun0", "root", "handle", "1:", "tbf"] == calls[2][:9]


def test_timeline_emits_reconnect_outage_for_sample_gap(tmp_path: Path) -> None:
    trace = _write_trace(
        tmp_path / "link_trace.jsonl",
        [
            _row(0.0, rtt_ms=20.0, loss_pct=0.0),
            _row(1.0, rtt_ms=20.0, loss_pct=0.0),
            _row(2.0, rtt_ms=20.0, loss_pct=0.0),
            _row(8.0, rtt_ms=20.0, loss_pct=0.0),
            _row(9.0, rtt_ms=20.0, loss_pct=0.0),
        ],
    )

    text = convert_trace_to_profile_yaml(
        trace,
        mode="timeline",
        config=TraceProfileConfig(
            name="gap_replay",
            directions=("uplink",),
            min_segment_s=1.0,
            gap_outage_after_s=3.0,
        ),
    )
    out = tmp_path / "profiles.yaml"
    out.write_text(text, encoding="utf-8")

    profile = load_profiles_file(out)["gap_replay"]
    assert [segment.outage for segment in profile.timeline] == [None, OUTAGE_RECONNECT, None]
    assert profile.timeline[1].for_s == 5.0
    steps = expand_timeline(profile, "tun0", direction="uplink")
    assert steps[1].commands == [["ip", "link", "set", "dev", "tun0", "down"]]


def test_timeline_emits_catchup_outage_for_sustained_probe_loss(tmp_path: Path) -> None:
    trace = _write_trace(
        tmp_path / "link_trace.jsonl",
        [
            _row(0.0, rtt_ms=20.0, loss_pct=0.0),
            _row(1.0, rtt_ms=20.0, loss_pct=0.0),
            _row(2.0, rtt_ms=20.0, loss_pct=100.0),
            _row(3.0, rtt_ms=20.0, loss_pct=100.0),
            _row(4.0, rtt_ms=20.0, loss_pct=0.0),
        ],
    )

    text = convert_trace_to_profile_yaml(
        trace,
        mode="timeline",
        config=TraceProfileConfig(
            name="loss_replay",
            directions=("uplink",),
            min_segment_s=1.0,
            loss_outage_min_s=2.0,
        ),
    )
    out = tmp_path / "profiles.yaml"
    out.write_text(text, encoding="utf-8")

    profile = load_profiles_file(out)["loss_replay"]
    assert [segment.outage for segment in profile.timeline] == [None, OUTAGE_CATCHUP, None]
    steps = expand_timeline(profile, "tun0", direction="uplink")
    assert steps[1].commands[-1][-3:] == ["netem", "loss", "100%"]


def test_static_percentiles_and_passive_rate_omission_are_exact(tmp_path: Path) -> None:
    trace = _write_trace(
        tmp_path / "link_trace.jsonl",
        [
            _row(0.0, rtt_ms=20.0, loss_pct=0.0, tx_kbps=9_000.0, rx_kbps=1_000.0, rx_marked=True),
            _row(1.0, rtt_ms=40.0, loss_pct=10.0, tx_kbps=9_000.0, rx_kbps=2_000.0, rx_marked=True),
            _row(2.0, rtt_ms=100.0, loss_pct=20.0, tx_kbps=9_000.0, rx_kbps=4_000.0, rx_marked=True),
        ],
    )

    text = convert_trace_to_profile_yaml(
        trace,
        mode="static",
        config=TraceProfileConfig(name="drive_static"),
    )
    profile_doc = yaml.safe_load(text)["profiles"]["drive_static"]

    assert profile_doc["uplink"] == {
        "delay": "50ms",
        "jitter": "30ms",
        "distribution": "normal",
        "loss": "10%",
    }
    assert profile_doc["downlink"] == {
        "rate": "2000000bit",
        "delay": "50ms",
        "jitter": "30ms",
        "distribution": "normal",
        "loss": "10%",
    }

    out = tmp_path / "profiles.yaml"
    out.write_text(text, encoding="utf-8")
    profile = load_profiles_file(out)["drive_static"]
    assert profile.uplink is not None and profile.uplink.rate_bps is None
    assert profile.downlink is not None and profile.downlink.rate_bps == parse_rate_bps("2mbit")


def test_cli_profile_from_trace_writes_loadable_yaml(tmp_path: Path) -> None:
    trace = _write_trace(
        tmp_path / "link_trace.jsonl",
        [_row(0.0, rtt_ms=20.0, loss_pct=0.0), _row(1.0, rtt_ms=40.0, loss_pct=0.0)],
    )
    out = tmp_path / "generated-profiles.yaml"

    assert (
        main(
            [
                "profile",
                "from-trace",
                str(trace),
                "--mode",
                "static",
                "--name",
                "owner_smoke",
                "--out",
                str(out),
            ]
        )
        == 0
    )
    profiles = load_profiles_file(out)
    assert set(profiles) == {"owner_smoke"}
