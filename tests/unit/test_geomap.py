# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import csv
import json
from pathlib import Path

import pytest
import yaml

from rosotacom.cli import main
from rosotacom.geomap import (
    CSV_COLUMNS,
    GeoSample,
    MetricSample,
    join_metric_samples,
    load_event_metrics,
    load_gps_csv,
    load_link_trace_metrics,
    write_geo_csv,
    write_geomap_html,
)


def _write_jsonl(path: Path, rows: list[dict[str, object]]) -> Path:
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
    return path


def _trace_row(time_s: float, *, tx_kbps: float, rtt_ms: float = 20.0, loss_pct: float = 0.0) -> dict[str, object]:
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
            "tx": {"bytes_delta": 0, "packets_delta": 0, "observed_kbps": tx_kbps},
            "rx": {"bytes_delta": 0, "packets_delta": 0, "observed_kbps": tx_kbps / 2.0},
        },
        "peer_probe": {
            "available": True,
            "rtt_ms": rtt_ms,
            "loss_pct": loss_pct,
            "assumption": "symmetric_path",
        },
    }


def _write_gps_csv(path: Path) -> Path:
    path.write_text(
        "\n".join(
            [
                "time_s,latitude,longitude,altitude_m",
                "100.0,49.000000,8.000000,110",
                "101.0,49.000100,8.000150,111",
                "102.0,49.000220,8.000300,112",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    return path


def test_join_applies_explicit_offset_and_drops_out_of_range_samples() -> None:
    gps = [
        GeoSample(time_s=100.0, latitude=49.0, longitude=8.0),
        GeoSample(time_s=101.0, latitude=49.001, longitude=8.001),
        GeoSample(time_s=102.0, latitude=49.002, longitude=8.002),
    ]
    metrics = [
        MetricSample(time_s=0.0, source="trace:1", values={"observed_tx_kbps": 500.0}, context={}),
        MetricSample(time_s=1.0, source="trace:2", values={"observed_tx_kbps": 600.0}, context={}),
        MetricSample(time_s=3.0, source="trace:3", values={"observed_tx_kbps": 700.0}, context={}),
    ]

    joined = join_metric_samples(
        gps,
        metrics,
        metric="observed_tx_kbps",
        trace_to_gps_offset_s=100.0,
        max_gap_s=0.25,
    )

    assert [sample.metric_value for sample in joined] == [500.0, 600.0]
    assert [sample.gps.time_s for sample in joined] == [100.0, 101.0]
    assert [sample.time_delta_s for sample in joined] == [0.0, 0.0]


def test_link_trace_join_writes_stable_csv_shape(tmp_path: Path) -> None:
    gps = load_gps_csv(_write_gps_csv(tmp_path / "gps.csv"))
    trace = _write_jsonl(
        tmp_path / "link_trace.jsonl",
        [
            _trace_row(0.0, tx_kbps=1000.0, rtt_ms=20.0),
            _trace_row(1.0, tx_kbps=800.0, rtt_ms=50.0),
            _trace_row(3.0, tx_kbps=200.0, rtt_ms=120.0),
        ],
    )

    metrics = load_link_trace_metrics([trace])
    joined = join_metric_samples(
        gps,
        metrics,
        metric="observed_tx_kbps",
        trace_to_gps_offset_s=100.0,
        max_gap_s=0.25,
    )
    out = tmp_path / "geo.csv"
    write_geo_csv(joined, out)

    with out.open(newline="", encoding="utf-8") as fp:
        reader = csv.DictReader(fp)
        rows = list(reader)
    assert tuple(reader.fieldnames or ()) == CSV_COLUMNS
    assert len(rows) == 2
    assert rows[0]["source"].startswith("link_trace:link_trace.jsonl:")
    assert rows[0]["metric"] == "observed_tx_kbps"
    assert rows[0]["gps_time_s"] == "100"
    assert rows[0]["source_time_s"] == "0"
    assert rows[0]["aligned_time_s"] == "100"
    assert rows[0]["time_delta_s"] == "0"
    assert rows[0]["metric_value"] == "1000"
    assert rows[0]["rtt_ms"] == "20"


def test_event_metrics_bin_delivery_and_infer_bounded_lost_time(tmp_path: Path) -> None:
    events = _write_jsonl(
        tmp_path / "events.jsonl",
        [
            {
                "kind": "transit",
                "source": "a",
                "target": "b",
                "topic": "/video",
                "seq": 1,
                "status": "delivered",
                "t_wrap": 100.1,
                "sections": {"ota_hop_ms": 20.0},
            },
            {
                "kind": "transit",
                "source": "a",
                "target": "b",
                "topic": "/video",
                "seq": 2,
                "status": "lost",
                "t_wrap": None,
                "sections": {"ota_hop_ms": None},
            },
            {
                "kind": "transit",
                "source": "a",
                "target": "b",
                "topic": "/video",
                "seq": 3,
                "status": "delivered",
                "t_wrap": 100.3,
                "sections": {"ota_hop_ms": 30.0},
            },
        ],
    )

    samples = load_event_metrics([events], bin_s=0.5, topic="a->b:/video")

    assert len(samples) == 1
    assert samples[0].time_s == pytest.approx(100.25)
    assert samples[0].values["delivery_pct"] == pytest.approx(66.666667)
    assert samples[0].values["event_loss_pct"] == pytest.approx(33.333333)
    assert samples[0].values["ota_hop_ms"] == pytest.approx(25.0)
    assert samples[0].context == {
        "event_topic": "a->b:/video",
        "event_expected": 3,
        "event_delivered": 2,
        "event_lost": 1,
    }


def test_html_map_smoke_writes_route_image_sibling(tmp_path: Path) -> None:
    samples = join_metric_samples(
        [
            GeoSample(time_s=10.0, latitude=49.0, longitude=8.0),
            GeoSample(time_s=11.0, latitude=49.001, longitude=8.002),
        ],
        [
            MetricSample(time_s=10.0, source="trace:1", values={"loss_pct": 0.0}, context={}),
            MetricSample(time_s=11.0, source="trace:2", values={"loss_pct": 20.0}, context={}),
        ],
        metric="loss_pct",
        max_gap_s=0.0,
    )
    out = tmp_path / "map.html"

    write_geomap_html(samples, out, metric="loss_pct", title="fixture map")

    text = out.read_text(encoding="utf-8")
    route_image = tmp_path / "map.route.png"
    assert route_image.read_bytes().startswith(b"\x89PNG\r\n\x1a\n")
    assert 'src="map.route.png"' in text
    assert "data:image" not in text
    assert "<svg" not in text
    assert "fixture map" in text
    assert "loss_pct" in text
    assert "hsl(" in text
    assert "<th>latitude</th>" not in text
    assert "<th>longitude</th>" not in text
    assert "49.001" not in text
    assert "8.002" not in text


def test_cli_geomap_writes_csv_html_and_manifest(tmp_path: Path) -> None:
    gps = _write_gps_csv(tmp_path / "gps.csv")
    trace = _write_jsonl(
        tmp_path / "link_trace.jsonl",
        [_trace_row(0.0, tx_kbps=1000.0), _trace_row(1.0, tx_kbps=800.0)],
    )
    out_csv = tmp_path / "geo.csv"
    out_html = tmp_path / "map.html"
    manifest = tmp_path / "manifest.yaml"

    assert (
        main(
            [
                "geomap",
                "--gps-csv",
                str(gps),
                "--trace",
                str(trace),
                "--metric",
                "observed_tx_kbps",
                "--trace-to-gps-offset-s",
                "100",
                "--max-gap-s",
                "0.25",
                "--out-csv",
                str(out_csv),
                "--out-html",
                str(out_html),
                "--manifest",
                str(manifest),
            ]
        )
        == 0
    )

    assert out_csv.is_file()
    assert out_html.is_file()
    assert (tmp_path / "map.route.png").is_file()
    payload = yaml.safe_load(manifest.read_text(encoding="utf-8"))
    assert payload["kind"] == "geo_link_quality_map"
    assert payload["samples"] == 2
    assert payload["route_image"].endswith("map.route.png")
