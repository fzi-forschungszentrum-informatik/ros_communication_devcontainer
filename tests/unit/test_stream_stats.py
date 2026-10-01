# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import json
import struct
import sys
import types
from pathlib import Path

import rosotacom.cli as rosotacom
from rosotacom.ffmpeg_packet import AV_PKT_FLAG_KEY
from rosotacom.stream_stats import (
    FFMPEG_TYPE,
    StreamSample,
    StreamSource,
    build_report,
    load_bag_source,
    load_events_source,
    parse_source_spec,
    render_markdown,
    samples_from_cdr_records,
    summarize_source,
)


def _cdr_string(offset: int, value: str) -> tuple[bytes, int]:
    pad = (-offset) % 4
    raw = value.encode() + b"\x00"
    chunk = b"\x00" * pad + struct.pack("<I", len(raw)) + raw
    return chunk, offset + len(chunk)


def _packet(*, flags: int, payload_size: int, pts: int = 0) -> bytes:
    body = b""
    offset = 0
    body += struct.pack("<iI", 7, 9)
    offset += 8
    chunk, offset = _cdr_string(offset, "camera")
    body += chunk
    pad = (-offset) % 4
    body += b"\x00" * pad + struct.pack("<ii", 640, 480)
    offset += pad + 8
    chunk, offset = _cdr_string(offset, "h264")
    body += chunk
    pad = (-offset) % 8
    body += b"\x00" * pad + struct.pack("<Q", pts)
    offset += pad + 8
    body += struct.pack("<BB", flags, 0)
    offset += 2
    pad = (-offset) % 4
    body += b"\x00" * pad + struct.pack("<I", payload_size) + b"\xaa" * payload_size
    return b"\x00\x01\x00\x00" + body


def _event(seq: int, *, size: int, topic: str = "/cam", period_s: float = 0.1) -> dict:
    t_wrap = 1_782_000_000.0 + seq * period_s
    return {
        "kind": "transit",
        "source": "a",
        "target": "b",
        "topic": topic,
        "seq": seq,
        "status": "delivered",
        "t_wrap": t_wrap,
        "t_com_in": t_wrap + 0.012,
        "sections": {"ota_hop_ms": 12.0},
        "size_bytes": size,
        "inter_arrival_ms": period_s * 1000.0,
    }


def test_source_stats_are_exact_for_regular_samples() -> None:
    source = StreamSource(
        label="native",
        kind="fixture",
        path=None,
        topic="/numbers",
        samples=tuple(
            StreamSample(timestamp_ns=index * 100_000_000, size_bytes=size)
            for index, size in enumerate([100, 200, 300, 400, 500])
        ),
    )

    stats = summarize_source(source)

    assert stats["count"] == 5
    assert stats["duration_s"] == 0.4
    assert stats["rate_hz"] == 10.0
    assert stats["size_bytes"] == {
        "min": 100.0,
        "mean": 300.0,
        "median": 300.0,
        "p90": 500.0,
        "max": 500.0,
        "std": 141.421,
    }
    assert stats["interval_ms"]["nominal_ms"] == 100.0
    assert stats["interval_ms"]["std"] == 0.0
    assert stats["interval_ms"]["within_10pct_nominal_pct"] == 100.0


def test_ffmpeg_gop_table_uses_packet_flags_and_payload_sizes() -> None:
    sizes = [44000, 3000, 4000, 4200, 3900, 88000, 500, 500, 500, 500]
    records = [
        (
            index * 100_000_000,
            _packet(
                flags=AV_PKT_FLAG_KEY if index % 5 == 0 else 0,
                payload_size=size,
                pts=index,
            ),
        )
        for index, size in enumerate(sizes)
    ]
    source = StreamSource(
        label="handoff",
        kind="fixture",
        path=None,
        topic="/cam/ffmpeg",
        samples=samples_from_cdr_records(records, message_type=FFMPEG_TYPE),
        message_type=FFMPEG_TYPE,
        size_basis="ffmpeg_payload_bytes",
    )

    stats = summarize_source(source)
    gop = stats["gop"]

    assert stats["size_bytes"]["max"] == 88000.0
    assert gop["method"] == "ffmpeg_packet.flags"
    assert gop["keyframes"] == 2
    assert gop["keyframe_spacing_frames"]["median"] == 5.0
    by_position = {entry["position"]: entry for entry in gop["position_size_bytes"]}
    assert by_position[0]["mean"] == 66000.0
    assert by_position[1]["mean"] == 1750.0
    assert gop["most_extreme_gop"]["start_index"] == 5
    assert gop["most_extreme_gop"]["key_to_delta_mean_ratio"] == 176.0


def test_events_source_filters_topic_and_detects_gop_by_size(tmp_path: Path) -> None:
    rows = []
    for seq in range(10):
        rows.append(_event(seq, size=40000 if seq % 5 == 0 else 3000))
        rows.append(_event(seq, size=999, topic="/other"))
    events = tmp_path / "events.jsonl"
    events.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
    spec = parse_source_spec("events", f"post={events}:/cam")

    source = load_events_source(spec)
    stats = summarize_source(source)

    assert source.label == "post"
    assert stats["count"] == 10
    assert stats["rate_hz"] == 10.0
    assert stats["gop"]["method"].startswith("size_bimodality")
    assert stats["gop"]["keyframes"] == 2


def test_events_source_prefers_recorded_keyframe_flags(tmp_path: Path) -> None:
    # Uniform sizes: only the transit rows' keyframe field can mark the GOP.
    rows = [{**_event(seq, size=3000), "keyframe": seq % 5 == 0} for seq in range(10)]
    events = tmp_path / "events.jsonl"
    events.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")

    stats = summarize_source(load_events_source(parse_source_spec("events", f"post={events}:/cam")))

    assert stats["gop"]["method"] == "ffmpeg_packet.flags"
    assert stats["gop"]["keyframes"] == 2


def test_mcap_bag_source_reads_metadata_without_rosbag2(tmp_path: Path, monkeypatch) -> None:
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "stage_0.mcap").write_bytes(b"not a real mcap; fake reader supplies rows")
    (bag / "metadata.yaml").write_text(
        """
rosbag2_bagfile_information:
  storage_identifier: mcap
  relative_file_paths:
    - stage_0.mcap
  topics_with_message_count:
    - topic_metadata:
        name: /numbers
        type: std_msgs/msg/UInt8MultiArray
      message_count: 2
""",
        encoding="utf-8",
    )

    class FakeReader:
        def iter_messages(self, *, topics):
            assert topics == ["/numbers"]
            schema = types.SimpleNamespace(name="std_msgs/msg/UInt8MultiArray")
            channel = types.SimpleNamespace(topic="/numbers")
            yield schema, channel, types.SimpleNamespace(log_time=100, data=b"abcd")
            yield schema, channel, types.SimpleNamespace(log_time=200, data=b"abcdef")

    reader_module = types.ModuleType("mcap.reader")
    reader_module.make_reader = lambda _fp: FakeReader()
    monkeypatch.setitem(sys.modules, "mcap", types.ModuleType("mcap"))
    monkeypatch.setitem(sys.modules, "mcap.reader", reader_module)

    source = load_bag_source(parse_source_spec("bag", f"handoff={bag}:/numbers"))
    stats = summarize_source(source)

    assert source.message_type == "std_msgs/msg/UInt8MultiArray"
    assert source.size_basis == "serialized_message_bytes"
    assert stats["count"] == 2
    assert stats["size_bytes"]["mean"] == 5.0


def test_comparison_markdown_has_one_row_per_stage() -> None:
    sources = [
        StreamSource("pre", "fixture", None, "/cam", (StreamSample(0, 100), StreamSample(100_000_000, 110))),
        StreamSource("handoff", "fixture", None, "/cam/ffmpeg", (StreamSample(0, 40), StreamSample(100_000_000, 4))),
        StreamSource("post", "fixture", None, "/cam", (StreamSample(0, 40), StreamSample(120_000_000, 4))),
    ]

    report = build_report(sources, argv=["rosotacom", "stream-stats"])
    markdown = render_markdown(report)

    assert [row["stage"] for row in report["comparison"]["rows"]] == ["pre", "handoff", "post"]
    assert "| `pre` | fixture | `/cam` |" in markdown
    assert "| `handoff` | fixture | `/cam/ffmpeg` |" in markdown
    assert "| `post` | fixture | `/cam` |" in markdown


def test_cli_stream_stats_events_writes_json_and_markdown(tmp_path: Path, capsys) -> None:
    events = tmp_path / "events.jsonl"
    events.write_text("\n".join(json.dumps(_event(seq, size=1000 + seq)) for seq in range(3)) + "\n", encoding="utf-8")
    out = tmp_path / "out"

    rc = rosotacom.main(["stream-stats", "--events", f"post={events}:/cam", "--out", str(out)])

    captured = capsys.readouterr()
    assert rc == 0
    assert "# Stream stats" in captured.out
    assert (out / "stream-stats.json").is_file()
    assert (out / "stream-stats.md").is_file()
    payload = json.loads((out / "stream-stats.json").read_text(encoding="utf-8"))
    assert payload["sources"][0]["label"] == "post"
