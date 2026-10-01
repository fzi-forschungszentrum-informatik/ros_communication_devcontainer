# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unit tests for rosotacom.forensics — synthetic events.jsonl instances.

Fixtures inject known degradation into an otherwise steady synthetic stream and
assert the detectors report it with exact boundaries, that context joins
(link trace, network profiles reference timeline, state transitions, keyframes) attach the right
evidence, and that every optional input degrades gracefully.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import Any

import pytest

from rosotacom.forensics import (
    LATENCY_EXCURSION,
    LOSS_BURST,
    RATE_COLLAPSE,
    DetectionConfig,
    build_report,
    build_stream_bins,
    build_streams,
    detect_latency_excursions,
    detect_loss_bursts,
    detect_rate_collapses,
    group_incidents,
    parse_timeline_anchor,
    render_markdown,
    write_report,
)

T0 = 1_782_000_000.0  # sender wall clock of seq 0
PERIOD_S = 0.05  # 20 Hz nominal


def _record(
    seq: int,
    *,
    status: str = "delivered",
    latency_ms: float = 10.0,
    size_bytes: int | None = 1000,
    topic: str = "/cam",
    t0: float = T0,
    period_s: float = PERIOD_S,
) -> dict[str, Any]:
    row: dict[str, Any] = {
        "kind": "transit",
        "peer": "b",
        "source": "a",
        "target": "b",
        "topic": topic,
        "direction": "inbound",
        "stage": "com_in",
        "seq": seq,
        "status": status,
    }
    if status == "lost":
        row.update({"t_wrap": None, "t_com_in": None, "sections": {"ota_hop_ms": None}, "size_bytes": None})
        return row
    t_wrap = t0 + seq * period_s
    row.update(
        {
            "t_wrap": t_wrap,
            "t_com_in": t_wrap + latency_ms / 1000.0,
            "sections": {"ota_hop_ms": latency_ms, "ota_hop_uncorrected_ms": latency_ms},
            "size_bytes": size_bytes,
            "inter_arrival_ms": period_s * 1000.0,
            "jitter_ms": 0.1,
        }
    )
    return row


def _steady(count: int, **kwargs: Any) -> list[dict[str, Any]]:
    return [_record(seq, **kwargs) for seq in range(count)]


def _sender_record(seq: int, *, topic: str = "/cam", t0: float = T0, period_s: float = PERIOD_S) -> dict[str, Any]:
    """The sending peer's own observation of the same message (stage com_out)."""
    return {
        "kind": "transit",
        "peer": "a",
        "source": "a",
        "target": "b",
        "topic": topic,
        "direction": "outbound",
        "stage": "com_out",
        "seq": seq,
        "status": "delivered",
        "t_wrap": t0 + seq * period_s,
        "t_com_in": None,
        "sections": {"ota_hop_ms": None},
        "size_bytes": 1000,
    }


def _write_instance(
    root: Path,
    rows: list[dict[str, Any]],
    *,
    peer: str = "b",
    extra_rows: list[dict[str, Any]] | None = None,
    status: dict[str, Any] | None = None,
    link_rows: list[dict[str, Any]] | None = None,
) -> Path:
    status_dir = root / "logs" / peer / "status"
    status_dir.mkdir(parents=True, exist_ok=True)
    lines = [json.dumps(row) for row in [*(extra_rows or []), *rows]]
    (status_dir / "events.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if status is not None:
        (status_dir / "status.json").write_text(json.dumps(status), encoding="utf-8")
    if link_rows is not None:
        (status_dir / "link_trace.jsonl").write_text(
            "\n".join(json.dumps(row) for row in link_rows) + "\n", encoding="utf-8"
        )
    return root


def _stream(rows: list[dict[str, Any]], **kwargs: Any):
    streams = build_streams(rows, **kwargs)
    assert len(streams) == 1
    return streams[0]


# -- detectors: exact boundaries --------------------------------------------- #


def test_loss_burst_detected_with_exact_boundaries() -> None:
    rows = [_record(seq, status="lost") if 100 <= seq <= 119 else _record(seq) for seq in range(300)]
    events = detect_loss_bursts(_stream(rows), DetectionConfig())
    assert [(event.seq_start, event.seq_end, event.count) for event in events] == [(100, 119, 20)]
    event = events[0]
    # Lost rows carry no timestamps; boundaries are nominal send times
    # (reconstructed from the inferred period — exact to well under a period).
    assert event.start_epoch == pytest.approx(T0 + 100 * PERIOD_S, abs=1e-3)
    assert event.end_epoch == pytest.approx(T0 + 119 * PERIOD_S, abs=1e-3)


def test_loss_run_shorter_than_min_burst_is_not_an_event() -> None:
    rows = [_record(seq, status="lost") if seq in (50, 51) else _record(seq) for seq in range(100)]
    assert detect_loss_bursts(_stream(rows), DetectionConfig(loss_burst_min=3)) == []


def test_latency_excursion_detected_with_exact_boundaries() -> None:
    rows = [_record(seq, latency_ms=200.0 if 200 <= seq <= 239 else 10.0) for seq in range(400)]
    events = detect_latency_excursions(_stream(rows), DetectionConfig())
    assert [(event.kind, event.seq_start, event.seq_end, event.count) for event in events] == [
        (LATENCY_EXCURSION, 200, 239, 40)
    ]
    details = events[0].details
    assert details["baseline_ms"] == pytest.approx(10.0)
    assert details["threshold_ms"] == pytest.approx(60.0)  # max(2x10, 10+50)
    assert details["peak_ms"] == pytest.approx(200.0)


def test_latency_excursion_tolerates_interior_dips() -> None:
    # Two sub-threshold samples inside the excursion (min_run is 3): still one
    # event, bounded by the first and last excursive sample.
    def latency(seq: int) -> float:
        if 100 <= seq <= 159:
            return 10.0 if seq in (120, 121) else 200.0
        return 10.0

    rows = [_record(seq, latency_ms=latency(seq)) for seq in range(300)]
    events = detect_latency_excursions(_stream(rows), DetectionConfig())
    assert [(event.seq_start, event.seq_end, event.count) for event in events] == [(100, 159, 58)]


def test_latency_excursion_closes_after_min_run_normal_samples() -> None:
    # Three consecutive normal samples (== min_run) end the event; a second
    # plateau afterwards is a separate event.
    def latency(seq: int) -> float:
        if 100 <= seq <= 119 or 123 <= seq <= 142:
            return 200.0
        return 10.0

    rows = [_record(seq, latency_ms=latency(seq)) for seq in range(300)]
    events = detect_latency_excursions(_stream(rows), DetectionConfig())
    assert [(event.seq_start, event.seq_end) for event in events] == [(100, 119), (123, 142)]


def test_latency_baseline_is_frozen_during_excursion() -> None:
    # Excursion longer than the whole baseline window: a rolling baseline that
    # kept absorbing samples would rise above threshold and split/end the event.
    config = DetectionConfig(latency_baseline_window=20, latency_baseline_min=5)
    rows = [_record(seq, latency_ms=500.0 if 50 <= seq <= 149 else 10.0) for seq in range(200)]
    events = detect_latency_excursions(_stream(rows), config)
    assert [(event.seq_start, event.seq_end) for event in events] == [(50, 149)]


def test_clean_stream_yields_no_events(tmp_path: Path) -> None:
    instance = _write_instance(tmp_path / "inst", _steady(200))
    report = build_report(instance, argv=["test"])
    assert report["events"] == []
    assert report["incidents"] == []
    assert "No degradation events detected" in render_markdown(report)


def test_rate_collapse_detected_on_interior_bins() -> None:
    # 20 Hz nominal; between seq 100 and 160 only every fourth message exists at
    # all (a stall whose gaps never materialized as lost rows), so delivered
    # rate drops to 5 Hz for 3 seconds with zero lost records.
    rows = [_record(seq) for seq in range(300) if not (100 <= seq < 160) or seq % 4 == 0]
    stream = _stream(rows)
    config = DetectionConfig()
    bins = build_stream_bins(stream, run_start=T0, bin_s=config.bin_s)
    events = detect_rate_collapses(stream, bins, config)
    assert len(events) == 1
    event = events[0]
    assert event.kind == RATE_COLLAPSE
    assert event.start_epoch == pytest.approx(T0 + 5.0)
    assert event.end_epoch == pytest.approx(T0 + 8.0)
    assert event.details["min_delivered_hz"] == pytest.approx(5.0)
    assert event.details["nominal_hz"] == pytest.approx(20.0)


def test_stream_edges_are_never_rate_collapsed() -> None:
    # First/last bins are partial by construction; a run starting mid-bin must
    # not produce a spurious edge event.
    rows = [_record(seq) for seq in range(10, 250)]
    stream = _stream(rows)
    config = DetectionConfig()
    bins = build_stream_bins(stream, run_start=T0, bin_s=config.bin_s)
    assert detect_rate_collapses(stream, bins, config) == []


def test_rate_collapse_cause_from_offered_rate() -> None:
    # Same delivered collapse (bins 5-7 at 5 Hz), three sender-side views:
    # offered stayed nominal -> the link dropped it; offered collapsed with it
    # -> the source did; no sender records -> unknowable from this instance.
    rows = [_record(seq) for seq in range(300) if not (100 <= seq < 160) or seq % 4 == 0]
    stream = _stream(rows)
    config = DetectionConfig()
    bins = build_stream_bins(stream, run_start=T0, bin_s=config.bin_s)

    (event,) = detect_rate_collapses(stream, bins, config, offered_hz=[20.0] * len(bins))
    assert event.details["cause"] == "link"
    assert event.details["min_offered_hz"] == pytest.approx(20.0)

    offered = [0.0 if 5 <= index <= 7 else 20.0 for index in range(len(bins))]
    (event,) = detect_rate_collapses(stream, bins, config, offered_hz=offered)
    assert event.details["cause"] == "source"
    assert event.details["max_offered_hz"] == pytest.approx(0.0)

    (event,) = detect_rate_collapses(stream, bins, config)
    assert event.details["cause"] == "unknown"
    assert "min_offered_hz" not in event.details


def test_report_names_link_collapse_when_offered_stays_nominal(tmp_path: Path) -> None:
    # Receiver sees 5 Hz in the stall window; the sender's own records show all
    # 300 messages offered at 20 Hz. Sender rows must feed only the offered
    # rate: merged into the delivered view they would fill the stall and hide
    # the collapse entirely.
    receiver = [_record(seq) for seq in range(300) if not (100 <= seq < 160) or seq % 4 == 0]
    sender = [_sender_record(seq) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", receiver, peer="b")
    _write_instance(instance, sender, peer="a")
    report = build_report(instance, argv=["test"])
    (event,) = report["events"]
    assert event["kind"] == RATE_COLLAPSE
    assert event["details"]["cause"] == "link"
    assert event["details"]["min_delivered_hz"] == pytest.approx(5.0)
    assert event["details"]["min_offered_hz"] == pytest.approx(20.0)
    assert "**rate collapse (link)**" in render_markdown(report)


def test_report_names_source_pause_with_sender_side_records(tmp_path: Path) -> None:
    # The source publishes nothing for 3 s (seq continues afterwards, so no
    # lost rows): both sides go to 0 Hz in bins 5-7 -- the 2026-08-17 cellular
    # shape, where the link delivered everything offered.
    receiver = [_record(seq, t0=T0 + 3.0 if seq >= 100 else T0) for seq in range(300)]
    sender = [_sender_record(seq, t0=T0 + 3.0 if seq >= 100 else T0) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", receiver, peer="b")
    _write_instance(instance, sender, peer="a")
    report = build_report(instance, argv=["test"])
    (event,) = report["events"]
    assert event["kind"] == RATE_COLLAPSE
    assert event["count"] == 3
    assert event["details"]["cause"] == "source"
    assert event["details"]["max_offered_hz"] == pytest.approx(0.0)
    assert "**rate collapse (source paused)**" in render_markdown(report)


def test_report_names_source_slowdown_distinct_from_pause(tmp_path: Path) -> None:
    # The source throttled to 5 Hz and the link delivered all of it: cause is
    # still the source, but the line must not claim a pause.
    pattern = [seq for seq in range(300) if not (100 <= seq < 160) or seq % 4 == 0]
    receiver = [_record(seq) for seq in pattern]
    sender = [_sender_record(seq) for seq in pattern]
    instance = _write_instance(tmp_path / "inst", receiver, peer="b")
    _write_instance(instance, sender, peer="a")
    report = build_report(instance, argv=["test"])
    (event,) = report["events"]
    assert event["details"]["cause"] == "source"
    assert event["details"]["min_offered_hz"] == pytest.approx(5.0)
    assert "**rate collapse (source slowed)**" in render_markdown(report)


def test_rate_collapse_without_sender_records_reads_cause_unknown(tmp_path: Path) -> None:
    rows = [_record(seq) for seq in range(300) if not (100 <= seq < 160) or seq % 4 == 0]
    report = build_report(_write_instance(tmp_path / "inst", rows), argv=["test"])
    (event,) = report["events"]
    assert event["details"]["cause"] == "unknown"
    assert "cause unknown (no sender-side records)" in render_markdown(report)


def test_offered_bins_align_when_stream_starts_mid_bin(tmp_path: Path) -> None:
    # A second stream starting 2.5 s earlier shifts the shared run_start, so
    # /cam's bins begin at a non-zero bin index. The offered rate must land in
    # those same bins: misaligned by the offset, the paused source would read
    # as offered-at-nominal and the cause would flip to "link".
    cam_receiver = [_record(seq, t0=T0 + 3.0 if seq >= 100 else T0) for seq in range(300)]
    tel_receiver = [_record(seq, topic="/tel", t0=T0 - 2.5) for seq in range(300)]
    cam_sender = [_sender_record(seq, t0=T0 + 3.0 if seq >= 100 else T0) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", [*cam_receiver, *tel_receiver], peer="b")
    _write_instance(instance, cam_sender, peer="a")
    report = build_report(instance, argv=["test"])
    collapses = [event for event in report["events"] if event["kind"] == RATE_COLLAPSE]
    assert [event["stream"] for event in collapses] == ["a->b:/cam"]
    assert collapses[0]["details"]["cause"] == "source"


# -- timeline bins ------------------------------------------------------------ #


def test_stream_bins_fill_interior_holes_with_zero_counts() -> None:
    rows = [_record(seq) for seq in range(300) if not (100 <= seq < 160)]
    stream = _stream(rows)
    bins = build_stream_bins(stream, run_start=T0, bin_s=1.0)
    assert [entry["bin_start_s"] for entry in bins] == [float(index) for index in range(15)]
    stalled = [entry for entry in bins if entry["delivered"] == 0]
    assert [entry["bin_start_s"] for entry in stalled] == [5.0, 6.0, 7.0]
    assert all(entry["expected"] == 0 for entry in stalled)


# -- incidents and context ---------------------------------------------------- #


def test_overlapping_events_group_into_one_incident() -> None:
    rows = [
        _record(seq, status="lost")
        if 100 <= seq <= 119
        else _record(seq, latency_ms=300.0 if 120 <= seq <= 139 else 10.0)
        for seq in range(300)
    ]
    stream = _stream(rows)
    config = DetectionConfig()
    events = [
        *detect_loss_bursts(stream, config),
        *detect_latency_excursions(stream, config),
    ]
    incidents = group_incidents(events, merge_gap_s=config.merge_gap_s)
    assert len(incidents) == 1
    assert sorted(event.kind for event in incidents[0].events) == [LATENCY_EXCURSION, LOSS_BURST]


def test_link_trace_context_joins_only_overlapping_samples(tmp_path: Path) -> None:
    rows = [_record(seq, status="lost") if 100 <= seq <= 119 else _record(seq) for seq in range(300)]
    burst_epoch = T0 + 110 * PERIOD_S

    def trace(epoch: float, rx_kbps: float) -> dict[str, Any]:
        return {
            "schema_version": 1,
            "kind": "link_trace",
            "generated_at": datetime.fromtimestamp(epoch).isoformat(),
            "peer": "b",
            "remote": "a",
            "passive_counter_delta": {"available": True, "rx": {"observed_kbps": rx_kbps}, "tx": {}},
            "peer_probe": {"available": True, "rtt_ms": 42.0, "loss_pct": 5.0},
        }

    instance = _write_instance(
        tmp_path / "inst",
        rows,
        link_rows=[trace(burst_epoch - 0.3, 12.5), trace(burst_epoch + 90.0, 999.0)],
    )
    report = build_report(instance, argv=["test"])
    context = report["incidents"][0]["context"]["link_trace"]
    assert context["available"] is True
    assert context["samples"] == 1  # the +90 s sample is outside the window
    assert context["observed_rx_kbps"] == {"min": 12.5, "max": 12.5}
    assert context["rtt_ms"] == {"min": 42.0, "max": 42.0}


def test_timeline_profile_step_context_with_explicit_anchor(tmp_path: Path) -> None:
    profiles = tmp_path / "profiles.yaml"
    profiles.write_text(
        json.dumps(
            {
                "profiles": {
                    "steps": {
                        "timeline": [
                            {"for": "5s", "uplink": {"rate": "8mbit"}},
                            {"for": "5s", "outage": "catchup"},
                            {"for": "50s", "uplink": {"rate": "1mbit", "loss": "3%"}},
                        ]
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    # Loss burst at seq 110-129 -> t=+5.5..+6.45s after T0: inside step 1 (the outage).
    rows = [_record(seq, status="lost") if 110 <= seq <= 129 else _record(seq) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", rows)
    report = build_report(
        instance,
        profile="steps",
        profiles_file=profiles,
        timeline_anchor=T0,
        argv=["test"],
    )
    assert report["provenance"]["profile"]["anchor_provenance"] == "explicit --timeline-anchor"
    active = report["incidents"][0]["context"]["profile"]["active_steps"]
    assert [step["index"] for step in active] == [1]
    assert active[0]["outage"] == "catchup"
    # The full expanded timeline is part of the provenance.
    assert [step["index"] for step in report["provenance"]["profile"]["steps"]] == [0, 1, 2]


def test_static_profile_context_is_constant(tmp_path: Path) -> None:
    profiles = tmp_path / "profiles.yaml"
    profiles.write_text(
        json.dumps({"profiles": {"flat": {"uplink": {"rate": "1mbit", "loss": "3%", "seed": 7}}}}),
        encoding="utf-8",
    )
    rows = [_record(seq, status="lost") if 100 <= seq <= 119 else _record(seq) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", rows)
    report = build_report(instance, profile="flat", profiles_file=profiles, argv=["test"])
    profile = report["provenance"]["profile"]
    assert profile["kind"] == "static"
    assert profile["uplink"]["seed"] == 7  # profile/seed provenance
    assert report["incidents"][0]["context"]["profile"]["active_steps"] == "static (constant for the whole run)"


def test_state_transition_context_joins_diagnosis(tmp_path: Path) -> None:
    rows = [_record(seq, status="lost") if 100 <= seq <= 119 else _record(seq) for seq in range(300)]
    transition = {
        "kind": "state_transition",
        "at": datetime.fromtimestamp(T0 + 110 * PERIOD_S).isoformat(timespec="milliseconds"),
        "peer": "b",
        "topic": "/cam",
        "direction": "inbound",
        "from": {"overall": "OK"},
        "to": {"overall": "STALLED"},
        "diagnosis": "com_in stopped receiving /cam",
    }
    instance = _write_instance(tmp_path / "inst", rows, extra_rows=[transition])
    report = build_report(instance, argv=["test"])
    joined = report["incidents"][0]["context"]["state_transitions"]
    assert len(joined) == 1
    assert joined[0]["to"] == "STALLED"
    assert "com_in stopped receiving" in joined[0]["diagnosis"]


# -- keyframes ---------------------------------------------------------------- #


def _gop_rows(count: int, *, gop: int = 5, key_size: int = 40000, delta_size: int = 3000) -> list[dict[str, Any]]:
    return [_record(seq, size_bytes=key_size if seq % gop == 0 else delta_size) for seq in range(count)]


def test_keyframes_annotated_for_declared_ffmpeg_stream(tmp_path: Path) -> None:
    status = {"topics": [{"base": "/cam", "type": "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"}]}
    instance = _write_instance(tmp_path / "inst", _gop_rows(200), status=status)
    report = build_report(instance, argv=["test"])
    keyframes = report["streams"]["a->b:/cam"]["keyframes"]
    assert keyframes["count"] == 40  # every 5th of 200
    assert "declared FFMPEGPacket" in keyframes["provenance"]


def test_keyframes_annotated_by_size_bimodality_without_declaration(tmp_path: Path) -> None:
    instance = _write_instance(tmp_path / "inst", _gop_rows(200))
    report = build_report(instance, argv=["test"])
    keyframes = report["streams"]["a->b:/cam"]["keyframes"]
    assert keyframes["count"] == 40
    assert "size_bimodality" in keyframes["provenance"]


def test_keyframes_prefer_transit_flags_over_size_bimodality(tmp_path: Path) -> None:
    # Uniform sizes carry no GOP signal and there is no declaration: only the
    # real per-row flags can be the source of this annotation.
    rows = [{**_record(seq), "keyframe": seq % 5 == 0} for seq in range(200)]
    instance = _write_instance(tmp_path / "inst", rows)
    report = build_report(instance, argv=["test"])
    keyframes = report["streams"]["a->b:/cam"]["keyframes"]
    assert keyframes["count"] == 40
    assert keyframes["provenance"] == "ffmpeg_flags (transit records)"


def test_keyframe_flag_coverage_counts_delivered_rows_only(tmp_path: Path) -> None:
    rows = [
        _record(seq, status="lost") if 50 <= seq < 80 else {**_record(seq), "keyframe": seq % 5 == 0}
        for seq in range(200)
    ]
    report = build_report(_write_instance(tmp_path / "inst", rows), argv=["test"])
    keyframes = report["streams"]["a->b:/cam"]["keyframes"]
    assert keyframes["provenance"] == "ffmpeg_flags (transit records)"
    assert keyframes["count"] == 34  # 40 flagged positions minus the 6 lost ones


def test_sparse_keyframe_flags_fall_back_to_size_bimodality(tmp_path: Path) -> None:
    # Half the rows predate the field (receiver upgraded mid-instance): coverage
    # 50% is below the gate, so the documented size fallback decides.
    rows = [
        {**_record(seq, size_bytes=40000 if seq % 5 == 0 else 3000), "keyframe": seq % 5 == 0}
        if seq < 100
        else _record(seq, size_bytes=40000 if seq % 5 == 0 else 3000)
        for seq in range(200)
    ]
    report = build_report(_write_instance(tmp_path / "inst", rows), argv=["test"])
    keyframes = report["streams"]["a->b:/cam"]["keyframes"]
    assert keyframes["count"] == 40
    assert keyframes["provenance"].startswith("size_bimodality")


def test_uniform_and_rare_spike_streams_are_not_keyframe_annotated(tmp_path: Path) -> None:
    uniform = _steady(200)
    # One 100x outlier in 200 messages: share 0.5 % — below the GOP gate.
    spiky = [_record(seq, size_bytes=100000 if seq == 50 else 1000) for seq in range(200)]
    report_uniform = build_report(_write_instance(tmp_path / "u", uniform), argv=["test"])
    report_spiky = build_report(_write_instance(tmp_path / "s", spiky), argv=["test"])
    assert report_uniform["streams"]["a->b:/cam"]["keyframes"] is None
    assert report_spiky["streams"]["a->b:/cam"]["keyframes"] is None


def test_keyframe_traffic_context_marks_coincident_event(tmp_path: Path) -> None:
    # Keyframe every 5th message; loss burst starting right after the seq-100 keyframe.
    rows = [
        _record(seq, status="lost") if 101 <= seq <= 110 else _record(seq, size_bytes=40000 if seq % 5 == 0 else 3000)
        for seq in range(300)
    ]
    status = {"topics": [{"base": "/cam", "type": "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"}]}
    instance = _write_instance(tmp_path / "inst", rows, status=status)
    report = build_report(instance, argv=["test"])
    traffic = report["incidents"][0]["context"]["traffic"]
    assert traffic["keyframes"] > 0
    assert traffic["keyframe_coincident_event_start"] is True


# -- graceful degradation and provenance -------------------------------------- #


def test_report_degrades_gracefully_without_optional_inputs(tmp_path: Path) -> None:
    rows = [_record(seq, status="lost") if 100 <= seq <= 119 else _record(seq) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", rows)  # no status.json, trace, manifest, profile
    report = build_report(instance, argv=["test"])
    context = report["incidents"][0]["context"]
    assert context["link_trace"] == {"available": False, "reason": "no link_trace.jsonl recorded"}
    assert context["profile"] is None
    assert context["state_transitions"] == []
    assert report["provenance"]["profile"] is None
    markdown = render_markdown(report)
    assert "link: unavailable" in markdown


def test_missing_events_jsonl_is_an_error(tmp_path: Path) -> None:
    (tmp_path / "inst" / "logs" / "b" / "status").mkdir(parents=True)
    with pytest.raises(RuntimeError, match="events.jsonl"):
        build_report(tmp_path / "inst", argv=["test"])


def test_report_provenance_is_self_describing(tmp_path: Path) -> None:
    instance = _write_instance(tmp_path / "inst", _steady(100))
    (instance / "manifest.yaml").write_text(
        "instance_id: abc123\neffective_config_sha256: deadbeef\nrosotacom_version: 2.1.dev0\n",
        encoding="utf-8",
    )
    report = build_report(instance, argv=["rosotacom", "report", str(instance)])
    provenance = report["provenance"]
    assert provenance["command"].startswith("rosotacom report ")
    assert provenance["rosotacom_version"]
    assert provenance["instance"]["instance_id"] == "abc123"
    assert provenance["instance"]["effective_config_sha256"] == "deadbeef"
    assert provenance["inputs"]["events"]
    assert provenance["detection_config"]["loss_burst_min"] == 3
    assert "correlation, not causation" in provenance["caveat"]
    assert "correlation, not causation" in render_markdown(report)


def test_parse_timeline_anchor_accepts_epoch_and_iso() -> None:
    assert parse_timeline_anchor(None) is None
    assert parse_timeline_anchor("123.5") == pytest.approx(123.5)
    iso = datetime.fromtimestamp(T0).isoformat()
    assert parse_timeline_anchor(iso) == pytest.approx(T0)
    with pytest.raises(RuntimeError, match="timeline-anchor"):
        parse_timeline_anchor("not-a-time")


# -- outputs and CLI ----------------------------------------------------------- #


def test_write_report_writes_json_and_markdown(tmp_path: Path) -> None:
    instance = _write_instance(tmp_path / "inst", _steady(100))
    report = build_report(instance, argv=["test"])
    written = write_report(report, tmp_path / "out", figures=False)
    payload = json.loads(written["json"].read_text(encoding="utf-8"))
    assert payload["kind"] == "forensics_report"
    assert written["markdown"].read_text(encoding="utf-8").startswith("# Degradation forensics report")
    assert written["figures"] == []
    assert written["figures_note"] == "figures disabled (--no-figures)"


def test_cli_report_end_to_end(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    import rosotacom.cli as rosotacom

    rows = [_record(seq, status="lost") if 100 <= seq <= 119 else _record(seq) for seq in range(300)]
    instance = _write_instance(tmp_path / "inst", rows)
    assert rosotacom.main(["report", str(instance), "--no-figures"]) == 0
    captured = capsys.readouterr()
    assert "loss burst" in captured.out
    assert (instance / "report" / "report.json").is_file()
    assert (instance / "report" / "report.md").is_file()


def test_cli_report_json_output_and_threshold_passthrough(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    import rosotacom.cli as rosotacom

    rows = [_record(seq, status="lost") if seq in (50, 51) else _record(seq) for seq in range(200)]
    instance = _write_instance(tmp_path / "inst", rows)
    out_dir = tmp_path / "custom-out"
    assert (
        rosotacom.main(
            ["report", str(instance), "--json", "--no-figures", "--loss-burst-min", "2", "--out", str(out_dir)]
        )
        == 0
    )
    payload = json.loads(capsys.readouterr().out)
    assert [(event["kind"], event["seq_start"], event["seq_end"]) for event in payload["events"]] == [
        (LOSS_BURST, 50, 51)
    ]
    assert payload["provenance"]["detection_config"]["loss_burst_min"] == 2
    assert (out_dir / "report.json").is_file()


def test_cli_report_on_non_instance_dir_fails(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    import rosotacom.cli as rosotacom

    assert rosotacom.main(["report", str(tmp_path)]) == 1
    assert "events.jsonl" in capsys.readouterr().err


# -- figures (requires the [plots] extra) -------------------------------------- #

try:
    import matplotlib  # noqa: F401

    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


@pytest.mark.skipif(not HAS_MATPLOTLIB, reason="matplotlib is required for forensics figures")
def test_report_figures_written_and_non_empty(tmp_path: Path) -> None:
    rows = [
        _record(seq, status="lost")
        if 100 <= seq <= 119
        else _record(seq, latency_ms=200.0 if 200 <= seq <= 239 else 10.0, size_bytes=40000 if seq % 5 == 0 else 3000)
        for seq in range(400)
    ]
    instance = _write_instance(tmp_path / "inst", rows)
    report = build_report(instance, argv=["test"])
    written = write_report(report, tmp_path / "out", figures=True)
    assert len(written["figures"]) == 1
    for figure in written["figures"]:
        assert figure.is_file()
        assert figure.stat().st_size > 0
