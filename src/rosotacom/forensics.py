# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Degradation forensics for one recorded session instance (``rosotacom report``).

Turns the artifacts a run already writes — transit records in
``logs/<peer>/status/events.jsonl``, ``status.json`` snapshots, and the optional
``link_trace.jsonl`` — into an explanation: *where* in time delivery degraded,
*how* (loss burst, latency excursion, rate collapse), and *with what context*
(link-trace samples, active profile segment, pipeline state transitions,
traffic/keyframe bursts). Offline only; a pure projection of recorded data.

Everything here is deterministic: the same inputs and thresholds always yield
the same events with the same boundaries. Context is correlation, not
causation — the report says so, and the causal test remains reproduction under
an emulated profile.
"""

from __future__ import annotations

import json
import math
import re
import shlex
import subprocess
import sys
from collections import deque
from collections.abc import Collection
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from . import __version__
from .benchmark import _infer_nominal_period_s, _percentile, _section_ms, _send_time
from .ffmpeg_packet import keyframes_by_size
from .transit import BUNCHED_GAP_FRACTION, STALLED_GAP_FRACTION, join_transit_records, load_transit_records

LOSS_BURST = "loss_burst"
LATENCY_EXCURSION = "latency_excursion"
RATE_COLLAPSE = "rate_collapse"

# Undeclared streams are keyframe-annotated only when the flagged share looks
# like a real GOP structure (one keyframe per gop_size, gop_size >= ~2.5).
KEYFRAME_SHARE_MIN = 0.02
KEYFRAME_SHARE_MAX = 0.4

# Real per-message FFMPEG flags win over the size heuristic once they cover
# most delivered rows; the slack tolerates isolated parse failures in a
# recording that otherwise carries the field.
KEYFRAME_FLAG_COVERAGE_MIN = 0.9

CAVEAT = (
    "Context is correlation, not causation: a link-trace sample or profile step that overlaps an event "
    "does not prove it caused the event. The causal test is reproduction under an emulated profile."
)


@dataclass(frozen=True)
class DetectionConfig:
    """Thresholds for the deterministic event detectors. All configurable via CLI."""

    bin_s: float = 1.0
    loss_burst_min: int = 3
    latency_baseline_window: int = 30
    latency_baseline_min: int = 10
    latency_ratio: float = 2.0
    latency_min_delta_ms: float = 50.0
    latency_min_run: int = 3
    rate_collapse_fraction: float = 0.5
    rate_collapse_min_bins: int = 2
    # None -> defaults to bin_s at use sites.
    incident_merge_gap_s: float | None = None
    context_margin_s: float | None = None

    def __post_init__(self) -> None:
        if self.bin_s <= 0.0:
            raise ValueError("bin_s must be > 0")
        if self.loss_burst_min < 1 or self.latency_min_run < 1 or self.rate_collapse_min_bins < 1:
            raise ValueError("minimum run lengths must be >= 1")
        if self.latency_baseline_min < 2 or self.latency_baseline_window < self.latency_baseline_min:
            raise ValueError("baseline window must hold at least baseline_min >= 2 samples")
        if self.latency_ratio <= 1.0:
            raise ValueError("latency_ratio must be > 1")
        if not 0.0 < self.rate_collapse_fraction < 1.0:
            raise ValueError("rate_collapse_fraction must be in (0, 1)")

    @property
    def merge_gap_s(self) -> float:
        return self.bin_s if self.incident_merge_gap_s is None else self.incident_merge_gap_s

    @property
    def margin_s(self) -> float:
        return self.bin_s if self.context_margin_s is None else self.context_margin_s


# --------------------------------------------------------------------------- #
# Input discovery and loading
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ReportInputs:
    instance_dir: Path
    peers: tuple[str, ...]
    events_paths: tuple[Path, ...]
    status_paths: tuple[Path, ...]
    link_trace_paths: tuple[Path, ...]
    manifest_path: Path | None


def discover_inputs(instance_dir: str | Path, *, peers: tuple[str, ...] = ()) -> ReportInputs:
    """Locate per-peer status artifacts under ``<instance>/logs/<peer>/status/``."""
    root = Path(instance_dir)
    if not root.is_dir():
        raise RuntimeError(f"not a directory: {root}")
    logs_dir = root / "logs"
    found = tuple(
        sorted(child.name for child in logs_dir.iterdir() if child.is_dir() and (child / "status").is_dir())
        if logs_dir.is_dir()
        else ()
    )
    selected = peers or found
    events = tuple(path for peer in selected if (path := logs_dir / peer / "status" / "events.jsonl").is_file())
    if not events:
        raise RuntimeError(
            f"no logs/<peer>/status/events.jsonl under {root} — not a session-instance directory, "
            "or the run had shared.use_status_overview disabled"
        )
    status = tuple(path for peer in selected if (path := logs_dir / peer / "status" / "status.json").is_file())
    traces = tuple(path for peer in selected if (path := logs_dir / peer / "status" / "link_trace.jsonl").is_file())
    manifest = root / "manifest.yaml"
    return ReportInputs(
        instance_dir=root,
        peers=selected,
        events_paths=events,
        status_paths=status,
        link_trace_paths=traces,
        manifest_path=manifest if manifest.is_file() else None,
    )


def _parse_wall_iso(value: Any) -> float | None:
    """ISO wall timestamp (as written by the recorders, naive local time) -> epoch seconds.

    Naive stamps are interpreted in the analysis host's local timezone; analyzing on a
    host in another timezone shifts trace/transition joins by the tz difference.
    """
    if not isinstance(value, str) or not value:
        return None
    try:
        return datetime.fromisoformat(value).timestamp()
    except ValueError:
        return None


def _load_jsonl_rows(paths: tuple[Path, ...], *, kind: str, time_key: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in paths:
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue  # forensics is best-effort on optional rows; transit loading stays strict
            if isinstance(row, dict) and row.get("kind") == kind:
                epoch = _parse_wall_iso(row.get(time_key))
                if epoch is not None:
                    row["_epoch"] = epoch
                    rows.append(row)
    rows.sort(key=lambda row: row["_epoch"])
    return rows


def load_state_transitions(paths: tuple[Path, ...]) -> list[dict[str, Any]]:
    return _load_jsonl_rows(paths, kind="state_transition", time_key="at")


def load_link_trace_rows(paths: tuple[Path, ...]) -> list[dict[str, Any]]:
    return _load_jsonl_rows(paths, kind="link_trace", time_key="generated_at")


def load_declared_ffmpeg_topics(status_paths: tuple[Path, ...]) -> set[str]:
    """Base topics whose declared native type is an FFMPEGPacket (from status.json)."""
    declared: set[str] = set()
    for path in status_paths:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        topics = payload.get("topics") if isinstance(payload, dict) else None
        if not isinstance(topics, list):
            continue
        for entry in topics:
            if not isinstance(entry, dict):
                continue
            if "FFMPEGPacket" in str(entry.get("type") or ""):
                base = str(entry.get("base") or "")
                if base:
                    declared.add(base)
    return declared


# --------------------------------------------------------------------------- #
# Streams — joined transit records grouped per (source, target, topic)
# --------------------------------------------------------------------------- #


@dataclass
class Stream:
    source: str
    target: str
    topic: str
    records: list[dict[str, Any]]  # joined, seq-ordered
    period_s: float | None
    anchor_seq: int
    anchor_t: float
    keyframe_seqs: frozenset[int] = frozenset()
    keyframe_provenance: str | None = None

    @property
    def label(self) -> str:
        if self.source or self.target:
            return f"{self.source}->{self.target}:{self.topic}"
        return self.topic

    @property
    def nominal_hz(self) -> float | None:
        return (1.0 / self.period_s) if self.period_s else None

    def send_time(self, record: dict[str, Any]) -> float:
        return _send_time(
            record, seq0=self.anchor_seq, t0=self.anchor_t, period_s=self.period_s or 0.0, time_field="t_wrap"
        )


def _annotate_keyframes(stream: Stream, *, declared: bool) -> None:
    """Mark keyframes, preferring the real FFMPEG flags in the transit records.

    The receiver parses ``AV_PKT_FLAG_KEY`` out of the wrapped FFMPEGPacket CDR
    at com_in and writes it as ``keyframe`` on delivered rows, so when the field
    covers (nearly) all delivered rows the real flags are authoritative — the
    field only exists where the wrapper's ``msg_type`` named an FFMPEGPacket,
    which is a stronger declaration than status.json, so no share gate applies.

    Recordings from before the field existed fall back to
    ``rosotacom.ffmpeg_packet.keyframes_by_size``: declared FFMPEGPacket streams
    (type from status.json) are always annotated; other streams only when the
    flagged share looks like a real GOP structure, so uniform or spiky
    non-video streams are not mislabeled.
    """
    delivered = [record for record in stream.records if record.get("status") != "lost"]
    flagged = [record for record in delivered if isinstance(record.get("keyframe"), bool)]
    if delivered and len(flagged) / len(delivered) > KEYFRAME_FLAG_COVERAGE_MIN:
        stream.keyframe_seqs = frozenset(int(record["seq"]) for record in flagged if record["keyframe"])
        stream.keyframe_provenance = "ffmpeg_flags (transit records)"
        return

    sized = [record for record in stream.records if record.get("size_bytes") is not None]
    if not sized:
        return
    flags = keyframes_by_size([float(record["size_bytes"]) for record in sized])
    share = sum(flags) / len(flags)
    if not declared and not (KEYFRAME_SHARE_MIN <= share <= KEYFRAME_SHARE_MAX):
        return
    if declared and not any(flags):
        return
    stream.keyframe_seqs = frozenset(int(record["seq"]) for record, flag in zip(sized, flags, strict=True) if flag)
    origin = "declared FFMPEGPacket stream" if declared else f"size share {share * 100.0:.1f}%"
    stream.keyframe_provenance = f"size_bimodality ({origin}); these transit records carry no FFMPEG flags"


def build_streams(records: list[dict[str, Any]], *, declared_ffmpeg_topics: Collection[str] = ()) -> list[Stream]:
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for record in join_transit_records(records):
        key = (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
        )
        grouped.setdefault(key, []).append(record)

    streams: list[Stream] = []
    for (source, target, topic), stream_records in sorted(grouped.items()):
        stream_records.sort(key=lambda record: int(record["seq"]))
        period_s = _infer_nominal_period_s(stream_records, time_field="t_wrap")
        stamped = [record for record in stream_records if record.get("t_wrap") is not None]
        anchor = min(stamped or stream_records, key=lambda record: int(record["seq"]))
        stream = Stream(
            source=source,
            target=target,
            topic=topic,
            records=stream_records,
            period_s=period_s,
            anchor_seq=int(anchor["seq"]),
            anchor_t=float(anchor.get("t_wrap") or 0.0),
        )
        _annotate_keyframes(stream, declared=topic in declared_ffmpeg_topics)
        streams.append(stream)
    return streams


# --------------------------------------------------------------------------- #
# Per-stream timeline bins
# --------------------------------------------------------------------------- #


def build_stream_bins(stream: Stream, *, run_start: float, bin_s: float) -> list[dict[str, Any]]:
    """Project one stream onto contiguous time bins on the publish timeline.

    Bins run from the stream's first to its last observed record; interior bins
    without records are emitted with zero counts (a stall is data, not absence).
    Lost records are placed at their reconstructed nominal send time.
    """
    if not stream.records:
        return []
    placed = [(stream.send_time(record), record) for record in stream.records]
    placed.sort(key=lambda item: item[0])
    first_index = math.floor((placed[0][0] - run_start) / bin_s)
    last_index = math.floor((placed[-1][0] - run_start) / bin_s)

    by_index: dict[int, list[tuple[float, dict[str, Any]]]] = {}
    for send_t, record in placed:
        by_index.setdefault(math.floor((send_t - run_start) / bin_s), []).append((send_t, record))

    nominal_ms = (stream.period_s or 0.0) * 1000.0
    bins: list[dict[str, Any]] = []
    for index in range(first_index, last_index + 1):
        members = by_index.get(index, [])
        delivered = [record for _t, record in members if record.get("status") != "lost"]
        lost = len(members) - len(delivered)
        latencies = [
            latency
            for record in delivered
            if (latency := _section_ms(record, "ota_hop_ms", "ota_hop_uncorrected_ms")) is not None
        ]
        gaps = [float(record["inter_arrival_ms"]) for record in delivered if record.get("inter_arrival_ms") is not None]
        sizes = [float(record["size_bytes"]) for record in delivered if record.get("size_bytes") is not None]
        keyframes = sum(1 for record in delivered if int(record["seq"]) in stream.keyframe_seqs)
        expected = len(members)
        bins.append(
            {
                "bin_start_s": round(index * bin_s, 6),
                "bin_end_s": round((index + 1) * bin_s, 6),
                "bin_start_epoch": run_start + index * bin_s,
                "expected": expected,
                "delivered": len(delivered),
                "lost": lost,
                "loss_pct": round(100.0 * lost / expected, 3) if expected else None,
                "delivered_hz": round(len(delivered) / bin_s, 3),
                "latency_p50_ms": _percentile(latencies, 0.50),
                "latency_p95_ms": _percentile(latencies, 0.95),
                "latency_max_ms": round(max(latencies), 3) if latencies else None,
                "inter_arrival_p95_ms": _percentile(gaps, 0.95),
                "stalled": sum(1 for gap in gaps if nominal_ms and gap > STALLED_GAP_FRACTION * nominal_ms),
                "bunched": sum(1 for gap in gaps if nominal_ms and gap < BUNCHED_GAP_FRACTION * nominal_ms),
                "payload_bandwidth_bps": round(sum(sizes) * 8.0 / bin_s, 3),
                "mean_size_bytes": round(sum(sizes) / len(sizes), 3) if sizes else None,
                "max_size_bytes": round(max(sizes), 3) if sizes else None,
                "keyframes": keyframes,
            }
        )
    return bins


def build_offered_timelines(
    offered_records: list[dict[str, Any]],
    streams: list[Stream],
    timelines: dict[str, list[dict[str, Any]]],
    *,
    run_start: float,
    bin_s: float,
) -> dict[str, list[float]]:
    """Per-bin offered rate per stream, from the sending peer's own transit records.

    ``offered_records`` are ``direction: outbound`` rows (stage ``com_out``):
    what the source handed to the link. Their ``t_wrap`` is the same header
    stamp the receiving side records, so offered and delivered bins share one
    publish timeline with no clock join. Streams without a single stamped
    sender-side row are absent from the result — their collapse cause stays
    unknowable.
    """
    grouped: dict[tuple[str, str, str], list[float]] = {}
    for record in join_transit_records(offered_records):
        if record.get("t_wrap") is None:
            continue
        key = (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
        )
        grouped.setdefault(key, []).append(float(record["t_wrap"]))

    offered: dict[str, list[float]] = {}
    for stream in streams:
        stamps = grouped.get((stream.source, stream.target, stream.topic))
        bins = timelines[stream.label]
        if not stamps or not bins:
            continue
        # bin_start_epoch is run_start + index * bin_s by construction; round
        # instead of floor so float error cannot shift the alignment by one.
        first_index = round((float(bins[0]["bin_start_epoch"]) - run_start) / bin_s)
        counts = [0] * len(bins)
        for stamp in stamps:
            index = math.floor((stamp - run_start) / bin_s) - first_index
            if 0 <= index < len(counts):
                counts[index] += 1
        offered[stream.label] = [round(count / bin_s, 3) for count in counts]
    return offered


# --------------------------------------------------------------------------- #
# Event detection
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class DegradationEvent:
    kind: str
    stream: str
    start_epoch: float
    end_epoch: float
    count: int
    seq_start: int | None = None
    seq_end: int | None = None
    details: dict[str, Any] = field(default_factory=dict)


def detect_loss_bursts(stream: Stream, config: DetectionConfig) -> list[DegradationEvent]:
    """Maximal runs of >= ``loss_burst_min`` consecutive lost sequence numbers.

    Lost records carry no timestamps; boundaries on the time axis are the
    reconstructed nominal send times of the first and last lost message.
    """
    events: list[DegradationEvent] = []
    run: list[dict[str, Any]] = []

    def flush() -> None:
        if len(run) >= config.loss_burst_min:
            events.append(
                DegradationEvent(
                    kind=LOSS_BURST,
                    stream=stream.label,
                    start_epoch=stream.send_time(run[0]),
                    end_epoch=stream.send_time(run[-1]),
                    count=len(run),
                    seq_start=int(run[0]["seq"]),
                    seq_end=int(run[-1]["seq"]),
                    details={"send_times_reconstructed": True},
                )
            )
        run.clear()

    for record in stream.records:
        if record.get("status") == "lost":
            run.append(record)
        else:
            flush()
    flush()
    return events


def detect_latency_excursions(stream: Stream, config: DetectionConfig) -> list[DegradationEvent]:
    """Sustained latency rises against a rolling per-stream baseline.

    Baseline: median of the last ``latency_baseline_window`` *normal* delivered
    latencies; at least ``latency_baseline_min`` samples must exist before
    detection starts. A sample is excursive when it reaches
    ``max(latency_ratio * baseline, baseline + latency_min_delta_ms)`` — the
    ratio makes small baselines honest, the absolute floor keeps tiny baselines
    from flagging noise.

    Hysteresis is symmetric in ``latency_min_run``: an event needs at least that
    many excursive samples, and it closes only after that many *consecutive*
    normal samples — so an oscillating excursion is one event, not a fragment
    per dip. The baseline is frozen for the whole event (interior dips never
    feed it; an excursion must not drag its own reference up), and event
    boundaries are the first and last excursive sample. Lost messages inside
    the window belong to loss-burst detection instead.
    """
    baseline: deque[float] = deque(maxlen=config.latency_baseline_window)
    events: list[DegradationEvent] = []
    current: list[tuple[dict[str, Any], float]] = []  # excursive samples of the open event
    cooldown: list[float] = []  # consecutive normal samples while an event is open
    current_baseline_ms = 0.0
    current_threshold_ms = 0.0

    def flush() -> None:
        if len(current) >= config.latency_min_run:
            records = [record for record, _latency in current]
            latencies = [latency for _record, latency in current]
            events.append(
                DegradationEvent(
                    kind=LATENCY_EXCURSION,
                    stream=stream.label,
                    start_epoch=stream.send_time(records[0]),
                    end_epoch=stream.send_time(records[-1]),
                    count=len(records),
                    seq_start=int(records[0]["seq"]),
                    seq_end=int(records[-1]["seq"]),
                    details={
                        "baseline_ms": round(current_baseline_ms, 3),
                        "threshold_ms": round(current_threshold_ms, 3),
                        "peak_ms": round(max(latencies), 3),
                        "mean_ms": round(sum(latencies) / len(latencies), 3),
                    },
                )
            )
        current.clear()
        cooldown.clear()

    for record in stream.records:
        if record.get("status") == "lost":
            continue
        latency = _section_ms(record, "ota_hop_ms", "ota_hop_uncorrected_ms")
        if latency is None:
            continue
        if len(baseline) >= config.latency_baseline_min:
            base = _percentile(list(baseline), 0.50) or 0.0
            threshold = max(config.latency_ratio * base, base + config.latency_min_delta_ms)
            if latency >= threshold:
                if not current:
                    current_baseline_ms, current_threshold_ms = base, threshold
                current.append((record, latency))
                cooldown.clear()
                continue  # frozen: excursive samples never enter the baseline
            if current:
                cooldown.append(latency)
                if len(cooldown) >= config.latency_min_run:
                    closing = list(cooldown)
                    flush()
                    baseline.extend(closing)  # the closing normals are the new normal
                continue  # a shorter dip stays interior to the open event
        baseline.append(latency)
    flush()
    return events


def detect_rate_collapses(
    stream: Stream,
    bins: list[dict[str, Any]],
    config: DetectionConfig,
    *,
    offered_hz: list[float] | None = None,
) -> list[DegradationEvent]:
    """Runs of >= ``rate_collapse_min_bins`` interior bins delivering far below nominal.

    Catches stalls whose sequence gaps have not materialized yet (a gap only
    becomes lost rows when a later message arrives) and throttled-but-delivering
    regimes. Needs a nominal rate; the first and last bin of a stream are never
    judged (they are partial by construction).

    ``offered_hz`` — one rate per entry of ``bins``, from the sending peer's own
    records (``build_offered_timelines``) — names each event's culprit as
    ``cause``: ``source`` when every collapsed bin was offered below the same
    threshold (the collapse is fully explained by the source pausing or slowing;
    the link delivered what it was given), ``link`` when at least one collapsed
    bin was offered at/above threshold and the link still delivered below it,
    and ``unknown`` without sender-side records. The receiver alone cannot tell
    these apart: on the 2026-08-17 cellular drive, 0 Hz "collapses" on the planner
    visualization were the source pausing during manual interventions, not the
    link.
    """
    nominal_hz = stream.nominal_hz
    if nominal_hz is None or len(bins) < 3:
        return []
    threshold_hz = config.rate_collapse_fraction * nominal_hz
    events: list[DegradationEvent] = []
    run: list[tuple[int, dict[str, Any]]] = []

    def cause_details() -> dict[str, Any]:
        if offered_hz is None or len(offered_hz) != len(bins):
            return {"cause": "unknown"}
        rates = [float(offered_hz[index]) for index, _entry in run]
        return {
            "cause": "source" if max(rates) < threshold_hz else "link",
            "min_offered_hz": round(min(rates), 3),
            "max_offered_hz": round(max(rates), 3),
        }

    def flush() -> None:
        if len(run) >= config.rate_collapse_min_bins:
            entries = [entry for _index, entry in run]
            events.append(
                DegradationEvent(
                    kind=RATE_COLLAPSE,
                    stream=stream.label,
                    start_epoch=float(entries[0]["bin_start_epoch"]),
                    end_epoch=float(entries[-1]["bin_start_epoch"]) + config.bin_s,
                    count=len(entries),
                    details={
                        "nominal_hz": round(nominal_hz, 3),
                        "min_delivered_hz": min(float(entry["delivered_hz"]) for entry in entries),
                        "threshold_hz": round(threshold_hz, 3),
                        **cause_details(),
                    },
                )
            )
        run.clear()

    for index, entry in enumerate(bins[1:-1], start=1):
        if float(entry["delivered_hz"]) < threshold_hz:
            run.append((index, entry))
        else:
            flush()
    flush()
    return events


def detect_events(
    stream: Stream,
    bins: list[dict[str, Any]],
    config: DetectionConfig,
    *,
    offered_hz: list[float] | None = None,
) -> list[DegradationEvent]:
    events = [
        *detect_loss_bursts(stream, config),
        *detect_latency_excursions(stream, config),
        *detect_rate_collapses(stream, bins, config, offered_hz=offered_hz),
    ]
    events.sort(key=lambda event: (event.start_epoch, event.end_epoch, event.kind, event.stream))
    return events


# --------------------------------------------------------------------------- #
# Incidents — events grouped by time overlap, then joined with context
# --------------------------------------------------------------------------- #


@dataclass
class Incident:
    index: int
    start_epoch: float
    end_epoch: float
    events: list[DegradationEvent]
    context: dict[str, Any] = field(default_factory=dict)


def group_incidents(events: list[DegradationEvent], *, merge_gap_s: float) -> list[Incident]:
    """Cluster events (any stream, any kind) whose windows touch within ``merge_gap_s``."""
    incidents: list[Incident] = []
    for event in sorted(events, key=lambda event: (event.start_epoch, event.end_epoch)):
        if incidents and event.start_epoch <= incidents[-1].end_epoch + merge_gap_s:
            incident = incidents[-1]
            incident.events.append(event)
            incident.end_epoch = max(incident.end_epoch, event.end_epoch)
        else:
            incidents.append(
                Incident(
                    index=len(incidents) + 1, start_epoch=event.start_epoch, end_epoch=event.end_epoch, events=[event]
                )
            )
    return incidents


def _summarize_link_rows(rows: list[dict[str, Any]]) -> dict[str, Any]:
    def _pick(path: tuple[str, ...]) -> list[float]:
        values: list[float] = []
        for row in rows:
            node: Any = row
            for key in path:
                node = node.get(key) if isinstance(node, dict) else None
            if isinstance(node, (int, float)):
                values.append(float(node))
        return values

    rtt = _pick(("peer_probe", "rtt_ms"))
    loss = _pick(("peer_probe", "loss_pct"))
    rx = _pick(("passive_counter_delta", "rx", "observed_kbps"))
    tx = _pick(("passive_counter_delta", "tx", "observed_kbps"))
    modem = next(
        (
            row["modem"]["metrics"]
            for row in reversed(rows)
            if isinstance(row.get("modem"), dict) and isinstance(row["modem"].get("metrics"), dict)
        ),
        None,
    )
    span = lambda values: {"min": round(min(values), 3), "max": round(max(values), 3)} if values else None  # noqa: E731
    return {
        "available": True,
        "samples": len(rows),
        "peers": sorted({str(row.get("peer") or "") for row in rows}),
        "rtt_ms": span(rtt),
        "probe_loss_pct": span(loss),
        "observed_rx_kbps": span(rx),
        "observed_tx_kbps": span(tx),
        "modem_metrics": modem,
    }


def _profile_steps(profile: Any) -> list[dict[str, Any]] | None:
    """Flatten a Profile into JSON-able context steps (timeline) or None (static)."""
    if not getattr(profile, "is_timeline", False):
        return None
    steps: list[dict[str, Any]] = []
    clock = 0.0
    for index, segment in enumerate(profile.timeline):
        start_s, end_s = clock, clock + segment.for_s
        steps.append(
            {
                "index": index,
                "start_s": round(start_s, 6),
                "end_s": round(end_s, 6),
                "outage": segment.outage,
                "uplink": _shaping_dict(segment.uplink),
                "downlink": _shaping_dict(segment.downlink),
            }
        )
        clock = end_s
    return steps


def _shaping_dict(shaping: Any) -> dict[str, Any] | None:
    if shaping is None:
        return None
    payload = asdict(shaping)
    return {key: value for key, value in payload.items() if value is not None}


def build_profile_context(
    profile_name: str | None,
    profiles_file: str | Path | None,
    *,
    anchor_epoch: float,
    anchor_provenance: str,
) -> dict[str, Any] | None:
    """Environment context from an network profiles reference profile; ``None`` when no profile given."""
    if not profile_name or profile_name == "none":
        return None
    context: dict[str, Any] = {
        "name": profile_name,
        "profiles_file": str(profiles_file) if profiles_file else None,
        "anchor_epoch": anchor_epoch,
        "anchor_provenance": anchor_provenance,
    }
    if not profiles_file:
        context["error"] = "no profiles file given; profile recorded by name only"
        return context
    try:
        from .network_profiles import load_profiles_file

        profile = load_profiles_file(profiles_file).get(profile_name)
    except Exception as exc:  # degrade gracefully: context stays name-only
        context["error"] = str(exc)
        return context
    if profile is None:
        context["error"] = f"profile {profile_name!r} not found in {profiles_file}"
        return context
    context["kind"] = profile.kind
    context["uplink"] = _shaping_dict(profile.uplink)
    context["downlink"] = _shaping_dict(profile.downlink)
    context["steps"] = _profile_steps(profile)
    return context


def _active_profile_steps(profile_context: dict[str, Any] | None, start: float, end: float) -> Any:
    if not profile_context:
        return None
    steps = profile_context.get("steps")
    if steps is None:
        return "static (constant for the whole run)" if "error" not in profile_context else None
    anchor = float(profile_context["anchor_epoch"])
    active = [step for step in steps if anchor + float(step["start_s"]) < end and anchor + float(step["end_s"]) > start]
    return active


def build_incident_context(
    incident: Incident,
    *,
    streams: list[Stream],
    link_rows: list[dict[str, Any]],
    transitions: list[dict[str, Any]],
    profile_context: dict[str, Any] | None,
    config: DetectionConfig,
) -> dict[str, Any]:
    margin = config.margin_s
    start, end = incident.start_epoch - margin, incident.end_epoch + margin

    overlapping_trace = [row for row in link_rows if start <= row["_epoch"] <= end]
    link = (
        _summarize_link_rows(overlapping_trace)
        if overlapping_trace
        else {
            "available": False,
            "reason": "no link_trace samples in window" if link_rows else "no link_trace.jsonl recorded",
        }
    )

    overlapping_transitions = [
        {
            "at": row.get("at"),
            "peer": row.get("peer"),
            "topic": row.get("topic"),
            "direction": row.get("direction"),
            "to": (row.get("to") or {}).get("overall") if isinstance(row.get("to"), dict) else None,
            "diagnosis": row.get("diagnosis"),
        }
        for row in transitions
        if start <= row["_epoch"] <= end
    ]

    messages = 0
    payload_bytes = 0.0
    keyframes: list[dict[str, Any]] = []
    largest: dict[str, Any] | None = None
    for stream in streams:
        for record in stream.records:
            if record.get("status") == "lost":
                continue
            send_t = stream.send_time(record)
            if not start <= send_t <= end:
                continue
            messages += 1
            size = record.get("size_bytes")
            if size is not None:
                payload_bytes += float(size)
                if largest is None or float(size) > largest["size_bytes"]:
                    largest = {"stream": stream.label, "seq": int(record["seq"]), "size_bytes": float(size)}
            if int(record["seq"]) in stream.keyframe_seqs:
                keyframes.append(
                    {
                        "stream": stream.label,
                        "seq": int(record["seq"]),
                        "size_bytes": float(size) if size is not None else None,
                        "at_epoch": send_t,
                    }
                )

    keyframe_coincident = False
    for event in incident.events:
        event_stream = next((entry for entry in streams if entry.label == event.stream), None)
        period = (event_stream.period_s or 0.0) if event_stream else 0.0
        for keyframe in keyframes:
            if keyframe["stream"] == event.stream and 0.0 <= event.start_epoch - keyframe["at_epoch"] <= max(
                period, config.bin_s
            ):
                keyframe_coincident = True

    return {
        "window": {"start_epoch": start, "end_epoch": end, "margin_s": margin},
        "link_trace": link,
        # The margin exists for sparse samples; profile steps are continuous, so
        # they join on the exact incident window.
        "profile": {
            "name": profile_context.get("name"),
            "active_steps": _active_profile_steps(profile_context, incident.start_epoch, incident.end_epoch),
        }
        if profile_context
        else None,
        "state_transitions": overlapping_transitions[:10],
        "state_transitions_truncated": max(0, len(overlapping_transitions) - 10),
        "traffic": {
            "messages": messages,
            "payload_bytes": round(payload_bytes, 1),
            "largest_message": largest,
            "keyframes": len(keyframes),
            "keyframe_details": keyframes[:10],
            "keyframe_coincident_event_start": keyframe_coincident,
        },
        "caveat": CAVEAT,
    }


# --------------------------------------------------------------------------- #
# Report assembly
# --------------------------------------------------------------------------- #


def _git_sha() -> str | None:
    try:
        result = subprocess.run(
            ["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return result.stdout.strip() if result.returncode == 0 and result.stdout.strip() else None


def _manifest_provenance(manifest_path: Path | None) -> dict[str, Any] | None:
    if manifest_path is None:
        return None
    try:
        import yaml

        loaded = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    except Exception:
        return None
    if not isinstance(loaded, dict):
        return None
    return {
        "instance_id": loaded.get("instance_id"),
        "effective_config_sha256": loaded.get("effective_config_sha256"),
        "run_rosotacom_version": loaded.get("rosotacom_version"),
        "created_at": loaded.get("created_at"),
    }


def _event_dict(event: DegradationEvent, *, run_start: float, incident_index: int | None) -> dict[str, Any]:
    return {
        "kind": event.kind,
        "stream": event.stream,
        "start_epoch": event.start_epoch,
        "end_epoch": event.end_epoch,
        "start_s": round(event.start_epoch - run_start, 3),
        "end_s": round(event.end_epoch - run_start, 3),
        "duration_s": round(event.end_epoch - event.start_epoch, 3),
        "seq_start": event.seq_start,
        "seq_end": event.seq_end,
        "count": event.count,
        "incident": incident_index,
        "details": event.details,
    }


def parse_timeline_anchor(raw: str | None) -> float | None:
    """``--timeline-anchor``: epoch seconds or an ISO timestamp (naive = local time)."""
    if raw is None:
        return None
    try:
        return float(raw)
    except ValueError:
        epoch = _parse_wall_iso(raw)
        if epoch is None:
            raise RuntimeError(f"invalid --timeline-anchor {raw!r}: use epoch seconds or an ISO timestamp") from None
        return epoch


def build_report(
    instance_dir: str | Path,
    *,
    config: DetectionConfig | None = None,
    peers: tuple[str, ...] = (),
    profile: str | None = None,
    profiles_file: str | Path | None = None,
    timeline_anchor: float | None = None,
    argv: list[str] | None = None,
) -> dict[str, Any]:
    """Analyze one session instance and return the full ``report.json`` payload."""
    config = config or DetectionConfig()
    inputs = discover_inputs(instance_dir, peers=peers)
    records = load_transit_records(inputs.events_paths)
    # The sending peer's own rows (direction=outbound, stage com_out) carry the
    # offered rate. They must stay out of the delivered view: the cross-peer
    # join keys on (source, topic, epoch, seq), so a sender row would upgrade
    # the receiver's lost row to delivered and flatten delivered_hz to the
    # offered rate.
    offered_records = [record for record in records if record.get("direction") == "outbound"]
    delivered_records = [record for record in records if record.get("direction") != "outbound"]
    declared = load_declared_ffmpeg_topics(inputs.status_paths)
    streams = build_streams(delivered_records, declared_ffmpeg_topics=declared)

    send_spans = [(stream.send_time(stream.records[0]), stream.send_time(stream.records[-1])) for stream in streams]
    run_start = min((span[0] for span in send_spans), default=0.0)
    run_end = max((span[1] for span in send_spans), default=0.0)

    anchor_epoch = timeline_anchor if timeline_anchor is not None else run_start
    anchor_provenance = (
        "explicit --timeline-anchor" if timeline_anchor is not None else "first observed publish (approximate)"
    )
    profile_context = build_profile_context(
        profile, profiles_file, anchor_epoch=anchor_epoch, anchor_provenance=anchor_provenance
    )

    timelines = {stream.label: build_stream_bins(stream, run_start=run_start, bin_s=config.bin_s) for stream in streams}
    offered = build_offered_timelines(offered_records, streams, timelines, run_start=run_start, bin_s=config.bin_s)
    events: list[DegradationEvent] = []
    for stream in streams:
        events.extend(detect_events(stream, timelines[stream.label], config, offered_hz=offered.get(stream.label)))
    events.sort(key=lambda event: (event.start_epoch, event.end_epoch, event.kind, event.stream))

    incidents = group_incidents(events, merge_gap_s=config.merge_gap_s)
    link_rows = load_link_trace_rows(inputs.link_trace_paths)
    transitions = load_state_transitions(inputs.events_paths)
    for incident in incidents:
        incident.context = build_incident_context(
            incident,
            streams=streams,
            link_rows=link_rows,
            transitions=transitions,
            profile_context=profile_context,
            config=config,
        )

    incident_of_event = {id(event): incident.index for incident in incidents for event in incident.events}

    from .transit import summarize_transit_records

    summary = summarize_transit_records(delivered_records)["topics"]
    stream_entries: dict[str, Any] = {}
    for stream in streams:
        entry = dict(summary.get(stream.label, {}))
        entry.update(
            {
                "source": stream.source,
                "target": stream.target,
                "topic": stream.topic,
                "nominal_hz": round(stream.nominal_hz, 3) if stream.nominal_hz else None,
                "nominal_period_ms": round(stream.period_s * 1000.0, 3) if stream.period_s else None,
                "keyframes": {
                    "count": len(stream.keyframe_seqs),
                    "provenance": stream.keyframe_provenance,
                }
                if stream.keyframe_provenance
                else None,
                "events": sum(1 for event in events if event.stream == stream.label),
            }
        )
        stream_entries[stream.label] = entry

    total = sum(len(stream.records) for stream in streams)
    lost = sum(1 for stream in streams for record in stream.records if record.get("status") == "lost")
    reordered = sum(1 for stream in streams for record in stream.records if record.get("status") == "reordered")

    return {
        "schema_version": 1,
        "kind": "forensics_report",
        "provenance": {
            "command": shlex.join(argv if argv is not None else sys.argv),
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "rosotacom_version": __version__,
            "git_sha": _git_sha(),
            "instance": {
                "path": str(inputs.instance_dir),
                **(_manifest_provenance(inputs.manifest_path) or {}),
            },
            "inputs": {
                "peers": list(inputs.peers),
                "events": [str(path) for path in inputs.events_paths],
                "status": [str(path) for path in inputs.status_paths],
                "link_trace": [str(path) for path in inputs.link_trace_paths],
                "manifest": str(inputs.manifest_path) if inputs.manifest_path else None,
            },
            "detection_config": asdict(config),
            "profile": profile_context,
            "caveat": CAVEAT,
        },
        "run": {
            "start_epoch": run_start,
            "end_epoch": run_end,
            "start_iso": datetime.fromtimestamp(run_start).isoformat(timespec="seconds") if streams else None,
            "duration_s": round(run_end - run_start, 3),
            "streams": len(streams),
            "records": total,
            "lost": lost,
            "reordered": reordered,
        },
        "streams": stream_entries,
        "timelines": timelines,
        "events": [
            _event_dict(event, run_start=run_start, incident_index=incident_of_event.get(id(event))) for event in events
        ],
        "incidents": [
            {
                "index": incident.index,
                "start_epoch": incident.start_epoch,
                "end_epoch": incident.end_epoch,
                "start_s": round(incident.start_epoch - run_start, 3),
                "end_s": round(incident.end_epoch - run_start, 3),
                "events": [
                    _event_dict(event, run_start=run_start, incident_index=incident.index) for event in incident.events
                ],
                "context": incident.context,
            }
            for incident in incidents
        ],
    }


# --------------------------------------------------------------------------- #
# Markdown rendering
# --------------------------------------------------------------------------- #


def _format_ms(value: Any) -> str:
    return f"{float(value):.1f}ms" if value is not None else "n/a"


def _event_line(event: dict[str, Any]) -> str:
    window = f"t=+{event['start_s']:.1f}s..+{event['end_s']:.1f}s"
    details = event.get("details") or {}
    if event["kind"] == LOSS_BURST:
        span = f"seq {event['seq_start']}-{event['seq_end']}"
        return f"**loss burst** on `{event['stream']}` — {event['count']} consecutive lost ({span}), {window}"
    if event["kind"] == LATENCY_EXCURSION:
        return (
            f"**latency excursion** on `{event['stream']}` — {event['count']} messages "
            f"(seq {event['seq_start']}-{event['seq_end']}), peak {_format_ms(details.get('peak_ms'))} vs "
            f"baseline {_format_ms(details.get('baseline_ms'))} (threshold {_format_ms(details.get('threshold_ms'))}), "
            f"{window}"
        )
    cause = details.get("cause")
    if cause == "source":
        name = (
            "rate collapse (source paused)" if details.get("max_offered_hz") == 0.0 else "rate collapse (source slowed)"
        )
    elif cause == "link":
        name = "rate collapse (link)"
    else:
        name = "rate collapse"
    low, high = details.get("min_offered_hz"), details.get("max_offered_hz")
    offered = "" if low is None else (f", offered {low}Hz" if low == high else f", offered {low}-{high}Hz")
    suffix = "" if cause in ("source", "link") else "; cause unknown (no sender-side records)"
    return (
        f"**{name}** on `{event['stream']}` — {event['count']} bins, delivered rate down to "
        f"{details.get('min_delivered_hz')}Hz vs nominal {details.get('nominal_hz')}Hz{offered}, {window}{suffix}"
    )


def _context_lines(context: dict[str, Any]) -> list[str]:
    lines: list[str] = []
    link = context.get("link_trace") or {}
    if link.get("available"):
        rx, tx, rtt = link.get("observed_rx_kbps"), link.get("observed_tx_kbps"), link.get("rtt_ms")
        parts = [f"{link['samples']} link-trace sample(s)"]
        if rx:
            parts.append(f"rx {rx['min']}-{rx['max']} kbps")
        if tx:
            parts.append(f"tx {tx['min']}-{tx['max']} kbps")
        if rtt:
            parts.append(f"rtt {rtt['min']}-{rtt['max']} ms")
        if link.get("modem_metrics"):
            parts.append(f"modem {json.dumps(link['modem_metrics'])}")
        lines.append("link: " + ", ".join(parts))
    else:
        lines.append(f"link: unavailable ({link.get('reason', 'n/a')})")

    profile = context.get("profile")
    if profile:
        active = profile.get("active_steps")
        if isinstance(active, list):
            described = [
                f"step {step['index']} ({step['start_s']:.0f}-{step['end_s']:.0f}s"
                + (f", outage={step['outage']}" if step.get("outage") else "")
                + ")"
                for step in active
            ]
            lines.append(f"profile `{profile['name']}`: " + (", ".join(described) or "no step overlaps"))
        elif active:
            lines.append(f"profile `{profile['name']}`: {active}")

    transitions = context.get("state_transitions") or []
    for transition in transitions[:3]:
        lines.append(
            f"pipeline: {transition.get('peer')}/{transition.get('topic')} -> {transition.get('to')} "
            f"({transition.get('diagnosis')})"
        )
    if len(transitions) > 3 or context.get("state_transitions_truncated"):
        lines.append(f"pipeline: ... {len(transitions) + context.get('state_transitions_truncated', 0) - 3} more")

    traffic = context.get("traffic") or {}
    kilobytes = float(traffic.get("payload_bytes") or 0.0) / 1000.0
    parts = [f"{traffic.get('messages', 0)} msgs / {kilobytes:.1f} kB in window"]
    if traffic.get("keyframes"):
        parts.append(f"{traffic['keyframes']} keyframe(s)")
        if traffic.get("keyframe_coincident_event_start"):
            parts.append("event start coincides with a keyframe")
    lines.append("traffic: " + ", ".join(parts))
    return lines


def render_markdown(report: dict[str, Any]) -> str:
    provenance = report["provenance"]
    run = report["run"]
    lines = [
        "# Degradation forensics report",
        "",
        f"- instance: `{provenance['instance']['path']}`",
        f"- generated: {provenance['generated_at']} by rosotacom {provenance['rosotacom_version']}"
        + (f" (`{provenance['git_sha'][:12]}`)" if provenance.get("git_sha") else ""),
        f"- command: `{provenance['command']}`",
    ]
    profile = provenance.get("profile")
    if profile:
        lines.append(
            f"- profile: `{profile['name']}`"
            + (f" ({profile.get('kind')})" if profile.get("kind") else "")
            + (f" — {profile['error']}" if profile.get("error") else "")
        )
    if run["streams"]:
        lines.append(
            f"- run: {run['start_iso']} +{run['duration_s']:.1f}s, {run['streams']} stream(s), "
            f"{run['records']} transit records ({run['lost']} lost, {run['reordered']} reordered)"
        )
    else:
        lines.append("- run: no transit records found (wrapped topics + status overview required)")
    lines += ["", f"> {provenance['caveat']}", "", "## Streams", ""]
    for label, entry in report["streams"].items():
        nominal = f"{entry['nominal_hz']}Hz nominal, " if entry.get("nominal_hz") else ""
        ota = entry.get("ota_hop_ms") or {}
        keyframes = entry.get("keyframes")
        keyframe_note = f", {keyframes['count']} keyframes" if keyframes else ""
        lines.append(
            f"- `{label}` — {nominal}{entry.get('delivered', 0)}/{entry.get('expected', 0)} delivered "
            f"({entry.get('loss_pct', 0.0)}% lost), ota p50 {_format_ms(ota.get('p50'))} "
            f"p95 {_format_ms(ota.get('p95'))}{keyframe_note}, {entry.get('events', 0)} event(s)"
        )

    incidents = report["incidents"]
    lines += ["", f"## Incidents ({len(incidents)})", ""]
    if not incidents:
        lines.append("No degradation events detected with the configured thresholds.")
    for incident in incidents:
        start_iso = datetime.fromtimestamp(incident["start_epoch"]).strftime("%H:%M:%S")
        end_iso = datetime.fromtimestamp(incident["end_epoch"]).strftime("%H:%M:%S")
        lines.append(
            f"### {incident['index']}. t=+{incident['start_s']:.1f}s..+{incident['end_s']:.1f}s "
            f"({start_iso}-{end_iso}) — {len(incident['events'])} event(s)"
        )
        lines.append("")
        for event in incident["events"]:
            lines.append(f"- {_event_line(event)}")
        for context_line in _context_lines(incident.get("context") or {}):
            lines.append(f"- context — {context_line}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


# --------------------------------------------------------------------------- #
# Output writing (report.json, report.md, figures)
# --------------------------------------------------------------------------- #


def _slug(label: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", label).strip("_") or "stream"


def write_figures(report: dict[str, Any], out_dir: Path) -> list[Path]:
    """Render one timeline figure per stream (requires the ``[plots]`` extra)."""
    from .plots import plot_forensics_stream

    run_start = float(report["run"]["start_epoch"])
    profile = report["provenance"].get("profile") or {}
    steps = profile.get("steps")
    rel_steps = None
    if steps:
        anchor = float(profile["anchor_epoch"]) - run_start
        rel_steps = [
            {
                "start_s": anchor + float(step["start_s"]),
                "end_s": anchor + float(step["end_s"]),
                "label": f"step {step['index']}" + (f" {step['outage']}" if step.get("outage") else ""),
            }
            for step in steps
        ]

    figures_dir = out_dir / "figures"
    figures_dir.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for label, bins in report["timelines"].items():
        if not bins:
            continue
        events = [event for event in report["events"] if event["stream"] == label]
        stream_entry = report["streams"].get(label) or {}
        out = figures_dir / f"{_slug(label)}.png"
        plot_forensics_stream(
            bins,
            events,
            out=out,
            title=f"Forensics timeline — {label}",
            nominal_hz=stream_entry.get("nominal_hz"),
            timeline_steps=rel_steps,
        )
        written.append(out)
    return written


def write_report(
    report: dict[str, Any],
    out_dir: str | Path,
    *,
    figures: bool = True,
) -> dict[str, Any]:
    """Write report.json + report.md (+ figures when matplotlib is available)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    json_path = out / "report.json"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    md_path = out / "report.md"
    md_path.write_text(render_markdown(report), encoding="utf-8")
    result: dict[str, Any] = {"json": json_path, "markdown": md_path, "figures": [], "figures_note": None}
    if figures:
        try:
            result["figures"] = write_figures(report, out)
        except ImportError:
            result["figures_note"] = "figures skipped: matplotlib missing (pip install rosotacom[plots])"
    else:
        result["figures_note"] = "figures disabled (--no-figures)"
    return result
