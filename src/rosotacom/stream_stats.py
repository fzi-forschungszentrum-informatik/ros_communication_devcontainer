# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Offline stream-stage statistics for bags and transit rows."""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from statistics import median
from typing import Any

import yaml

from . import __version__
from .ffmpeg_packet import FFMPEGPacketInfo, keyframes_by_size, parse_ffmpeg_packet
from .transit import join_transit_records, load_transit_records

FFMPEG_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"
KEYFRAME_SHARE_MIN = 0.02
KEYFRAME_SHARE_MAX = 0.40


class McapUnavailableError(RuntimeError):
    pass


@dataclass(frozen=True)
class SourceSpec:
    kind: str
    label: str
    path: Path
    topic: str


@dataclass(frozen=True)
class StreamSample:
    timestamp_ns: int
    size_bytes: int
    serialized_size_bytes: int | None = None
    keyframe: bool | None = None
    ffmpeg: FFMPEGPacketInfo | None = None


@dataclass(frozen=True)
class StreamSource:
    label: str
    kind: str
    path: Path | None
    topic: str
    samples: tuple[StreamSample, ...]
    message_type: str | None = None
    size_basis: str = "message_bytes"


def parse_source_spec(kind: str, value: str) -> SourceSpec:
    """Parse ``LABEL=PATH:/topic`` source arguments."""
    label: str | None
    rest: str
    if "=" in value:
        label, rest = value.split("=", 1)
        label = label.strip()
        if not label:
            raise ValueError(f"{kind} source has an empty label: {value!r}")
    else:
        label, rest = None, value
    if ":" not in rest:
        raise ValueError(f"{kind} source must be LABEL=PATH:/topic, got {value!r}")
    path_text, topic = rest.rsplit(":", 1)
    if not path_text:
        raise ValueError(f"{kind} source has an empty path: {value!r}")
    if not topic.startswith("/"):
        raise ValueError(f"{kind} source topic must start with '/', got {topic!r}")
    path = Path(path_text).expanduser()
    return SourceSpec(kind=kind, label=label or f"{path.name}:{topic}", path=path, topic=topic)


def _round(value: float | None, digits: int = 3) -> float | None:
    if value is None:
        return None
    rounded = round(float(value), digits)
    if rounded == 0.0 and value > 0.0:
        return float(f"{value:.3g}")
    return rounded


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    rank = max(0, min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1))
    return _round(ordered[rank])


def distribution(values: Sequence[float]) -> dict[str, float | None]:
    if not values:
        return dict.fromkeys(("min", "mean", "median", "p90", "max", "std"))
    vals = [float(value) for value in values]
    mean = sum(vals) / len(vals)
    std = math.sqrt(sum((value - mean) ** 2 for value in vals) / len(vals))
    return {
        "min": _round(min(vals)),
        "mean": _round(mean),
        "median": _round(float(median(vals))),
        "p90": _percentile(vals, 0.90),
        "max": _round(max(vals)),
        "std": _round(std),
    }


def _ordered_samples(samples: Iterable[StreamSample]) -> list[StreamSample]:
    return sorted(samples, key=lambda sample: sample.timestamp_ns)


def _intervals_s(samples: Sequence[StreamSample]) -> list[float]:
    ordered = _ordered_samples(samples)
    return [
        (right.timestamp_ns - left.timestamp_ns) / 1_000_000_000.0
        for left, right in zip(ordered, ordered[1:], strict=False)
        if right.timestamp_ns >= left.timestamp_ns
    ]


def interval_stats(samples: Sequence[StreamSample]) -> dict[str, Any]:
    intervals = _intervals_s(samples)
    stats: dict[str, Any] = distribution([gap * 1000.0 for gap in intervals])
    nominal_s = float(median(intervals)) if intervals else None
    stats["nominal_ms"] = _round(nominal_s * 1000.0 if nominal_s is not None else None)
    stats["nominal_hz"] = _round(1.0 / nominal_s if nominal_s and nominal_s > 0.0 else None)
    if intervals and nominal_s and nominal_s > 0.0:
        within = sum(abs(gap - nominal_s) <= 0.10 * nominal_s for gap in intervals)
        stats["within_10pct_nominal_pct"] = _round(100.0 * within / len(intervals))
    else:
        stats["within_10pct_nominal_pct"] = None
    return stats


def _rate_hz(samples: Sequence[StreamSample]) -> tuple[float | None, float | None]:
    ordered = _ordered_samples(samples)
    if len(ordered) < 2:
        return None, None
    duration_s = (ordered[-1].timestamp_ns - ordered[0].timestamp_ns) / 1_000_000_000.0
    if duration_s <= 0.0:
        return None, _round(duration_s)
    return _round((len(ordered) - 1) / duration_s), _round(duration_s)


def samples_from_cdr_records(
    records: Iterable[tuple[int, bytes]],
    *,
    message_type: str | None,
) -> tuple[StreamSample, ...]:
    """Build samples from ``(timestamp_ns, serialized_message)`` records."""
    parse_ffmpeg = message_type == FFMPEG_TYPE or (message_type is not None and "FFMPEGPacket" in message_type)
    samples: list[StreamSample] = []
    for timestamp_ns, data in records:
        serialized = bytes(data)
        if parse_ffmpeg:
            info = parse_ffmpeg_packet(serialized)
            samples.append(
                StreamSample(
                    timestamp_ns=int(timestamp_ns),
                    size_bytes=info.payload_size,
                    serialized_size_bytes=len(serialized),
                    keyframe=info.is_keyframe,
                    ffmpeg=info,
                )
            )
        else:
            samples.append(
                StreamSample(
                    timestamp_ns=int(timestamp_ns),
                    size_bytes=len(serialized),
                    serialized_size_bytes=len(serialized),
                )
            )
    return tuple(samples)


def _metadata_path(path: Path) -> Path | None:
    if path.is_dir() and (path / "metadata.yaml").is_file():
        return path / "metadata.yaml"
    if path.is_file() and path.name == "metadata.yaml":
        return path
    return None


def _load_bag_metadata(path: Path) -> dict[str, Any] | None:
    metadata = _metadata_path(path)
    if metadata is None:
        return None
    loaded = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"bag metadata must be a mapping: {metadata}")
    info = loaded.get("rosbag2_bagfile_information") or {}
    if not isinstance(info, dict):
        raise ValueError(f"bag metadata missing rosbag2_bagfile_information: {metadata}")
    return loaded


def _bag_info(metadata: dict[str, Any]) -> dict[str, Any]:
    info = metadata.get("rosbag2_bagfile_information") or {}
    if not isinstance(info, dict):
        raise ValueError("bag metadata missing rosbag2_bagfile_information")
    return info


def _bag_base_dir(path: Path) -> Path:
    metadata = _metadata_path(path)
    if metadata is None:
        raise ValueError(f"not a rosbag2 directory or metadata.yaml: {path}")
    return metadata.parent


def _bag_file_paths(path: Path, metadata: dict[str, Any], suffix: str) -> list[Path]:
    base = _bag_base_dir(path)
    info = _bag_info(metadata)
    raw_paths = info.get("relative_file_paths") or []
    if isinstance(raw_paths, list) and raw_paths:
        files = [base / str(raw) for raw in raw_paths]
    else:
        files = sorted(base.glob(f"*{suffix}"))
    if not files:
        raise ValueError(f"bag has no {suffix} storage files: {base}")
    return files


def _topic_types_from_metadata(metadata: dict[str, Any]) -> dict[str, str]:
    topics = _bag_info(metadata).get("topics_with_message_count") or []
    result: dict[str, str] = {}
    if not isinstance(topics, list):
        return result
    for entry in topics:
        if not isinstance(entry, dict):
            continue
        topic_metadata = entry.get("topic_metadata") or {}
        if not isinstance(topic_metadata, dict):
            continue
        name = topic_metadata.get("name")
        msg_type = topic_metadata.get("type")
        if name is not None and msg_type is not None:
            result[str(name)] = str(msg_type)
    return result


def _mcap_paths(path: Path, metadata: dict[str, Any] | None) -> list[Path]:
    if metadata is not None:
        return _bag_file_paths(path, metadata, ".mcap")
    if path.is_file() and path.suffix == ".mcap":
        return [path]
    raise ValueError(f"not an mcap rosbag2 directory, metadata.yaml, or .mcap file: {path}")


def _load_mcap_bag_source(spec: SourceSpec) -> StreamSource:
    try:
        from mcap.reader import make_reader
    except ImportError as exc:
        raise McapUnavailableError(
            "mcap bag sources require the Python package `mcap`; reinstall rosotacom or install mcap"
        ) from exc

    metadata = _load_bag_metadata(spec.path)
    message_type = None
    if metadata is not None:
        topic_types = _topic_types_from_metadata(metadata)
        if spec.topic not in topic_types:
            available = ", ".join(sorted(topic_types)) or "none"
            raise RuntimeError(f"{spec.path}: topic {spec.topic!r} not found; available topics: {available}")
        message_type = topic_types[spec.topic]

    records: list[tuple[int, bytes]] = []
    for mcap_path in _mcap_paths(spec.path, metadata):
        with mcap_path.open("rb") as fp:
            reader = make_reader(fp)
            for schema, channel, message in reader.iter_messages(topics=[spec.topic]):
                if channel.topic != spec.topic:
                    continue
                if message_type is None and schema is not None:
                    schema_name = getattr(schema, "name", None)
                    message_type = str(schema_name) if schema_name is not None else None
                records.append((int(message.log_time), bytes(message.data)))
    if not records:
        raise RuntimeError(f"{spec.path}: no messages found for topic {spec.topic!r}")
    records.sort(key=lambda row: row[0])
    samples = samples_from_cdr_records(records, message_type=message_type)
    size_basis = (
        "ffmpeg_payload_bytes" if message_type and "FFMPEGPacket" in message_type else "serialized_message_bytes"
    )
    return StreamSource(
        label=spec.label,
        kind="bag",
        path=spec.path,
        topic=spec.topic,
        samples=samples,
        message_type=message_type,
        size_basis=size_basis,
    )


def load_bag_source(spec: SourceSpec, *, storage_id: str = "mcap") -> StreamSource:
    """Load one topic from a rosbag2 bag."""
    if storage_id == "mcap":
        try:
            return _load_mcap_bag_source(spec)
        except McapUnavailableError:
            pass

    try:
        import rosbag2_py  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError(
            "bag sources require the Python package `mcap` for MCAP bags or rosbag2_py in a ROS environment. "
            "Use --events for events.jsonl sources."
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(spec.path), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if spec.topic not in topics:
        available = ", ".join(sorted(topics)) or "none"
        raise RuntimeError(f"{spec.path}: topic {spec.topic!r} not found; available topics: {available}")
    message_type = topics.get(spec.topic)
    records: list[tuple[int, bytes]] = []
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic == spec.topic:
            records.append((int(timestamp_ns), bytes(serialized)))
    samples = samples_from_cdr_records(records, message_type=message_type)
    size_basis = (
        "ffmpeg_payload_bytes" if message_type and "FFMPEGPacket" in message_type else "serialized_message_bytes"
    )
    return StreamSource(
        label=spec.label,
        kind="bag",
        path=spec.path,
        topic=spec.topic,
        samples=samples,
        message_type=message_type,
        size_basis=size_basis,
    )


def load_events_source(spec: SourceSpec) -> StreamSource:
    records = [
        record
        for record in join_transit_records(load_transit_records([spec.path]))
        if str(record.get("topic") or "") == spec.topic and record.get("status") != "lost"
    ]
    samples: list[StreamSample] = []
    for record in records:
        size = record.get("size_bytes")
        if size is None:
            continue
        stamp = record.get("t_com_in")
        if stamp is None:
            stamp = record.get("t_wrap")
        if stamp is None:
            continue
        keyframe = record.get("keyframe")
        samples.append(
            StreamSample(
                timestamp_ns=int(float(stamp) * 1_000_000_000),
                size_bytes=int(size),
                keyframe=keyframe if isinstance(keyframe, bool) else None,
            )
        )
    return StreamSource(
        label=spec.label,
        kind="events",
        path=spec.path,
        topic=spec.topic,
        samples=tuple(samples),
        size_basis="events_size_bytes",
    )


def _keyframe_mask(source: StreamSource) -> tuple[list[bool], str] | None:
    if not source.samples:
        return None
    explicit = [sample.keyframe for sample in source.samples]
    if any(value is not None for value in explicit):
        return [bool(value) for value in explicit], "ffmpeg_packet.flags"

    flags = keyframes_by_size([sample.size_bytes for sample in source.samples])
    count = sum(flags)
    if count < 2:
        return None
    share = count / len(flags)
    if not (KEYFRAME_SHARE_MIN <= share <= KEYFRAME_SHARE_MAX):
        return None
    return flags, f"size_bimodality ({share * 100.0:.1f}% frames)"


def _gop_stats(source: StreamSource) -> dict[str, Any] | None:
    mask_with_method = _keyframe_mask(source)
    if mask_with_method is None:
        return None
    flags, method = mask_with_method
    key_indices = [index for index, is_keyframe in enumerate(flags) if is_keyframe]
    if not key_indices:
        return None

    spacings = [right - left for left, right in zip(key_indices, key_indices[1:], strict=False)]
    gops: list[list[StreamSample]] = []
    for current, next_start in zip(key_indices, [*key_indices[1:], len(source.samples)], strict=False):
        gop = list(source.samples[current:next_start])
        if gop:
            gops.append(gop)

    by_position: dict[int, list[float]] = {}
    for gop in gops:
        for position, sample in enumerate(gop):
            by_position.setdefault(position, []).append(float(sample.size_bytes))
    position_table = [
        {"position": position, "count": len(values), **distribution(values)}
        for position, values in sorted(by_position.items())
    ]

    extreme: dict[str, Any] | None = None
    best_ratio = -1.0
    for start_index, gop in zip(key_indices, gops, strict=True):
        if len(gop) < 2:
            continue
        delta_mean = sum(sample.size_bytes for sample in gop[1:]) / (len(gop) - 1)
        if delta_mean <= 0:
            continue
        ratio = gop[0].size_bytes / delta_mean
        if ratio > best_ratio:
            best_ratio = ratio
            extreme = {
                "start_index": start_index,
                "start_time_s": _round((gop[0].timestamp_ns - source.samples[0].timestamp_ns) / 1_000_000_000.0),
                "length": len(gop),
                "keyframe_size_bytes": gop[0].size_bytes,
                "delta_mean_bytes": _round(delta_mean),
                "key_to_delta_mean_ratio": _round(ratio),
                "sizes_bytes": [sample.size_bytes for sample in gop],
            }

    return {
        "method": method,
        "keyframes": len(key_indices),
        "gops": len(gops),
        "keyframe_spacing_frames": distribution([float(value) for value in spacings]),
        "gop_length_frames": distribution([float(len(gop)) for gop in gops]),
        "position_size_bytes": position_table,
        "most_extreme_gop": extreme,
    }


def summarize_source(source: StreamSource) -> dict[str, Any]:
    ordered = _ordered_samples(source.samples)
    rate_hz, duration_s = _rate_hz(ordered)
    sizes = [float(sample.size_bytes) for sample in ordered]
    ordered_source = StreamSource(
        label=source.label,
        kind=source.kind,
        path=source.path,
        topic=source.topic,
        samples=tuple(ordered),
        message_type=source.message_type,
        size_basis=source.size_basis,
    )
    return {
        "label": source.label,
        "kind": source.kind,
        "path": str(source.path) if source.path else None,
        "topic": source.topic,
        "message_type": source.message_type,
        "size_basis": source.size_basis,
        "count": len(ordered),
        "duration_s": duration_s,
        "rate_hz": rate_hz,
        "size_bytes": distribution(sizes),
        "interval_ms": interval_stats(ordered),
        "gop": _gop_stats(ordered_source),
    }


def build_report(sources: Sequence[StreamSource], *, argv: Sequence[str] | None = None) -> dict[str, Any]:
    summaries = [summarize_source(source) for source in sources]
    comparison_rows = [
        {
            "stage": entry["label"],
            "kind": entry["kind"],
            "topic": entry["topic"],
            "count": entry["count"],
            "rate_hz": entry["rate_hz"],
            "size_mean_bytes": entry["size_bytes"]["mean"],
            "size_p90_bytes": entry["size_bytes"]["p90"],
            "size_max_bytes": entry["size_bytes"]["max"],
            "interval_std_ms": entry["interval_ms"]["std"],
            "within_10pct_nominal_pct": entry["interval_ms"]["within_10pct_nominal_pct"],
            "gop_keyframes": (entry["gop"] or {}).get("keyframes"),
            "gop_spacing_median_frames": ((entry["gop"] or {}).get("keyframe_spacing_frames") or {}).get("median"),
        }
        for entry in summaries
    ]
    return {
        "kind": "stream_stats",
        "schema_version": 1,
        "provenance": {
            "generated_at": datetime.now().isoformat(timespec="seconds"),
            "rosotacom_version": __version__,
            "command": " ".join(argv or ()),
        },
        "sources": summaries,
        "comparison": {
            "note": ("Aggregate stage comparison only; no per-message join is attempted across decimating stages."),
            "rows": comparison_rows,
        },
    }


def _fmt(value: Any, *, suffix: str = "") -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:g}{suffix}"
    return f"{value}{suffix}"


def _markdown_table(headers: Sequence[str], rows: Sequence[Sequence[Any]]) -> list[str]:
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join("---" for _ in headers) + " |"]
    for row in rows:
        lines.append("| " + " | ".join(str(cell) for cell in row) + " |")
    return lines


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# Stream stats",
        "",
        f"- generated: {report['provenance']['generated_at']} by rosotacom {report['provenance']['rosotacom_version']}",
    ]
    command = report["provenance"].get("command")
    if command:
        lines.append(f"- command: `{command}`")
    lines += ["", "## Stage Comparison", "", report["comparison"]["note"], ""]
    rows = []
    for row in report["comparison"]["rows"]:
        rows.append(
            [
                f"`{row['stage']}`",
                row["kind"],
                f"`{row['topic']}`",
                row["count"],
                _fmt(row["rate_hz"]),
                _fmt(row["size_mean_bytes"]),
                _fmt(row["size_p90_bytes"]),
                _fmt(row["interval_std_ms"]),
                _fmt(row["within_10pct_nominal_pct"], suffix="%"),
                _fmt(row["gop_spacing_median_frames"]),
            ]
        )
    lines.extend(
        _markdown_table(
            [
                "stage",
                "kind",
                "topic",
                "msgs",
                "hz",
                "mean B",
                "p90 B",
                "gap std ms",
                "gaps within +/-10%",
                "GOP spacing",
            ],
            rows,
        )
    )

    lines += ["", "## Per-Source Details", ""]
    for source in report["sources"]:
        lines.append(f"### {source['label']}")
        lines.append("")
        lines.append(f"- topic: `{source['topic']}`")
        lines.append(f"- size basis: `{source['size_basis']}`")
        lines.append(f"- count/rate: {source['count']} messages, {_fmt(source['rate_hz'])} Hz")
        size = source["size_bytes"]
        interval = source["interval_ms"]
        lines.append(
            "- size bytes: "
            f"min {_fmt(size['min'])}, mean {_fmt(size['mean'])}, median {_fmt(size['median'])}, "
            f"p90 {_fmt(size['p90'])}, max {_fmt(size['max'])}, std {_fmt(size['std'])}"
        )
        lines.append(
            "- intervals: "
            f"nominal {_fmt(interval['nominal_ms'])} ms, std {_fmt(interval['std'])} ms, "
            f"{_fmt(interval['within_10pct_nominal_pct'], suffix='%')} within +/-10%"
        )
        gop = source.get("gop")
        if gop:
            spacing = gop["keyframe_spacing_frames"]
            lines.append(
                f"- GOP: {gop['keyframes']} keyframes across {gop['gops']} GOPs "
                f"({gop['method']}), median spacing {_fmt(spacing['median'])} frames"
            )
            extreme = gop.get("most_extreme_gop")
            if extreme:
                lines.append(
                    "- most extreme GOP: "
                    f"index {extreme['start_index']}, length {extreme['length']}, "
                    f"ratio {_fmt(extreme['key_to_delta_mean_ratio'])}, sizes {extreme['sizes_bytes']}"
                )
            lines.append("")
            lines.extend(
                _markdown_table(
                    ["GOP pos", "count", "mean B", "median B", "p90 B", "max B"],
                    [
                        [
                            entry["position"],
                            entry["count"],
                            _fmt(entry["mean"]),
                            _fmt(entry["median"]),
                            _fmt(entry["p90"]),
                            _fmt(entry["max"]),
                        ]
                        for entry in gop["position_size_bytes"]
                    ],
                )
            )
        else:
            lines.append("- GOP: not detected")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def write_report(report: dict[str, Any], out_dir: str | Path) -> dict[str, Path]:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    json_path = out / "stream-stats.json"
    markdown_path = out / "stream-stats.md"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=False) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report), encoding="utf-8")
    return {"json": json_path, "markdown": markdown_path}


def load_sources(
    *,
    bag_specs: Sequence[str] = (),
    events_specs: Sequence[str] = (),
    storage_id: str = "mcap",
) -> list[StreamSource]:
    sources: list[StreamSource] = []
    for value in bag_specs:
        sources.append(load_bag_source(parse_source_spec("bag", value), storage_id=storage_id))
    for value in events_specs:
        sources.append(load_events_source(parse_source_spec("events", value)))
    if not sources:
        raise ValueError("provide at least one --bag or --events source")
    return sources
