# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Convert recorded link traces into network profiles reference network profiles."""

from __future__ import annotations

import hashlib
import json
import math
import statistics
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import yaml

from rosotacom.network_profiles import OUTAGE_CATCHUP, OUTAGE_RECONNECT

PROFILE_MODES = ("timeline", "static")
PROFILE_DIRECTIONS = ("uplink", "downlink")


@dataclass(frozen=True)
class TraceSample:
    """One normalized row from ``link_trace.jsonl``."""

    time_s: float
    window_s: float | None = None
    delay_ms: float | None = None
    loss_pct: float | None = None
    uplink_rate_bps: float | None = None
    downlink_rate_bps: float | None = None


@dataclass(frozen=True)
class TraceProfileConfig:
    """Conversion knobs for trace-derived profiles."""

    name: str
    directions: tuple[str, ...] = PROFILE_DIRECTIONS
    min_segment_s: float = 5.0
    change_sensitivity: float = 0.25
    gap_outage_after_s: float | None = None
    loss_outage_min_s: float | None = None
    window_start_s: float | None = None
    window_end_s: float | None = None
    rate_percentile: float = 50.0
    delay_percentile: float = 90.0
    jitter_spread_percentile: float = 90.0

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("profile name must be non-empty")
        invalid = sorted(set(self.directions) - set(PROFILE_DIRECTIONS))
        if invalid:
            raise ValueError(f"directions must be drawn from {PROFILE_DIRECTIONS}, got {invalid}")
        if not self.directions:
            raise ValueError("at least one direction must be selected")
        if self.min_segment_s <= 0:
            raise ValueError("min_segment_s must be > 0")
        if self.change_sensitivity <= 0:
            raise ValueError("change_sensitivity must be > 0")
        if self.gap_outage_after_s is not None and self.gap_outage_after_s <= 0:
            raise ValueError("gap_outage_after_s must be > 0")
        if self.loss_outage_min_s is not None and self.loss_outage_min_s <= 0:
            raise ValueError("loss_outage_min_s must be > 0")
        if self.window_start_s is not None and self.window_start_s < 0:
            raise ValueError("window_start_s must be >= 0")
        if self.window_end_s is not None and self.window_end_s <= 0:
            raise ValueError("window_end_s must be > 0")
        if (
            self.window_start_s is not None
            and self.window_end_s is not None
            and self.window_start_s >= self.window_end_s
        ):
            raise ValueError("window start must be before window end")
        for name, value in (
            ("rate_percentile", self.rate_percentile),
            ("delay_percentile", self.delay_percentile),
            ("jitter_spread_percentile", self.jitter_spread_percentile),
        ):
            if not 0 <= value <= 100:
                raise ValueError(f"{name} must be within [0, 100]")


def parse_window(value: str | None) -> tuple[float | None, float | None]:
    """Parse ``START:END`` seconds; either side may be omitted."""
    if value is None:
        return None, None
    if ":" not in value:
        raise ValueError("--window must use START:END seconds")
    start_text, end_text = value.split(":", 1)
    start = float(start_text) if start_text else None
    end = float(end_text) if end_text else None
    if start is not None and start < 0:
        raise ValueError("--window start must be >= 0")
    if end is not None and end <= 0:
        raise ValueError("--window end must be > 0")
    if start is not None and end is not None and start >= end:
        raise ValueError("--window start must be before end")
    return start, end


def load_link_trace(path: str | Path) -> list[TraceSample]:
    """Load and normalize a link-trace JSONL file."""
    trace_path = Path(path)
    samples: list[TraceSample] = []
    for line_number, line in enumerate(trace_path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{trace_path}:{line_number}: invalid JSON: {exc}") from exc
        if not isinstance(row, Mapping):
            raise ValueError(f"{trace_path}:{line_number}: each trace row must be a JSON object")
        if row.get("kind") not in (None, "link_trace"):
            continue
        samples.append(_sample_from_row(row, line_number=line_number))
    if not samples:
        raise ValueError(f"{trace_path}: no link_trace rows found")

    first_time = min(sample.time_s for sample in samples)
    normalized = [
        TraceSample(
            time_s=sample.time_s - first_time,
            window_s=sample.window_s,
            delay_ms=sample.delay_ms,
            loss_pct=sample.loss_pct,
            uplink_rate_bps=sample.uplink_rate_bps,
            downlink_rate_bps=sample.downlink_rate_bps,
        )
        for sample in sorted(samples, key=lambda item: item.time_s)
    ]
    return normalized


def convert_trace_to_profile_yaml(
    trace_path: str | Path,
    *,
    mode: str,
    config: TraceProfileConfig,
) -> str:
    """Convert ``trace_path`` into a profiles-file YAML string with provenance comments."""
    if mode not in PROFILE_MODES:
        raise ValueError(f"mode must be one of {PROFILE_MODES}, got {mode!r}")

    path = Path(trace_path)
    samples = _window_samples(load_link_trace(path), config)
    if mode == "timeline":
        profile_doc = _timeline_profile(samples, config)
    else:
        profile_doc = _static_profile(samples, config)

    body = yaml.safe_dump({"profiles": {config.name: profile_doc}}, sort_keys=False)
    return _provenance_header(path, mode=mode, config=config) + body


def write_trace_profile(
    trace_path: str | Path,
    output_path: str | Path | None,
    *,
    mode: str,
    config: TraceProfileConfig,
) -> str:
    """Write the generated profile when requested and return the YAML text."""
    text = convert_trace_to_profile_yaml(trace_path, mode=mode, config=config)
    if output_path is not None:
        Path(output_path).write_text(text, encoding="utf-8")
    return text


def _sample_from_row(row: Mapping[str, Any], *, line_number: int) -> TraceSample:
    time_s = _required_number(row.get("monotonic_s"), f"line {line_number}: monotonic_s")
    passive = _mapping_or_none(row.get("passive_counter_delta"))
    probe = _mapping_or_none(row.get("peer_probe"))
    window_s = _optional_number(passive.get("window_s") if passive is not None else None)
    delay_ms: float | None = None
    loss_pct: float | None = None

    if probe is not None and probe.get("available", True):
        rtt_ms = _optional_number(probe.get("rtt_ms"))
        if rtt_ms is not None:
            delay_ms = max(0.0, rtt_ms / 2.0)
        raw_loss = _optional_number(probe.get("loss_pct"))
        if raw_loss is not None:
            loss_pct = min(100.0, max(0.0, raw_loss))

    return TraceSample(
        time_s=time_s,
        window_s=window_s,
        delay_ms=delay_ms,
        loss_pct=loss_pct,
        uplink_rate_bps=_direction_rate_bps(passive, "tx"),
        downlink_rate_bps=_direction_rate_bps(passive, "rx"),
    )


def _mapping_or_none(value: Any) -> Mapping[str, Any] | None:
    return cast(Mapping[str, Any], value) if isinstance(value, Mapping) else None


def _required_number(value: Any, label: str) -> float:
    number = _optional_number(value)
    if number is None:
        raise ValueError(f"{label} must be a number")
    return number


def _optional_number(value: Any) -> float | None:
    if value is None or isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _direction_rate_bps(passive: Mapping[str, Any] | None, direction: str) -> float | None:
    if passive is None or not passive.get("available", True):
        return None
    direction_block = _mapping_or_none(passive.get(direction))
    if direction_block is None or not _rate_is_capacity_like(passive, direction_block):
        return None

    for key, multiplier in (
        ("available_bps", 1.0),
        ("capacity_bps", 1.0),
        ("rate_bps", 1.0),
        ("throughput_bps", 1.0),
        ("observed_bps", 1.0),
        ("available_kbps", 1_000.0),
        ("capacity_kbps", 1_000.0),
        ("rate_kbps", 1_000.0),
        ("throughput_kbps", 1_000.0),
        ("observed_kbps", 1_000.0),
    ):
        value = _optional_number(direction_block.get(key))
        if value is not None and value > 0:
            return value * multiplier
    return None


def _rate_is_capacity_like(passive: Mapping[str, Any], direction_block: Mapping[str, Any]) -> bool:
    for block in (direction_block, passive):
        for key in ("available_bandwidth", "capacity_probe", "probed", "rate_valid", "saturated"):
            if block.get(key) is True:
                return True
        source = str(block.get("rate_source") or block.get("provenance") or "").lower()
        if any(token in source for token in ("available", "capacity", "probe", "saturated")):
            return True
    return False


def _window_samples(samples: Sequence[TraceSample], config: TraceProfileConfig) -> list[TraceSample]:
    selected = [
        sample
        for sample in samples
        if (config.window_start_s is None or sample.time_s >= config.window_start_s)
        and (config.window_end_s is None or sample.time_s <= config.window_end_s)
    ]
    if not selected:
        raise ValueError("trace window contains no samples")
    first_time = selected[0].time_s
    return [
        TraceSample(
            time_s=sample.time_s - first_time,
            window_s=sample.window_s,
            delay_ms=sample.delay_ms,
            loss_pct=sample.loss_pct,
            uplink_rate_bps=sample.uplink_rate_bps,
            downlink_rate_bps=sample.downlink_rate_bps,
        )
        for sample in selected
    ]


def _static_profile(samples: Sequence[TraceSample], config: TraceProfileConfig) -> dict[str, Any]:
    profile: dict[str, Any] = {}
    for direction in config.directions:
        direction_doc = _direction_doc(
            samples,
            direction,
            delay_percentile=config.delay_percentile,
            rate_percentile=config.rate_percentile,
            jitter_spread_percentile=config.jitter_spread_percentile,
        )
        if direction_doc:
            profile[direction] = direction_doc
    if not profile:
        profile["uplink"] = {}
    return profile


def _timeline_profile(samples: Sequence[TraceSample], config: TraceProfileConfig) -> dict[str, Any]:
    interval_s = _typical_interval_s(samples)
    gap_outage_after_s = config.gap_outage_after_s or max(interval_s * 3.0, interval_s + 0.001)
    loss_outage_min_s = config.loss_outage_min_s or interval_s
    timeline: list[dict[str, Any]] = []
    normal_segment: list[TraceSample] = []

    def flush_normal() -> None:
        nonlocal normal_segment
        if not normal_segment:
            return
        segment_doc: dict[str, Any] = {"for": _duration_text(_samples_duration_s(normal_segment, interval_s))}
        for direction in config.directions:
            direction_doc = _direction_doc(
                normal_segment,
                direction,
                delay_percentile=50.0,
                rate_percentile=config.rate_percentile,
                jitter_spread_percentile=config.jitter_spread_percentile,
            )
            if direction_doc:
                segment_doc[direction] = direction_doc
        timeline.append(segment_doc)
        normal_segment = []

    def append_normal(sample: TraceSample) -> None:
        nonlocal normal_segment
        if not normal_segment:
            normal_segment = [sample]
            return
        if _is_change_point(normal_segment, sample, interval_s, config):
            flush_normal()
            normal_segment = [sample]
        else:
            normal_segment.append(sample)

    index = 0
    while index < len(samples):
        sample = samples[index]
        if index > 0:
            gap_s = sample.time_s - samples[index - 1].time_s
            if gap_s >= gap_outage_after_s:
                flush_normal()
                timeline.append(
                    {"for": _duration_text(max(interval_s, gap_s - interval_s)), "outage": OUTAGE_RECONNECT}
                )

        if _is_full_loss_sample(sample):
            flush_normal()
            loss_samples = [sample]
            index += 1
            while index < len(samples) and _is_full_loss_sample(samples[index]):
                if samples[index].time_s - samples[index - 1].time_s >= gap_outage_after_s:
                    break
                loss_samples.append(samples[index])
                index += 1
            duration_s = _samples_duration_s(loss_samples, interval_s)
            if duration_s >= loss_outage_min_s:
                timeline.append({"for": _duration_text(duration_s), "outage": OUTAGE_CATCHUP})
            else:
                for loss_sample in loss_samples:
                    append_normal(loss_sample)
            continue

        append_normal(sample)
        index += 1

    flush_normal()
    if not timeline:
        timeline.append({"for": _duration_text(interval_s)})
    return {"timeline": timeline}


def _direction_doc(
    samples: Sequence[TraceSample],
    direction: str,
    *,
    delay_percentile: float,
    rate_percentile: float,
    jitter_spread_percentile: float,
) -> dict[str, Any]:
    delays = [sample.delay_ms for sample in samples if sample.delay_ms is not None]
    losses = [sample.loss_pct for sample in samples if sample.loss_pct is not None]
    rates: list[float] = []
    for sample in samples:
        rate = _rate_for_direction(sample, direction)
        if rate is not None:
            rates.append(rate)

    doc: dict[str, Any] = {}
    if rates:
        doc["rate"] = _rate_text(_percentile(rates, rate_percentile))
    if delays:
        delay_ms = _percentile(delays, delay_percentile)
        doc["delay"] = _ms_text(delay_ms)
        jitter_ms = max(0.0, _percentile(delays, jitter_spread_percentile) - statistics.median(delays))
        if jitter_ms > 0:
            doc["jitter"] = _ms_text(jitter_ms)
            doc["distribution"] = "normal"
    if losses:
        loss_pct = statistics.fmean(losses)
        if loss_pct > 0:
            doc["loss"] = _pct_text(loss_pct)
    return doc


def _rate_for_direction(sample: TraceSample, direction: str) -> float | None:
    if direction == "uplink":
        return sample.uplink_rate_bps
    if direction == "downlink":
        return sample.downlink_rate_bps
    raise ValueError(f"unsupported direction {direction!r}")


def _typical_interval_s(samples: Sequence[TraceSample]) -> float:
    diffs = [
        right.time_s - left.time_s
        for left, right in zip(samples, samples[1:], strict=False)
        if right.time_s > left.time_s
    ]
    if diffs:
        return max(0.001, statistics.median(diffs))
    windows = [sample.window_s for sample in samples if sample.window_s is not None and sample.window_s > 0]
    if windows:
        return max(0.001, statistics.median(windows))
    return 1.0


def _samples_duration_s(samples: Sequence[TraceSample], interval_s: float) -> float:
    if not samples:
        return interval_s
    return max(interval_s, samples[-1].time_s - samples[0].time_s + interval_s)


def _is_full_loss_sample(sample: TraceSample) -> bool:
    return sample.loss_pct is not None and sample.loss_pct >= 100.0


def _is_change_point(
    current: Sequence[TraceSample],
    sample: TraceSample,
    interval_s: float,
    config: TraceProfileConfig,
) -> bool:
    if _samples_duration_s(current, interval_s) < config.min_segment_s:
        return False

    current_features = _segment_features(current, config.directions, rate_percentile=config.rate_percentile)
    sample_features = _segment_features([sample], config.directions, rate_percentile=config.rate_percentile)
    for key, current_value in current_features.items():
        sample_value = sample_features.get(key)
        if current_value is None and sample_value is None:
            continue
        if current_value is None or sample_value is None:
            return True
        if _changed(key, current_value, sample_value, config.change_sensitivity):
            return True
    return False


def _segment_features(
    samples: Sequence[TraceSample],
    directions: Sequence[str],
    *,
    rate_percentile: float,
) -> dict[str, float | None]:
    features: dict[str, float | None] = {
        "delay_ms": _maybe_percentile([sample.delay_ms for sample in samples], 50.0),
        "loss_pct": _maybe_mean([sample.loss_pct for sample in samples]),
    }
    for direction in directions:
        features[f"{direction}_rate_bps"] = _maybe_percentile(
            [_rate_for_direction(sample, direction) for sample in samples],
            rate_percentile,
        )
    return features


def _changed(key: str, current_value: float, sample_value: float, sensitivity: float) -> bool:
    diff = abs(current_value - sample_value)
    if key.endswith("_rate_bps"):
        threshold = max(100_000.0, abs(current_value) * sensitivity)
    elif key == "loss_pct":
        threshold = max(1.0, abs(current_value) * sensitivity)
    else:
        threshold = max(5.0, abs(current_value) * sensitivity)
    return diff > threshold


def _maybe_percentile(values: Sequence[float | None], percentile: float) -> float | None:
    present = [value for value in values if value is not None]
    return _percentile(present, percentile) if present else None


def _maybe_mean(values: Sequence[float | None]) -> float | None:
    present = [value for value in values if value is not None]
    return statistics.fmean(present) if present else None


def _percentile(values: Sequence[float], percentile: float) -> float:
    if not values:
        raise ValueError("cannot compute percentile of empty values")
    ordered = sorted(values)
    if percentile == 50:
        return float(statistics.median(ordered))
    if percentile <= 0:
        return float(ordered[0])
    if percentile >= 100:
        return float(ordered[-1])
    index = math.ceil((percentile / 100.0) * len(ordered)) - 1
    return float(ordered[max(0, min(index, len(ordered) - 1))])


def _duration_text(value: float) -> str:
    return f"{value:g}s"


def _ms_text(value: float) -> str:
    return f"{value:g}ms"


def _pct_text(value: float) -> str:
    return f"{value:g}%"


def _rate_text(value: float) -> str:
    return f"{int(round(value))}bit"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _provenance_header(path: Path, *, mode: str, config: TraceProfileConfig) -> str:
    params = {
        "mode": mode,
        "name": config.name,
        "directions": ",".join(config.directions),
        "min_segment_s": config.min_segment_s,
        "change_sensitivity": config.change_sensitivity,
        "gap_outage_after_s": config.gap_outage_after_s,
        "loss_outage_min_s": config.loss_outage_min_s,
        "window": (
            f"{config.window_start_s if config.window_start_s is not None else ''}:"
            f"{config.window_end_s if config.window_end_s is not None else ''}"
        ),
        "rate_percentile": config.rate_percentile,
        "delay_percentile": config.delay_percentile,
        "jitter_spread_percentile": config.jitter_spread_percentile,
    }
    param_text = ", ".join(f"{key}={value}" for key, value in params.items() if value is not None)
    return (
        "# Generated by `rosotacom profile from-trace`.\n"
        f"# Source trace: {path}\n"
        f"# Source sha256: {_sha256(path)}\n"
        f"# Conversion parameters: {param_text}\n"
        "# Caveat: this is an approximate piecewise-constant replay profile, not packet-level replay.\n"
        "# Caveat: passive throughput is a lower-bound observation; rate is emitted only for "
        "saturated/probed samples.\n"
        "# Caveat: RTT is mapped to symmetric one-way delay when peer_probe declares the path assumption.\n"
    )
