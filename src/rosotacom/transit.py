# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Offline analysis for transit records."""

from __future__ import annotations

import json
import math
from collections.abc import Iterable
from pathlib import Path
from typing import Any


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = max(0, min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1))
    return round(ordered[rank], 3)


def load_transit_records(paths: Iterable[Path]) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for path in paths:
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
            if event.get("kind") == "transit":
                records.append(event)
    return records


def record_epoch(record: dict[str, Any]) -> int:
    """Which run of the publishing peer's wrapper produced this record.

    A peer restart resets ``seq`` to zero mid-instance, so ``(source, topic,
    seq)`` is not unique over an instance that outlived one. Records written
    before the field learned this carry no ``epoch``; they are all epoch 0,
    which is what a single-epoch instance produces anyway.
    """
    value = record.get("epoch")
    return 0 if value is None else int(value)


#: Statuses only a receiving peer can produce. Everything else on a joined row
#: came from the sender, which cannot know whether the message arrived.
_RECEIVER_STATUSES = frozenset({"delivered", "reordered", "lost"})


def join_transit_records(records: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    """Join duplicate observations by ``(source, topic, epoch, seq)``.

    Keeps the richest row per key. ``epoch`` is part of the key because it is
    what makes the key unique: on 2026-08-13 the vehicle restarted four times
    inside one centre instance, and joining on ``(source, topic, seq)`` alone
    silently dropped 123,253 of 304,876 raw transit lines -- 40% of the run,
    concentrated in exactly the mission window the analysis was about.
    """
    joined: dict[tuple[str, str, int, int], dict[str, Any]] = {}
    status_rank = {"lost": 0, "reordered": 1, "delivered": 2}
    for record in records:
        topic = str(record.get("topic") or "")
        source = str(record.get("source") or "")
        seq = int(record["seq"])
        key = (source, topic, record_epoch(record), seq)
        current = joined.get(key)
        if current is None:
            joined[key] = dict(record)
            continue
        if status_rank.get(str(record.get("status")), -1) > status_rank.get(str(current.get("status")), -1):
            current["status"] = record.get("status")
        for field, value in record.items():
            if current.get(field) is None and value is not None:
                current[field] = value
            elif field == "sections" and isinstance(value, dict):
                sections = current.setdefault("sections", {})
                for section, section_value in value.items():
                    if sections.get(section) is None and section_value is not None:
                        sections[section] = section_value
    return [joined[key] for key in sorted(joined)]


def filter_transit_records_by_publish_window(
    records: Iterable[dict[str, Any]],
    *,
    start_s: float,
    end_s: float,
    time_field: str = "t_wrap",
) -> list[dict[str, Any]]:
    """Keep records whose publish time falls inside a measurement window.

    Lost records do not carry timestamps. For each source/target/topic/epoch
    stream, keep lost sequence records only when they are bounded by
    delivered/reordered records inside the window. This excludes startup and
    teardown sequence gaps without hiding losses that happened during the
    measured interval.

    The epoch belongs in the grouping key for the same reason it belongs in the
    join key: sequence numbers only order messages within one run of the
    publishing wrapper, so a post-restart seq 40 must not bound a pre-restart
    seq 40's loss.
    """
    if end_s < start_s:
        raise ValueError("end_s must be >= start_s")
    grouped: dict[tuple[str, str, str, int], list[dict[str, Any]]] = {}
    for record in join_transit_records(records):
        key = (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
            record_epoch(record),
        )
        grouped.setdefault(key, []).append(record)

    filtered: list[dict[str, Any]] = []
    for key in sorted(grouped):
        stream = grouped[key]
        timed: list[dict[str, Any]] = []
        for record in stream:
            stamp = record.get(time_field)
            if stamp is None:
                continue
            if start_s <= float(stamp) <= end_s:
                timed.append(record)
        if not timed:
            continue
        seq_min = min(int(record["seq"]) for record in timed)
        seq_max = max(int(record["seq"]) for record in timed)
        for record in stream:
            if record.get("status") == "lost":
                seq = int(record["seq"])
                if seq_min <= seq <= seq_max:
                    filtered.append(record)
                continue
            if record in timed:
                filtered.append(record)
    return sorted(
        filtered,
        key=lambda record: (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
            record_epoch(record),
            int(record["seq"]),
        ),
    )


def _record_latency_ms(record: dict[str, Any]) -> float | None:
    sections = record.get("sections")
    if not isinstance(sections, dict):
        return None
    value = sections.get("ota_hop_ms")
    if value is None:
        value = sections.get("ota_hop_uncorrected_ms")
    return None if value is None else float(value)


def _distribution_ms(values: list[float]) -> dict[str, float | None]:
    """Full distribution of one per-message metric (ms): spread ends and quantiles."""
    if not values:
        return dict.fromkeys(("min", "mean", "p50", "p95", "p99", "max", "std"))
    mean = sum(values) / len(values)
    std = math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))
    return {
        "min": round(min(values), 3),
        "mean": round(mean, 3),
        "p50": _percentile(values, 0.50),
        "p95": _percentile(values, 0.95),
        "p99": _percentile(values, 0.99),
        "max": round(max(values), 3),
        "std": round(std, 3),
    }


def _latency_trend_ms(
    topic_records: list[dict[str, Any]],
    *,
    time_field: str = "t_wrap",
    min_points: int = 6,
) -> dict[str, float | None]:
    """Median latency of the first vs the last third of the run (by publish time).

    A positive ``delta`` means the median rose over the course of the run — the
    bufferbloat signature a whole-run p50 hides.
    """
    empty: dict[str, float | None] = dict.fromkeys(("first_third_p50", "last_third_p50", "delta"))
    stamped = sorted(
        (float(record[time_field]), latency)
        for record in topic_records
        if record.get(time_field) is not None and (latency := _record_latency_ms(record)) is not None
    )
    if len(stamped) < min_points:
        return empty
    span = stamped[-1][0] - stamped[0][0]
    if span <= 0.0:
        return empty
    first_p50 = _percentile([latency for stamp, latency in stamped if stamp <= stamped[0][0] + span / 3.0], 0.50)
    last_p50 = _percentile([latency for stamp, latency in stamped if stamp >= stamped[-1][0] - span / 3.0], 0.50)
    if first_p50 is None or last_p50 is None:
        return empty
    return {"first_third_p50": first_p50, "last_third_p50": last_p50, "delta": round(last_p50 - first_p50, 3)}


def _nominal_send_period_ms(topic_records: list[dict[str, Any]], *, time_field: str = "t_wrap") -> float | None:
    """Median publish spacing per sequence step — the send cadence of the stream.

    Per epoch: a sequence step only means "one message" inside one run of the
    publishing wrapper. Across a restart the step is a reset, and dividing the
    wall-clock gap by it produces a period that describes nothing.
    """
    periods: list[float] = []
    for epoch in {record_epoch(record) for record in topic_records}:
        stamped = sorted(
            (int(record["seq"]), float(record[time_field]))
            for record in topic_records
            if record.get(time_field) is not None and record_epoch(record) == epoch
        )
        periods.extend(
            (t1 - t0) / (s1 - s0)
            for (s0, t0), (s1, t1) in zip(stamped, stamped[1:], strict=False)
            if s1 > s0 and t1 >= t0
        )
    if not periods:
        return None
    ordered = sorted(periods)
    return round(ordered[len(ordered) // 2] * 1000.0, 3)


# Arrival-spacing regimes relative to the nominal send period: a gap well below
# nominal means messages arrived bunched (a delayed message released together with
# the one it head-of-line-blocked); a gap well above nominal means the stream stalled.
BUNCHED_GAP_FRACTION = 0.5
STALLED_GAP_FRACTION = 1.5


def _arrival_spacing_ms(topic_records: list[dict[str, Any]], *, time_field: str = "t_wrap") -> dict[str, Any]:
    """Distribution of receiver inter-arrival gaps, plus bunching/stall shares.

    ``stall_then_bunch_pct`` is the share of stalled gaps immediately followed by
    a bunched gap — equidistantly sent messages arriving as "long wait, then a
    clump", the head-of-line-blocking signature.
    """
    gaps = [float(record["inter_arrival_ms"]) for record in topic_records if record.get("inter_arrival_ms") is not None]
    stats: dict[str, Any] = _distribution_ms(gaps)
    stats["p05"] = _percentile(gaps, 0.05)
    nominal_ms = _nominal_send_period_ms(topic_records, time_field=time_field)
    stats["nominal_period_ms"] = nominal_ms
    if not gaps or not nominal_ms:
        stats.update(dict.fromkeys(("bunched_pct", "stalled_pct", "stall_then_bunch_pct")))
        return stats
    bunched = [gap < BUNCHED_GAP_FRACTION * nominal_ms for gap in gaps]
    stalled = [gap > STALLED_GAP_FRACTION * nominal_ms for gap in gaps]
    stats["bunched_pct"] = round(100.0 * sum(bunched) / len(gaps), 3)
    stats["stalled_pct"] = round(100.0 * sum(stalled) / len(gaps), 3)
    followers = [bunched[index + 1] for index in range(len(gaps) - 1) if stalled[index]]
    stats["stall_then_bunch_pct"] = round(100.0 * sum(followers) / len(followers), 3) if followers else None
    return stats


def summarize_transit_records(records: Iterable[dict[str, Any]]) -> dict[str, Any]:
    by_topic: dict[str, list[dict[str, Any]]] = {}
    for record in join_transit_records(records):
        topic = str(record.get("topic") or "")
        source = str(record.get("source") or "")
        target = str(record.get("target") or "")
        label = f"{source}->{target}:{topic}" if source or target else topic
        by_topic.setdefault(label, []).append(record)

    topics: dict[str, Any] = {}
    for topic, topic_records in sorted(by_topic.items()):
        lost = sum(record.get("status") == "lost" for record in topic_records)
        reordered = sum(record.get("status") == "reordered" for record in topic_records)
        # Only the receiving peer can say a message arrived. A row that is still
        # `sent` after the join was written by the sender and never met a
        # receiver row, so it is neither delivered nor confirmed lost — an
        # in-flight message at the window edge looks exactly like one the link
        # never carried. Counting them as delivered is how a link that delivered
        # *nothing* reported `delivered == expected, loss 0%`: sender-side rows
        # (issue #294) arrive whether or not the far side ever sees anything.
        receiver_seen = [record for record in topic_records if record.get("status") in _RECEIVER_STATUSES]
        unconfirmed = len(topic_records) - len(receiver_seen)
        delivered = len(receiver_seen) - lost
        ota = [latency for record in topic_records if (latency := _record_latency_ms(record)) is not None]
        jitter = [float(record["jitter_ms"]) for record in topic_records if record.get("jitter_ms") is not None]
        topics[topic] = {
            "expected": len(topic_records),
            "delivered": delivered,
            "lost": lost,
            # >1 means the publishing peer restarted inside this instance. Kept
            # in the summary because it changes how every rate-like number
            # below should be read, and because it is otherwise invisible.
            "epochs": len({record_epoch(record) for record in topic_records}),
            # Loss is a receiver fact. With no receiver row at all there is no
            # denominator, and `0.0` would be a claim the data cannot support.
            "loss_pct": round(100.0 * lost / len(receiver_seen), 3) if receiver_seen else None,
            "reordered": reordered,
            # Sent and never accounted for by the receiver.
            "unconfirmed": unconfirmed,
            "ota_hop_ms": _distribution_ms(ota),
            "ota_hop_trend_ms": _latency_trend_ms(topic_records),
            "jitter_ms": _distribution_ms(jitter),
            "inter_arrival_ms": _arrival_spacing_ms(topic_records),
        }
    return {"schema_version": 1, "topics": topics}
