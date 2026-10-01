# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""benchmark commands reference — benchmark genres (sweep/capacity & perturbation/recovery).

Pure, host-testable driver and verdict logic. The *runs* themselves (live ROS
graphs, emulated profiles, the nightly runner topology) are non-deterministic and
FZI-private — they live in the harness, not here (benchmark commands reference, "CI distribution").
What lives here is everything that can be exercised by a deterministic host test:

* size-pattern expansion (the a/b irregular-size load, mirrors ``sized_publisher``);
* the capacity binary-search driver and its oracle (``loss < p`` and ``latency < L``);
* sweep bounds + the shared-link guard (an unshaped run never saturates the LAN);
* the committed band store, the two-sided compare, and the ratchet (performance bands reference);
* recovery-metric extraction from a timeline of transit records;
* fixed-probe time-bin characterization for latency/loss/Hz/bandwidth plots;
* the coarse linear-ramp curve (monitor-only trend).
"""

from __future__ import annotations

import json
import math
import os
import platform
import re
import socket
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass, field, replace
from enum import Enum
from pathlib import Path
from typing import Any

# --------------------------------------------------------------------------- #
# Genre 1 — load driver: a/b size pattern
# --------------------------------------------------------------------------- #


def parse_size_pattern(pattern: str) -> list[str]:
    """Parse ``"a*4,b*1"`` / ``"ax4,bx1"`` into a cyclic list of ``a``/``b`` tokens.

    Mirrors ``sized_publisher._parse_pattern`` so the host test validates the same
    generation logic the ROS node uses. An empty pattern means a single ``a`` size.
    """
    if not pattern.strip():
        return ["a"]

    tokens: list[str] = []
    for raw_token in pattern.split(","):
        token = raw_token.strip().lower()
        match = re.fullmatch(r"([a-z])(?:[*x](\d+))?", token)
        if not match:
            raise ValueError(f"Invalid pattern token '{raw_token}'. Use tokens like 'a', 'b', 'ax4', or 'b*2'.")
        count = int(match.group(2) or "1")
        if count < 1:
            raise ValueError(f"Pattern token '{raw_token}' must repeat at least once.")
        tokens.extend([match.group(1)] * count)
    return tokens


def parse_payload_size_bytes(raw: str) -> int:
    """Parse a payload byte size with optional decimal/binary units."""
    text = raw.strip()
    match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*([A-Za-z]*)", text)
    if not match:
        raise ValueError(f"Invalid payload size {raw!r}. Use values like 20000, 20KB, or 20KiB.")

    value = float(match.group(1))
    unit = match.group(2).lower()
    units = {
        "": 1,
        "b": 1,
        "k": 1_000,
        "kb": 1_000,
        "m": 1_000_000,
        "mb": 1_000_000,
        "g": 1_000_000_000,
        "gb": 1_000_000_000,
        "ki": 1024,
        "kib": 1024,
        "mi": 1024 * 1024,
        "mib": 1024 * 1024,
        "gi": 1024 * 1024 * 1024,
        "gib": 1024 * 1024 * 1024,
    }
    if unit not in units:
        raise ValueError(f"Invalid payload size unit {match.group(2)!r}. Use B, KB, MB, KiB, or MiB.")
    bytes_value = value * units[unit]
    if bytes_value < 0 or not bytes_value.is_integer():
        raise ValueError(f"Payload size {raw!r} does not resolve to a whole non-negative byte count.")
    return int(bytes_value)


def _compress_size_pattern(tokens: Sequence[str]) -> str:
    compressed: list[str] = []
    current = tokens[0]
    count = 1
    for token in tokens[1:]:
        if token == current:
            count += 1
            continue
        compressed.append(f"{current}*{count}")
        current = token
        count = 1
    compressed.append(f"{current}*{count}")
    return ",".join(compressed)


def parse_size_pattern_load(pattern: str) -> dict[str, Any]:
    """Parse ``1x20KB+1x0KB`` into ``sized_publisher`` load parameters.

    Pattern terms are ``COUNTxSIZE`` or just ``SIZE``, separated by ``+`` or
    ``,``. Decimal units (KB/MB/GB) use powers of 1000; binary units
    (KiB/MiB/GiB) use powers of 1024.
    """
    if not pattern.strip():
        raise ValueError("size pattern must not be empty.")

    label_by_size: dict[int, str] = {}
    size_by_label: dict[str, int] = {}
    tokens: list[str] = []

    for raw_term in re.split(r"[+,]", pattern):
        term = raw_term.strip()
        if not term:
            raise ValueError(f"Invalid size pattern {pattern!r}: empty term.")
        match = re.fullmatch(r"(?:(\d+)\s*[x*]\s*)?(.+)", term)
        if not match:
            raise ValueError(f"Invalid size pattern term {raw_term!r}. Use terms like 1x20KB or 0KB.")
        count = int(match.group(1) or "1")
        if count < 1:
            raise ValueError(f"Size pattern term {raw_term!r} must repeat at least once.")
        size = parse_payload_size_bytes(match.group(2))
        label = label_by_size.get(size)
        if label is None:
            label = chr(ord("a") + len(label_by_size))
            label_by_size[size] = label
            size_by_label[label] = size
        tokens.extend([label] * count)

    sizes_list = [size_by_label[t] for t in tokens]

    load: dict[str, Any] = {
        "sizes": sizes_list,
        "pattern": _compress_size_pattern(tokens),
        "size_pattern": pattern,
    }
    for label, size in size_by_label.items():
        load[f"size_{label}"] = size

    return load


def expand_size_pattern(pattern: str, size_a: int, size_b: int | None = None, **kwargs: Any) -> list[int]:
    """Expand a pattern into the concrete cyclic byte-size sequence it publishes."""
    if size_a < 0:
        raise ValueError("size_a must be >= 0.")
    tokens = parse_size_pattern(pattern)
    sizes = {"a": size_a, "b": size_b}
    sizes.update(kwargs)
    for token in tokens:
        if token not in sizes or sizes[token] is None:
            raise ValueError(f"Pattern references size {token!r} but it was not provided.")
    return [int(sizes[token]) for token in tokens]  # type: ignore[arg-type]


def pattern_mean_bytes(pattern: str, size_a: int, size_b: int | None = None, **kwargs: Any) -> float:
    """Mean payload of one pattern cycle — the basis for the offered-bandwidth bound."""
    sizes = expand_size_pattern(pattern, size_a, size_b, **kwargs)
    return sum(sizes) / len(sizes)


# --------------------------------------------------------------------------- #
# Genre 1 — oracle: loss < p AND latency < L over a window
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class OracleThresholds:
    """Pass/fail bound for a capacity probe, read off the transit metrics."""

    max_loss_pct: float
    max_latency_ms: float
    latency_quantile: str = "p95"


def oracle_passes(loss_pct: float, latency_ms: float | None, thresholds: OracleThresholds) -> bool:
    """A point passes iff loss and the chosen latency quantile are both under bound.

    ``latency_ms is None`` means nothing was delivered in the window — a failure.
    """
    if latency_ms is None:
        return False
    return loss_pct <= thresholds.max_loss_pct and latency_ms <= thresholds.max_latency_ms


def _oracle_loss_pct(value: Any) -> float:
    """A topic's loss for an oracle, with "unknown" read as the worst case.

    `summarize_transit_records` reports `None` when no receiving peer accounted
    for the topic at all. That is not zero loss; it is no evidence, and an
    oracle that let it pass would turn a dead link into a green run.
    """
    return 100.0 if value is None else float(value)


def oracle_passes_topic(topic_summary: dict[str, Any], thresholds: OracleThresholds) -> bool:
    """Apply the oracle to one topic of ``transit.summarize_transit_records`` output."""
    loss_pct = _oracle_loss_pct(topic_summary.get("loss_pct", 100.0))
    latency = (topic_summary.get("ota_hop_ms") or {}).get(thresholds.latency_quantile)
    return oracle_passes(loss_pct, None if latency is None else float(latency), thresholds)


# --------------------------------------------------------------------------- #
# Bag-as-load oracle: a whole replay contract, judged per topic
# --------------------------------------------------------------------------- #
#
# When the benchmark *load* is a bag replay (a real session/scenario) rather than
# a single synthetic stream, one probe point delivers many topics at once. The
# verdict is then per topic — each contract topic must clear the network-loss and
# latency bound *and* deliver a high-enough share of the bag's known messages
# (completeness ground truth from ``bag_ground_truth``) — aggregated to a run
# verdict: the run passes iff every required topic passes, and the failing topics
# are named. This mirrors ``oracle_passes_topic`` for the loss/latency part and
# adds the completeness gate the known-source replay makes possible (expectation concepts reference,
# "Live vs replay"). It is pure so the search over replay verdicts is host-tested.


@dataclass(frozen=True)
class TopicVerdict:
    """Per-topic verdict of one replay run against the contract + bag ground truth."""

    topic: str
    passes: bool
    expected: int | None  # bag ground-truth count when known, else the run's own expected
    delivered: int | None
    lost: int | None
    reordered: int | None
    loss_pct: float
    completeness: float | None  # delivered / bag-count; None when no ground truth
    latency_p95_ms: float | None
    jitter_p95_ms: float | None
    loss_free: bool  # zero network loss AND (when known) complete against the bag
    latency_ok: bool
    reason: str  # "ok" or the first failing check, for the failing-topics list


@dataclass(frozen=True)
class BagRunVerdict:
    """Run verdict aggregated over every required contract topic of one replay."""

    passes: bool
    loss_free: bool
    latency_ok: bool
    topics: tuple[TopicVerdict, ...]
    failing_topics: tuple[str, ...]
    representative_topic: str
    # Row aggregates (summed counts; worst-case rate metrics across topics):
    expected: int
    delivered: int
    lost: int
    reordered: int
    loss_pct: float
    latency_p95_ms: float | None
    jitter_p95_ms: float | None


def summary_topic_name(label: str) -> str:
    """The bare ROS topic of a ``summarize_transit_records`` label.

    Labels are ``"<source>-><target>:<topic>"`` when directions are known (peer
    identities carry no ``:``) and just ``"<topic>"`` otherwise; ROS topics never
    contain ``:``, so the topic is everything after the first ``:``.
    """
    if "->" in label and ":" in label:
        return label.split(":", 1)[1]
    return label


def _topic_data_by_name(summary: dict[str, Any]) -> dict[str, tuple[str, dict[str, Any]]]:
    """Index a run summary by bare topic name, keeping the richest per-topic row.

    A contract topic flows one direction, but if a name appears under several
    labels the entry with the most delivered messages is the real stream.
    """
    topics = summary.get("topics")
    if not isinstance(topics, dict):
        return {}
    indexed: dict[str, tuple[str, dict[str, Any]]] = {}
    for label, data in topics.items():
        if not isinstance(data, dict):
            continue
        name = summary_topic_name(str(label))
        current = indexed.get(name)
        if current is None or int(data.get("delivered") or 0) > int(current[1].get("delivered") or 0):
            indexed[name] = (str(label), data)
    return indexed


def evaluate_bag_run(
    summary: dict[str, Any],
    *,
    thresholds: OracleThresholds,
    ground_truth: dict[str, dict[str, Any]] | None = None,
    min_completeness: float = 0.95,
    required_topics: Sequence[str] | None = None,
) -> BagRunVerdict:
    """Judge one replay run's transit summary against the whole contract.

    ``required_topics`` is the set the run must deliver (the session's carried
    topics); when omitted it is taken from ``ground_truth`` if given, else from
    every topic in the summary. ``ground_truth`` maps ROS topic -> bag facts
    (``bag_ground_truth``); when a topic has a bag ``count`` the completeness gate
    (``delivered / count >= min_completeness``) applies on top of the loss/latency
    oracle. A required topic missing from the summary counts as fully lost.
    """
    if not 0.0 < min_completeness <= 1.0:
        raise ValueError("min_completeness must be within (0, 1].")
    indexed = _topic_data_by_name(summary)
    if required_topics is not None:
        names = [str(name) for name in required_topics]
    elif ground_truth is not None:
        names = sorted(name for name in ground_truth if name in indexed) or sorted(indexed)
    else:
        names = sorted(indexed)

    quantile = thresholds.latency_quantile
    verdicts: list[TopicVerdict] = []
    for name in names:
        label, data = indexed.get(name, (name, {}))
        gt = (ground_truth or {}).get(name) or {}
        gt_count = int(gt.get("count") or 0) if gt else 0

        delivered = data.get("delivered")
        delivered_n = int(delivered) if delivered is not None else 0
        lost = data.get("lost")
        reordered = data.get("reordered")
        run_expected = data.get("expected")
        loss_pct = _oracle_loss_pct(data.get("loss_pct", 100.0)) if data else 100.0
        latency_p95 = (data.get("ota_hop_ms") or {}).get(quantile) if data else None
        jitter_p95 = (data.get("jitter_ms") or {}).get("p95") if data else None
        latency_val = None if latency_p95 is None else float(latency_p95)

        expected_out = gt_count if gt_count > 0 else (int(run_expected) if run_expected is not None else None)
        completeness = (delivered_n / gt_count) if gt_count > 0 else None

        latency_ok = latency_val is not None and latency_val <= thresholds.max_latency_ms
        loss_ok = loss_pct <= thresholds.max_loss_pct
        no_network_loss = loss_pct <= 0.0 and int(lost or 0) == 0
        complete = completeness is None or completeness >= min_completeness
        passes = bool(data) and loss_ok and latency_ok and complete
        loss_free = bool(data) and no_network_loss and complete

        if not data:
            reason = "absent"
        elif not complete:
            reason = f"incomplete({completeness:.3f}<{min_completeness:g})"
        elif not loss_ok:
            reason = f"loss({loss_pct:.3g}%>{thresholds.max_loss_pct:g}%)"
        elif not latency_ok:
            if latency_val is None:
                reason = "latency(none)"
            else:
                reason = f"latency({latency_val:.3g}>{thresholds.max_latency_ms:g}ms)"
        else:
            reason = "ok"

        verdicts.append(
            TopicVerdict(
                topic=name,
                passes=passes,
                expected=expected_out,
                delivered=delivered_n if data else 0,
                lost=int(lost) if lost is not None else (expected_out if not data and expected_out else None),
                reordered=int(reordered) if reordered is not None else None,
                loss_pct=loss_pct,
                completeness=round(completeness, 6) if completeness is not None else None,
                latency_p95_ms=latency_val,
                jitter_p95_ms=None if jitter_p95 is None else float(jitter_p95),
                loss_free=loss_free,
                latency_ok=latency_ok,
                reason=reason,
            )
        )

    failing = tuple(v.topic for v in verdicts if not v.passes)
    passes = bool(verdicts) and not failing
    loss_free = bool(verdicts) and all(v.loss_free for v in verdicts)
    latency_ok = bool(verdicts) and all(v.latency_ok for v in verdicts)

    expected_total = sum(int(v.expected) for v in verdicts if v.expected is not None)
    delivered_total = sum(int(v.delivered) for v in verdicts if v.delivered is not None)
    lost_total = sum(int(v.lost) for v in verdicts if v.lost is not None)
    reordered_total = sum(int(v.reordered) for v in verdicts if v.reordered is not None)
    loss_values = [v.loss_pct for v in verdicts]
    latency_values = [v.latency_p95_ms for v in verdicts if v.latency_p95_ms is not None]
    jitter_values = [v.jitter_p95_ms for v in verdicts if v.jitter_p95_ms is not None]
    representative = failing[0] if failing else (verdicts[0].topic if verdicts else "")

    return BagRunVerdict(
        passes=passes,
        loss_free=loss_free,
        latency_ok=latency_ok,
        topics=tuple(verdicts),
        failing_topics=failing,
        representative_topic=representative,
        expected=expected_total,
        delivered=delivered_total,
        lost=lost_total,
        reordered=reordered_total,
        loss_pct=max(loss_values) if loss_values else 100.0,
        latency_p95_ms=max(latency_values) if latency_values else None,
        jitter_p95_ms=max(jitter_values) if jitter_values else None,
    )


# --------------------------------------------------------------------------- #
# Genre 1 — sweep bounds + shared-link guard
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class SweepBounds:
    """Operationally-relevant ceilings; a sweep stops here, not "until it breaks".

    ``max_bandwidth_bps`` is the offered-load ceiling that keeps an unshaped / LAN
    run from saturating the shared network (benchmark commands reference, "Bounded, not unbounded").
    """

    max_size: int | None = None
    max_rate: float | None = None
    max_bandwidth_bps: float | None = None


def offered_bandwidth_bps(size_bytes: float, rate_hz: float) -> float:
    return size_bytes * 8.0 * rate_hz


def within_shared_link_budget(size_bytes: float, rate_hz: float, bounds: SweepBounds) -> bool:
    if bounds.max_bandwidth_bps is None:
        return True
    return offered_bandwidth_bps(size_bytes, rate_hz) <= bounds.max_bandwidth_bps


def guard_shared_link(size_bytes: float, rate_hz: float, bounds: SweepBounds, *, shared_link: bool) -> None:
    """Raise if a *shared* (unshaped/LAN) run would exceed the offered-load budget."""
    if shared_link and not within_shared_link_budget(size_bytes, rate_hz, bounds):
        raise ValueError(
            f"offered load {offered_bandwidth_bps(size_bytes, rate_hz):.0f} bps at "
            f"size={size_bytes} B, rate={rate_hz} Hz would saturate the shared link "
            f"(budget {bounds.max_bandwidth_bps:.0f} bps)"
        )


def size_ceiling(bounds: SweepBounds, *, rate_hz: float) -> int | None:
    """Largest payload the sweep may probe at ``rate_hz`` under the configured bounds."""
    ceilings: list[int] = []
    if bounds.max_size is not None:
        ceilings.append(int(bounds.max_size))
    if bounds.max_bandwidth_bps is not None and rate_hz > 0:
        ceilings.append(int(bounds.max_bandwidth_bps // (8.0 * rate_hz)))
    return min(ceilings) if ceilings else None


# --------------------------------------------------------------------------- #
# Genre 1 — capacity binary search
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class CapacitySlice:
    """A capacity is a slice of (size × rate × profile); always state the slice."""

    profile: str
    knob: str  # the swept parameter, e.g. "size" or "rate"
    fixed: dict[str, float] = field(default_factory=dict)


@dataclass(frozen=True)
class CapacityResult:
    slice: CapacitySlice
    capacity: int | None  # highest passing value, or None if even ``low`` fails


def capacity_binary_search(low: int, high: int, probe: Callable[[int], bool]) -> int | None:
    """Highest integer value in ``[low, high]`` for which ``probe`` passes.

    Assumes monotonicity (passing at smaller values) — see the RFC's "honest
    limits": real cellular's non-monotonic loss can fool a single search, so the
    live driver confirms with repeats/medians. Returns ``None`` if ``low`` fails.
    """
    if low > high:
        raise ValueError("low must be <= high")
    if not probe(low):
        return None
    best = low
    lo, hi = low + 1, high
    while lo <= hi:
        mid = (lo + hi) // 2
        if probe(mid):
            best = mid
            lo = mid + 1
        else:
            hi = mid - 1
    return best


def find_capacity(
    slice_: CapacitySlice,
    low: int,
    high: int,
    probe: Callable[[int], bool],
    *,
    bounds: SweepBounds | None = None,
    rate_hz: float | None = None,
) -> CapacityResult:
    """Bounded capacity search: clamp ``high`` to the sweep ceiling, then search.

    When ``bounds`` and ``rate_hz`` are given the swept ceiling never exceeds the
    offered-load budget, so a sweep on a shared link cannot be asked to saturate it.
    """
    if bounds is not None and rate_hz is not None:
        ceiling = size_ceiling(bounds, rate_hz=rate_hz)
        if ceiling is not None:
            high = min(high, ceiling)
    return CapacityResult(slice=slice_, capacity=capacity_binary_search(low, high, probe))


# --------------------------------------------------------------------------- #
# Genre 1/2 — committed bands + the ratchet (performance bands reference, the analogue of ``expect``)
# --------------------------------------------------------------------------- #

BAND_SCHEMA = 2

# Half-width = max(k·σ, floor). The defaults are starting points; the runner-class
# calibration (performance bands reference checklist) settles them empirically per metric.
DEFAULT_WIDTH_K = 3.0
DEFAULT_FLOOR_FRAC = 0.02

# Fingerprint for bands rewritten mechanically without measured runs (e.g. the v1
# → v2 schema rewrite). It never matches a real runner class, so ``compare``
# refuses these bands until a real calibration replaces them.
UNCALIBRATED_FINGERPRINT = "uncalibrated"

RECALIBRATE_HINT = "recalibrate on the target runner class: rosotacom benchmark ratchet <result.json ...> --recalibrate"


class Better(str, Enum):
    """A metric's better-direction: which way out of the band is an improvement."""

    HIGHER = "higher"  # capacity numbers, completeness
    LOWER = "lower"  # latency, loss, recovery times/bursts


class Verdict(str, Enum):
    WITHIN = "WITHIN"
    REGRESSED = "REGRESSED"  # out of band on the worse side
    IMPROVED = "IMPROVED"  # out of band on the better side — gate-red too: ratchet it


class BandError(ValueError):
    """A band refusal. Every path that declines to compare or ratchet raises this."""


class FingerprintMismatch(BandError):
    """Bands never transfer across runner classes (performance bands reference)."""


class WideningRefused(BandError):
    """A plain ratchet only turns one way; moving toward worse needs ``--recalibrate``."""


@dataclass(frozen=True)
class BandProvenance:
    """Where a band's width comes from — a band without provenance is a guess."""

    fingerprint: str  # runner class whose variance calibrated the width
    window_s: float  # measurement window of one calibration run
    repeats: int  # K calibration runs behind sigma
    sigma: float  # run-to-run standard deviation across the K runs
    floor: float  # minimum half-width; keeps one lucky calibration from minting an impossibly tight band
    k: float  # half-width = max(k·sigma, floor)
    source_sha: str  # commit whose run(s) last moved the band
    ratcheted_at: str  # ISO timestamp of the last ratchet
    note: str = ""  # one-line cause note for the last move ("gop default changed → smaller keyframes")


@dataclass(frozen=True)
class Band:
    """Committed two-sided envelope for one ``(row, profile, metric)``.

    Leaving ``[lo, hi]`` toward worse is ``REGRESSED``; toward better is
    ``IMPROVED`` — and in a gate lane both are red, the latter with the exact
    ratchet command. Bands are never hand-edited: they change only through
    :func:`ratchet_band`.
    """

    row: str
    profile: str
    metric: str
    lo: float
    hi: float
    better: Better
    provenance: BandProvenance

    def __post_init__(self) -> None:
        if self.lo > self.hi:
            raise BandError(f"band [{self.lo}, {self.hi}] for {self.metric!r} is inverted (lo > hi)")

    @property
    def center(self) -> float:
        return 0.5 * (self.lo + self.hi)

    @property
    def half_width(self) -> float:
        return 0.5 * (self.hi - self.lo)

    @property
    def key(self) -> tuple[str, str, str]:
        return (self.row, self.profile, self.metric)


@dataclass(frozen=True)
class BandComparison:
    band: Band
    value: float
    verdict: Verdict


def _two_sided_verdict(value: float, *, lo: float, hi: float, better: Better) -> Verdict:
    """WITHIN / IMPROVED / REGRESSED for a value against a closed ``[lo, hi]`` envelope.

    Shared by the committed-band gate (:func:`band_verdict`) and A/B experiments
    (:func:`classify_change`) so both speak one verdict language.
    """
    if value < lo:
        return Verdict.REGRESSED if better is Better.HIGHER else Verdict.IMPROVED
    if value > hi:
        return Verdict.IMPROVED if better is Better.HIGHER else Verdict.REGRESSED
    return Verdict.WITHIN


def band_verdict(band: Band, value: float) -> Verdict:
    """Two-sided verdict; the interval is closed (values on the edge are WITHIN)."""
    return _two_sided_verdict(value, lo=band.lo, hi=band.hi, better=band.better)


def compare_to_band(band: Band, value: float, *, fingerprint: str) -> BandComparison:
    """Verdict for one measured value, refusing cross-runner-class comparisons.

    A runner change must force a visible recalibration, never a silent shift —
    so a fingerprint mismatch is a refusal, not a verdict.
    """
    if fingerprint != band.provenance.fingerprint:
        raise FingerprintMismatch(
            f"{band.metric} band for ({band.row}, {band.profile}) was calibrated on runner class "
            f"{band.provenance.fingerprint!r}, but this run comes from {fingerprint!r}. "
            f"Bands never transfer across runner classes — {RECALIBRATE_HINT}"
        )
    return BandComparison(band=band, value=float(value), verdict=band_verdict(band, float(value)))


def save_bands(path: Path, bands: Iterable[Band]) -> None:
    """Write the band store: one JSON line per (row, profile, metric), stably ordered.

    The band diff is part of the reviewed change (performance bands reference), so ordering and
    key order are deterministic — a ratchet shows up as a minimal, readable diff.
    """
    ordered = sorted(bands, key=lambda band: band.key)
    duplicates = [key for key, count in _count_keys(ordered).items() if count > 1]
    if duplicates:
        raise BandError(f"duplicate band keys {duplicates!r}; one band per (row, profile, metric)")
    lines = [
        json.dumps(
            {
                "schema": BAND_SCHEMA,
                "row": band.row,
                "profile": band.profile,
                "metric": band.metric,
                "lo": band.lo,
                "hi": band.hi,
                "better": band.better.value,
                "provenance": asdict(band.provenance),
            },
            sort_keys=True,
        )
        for band in ordered
    ]
    path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def load_bands(path: Path) -> list[Band]:
    bands: list[Band] = []
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        if not line.strip():
            continue
        raw = json.loads(line)
        schema = raw.get("schema")
        if schema != BAND_SCHEMA:
            raise BandError(
                f"{path}:{line_no}: band schema {schema!r} is not supported (expected {BAND_SCHEMA}). "
                f"v1 budget entries were removed without a migration shim — {RECALIBRATE_HINT}"
            )
        bands.append(
            Band(
                row=str(raw["row"]),
                profile=str(raw["profile"]),
                metric=str(raw["metric"]),
                lo=float(raw["lo"]),
                hi=float(raw["hi"]),
                better=Better(raw["better"]),
                provenance=BandProvenance(**raw["provenance"]),
            )
        )
    duplicates = [key for key, count in _count_keys(bands).items() if count > 1]
    if duplicates:
        raise BandError(f"{path}: duplicate band keys {duplicates!r}; one band per (row, profile, metric)")
    return bands


def _count_keys(bands: Iterable[Band]) -> dict[tuple[str, str, str], int]:
    counts: dict[tuple[str, str, str], int] = {}
    for band in bands:
        counts[band.key] = counts.get(band.key, 0) + 1
    return counts


def find_band(bands: Iterable[Band], *, row: str, profile: str, metric: str) -> Band | None:
    for band in bands:
        if band.key == (row, profile, metric):
            return band
    return None


def bands_for(bands: Iterable[Band], *, row: str, profile: str) -> list[Band]:
    return [band for band in bands if band.row == row and band.profile == profile]


def ratchet_band(
    existing: Band | None,
    values: Sequence[float],
    *,
    row: str,
    profile: str,
    metric: str,
    better: Better,
    fingerprint: str,
    window_s: float,
    source_sha: str,
    ratcheted_at: str,
    note: str = "",
    recalibrate: bool = False,
    k: float = DEFAULT_WIDTH_K,
    floor: float = 0.0,
    floor_frac: float = DEFAULT_FLOOR_FRAC,
) -> Band:
    """The only way a band changes (performance bands reference). Center = median of ``values``.

    Plain ratchet re-centers inside the calibrated width and only turns one way:
    the worse edge may move toward better, never toward worse, and the width and
    calibration provenance (σ, floor, k, fingerprint, window, repeats) are
    preserved. ``recalibrate`` recomputes the width from the given runs — K fresh
    repeats — with half-width ``max(k·σ, floor)``; that is the only path that may
    widen a band, move it toward worse, or change its runner class.
    """
    if not values:
        raise BandError(f"no values for {metric!r} — a ratchet needs at least one run")
    center = _median([float(value) for value in values])

    if recalibrate:
        sigma = _sample_stdev([float(value) for value in values])
        floor_used = max(float(floor), float(floor_frac) * abs(center))
        half_width = max(float(k) * sigma, floor_used)
        if half_width <= 0.0:
            raise BandError(
                f"recalibrated half-width for {metric!r} is 0 (σ=0 from {len(values)} run(s), floor=0) — "
                "pass --floor or --floor-frac so the band has a width"
            )
        provenance = BandProvenance(
            fingerprint=fingerprint,
            window_s=float(window_s),
            repeats=len(values),
            sigma=sigma,
            floor=floor_used,
            k=float(k),
            source_sha=source_sha,
            ratcheted_at=ratcheted_at,
            note=note,
        )
        return Band(
            row=row,
            profile=profile,
            metric=metric,
            lo=center - half_width,
            hi=center + half_width,
            better=better,
            provenance=provenance,
        )

    if existing is None:
        raise BandError(
            f"no committed band for ({row}, {profile}, {metric}) — a first band is a calibration; "
            "rerun with --recalibrate"
        )
    if better is not existing.better:
        raise BandError(
            f"better-direction for {metric!r} changed ({existing.better.value} → {better.value}); "
            "that is a redesign of the metric — rerun with --recalibrate"
        )
    if fingerprint != existing.provenance.fingerprint:
        raise FingerprintMismatch(
            f"cannot ratchet the ({row}, {profile}, {metric}) band from runner class {fingerprint!r}: "
            f"its width was calibrated on {existing.provenance.fingerprint!r}. {RECALIBRATE_HINT}"
        )

    half_width = existing.half_width
    lo, hi = center - half_width, center + half_width
    if existing.better is Better.HIGHER and lo < existing.lo:
        raise WideningRefused(
            f"ratchet would move the {metric!r} floor toward worse ({existing.lo:g} → {lo:g}). "
            "A ratchet only tightens; a deliberate move toward worse needs --recalibrate"
        )
    if existing.better is Better.LOWER and hi > existing.hi:
        raise WideningRefused(
            f"ratchet would move the {metric!r} ceiling toward worse ({existing.hi:g} → {hi:g}). "
            "A ratchet only tightens; a deliberate move toward worse needs --recalibrate"
        )
    provenance = replace(existing.provenance, source_sha=source_sha, ratcheted_at=ratcheted_at, note=note)
    return Band(row=row, profile=profile, metric=metric, lo=lo, hi=hi, better=existing.better, provenance=provenance)


def _sample_stdev(values: Sequence[float]) -> float:
    if len(values) < 2:
        return 0.0
    mean = sum(values) / len(values)
    return math.sqrt(sum((value - mean) ** 2 for value in values) / (len(values) - 1))


def runner_fingerprint(env: Mapping[str, str] | None = None) -> str:
    """Runner-class fingerprint recorded in bands and in every ``result.json``.

    ``ROSOTACOM_RUNNER_CLASS`` names the class explicitly (the private bench pair
    sets it); GitHub-hosted runners collapse to one class per OS/arch (their CPU
    models vary run-to-run, the class does not); anything else is pinned to the
    host, so an ad-hoc machine never silently compares against CI bands.
    """
    env = os.environ if env is None else env
    explicit = env.get("ROSOTACOM_RUNNER_CLASS", "").strip()
    if explicit:
        return explicit
    system = platform.system().lower()
    machine = platform.machine().lower()
    if env.get("RUNNER_ENVIRONMENT", "") == "github-hosted":
        return f"github-hosted-{system}-{machine}"
    return f"host-{socket.gethostname()}-{system}-{machine}"


# Better-directions for the metrics the extraction below produces. New banded
# metrics extend this table (or pass an explicit direction at ratchet time).
_DEFAULT_BETTER: dict[str, Better] = {
    "t_recover_s": Better.LOWER,
    "t_steady_s": Better.LOWER,
    "recovery_burst": Better.LOWER,
    "lost_during_outage_total": Better.LOWER,
    "completeness_pct": Better.HIGHER,
    "loss_pct": Better.LOWER,
    "latency_p50_ms": Better.LOWER,
    "latency_p95_ms": Better.LOWER,
    "payload_bandwidth_bps": Better.HIGHER,
}


def default_better(metric: str) -> Better:
    if metric.startswith("capacity_"):
        return Better.HIGHER
    better = _DEFAULT_BETTER.get(metric)
    if better is None:
        raise BandError(f"no default better-direction for metric {metric!r}; pass --better higher|lower")
    return better


def probe_delivery_totals(doc: Mapping[str, Any]) -> tuple[int, int, int]:
    """``(expected, delivered, attempts)`` summed over a probe/replay run.

    The whole-bag expect assertion (performance bands reference) needs the raw counts, not the
    percentages a band compares, so both readers take them from here.
    """
    attempts = (doc.get("measurements") or {}).get("attempts") or []
    expected = 0
    lost = 0
    counted_attempts = 0
    for attempt in attempts:
        contributed = False
        for topic_row in attempt.get("topics") or []:
            if topic_row.get("expected") is None:
                continue
            expected += int(topic_row["expected"])
            lost += int(topic_row.get("lost") or 0)
            contributed = True
        if contributed:
            counted_attempts += 1
    return expected, expected - lost, counted_attempts


def _probe_metrics_from_result(doc: Mapping[str, Any]) -> dict[str, float]:
    """Aggregate a probe run's per-attempt topic summaries into band metrics.

    ``loss_pct`` is delivered-weighted across every attempt and topic (the
    bottleneck-dominated gate metric); the latency percentiles are medians of
    the per-attempt summaries (host-timing-dominated — monitor material on
    shared runners, performance bands reference).
    """
    attempts = (doc.get("measurements") or {}).get("attempts") or []
    p50s: list[float] = []
    p95s: list[float] = []
    bandwidth_by_bin: dict[tuple[int, float, float], float] = {}
    for attempt in attempts:
        for topic_row in attempt.get("topics") or []:
            if topic_row.get("expected") is None:
                continue
            latency = topic_row.get("latency_ms") or {}
            if latency.get("p50") is not None:
                p50s.append(float(latency["p50"]))
            if latency.get("p95") is not None:
                p95s.append(float(latency["p95"]))
    expected, delivered, _attempts = probe_delivery_totals(doc)
    for bin_row in (doc.get("measurements") or {}).get("time_bins") or []:
        bandwidth = bin_row.get("payload_bandwidth_bps")
        if bandwidth is None:
            continue
        key = (
            int(bin_row.get("attempt") or 0),
            float(bin_row.get("bin_start_s") or 0.0),
            float(bin_row.get("bin_end_s") or 0.0),
        )
        bandwidth_by_bin[key] = bandwidth_by_bin.get(key, 0.0) + float(bandwidth)
    if expected <= 0:
        raise BandError("probe run carries no per-topic expected/lost counts — there is nothing to band")
    metrics = {
        "completeness_pct": 100.0 * delivered / expected,
        "loss_pct": 100.0 * (expected - delivered) / expected,
    }
    if p50s:
        metrics["latency_p50_ms"] = _median(p50s)
    if p95s:
        metrics["latency_p95_ms"] = _median(p95s)
    if bandwidth_by_bin:
        metrics["payload_bandwidth_bps"] = _median(list(bandwidth_by_bin.values()))
    return metrics


def _replay_metrics_from_result(doc: Mapping[str, Any]) -> dict[str, float]:
    """A replay run's probe metrics plus the two the bag comparison is made of.

    ``delivered_count`` and ``delivered_hz`` are per attempt, so a row with
    repeats stays comparable to the bag's own single-pass count and rate.
    """
    metrics = _probe_metrics_from_result(doc)
    _expected, delivered, attempts = probe_delivery_totals(doc)
    if attempts <= 0:
        raise BandError("replay run carries no measured attempt — there is nothing to band")
    per_attempt = delivered / attempts
    metrics["delivered_count"] = per_attempt
    window_s = result_window_s(doc)
    if window_s > 0.0:
        metrics["delivered_hz"] = per_attempt / window_s
    return metrics


def metrics_from_result(doc: Mapping[str, Any]) -> dict[str, float]:
    """The band-comparable metrics of one self-contained ``result.json``.

    Covers the deterministic genres the gate bands today: ``capacity`` (the
    breakpoint), ``probe`` (loss/completeness plus latency percentiles),
    ``replay`` (the probe set plus the delivered count/rate the bag comparison
    is made of) and ``recovery`` (the benchmark commands reference recovery metric set). The
    benched-set registry (performance bands reference) picks which of these actually gate per row.
    """
    genre = doc.get("genre")
    if genre == "probe":
        return _probe_metrics_from_result(doc)
    if genre == "replay":
        return _replay_metrics_from_result(doc)
    if genre == "capacity":
        knob = str((doc.get("configuration") or {}).get("knob") or "size")
        capacity = (doc.get("result") or {}).get("capacity")
        if capacity is None:
            raise BandError("capacity run found no passing probe — there is no value to band")
        return {f"capacity_{knob}": float(capacity)}
    if genre == "recovery":
        result = doc.get("result") or {}
        metrics: dict[str, float] = {}
        for metric, field_name in (("t_recover_s", "t_recover"), ("t_steady_s", "t_steady")):
            if result.get(field_name) is not None:
                metrics[metric] = float(result[field_name])
        if result.get("recovery_burst") is not None:
            metrics["recovery_burst"] = float(result["recovery_burst"])
        lost = result.get("lost_during_outage") or {}
        metrics["lost_during_outage_total"] = float(sum(int(count) for count in lost.values()))
        return metrics
    raise BandError(
        f"genre {genre!r} has no band metrics yet — banded rows cover probe, capacity, replay and recovery today"
    )


def result_row_id(doc: Mapping[str, Any]) -> str:
    """Canonical row id of a run until the benched-set registry formalizes rows."""
    genre = str(doc.get("genre") or "")
    if genre == "capacity":
        knob = str((doc.get("configuration") or {}).get("knob") or "size")
        return f"capacity-{knob}"
    return genre or "unknown"


def result_profile(doc: Mapping[str, Any]) -> str:
    profile = (doc.get("configuration") or {}).get("profile")
    if not profile:
        raise BandError("result.json carries no configuration.profile — cannot key a band without one")
    return str(profile)


def result_fingerprint(doc: Mapping[str, Any]) -> str:
    fingerprint = (doc.get("runner") or {}).get("fingerprint")
    if not fingerprint:
        raise BandError("result.json carries no runner fingerprint — it predates band schema v2; re-run the benchmark")
    return str(fingerprint)


def result_window_s(doc: Mapping[str, Any]) -> float:
    return float((doc.get("configuration") or {}).get("duration_s") or 0.0)


def result_sha(doc: Mapping[str, Any]) -> str:
    return str(doc.get("sha") or "unknown")


# --------------------------------------------------------------------------- #
# Fixed probe — time-binned connection characterization
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class ProbeBin:
    topic: str
    bin_start_s: float
    bin_end_s: float
    expected: int
    delivered: int
    lost: int
    loss_pct: float
    delivered_hz: float
    expected_hz: float
    payload_bandwidth_bps: float
    mean_size_bytes: float | None
    latency_p50_ms: float | None
    latency_p95_ms: float | None
    jitter_p50_ms: float | None
    jitter_p95_ms: float | None
    inter_arrival_p50_ms: float | None
    inter_arrival_p95_ms: float | None


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    rank = max(0, min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1))
    return round(ordered[rank], 3)


def _transit_topic_label(record: dict[str, Any]) -> str:
    topic = str(record.get("topic") or "")
    source = str(record.get("source") or "")
    target = str(record.get("target") or "")
    return f"{source}->{target}:{topic}" if source or target else topic


def _section_ms(record: dict[str, Any], *names: str) -> float | None:
    sections = record.get("sections")
    if not isinstance(sections, dict):
        return None
    for name in names:
        value = sections.get(name)
        if value is not None:
            return float(value)
    return None


def _infer_nominal_period_s(records: Sequence[dict[str, Any]], *, time_field: str) -> float | None:
    candidates: list[float] = []
    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for record in records:
        key = (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
        )
        grouped.setdefault(key, []).append(record)

    for stream in grouped.values():
        stamped = sorted(
            (record for record in stream if record.get(time_field) is not None),
            key=lambda record: int(record["seq"]),
        )
        for previous, current in zip(stamped, stamped[1:], strict=False):
            seq_delta = int(current["seq"]) - int(previous["seq"])
            time_delta = float(current[time_field]) - float(previous[time_field])
            if seq_delta > 0 and time_delta >= 0.0:
                candidates.append(time_delta / seq_delta)
    if not candidates:
        return None
    ordered = sorted(candidates)
    return ordered[len(ordered) // 2]


def _median(values: Sequence[float]) -> float:
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    return ordered[mid] if n % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def _mad(values: Sequence[float]) -> float:
    """Median absolute deviation — a spread estimate that ignores outliers."""
    center = _median(values)
    return _median([abs(v - center) for v in values])


# A fixed-probe sample is either INCLUDED (the impaired regime under test) or
# EXCLUDED (the start-up: warm-up floor, the partial packet, and any startup drop).
INCLUDED = "included"
EXCLUDED = "excluded"


def find_probe_onset(
    samples: Sequence[float | None],
    *,
    min_points: int = 12,
    step_window: int = 6,
    min_step_ratio: float = 3.0,
    min_step_ms: float = 15.0,
    settle_fraction: float = 0.6,
    settle_window: int = 6,
) -> int | None:
    """Index in ``samples`` where the clean impaired regime begins, or ``None``.

    ``samples`` is one stream ordered by send time; each entry is a delivered
    packet's latency in ms, or ``None`` for a lost packet.

    A fixed probe records packets before the tc/netem shaping is applied (the low
    transport floor), then a brief transition where the in-flight packet sees a
    partial delay and/or is dropped as the qdisc changes, then the impaired regime
    under test (a plateau *or* a rising bufferbloat ramp). This returns the first
    index from which the stream is *cleanly* in the impaired regime — a sustained
    window whose latencies are at/above a threshold set between the floor and the
    impaired level, with no loss dragging the window down. Everything before it
    (warm-up, the immature partial packet, and any startup drop) is start-up and
    should be excluded, so plot and summary agree on a clean measurement window.

    Detection is conservative: without a clear low-floor→high step (``impaired`` at
    least ``min_step_ratio``× **or** ``min_step_ms`` above the floor) it returns
    ``None`` and nothing is excluded — e.g. impairment live from the first packet,
    a no-op profile, or a smooth ramp with no warm-up.
    """
    n = len(samples)
    delivered = [s for s in samples if s is not None]
    if len(delivered) < min_points:
        return None

    window = max(1, min(step_window, len(delivered) // 2))
    # Sharpest upward step on the delivered latencies: the largest ratio between
    # the windowed median just after and just before an index. The un-impaired
    # floor is the lowest level, so the warm-up→impaired jump dominates, and
    # windowed medians stop a single ramp spike from masquerading as the step.
    step_index: int | None = None
    best_ratio = 1.0
    for i in range(window, len(delivered) - window):
        pre = _median(delivered[i - window : i])
        post = _median(delivered[i : i + window])
        if pre <= 0.0:
            continue
        ratio = post / pre
        if ratio > best_ratio:
            best_ratio = ratio
            step_index = i
    if step_index is None:
        return None

    floor = _median(delivered[max(0, step_index - window) : step_index])
    impaired = _median(delivered[step_index : step_index + window])
    step_is_real = impaired >= floor * min_step_ratio or impaired - floor >= min_step_ms
    if best_ratio < min_step_ratio or not step_is_real:
        return None

    # Threshold between floor and impaired level; ``settle_fraction`` sits it above
    # the partial packet so the immature transition sample is excluded.
    impaired_lo = floor + settle_fraction * (impaired - floor)
    hold = max(1, min(settle_window, len(delivered) // 2))

    # Onset = first delivered sample at/above the regime that *starts* a sustained
    # window mostly at/above it. A startup drop (``None``) or the partial packet
    # keeps its window from qualifying, so onset lands on the first mature packet
    # after them — the point from which the measurement is clean.
    for i in range(n):
        value = samples[i]
        if value is None or value < impaired_lo:
            continue
        ahead = samples[i : i + hold]
        if len(ahead) < hold:
            break
        good = sum(1 for s in ahead if s is not None and s >= impaired_lo)
        if good >= 0.7 * hold:
            return i
    return None


def _stream_samples(
    stream: Sequence[dict[str, Any]],
    *,
    seq0: int,
    t0: float,
    period_s: float,
    time_field: str,
) -> list[tuple[float, float | None]]:
    """One seq-ordered stream as ``(send_time, latency_or_None)`` — ``None`` = lost."""
    samples: list[tuple[float, float | None]] = []
    for record in stream:
        send_t = _send_time(record, seq0=seq0, t0=t0, period_s=period_s, time_field=time_field)
        latency = (
            None if record.get("status") == "lost" else _section_ms(record, "ota_hop_ms", "ota_hop_uncorrected_ms")
        )
        samples.append((send_t, latency))
    return samples


def exclude_probe_warmup(
    joined_records: Sequence[dict[str, Any]],
    *,
    nominal_period_s: float | None = None,
    time_field: str = "t_wrap",
) -> tuple[list[dict[str, Any]], float | None]:
    """Drop the start-up prefix from already-joined transit records.

    Returns ``(included_records, onset_send_time)`` where ``onset_send_time`` is
    the earliest per-stream impairment onset on the publish timeline — the origin
    that re-anchors "time since first publish" so ``t=0`` is the clean measurement
    start. Every record before its stream's onset (warm-up, the partial packet,
    and any startup drop) is dropped, so the summary and the raw plot agree on the
    same window. Streams with no clear warm-up step are kept whole; when no stream
    shows a step, nothing is dropped and ``onset_send_time`` is ``None``.
    """
    joined = list(joined_records)
    if not joined:
        return [], None
    period_s = (
        nominal_period_s if nominal_period_s is not None else _infer_nominal_period_s(joined, time_field=time_field)
    ) or 0.0

    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for record in joined:
        key = (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
        )
        grouped.setdefault(key, []).append(record)

    kept: list[dict[str, Any]] = []
    onsets: list[float] = []
    for _key, stream in grouped.items():
        stream = sorted(stream, key=lambda record: int(record["seq"]))
        stamped = [record for record in stream if record.get(time_field) is not None]
        anchor = min(stamped or stream, key=lambda record: int(record["seq"]))
        seq0 = int(anchor["seq"])
        t0 = float(anchor.get(time_field) or 0.0)
        samples = _stream_samples(stream, seq0=seq0, t0=t0, period_s=period_s, time_field=time_field)
        onset_index = find_probe_onset([latency for _, latency in samples])
        if onset_index is None:
            kept.extend(stream)
            continue
        onset_send = samples[onset_index][0]
        onsets.append(onset_send)
        kept.extend(record for record, (send_t, _lat) in zip(stream, samples, strict=True) if send_t >= onset_send)
    return kept, (min(onsets) if onsets else None)


def characterize_probe_records(
    records: Iterable[dict[str, Any]],
    *,
    bin_s: float = 1.0,
    nominal_period_s: float | None = None,
    topic: str = "",
    time_field: str = "t_wrap",
    exclude_warmup: bool = True,
) -> list[dict[str, Any]]:
    """Summarize one fixed probe as per-topic time bins.

    Bins use publish time, not arrival time, so latency/loss spikes line up with
    the traffic that caused them. Lost records often lack timestamps; when a
    nominal period is supplied (usually ``1 / rate_hz``), their send time is
    reconstructed from sequence number just like the recovery metrics do.

    With ``exclude_warmup`` (the default) the un-impaired warm-up plateau and the
    partial-impairment transition packet are dropped via
    :func:`exclude_probe_warmup`, and the bins are aligned to the impairment
    onset so ``bin_start_s == 0`` is where shaping is fully applied. Set it False
    to bin the raw timeline from the first publish.
    """
    if bin_s <= 0.0:
        raise ValueError("bin_s must be > 0.")

    from .transit import join_transit_records

    joined_records = join_transit_records(records)
    if not joined_records:
        return []
    period_s = (
        nominal_period_s
        if nominal_period_s is not None
        else _infer_nominal_period_s(joined_records, time_field=time_field)
    )
    period_s = float(period_s or 0.0)

    if exclude_warmup:
        joined_records, _onset = exclude_probe_warmup(joined_records, nominal_period_s=period_s, time_field=time_field)
        if not joined_records:
            return []

    grouped: dict[tuple[str, str, str], list[dict[str, Any]]] = {}
    for record in joined_records:
        key = (
            str(record.get("source") or ""),
            str(record.get("target") or ""),
            str(record.get("topic") or ""),
        )
        grouped.setdefault(key, []).append(record)

    expanded: list[tuple[str, dict[str, Any], float]] = []
    for _key, stream in sorted(grouped.items()):
        stream = sorted(stream, key=lambda record: int(record["seq"]))
        stamped = [record for record in stream if record.get(time_field) is not None]
        anchor = min(stamped or stream, key=lambda record: int(record["seq"]))
        seq0 = int(anchor["seq"])
        t0 = float(anchor.get(time_field) or 0.0)
        for record in stream:
            label = _transit_topic_label(record)
            if topic and topic not in {label, str(record.get("topic") or "")}:
                continue
            expanded.append(
                (
                    label,
                    record,
                    _send_time(record, seq0=seq0, t0=t0, period_s=period_s, time_field=time_field),
                )
            )
    if not expanded:
        return []

    first_send_s = min(send_t for _label, _record, send_t in expanded)
    buckets: dict[tuple[str, float], dict[str, Any]] = {}
    for label, record, send_t in expanded:
        relative_s = max(0.0, send_t - first_send_s)
        bucket_start = math.floor(relative_s / bin_s) * bin_s
        bucket = buckets.setdefault(
            (label, bucket_start),
            {
                "topic": label,
                "bin_start_s": bucket_start,
                "bin_end_s": bucket_start + bin_s,
                "expected": 0,
                "delivered": 0,
                "lost": 0,
                "payload_bytes": 0.0,
                "latencies": [],
                "jitters": [],
                "inter_arrivals": [],
            },
        )
        bucket["expected"] += 1
        if record.get("status") == "lost":
            bucket["lost"] += 1
            continue
        bucket["delivered"] += 1
        if record.get("size_bytes") is not None:
            bucket["payload_bytes"] += float(record["size_bytes"])
        latency_ms = _section_ms(record, "ota_hop_ms", "ota_hop_uncorrected_ms")
        if latency_ms is not None:
            bucket["latencies"].append(latency_ms)
        if record.get("jitter_ms") is not None:
            bucket["jitters"].append(float(record["jitter_ms"]))
        if record.get("inter_arrival_ms") is not None:
            bucket["inter_arrivals"].append(float(record["inter_arrival_ms"]))

    rows: list[dict[str, Any]] = []
    for (_label, _bucket_start), bucket in sorted(buckets.items()):
        expected = int(bucket["expected"])
        delivered = int(bucket["delivered"])
        lost = int(bucket["lost"])
        payload_bytes = float(bucket["payload_bytes"])
        row = ProbeBin(
            topic=str(bucket["topic"]),
            bin_start_s=round(float(bucket["bin_start_s"]), 6),
            bin_end_s=round(float(bucket["bin_end_s"]), 6),
            expected=expected,
            delivered=delivered,
            lost=lost,
            loss_pct=round(100.0 * lost / expected, 3) if expected else 0.0,
            delivered_hz=round(delivered / bin_s, 3),
            expected_hz=round(expected / bin_s, 3),
            payload_bandwidth_bps=round(payload_bytes * 8.0 / bin_s, 3),
            mean_size_bytes=round(payload_bytes / delivered, 3) if delivered else None,
            latency_p50_ms=_percentile(bucket["latencies"], 0.50),
            latency_p95_ms=_percentile(bucket["latencies"], 0.95),
            jitter_p50_ms=_percentile(bucket["jitters"], 0.50),
            jitter_p95_ms=_percentile(bucket["jitters"], 0.95),
            inter_arrival_p50_ms=_percentile(bucket["inter_arrivals"], 0.50),
            inter_arrival_p95_ms=_percentile(bucket["inter_arrivals"], 0.95),
        )
        rows.append(asdict(row))
    return rows


# --------------------------------------------------------------------------- #
# Genre 2 — perturbation / recovery metrics
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class OutageWindow:
    start: float  # seconds on the publish timeline
    end: float  # restore time


@dataclass(frozen=True)
class RecoveryMetrics:
    t_recover: float | None  # restore -> first message arriving after restore
    t_steady: float | None  # restore -> inter-arrival back within tolerance
    recovery_burst: int  # messages arriving in the burst window after restore
    lost_during_outage: dict[str, int]  # per topic
    latched_rearrival: dict[str, bool]  # per latched topic


def _send_time(record: dict[str, Any], *, seq0: int, t0: float, period_s: float, time_field: str) -> float:
    """Position on the publish timeline; reconstruct lost messages from the schedule.

    Delivered records carry ``t_wrap`` (publish time). Lost records carry no
    timestamp, so their position is the nominal send time ``t0 + (seq - seq0)·T``.
    """
    stamp = record.get(time_field)
    if stamp is not None:
        return float(stamp)
    return t0 + (int(record["seq"]) - seq0) * period_s


def _arrival_time(send_t: float, record: dict[str, Any]) -> float:
    sections = record.get("sections") or {}
    ota = sections.get("ota_hop_ms")
    if ota is None:
        ota = sections.get("ota_hop_uncorrected_ms")
    return send_t + (float(ota) / 1000.0 if ota is not None else 0.0)


def recovery_metrics(
    records: Iterable[dict[str, Any]],
    outage: OutageWindow,
    *,
    nominal_period_s: float,
    time_field: str = "t_wrap",
    steady_factor: float = 1.5,
    burst_window_s: float = 1.0,
    latched_topics: Iterable[str] = (),
) -> RecoveryMetrics:
    """Extract recovery dynamics from a timeline of transit records.

    ``t_recover`` is the delay from restore to the first message arriving at/after
    restore; ``recovery_burst`` counts arrivals within ``burst_window_s`` of restore
    (the reconnect burst the baseline calls "the part that actually hurts");
    ``t_steady`` is the delay until inter-arrival returns within
    ``steady_factor × nominal_period``; ``lost_during_outage`` counts per-topic lost
    messages whose send time falls in the outage; ``latched_rearrival`` reports
    whether each latched topic delivered a value again after restore.
    """
    records = list(records)
    if not records:
        return RecoveryMetrics(None, None, 0, {}, {topic: False for topic in latched_topics})

    stamped = [record for record in records if record.get(time_field) is not None]
    anchor = min(stamped or records, key=lambda record: int(record["seq"]))
    seq0 = int(anchor["seq"])
    t0 = float(anchor.get(time_field) or 0.0)

    lost_during_outage: dict[str, int] = {}
    delivered: list[tuple[float, dict[str, Any]]] = []
    for record in records:
        send_t = _send_time(record, seq0=seq0, t0=t0, period_s=nominal_period_s, time_field=time_field)
        topic = str(record.get("topic") or "")
        if record.get("status") == "lost":
            if outage.start <= send_t <= outage.end:
                lost_during_outage[topic] = lost_during_outage.get(topic, 0) + 1
        else:
            delivered.append((_arrival_time(send_t, record), record))

    delivered.sort(key=lambda item: item[0])
    after = [arrival for arrival, _ in delivered if arrival >= outage.end]

    t_recover = after[0] - outage.end if after else None
    recovery_burst = sum(1 for arrival in after if arrival <= outage.end + burst_window_s)

    # Steady state is inter-arrival back *near* the nominal period — a symmetric band.
    # A reconnect burst arrives in a clump (gaps far below nominal), so it is not steady.
    t_steady: float | None = None
    low_gap = nominal_period_s / steady_factor
    high_gap = nominal_period_s * steady_factor
    for index in range(1, len(after)):
        if low_gap <= after[index] - after[index - 1] <= high_gap:
            t_steady = after[index] - outage.end
            break

    latched_rearrival = {
        topic: any(record.get("topic") == topic and arrival >= outage.end for arrival, record in delivered)
        for topic in latched_topics
    }
    return RecoveryMetrics(t_recover, t_steady, recovery_burst, lost_during_outage, latched_rearrival)


# --------------------------------------------------------------------------- #
# Genre 1 — coarse linear ramp (monitor-only trend)
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class RampPoint:
    value: float
    metric: float


def linear_ramp(values: Iterable[float], measure: Callable[[float], float]) -> list[RampPoint]:
    """The whole response curve (latency-vs-load). Monitor-only: trended, never gated."""
    return [RampPoint(float(value), float(measure(value))) for value in values]


# --------------------------------------------------------------------------- #
# A/B tuning experiments (#22): candidate config vs baseline, same load+profile
# --------------------------------------------------------------------------- #
#
# The regression gate asks "is this run within a committed envelope?"; an A/B
# experiment asks the sibling question "is candidate config B better, worse or
# unchanged vs baseline config A on the *same* load and profile?". Both are
# two-sided and both speak the same Better/Verdict language: an A/B cell is just
# :func:`_two_sided_verdict` on the ephemeral ``[baseline +/- tolerance]``
# envelope. This layer is pure and host-testable; the live runs (Docker graphs,
# emulated profiles) live in the CLI, injected as a ``run_point`` (benchmark commands reference).

# The per-topic metrics an A/B experiment watches, with their better-direction.
# Names mirror the gate's band metrics (``_DEFAULT_BETTER``) so a verdict reads
# the same in both places. Read off ``transit.summarize_transit_records``.
AB_METRICS: dict[str, Better] = {
    "completeness_pct": Better.HIGHER,
    "loss_pct": Better.LOWER,
    "latency_p50_ms": Better.LOWER,
    "latency_p95_ms": Better.LOWER,
    "jitter_p50_ms": Better.LOWER,
    "jitter_p95_ms": Better.LOWER,
}

# The default watched set: the completeness/loss pair (bottleneck-dominated —
# counted off exact sequence numbers, so trustworthy even on a shared runner)
# plus the p95 tails (host-timing-dominated — directional hints, which is why the
# spread is always reported, not just the median).
DEFAULT_AB_METRICS: tuple[str, ...] = ("completeness_pct", "loss_pct", "latency_p95_ms", "jitter_p95_ms")

# A candidate median within this relative OR the metric's absolute half-width of
# the baseline median is called unchanged (WITHIN). The absolute floors keep
# sub-millisecond tail noise on a tiny baseline from reading as a change; loss
# and completeness are exact percentages and get no floor (a real 1% delta is
# signal).
DEFAULT_AB_REL_TOLERANCE = 0.10
DEFAULT_AB_ABS_TOLERANCE: dict[str, float] = {
    "latency_p50_ms": 1.0,
    "latency_p95_ms": 2.0,
    "jitter_p50_ms": 1.0,
    "jitter_p95_ms": 2.0,
}


def ab_better(metric: str) -> Better:
    """The better-direction of an A/B-watched metric (raises on an unknown one)."""
    better = AB_METRICS.get(metric)
    if better is None:
        raise BandError(f"metric {metric!r} is not an A/B-watched metric; known: {sorted(AB_METRICS)}")
    return better


def _bare_topic(label: str) -> str:
    """``a->b:/topic`` -> ``/topic``; a plain topic label is returned unchanged."""
    return label.split(":", 1)[1] if ":" in label else label


def ab_metrics_from_topic(topic_summary: Mapping[str, Any]) -> dict[str, float | None]:
    """Pull the A/B-watched metrics out of one topic's transit-summary block.

    A metric is ``None`` when the run produced no sample for it (e.g. latency
    with 100% loss); the aggregator drops ``None`` holes from a spread.
    """
    expected = topic_summary.get("expected")
    delivered = topic_summary.get("delivered")
    completeness: float | None = None
    if expected and delivered is not None:
        completeness = round(100.0 * float(delivered) / float(expected), 3)
    ota = topic_summary.get("ota_hop_ms") or {}
    jitter = topic_summary.get("jitter_ms") or {}

    def opt(value: Any) -> float | None:
        return None if value is None else float(value)

    return {
        "completeness_pct": completeness,
        "loss_pct": opt(topic_summary.get("loss_pct")),
        "latency_p50_ms": opt(ota.get("p50")),
        "latency_p95_ms": opt(ota.get("p95")),
        "jitter_p50_ms": opt(jitter.get("p50")),
        "jitter_p95_ms": opt(jitter.get("p95")),
    }


@dataclass(frozen=True)
class AbTolerance:
    """Half-width of the unchanged band around a baseline value: ``max(abs, rel*|baseline|)``."""

    rel: float = DEFAULT_AB_REL_TOLERANCE
    abs: float = 0.0

    def half_width(self, baseline: float) -> float:
        return max(self.abs, abs(baseline) * self.rel)


def default_ab_tolerance(metric: str, *, rel: float = DEFAULT_AB_REL_TOLERANCE) -> AbTolerance:
    """The default tolerance for a metric: ``rel`` plus the metric's absolute floor."""
    return AbTolerance(rel=rel, abs=DEFAULT_AB_ABS_TOLERANCE.get(metric, 0.0))


def classify_change(baseline: float, candidate: float, *, better: Better, tol: AbTolerance) -> Verdict:
    """Candidate vs baseline as WITHIN / IMPROVED / REGRESSED.

    ``_two_sided_verdict`` on the ephemeral ``[baseline +/- tol]`` envelope, so an
    A/B cell classifies identically to a committed-band gate cell.
    """
    half = tol.half_width(baseline)
    return _two_sided_verdict(candidate, lo=baseline - half, hi=baseline + half, better=better)


def ab_schedule(configs: Sequence[str], repeats: int) -> list[tuple[str, int]]:
    """Deterministic interleaving of ``configs`` over ``repeats`` as ``(config, repeat)``.

    Every repeat runs each config exactly once; the starting config rotates each
    repeat so no config is always measured first. Interleaving (not "all of A,
    then all of B") spreads any slow host-drift across every config, and the
    rotation removes the residual first-slot warm-up bias — both guard the verdict
    against drift being mistaken for a config effect. Deterministic by
    construction, so a verdict is reproducible from the same runs.
    """
    if repeats < 1:
        raise ValueError("repeats must be >= 1.")
    if not configs:
        raise ValueError("need at least one config.")
    order: list[tuple[str, int]] = []
    count = len(configs)
    for repeat in range(1, repeats + 1):
        rotation = (repeat - 1) % count
        rotated = [*configs[rotation:], *configs[:rotation]]
        order.extend((config, repeat) for config in rotated)
    return order


@dataclass(frozen=True)
class MetricSpread:
    """Repeat-to-repeat spread of one metric for one config+topic."""

    n: int  # repeats that produced a value
    min: float
    median: float
    max: float
    values: list[float]  # per-repeat, in execution order


def _metric_spread(values: Sequence[float]) -> MetricSpread | None:
    vals = [float(v) for v in values]
    if not vals:
        return None
    return MetricSpread(
        n=len(vals),
        min=round(min(vals), 3),
        median=round(_median(vals), 3),
        max=round(max(vals), 3),
        values=[round(v, 3) for v in vals],
    )


@dataclass(frozen=True)
class AbRun:
    """One executed run in an experiment: which config, which 1-based repeat, its summary."""

    config: str
    repeat: int
    summary: Mapping[str, Any]  # transit.summarize_transit_records output


@dataclass(frozen=True)
class AbCell:
    """One (topic, metric) candidate-vs-baseline verdict, both spreads carried."""

    topic: str
    metric: str
    better: str  # Better value
    baseline: MetricSpread | None
    candidate: MetricSpread | None
    verdict: str | None  # Verdict value; None when a side has no sample
    delta: float | None  # candidate.median - baseline.median
    separated: bool | None  # spreads disjoint => effect separable at this N


@dataclass(frozen=True)
class CandidateReport:
    """A candidate config's full comparison against the baseline."""

    config: str
    cells: list[AbCell]
    regressed: list[list[str]]  # [topic, metric] pairs that regressed beyond tolerance
    improved: list[list[str]]  # [topic, metric] pairs that improved beyond tolerance
    dropped_topics: list[str]  # topics the baseline delivered that this config did not
    passed: bool  # no regression and no dropped topic


@dataclass(frozen=True)
class AbReport:
    baseline: str
    metrics: list[str]
    repeats: int
    candidates: list[CandidateReport]
    passed: bool  # every candidate passed


def _collect_ab_metrics(
    runs: Iterable[AbRun], *, metrics: Sequence[str], topic_filter: str
) -> dict[str, dict[str, dict[str, list[float]]]]:
    """``{config: {topic: {metric: [per-repeat values]}}}`` in execution order."""
    collected: dict[str, dict[str, dict[str, list[float]]]] = {}
    for run in runs:
        topics = run.summary.get("topics") or {}
        for topic_label, topic_summary in topics.items():
            if topic_filter and topic_filter not in {topic_label, _bare_topic(topic_label)}:
                continue
            values = ab_metrics_from_topic(topic_summary)
            per_topic = collected.setdefault(run.config, {}).setdefault(topic_label, {})
            for metric in metrics:
                value = values.get(metric)
                if value is not None:
                    per_topic.setdefault(metric, []).append(value)
    return collected


def ab_verdict(
    runs: Sequence[AbRun],
    *,
    baseline: str,
    metrics: Sequence[str] = DEFAULT_AB_METRICS,
    tolerances: Mapping[str, AbTolerance] | None = None,
    topic: str = "",
) -> AbReport:
    """Classify every candidate config against the baseline, per topic and metric.

    Comparison is on the per-config **median** of each metric across repeats (one
    unlucky repeat cannot flip a verdict), and both spreads travel with the cell
    so a reader sees whether the medians are actually separable at this N
    (``separated``). A candidate ``passed`` iff nothing regressed beyond tolerance
    and it did not drop a topic the baseline delivered.
    """
    metric_list = list(metrics)
    tol_map = {metric: (tolerances or {}).get(metric) or default_ab_tolerance(metric) for metric in metric_list}
    ordered = sorted(runs, key=lambda run: (run.config, run.repeat))
    collected = _collect_ab_metrics(ordered, metrics=metric_list, topic_filter=topic)

    if baseline not in collected:
        raise BandError(f"baseline config {baseline!r} produced no runs to compare against.")

    configs_in_order: list[str] = []
    for run in ordered:
        if run.config not in configs_in_order:
            configs_in_order.append(run.config)
    repeats = max((run.repeat for run in ordered), default=0)
    baseline_topics = collected[baseline]

    candidates: list[CandidateReport] = []
    for config in configs_in_order:
        if config == baseline:
            continue
        candidate_topics = collected.get(config, {})
        cells: list[AbCell] = []
        regressed: list[list[str]] = []
        improved: list[list[str]] = []
        dropped: list[str] = []
        for topic_label in sorted(baseline_topics):
            candidate_metrics = candidate_topics.get(topic_label)
            if candidate_metrics is None:
                dropped.append(topic_label)
                continue
            baseline_metrics = baseline_topics[topic_label]
            for metric in metric_list:
                better = ab_better(metric)
                baseline_spread = _metric_spread(baseline_metrics.get(metric, []))
                candidate_spread = _metric_spread(candidate_metrics.get(metric, []))
                verdict: Verdict | None = None
                delta: float | None = None
                separated: bool | None = None
                if baseline_spread is not None and candidate_spread is not None:
                    verdict = classify_change(
                        baseline_spread.median, candidate_spread.median, better=better, tol=tol_map[metric]
                    )
                    delta = round(candidate_spread.median - baseline_spread.median, 3)
                    separated = candidate_spread.max < baseline_spread.min or baseline_spread.max < candidate_spread.min
                    if verdict is Verdict.REGRESSED:
                        regressed.append([topic_label, metric])
                    elif verdict is Verdict.IMPROVED:
                        improved.append([topic_label, metric])
                cells.append(
                    AbCell(
                        topic=topic_label,
                        metric=metric,
                        better=better.value,
                        baseline=baseline_spread,
                        candidate=candidate_spread,
                        verdict=None if verdict is None else verdict.value,
                        delta=delta,
                        separated=separated,
                    )
                )
        candidates.append(
            CandidateReport(
                config=config,
                cells=cells,
                regressed=regressed,
                improved=improved,
                dropped_topics=dropped,
                passed=not regressed and not dropped,
            )
        )

    return AbReport(
        baseline=baseline,
        metrics=metric_list,
        repeats=repeats,
        candidates=candidates,
        passed=all(candidate.passed for candidate in candidates),
    )


def _fmt_spread(spread: MetricSpread | None) -> str:
    if spread is None:
        return "n/a"
    if spread.min == spread.max:
        return f"{spread.median:g}"
    return f"{spread.median:g} ({spread.min:g}–{spread.max:g})"


def render_ab_markdown(report: AbReport) -> str:
    """A self-describing markdown table per candidate: the paper-grade byproduct."""
    lines: list[str] = []
    verdict_word = "PASS" if report.passed else "FAIL"
    lines.append(f"# A/B verdict: **{verdict_word}** (baseline `{report.baseline}`, {report.repeats} repeats)")
    lines.append("")
    lines.append(
        "Cells compare the candidate median against the baseline median; "
        "`baseline`/`candidate` show `median (min–max)` across repeats. "
        "`sep?` is yes when the two spreads do not overlap (the effect is "
        "separable at this repeat count)."
    )
    for candidate in report.candidates:
        lines.append("")
        status = "PASS" if candidate.passed else "FAIL"
        lines.append(f"## `{candidate.config}` vs `{report.baseline}` — **{status}**")
        if candidate.dropped_topics:
            lines.append("")
            lines.append(
                f"Dropped topics (baseline delivered, candidate did not): {', '.join(candidate.dropped_topics)}"
            )
        lines.append("")
        lines.append("| topic | metric | better | baseline | candidate | Δ | sep? | verdict |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for cell in candidate.cells:
            delta = "" if cell.delta is None else f"{cell.delta:+g}"
            sep = "" if cell.separated is None else ("yes" if cell.separated else "no")
            verdict = cell.verdict or "n/a"
            lines.append(
                f"| {cell.topic} | {cell.metric} | {cell.better} | "
                f"{_fmt_spread(cell.baseline)} | {_fmt_spread(cell.candidate)} | {delta} | {sep} | {verdict} |"
            )
    return "\n".join(lines)
