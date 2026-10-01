# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The benched set: the curated registry of gated benchmark rows.

The registry is a whitelist, not a crawler: every row is one deliberate
``(rmw × profile × load × metric set)`` combination with an operator-visible
reason to exist. Workflows and the ``benchmark`` CLI *read* this registry —
adding an RMW variant or profile extends the matrix without editing CI, and a
matrix nobody can afford to keep green is a monitor with extra steps.

Three genres run here. ``probe`` and ``capacity`` are synthetic: the load is
whatever the row says it is. ``replay`` is not — its load is derived from a named
recording, so the row also carries that bag's provenance and its whole-bag
``expect`` fragment, both of which are checked: the provenance against the load
at load time (so the shape cannot drift away from the bag it claims), the expect
fragment against the run (so a replay that did not arrive is red without waiting
for a calibrated band).

This module is pure and host-testable: loading, structural validation, the
whole-bag expect evaluation, and the machine-readable per-run verdict document.
Cross-file checks (profiles exist, bands committed, workflows consume the
registry) live in the contract tests.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path
from typing import Any

import yaml

REGISTRY_SCHEMA = 2
REGISTRY_RESOURCE = "resources/benched-set.yaml"

LANES = ("merge-gate", "nightly")
ROW_KINDS = ("performance", "boundary")
GENRES = ("probe", "capacity", "replay")

# The transit metrics each genre can put under a band today. The metric policy
# (performance bands reference) prefers bottleneck-dominated metrics for tight public bands;
# latency percentiles on shared runners are monitor-only unless calibration
# proves otherwise.
GENRE_METRICS: dict[str, tuple[str, ...]] = {
    "probe": ("completeness_pct", "loss_pct", "latency_p50_ms", "latency_p95_ms", "payload_bandwidth_bps"),
    "capacity": ("capacity_size", "capacity_rate", "capacity_bandwidth"),
    "replay": (
        "completeness_pct",
        "loss_pct",
        "latency_p50_ms",
        "latency_p95_ms",
        "payload_bandwidth_bps",
        "delivered_count",
        "delivered_hz",
    ),
}

PROBE_LOAD_KEYS = frozenset(
    {"size", "size_pattern", "rate_hz", "streams", "interval_jitter_ms", "interval_jitter_seed"}
)
CAPACITY_KNOBS = ("size", "rate", "bandwidth")

# How far a replay row's committed load may drift from the bag provenance it
# claims to reproduce. The cadence and the replay jitter are measured directly
# and must match closely; the mean payload is looser because a GOP `size_pattern`
# approximates a real frame-size distribution with two sizes (performance bands reference).
REPLAY_RATE_TOLERANCE = 0.02
REPLAY_JITTER_TOLERANCE = 0.05
REPLAY_SIZE_TOLERANCE = 0.10

# A gated window must span >=2 CycloneDDS SPDP discovery periods (2 x 30 s,
# benchmark commands reference "discovery traffic is real load") or explicitly annotate why not.
MIN_GATED_WINDOW_S = 60.0

VERDICT_SCHEMA = 1


class RegistryError(ValueError):
    """The registry refused to load; the message names the offending row/field."""


@dataclass(frozen=True)
class GateRow:
    """One benched row: what runs, under what, and which metrics gate."""

    id: str
    lane: str
    reason: str
    rmw: str
    genre: str
    profile: str
    duration_s: float
    metrics: tuple[str, ...]  # banded — these gate
    monitor: tuple[str, ...] = ()  # recorded in the verdict, never gated
    load: dict[str, Any] = field(default_factory=dict)  # probe load parameters
    search: dict[str, Any] = field(default_factory=dict)  # capacity binary-search bounds
    oracle: dict[str, Any] = field(default_factory=dict)  # capacity pass/fail thresholds
    replay: dict[str, Any] = field(default_factory=dict)  # replay provenance/expect fragment for replay-derived rows
    floors: dict[str, float] = field(default_factory=dict)  # committed minimum half-widths per gated metric
    repeats: int = 1  # in-run repeats (median vote / attempts)
    window_note: str = ""  # why a sub-60 s window is still a valid gate
    kind: str = "performance"  # performance = normal band row; boundary = good/bad envelope assertion
    boundary: dict[str, Any] = field(default_factory=dict)


def packaged_registry_path() -> Path:
    """The committed registry (packaged with the wheel, same file as in-repo)."""
    return Path(str(resources.files("rosotacom").joinpath(REGISTRY_RESOURCE)))


def _require_str(row_id: str, raw: dict[str, Any], key: str) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or not value.strip():
        raise RegistryError(f"row {row_id!r}: {key!r} must be a non-empty string")
    return value.strip()


def _string_tuple(row_id: str, raw: dict[str, Any], key: str) -> tuple[str, ...]:
    value = raw.get(key, [])
    if not isinstance(value, list) or not all(isinstance(item, str) and item.strip() for item in value):
        raise RegistryError(f"row {row_id!r}: {key!r} must be a list of metric names")
    return tuple(item.strip() for item in value)


_ROW_KEYS = frozenset(
    {
        "id",
        "kind",
        "lane",
        "reason",
        "rmw",
        "genre",
        "profile",
        "duration_s",
        "metrics",
        "monitor",
        "load",
        "search",
        "oracle",
        "replay",
        "floors",
        "repeats",
        "window_note",
        "boundary",
    }
)


def _threshold_rules(
    row_id: str,
    boundary: dict[str, Any],
    key: str,
    metrics: tuple[str, ...],
) -> dict[str, dict[str, float]]:
    raw = boundary.get(key)
    if not isinstance(raw, dict) or not raw:
        raise RegistryError(f"row {row_id!r}: boundary.{key!r} must be a non-empty metric threshold mapping")
    parsed: dict[str, dict[str, float]] = {}
    for metric, rule in raw.items():
        metric_name = str(metric)
        if metric_name not in metrics:
            raise RegistryError(f"row {row_id!r}: boundary.{key!r} references unbanded metric {metric_name!r}")
        if not isinstance(rule, dict) or not rule:
            raise RegistryError(f"row {row_id!r}: boundary.{key!r}.{metric_name!r} must be a threshold mapping")
        unknown = sorted(set(rule) - {"min", "max"})
        if unknown:
            raise RegistryError(f"row {row_id!r}: boundary.{key!r}.{metric_name!r} has unknown keys {unknown}")
        parsed_rule: dict[str, float] = {}
        for side in ("min", "max"):
            if side not in rule:
                continue
            value = rule[side]
            if not isinstance(value, int | float):
                raise RegistryError(f"row {row_id!r}: boundary.{key!r}.{metric_name!r}.{side} must be numeric")
            parsed_rule[side] = float(value)
        if not parsed_rule:
            raise RegistryError(f"row {row_id!r}: boundary.{key!r}.{metric_name!r} needs min and/or max")
        if "min" in parsed_rule and "max" in parsed_rule and parsed_rule["min"] > parsed_rule["max"]:
            raise RegistryError(f"row {row_id!r}: boundary.{key!r}.{metric_name!r} has min > max")
        parsed[metric_name] = parsed_rule
    return parsed


def _parse_boundary(row_id: str, raw: dict[str, Any], metrics: tuple[str, ...]) -> dict[str, Any]:
    boundary = raw.get("boundary") or {}
    if not isinstance(boundary, dict):
        raise RegistryError(f"row {row_id!r}: 'boundary' must be a mapping")
    expected = {"finding", "bad_profile", "good_oracle", "failure_signature", "next_steps"}
    unknown = sorted(set(boundary) - expected)
    if unknown:
        raise RegistryError(f"row {row_id!r}: boundary has unknown keys {unknown}")
    missing = sorted(key for key in expected if key not in boundary)
    if missing:
        raise RegistryError(f"row {row_id!r}: boundary is missing {missing}")
    finding = str(boundary["finding"]).strip()
    bad_profile = str(boundary["bad_profile"]).strip()
    next_steps = str(boundary["next_steps"]).strip()
    if not finding:
        raise RegistryError(f"row {row_id!r}: boundary.finding must be a non-empty string")
    if not bad_profile:
        raise RegistryError(f"row {row_id!r}: boundary.bad_profile must be a non-empty string")
    if not next_steps:
        raise RegistryError(f"row {row_id!r}: boundary.next_steps must be a non-empty string")
    return {
        "finding": finding,
        "bad_profile": bad_profile,
        "good_oracle": _threshold_rules(row_id, boundary, "good_oracle", metrics),
        "failure_signature": _threshold_rules(row_id, boundary, "failure_signature", metrics),
        "next_steps": next_steps,
    }


def _parse_row(raw: Any) -> GateRow:
    if not isinstance(raw, dict):
        raise RegistryError(f"registry rows must be mappings, got {type(raw).__name__}")
    row_id = str(raw.get("id") or "").strip()
    if not row_id:
        raise RegistryError("every registry row needs an 'id'")
    unknown = sorted(set(raw) - _ROW_KEYS)
    if unknown:
        raise RegistryError(f"row {row_id!r}: unknown keys {unknown} — the registry is a whitelist, extend it in code")

    kind = _require_str(row_id, raw, "kind")
    if kind not in ROW_KINDS:
        raise RegistryError(f"row {row_id!r}: kind {kind!r} is not one of {list(ROW_KINDS)}")
    lane = _require_str(row_id, raw, "lane")
    if lane not in LANES:
        raise RegistryError(f"row {row_id!r}: lane {lane!r} is not one of {list(LANES)}")
    genre = _require_str(row_id, raw, "genre")
    if genre not in GENRES:
        raise RegistryError(f"row {row_id!r}: genre {genre!r} is not one of {list(GENRES)}")
    reason = _require_str(row_id, raw, "reason")

    duration_raw = raw.get("duration_s")
    if not isinstance(duration_raw, int | float) or duration_raw <= 0:
        raise RegistryError(f"row {row_id!r}: 'duration_s' must be a positive number")
    duration_s = float(duration_raw)

    metrics = _string_tuple(row_id, raw, "metrics")
    # A replay row still gates without a band: its whole-bag `expect` fragment is
    # bag-derived, not runner-derived, so it asserts from the day the row lands
    # and the bands follow from a calibration run (performance bands reference).
    if not metrics and genre != "replay":
        raise RegistryError(f"row {row_id!r}: a gated row needs at least one banded metric")
    monitor = _string_tuple(row_id, raw, "monitor")
    producible = set(GENRE_METRICS[genre])
    for name, values in (("metrics", metrics), ("monitor", monitor)):
        unknown_metrics = sorted(set(values) - producible)
        if unknown_metrics:
            raise RegistryError(
                f"row {row_id!r}: {name} {unknown_metrics} are not produced by genre {genre!r} "
                f"(known: {sorted(producible)})"
            )
    overlap = sorted(set(metrics) & set(monitor))
    if overlap:
        raise RegistryError(f"row {row_id!r}: {overlap} cannot be both gated and monitor-only")

    load = raw.get("load") or {}
    search = raw.get("search") or {}
    oracle = raw.get("oracle") or {}
    replay = raw.get("replay") or {}
    floors = raw.get("floors") or {}
    for name, value in (
        ("load", load),
        ("search", search),
        ("oracle", oracle),
        ("replay", replay),
        ("floors", floors),
    ):
        if not isinstance(value, dict):
            raise RegistryError(f"row {row_id!r}: {name!r} must be a mapping")
    for metric, floor in floors.items():
        if metric not in metrics:
            raise RegistryError(f"row {row_id!r}: floor for {metric!r} but only gated metrics take committed floors")
        if not isinstance(floor, int | float) or floor <= 0:
            raise RegistryError(f"row {row_id!r}: floor for {metric!r} must be a positive number")

    if kind == "performance":
        if raw.get("boundary"):
            raise RegistryError(f"row {row_id!r}: performance rows do not carry boundary metadata")
        boundary: dict[str, Any] = {}
    else:
        boundary = _parse_boundary(row_id, raw, metrics)

    repeats_raw = raw.get("repeats", 1)
    if not isinstance(repeats_raw, int) or repeats_raw < 1:
        raise RegistryError(f"row {row_id!r}: 'repeats' must be an integer >= 1")

    window_note = str(raw.get("window_note") or "").strip()
    if duration_s < MIN_GATED_WINDOW_S and not window_note:
        raise RegistryError(
            f"row {row_id!r}: window {duration_s:g}s is shorter than {MIN_GATED_WINDOW_S:g}s "
            "(2 discovery periods, performance bands reference): lengthen it or add a 'window_note' "
            "saying why it still gates"
        )

    row = GateRow(
        id=row_id,
        lane=lane,
        reason=reason,
        rmw=_require_str(row_id, raw, "rmw"),
        genre=genre,
        profile=_require_str(row_id, raw, "profile"),
        duration_s=duration_s,
        metrics=metrics,
        monitor=monitor,
        load=dict(load),
        search=dict(search),
        oracle=dict(oracle),
        replay=dict(replay),
        floors={str(metric): float(floor) for metric, floor in floors.items()},
        repeats=repeats_raw,
        window_note=window_note,
        kind=kind,
        boundary=boundary,
    )
    _validate_genre_parameters(row)
    return row


def _validate_genre_parameters(row: GateRow) -> None:
    if row.genre in ("probe", "replay"):
        if row.search or row.oracle:
            raise RegistryError(f"row {row.id!r}: 'search'/'oracle' belong to capacity rows")
        unknown = sorted(set(row.load) - PROBE_LOAD_KEYS)
        if unknown:
            raise RegistryError(f"row {row.id!r}: unknown probe load keys {unknown} (known: {sorted(PROBE_LOAD_KEYS)})")
        if ("size" in row.load) == ("size_pattern" in row.load):
            raise RegistryError(f"row {row.id!r}: a probe load needs exactly one of 'size' or 'size_pattern'")
        if float(row.load.get("rate_hz") or 0.0) <= 0.0:
            raise RegistryError(f"row {row.id!r}: probe load needs a positive 'rate_hz'")
        # Seed policy is explicit (vision): a seeded-random load must commit its seed.
        if float(row.load.get("interval_jitter_ms") or 0.0) > 0.0 and "interval_jitter_seed" not in row.load:
            raise RegistryError(
                f"row {row.id!r}: interval jitter without 'interval_jitter_seed' — gated rows commit their seeds"
            )
        if row.genre == "probe":
            # Replay provenance is what makes a row a replay row; carrying it on a
            # synthetic probe row hides the difference the two genres exist to make.
            if row.replay:
                raise RegistryError(f"row {row.id!r}: replay metadata belongs to replay rows (genre: replay)")
            return
        _validate_replay_metadata(row)
        _validate_replay_load_matches_bag(row)
        return

    # capacity
    if row.replay:
        raise RegistryError(f"row {row.id!r}: replay metadata belongs to replay rows")
    if row.load:
        raise RegistryError(f"row {row.id!r}: capacity rows configure 'search'/'oracle', not 'load'")
    knob = str(row.search.get("knob") or "")
    if knob not in CAPACITY_KNOBS:
        raise RegistryError(f"row {row.id!r}: search knob {knob!r} is not one of {list(CAPACITY_KNOBS)}")
    low, high = row.search.get("low"), row.search.get("high")
    if not isinstance(low, int) or not isinstance(high, int) or not 0 < low <= high:
        raise RegistryError(f"row {row.id!r}: search needs integers 0 < low <= high")
    if float(row.search.get("rate_hz") or 0.0) <= 0.0:
        raise RegistryError(f"row {row.id!r}: search needs a positive 'rate_hz'")
    for key in ("max_loss_pct", "max_latency_ms"):
        if not isinstance(row.oracle.get(key), int | float):
            raise RegistryError(f"row {row.id!r}: oracle needs numeric {key!r}")
    expected_metric = f"capacity_{knob}"
    if set(row.metrics) != {expected_metric}:
        raise RegistryError(
            f"row {row.id!r}: a capacity row bands exactly [{expected_metric!r}] (the breakpoint of its knob)"
        )


def _validate_replay_metadata(row: GateRow) -> None:
    if not row.replay:
        raise RegistryError(
            f"row {row.id!r}: a replay row needs 'replay' provenance naming the bag it reproduces "
            "(source, public_example, bag_topic, native_*, size_basis, expect)"
        )
    required = {
        "source",
        "public_example",
        "bag_topic",
        "native_count",
        "native_duration_s",
        "native_hz",
        "size_basis",
        "size_mean_bytes",
        "expect",
    }
    missing = sorted(required - set(row.replay))
    if missing:
        raise RegistryError(f"row {row.id!r}: replay metadata missing {missing}")
    for key in ("native_count", "native_duration_s", "native_hz", "size_mean_bytes"):
        value = row.replay[key]
        if not isinstance(value, int | float) or value <= 0:
            raise RegistryError(f"row {row.id!r}: replay.{key} must be a positive number")
    expect = row.replay.get("expect")
    if not isinstance(expect, dict):
        raise RegistryError(f"row {row.id!r}: replay.expect must be a mapping")
    for key in ("min_count", "completeness_min_ratio", "vs_bag_ratio"):
        if key not in expect:
            raise RegistryError(f"row {row.id!r}: replay.expect missing {key!r}")
    if int(expect["min_count"]) < 1:
        raise RegistryError(f"row {row.id!r}: replay.expect.min_count must be positive")
    for key in ("completeness_min_ratio", "vs_bag_ratio"):
        value = float(expect[key])
        if not 0.0 < value <= 1.0:
            raise RegistryError(f"row {row.id!r}: replay.expect.{key} must be in (0, 1]")
    # The generator that mints these fragments takes `min_count` from the bag's own
    # message count; a hand-edited one that no longer does is asserting a threshold
    # the bag never justified.
    native_count = int(row.replay["native_count"])
    expected_min_count = int(math.floor(native_count * float(expect["completeness_min_ratio"])))
    if int(expect["min_count"]) != expected_min_count:
        raise RegistryError(
            f"row {row.id!r}: replay.expect.min_count {int(expect['min_count'])} is not "
            f"floor(native_count {native_count} x completeness_min_ratio "
            f"{float(expect['completeness_min_ratio']):g}) = {expected_min_count}"
        )


def replay_load_mean_bytes(load: dict[str, Any]) -> float:
    """Mean payload of one replay load cycle — `size`, or the pattern's mean."""
    if "size" in load:
        return float(load["size"])
    from .benchmark import parse_size_pattern_load

    sizes = parse_size_pattern_load(str(load["size_pattern"]))["sizes"]
    return sum(float(size) for size in sizes) / len(sizes)


def _relative_drift(value: float, reference: float) -> float:
    return abs(value - reference) / reference


def _validate_replay_load_matches_bag(row: GateRow) -> None:
    """The committed load must still be the bag's shape.

    Provenance nobody checks is decoration: without this, the `replay` block can
    keep naming a 10 Hz, 8.3 KB stream while the load drifts to something else,
    and every downstream claim about "the S2 shape" quietly becomes false.
    """
    replay = row.replay
    native_hz = float(replay["native_hz"])
    rate_hz = float(row.load["rate_hz"])
    if _relative_drift(rate_hz, native_hz) > REPLAY_RATE_TOLERANCE:
        raise RegistryError(
            f"row {row.id!r}: load rate {rate_hz:g} Hz is more than "
            f"{REPLAY_RATE_TOLERANCE:.0%} off the bag's native {native_hz:g} Hz"
        )

    native_duration_s = float(replay["native_duration_s"])
    if row.duration_s < native_duration_s:
        raise RegistryError(
            f"row {row.id!r}: window {row.duration_s:g}s is shorter than the bag it replays "
            f"({native_duration_s:g}s) — a whole-bag expect fragment needs the whole bag"
        )

    size_mean_bytes = float(replay["size_mean_bytes"])
    load_mean_bytes = replay_load_mean_bytes(row.load)
    if _relative_drift(load_mean_bytes, size_mean_bytes) > REPLAY_SIZE_TOLERANCE:
        raise RegistryError(
            f"row {row.id!r}: load mean payload {load_mean_bytes:.0f} B is more than "
            f"{REPLAY_SIZE_TOLERANCE:.0%} off the bag's mean {size_mean_bytes:.0f} B"
        )

    interval_std_ms = replay.get("interval_std_ms")
    jitter_ms = float(row.load.get("interval_jitter_ms") or 0.0)
    if interval_std_ms is None:
        if jitter_ms > 0.0:
            raise RegistryError(
                f"row {row.id!r}: load commits {jitter_ms:g} ms interval jitter but replay metadata "
                "records no 'interval_std_ms' to justify it"
            )
        return
    if _relative_drift(jitter_ms, float(interval_std_ms)) > REPLAY_JITTER_TOLERANCE:
        raise RegistryError(
            f"row {row.id!r}: load interval jitter {jitter_ms:g} ms is more than "
            f"{REPLAY_JITTER_TOLERANCE:.0%} off the bag's measured {float(interval_std_ms):g} ms"
        )


def replay_expect_failures(row: GateRow, *, expected: int, delivered: int, duration_s: float) -> list[str]:
    """Whole-bag expect fragment vs. one replay run — the reasons it did not hold.

    Bag-derived, so it asserts on the runner class from the day the row lands:
    unlike a band, none of these thresholds is a measurement of the runner.
    """
    expect = row.replay["expect"]
    failures: list[str] = []

    min_count = int(expect["min_count"])
    if delivered < min_count:
        failures.append(f"delivered {delivered} msgs < replay.expect.min_count {min_count}")

    min_ratio = float(expect["completeness_min_ratio"])
    if expected > 0:
        completeness = delivered / expected
        if completeness < min_ratio:
            failures.append(f"completeness {completeness:.3f} < replay.expect.completeness_min_ratio {min_ratio:g}")
    else:
        failures.append("the run carries no expected message count — completeness cannot be asserted")

    vs_bag_ratio = float(expect["vs_bag_ratio"])
    native_hz = float(row.replay["native_hz"])
    if duration_s > 0.0:
        delivered_hz = delivered / duration_s
        if delivered_hz < vs_bag_ratio * native_hz:
            failures.append(
                f"delivered {delivered_hz:.2f} Hz < replay.expect.vs_bag_ratio {vs_bag_ratio:g} "
                f"x native {native_hz:g} Hz"
            )
    else:
        failures.append("the run carries no window length — the bag-relative rate cannot be asserted")

    return failures


def load_registry(path: Path | None = None) -> list[GateRow]:
    """Load and structurally validate the benched set. Refusals name the row."""
    registry_path = path if path is not None else packaged_registry_path()
    doc = yaml.safe_load(registry_path.read_text(encoding="utf-8"))
    if not isinstance(doc, dict):
        raise RegistryError(f"{registry_path}: registry must be a mapping")
    if doc.get("version") != REGISTRY_SCHEMA:
        raise RegistryError(f"{registry_path}: registry version {doc.get('version')!r} != {REGISTRY_SCHEMA}")
    raw_rows = doc.get("rows")
    if not isinstance(raw_rows, list) or not raw_rows:
        raise RegistryError(f"{registry_path}: registry needs a non-empty 'rows' list")
    rows = [_parse_row(raw) for raw in raw_rows]
    seen: set[str] = set()
    for row in rows:
        if row.id in seen:
            raise RegistryError(f"duplicate row id {row.id!r}; row ids key committed bands")
        seen.add(row.id)
    return rows


def rows_for_lane(rows: list[GateRow], lane: str | None) -> list[GateRow]:
    if lane is None:
        return list(rows)
    if lane not in LANES:
        raise RegistryError(f"lane {lane!r} is not one of {list(LANES)}")
    return [row for row in rows if row.lane == lane]


def rows_for_calibration(rows: list[GateRow]) -> list[GateRow]:
    """Rows that mint committed two-sided bands through the calibration workflow."""
    return [row for row in rows if row.kind == "performance"]


def profiles_for_row(row: GateRow) -> tuple[str, ...]:
    profiles = [row.profile]
    if row.kind == "boundary":
        profiles.append(str(row.boundary["bad_profile"]))
    return tuple(profiles)


def find_row(rows: list[GateRow], row_id: str) -> GateRow:
    for row in rows:
        if row.id == row_id:
            return row
    known = ", ".join(row.id for row in rows)
    raise RegistryError(f"no benched row {row_id!r} in the registry (rows: {known})")


def verdict_document(
    row: GateRow,
    *,
    verdict: str,
    exit_code: int,
    gate: bool,
    sha: str,
    fingerprint: str,
    created_at: str,
    metrics: dict[str, float],
    monitor_metrics: dict[str, float],
    bands: dict[str, dict[str, Any]],
    result_path: str,
    refusal: str | None = None,
    ratchet_command: str | None = None,
    boundary: dict[str, Any] | None = None,
    replay_expect: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """The machine-readable per-run verdict a downstream gate can consume."""
    doc = {
        "schema": VERDICT_SCHEMA,
        "kind": "benchmark-gate-verdict",
        "row": row.id,
        "row_kind": row.kind,
        "lane": row.lane,
        "rmw": row.rmw,
        "genre": row.genre,
        "profile": row.profile,
        "verdict": verdict,
        "exit_code": exit_code,
        "gate": gate,
        "sha": sha,
        "fingerprint": fingerprint,
        "created_at": created_at,
        "metrics": metrics,
        "monitor_metrics": monitor_metrics,
        "bands": bands,
        "result": result_path,
        "refusal": refusal,
        "ratchet_command": ratchet_command,
    }
    if boundary is not None:
        doc["boundary"] = boundary
    if replay_expect is not None:
        doc["replay_expect"] = replay_expect
    return doc


def write_verdict(path: Path, doc: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def summarize_verdicts(
    expected_rows: list[GateRow],
    verdicts: dict[str, dict[str, Any]],
    *,
    run: dict[str, Any],
) -> dict[str, Any]:
    """Aggregate per-row verdicts into the one document a promotion gate reads.

    Failure semantics (performance bands reference): a missing verdict is a setup failure and
    red; ``REGRESSED`` is red; ``IMPROVED`` is red too — the fix is the ratchet
    command recorded in the row's verdict, not a revert.
    """
    rows_out: list[dict[str, Any]] = []
    red: list[str] = []
    for row in expected_rows:
        doc = verdicts.get(row.id)
        if doc is None:
            rows_out.append({"row": row.id, "verdict": "MISSING", "exit_code": None, "gate": True})
            red.append(row.id)
            continue
        rows_out.append(doc)
        gate = bool(doc.get("gate", True))
        if gate and (doc.get("verdict") != "WITHIN" or doc.get("exit_code") != 0):
            red.append(row.id)
    return {
        "schema": VERDICT_SCHEMA,
        "kind": "benchmark-gate-summary",
        "run": run,
        "overall": "red" if red else "green",
        "red_rows": red,
        "rows": rows_out,
    }
