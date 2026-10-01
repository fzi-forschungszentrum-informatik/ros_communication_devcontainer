# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Per-fragment loss from transit records: the size law.

A message of n DDS fragments is lost if any required fragment is missing.
The independent-loss model gives probability 1-(1-p)^n. Its fitted p is an
effective message-loss parameter, not a measurement of datagram loss: real
loss can be bursty and receiving middleware can discard complete messages.
This module fits p from joined transit records in three steps:

1. **Link events.** A second in which ``EVENT_TOPICS`` or more distinct topics
   lose a message is one event on the link rather than several topic defects.
   That second and ``EVENT_PAD_S`` seconds either side form the event layer;
   everything else is the steady state.
2. **Maximum likelihood.** The rate of each layer maximizes the likelihood of
   the observed losses given every message's fragment count. The likelihood
   depends only on how many messages of each fragment count were offered and
   lost, so the fit works on those counts and is exact.
3. **Block bootstrap.** Loss clusters in time, so a message is not an
   independent draw. The interval resamples whole blocks of ``block_s``
   seconds with replacement and refits each draw.

A message needs a receiver verdict (``delivered``, ``reordered`` or ``lost``),
its serialized size and its send time (``t_wrap``); the sender's rows supply
the last two for a lost message once the records are joined. Messages without
them are counted and left out.

    python -m rosotacom.fragment_loss records.jsonl [...] --fragment-bytes 1024
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import random
import sys
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rosotacom.transit import join_transit_records

#: A second in which this many distinct topics lose is a link event.
EVENT_TOPICS = 3

#: Event seconds are extended by this many seconds either side: the link does
#: not recover on a bin boundary.
EVENT_PAD_S = 1

#: Upper end of the rate search; a link losing half its fragments is not a
#: steady state.
RATE_MAX = 0.5

BOOTSTRAP_DRAWS = 2000
BLOCK_S = 60

#: Fewer losses than this, fewer messages or fewer blocks, and a fit or an
#: interval says more about the estimator than about the link.
MIN_LOSSES = 5
MIN_MESSAGES = 100
MIN_BLOCKS = 8

#: Upper edges of the fragment-count bins in the size table.
SIZE_BIN_EDGES = (1, 2, 3, 5, 8, 13, 21, 34, 55)

_ARRIVED = {"delivered": False, "reordered": False, "lost": True}


@dataclass(frozen=True)
class Message:
    topic: str
    t_s: float
    size_bytes: int
    nfrag: int
    lost: bool


#: Messages offered and lost per fragment count.
Counts = dict[int, tuple[int, int]]


def read_records(paths: Iterable[Path]) -> list[dict[str, Any]]:
    """Transit records from JSONL files, gzip-compressed or not."""
    records: list[dict[str, Any]] = []
    for path in paths:
        opener = gzip.open if path.suffix == ".gz" else open
        with opener(path, "rt", encoding="utf-8") as handle:
            for line_number, line in enumerate(handle, start=1):
                if not line.strip():
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{path}:{line_number}: invalid JSON: {exc}") from exc
                if event.get("kind") == "transit":
                    records.append(event)
    return records


def messages_from_records(
    records: Iterable[dict[str, Any]], fragment_bytes: int, source: str | None = None
) -> tuple[list[Message], int]:
    """(messages with a verdict, size and send time; number left out)."""
    if fragment_bytes <= 0:
        raise ValueError("fragment_bytes must be positive")
    out: list[Message] = []
    skipped = 0
    for record in join_transit_records(records):
        if source is not None and record.get("source") != source:
            continue
        status = record.get("status")
        size = record.get("size_bytes")
        t_s = record.get("t_wrap")
        if status not in _ARRIVED or size is None or t_s is None or int(size) <= 0:
            skipped += 1
            continue
        out.append(
            Message(
                topic=str(record.get("topic") or ""),
                t_s=float(t_s),
                size_bytes=int(size),
                nfrag=max(1, math.ceil(int(size) / fragment_bytes)),
                lost=_ARRIVED[str(status)],
            )
        )
    return out, skipped


def link_event_seconds(messages: Sequence[Message]) -> set[int]:
    """Seconds (floor of the send time) that belong to a link event."""
    topics_losing: dict[int, set[str]] = {}
    for m in messages:
        if m.lost:
            topics_losing.setdefault(math.floor(m.t_s), set()).add(m.topic)
    seconds: set[int] = set()
    for second, topics in topics_losing.items():
        if len(topics) >= EVENT_TOPICS:
            seconds.update(range(second - EVENT_PAD_S, second + EVENT_PAD_S + 1))
    return seconds


def split_layers(messages: Sequence[Message]) -> tuple[list[Message], list[Message], int]:
    """(steady-state messages, event messages, number of event seconds)."""
    event = link_event_seconds(messages)
    steady = [m for m in messages if math.floor(m.t_s) not in event]
    inside = [m for m in messages if math.floor(m.t_s) in event]
    return steady, inside, len(event)


def counts(messages: Iterable[Message]) -> Counts:
    table: dict[int, list[int]] = {}
    for m in messages:
        row = table.setdefault(m.nfrag, [0, 0])
        row[0] += 1
        row[1] += int(m.lost)
    return {n: (offered, lost) for n, (offered, lost) in table.items()}


def _negative_log_likelihood(p: float, table: Counts) -> float:
    total = 0.0
    for n, (offered, lost) in table.items():
        pm = min(max(1.0 - (1.0 - p) ** n, 1e-12), 1.0 - 1e-12)
        total -= lost * math.log(pm) + (offered - lost) * math.log1p(-pm)
    return total


def fit_rate(table: Counts) -> float:
    """Maximum-likelihood per-fragment loss probability (nan without losses).

    The log-likelihood is concave in p, so a golden-section search on log p
    finds its maximum; 200 steps narrow the bracket far below any digit that
    is reported.
    """
    if not any(lost for _, lost in table.values()):
        return math.nan
    lo, hi = math.log(1e-9), math.log(RATE_MAX)
    ratio = (math.sqrt(5.0) - 1.0) / 2.0
    a, b = hi - ratio * (hi - lo), lo + ratio * (hi - lo)
    fa, fb = _negative_log_likelihood(math.exp(a), table), _negative_log_likelihood(math.exp(b), table)
    for _ in range(200):
        if fa < fb:
            hi, b, fb = b, a, fa
            a = hi - ratio * (hi - lo)
            fa = _negative_log_likelihood(math.exp(a), table)
        else:
            lo, a, fa = a, b, fb
            b = lo + ratio * (hi - lo)
            fb = _negative_log_likelihood(math.exp(b), table)
    return math.exp((lo + hi) / 2.0)


def _percentile(values: Sequence[float], q: float) -> float:
    """Linear-interpolation percentile, q in [0, 100]."""
    ordered = sorted(values)
    rank = (len(ordered) - 1) * q / 100.0
    below = math.floor(rank)
    above = min(below + 1, len(ordered) - 1)
    return ordered[below] + (ordered[above] - ordered[below]) * (rank - below)


def bootstrap_interval(
    messages: Sequence[Message], block_s: float = BLOCK_S, draws: int = BOOTSTRAP_DRAWS, seed: int = 20260828
) -> tuple[float, float]:
    """95 % percentile interval for the rate, resampling blocks of ``block_s`` seconds."""
    if len(messages) < MIN_MESSAGES or not any(m.lost for m in messages):
        return (math.nan, math.nan)
    blocks: dict[int, list[Message]] = {}
    for m in messages:
        blocks.setdefault(math.floor(math.floor(m.t_s) // block_s), []).append(m)
    per_block = [counts(group) for group in blocks.values()]
    if len(per_block) < MIN_BLOCKS:
        return (math.nan, math.nan)
    rng = random.Random(seed)
    rates: list[float] = []
    for _ in range(draws):
        pooled: dict[int, list[int]] = {}
        for _block in range(len(per_block)):
            for n, (offered, lost) in per_block[rng.randrange(len(per_block))].items():
                row = pooled.setdefault(n, [0, 0])
                row[0] += offered
                row[1] += lost
        rate = fit_rate({n: (o, lo) for n, (o, lo) in pooled.items()})
        if not math.isnan(rate):
            rates.append(rate)
    if not rates:
        return (math.nan, math.nan)
    return (_percentile(rates, 2.5), _percentile(rates, 97.5))


def size_table(messages: Sequence[Message], p: float) -> list[dict[str, Any]]:
    """Observed loss per fragment-count bin beside the loss 1-(1-p)^n predicts."""
    rows = []
    lower = 0
    for edge in SIZE_BIN_EDGES:
        group = [m for m in messages if lower < m.nfrag <= edge]
        lower = edge
        if not group:
            continue
        lost = sum(m.lost for m in group)
        predicted = sum(1.0 - (1.0 - p) ** m.nfrag for m in group) / len(group) if not math.isnan(p) else math.nan
        rows.append(
            {
                "fragments": f"{edge}" if edge == 1 else f"<={edge}",
                "messages": len(group),
                "lost": lost,
                "loss_pct": 100.0 * lost / len(group),
                "predicted_pct": 100.0 * predicted,
            }
        )
    return rows


def _layer(messages: Sequence[Message], block_s: float, draws: int, seed: int) -> dict[str, Any]:
    lost = sum(m.lost for m in messages)
    p = lo = hi = math.nan
    if lost >= MIN_LOSSES:
        p = fit_rate(counts(messages))
        lo, hi = bootstrap_interval(messages, block_s, draws, seed)
    return {
        "messages": len(messages),
        "lost": lost,
        "rate_pct": 100 * p,
        "rate_lo_pct": 100 * lo,
        "rate_hi_pct": 100 * hi,
    }


def analyze(
    messages: Sequence[Message], block_s: float = BLOCK_S, draws: int = BOOTSTRAP_DRAWS, seed: int = 20260828
) -> dict[str, Any]:
    steady, event, event_seconds = split_layers(messages)
    result: dict[str, Any] = {
        "messages": len(messages),
        "lost": sum(m.lost for m in messages),
        "event_seconds": event_seconds,
        "steady": _layer(steady, block_s, draws, seed),
        "event": _layer(event, block_s, draws, seed),
    }
    result["steady"]["size_bins"] = size_table(steady, result["steady"]["rate_pct"] / 100)
    return result


def _format(result: dict[str, Any], fragment_bytes: int) -> str:
    lines = [
        f"{result['messages']} messages, {result['lost']} lost, {result['event_seconds']} s in link events"
        + (f", {result['skipped']} records without verdict, size or send time" if result.get("skipped") else "")
    ]
    for name in ("steady", "event"):
        layer = result[name]
        lines.append(
            f"{name:6}  {layer['messages']:>8} messages {layer['lost']:>6} lost  "
            f"per-fragment loss ({fragment_bytes} B) {layer['rate_pct']:.3f} % "
            f"[{layer['rate_lo_pct']:.3f}, {layer['rate_hi_pct']:.3f}]"
        )
    lines.append("steady state by fragment count: observed loss against 1-(1-p)^n")
    for row in result["steady"]["size_bins"]:
        lines.append(
            f"  {row['fragments']:>5} fragments  {row['messages']:>8}  "
            f"{row['loss_pct']:7.3f} %  {row['predicted_pct']:7.3f} %"
        )
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m rosotacom.fragment_loss",
        description="Fit the per-fragment loss rate of a link from transit records.",
    )
    parser.add_argument("records", nargs="+", type=Path, help="transit JSONL files (.gz allowed), both peers")
    parser.add_argument("--fragment-bytes", type=int, required=True, help="the DDS fragment size of the link")
    parser.add_argument("--source", help="only messages published by this peer")
    parser.add_argument("--block-s", type=float, default=BLOCK_S, help="bootstrap block length in seconds")
    parser.add_argument("--draws", type=int, default=BOOTSTRAP_DRAWS)
    parser.add_argument("--json", action="store_true", help="print the result as JSON")
    args = parser.parse_args(argv)
    messages, skipped = messages_from_records(read_records(args.records), args.fragment_bytes, args.source)
    if not messages:
        print("no transit record carries a verdict, a size and a send time", file=sys.stderr)
        return 1
    result = analyze(messages, args.block_s, args.draws)
    result["skipped"] = skipped
    print(json.dumps(result, indent=2) if args.json else _format(result, args.fragment_bytes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
