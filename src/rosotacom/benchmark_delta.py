# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Compare two benchmark result sets taken in different environments.

The local emulated lanes encode most of the performance truth cheaply and
deterministically. What they cannot show is **which effects exist only on the
real link** — cellular variability, VPN/MTU quirks, radio behaviour, real
cross-traffic — and which are emulation artifacts. Real links stay monitor-only
by the determinism rule, so the missing piece was never another measurement: it
was the *comparison* between two sets of the same rows.

This is that comparison. It reads two directories (or lists) of `result.json`
runs, pairs them by what makes a row a row, and reports per-row, per-metric
deltas as JSON plus a short Markdown summary.

Three properties are deliberate, because it is easy to produce numbers that lie:

* **Pairing is explicit and refuses to guess.** Rows are matched on genre,
  profile, RMW and session name. A row present on one side only is reported as
  unmatched rather than silently dropped, because "it got faster" and "it did
  not run" must not look the same.
* **Every report says it is correlational.** The two sets differ in the
  environment *and* in everything the environment drags along; a delta is an
  observation, never a verdict. `monitor_only: true` is in the JSON and the
  first line of the Markdown.
* **It is self-describing.** Each side carries its rosotacom SHA, profile file,
  and the command that produced it, so a report pasted into a paper or an issue
  can be traced back without the directory it came from.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1

#: Numeric leaves under `result` that are worth comparing across environments.
#: A deny-list rather than an allow-list would drag in ids and counts whose
#: "delta" means nothing.
INTERESTING_SUFFIXES = (
    "capacity",
    "latency_ms",
    "latency_p95_ms",
    "loss_pct",
    "rate_hz",
    "mean_payload_bytes",
    "offered_bandwidth_bps",
    "jitter_ms",
    "throughput_bps",
    "delivered",
    "received",
    "published",
)


@dataclass(frozen=True)
class RowKey:
    """What makes two runs the same row in different environments."""

    genre: str
    profile: str
    rmw: str
    session: str

    def label(self) -> str:
        return f"{self.genre}/{self.profile}/{self.rmw}/{self.session}"


@dataclass
class Run:
    key: RowKey
    metrics: dict[str, float]
    provenance: dict[str, Any] = field(default_factory=dict)
    path: str = ""


def _flatten_numbers(value: Any, prefix: str = "") -> dict[str, float]:
    out: dict[str, float] = {}
    if isinstance(value, dict):
        for key, child in value.items():
            out.update(_flatten_numbers(child, f"{prefix}.{key}" if prefix else str(key)))
    elif isinstance(value, bool):
        # Explicitly before the numeric branch: bool is an int in Python, and a
        # "delta" between True and False is a category error dressed as -1.
        return {}
    elif isinstance(value, (int, float)):
        return {prefix: float(value)}
    return out


def interesting_metrics(result: Any) -> dict[str, float]:
    numbers = _flatten_numbers(result)
    return {
        name: value
        for name, value in numbers.items()
        if any(name == suffix or name.endswith("." + suffix) for suffix in INTERESTING_SUFFIXES)
    }


def load_run(path: Path) -> Run | None:
    """Read one `result.json`. Returns None when it is not one."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict) or "genre" not in data:
        return None

    context = data.get("context") or {}
    session = context.get("session") or {}
    profile = context.get("profile") or {}
    rmw = context.get("rmw") or {}

    key = RowKey(
        genre=str(data.get("genre") or ""),
        profile=str(profile.get("name") or (data.get("configuration") or {}).get("profile") or ""),
        rmw=str(rmw.get("requested") or session.get("rmw") or ""),
        session=str(session.get("name") or ""),
    )
    return Run(
        key=key,
        metrics=interesting_metrics(data.get("result")),
        provenance={
            "created_at": data.get("created_at"),
            "command": context.get("command"),
            "profiles_file": (context.get("paths") or {}).get("profiles_file"),
            "rosotacom_sha": (context.get("versions") or {}).get("rosotacom_sha"),
            "verdict": (data.get("verdict") or {}).get("status"),
        },
        path=str(path),
    )


def collect(paths: list[Path]) -> dict[RowKey, Run]:
    """Every `result.json` under the given files or directories, by row."""
    found: dict[RowKey, Run] = {}
    for path in paths:
        candidates = [path] if path.is_file() else sorted(path.rglob("result.json"))
        for candidate in candidates:
            run = load_run(candidate)
            if run is None:
                continue
            # Last one wins, and it is the newest by directory ordering — a
            # re-run of a row supersedes it rather than doubling it.
            found[run.key] = run
    return found


def compare(
    reference: dict[RowKey, Run],
    measured: dict[RowKey, Run],
    *,
    label_reference: str,
    label_measured: str,
) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for key in sorted(set(reference) & set(measured), key=lambda k: k.label()):
        a, b = reference[key], measured[key]
        metrics = []
        for name in sorted(set(a.metrics) | set(b.metrics)):
            left = a.metrics.get(name)
            right = b.metrics.get(name)
            if left is None or right is None:
                metrics.append(
                    {
                        "metric": name,
                        label_reference: left,
                        label_measured: right,
                        "delta": None,
                        "relative": None,
                        "note": "present on one side only",
                    }
                )
                continue
            delta = right - left
            metrics.append(
                {
                    "metric": name,
                    label_reference: left,
                    label_measured: right,
                    "delta": delta,
                    "relative": (delta / left) if left else None,
                }
            )
        rows.append(
            {
                "row": key.label(),
                "genre": key.genre,
                "profile": key.profile,
                "rmw": key.rmw,
                "session": key.session,
                "metrics": metrics,
                "provenance": {label_reference: a.provenance, label_measured: b.provenance},
            }
        )

    return {
        "schema_version": SCHEMA_VERSION,
        # First key, and repeated in the Markdown: a delta between two
        # environments is correlational. The two sets differ in the environment
        # and in everything it drags along.
        "monitor_only": True,
        "labels": {"reference": label_reference, "measured": label_measured},
        "rows": rows,
        "unmatched": {
            label_reference: sorted(k.label() for k in set(reference) - set(measured)),
            label_measured: sorted(k.label() for k in set(measured) - set(reference)),
        },
        "counts": {
            "matched": len(rows),
            f"only_{label_reference}": len(set(reference) - set(measured)),
            f"only_{label_measured}": len(set(measured) - set(reference)),
        },
    }


def render_markdown(report: dict[str, Any]) -> str:
    ref = report["labels"]["reference"]
    mea = report["labels"]["measured"]
    lines = [
        f"# Benchmark delta: {ref} vs {mea}",
        "",
        "**Monitor-only and correlational.** The two sets differ in the environment "
        "and in everything the environment drags along; a delta here is an "
        "observation to explain, never a verdict and never a gate.",
        "",
        f"{report['counts']['matched']} row(s) matched, "
        f"{report['counts'][f'only_{ref}']} only in {ref}, "
        f"{report['counts'][f'only_{mea}']} only in {mea}.",
        "",
    ]

    for row in report["rows"]:
        lines.append(f"## `{row['row']}`")
        lines.append("")
        lines.append(f"| metric | {ref} | {mea} | Δ | Δ% |")
        lines.append("|---|---:|---:|---:|---:|")
        for metric in row["metrics"]:
            left = metric[ref]
            right = metric[mea]
            delta = metric["delta"]
            relative = metric["relative"]
            lines.append(
                f"| `{metric['metric']}` | {_fmt(left)} | {_fmt(right)} | {_fmt(delta, sign=True)} | {_pct(relative)} |"
            )
        lines.append("")
        for side in (ref, mea):
            provenance = row["provenance"][side]
            command = provenance.get("command") or "-"
            lines.append(f"- {side}: `{command}`")
        lines.append("")

    for side in (ref, mea):
        missing = report["unmatched"][side]
        if missing:
            lines.append(f"## Only in {side}")
            lines.append("")
            lines.extend(f"- `{name}`" for name in missing)
            lines.append("")
            lines.append(
                "A row on one side only is not a delta. It ran in one environment "
                "and not the other, and treating that as an improvement is the "
                "commonest way this kind of report lies."
            )
            lines.append("")

    return "\n".join(lines)


def _fmt(value: float | None, *, sign: bool = False) -> str:
    if value is None:
        return "—"
    formatted = f"{value:+.4g}" if sign else f"{value:.4g}"
    return formatted


def _pct(value: float | None) -> str:
    return "—" if value is None else f"{value * 100:+.1f}%"
