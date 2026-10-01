# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Append-only link trace samples for rosotacom session artifacts."""

from __future__ import annotations

import datetime
import json
import os
import subprocess
import time
from typing import Any, Callable, Dict, Optional


def passive_counter_delta_block(link_sample: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Schema block for passive /proc/net/dev counter deltas.

    The rates are observed traffic over the sampling window. They are not a
    capacity estimate and cannot reveal unused bandwidth.
    """
    if not link_sample:
        return {
            "available": False,
            "provenance": "proc_net_dev_counter_delta",
            "reason": "no counter delta sample",
            "observed_not_available_bandwidth": True,
        }
    return {
        "available": True,
        "provenance": "proc_net_dev_counter_delta",
        "interface": link_sample.get("interface"),
        "window_s": round(float(link_sample.get("window_s") or 0.0), 6),
        "observed_not_available_bandwidth": True,
        "rx": {
            "bytes_delta": int(link_sample.get("rx_bytes_delta") or 0),
            "packets_delta": link_sample.get("rx_packets_delta"),
            "observed_kbps": round(float(link_sample.get("rx_kbps") or 0.0), 3),
        },
        "tx": {
            "bytes_delta": int(link_sample.get("tx_bytes_delta") or 0),
            "packets_delta": link_sample.get("tx_packets_delta"),
            "observed_kbps": round(float(link_sample.get("tx_kbps") or 0.0), 3),
        },
    }


def echo_probe_block(snapshot: Dict[str, Any]) -> Dict[str, Any]:
    """Extract RTT/offset/loss as observed by the status snapshot."""
    clock_sync = snapshot.get("clock_sync") if isinstance(snapshot.get("clock_sync"), dict) else None
    rtt_ms = clock_sync.get("rtt_ms") if clock_sync else None
    peer_offset_ms = clock_sync.get("peer_offset_ms") if clock_sync else None
    loss_pct = None
    for topic in snapshot.get("topics") or []:
        if not isinstance(topic, dict):
            continue
        for stage in topic.get("stages") or []:
            if not isinstance(stage, dict):
                continue
            if rtt_ms is None and stage.get("rtt_ms") is not None:
                rtt_ms = stage.get("rtt_ms")
            if loss_pct is None and stage.get("loss_pct") is not None:
                loss_pct = stage.get("loss_pct")
            if rtt_ms is not None and loss_pct is not None:
                break
        if rtt_ms is not None and loss_pct is not None:
            break
    return {
        "available": rtt_ms is not None or peer_offset_ms is not None or loss_pct is not None,
        "provenance": "echo_heartbeat_status_snapshot",
        "rtt_ms": rtt_ms,
        "peer_offset_ms": peer_offset_ms,
        "loss_pct": loss_pct,
        "assumption": clock_sync.get("assumption") if clock_sync else None,
    }


def run_modem_metrics_hook(
    command: str,
    *,
    timeout_s: float = 2.0,
    runner: Callable[..., subprocess.CompletedProcess[str]] = subprocess.run,
) -> Dict[str, Any]:
    """Run a configured modem metrics command and parse its JSON object output."""
    command = (command or "").strip()
    if not command:
        return {"available": False, "provenance": "modem_metrics_command", "reason": "not configured"}
    try:
        result = runner(command, shell=True, text=True, capture_output=True, timeout=timeout_s, check=False)
    except Exception as exc:  # noqa: BLE001 - surfaced as trace provenance, not fatal.
        return {
            "available": False,
            "provenance": "modem_metrics_command",
            "command": command,
            "reason": f"command failed to run: {exc}",
        }
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        return {
            "available": False,
            "provenance": "modem_metrics_command",
            "command": command,
            "reason": f"exit {result.returncode}" + (f": {detail}" if detail else ""),
        }
    try:
        metrics = json.loads((result.stdout or "").strip() or "{}")
    except json.JSONDecodeError as exc:
        return {
            "available": False,
            "provenance": "modem_metrics_command",
            "command": command,
            "reason": f"invalid JSON: {exc}",
        }
    if not isinstance(metrics, dict):
        return {
            "available": False,
            "provenance": "modem_metrics_command",
            "command": command,
            "reason": "JSON output is not an object",
        }
    return {
        "available": True,
        "provenance": "modem_metrics_command",
        "command": command,
        "metrics": metrics,
    }


def build_link_trace_sample(
    snapshot: Dict[str, Any],
    link_sample: Optional[Dict[str, Any]],
    *,
    modem_metrics: Optional[Dict[str, Any]] = None,
    monotonic_s: Optional[float] = None,
) -> Dict[str, Any]:
    """Build one JSON-serializable link trace row."""
    return {
        "schema_version": 1,
        "kind": "link_trace",
        "generated_at": snapshot.get("generated_at")
        or datetime.datetime.now().isoformat(timespec="milliseconds"),
        "monotonic_s": round(float(monotonic_s if monotonic_s is not None else time.monotonic()), 6),
        "peer": snapshot.get("peer"),
        "remote": snapshot.get("remote"),
        "passive_counter_delta": passive_counter_delta_block(link_sample),
        "peer_probe": echo_probe_block(snapshot),
        "modem": modem_metrics or {
            "available": False,
            "provenance": "modem_metrics_command",
            "reason": "not configured",
        },
    }


class LinkTraceRecorder:
    """Write link trace rows as append-only JSONL."""

    #: How early a tick may be and still count as on time. The caller ticks at
    #: the same nominal period, so its jitter is the whole question; a tenth of
    #: the interval absorbs that without ever letting rows run at half the
    #: configured period.
    _TOLERANCE_FRACTION = 0.1

    def __init__(
        self,
        path: str,
        *,
        interval_s: float = 1.0,
        modem_metrics_command: str = "",
        modem_metrics_timeout_s: float = 2.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.path = path
        self.interval_s = max(0.1, float(interval_s))
        self.modem_metrics_command = modem_metrics_command
        self.modem_metrics_timeout_s = modem_metrics_timeout_s
        self._clock = clock
        self._due_at: Optional[float] = None

    def maybe_write(self, snapshot: Dict[str, Any], link_sample: Optional[Dict[str, Any]]) -> bool:
        """Write one row if this tick is the one the schedule was waiting for.

        The schedule is a *deadline that advances*, not a stopwatch since the
        last row, and the tolerance is not cosmetic. The caller is deliberately
        ticking at the same period as this one — `generate_session_files` sets
        `status_write_interval_s` to the trace interval — so a plain
        "never faster than interval_s" gate rejects every tick that lands a hair
        early and, with no catch-up, makes it wait a whole further period.

        That halves the trace, and it does not merely halve its resolution: the
        passive counter delta in a row covers the caller's own tick, so with
        rows twice as far apart every second of traffic is described by no row
        at all. Measured on the 2026-08-19 drive before this fix: rows 1.984 s
        apart carrying 1.002 s windows.
        """
        now = self._clock()
        if self._due_at is None:
            self._due_at = now
        if now < self._due_at - self._TOLERANCE_FRACTION * self.interval_s:
            return False
        # Advance the deadline rather than restarting it from now, so the rows
        # stay on one cadence instead of drifting by the jitter of every tick.
        # After a stall long enough that the next deadline is already behind us,
        # start a fresh interval from the present rather than bursting through
        # the deadlines that were missed: the rows describe the link now, and a
        # burst of them would describe nothing.
        next_due = self._due_at + self.interval_s
        self._due_at = next_due if next_due > now else now + self.interval_s
        modem = run_modem_metrics_hook(
            self.modem_metrics_command,
            timeout_s=self.modem_metrics_timeout_s,
        )
        row = build_link_trace_sample(snapshot, link_sample, modem_metrics=modem, monotonic_s=now)
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(self.path, "a", encoding="utf-8") as fp:
            fp.write(json.dumps(row, sort_keys=True) + "\n")
        return True
