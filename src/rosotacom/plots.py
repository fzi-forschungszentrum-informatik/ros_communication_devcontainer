# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""benchmark commands reference — benchmark plot rendering (the ``[plots]`` optional extra).

All plot functions accept plain dicts/sequences (the output of the benchmark
drivers), not domain objects, so they remain testable without a live run.
``matplotlib`` is import-guarded: the core package never depends on it.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
from typing import Any


def _require_matplotlib() -> tuple[Any, Any]:
    """Return ``(matplotlib, matplotlib.pyplot)`` or raise a helpful error."""
    try:
        import matplotlib

        matplotlib.use("Agg")  # non-interactive backend
        import matplotlib.pyplot as plt

        return matplotlib, plt
    except ImportError:
        raise ImportError(
            "matplotlib is required for benchmark plots. Install with: pip install rosotacom[plots]"
        ) from None


# --------------------------------------------------------------------------- #
# Fixed probe — time-series characterization
# --------------------------------------------------------------------------- #


def plot_probe_timeseries(
    bins: Sequence[dict[str, Any]],
    *,
    out: str | Path,
    title: str = "Probe time series",
) -> Path:
    """Render 1-second probe bins: latency/loss plus delivered Hz/bandwidth."""
    _, plt = _require_matplotlib()
    out = Path(out)

    grouped: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for row in bins:
        topic = str(row.get("topic") or "")
        attempt = int(row.get("attempt", 1) or 1)
        grouped.setdefault((topic, attempt), []).append(row)

    fig, (ax_quality, ax_rate) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax_loss = ax_quality.twinx()
    ax_bw = ax_rate.twinx()
    cmap = plt.colormaps["tab10"].resampled(max(len(grouped), 1))

    for index, ((topic, attempt), rows) in enumerate(sorted(grouped.items())):
        rows = sorted(rows, key=lambda row: float(row["bin_start_s"]))
        xs = [float(row["bin_start_s"]) for row in rows]
        latency = [row.get("latency_p95_ms") for row in rows]
        loss = [float(row.get("loss_pct") or 0.0) for row in rows]
        hz = [float(row.get("delivered_hz") or 0.0) for row in rows]
        bandwidth_mbps = [float(row.get("payload_bandwidth_bps") or 0.0) / 1_000_000.0 for row in rows]
        label = topic or "topic"
        if len({key[1] for key in grouped}) > 1:
            label = f"{label} #{attempt}"
        color = cmap(index)
        ax_quality.plot(xs, latency, "o-", color=color, label=f"{label} latency p95")
        ax_loss.plot(xs, loss, "x--", color=color, alpha=0.65, label=f"{label} loss")
        ax_rate.plot(xs, hz, "o-", color=color, label=f"{label} delivered Hz")
        ax_bw.plot(xs, bandwidth_mbps, "s--", color=color, alpha=0.65, label=f"{label} payload Mbit/s")

    ax_quality.set_ylabel("Latency p95 (ms)")
    ax_loss.set_ylabel("Loss (%)")
    ax_rate.set_ylabel("Delivered Hz")
    ax_bw.set_ylabel("Payload (Mbit/s)")
    ax_rate.set_xlabel("Time since first publish (s)")
    ax_quality.set_title(title)

    handles: list[Any] = []
    labels: list[str] = []
    for axis in (ax_quality, ax_loss, ax_rate, ax_bw):
        axis_handles, axis_labels = axis.get_legend_handles_labels()
        handles.extend(axis_handles)
        labels.extend(axis_labels)
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=2, fontsize="x-small")
        fig.tight_layout(rect=(0, 0, 1, 0.88))
    else:
        fig.tight_layout()

    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def plot_video_quality(
    report: dict[str, Any],
    *,
    out: str | Path,
    title: str = "Video quality",
) -> Path:
    """Render per-frame PSNR and SSIM from a ``rosotacom videoquality`` report."""
    _, plt = _require_matplotlib()
    out = Path(out)

    compared = [row for row in report.get("frames", []) if row.get("status") == "compared"]
    lost = [row for row in report.get("frames", []) if row.get("status") == "lost"]
    xs = [int(row["ref_index"]) for row in compared]
    psnr = [None if row.get("psnr_db") == "inf" else row.get("psnr_db") for row in compared]
    quality = [row.get("ssim") for row in compared]

    fig, ax_psnr = plt.subplots(figsize=(10, 5))
    ax_ssim = ax_psnr.twinx()
    if xs:
        ax_psnr.plot(xs, psnr, "o-", color="tab:blue", label="PSNR dB")
        ax_ssim.plot(xs, quality, "s-", color="tab:green", label="SSIM")
    if lost:
        lost_xs = [int(row["ref_index"]) for row in lost]
        ax_psnr.scatter(lost_xs, [0.0] * len(lost_xs), marker="x", color="tab:red", label="lost frame")

    ax_psnr.set_xlabel("Reference frame index")
    ax_psnr.set_ylabel("PSNR (dB)", color="tab:blue")
    ax_ssim.set_ylabel("SSIM", color="tab:green")
    ax_psnr.set_title(title)
    handles: list[Any] = []
    labels: list[str] = []
    for axis in (ax_psnr, ax_ssim):
        axis_handles, axis_labels = axis.get_legend_handles_labels()
        handles.extend(axis_handles)
        labels.extend(axis_labels)
    if handles:
        fig.legend(handles, labels, loc="upper center", ncol=3, fontsize="small")
        fig.tight_layout(rect=(0, 0, 1, 0.90))
    else:
        fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# 1.1 / 2.1 — capacity frontier
# --------------------------------------------------------------------------- #


def plot_capacity_frontier(
    results: Sequence[dict[str, Any]],
    *,
    x_key: str = "bandwidth_bps",
    out: str | Path,
    title: str = "Capacity frontier",
) -> Path:
    """Latency & loss vs environment params.

    X-axis is the swept knob (``x_key``); two Y-axes for ``loss_pct``
    (left) and ``latency_p95_ms`` (right).
    """
    _, plt = _require_matplotlib()
    out = Path(out)

    # Determine the actual key to use on the x-axis
    actual_x_key = x_key
    if results and x_key not in results[0] and "profile" in results[0]:
        actual_x_key = "profile"

    xs = [r[actual_x_key] for r in results]
    loss = [r["loss_pct"] for r in results]
    lat = [r["latency_p95_ms"] for r in results]

    fig, ax_loss = plt.subplots()
    ax_loss.set_xlabel(x_key)
    ax_loss.set_ylabel("Loss (%)", color="tab:red")
    ax_loss.plot(xs, loss, "o-", color="tab:red", label="loss_pct")
    ax_loss.tick_params(axis="y", labelcolor="tab:red")

    ax_lat = ax_loss.twinx()
    ax_lat.set_ylabel("Latency p95 (ms)", color="tab:blue")
    ax_lat.plot(xs, lat, "s--", color="tab:blue", label="latency_p95_ms")
    ax_lat.tick_params(axis="y", labelcolor="tab:blue")

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# 1.2 — offered bandwidth
# --------------------------------------------------------------------------- #


def plot_offered_bw(
    results: Sequence[dict[str, Any]],
    *,
    out: str | Path,
    title: str = "Latency vs offered bandwidth",
) -> Path:
    """Latency vs offered bandwidth.

    Points colored by ``(size, rate_hz, streams)`` tuple.  Each result dict
    has: ``offered_bw_bps``, ``latency_p95_ms``, ``size``, ``rate_hz``,
    ``streams``.
    """
    _, plt = _require_matplotlib()
    out = Path(out)

    xs = [r["offered_bw_bps"] for r in results]
    ys = [r["latency_p95_ms"] for r in results]
    labels = [f"{r['size']}B/{r['rate_hz']}Hz/×{r['streams']}" for r in results]

    # Assign a unique colour per label.
    unique = sorted(set(labels))
    cmap = plt.colormaps["tab10"].resampled(max(len(unique), 1))
    colour_map = {lbl: cmap(i) for i, lbl in enumerate(unique)}

    fig, ax = plt.subplots()
    for x, y, lbl in zip(xs, ys, labels, strict=True):
        ax.scatter(x, y, color=colour_map[lbl], label=lbl, zorder=3)

    # De-duplicate legend entries.
    handles, leg_labels = ax.get_legend_handles_labels()
    seen: set[str] = set()
    deduped: list[tuple[Any, str]] = []
    for handle, label in zip(handles, leg_labels, strict=True):
        if label not in seen:
            seen.add(label)
            deduped.append((handle, label))
    if deduped:
        ax.legend(*zip(*deduped, strict=True), fontsize="small")

    ax.set_xlabel("Offered bandwidth (bps)")
    ax.set_ylabel("Latency p95 (ms)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# 1.3 — degradation ramp
# --------------------------------------------------------------------------- #


def plot_ramp(
    curve: Sequence[dict[str, float]],
    *,
    out: str | Path,
    title: str = "Degradation ramp",
    x_label: str = "Load",
    y_label: str = "Latency p95 (ms)",
) -> Path:
    """Latency-vs-load ramp curve with knee annotation.

    Each dict has ``value`` (the load knob) and ``metric`` (the observed
    latency).
    """
    _, plt = _require_matplotlib()
    out = Path(out)

    xs = [p["value"] for p in curve]
    ys = [p["metric"] for p in curve]

    fig, ax = plt.subplots()
    ax.plot(xs, ys, "o-", color="tab:green")

    # Simple knee detection: point with largest second-order difference.
    if len(ys) >= 3:
        diffs = [ys[i + 1] - ys[i] for i in range(len(ys) - 1)]
        second = [diffs[i + 1] - diffs[i] for i in range(len(diffs) - 1)]
        knee_idx = second.index(max(second)) + 1
        ax.annotate(
            "knee",
            xy=(xs[knee_idx], ys[knee_idx]),
            fontsize=9,
            ha="center",
            arrowprops={"arrowstyle": "->"},
            xytext=(xs[knee_idx], ys[knee_idx] * 1.15 + 1),
        )

    ax.set_xlabel(x_label)
    ax.set_ylabel(y_label)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# 1.4 / 2.3 — recovery timeline
# --------------------------------------------------------------------------- #


def plot_recovery_timeline(
    records: Sequence[dict[str, Any]],
    outage_start: float,
    outage_end: float,
    *,
    out: str | Path,
    title: str = "Recovery timeline",
) -> Path:
    """Message arrivals around the outage window.

    Plots arrival times, marks the outage window, and annotates
    ``t_recover`` and burst count.
    """
    _, plt = _require_matplotlib()
    out = Path(out)

    arrivals = sorted(float(r["arrival_s"]) for r in records if r.get("arrival_s") is not None)

    fig, ax = plt.subplots()
    ax.eventplot([arrivals], orientation="horizontal", lineoffsets=0.5, colors="tab:blue")
    ax.axvspan(outage_start, outage_end, alpha=0.25, color="red", label="outage")

    # First arrival after outage end → recovery time.
    post = [a for a in arrivals if a > outage_end]
    if post:
        t_recover = post[0] - outage_end
        ax.axvline(post[0], linestyle="--", color="tab:green", label=f"recover Δ{t_recover:.1f}s")

    # Burst: arrivals within 1 s of outage_end.
    burst = sum(1 for a in arrivals if outage_end <= a <= outage_end + 1.0)
    if burst:
        ax.set_xlabel(f"Time (s) — burst={burst}")
    else:
        ax.set_xlabel("Time (s)")

    ax.set_title(title)
    ax.legend(fontsize="small")
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# 2.2 — per-topic heatmap
# --------------------------------------------------------------------------- #


def plot_topic_heatmap(
    per_topic: dict[str, dict[str, float]],
    *,
    metric: str = "loss_pct",
    out: str | Path,
    title: str = "Per-topic loss heatmap",
) -> Path:
    """Per-topic × condition heatmap.

    ``per_topic`` maps ``topic_name → {condition_name → metric_value}``.
    """
    _, plt = _require_matplotlib()
    out = Path(out)

    topics = sorted(per_topic)
    if not topics:
        # Empty data — write a blank figure.
        fig, ax = plt.subplots()
        ax.set_title(title)
        fig.savefig(out, dpi=150, bbox_inches="tight")
        plt.close(fig)
        return out

    conditions = sorted({c for vals in per_topic.values() for c in vals})
    matrix = [[per_topic[t].get(c, 0.0) for c in conditions] for t in topics]

    fig, ax = plt.subplots(figsize=(max(4, len(conditions) * 0.8), max(3, len(topics) * 0.5)))
    im = ax.imshow(matrix, aspect="auto", cmap="YlOrRd")
    fig.colorbar(im, ax=ax, label=metric)

    ax.set_xticks(range(len(conditions)))
    ax.set_xticklabels(conditions, rotation=45, ha="right", fontsize=8)
    ax.set_yticks(range(len(topics)))
    ax.set_yticklabels(topics, fontsize=8)
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# Raw fixed probe — individual packet latencies and packet losses
# --------------------------------------------------------------------------- #


def plot_probe_raw(
    records: Sequence[dict[str, Any]],
    *,
    out: str | Path,
    title: str = "Probe raw latency and loss",
) -> Path:
    """Render raw probe transit records: individual packet latencies and losses."""
    _, plt = _require_matplotlib()
    out = Path(out)

    from .benchmark import _infer_nominal_period_s, _section_ms, _send_time, find_probe_onset

    grouped: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for row in records:
        source = str(row.get("source") or "")
        target = str(row.get("target") or "")
        topic = str(row.get("topic") or "")
        topic_label = f"{source}->{target}:{topic}" if source or target else topic
        attempt = int(row.get("attempt", 1) or 1)
        grouped.setdefault((topic_label, attempt), []).append(row)

    fig, ax = plt.subplots(figsize=(10, 6))
    cmap = plt.colormaps["tab10"].resampled(max(len(grouped), 1))

    setup_lost_xs: list[float] = []
    real_lost_xs: list[float] = []
    reanchored = False

    for index, ((label, attempt), stream) in enumerate(sorted(grouped.items())):
        stream = sorted(stream, key=lambda record: int(record.get("seq", 0)))

        time_field = "t_wrap"
        period_s = _infer_nominal_period_s(stream, time_field=time_field)
        period_s = float(period_s or 0.0)

        stamped = [record for record in stream if record.get(time_field) is not None]
        if not stamped:
            continue
        anchor = min(stamped, key=lambda record: int(record.get("seq", 0)))
        seq0 = int(anchor["seq"])
        t0 = float(anchor.get(time_field) or 0.0)

        # One seq-ordered sample per packet: (x, latency) or (x, None) for a loss.
        first_send_s = min(_send_time(r, seq0=seq0, t0=t0, period_s=period_s, time_field=time_field) for r in stream)
        ordered: list[tuple[float, float | None]] = []
        for record in stream:
            send_t = _send_time(record, seq0=seq0, t0=t0, period_s=period_s, time_field=time_field)
            x = max(0.0, send_t - first_send_s)
            latency = (
                None if record.get("status") == "lost" else _section_ms(record, "ota_hop_ms", "ota_hop_uncorrected_ms")
            )
            ordered.append((x, latency))
        if not ordered:
            continue

        # Onset splits start-up (excluded) from the impaired regime (included).
        onset_index = find_probe_onset([latency for _, latency in ordered])
        onset_x = ordered[onset_index][0] if onset_index is not None else 0.0
        reanchored = reanchored or onset_index is not None

        inc_xs: list[float] = []
        inc_ys: list[float] = []
        exc_xs: list[float] = []
        exc_ys: list[float] = []
        for i, (x, latency) in enumerate(ordered):
            included = onset_index is None or i >= onset_index
            shifted = x - onset_x
            if latency is None:
                (real_lost_xs if included else setup_lost_xs).append(shifted)
            elif included:
                inc_xs.append(shifted)
                inc_ys.append(latency)
            else:
                exc_xs.append(shifted)
                exc_ys.append(latency)

        color = cmap(index)
        stream_label = label
        if len({key[1] for key in grouped}) > 1:
            stream_label = f"{stream_label} #{attempt}"

        if inc_xs:
            ax.scatter(inc_xs, inc_ys, color=color, s=12, alpha=0.6, label=f"{stream_label} latency")
        if exc_xs:
            ax.scatter(exc_xs, exc_ys, facecolors="none", edgecolors="0.6", s=14, alpha=0.7, label="excluded: start-up")
        if onset_index is not None:
            ax.axvline(0.0, color="0.5", linestyle="--", linewidth=1.0, alpha=0.8, label="impairment onset (t=0)")

    ax.set_ylim(bottom=0.0)
    ymin, ymax = ax.get_ylim()

    # A loss inside the included window is real; one at t<0 was dropped during
    # start-up (e.g. as the qdisc changed) and is excluded like the warm-up, so
    # the summary and the plot agree on the same clean window.
    if real_lost_xs:
        ax.vlines(real_lost_xs, ymin, ymax, colors="crimson", alpha=0.3, linewidth=1.0, label="lost packet")
    if setup_lost_xs:
        ax.vlines(setup_lost_xs, ymin, ymax, colors="0.6", alpha=0.4, linewidth=1.0, label="excluded: start-up loss")

    ax.set_ylabel("Latency (ms)")
    ax.set_xlabel(
        "Time relative to impairment onset (s); setup at t<0" if reanchored else "Time since first publish (s)"
    )
    ax.set_title(title)

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles, strict=False))
    if by_label:
        ax.legend(by_label.values(), by_label.keys(), loc="upper right", fontsize="small")

    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


# --------------------------------------------------------------------------- #
# Degradation forensics — per-stream timeline with events marked
# --------------------------------------------------------------------------- #

_FORENSICS_EVENT_STYLE = {
    "loss_burst": ("crimson", "loss burst"),
    "latency_excursion": ("darkorange", "latency excursion"),
    "rate_collapse": ("purple", "rate collapse"),
}


def plot_forensics_stream(
    bins: Sequence[dict[str, Any]],
    events: Sequence[dict[str, Any]],
    *,
    out: str | Path,
    title: str = "Forensics timeline",
    nominal_hz: float | None = None,
    timeline_steps: Sequence[dict[str, Any]] | None = None,
) -> Path:
    """One stream's forensics timeline: latency, delivery rate/loss, and sizes.

    ``bins`` are the report's per-stream time bins (relative ``bin_start_s``);
    ``events`` are the report's event dicts for this stream (relative
    ``start_s``/``end_s`` plus ``kind``); ``timeline_steps`` optionally marks
    network profiles reference profile segments (``start_s``/``end_s``/``label`` on the same
    relative axis). Event windows are shaded across all axes — this is "the
    plot that shows the degradation moment".
    """
    _, plt = _require_matplotlib()
    out = Path(out)

    rows = sorted(bins, key=lambda row: float(row["bin_start_s"]))
    xs = [float(row["bin_start_s"]) for row in rows]
    fig, (ax_latency, ax_rate, ax_size) = plt.subplots(3, 1, sharex=True, figsize=(11, 7.5))

    ax_latency.plot(xs, [row.get("latency_p50_ms") for row in rows], "-", color="tab:blue", label="latency p50")
    ax_latency.plot(xs, [row.get("latency_p95_ms") for row in rows], "--", color="tab:cyan", label="latency p95")
    ax_latency.plot(
        xs, [row.get("latency_max_ms") for row in rows], ":", color="tab:gray", alpha=0.7, label="latency max"
    )
    ax_latency.set_ylabel("OTA hop (ms)")
    ax_latency.set_title(title)

    ax_rate.plot(
        xs, [float(row.get("delivered_hz") or 0.0) for row in rows], "-", color="tab:green", label="delivered Hz"
    )
    if nominal_hz:
        ax_rate.axhline(nominal_hz, linestyle="--", color="0.5", linewidth=1.0, label="nominal Hz")
    ax_lost = ax_rate.twinx()
    width = (xs[1] - xs[0]) if len(xs) > 1 else 1.0
    ax_lost.bar(
        xs,
        [int(row.get("lost") or 0) for row in rows],
        width=width * 0.9,
        align="edge",
        color="crimson",
        alpha=0.35,
        label="lost / bin",
    )
    ax_lost.set_ylabel("Lost / bin")
    ax_rate.set_ylabel("Delivered Hz")

    ax_size.plot(xs, [row.get("mean_size_bytes") for row in rows], "-", color="tab:brown", label="mean size")
    ax_size.plot(xs, [row.get("max_size_bytes") for row in rows], ":", color="tab:brown", alpha=0.6, label="max size")
    keyframe_xs = [x for x, row in zip(xs, rows, strict=True) if int(row.get("keyframes") or 0) > 0]
    keyframe_ys = [row.get("max_size_bytes") for row in rows if int(row.get("keyframes") or 0) > 0]
    if keyframe_xs:
        ax_size.scatter(keyframe_xs, keyframe_ys, marker="^", color="tab:red", s=24, zorder=3, label="keyframe bin")
    ax_size.set_ylabel("Size (bytes)")
    ax_size.set_xlabel("Time since first publish (s)")

    for event in events:
        color, label = _FORENSICS_EVENT_STYLE.get(str(event.get("kind")), ("0.4", str(event.get("kind"))))
        for axis in (ax_latency, ax_rate, ax_size):
            axis.axvspan(float(event["start_s"]), float(event["end_s"]), color=color, alpha=0.18, label=label)

    for step in timeline_steps or ():
        ax_latency.axvline(float(step["start_s"]), color="0.6", linestyle=":", linewidth=1.0)
        ax_latency.annotate(
            str(step.get("label") or ""),
            xy=(float(step["start_s"]), 1.01),
            xycoords=("data", "axes fraction"),
            fontsize=7,
            color="0.4",
            ha="left",
        )

    handles: list[Any] = []
    labels: list[str] = []
    for axis in (ax_latency, ax_rate, ax_lost, ax_size):
        axis_handles, axis_labels = axis.get_legend_handles_labels()
        handles.extend(axis_handles)
        labels.extend(axis_labels)
    by_label = dict(zip(labels, handles, strict=False))
    if by_label:
        fig.legend(by_label.values(), by_label.keys(), loc="upper center", ncol=4, fontsize="x-small")
        fig.tight_layout(rect=(0, 0, 1, 0.93))
    else:
        fig.tight_layout()

    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out
