#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Replay a drive's camera frame plan through its fitted netem profile.

This is the check behind the field profiles: the drive's own camera stream,
sent through netem shaped by the profile fitted from the same drive, loses
about as many frames as the drive did. It runs for `r3`, the one window whose
frame plan is published here (`r3_frame_plan.csv`, see README.md).

Sender and receiver are two disposable containers on their own Docker network
(`netem-replay-*`); only the sender gets NET_ADMIN. The sender plays the frame
plan as UDP datagrams with up to 1188 B of synthetic payload plus a replay
header, using the fragment count implied by the 1200 B DDS-fragment model,
with a frame's fragments back to back at its recorded time. The receiver counts
which frames arrived complete. The sender's egress is shaped by

  none     no shaping (sanity: no loss expected)
  steady   `<key>-steady`
  events   `<key>-events`: the timeline is looped inside the sending container,
           one `tc qdisc change` per segment, and every switch is logged, so the
           realized share of time in the event segment is measured

    python3 replay.py --scenario none steady events --repeat 9 --work /tmp/replay

Each run writes `result_<scenario>_<n>.json` into the work directory, and
`summary.json` collects mean, spread and range per scenario beside the loss
the profile implies for this plan, 1-(1-p)^n per frame of n fragments. Needs
Docker and PyYAML on the host; the containers need nothing but Python.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import statistics
import struct
import subprocess
import sys
import time
import uuid
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

FRAG = 1188
HDR = struct.Struct("<IHHdB")  # frame_id, frag_idx, n_frags, t_sched, kf
PORT = 47555
IMAGE = "netem-replay:latest"
# Containers run under the Docker daemon, outside any memory limit of the
# shell that starts them, so each gets its own (a sender or receiver needs ~12 MB).
MEM = ["--memory=256m", "--memory-swap=256m"]
DOCKERFILE = """FROM ubuntu:24.04
RUN apt-get update && apt-get install -y --no-install-recommends python3 iproute2 \\
    && rm -rf /var/lib/apt/lists/*
"""
HERE = Path(__file__).resolve()
PROFILES = HERE.parent.parent / "netem_profiles.yaml"
SCENARIOS = ("none", "steady", "events")


def run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    kw.setdefault("timeout", 60)
    return subprocess.run(cmd, check=True, capture_output=True, text=True, **kw)


def read_plan(path: Path, duration: float) -> list[tuple[float, int, bool]]:
    """(time from the first frame [s], size [B], key frame) for the first `duration` seconds."""
    with open(path) as f:
        rows = [(float(r["t_s"]), int(r["size"]), r["kf"] in ("True", "1")) for r in csv.DictReader(f)]
    if not rows or any(not math.isfinite(t) or size <= 0 for t, size, _ in rows):
        raise ValueError("frame plan must contain finite timestamps and positive sizes")
    if any(a[0] > b[0] for a, b in zip(rows, rows[1:], strict=False)):
        raise ValueError("frame plan timestamps must be ordered")
    t_base = rows[0][0]
    return [(t - t_base, size, kf) for t, size, kf in rows if t - t_base <= duration]


def n_frags(size: int) -> int:
    return max(1, math.ceil(size / FRAG))


# ------------------------------------------------------------ in containers
def mode_send(args) -> None:
    plan = read_plan(Path(args.plan), args.duration)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
    t0 = time.monotonic() + 0.5
    for i, (t_s, size, kf) in enumerate(plan):
        while (dt := (t0 + t_s) - time.monotonic()) > 0:
            time.sleep(min(dt, 0.05))
        n = n_frags(size)
        for fi in range(n):
            chunk = min(FRAG, size - fi * FRAG)
            s.sendto(HDR.pack(i, fi, n, t_s, int(kf)) + b"\x00" * max(chunk, 1), (args.dest, PORT))
    time.sleep(2.0)
    # END crosses the same netem as the frames, so one datagram can be lost.
    for _ in range(20):
        s.sendto(b"END", (args.dest, PORT))
        time.sleep(0.005)
    print(f"sent {len(plan)} frames")


def mode_recv(args) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    s.bind(("0.0.0.0", PORT))
    s.settimeout(args.duration + 60)
    got: dict[int, set[int]] = {}
    n_of: dict[int, int] = {}
    try:
        while True:
            data, _ = s.recvfrom(2048)
            if data[:3] == b"END":
                break
            s.settimeout(10)  # the frames arrive at 10 Hz: 10 s of silence is the end
            fid, fi, n, _t, _kf = HDR.unpack(data[: HDR.size])
            got.setdefault(fid, set()).add(fi)
            n_of[fid] = n
    except TimeoutError:
        pass
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_id", "n_frags", "frags_got", "complete"])
        for fid in sorted(got):
            w.writerow([fid, n_of[fid], len(got[fid]), int(len(got[fid]) == n_of[fid])])
    print(f"received {len(got)} frames (any fragment)")


# ------------------------------------------------------------------ on host
def netem_args(uplink: dict) -> str:
    parts = []
    if "delay" in uplink:
        parts += ["delay", str(uplink["delay"])]
        if "jitter" in uplink:
            parts.append(str(uplink["jitter"]))
            if "distribution" in uplink:
                parts += ["distribution", str(uplink["distribution"])]
    if "loss" in uplink:
        parts += ["loss", str(uplink["loss"])]
        if "loss_correlation" in uplink:
            parts.append(str(uplink["loss_correlation"]))
    return " ".join(parts)


def seconds(value) -> float:
    return float(str(value).rstrip("s"))


def segments(scenario: str, profiles: dict, key: str) -> list[tuple[float, dict]] | None:
    """The shaping of a scenario as [(seconds, uplink), ...]; one entry without a cycle."""
    if scenario == "none":
        return None
    if scenario == "steady":
        return [(math.inf, profiles[f"{key}-steady"]["uplink"])]
    return [(seconds(s["for"]), s["uplink"]) for s in profiles[f"{key}-events"]["timeline"]]


def sender_shell(segs, plan: str, dest: str, duration: int, switch_log: str) -> str:
    """The sender's command line: shape, start the timeline loop, send."""
    me = f"python3 /w/{HERE.name} send --dest {dest} --plan {plan} --duration {duration}"
    if segs is None:
        return me
    shape = f"tc qdisc replace dev eth0 root handle 1: netem {netem_args(segs[0][1])}"
    if len(segs) == 1:
        return f"{shape} && {me}"
    # One process owns the qdisc: each segment is one `tc qdisc change`, then a
    # sleep of the segment's length, and every switch is logged with its wall
    # time so the realized durations can be read back.
    steps = "; ".join(
        f'tc qdisc change dev eth0 root handle 1: netem {netem_args(u)} || {{ kill -TERM "$owner"; exit 1; }}; '
        f"echo {i} $(date +%s.%N) >> {switch_log}; sleep {d}"
        for i, (d, u) in enumerate(segs)
    )
    return (
        f"{shape} && {{ owner=$$; (while true; do {steps}; done) & timeline=$!; "
        f"trap 'kill \"$timeline\" 2>/dev/null || true' EXIT; {me}; }}"
    )


def realized(switch_log: Path, segs) -> dict:
    """Measured share of time in the last (event) segment, from the switch log."""
    t = [(int(i), float(ts)) for i, ts in (ln.split() for ln in switch_log.read_text().splitlines() if ln.strip())]
    dur = {i: 0.0 for i in range(len(segs))}
    for (i, a), (_, b) in zip(t, t[1:], strict=False):
        dur[i] += b - a
    total = sum(dur.values())
    last = len(segs) - 1
    n_ev = sum(1 for (i, _), _next in zip(t, t[1:], strict=False) if i == last)
    return {
        "switches": len(t),
        "event_share_realized": dur[last] / total if total else None,
        "event_share_nominal": segs[last][0] / sum(d for d, _ in segs),
        "event_mean_s": dur[last] / n_ev if n_ev else None,
    }


def loss_pct(plan, arrived) -> dict:
    out = {}
    for name, sel in (("kf", lambda kf: kf), ("delta", lambda kf: not kf), ("overall", lambda kf: True)):
        idx = [i for i, (_t, _s, kf) in enumerate(plan) if sel(kf)]
        if name != "overall":
            out[f"{name}_n"] = len(idx)
        out[f"{name}_loss_pct"] = round(100 * (1 - sum(arrived[i] for i in idx) / len(idx)), 3) if idx else None
    return out


def wait_container(name: str, timeout: float) -> None:
    result = run(["docker", "wait", name], timeout=timeout)
    if result.stdout.strip() != "0":
        logs = run(["docker", "logs", "--tail", "30", name])
        raise RuntimeError(f"{name} exited {result.stdout.strip()}: {logs.stdout}{logs.stderr}")


def one_run(args, scenario: str, rep: int, profiles: dict) -> dict:
    tag = f"{scenario}_{rep}"
    prefix = f"netem-replay-{uuid.uuid4().hex[:12]}-{tag}"
    net, recv, send = prefix, f"{prefix}-recv", f"{prefix}-send"
    work = args.work
    switch_log = work / f"switches_{tag}.log"
    switch_log.unlink(missing_ok=True)
    run(["docker", "network", "create", net])
    segs = segments(scenario, profiles, args.key)
    mounts = [
        "--user",
        f"{os.getuid()}:{os.getgid()}",
        "-v",
        f"{HERE.parent}:/w:ro",
        "-v",
        f"{args.plan}:/plan.csv:ro",
        "-v",
        f"{work}:/data",
    ]
    try:
        run(
            [
                "docker",
                "run",
                "-d",
                "--name",
                recv,
                "--network",
                net,
                *MEM,
                *mounts,
                IMAGE,
                "python3",
                f"/w/{HERE.name}",
                "recv",
                "--duration",
                str(args.duration),
                "--out",
                f"/data/recv_{tag}.csv",
            ]
        )
        ip = run(
            ["docker", "inspect", "-f", "{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}", recv]
        ).stdout.strip()
        run(
            [
                "docker",
                "run",
                "-d",
                "--name",
                send,
                "--network",
                net,
                *MEM,
                "--cap-add",
                "NET_ADMIN",
                *mounts,
                "--user",
                "0:0",
                IMAGE,
                "bash",
                "-c",
                sender_shell(segs, "/plan.csv", ip, args.duration, f"/data/{switch_log.name}"),
            ]
        )
        wait_container(send, timeout=args.duration + 120)
        wait_container(recv, timeout=120)
    finally:
        subprocess.run(["docker", "rm", "-f", recv, send], capture_output=True, timeout=30)
        subprocess.run(["docker", "network", "rm", net], capture_output=True, timeout=30)

    plan = read_plan(args.plan, args.duration)
    with open(work / f"recv_{tag}.csv") as f:
        complete = {int(r["frame_id"]) for r in csv.DictReader(f) if r["complete"] == "1"}
    res = {
        "scenario": scenario,
        "repeat": rep,
        "duration": args.duration,
        "frames": len(plan),
        "profile": f"{args.key}-{scenario}" if segs else None,
        "shaping": [[d if math.isfinite(d) else None, netem_args(u)] for d, u in segs] if segs else None,
    }
    res.update(loss_pct(plan, [i in complete for i in range(len(plan))]))
    if segs and len(segs) > 1 and switch_log.exists():
        res.update(realized(switch_log, segs))
    (work / f"result_{tag}.json").write_text(json.dumps(res, indent=2) + "\n")
    print(json.dumps({k: v for k, v in res.items() if k not in ("shaping", "profile")}), flush=True)
    return res


def expectation(scenario: str, profiles: dict, key: str, plan) -> dict:
    """Frame loss the profile implies for this plan, 1-(1-p)^n per frame, before any netem runs."""
    segs = segments(scenario, profiles, key)

    def rate(uplink: dict) -> float:
        return float(str(uplink.get("loss", "0%")).rstrip("%")) / 100

    def p_lost(size: int) -> float:
        if segs is None:
            return 0.0
        if len(segs) == 1:
            return 1 - (1 - rate(segs[0][1])) ** n_frags(size)
        total = sum(d for d, _ in segs)
        return sum(d / total * (1 - (1 - rate(u)) ** n_frags(size)) for d, u in segs)

    e = [p_lost(size) for _t, size, _kf in plan]
    out = {}
    for name, sel in (("kf", lambda kf: kf), ("delta", lambda kf: not kf), ("overall", lambda kf: True)):
        v = [e[i] for i, (_t, _s, kf) in enumerate(plan) if sel(kf)]
        out[f"{name}_loss_pct"] = round(100 * sum(v) / len(v), 3) if v else None
    return out


def summarize(args, profiles: dict) -> dict:
    plan = read_plan(args.plan, args.duration)
    out = {
        "profiles": PROFILES.name,
        "key": args.key,
        "duration_s": args.duration,
        "frames": len(plan),
        "scenarios": {},
    }
    for sc in SCENARIOS:
        runs = [json.loads(f.read_text()) for f in sorted(args.work.glob(f"result_{sc}_*.json"))]
        runs = [r for r in runs if r.get("scenario") == sc and r.get("duration") == args.duration]
        if not runs:
            continue
        agg: dict = {"runs": len(runs), "expected": expectation(sc, profiles, args.key, plan)}
        for k in ("overall_loss_pct", "kf_loss_pct", "delta_loss_pct"):
            v = [r[k] for r in runs if r[k] is not None]
            if not v:
                agg[k] = None
                continue
            agg[k] = {
                "mean": round(statistics.mean(v), 3),
                "sd": round(statistics.stdev(v), 3) if len(v) > 1 else 0.0,
                "min": min(v),
                "max": max(v),
            }
        if runs[0].get("event_share_realized") is not None:
            agg["event_share_realized_mean"] = round(statistics.mean(r["event_share_realized"] for r in runs), 4)
            agg["event_share_nominal"] = round(runs[0]["event_share_nominal"], 4)
        out["scenarios"][sc] = agg
    (args.work / "summary.json").write_text(json.dumps(out, indent=2) + "\n")
    return out


def load_profiles() -> dict:
    import yaml

    return yaml.safe_load(PROFILES.read_text())["profiles"]


def mode_orchestrate(args) -> int:
    profiles = load_profiles()
    if args.duration <= 0 or args.repeat <= 0 or not 1 <= args.parallel <= 4:
        raise ValueError("duration and repeat must be positive; parallel must be between 1 and 4")
    read_plan(args.plan, args.duration)
    for scenario in args.scenario:
        segments(scenario, profiles, args.key)
    if args.work.exists() and any(args.work.iterdir()):
        raise ValueError("work directory must be empty; choose a new directory for each experiment")
    args.work.mkdir(parents=True, exist_ok=True)
    if subprocess.run(["docker", "image", "inspect", IMAGE], capture_output=True, timeout=30).returncode:
        subprocess.run(["docker", "build", "-t", IMAGE, "-"], input=DOCKERFILE, text=True, check=True, timeout=600)
    jobs = [(sc, r) for sc in args.scenario for r in range(1, args.repeat + 1)]
    print(f"Running {len(jobs)} replays, {args.duration} s each, at most {args.parallel} in parallel", flush=True)

    def guarded(job):
        try:
            return one_run(args, job[0], job[1], profiles)
        except Exception as exc:  # one failed run must not discard the others
            print(f"FAILED {job}: {exc}", flush=True)
            return None

    with ThreadPoolExecutor(max_workers=args.parallel) as ex:
        results = list(ex.map(guarded, jobs))
    print(json.dumps(summarize(args, profiles)["scenarios"], indent=1))
    return 0 if all(results) else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode")
    for name in ("orchestrate", "summary"):
        p = sub.add_parser(name)
        p.add_argument("--key", default="r3", help="profile key: <key>-steady and <key>-events")
        p.add_argument("--plan", type=Path, default=HERE.parent / "r3_frame_plan.csv")
        p.add_argument("--work", type=Path, default=Path("replay-work"))
        p.add_argument("--duration", type=int, default=300, help="seconds of the plan to send")
    o = sub.choices["orchestrate"]
    o.add_argument("--scenario", nargs="+", default=list(SCENARIOS), choices=SCENARIOS)
    o.add_argument("--repeat", type=int, default=1)
    o.add_argument("--parallel", type=int, default=4)
    s = sub.add_parser("send")
    s.add_argument("--dest", required=True)
    s.add_argument("--plan", required=True)
    s.add_argument("--duration", type=int, default=300)
    r = sub.add_parser("recv")
    r.add_argument("--duration", type=int, default=300)
    r.add_argument("--out", required=True)
    argv = sys.argv[1:]
    if not argv or argv[0].startswith("-"):
        argv = ["orchestrate", *argv]
    args = ap.parse_args(argv)
    if args.mode == "send":
        mode_send(args)
        return 0
    if args.mode == "recv":
        mode_recv(args)
        return 0
    args.plan, args.work = args.plan.resolve(), args.work.resolve()
    if args.mode == "summary":
        print(json.dumps(summarize(args, load_profiles()), indent=1))
        return 0
    return mode_orchestrate(args)


if __name__ == "__main__":
    sys.exit(main())
