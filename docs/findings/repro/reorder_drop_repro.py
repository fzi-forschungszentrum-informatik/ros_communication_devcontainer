#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Does jitter-induced reordering become LOSS at a CycloneDDS best-effort
reader, and does history depth help?

Two containers (rcd image, cyclone RMW). Publisher shapes its egress with
pure delay+jitter (NO configured loss, no rate limit). Receiver subscribes
the same topic twice: depth 1 and depth 50.

Discriminates two mechanisms:
  - loss seen by depth-50 sub  = reader-proxy drop of overtaken samples
    (depth cannot help)        (or nothing, if cyclone delivers OOO)
  - extra loss of depth-1 sub  = history overwrite from arrival bunching
    (depth DOES help)
  - delivered inversions       = out-of-order delivery (would mean no drop)

Modes: orchestrate (host) | pub | sub (in container).
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

IMG = "ros-communication:latest"  # any image with rclpy + rmw_cyclonedds + tc (--image overrides)
NET = "reorder_repro_net"
TOPIC = "/reorder_probe"


def mode_pub(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int64

    rclpy.init()
    node = Node("reorder_pub")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
    pub = node.create_publisher(Int64, TOPIC, qos)
    time.sleep(5.0)  # discovery
    n = int(args.rate * args.duration)
    period = 1.0 / args.rate
    t0 = time.monotonic()
    for i in range(n):
        while time.monotonic() < t0 + i * period:
            time.sleep(0.0005)
        m = Int64()
        m.data = i
        pub.publish(m)
    time.sleep(3.0)
    print(f"published {n}")
    node.destroy_node()
    rclpy.shutdown()


def mode_sub(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int64

    rclpy.init()
    node = Node("reorder_sub")
    got = {1: [], 50: []}

    def make_cb(depth):
        def cb(msg):
            got[depth].append(msg.data)

        return cb

    subs = []
    for depth in (1, 50):
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth)
        subs.append(node.create_subscription(Int64, TOPIC, make_cb(depth), qos))
    t_end = time.monotonic() + args.duration + 20
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    out = {}
    for depth, seqs in got.items():
        inv = sum(1 for a, b in zip(seqs, seqs[1:], strict=False) if b < a)
        out[f"depth{depth}"] = {
            "received": len(seqs),
            "unique": len(set(seqs)),
            "inversions": inv,
            "first": seqs[0] if seqs else None,
            "last": max(seqs) if seqs else None,
        }
    Path(args.out).write_text(json.dumps(out, indent=2))
    print(json.dumps(out))
    node.destroy_node()
    rclpy.shutdown()


def run(cmd):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f"{cmd}\n{r.stderr or r.stdout}")
    return r.stdout


def mode_orchestrate(args):
    global IMG
    IMG = args.image
    me = Path(__file__).resolve()
    outdir = me.parent / "out"
    scenarios = [
        ("jitter45", "tc qdisc replace dev eth0 root netem delay 50ms 45ms"),
        ("jitter20", "tc qdisc replace dev eth0 root netem delay 50ms 20ms"),
    ]
    ip_sub, ip_pub = "10.97.0.2", "10.97.0.3"
    ddsuri = (
        "<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast>"
        '</General><Discovery><Peers><Peer address=\\"' + ip_sub + '\\"/>'
        '<Peer address=\\"' + ip_pub + '\\"/></Peers>'
        "<ParticipantIndex>auto</ParticipantIndex></Discovery></Domain></CycloneDDS>"
    )
    subprocess.run(f"docker network rm {NET}", shell=True, capture_output=True)
    run(f"docker network create --subnet 10.97.0.0/24 {NET}")
    results = {}
    try:
        for name, netem in scenarios:
            subprocess.run("docker rm -f reorder_repro_pub reorder_repro_sub", shell=True, capture_output=True)
            env = f'-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=77 -e CYCLONEDDS_URI="{ddsuri}"'
            run(
                f"docker run -d --name reorder_repro_sub --network {NET} --ip {ip_sub} {env} "
                f"-v {me.parent}:/w -v {outdir}:/out {IMG} bash -lc "
                f"'python3 /w/{me.name} sub --duration {args.duration} "
                f"--out /out/reorder_{name}.json'"
            )
            time.sleep(2)
            run(
                f"docker run -d --name reorder_repro_pub --network {NET} --ip {ip_pub} "
                f"--cap-add NET_ADMIN {env} -v {me.parent}:/w {IMG} bash -lc "
                f"'{netem} && python3 /w/{me.name} pub --rate {args.rate} "
                f"--duration {args.duration}'"
            )
            run("docker wait reorder_repro_pub")
            run("docker wait reorder_repro_sub")
            data = json.loads((outdir / f"reorder_{name}.json").read_text())
            sent = int(args.rate * args.duration)
            for d in ("depth1", "depth50"):
                span = (data[d]["last"] or 0) - (data[d]["first"] or 0) + 1
                data[d]["loss_pct_of_span"] = round(100 * (1 - data[d]["unique"] / max(span, 1)), 2)
            data["sent"] = sent
            results[name] = data
            print(f"== {name}: {json.dumps(data)}")
    finally:
        subprocess.run("docker rm -f reorder_repro_pub reorder_repro_sub", shell=True, capture_output=True)
        subprocess.run(f"docker network rm {NET}", shell=True, capture_output=True)
    (outdir / "reorder_results.json").write_text(json.dumps(results, indent=2))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    for m in ("orchestrate", "pub", "sub"):
        p = sub.add_parser(m)
        p.add_argument("--rate", type=float, default=100.0)
        p.add_argument("--duration", type=float, default=40.0)
        p.add_argument("--image", default=IMG)
        if m == "sub":
            p.add_argument("--out", required=True)
    args = ap.parse_args()
    {"pub": mode_pub, "sub": mode_sub, "orchestrate": mode_orchestrate}[args.mode](args)


if __name__ == "__main__":
    sys.exit(main() or 0)
