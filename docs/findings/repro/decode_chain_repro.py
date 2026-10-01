#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Reproduce the keyframe decode-chain rule of a best-effort libx264 stream.

Encodes a deterministic synthetic motion clip the way the ffmpeg transport
does (libx264, ABR, ``tune: zerolatency``, fixed GOP), then deletes single
access units and feeds the survivors to a fresh libavcodec h264 decoder --
which is exactly what a best-effort receiver does after a network loss.

Measured rule (docs/findings/keyframe-loss-kills-the-next-deltas.md):

* delete a KEYFRAME  -> the next ``gop_size - 2`` arrived deltas produce NO
  output at all, the last delta before the next keyframe decodes with
  concealment artifacts, the next keyframe restores clean decode;
* delete a DELTA     -> every following access unit still decodes (artifacts
  until the next keyframe, never a gap).

Usage (needs ``pip install av numpy``; no network, no privileges)::

    python3 docs/findings/repro/decode_chain_repro.py [--gop 5] [--frames 40]

Prints the per-AU decode table for both deletions and exits non-zero if the
rule above does not hold.
"""

from __future__ import annotations

import argparse
import sys
from fractions import Fraction

import av
import numpy as np


def synthetic_frames(count: int, width: int = 320, height: int = 240):
    """Deterministic motion: drifting background plus a moving block."""
    for index in range(count):
        image = np.full((height, width, 3), (index * 3) % 200, dtype=np.uint8)
        x = (index * 7) % (width - 40)
        image[60:180, x : x + 40] = 255
        frame = av.VideoFrame.from_ndarray(image, format="rgb24")
        yield frame.reformat(format="yuv420p")


def encode(gop: int, count: int) -> list[tuple[bytes, bool]]:
    codec = av.CodecContext.create("libx264", "w")
    codec.width, codec.height = 320, 240
    codec.pix_fmt = "yuv420p"
    codec.framerate = Fraction(10, 1)
    codec.bit_rate = 1_000_000
    codec.options = {"tune": "zerolatency", "g": str(gop)}
    packets: list[tuple[bytes, bool]] = []
    for frame in synthetic_frames(count):
        for packet in codec.encode(frame):
            packets.append((bytes(packet), bool(packet.is_keyframe)))
    for packet in codec.encode(None):
        packets.append((bytes(packet), bool(packet.is_keyframe)))
    return packets


def decode_survivors(packets: list[tuple[bytes, bool]], deleted: set[int]) -> list[tuple[int, int]]:
    """Feed every surviving AU to a fresh decoder; return (au_index, frames_out)."""
    decoder = av.CodecContext.create("h264", "r")
    out: list[tuple[int, int]] = []
    for index, (payload, _kf) in enumerate(packets):
        if index in deleted:
            continue
        try:
            frames = decoder.decode(av.Packet(payload))
        except av.error.InvalidDataError:
            frames = []
        out.append((index, len(frames)))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gop", type=int, default=5)
    parser.add_argument("--frames", type=int, default=40)
    args = parser.parse_args()

    packets = encode(args.gop, args.frames)
    kf_indices = [i for i, (_p, kf) in enumerate(packets) if kf]
    if len(kf_indices) < 3:
        print("clip too short to delete a mid-stream keyframe", file=sys.stderr)
        return 2

    baseline = decode_survivors(packets, set())
    silent_baseline = [i for i, n in baseline if n == 0]

    failures: list[str] = []

    # --- delete the second keyframe -------------------------------------
    kf = kf_indices[1]
    next_kf = kf_indices[2]
    rows = decode_survivors(packets, {kf})
    silent = [i for i, n in rows if n == 0 and i not in silent_baseline]
    deltas_in_gap = [i for i in range(kf + 1, next_kf)]
    expected_silent = deltas_in_gap[: max(0, args.gop - 2)]
    print(f"deleted keyframe AU {kf} (gop {args.gop}, next keyframe {next_kf})")
    print(f"  silent AUs: {silent}")
    print(f"  expected  : {expected_silent} (the next gop_size-2 deltas)")
    if silent != expected_silent:
        failures.append(f"keyframe deletion: silent AUs {silent} != expected {expected_silent}")
    resumed = [i for i, n in rows if i >= next_kf and n == 0]
    if resumed:
        failures.append(f"decode did not resume at the next keyframe: silent {resumed}")

    # --- delete one delta ------------------------------------------------
    delta = kf_indices[1] + 1
    rows = decode_survivors(packets, {delta})
    silent = [i for i, n in rows if n == 0 and i not in silent_baseline]
    print(f"deleted delta AU {delta}")
    print(f"  silent AUs: {silent} (must be empty -- artifacts, never a gap)")
    if silent:
        failures.append(f"delta deletion silenced AUs {silent}; expected none")

    if failures:
        print("FINDING VIOLATED:", *failures, sep="\n  ", file=sys.stderr)
        return 1
    print("ok: keyframe loss silences exactly the next gop_size-2 deltas; delta loss silences nothing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
