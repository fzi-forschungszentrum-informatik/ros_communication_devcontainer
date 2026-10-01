# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Keyframe identification for ffmpeg camera streams (FFMPEGPacket).

An ffmpeg-transported camera stream is a sequence of
``ffmpeg_image_transport_msgs/msg/FFMPEGPacket`` messages with a periodic GOP
structure: one keyframe (I-frame, self-contained, large) followed by
``gop_size - 1`` delta frames (P-frames, tiny). The two classes differ in size
by one to two orders of magnitude, so distinguishing them is essential for
network analysis -- the keyframes are the bursts a link actually has to absorb.

Primary algorithm: the packet's ``flags`` field mirrors libav's packet flags;
bit 0 is ``AV_PKT_FLAG_KEY`` -- set exactly on keyframes. ``rosotacom
anonymize`` preserves ``flags`` (and ``pts``) when it zeroes payloads, so this
works on live streams, recorded handoff traces, and anonymized replay bags
alike. See docs/ffmpeg-keyframes.md.

This module parses the serialized CDR bytes directly (as stored in mcap
recordings or delivered by raw subscriptions), so offline tooling needs no ROS
runtime. Only little-endian CDR (the ROS 2 default on all supported platforms)
is handled.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from statistics import median

AV_PKT_FLAG_KEY = 0x1


@dataclass(frozen=True)
class FFMPEGPacketInfo:
    """Structure of one serialized FFMPEGPacket, without its payload bytes."""

    stamp_sec: int
    stamp_nanosec: int
    frame_id: str
    width: int
    height: int
    encoding: str
    pts: int
    flags: int
    payload_size: int

    @property
    def is_keyframe(self) -> bool:
        return bool(self.flags & AV_PKT_FLAG_KEY)


def _align(offset: int, alignment: int) -> int:
    return (offset + alignment - 1) // alignment * alignment


def parse_ffmpeg_packet(cdr: bytes) -> FFMPEGPacketInfo:
    """Parse a serialized ``FFMPEGPacket`` (CDR bytes as stored in a bag).

    Field order per the message definition: ``std_msgs/Header header``,
    ``int32 width``, ``int32 height``, ``string encoding``, ``uint64 pts``,
    ``uint8 flags``, ``bool is_bigendian``, ``uint8[] data``. CDR alignment is
    relative to the payload start, after the 4-byte encapsulation header.
    """
    if len(cdr) < 4 or cdr[1] != 0x01:
        raise ValueError("expected little-endian CDR encapsulation (0x00 0x01 ...)")
    body = cdr[4:]
    offset = 0

    stamp_sec, stamp_nanosec = struct.unpack_from("<iI", body, offset)
    offset += 8

    offset = _align(offset, 4)
    (str_len,) = struct.unpack_from("<I", body, offset)
    offset += 4
    frame_id = body[offset : offset + str_len - 1].decode("utf-8", errors="replace")
    offset += str_len

    offset = _align(offset, 4)
    width, height = struct.unpack_from("<ii", body, offset)
    offset += 8

    offset = _align(offset, 4)
    (str_len,) = struct.unpack_from("<I", body, offset)
    offset += 4
    encoding = body[offset : offset + str_len - 1].decode("utf-8", errors="replace")
    offset += str_len

    offset = _align(offset, 8)
    (pts,) = struct.unpack_from("<Q", body, offset)
    offset += 8

    flags = body[offset]
    offset += 2  # flags + is_bigendian

    offset = _align(offset, 4)
    (payload_size,) = struct.unpack_from("<I", body, offset)

    return FFMPEGPacketInfo(
        stamp_sec=stamp_sec,
        stamp_nanosec=stamp_nanosec,
        frame_id=frame_id,
        width=width,
        height=height,
        encoding=encoding,
        pts=pts,
        flags=flags,
        payload_size=payload_size,
    )


def keyframes_by_size(sizes: list[int] | list[float], *, min_ratio: float = 3.0) -> list[bool]:
    """Size-bimodality fallback when ``flags`` is unavailable or was zeroed.

    Marks a frame as keyframe iff its size exceeds ``min_ratio`` times the
    stream's median frame size. Works because delta frames dominate the stream
    (a GOP of N has N-1 of them), pinning the median to the delta-frame mode,
    while keyframes are typically 5-100x larger. Misclassifies only when the
    encoder emits unusually large delta frames (heavy motion) or near-empty
    keyframes (static scene at very low bitrate) -- prefer ``flags`` whenever
    present.
    """
    if not sizes:
        return []
    threshold = min_ratio * median(sizes)
    return [size > threshold for size in sizes]
