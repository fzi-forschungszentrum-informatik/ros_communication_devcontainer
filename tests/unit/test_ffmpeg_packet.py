# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import struct

import pytest

from rosotacom.ffmpeg_packet import (
    AV_PKT_FLAG_KEY,
    keyframes_by_size,
    parse_ffmpeg_packet,
)


def _cdr_string(offset: int, value: str) -> tuple[bytes, int]:
    pad = (-offset) % 4
    raw = value.encode() + b"\x00"
    chunk = b"\x00" * pad + struct.pack("<I", len(raw)) + raw
    return chunk, offset + len(chunk)


def make_packet(
    *,
    frame_id: str = "camera",
    width: int = 640,
    height: int = 480,
    encoding: str = "h264",
    pts: int = 42,
    flags: int = AV_PKT_FLAG_KEY,
    payload: bytes = b"\x01\x02\x03",
) -> bytes:
    """Serialize an FFMPEGPacket by hand, following XCDR1 little-endian rules."""
    body = b""
    offset = 0
    body += struct.pack("<iI", 7, 9)  # header.stamp
    offset += 8
    chunk, offset = _cdr_string(offset, frame_id)
    body += chunk
    pad = (-offset) % 4
    body += b"\x00" * pad + struct.pack("<ii", width, height)
    offset += pad + 8
    chunk, offset = _cdr_string(offset, encoding)
    body += chunk
    pad = (-offset) % 8
    body += b"\x00" * pad + struct.pack("<Q", pts)
    offset += pad + 8
    body += struct.pack("<BB", flags, 0)  # flags + is_bigendian
    offset += 2
    pad = (-offset) % 4
    body += b"\x00" * pad + struct.pack("<I", len(payload)) + payload
    return b"\x00\x01\x00\x00" + body


# Odd-length strings shift subsequent fields across alignment boundaries; a
# parser with a wrong alignment origin fails exactly there.
@pytest.mark.parametrize("frame_id", ["camera", "c", "front_medium_x"])
@pytest.mark.parametrize("encoding", ["h264", "hevc_nvenc"])
def test_parse_round_trips_hand_built_packets(frame_id: str, encoding: str) -> None:
    cdr = make_packet(frame_id=frame_id, encoding=encoding, pts=123456789, flags=1, payload=b"\xaa" * 501)

    info = parse_ffmpeg_packet(cdr)

    assert info.stamp_sec == 7
    assert info.stamp_nanosec == 9
    assert info.frame_id == frame_id
    assert info.width == 640
    assert info.height == 480
    assert info.encoding == encoding
    assert info.pts == 123456789
    assert info.flags == 1
    assert info.is_keyframe
    assert info.payload_size == 501


def test_delta_frame_is_not_keyframe() -> None:
    info = parse_ffmpeg_packet(make_packet(flags=0))

    assert info.flags == 0
    assert not info.is_keyframe


def test_non_key_flag_bits_do_not_mark_keyframes() -> None:
    # e.g. AV_PKT_FLAG_CORRUPT (0x2) alone is not a keyframe marker.
    assert not parse_ffmpeg_packet(make_packet(flags=0x2)).is_keyframe
    assert parse_ffmpeg_packet(make_packet(flags=0x3)).is_keyframe


def test_big_endian_encapsulation_is_rejected() -> None:
    cdr = bytearray(make_packet())
    cdr[1] = 0x00

    with pytest.raises(ValueError, match="little-endian"):
        parse_ffmpeg_packet(bytes(cdr))


def test_keyframes_by_size_marks_the_bimodal_upper_mode() -> None:
    # A GOP-of-5 stream: the median sits on the delta frames, keyframes are
    # far above 3x that median.
    sizes = [44000, 3000, 4000, 4200, 3900] * 4

    mask = keyframes_by_size(sizes)

    assert mask == [True, False, False, False, False] * 4


def test_keyframes_by_size_empty_input() -> None:
    assert keyframes_by_size([]) == []
