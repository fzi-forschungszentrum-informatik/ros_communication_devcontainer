# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unit tests for com_py/ffmpeg_flags.py — the receiver-side keyframe-bit walk.

The module mirrors ``rosotacom.ffmpeg_packet.parse_ffmpeg_packet`` but runs in
the ROS container without the host analysis package, so it is loaded by file
path and cross-checked against the host parser on the same hand-built CDR
bytes. Unlike the host parser it must never raise into a subscription
callback: every unparseable input yields ``None``.
"""

from __future__ import annotations

import importlib.util
import struct
import sys
from pathlib import Path
from types import ModuleType

import pytest

from rosotacom.ffmpeg_packet import AV_PKT_FLAG_KEY, parse_ffmpeg_packet

REPO_ROOT = Path(__file__).resolve().parents[2]
FFMPEG_FLAGS_PY = (
    REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py" / "ffmpeg_flags.py"
)


def _load(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


ffmpeg_flags = _load(FFMPEG_FLAGS_PY, "rosotacom_com_py_ffmpeg_flags")


def _cdr_string(offset: int, value: str) -> tuple[bytes, int]:
    pad = (-offset) % 4
    raw = value.encode() + b"\x00"
    chunk = b"\x00" * pad + struct.pack("<I", len(raw)) + raw
    return chunk, offset + len(chunk)


def make_packet(
    *,
    frame_id: str = "camera",
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
    body += b"\x00" * pad + struct.pack("<ii", 640, 480)
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


def _flags_offset() -> int:
    """Index of the flags byte, located by diffing a flags=0 against a flags=1 build."""
    diffs = [
        index for index, (a, b) in enumerate(zip(make_packet(flags=0), make_packet(flags=1), strict=True)) if a != b
    ]
    assert len(diffs) == 1
    return diffs[0]


def test_keyframe_bit_set_and_unset() -> None:
    assert ffmpeg_flags.parse_keyframe_flag(make_packet(flags=AV_PKT_FLAG_KEY)) is True
    assert ffmpeg_flags.parse_keyframe_flag(make_packet(flags=0)) is False


def test_non_key_flag_bits_do_not_mark_keyframes() -> None:
    # e.g. AV_PKT_FLAG_CORRUPT (0x2) alone is not a keyframe marker.
    assert ffmpeg_flags.parse_keyframe_flag(make_packet(flags=0x2)) is False
    assert ffmpeg_flags.parse_keyframe_flag(make_packet(flags=0x3)) is True


# Odd-length strings shift subsequent fields across alignment boundaries; a
# walk with a wrong alignment origin reads the wrong byte exactly there.
@pytest.mark.parametrize("frame_id", ["camera", "c", "front_medium_x", ""])
@pytest.mark.parametrize("encoding", ["h264", "hevc_nvenc", "x"])
@pytest.mark.parametrize("flags", [0, 1])
def test_agrees_with_the_host_parser_on_hand_built_packets(frame_id: str, encoding: str, flags: int) -> None:
    cdr = make_packet(frame_id=frame_id, encoding=encoding, flags=flags)

    assert ffmpeg_flags.parse_keyframe_flag(cdr) is parse_ffmpeg_packet(cdr).is_keyframe


def test_every_truncation_before_the_flags_byte_yields_none() -> None:
    cdr = make_packet(flags=AV_PKT_FLAG_KEY)
    offset = _flags_offset()

    for length in range(offset + 1):
        assert ffmpeg_flags.parse_keyframe_flag(cdr[:length]) is None
    # The flags byte itself is the last one the walk needs.
    assert ffmpeg_flags.parse_keyframe_flag(cdr[: offset + 1]) is True


def test_big_endian_encapsulation_yields_none() -> None:
    cdr = bytearray(make_packet())
    cdr[1] = 0x00

    assert ffmpeg_flags.parse_keyframe_flag(bytes(cdr)) is None


def test_garbage_string_length_yields_none_not_an_exception() -> None:
    # A frame_id length pointing far past the buffer must not escape as an error.
    cdr = bytearray(make_packet())
    struct.pack_into("<I", cdr, 12, 2**31)  # frame_id length field

    assert ffmpeg_flags.parse_keyframe_flag(bytes(cdr)) is None
