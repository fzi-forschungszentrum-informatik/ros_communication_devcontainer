# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Keyframe bit of a serialized FFMPEGPacket, straight from its CDR bytes.

An ``OtaStamped`` envelope carries the wrapped message as serialized CDR
(``serialized_msg``), so the receiving status overview can read the packet's
``flags`` field -- bit 0 is libav's ``AV_PKT_FLAG_KEY``, set exactly on
keyframes -- without deserializing through rclpy. That matters twice: the inner
message package need not be installed on the receiving peer, and the OTA
observer must never grow a dependency on it just for one bit.

This mirrors the field walk of ``rosotacom.ffmpeg_packet.parse_ffmpeg_packet``
(the host-side analysis parser) but lives in com_py because the ROS container
does not ship the host analysis package. Only little-endian CDR (the ROS 2
default on all supported platforms) is handled; anything else -- including a
truncated or foreign buffer -- yields ``None``, never an exception.
"""

from __future__ import annotations

import struct
from typing import Optional

AV_PKT_FLAG_KEY = 0x1

FFMPEG_PACKET_TYPE = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"


def _align(offset: int, alignment: int) -> int:
    return (offset + alignment - 1) // alignment * alignment


def parse_keyframe_flag(cdr: bytes) -> Optional[bool]:
    """``flags & AV_PKT_FLAG_KEY`` of a serialized FFMPEGPacket, or ``None``.

    Field order per the message definition: ``std_msgs/Header header``,
    ``int32 width``, ``int32 height``, ``string encoding``, ``uint64 pts``,
    ``uint8 flags``, ... CDR alignment is relative to the payload start,
    after the 4-byte encapsulation header.
    """
    try:
        if len(cdr) < 4 or cdr[1] != 0x01:
            return None
        body = cdr[4:]
        offset = 8  # header.stamp: int32 sec + uint32 nanosec

        offset = _align(offset, 4)
        (str_len,) = struct.unpack_from("<I", body, offset)
        offset += 4 + str_len  # frame_id, NUL included in str_len

        offset = _align(offset, 4) + 8  # int32 width + int32 height

        offset = _align(offset, 4)
        (str_len,) = struct.unpack_from("<I", body, offset)
        offset += 4 + str_len  # encoding

        offset = _align(offset, 8) + 8  # uint64 pts
        return bool(body[offset] & AV_PKT_FLAG_KEY)
    except (struct.error, IndexError):
        return None
