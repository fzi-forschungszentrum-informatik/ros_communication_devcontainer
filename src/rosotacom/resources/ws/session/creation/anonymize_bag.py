#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import argparse
import array
import json
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

# Fields that anonymization must keep per message type (ROS 2 Python message
# __slots__ carry a leading underscore). Anonymization deliberately preserves
# structure, sizes, and timing; these fields are equally non-sensitive stream
# structure. FFMPEGPacket.flags bit 0 (libav AV_PKT_FLAG_KEY) marks keyframes
# and pts keeps the encode order -- both are essential for GOP-aware network
# analysis of the replayed stream and reveal nothing about scene content.
PRESERVED_SLOTS: dict[str, frozenset[str]] = {
    "FFMPEGPacket": frozenset({"_flags", "_pts"}),
}


def anonymize_msg(msg) -> None:
    """Recursively anonymize message fields in place, preserving structures and sizes."""
    if not hasattr(msg, "__slots__"):
        return

    preserved = PRESERVED_SLOTS.get(type(msg).__name__, frozenset())
    for slot in msg.__slots__:
        if slot in preserved:
            continue
        val = getattr(msg, slot)
        if val is None:
            continue

        val_type_name = type(val).__name__
        val_module = getattr(type(val), "__module__", "")
        if val_type_name in ("Time", "Duration") and val_module.startswith("builtin_interfaces"):
            continue

        if hasattr(val, "__slots__"):
            anonymize_msg(val)
        elif isinstance(val, (list, tuple)):
            if isinstance(val, tuple):
                mut_val = list(val)
                for i in range(len(mut_val)):
                    if hasattr(mut_val[i], "__slots__"):
                        anonymize_msg(mut_val[i])
                    elif isinstance(mut_val[i], str):
                        mut_val[i] = "x" * len(mut_val[i])
                    elif isinstance(mut_val[i], (int, float)):
                        mut_val[i] = type(mut_val[i])(0)
                    elif isinstance(mut_val[i], bool):
                        mut_val[i] = False
                setattr(msg, slot, tuple(mut_val))
            else:
                for i in range(len(val)):
                    if hasattr(val[i], "__slots__"):
                        anonymize_msg(val[i])
                    elif isinstance(val[i], str):
                        val[i] = "x" * len(val[i])
                    elif isinstance(val[i], (int, float)):
                        val[i] = type(val[i])(0)
                    elif isinstance(val[i], bool):
                        val[i] = False
        elif isinstance(val, (bytes, bytearray)):
            setattr(msg, slot, type(val)(len(val)))
        elif isinstance(val, array.array):
            setattr(msg, slot, array.array(val.typecode, [0] * len(val)))
        elif isinstance(val, str):
            setattr(msg, slot, "x" * len(val))
        elif isinstance(val, bool):
            setattr(msg, slot, False)
        elif isinstance(val, (int, float)):
            setattr(msg, slot, type(val)(0))


def main() -> int:
    parser = argparse.ArgumentParser(description="Anonymize a rosbag's messages and topics.")
    parser.add_argument("--input-bag", required=True, help="Path to input bag directory")
    parser.add_argument("--output-bag", required=True, help="Path to output bag directory")
    parser.add_argument("--topics-map", required=True, help="JSON string mapping original to anonymized topics")
    parser.add_argument("--storage-id", default="mcap", help="Rosbag2 storage identifier")
    args = parser.parse_args()

    input_path = Path(args.input_bag)
    output_path = Path(args.output_bag)
    topics_map = json.loads(args.topics_map)

    if not input_path.exists():
        print(f"Error: input bag '{input_path}' does not exist", file=sys.stderr)
        return 1

    # Initialize sequential reader
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(input_path), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )

    # The reader's typed TopicMetadata (with rosbag2_py.QoS offered profiles) is
    # the only construction the writer accepts; re-parsing metadata.yaml into
    # dicts breaks on current rosbag2.
    input_topics = {tm.name: tm for tm in reader.get_all_topics_and_types()}

    missing_topics = [orig for orig in topics_map if orig not in input_topics]
    if missing_topics:
        print("Error: mapped topic(s) missing from the input bag:", file=sys.stderr)
        for topic in missing_topics:
            print(f"  {topic}", file=sys.stderr)
        return 1
    active_topics_map = dict(topics_map)

    # Initialize sequential writer
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_path), storage_id=args.storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )

    # Register output topics with the writer
    for orig, anonymized in active_topics_map.items():
        tm = input_topics[orig]
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=0,
                name=anonymized,
                type=tm.type,
                serialization_format=tm.serialization_format,
                offered_qos_profiles=tm.offered_qos_profiles,
            )
        )

    # Read and anonymize messages
    print(f"Anonymizing bag: {input_path} -> {output_path}")
    count = 0
    while reader.has_next():
        topic, serialized_bytes, timestamp = reader.read_next()
        if topic in active_topics_map:
            anonymized_topic = active_topics_map[topic]
            msg_cls = get_message(input_topics[topic].type)
            msg = deserialize_message(serialized_bytes, msg_cls)
            anonymize_msg(msg)
            anonymized_bytes = serialize_message(msg)
            writer.write(anonymized_topic, anonymized_bytes, timestamp)
            count += 1

    print(f"Successfully wrote {count} anonymized messages to {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
