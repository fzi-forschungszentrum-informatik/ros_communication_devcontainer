# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Receiver-side playout pacer node: re-time a topic to `stamp + budget`.

Subscribes ``topic`` (any type with a ``header.stamp``, resolved dynamically),
holds each message until its release time from
:class:`com_py.playout_pacer_core.PlayoutSchedule`, and republishes on
``<topic><topic_suffix>`` (default ``/paced``) — in order, never dropping.
Inserted between the OTA unwrapper and a decoder chain it turns network jitter
into a constant, bounded age instead of visible stutter.

Debug topics next to the output: ``.../paced/budget_ms`` (current delay
budget) and ``.../paced/queue_depth``, both at 1 Hz, and ``.../paced/hold_ms``
— published with *every* release, carrying how long that message was actually
held.

``hold_ms`` exists so the status overview can tell the link's contribution from
this node's. Deliberately the applied hold rather than the budget: a message
that arrived later than its release time passes straight through and is held
for 0 ms, so a genuinely slow link cannot be credited with a buffer it never
used. Subtracting the *budget* would do exactly that, and would quietly stop
reporting the problem it exists to make visible.
"""

from __future__ import annotations

import collections
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64, Int32

from com_py.playout_pacer_core import PacerConfig, PlayoutSchedule
from com_py.qos import get_topic_qos, load_qos_config


class PlayoutPacerNode(Node):
    def __init__(self) -> None:
        super().__init__("playout_pacer")
        self.declare_parameter("topic", "")
        self.declare_parameter("msg_type", "ffmpeg_image_transport_msgs/msg/FFMPEGPacket")
        self.declare_parameter("topic_suffix", "/paced")
        self.declare_parameter("target_ms", 350.0)
        self.declare_parameter("min_ms", 100.0)
        self.declare_parameter("max_ms", 800.0)
        self.declare_parameter("adaptive", True)
        self.declare_parameter("qos_config_file", "")

        topic = self.get_parameter("topic").value
        if not topic:
            raise ValueError("playout_pacer requires a 'topic' parameter")
        msg_cls = get_message(self.get_parameter("msg_type").value)
        out_topic = topic + self.get_parameter("topic_suffix").value

        self.schedule = PlayoutSchedule(
            PacerConfig(
                target_ms=float(self.get_parameter("target_ms").value),
                min_ms=float(self.get_parameter("min_ms").value),
                max_ms=float(self.get_parameter("max_ms").value),
                adaptive=bool(self.get_parameter("adaptive").value),
            )
        )
        self._queue: collections.deque = collections.deque()

        qos_file = self.get_parameter("qos_config_file").value
        if qos_file:
            qos_config = load_qos_config(self.get_logger(), qos_file)
            qos = get_topic_qos(self.get_logger(), qos_config, topic, "playout_pacer")
        else:
            qos = 10
        self._pub = self.create_publisher(msg_cls, out_topic, qos)
        self._budget_pub = self.create_publisher(Float64, out_topic + "/budget_ms", 10)
        self._depth_pub = self.create_publisher(Int32, out_topic + "/queue_depth", 10)
        self._hold_pub = self.create_publisher(Float64, out_topic + "/hold_ms", 10)
        self.create_subscription(msg_cls, topic, self._on_message, qos)
        # 2 ms release tick: bounds added jitter to well below frame cadence.
        self.create_timer(0.002, self._release_due)
        self.create_timer(1.0, self._publish_debug)
        self.get_logger().info(f"pacing {topic} -> {out_topic}")

    def _on_message(self, msg) -> None:
        now = time.time()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        release = self.schedule.on_message(stamp, now)
        self._queue.append((release, msg, now))

    def _release_due(self) -> None:
        now = time.time()
        while self._queue and self._queue[0][0] <= now:
            release, msg, arrival = self._queue.popleft()
            self._pub.publish(msg)
            # What this node added, not what it was allowed to add. Clamped
            # because a late message's release time is already in the past.
            self._hold_pub.publish(Float64(data=max(0.0, (release - arrival) * 1000.0)))

    def _publish_debug(self) -> None:
        self._budget_pub.publish(Float64(data=float(self.schedule.budget_ms)))
        self._depth_pub.publish(Int32(data=len(self._queue)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlayoutPacerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
