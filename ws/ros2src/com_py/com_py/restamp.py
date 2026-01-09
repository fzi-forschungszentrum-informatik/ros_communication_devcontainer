#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2026-01-09
#
# ---------------------------------------------------------------------

from __future__ import annotations

"""
Generic timestamp rebasing node ("restamp").

Rebases all occurrences of fields named `stamp` (including header.stamp and nested stamps)
to "now" to create a common, monotonic timeline across topics.

This is helpful when dealing with looping / switching rosbags to avoid backward time
jumps during bag switching/looping, which can break TF buffering and visualization caching.
--clock alone doesn’t prevent these backward jumps; it only publishes the bag’s time.
"""

from typing import Any, List

import rclpy
from rclpy.node import Node

from com_py.msg_transform import _iter_field_names, update_timestamps_in_msg
from com_py.pair_management import PairRefreshMixin
from com_py.pub_sub_pair import PubSubPair
from com_py.qos import load_qos_config


class RestampPubSubPair(PubSubPair):
    def __init__(self, *, node, **kwargs):
        super().__init__(node=node, **kwargs)

    def _callback(self, msg):
        if self.first_msg:
            self.logger.info(
                f"[RestampPubSubPair] FIRST msg on sub='{self.sub_topic}' => pub='{self.pub_topic}'"
            )
            self.first_msg = False

        out = self.node.transform_message(msg, topic=self.sub_topic)
        self.publisher.publish(out)


class Restamp(Node, PairRefreshMixin):
    def __init__(self) -> None:
        super().__init__("restamp")

        # -----------------------
        # Parameters
        # -----------------------
        self.topic_suffix = self.declare_parameter("topic_suffix", "/restamped").value
        self.qos_config_file = self.declare_parameter("qos_config_file", "").value
        self.sub_role = "restamp_sub"
        self.pub_role = "restamp_pub"

        # Base topics to process
        self.restamp_topics = (
            self.declare_parameter("restamp_topics", rclpy.Parameter.Type.STRING_ARRAY).value
            or []
        )  # subscribe base, publish base+suffix

        # Debug params
        self.debug_first_n = int(self.declare_parameter("debug_first_n", 3).value)
        self.debug_samples_per_msg = int(self.declare_parameter("debug_samples_per_msg", 5).value)
        self.debug_log_unchanged = bool(self.declare_parameter("debug_log_unchanged", True).value)

        self._dbg_seen: dict[str, int] = {}
        self._dbg_warned: set[str] = set()

        # QoS
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # -----------------------
        # Startup logging
        # -----------------------
        self.get_logger().info("=== Restamp configuration ===")
        self.get_logger().info(f"topic_suffix='{self.topic_suffix}'")
        self.get_logger().info(f"restamp_topics={self.restamp_topics}")

        # -----------------------
        # Build pairs
        # -----------------------
        self.pairs: List[PubSubPair] = []

        for base in self.restamp_topics:
            self.pairs.append(
                RestampPubSubPair(
                    node=self,
                    base_topic_name=base,
                    sub_topic=base,
                    pub_topic=f"{base}{self.topic_suffix}",
                    sub_role=self.sub_role,
                    pub_role=self.pub_role,
                    qos_config=self.qos_config,
                )
            )

        self.init_pair_refresh(period_s=5.0, log_prefix="Restamp")

    def transform_message(self, msg: Any, *, topic: str) -> Any:
        self._dbg_seen[topic] = self._dbg_seen.get(topic, 0) + 1
        n = self._dbg_seen[topic]

        if n == 1:
            self.get_logger().info(
                f"[Restamp] topic='{topic}' type={type(msg)} "
                f"has_slots={hasattr(msg,'__slots__')} has_get_fields={hasattr(msg,'get_fields_and_field_types')}"
            )
            if hasattr(msg, "__slots__"):
                self.get_logger().info(f"[Restamp] topic='{topic}' __slots__={list(msg.__slots__)}")
            if hasattr(msg, "get_fields_and_field_types"):
                self.get_logger().info(
                    f"[Restamp] topic='{topic}' fields={list(msg.get_fields_and_field_types().keys())}"
                )
            self.get_logger().info(f"[Restamp] topic='{topic}' iter_fields={_iter_field_names(msg)}")

        collect = self.debug_samples_per_msg if n <= self.debug_first_n else 0
        new_stamp = self.get_clock().now().to_msg()

        _, changes, samples = update_timestamps_in_msg(
            msg,
            new_stamp=new_stamp,
            collect_samples=collect,
        )

        if n <= self.debug_first_n:
            self.get_logger().info(f"[Restamp] msg#{n} topic='{topic}' changes={changes} samples={samples}")

        if (
            self.debug_log_unchanged
            and n == self.debug_first_n + 1
            and changes == 0
            and topic not in self._dbg_warned
        ):
            self._dbg_warned.add(topic)
            self.get_logger().warn(
                f"[Restamp] topic='{topic}': no stamp changes observed. "
                f"Likely cause: this message type has no 'stamp' fields."
            )

        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Restamp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Restamp …")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()





