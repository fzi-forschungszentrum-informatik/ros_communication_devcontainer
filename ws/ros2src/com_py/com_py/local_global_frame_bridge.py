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
Local↔Global frame bridge node.

Use-case semantics:
- **Local side**: messages use *un-prefixed* frame IDs (e.g. "map", "base_link").
- **Global side**: messages use a *global frame prefix* so multiple robots / local
  namespaces can coexist without frame-id collisions (e.g. "robot1_map").

This node maps between these two viewpoints by rewriting occurrences of:
- header.frame_id
- frame_id
- child_frame_id

anywhere inside a ROS2 Python message (including nested messages and arrays).

Topic mapping model:
- local → global: subscribe base topic, publish base + global_topic_suffix
- global → local: subscribe base + global_topic_suffix, publish base topic
"""

from typing import Any, List

import rclpy
from rclpy.node import Node

from com_py.msg_transform import _iter_field_names, remap_frames_in_msg
from com_py.pair_management import PairRefreshMixin
from com_py.pub_sub_pair import PubSubPair
from com_py.qos import load_qos_config


# ======================================================================
#  PubSubPair specialization
# ======================================================================


class LocalGlobalFrameBridgePubSubPair(PubSubPair):
    def __init__(self, *, node, map_direction: str, **kwargs):
        super().__init__(node=node, **kwargs)
        self._map_direction = map_direction

    def _callback(self, msg):
        if self.first_msg:
            self.logger.info(
                "[LocalGlobalFrameBridgePubSubPair] "
                f"FIRST msg on sub='{self.sub_topic}' => pub='{self.pub_topic}'"
            )
            self.first_msg = False

        out = self.node.transform_message(msg, self._map_direction, topic=self.sub_topic)
        self.publisher.publish(out)


# ======================================================================
#  Node
# ======================================================================


class LocalGlobalFrameBridge(Node, PairRefreshMixin):
    def __init__(self, *, node_name: str = "local_global_frame_bridge") -> None:
        super().__init__(node_name)

        # -----------------------
        # Parameters
        # -----------------------
        def declare(name: str, default):
            p = self.declare_parameter(name, default)
            return p.value

        self.global_frame_prefix = declare("global_frame_prefix", "")
        self.global_topic_suffix = declare("global_topic_suffix", "/globalframe")
        self.local_to_global_topics = (
            declare("local_to_global_topics", rclpy.Parameter.Type.STRING_ARRAY) or []
        )
        self.global_to_local_topics = (
            declare("global_to_local_topics", rclpy.Parameter.Type.STRING_ARRAY) or []
        )

        self.exclude_frames = (
            declare("exclude_frames", rclpy.Parameter.Type.STRING_ARRAY) or []
        )
        self.qos_config_file = declare("qos_config_file", "")
        self.sub_role = "framebridge_sub"
        self.pub_role = "framebridge_pub"

        # Debug params
        self.debug_first_n = int(declare("debug_first_n", 3))
        self.debug_samples_per_msg = int(declare("debug_samples_per_msg", 5))
        self.debug_log_unchanged = bool(declare("debug_log_unchanged", True))

        self._dbg_seen: dict[tuple[str, str], int] = {}
        self._dbg_warned: set[tuple[str, str]] = set()

        # QoS
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # Prefix token used on the **global** side.
        self._global_prefix_token = (
            self.global_frame_prefix.strip()
            if isinstance(self.global_frame_prefix, str)
            else str(self.global_frame_prefix)
        )
        if self._global_prefix_token and not self._global_prefix_token.endswith("_"):
            self._global_prefix_token += "_"

        # -----------------------
        # Startup logging
        # -----------------------
        self.get_logger().info("=== LocalGlobalFrameBridge configuration ===")
        self.get_logger().info(
            f"global_prefix_token='{self._global_prefix_token}' (raw='{self.global_frame_prefix}')"
        )
        self.get_logger().info(f"global_topic_suffix='{self.global_topic_suffix}'")
        self.get_logger().info(f"exclude_frames={self.exclude_frames}")
        self.get_logger().info(
            f"local_to_global_topics={self.local_to_global_topics}"
        )
        self.get_logger().info(
            f"global_to_local_topics={self.global_to_local_topics}"
        )

        if not self._global_prefix_token:
            self.get_logger().warn(
                "global_frame_prefix is empty -> global-side frame IDs will NOT be prefixed"
            )

        # -----------------------
        # Build pairs
        # -----------------------
        self.pairs: List[PubSubPair] = []

        # local -> global: publish on suffixed topic and prefix frame IDs
        for base in self.local_to_global_topics:
            self.pairs.append(
                LocalGlobalFrameBridgePubSubPair(
                    node=self,
                    base_topic_name=base,
                    sub_topic=base,
                    pub_topic=f"{base}{self.global_topic_suffix}",
                    sub_role=self.sub_role,
                    pub_role=self.pub_role,
                    qos_config=self.qos_config,
                    map_direction="local_to_global",
                )
            )

        # global -> local: subscribe suffixed topic and un-prefix frame IDs
        for base in self.global_to_local_topics:
            self.pairs.append(
                LocalGlobalFrameBridgePubSubPair(
                    node=self,
                    base_topic_name=base,
                    sub_topic=f"{base}{self.global_topic_suffix}",
                    pub_topic=base,
                    sub_role=self.sub_role,
                    pub_role=self.pub_role,
                    qos_config=self.qos_config,
                    map_direction="global_to_local",
                )
            )

        self.init_pair_refresh(period_s=5.0, log_prefix="LocalGlobalFrameBridge")

    # ------------------------------------------------------------------
    # Transform + debug
    # ------------------------------------------------------------------

    def transform_message(self, msg: Any, map_direction: str, *, topic: str) -> Any:
        key = (topic, map_direction)
        self._dbg_seen[key] = self._dbg_seen.get(key, 0) + 1
        n = self._dbg_seen[key]

        # Hard debug: prove message structure
        if n == 1:
            self.get_logger().info(
                f"[LocalGlobalFrameBridge] topic='{topic}' type={type(msg)} "
                f"has_slots={hasattr(msg,'__slots__')} "
                f"has_get_fields={hasattr(msg,'get_fields_and_field_types')}"
            )
            if hasattr(msg, "__slots__"):
                self.get_logger().info(
                    f"[LocalGlobalFrameBridge] topic='{topic}' __slots__={list(msg.__slots__)}"
                )
            if hasattr(msg, "get_fields_and_field_types"):
                self.get_logger().info(
                    f"[LocalGlobalFrameBridge] topic='{topic}' "
                    f"fields={list(msg.get_fields_and_field_types().keys())}"
                )
            self.get_logger().info(
                f"[LocalGlobalFrameBridge] topic='{topic}' iter_fields={_iter_field_names(msg)}"
            )

        collect = self.debug_samples_per_msg if n <= self.debug_first_n else 0

        # Bridge-level mapping names → transform-level direction names.
        # msg_transform.remap_frames_in_msg() operates on 'add'/'remove'.
        if map_direction == "local_to_global":
            remap_direction = "add"
        elif map_direction == "global_to_local":
            remap_direction = "remove"
        else:
            # Allow passing 'add'/'remove' directly for compatibility.
            remap_direction = map_direction

        _, changes, samples = remap_frames_in_msg(
            msg,
            direction=remap_direction,
            prefix_token=self._global_prefix_token,
            exclude_frames=set(self.exclude_frames),
            collect_samples=collect,
        )

        if n <= self.debug_first_n:
            self.get_logger().info(
                f"[LocalGlobalFrameBridge] msg#{n} topic='{topic}' dir='{map_direction}' "
                f"changes={changes} samples={samples}"
            )

        if (
            self.debug_log_unchanged
            and n == self.debug_first_n + 1
            and changes == 0
            and key not in self._dbg_warned
        ):
            self._dbg_warned.add(key)
            self.get_logger().warn(
                f"[LocalGlobalFrameBridge] topic='{topic}' dir='{map_direction}': "
                "no frame-id changes observed. Likely causes: frames already mapped or excluded."
            )

        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalGlobalFrameBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LocalGlobalFrameBridge …")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


