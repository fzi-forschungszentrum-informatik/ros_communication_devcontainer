#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-02-25
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

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
#    * Neither the name of the copyright holder nor the names of its
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
# \date    2026-02-25
#
# ---------------------------------------------------------------------

from __future__ import annotations

"""
Trickle node ("trickle").

Subscribes to topics and periodically republishes the last received message at
a fixed rate, regardless of whether new data arrived.  This is a local-only
post-processing step intended to convert infrequently published topics (e.g.
latched / on-change topics) into a steady stream for visualization software
that expects periodic updates (state diagrams, etc.).

The trickle output is published to ``{sub_topic}{suffix}`` (e.g.
``/can/is_autonomous/latched/trickle``).  Because the trickle topics are
not in the OTA topic lists, they are never sent over the air.

Typical usage in a session template::

    processing:
      latch: true
      trickle_hz: 1        # republish locally at 1 Hz for visualization
    qos:
      reliability: reliable
      durability: transient_local
      for_role:
        latch_sub:
          durability: volatile
"""

from typing import List

import rclpy
from rclpy.node import Node

from com_py.pair_management import PairRefreshMixin
from com_py.pub_sub_pair import PubSubPair
from com_py.qos import load_qos_config


class TricklePubSubPair(PubSubPair):
    """PubSubPair that caches the last received message and lets a node-level
    timer republish it periodically."""

    def __init__(self, *, node, **kwargs):
        self._last_msg = None
        super().__init__(node=node, **kwargs)

    def _callback(self, msg):
        self._last_msg = msg
        if self.first_msg:
            self.logger.info(
                f"[TricklePubSubPair] FIRST msg on sub='{self.sub_topic}' => pub='{self.pub_topic}'"
            )
            self.first_msg = False

        # Forward immediately on every incoming message so a status change is
        # visible downstream without waiting for the next periodic tick. The
        # timer-driven tick() continues to republish the cached state in
        # between, providing the steady stream that visualisation consumers
        # expect.
        if self.is_valid and self.publisher is not None:
            self.publisher.publish(msg)

    def tick(self):
        """Called by the node timer.  Republish the cached message (if any)."""
        if self._last_msg is not None and self.is_valid:
            self.publisher.publish(self._last_msg)


class Trickle(Node, PairRefreshMixin):
    """Node that manages multiple TricklePubSubPair instances and a shared
    periodic timer that triggers republishing."""

    def __init__(self) -> None:
        super().__init__("trickle")

        # -----------------------
        # Parameters
        # -----------------------
        self.topic_suffix = self.declare_parameter("topic_suffix", "/trickle").value
        self.qos_config_file = self.declare_parameter("qos_config_file", "").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 1.0).value)
        self.sub_role = "trickle_sub"
        self.pub_role = "trickle_pub"

        self.trickle_topics: list[str] = (
            self.declare_parameter("trickle_topics", rclpy.Parameter.Type.STRING_ARRAY).value
            or []
        )

        # QoS
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # -----------------------
        # Startup logging
        # -----------------------
        self.get_logger().info("=== Trickle configuration ===")
        self.get_logger().info(f"topic_suffix='{self.topic_suffix}'")
        self.get_logger().info(f"rate_hz={self.rate_hz}")
        self.get_logger().info(f"trickle_topics={self.trickle_topics}")

        # -----------------------
        # Build pairs
        # -----------------------
        self.pairs: List[PubSubPair] = []

        for base in self.trickle_topics:
            self.pairs.append(
                TricklePubSubPair(
                    node=self,
                    base_topic_name=base,
                    sub_topic=base,
                    pub_topic=f"{base}{self.topic_suffix}",
                    sub_role=self.sub_role,
                    pub_role=self.pub_role,
                    qos_config=self.qos_config,
                )
            )

        # Periodic republish timer
        period_s = 1.0 / self.rate_hz if self.rate_hz > 0 else 1.0
        self._trickle_timer = self.create_timer(period_s, self._tick)

        self.init_pair_refresh(period_s=5.0, log_prefix="Trickle")

    def _tick(self):
        """Republish the cached message for every pair."""
        for pair in self.pairs:
            if isinstance(pair, TricklePubSubPair):
                pair.tick()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Trickle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Trickle …")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
