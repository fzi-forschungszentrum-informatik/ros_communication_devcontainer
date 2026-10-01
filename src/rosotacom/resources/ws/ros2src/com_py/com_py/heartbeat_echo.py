#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
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

"""Symmetric heartbeat echo and health monitor."""

from __future__ import annotations

import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64, String

from com_msgs.msg import EchoHeartbeat
from com_py.qos import get_topic_qos, load_qos_config
from com_py.status_overview_core import ClockOffsetEstimator, StageObservation


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


class HeartbeatEcho(Node):
    """Symmetric fixed-rate heartbeat with piggybacked NTP-style echo data."""

    def __init__(self) -> None:
        super().__init__("heartbeat_echo")
        self.local_topic = str(self.declare_parameter("local_topic", "").value)
        self.remote_topic = str(self.declare_parameter("remote_topic", "").value)
        self.qos_config_file = str(self.declare_parameter("qos_config_file", "").value)
        self.hz = float(self.declare_parameter("hz", 10.0).value)
        self.loss_bad_pct = float(self.declare_parameter("loss_bad_pct", 10.0).value)
        self.delay_bad_ms = float(self.declare_parameter("delay_bad_ms", 200.0).value)
        if not self.local_topic or not self.remote_topic:
            raise ValueError("local_topic and remote_topic are required")
        if self.hz <= 0.0:
            raise ValueError("hz must be > 0")

        qos_config = load_qos_config(self.get_logger(), self.qos_config_file)
        pub_qos = get_topic_qos(self.get_logger(), qos_config, self.local_topic, "heartbeat_pub")
        sub_qos = get_topic_qos(self.get_logger(), qos_config, self.remote_topic, "heartbeat_sub")
        self.publisher = self.create_publisher(EchoHeartbeat, self.local_topic, pub_qos)
        self.subscription = self.create_subscription(
            EchoHeartbeat, self.remote_topic, self._on_remote, sub_qos
        )

        metrics_qos = QoSProfile(depth=10)
        self.rtt_pub = self.create_publisher(Float64, self.remote_topic + "/rtt_ms", metrics_qos)
        self.offset_pub = self.create_publisher(
            Float64, self.remote_topic + "/clock_offset_ms", metrics_qos
        )
        self.delay_pub = self.create_publisher(
            Float64, self.remote_topic + "/delay_ms", metrics_qos
        )
        self.loss_pub = self.create_publisher(
            Float64, self.remote_topic + "/loss_pct", metrics_qos
        )
        self.status_pub = self.create_publisher(
            String, self.remote_topic + "/status", metrics_qos
        )
        self.summary_pub = self.create_publisher(
            String, self.remote_topic + "/summary", metrics_qos
        )

        self.seq = 0
        self.pending_echo: Optional[Tuple[int, object, object]] = None
        self.observation = StageObservation(type_str="com_msgs/msg/EchoHeartbeat")
        self.clock_estimator = ClockOffsetEstimator()
        self.create_timer(1.0 / self.hz, self._publish)
        self.create_timer(0.2, self._publish_diagnostics)
        self.get_logger().info(
            f"heartbeat echo: {self.local_topic} -> {self.remote_topic} at {self.hz:.1f} Hz"
        )

    @staticmethod
    def _publish_float(publisher, value: float) -> None:
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

    @staticmethod
    def _publish_string(publisher, value: str) -> None:
        msg = String()
        msg.data = value
        publisher.publish(msg)

    def _publish(self) -> None:
        now = self.get_clock().now().to_msg()
        msg = EchoHeartbeat()
        msg.header.stamp = now
        msg.header.frame_id = "rosotacom_echo"
        msg.seq = self.seq
        if self.pending_echo is not None:
            echo_seq, echo_t1, echo_t2 = self.pending_echo
            msg.echo_valid = True
            msg.echo_seq = echo_seq
            msg.echo_t1 = echo_t1
            msg.echo_t2 = echo_t2
            msg.echo_t3 = now
        self.publisher.publish(msg)
        self.seq += 1

    def _on_remote(self, msg: EchoHeartbeat) -> None:
        now = self.get_clock().now()
        now_s = now.nanoseconds / 1e9
        self.pending_echo = (int(msg.seq), msg.header.stamp, now.to_msg())

        if msg.echo_valid:
            self.clock_estimator.update(
                t1_s=_stamp_seconds(msg.echo_t1),
                t2_s=_stamp_seconds(msg.echo_t2),
                t3_s=_stamp_seconds(msg.echo_t3),
                t4_s=now_s,
                sample_id=int(msg.echo_seq),
            )
        estimate = self.clock_estimator.estimate()
        delay_s = None
        rtt_s = None
        offset_s = None
        if estimate is not None:
            rtt_s = estimate["rtt_s"]
            offset_s = estimate["offset_s"]
            delay_s = now_s + offset_s - _stamp_seconds(msg.header.stamp)
            if delay_s < 0.0:
                delay_s = None
        self.observation.record(
            0,
            delay_s,
            seq=int(msg.seq),
            rtt_s=rtt_s,
            clock_offset_s=offset_s,
        )

    def _publish_diagnostics(self) -> None:
        now_mono = time.monotonic()
        metrics = self.observation.metrics(now_mono, 3.0)
        estimate = self.clock_estimator.estimate(now_mono)
        if estimate is not None:
            self._publish_float(self.rtt_pub, estimate["rtt_s"] * 1000.0)
            self._publish_float(self.offset_pub, estimate["offset_s"] * 1000.0)
        if metrics["last_delay_s"] is not None:
            self._publish_float(self.delay_pub, metrics["last_delay_s"] * 1000.0)
        loss_pct = float(metrics["loss_pct"] or 0.0)
        self._publish_float(self.loss_pub, loss_pct)

        age_s = metrics["age_s"]
        delay_ms = (
            metrics["last_delay_s"] * 1000.0
            if metrics["last_delay_s"] is not None
            else None
        )
        if age_s is None or age_s >= max(1.0, 5.0 / self.hz):
            status, reason = "LOST", "age"
        elif loss_pct > self.loss_bad_pct:
            status, reason = "BAD", "loss"
        elif delay_ms is not None and delay_ms > self.delay_bad_ms:
            status, reason = "BAD", "delay"
        elif age_s >= max(0.5, 2.5 / self.hz):
            status, reason = "DEGRADED", "age"
        elif loss_pct > self.loss_bad_pct / 2.0:
            status, reason = "DEGRADED", "loss"
        elif delay_ms is not None and delay_ms > self.delay_bad_ms / 2.0:
            status, reason = "DEGRADED", "delay"
        else:
            status, reason = "GOOD", "-"
        self._publish_string(self.status_pub, status)
        self._publish_string(
            self.summary_pub,
            (
                f"status={status} reason={reason} hz={metrics['hz']:.2f} "
                f"loss={loss_pct:.2f}% reordered={metrics['reordered'] or 0} "
                f"rtt_ms={(estimate['rtt_s'] * 1000.0) if estimate else float('nan'):.2f} "
                "peer_offset_ms="
                f"{(estimate['offset_s'] * 1000.0) if estimate else float('nan'):.3f}"
            ),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeartbeatEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
