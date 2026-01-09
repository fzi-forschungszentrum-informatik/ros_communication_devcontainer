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
# \date    2024-12-17
#
# ---------------------------------------------------------------------

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String

from com_msgs.msg import Heartbeat
from com_py.qos import load_qos_config, get_topic_qos

from rclpy.qos import QoSProfile

class HeartbeatInMonitor(Node):
    """
    Subscribes to a Heartbeat topic and:
      - prints delay to terminal (similar to `ros2 topic delay`)
      - publishes delay in ms
      - publishes binary connectivity status (based on timeout threshold)
      - publishes reordering / loss indication via the heartbeat sequence number
    """

    def __init__(self) -> None:
        super().__init__("heartbeat_in_monitor")

        # Parameters
        self.declare_parameter("heartbeat_topic", "/heartbeat")
        self.declare_parameter("qos_config_file", "")
        self.declare_parameter("hz_window_s", 1.0)

        self.heartbeat_topic = self.get_parameter("heartbeat_topic").value
        self.qos_config_file = self.get_parameter("qos_config_file").value
        self.hz_window_s = float(self.get_parameter("hz_window_s").value)
        self.sub_role = "heartbeat_sub"
        self.pub_role = "heartbeat_pub"

        # Load QoS configuration
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # Build QoS profiles from config
        sub_qos = get_topic_qos(
            self.get_logger(), self.qos_config, self.heartbeat_topic, self.sub_role
        )

        # Topics for published diagnostics
        self.delay_topic = self.heartbeat_topic + "/delay_readable"
        self.hz_topic = self.heartbeat_topic + "/hz_readable"
        self.delay_pub = self.create_publisher(
            String, self.delay_topic, qos_profile=QoSProfile(depth=10)
        )
        self.hz_pub = self.create_publisher(
            String, self.hz_topic, qos_profile=QoSProfile(depth=10)
        )

        # Subscriber
        self.subscription = self.create_subscription(
            Heartbeat,
            self.heartbeat_topic,
            self.heartbeat_callback,
            qos_profile=sub_qos,
        )

        # State
        self.last_heartbeat_time: Time | None = None
        self.last_seq: int | None = None
        self.expected_next_seq: int | None = None

        # For Hz computation over a sliding time window (similar to hz_monitor)
        self.msg_count: int = 0
        self.hz_last_time: Time = self.get_clock().now()

        # Timer to periodically log Hz
        self.timer = self.create_timer(self.hz_window_s, self.hz_timer_callback)

        self.get_logger().info(
            f"HeartbeatSubscriber initialized. Listening on '{self.heartbeat_topic}', "
            f"Hz window={self.hz_window_s:.2f}s"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def heartbeat_callback(self, msg: Heartbeat) -> None:
        now = self.get_clock().now()
        src_time = Time.from_msg(msg.header.stamp)
        delay_ms = (now - src_time).nanoseconds / 1e6

        # Compute sequence behaviour: expected_next_seq vs actual
        seq = int(msg.seq)
        seq_delta = 0
        if self.expected_next_seq is None:
            # First message: just initialize sequence tracking
            self.expected_next_seq = seq + 1
        else:
            seq_delta = seq - self.expected_next_seq
            self.expected_next_seq = seq + 1

        # Logging to terminal
        self.get_logger().debug(f"Heartbeat received: seq={seq}, delay={delay_ms:.2f} ms")
        if seq_delta > 0:
            # Gap: some IDs were not seen
            missing_start = self.expected_next_seq - (seq_delta + 1)
            missing_end = seq - 1
            if missing_start == missing_end:
                self.get_logger().warning(
                    f"Heartbeat gap detected: received id {seq}, but id {missing_start} is missing"
                )
            else:
                num_missing = missing_end - missing_start + 1
                self.get_logger().warning(
                    f"Heartbeat gap detected: received id {seq}, but {num_missing} ids ({missing_start}-{missing_end}) are missing"
                )
        elif seq_delta < 0 and self.last_seq is not None:
            # Out-of-order / late arrival
            self.get_logger().warning(
                f"Out-of-order heartbeat: id {seq} arrived after already receiving id {self.last_seq}"
            )

        # Publish delay in ms
        delay_msg = String()
        delay_msg.data = f"Incoming Heartbeat latency {int(delay_ms)} ms"
        self.delay_pub.publish(delay_msg)

        # Update state for Hz and anomaly calculations
        self.last_heartbeat_time = now
        self.last_seq = seq
        self.msg_count += 1

    def hz_timer_callback(self) -> None:
        """
        Periodic timer to compute and publish the observed heartbeat Hz
        over the last `hz_window_s` seconds.
        """
        now = self.get_clock().now()
        elapsed_ns = (now - self.hz_last_time).nanoseconds
        dt = elapsed_ns / 1e9 if elapsed_ns > 0 else 0.0

        hz = 0.0
        if dt > 0.0:
            hz = self.msg_count / dt

        # Publish Hz (0.0 if no messages in window)
        hz_msg = String()
        hz_msg.data = f"Incoming Heartbeat frequency: {int(hz)} Hz"
        self.hz_pub.publish(hz_msg)

        self.hz_last_time = now
        self.msg_count = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HeartbeatInMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down HeartbeatSubscriber …")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

