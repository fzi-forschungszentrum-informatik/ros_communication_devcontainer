#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2024-11-13
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

#
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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import serialize_message
from std_msgs.msg import Float64

from com_py.link_bytes import LinkByteSampler, resolve_link_interface


def is_internal_transport_topic(topic_name: str) -> bool:
    parts = topic_name.strip("/").split("/")
    return any(part.startswith("_buf_") for part in parts)


class TopicStats:
    """Accumulate stats for one topic over the sampling window."""
    def __init__(self):
        self.message_count = 0
        self.byte_count = 0
        self.delay_accumulator = 0.0
        self.delay_count = 0
        self.start_time = time.time()

    def record_message(self, msg_size: int, delay: float = None):
        self.message_count += 1
        self.byte_count += msg_size
        if isinstance(delay, (int, float)):  # Ensure it's a number before adding
            self.delay_accumulator += delay
            self.delay_count += 1

    def compute_results(self, window_duration: float):
        """Return (bandwidth_kbit/s, frequency_hz, avg_delay_s or 'No header')."""
        if window_duration <= 0:
            return (0.0, 0.0, "-")
        bandwidth_kbit_s = (self.byte_count * 8 / 1024.0) / window_duration
        freq = self.message_count / window_duration
        avg_delay = self.delay_accumulator / self.delay_count if self.delay_count > 0 else "No header"
        return (bandwidth_kbit_s, freq, avg_delay)


class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')

        # Callback groups so print_stats waiting does not starve subscriptions
        self.cb_subs = ReentrantCallbackGroup()
        self.cb_timers = ReentrantCallbackGroup()

        # Declare parameters for timer intervals
        self.declare_parameter('refresh_interval', 10.0)  # seconds
        self.declare_parameter('print_interval', 5.0)   # seconds

        # Declare parameters for topic filtering
        self.declare_parameter('sourcename', '')  # e.g., 'shuttle_ella' or 'control_center'
        self.declare_parameter('direction', '')  # 'in' or 'out'
        self.declare_parameter('to_adressant', '')  # optional, e.g., 'shuttle_ella' for /to_shuttle_ella
        self.declare_parameter("ip_local", "")
        self.declare_parameter("ip_remote", "")
        # Explicit override for the OTA link interface. Normally left empty:
        # it is resolved from ip_local below.
        self.declare_parameter("interface", "")
        # Accepted for launch compatibility; the /proc/net/dev sampler needs no
        # capture window, so this tshark-era buffer is no longer used.
        self.declare_parameter("link_duration_buffer_s", 1.0)

        self.refresh_interval = self.get_parameter('refresh_interval').value
        self.print_interval = self.get_parameter('print_interval').value
        self.sourcename = self.get_parameter('sourcename').value
        self.direction = self.get_parameter('direction').value
        self.to_adressant = self.get_parameter('to_adressant').value
        self.host_ip = str(self.get_parameter("ip_local").value or "").strip()
        self.peer_ip = str(self.get_parameter("ip_remote").value or "").strip()
        self.interface = str(self.get_parameter("interface").value or "")

        # Track actual elapsed time between prints (avoid timer drift issues)
        self._last_print_t = time.monotonic()

        # Validate parameters
        if not self.sourcename:
            print("ERROR: Parameter 'sourcename' is required!")
            raise ValueError("Parameter 'sourcename' is required!")

        if self.direction not in ['in', 'out']:
            print(f"ERROR: Parameter 'direction' must be 'in' or 'out', got '{self.direction}'")
            raise ValueError(f"Parameter 'direction' must be 'in' or 'out'")

        # Build topic prefix pattern
        if self.to_adressant:
            self.topic_prefix = f"/com/{self.direction}/{self.sourcename}/to_{self.to_adressant}/"
        else:
            self.topic_prefix = f"/com/{self.direction}/{self.sourcename}/"

        # The OTA link interface. `ip_local` is the peer's own OTA address, so
        # the interface follows from it -- the same resolution status_overview
        # does, in the same place, from the same helper. `interface` stays an
        # explicit override for a setup the address cannot describe.
        #
        # It used to be resolved in the session template instead, by a shell
        # line that expanded a catmux parameter with bash's substring-replace
        # syntax. catmux substitutes `${name}` and nothing else, so the pattern
        # stayed literal, bash expanded an unset variable to nothing, and the
        # awk program collapsed to "first address line with a prefix length" --
        # `lo`, on every machine. See docs/link-bandwidth.md.
        try:
            self.interface, provenance = resolve_link_interface(self.interface, self.host_ip)
        except ValueError as exc:
            raise ValueError(f"topic_monitor: {exc}") from exc
        self.get_logger().info(f"OTA link interface '{self.interface}' ({provenance})")

        # Link bandwidth is measured from the kernel's own interface byte counters
        # (/proc/net/dev) via the shared link_bytes sampler -- no tshark, no sudo,
        # and no per-peer IP filter needed (the OTA interface is a dedicated link).
        self._link_sampler = LinkByteSampler(self.interface)

        # Publisher for link bandwidth (direction-specific)
        self.link_bandwidth_topic = f"/topic_monitor/{self.direction}/{self.sourcename}/link_bandwidth_kbps"
        if self.to_adressant:
            self.link_bandwidth_topic = f"/topic_monitor/{self.direction}/{self.sourcename}/to_{self.to_adressant}/link_bandwidth_kbps"
        self.link_bandwidth_pub = self.create_publisher(
            Float64, self.link_bandwidth_topic, QoSProfile(depth=10)
        )

        # Dictionary for accumulating topic stats: topic_name -> TopicStats
        self.topics_stats = {}

        # Timers to refresh subscriptions and print stats
        self.refresh_timer = self.create_timer(self.refresh_interval, self.refresh_subscriptions, callback_group=self.cb_timers)
        self.print_timer = self.create_timer(self.print_interval, self.print_stats, callback_group=self.cb_timers)

        # Publisher for total ROS topic bandwidth
        self.ros_topic_bandwidth_topic = f"/topic_monitor/{self.direction}/{self.sourcename}/ros_topic_bandwidth_kbps"
        if self.to_adressant:
            self.ros_topic_bandwidth_topic = f"/topic_monitor/{self.direction}/{self.sourcename}/to_{self.to_adressant}/ros_topic_bandwidth_kbps"
        self.ros_topic_bandwidth_pub = self.create_publisher(
            Float64, self.ros_topic_bandwidth_topic, qos_profile=QoSProfile(depth=10)
        )

        self.get_logger().info(f"topic_monitor node started. Searching for topics with prefix: {self.topic_prefix}")
        self.get_logger().info(
            f"Publishing ROS topic bandwidth to: {self.ros_topic_bandwidth_topic} "
            f"(serialized payload of {self.topic_prefix}*, Kibit/s)"
        )
        self.get_logger().info(
            f"Publishing link bandwidth to: {self.link_bandwidth_topic} "
            f"(wire bytes on '{self.interface}', {'tx' if self.direction == 'out' else 'rx'}, Kibit/s)"
        )

        # Prime the link sampler so the first print reports a real delta.
        self._link_sampler.sample()

    def refresh_subscriptions(self):
        all_topics = self.get_topic_names_and_types()
        topic_map = {tname: ttypes[0] for tname, ttypes in all_topics if ttypes}

        filtered_topics = [
            t for t in topic_map.keys()
            if t.startswith(self.topic_prefix)
            and not is_internal_transport_topic(t)
        ]

        # Shorten topic names for logging (remove redundant prefix)
        shortened_topics = [t[len(self.topic_prefix):] if t.startswith(self.topic_prefix) else t for t in filtered_topics]
        shortened_list = ", ".join(f"../{st}" for st in shortened_topics[:10])  # Show first 10 with ../ prefix
        if len(shortened_topics) > 10:
            shortened_list += f", ... ({len(shortened_topics) - 10} more)"

        self.get_logger().info(f"found {len(filtered_topics)} topics with {self.topic_prefix} prefixes")

        for topic_name in filtered_topics:
            if topic_name not in self.topics_stats:
                msg_type_str = topic_map[topic_name]  # Get detected type
                msg_class = get_message(msg_type_str)

                if msg_class is None:
                    self.get_logger().error(f"Could not determine message type for {topic_name}, skipping...")
                    continue

                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )

                self.create_subscription(
                    msg_class,
                    topic_name,
                    self.make_callback(topic_name),
                    qos_profile,
                    callback_group=self.cb_subs,
                )

                self.topics_stats[topic_name] = TopicStats()
                self.get_logger().info(f"Subscribed to {topic_name} with type {msg_type_str}")

    def make_callback(self, topic_name: str):
        """Return a callback function that dynamically handles any message type."""
        def callback(msg):
            msg_size = len(serialize_message(msg))

            # Extract timestamp if the message has a header
            msg_delay = None
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                current_time = self.get_clock().now()
                msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
                msg_delay = (current_time - msg_time).nanoseconds / 1e9  # Convert to seconds

                # If delay exceeds 1000s, set a descriptive label
                if msg_delay > 1000:
                    msg_delay = "Clocks unsynced"

            # Logging and stats recording
            stats = self.topics_stats.get(topic_name)
            self.get_logger().debug(f"Received message on {topic_name}, size={msg_size} bytes, delay={msg_delay}")

            if stats:
                stats.record_message(msg_size, msg_delay)
            else:
                self.get_logger().warning(f"No stats entry for {topic_name}, message ignored!")

        return callback

    def print_stats(self):
        # Use actual elapsed time to compute Hz/bandwidth correctly even if timers slip
        now = time.monotonic()
        window_duration = max(1e-9, now - self._last_print_t)
        self._last_print_t = now

        details = []
        for t_name, stats in self.topics_stats.items():
            (bandwidth, freq, delay) = stats.compute_results(window_duration)
            # Shorten topic name for display
            short_name = t_name[len(self.topic_prefix):] if t_name.startswith(self.topic_prefix) else t_name
            details.append((short_name, bandwidth, freq, delay))
            # Reset stats for next print interval
            self.topics_stats[t_name] = TopicStats()

        details.sort(key=lambda d: d[1] if isinstance(d[1], float) else 0, reverse=True)
        ros_topic_bw = sum(d[1] for d in details if isinstance(d[1], float))

        # Pre-format cells and compute dynamic column widths (no hardcoded widths)
        rows = []
        for (topic, bw, freq, delay) in details:
            bw_str = f"{bw:7.1f} Kbit/s" if isinstance(bw, float) else str(bw)
            freq_str = f"{freq:6.1f} Hz" if isinstance(freq, float) else str(freq)
            delay_str = f"{delay:6.3f} s" if isinstance(delay, float) else str(delay)
            rows.append((topic, bw_str, freq_str, delay_str, bw))

        topic_w = max(len("Topic"), *(len(r[0]) for r in rows)) if rows else len("Topic")
        bw_w = max(len("Bandwidth"), *(len(r[1]) for r in rows)) if rows else len("Bandwidth")
        freq_w = max(len("Frequency"), *(len(r[2]) for r in rows)) if rows else len("Frequency")
        delay_w = max(len("Delay"), *(len(r[3]) for r in rows)) if rows else len("Delay")

        self.get_logger().info("")
        self.get_logger().info(f"Overview for the {len(details)} found topics with the prefix {self.topic_prefix}:")
        header = (
            f"{'Topic'.ljust(topic_w)} | "
            f"{'Bandwidth'.rjust(bw_w)} | "
            f"{'Frequency'.rjust(freq_w)} | "
            f"{'Delay'.rjust(delay_w)}"
        )
        print(header)
        print("-" * len(header))
        for (topic, bw_str, freq_str, delay_str, _bw_raw) in rows:
            print(
                f"{topic:<{topic_w}} | "
                f"{bw_str:>{bw_w}} | "
                f"{freq_str:>{freq_w}} | "
                f"{delay_str:>{delay_w}}"
            )
        self.get_logger().info(f"ROS Topic Bandwidth: {ros_topic_bw:7.1f} Kbit/s")

        # Publish total ROS topic bandwidth as a topic
        bandwidth_msg = Float64()
        bandwidth_msg.data = float(ros_topic_bw)
        self.ros_topic_bandwidth_pub.publish(bandwidth_msg)

        # Link bandwidth over this print interval, from the interface counters.
        # direction 'out' -> tx (host -> peer), 'in' -> rx (peer -> host).
        sample = self._link_sampler.sample()
        if sample is not None:
            link_kbps = sample["tx_kbps"] if self.direction == "out" else sample["rx_kbps"]
            self.get_logger().info(f"Link Bandwidth: {link_kbps:7.1f} Kbit/s")
            m = Float64()
            m.data = float(link_kbps)
            self.link_bandwidth_pub.publish(m)

        self.get_logger().info("")


def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Exiting the program on KeyboardInterrupt...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
