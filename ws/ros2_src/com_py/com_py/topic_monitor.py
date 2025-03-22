#!/usr/bin/env python3
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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import datetime
import re
from rosidl_runtime_py.utilities import get_message

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
        # Declare parameters for timer intervals
        self.declare_parameter('refresh_interval', 10.0)  # seconds
        self.declare_parameter('print_interval', 5.0)   # seconds

        self.refresh_interval = self.get_parameter('refresh_interval').value
        self.print_interval = self.get_parameter('print_interval').value

        # Dictionary for accumulating topic stats: topic_name -> TopicStats
        self.topics_stats = {}

        # Timers to refresh subscriptions and print stats
        self.refresh_timer = self.create_timer(self.refresh_interval, self.refresh_subscriptions)
        self.print_timer = self.create_timer(self.print_interval, self.print_stats)

        self.warned_topics = set()  # Track topics that triggered a warning

        self.get_logger().info("topic_monitor node started. Searching for topics with /com/out/ or /com/in/ prefixes...")

    def refresh_subscriptions(self):
        all_topics = self.get_topic_names_and_types()
        topic_map = {tname: ttypes[0] for tname, ttypes in all_topics if ttypes}

        filtered_topics = [t for t in topic_map.keys() if t.startswith("/com/out/") or t.startswith("/com/in/")]
        self.get_logger().info(f"Found {len(filtered_topics)} topics with /com/out/ or /com/in/ prefixes.")

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
                    qos_profile
                )

                self.topics_stats[topic_name] = TopicStats()
                self.get_logger().info(f"Subscribed to {topic_name} with type {msg_type_str}")

    def make_callback(self, topic_name: str):
        """Return a callback function that dynamically handles any message type."""
        def callback(msg):
            msg_size = 0

            # Handling different data types
            if hasattr(msg, 'data'):
                if isinstance(msg.data, bool):
                    msg_size = 1  # Booleans are typically 1 byte
                elif isinstance(msg.data, (int, float)):
                    msg_size = msg.data.__sizeof__()
                elif isinstance(msg.data, (str, bytes, list, tuple)):
                    msg_size = len(msg.data)
                else:
                    msg_size = msg.data.__sizeof__()
            elif isinstance(msg, bool):
                msg_size = 1
            elif isinstance(msg, (int, float)):
                msg_size = msg.__sizeof__()
            else:
                # Prevent spam: Only warn once per topic
                if topic_name not in self.warned_topics:
                    self.get_logger().warn(f"Cannot determine size for message on {topic_name}, using default 0 bytes.")
                    self.warned_topics.add(topic_name)  # Mark as warned
                msg_size = 0

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
                self.get_logger().warn(f"No stats entry for {topic_name}, message ignored!")

        return callback


    def print_stats(self):
        window_duration = self.print_interval
        details = []
        for t_name, stats in self.topics_stats.items():
            (bandwidth, freq, delay) = stats.compute_results(window_duration)
            details.append((t_name, bandwidth, freq, delay))
            # Reset stats for next print interval
            self.topics_stats[t_name] = TopicStats()

        details.sort(key=lambda d: d[1] if isinstance(d[1], float) else 0, reverse=True)
        total_bw = sum(d[1] for d in details if isinstance(d[1], float))

        self.get_logger().info(f"{'Topic'.ljust(87)} | {'Bandwidth'.rjust(14)} | {'Frequency'.rjust(9)} | {'Delay'.rjust(8)}")
        self.get_logger().info('-' * 155)
        for d in details:
            topic_str = d[0].ljust(87)
            bw_str = f"{d[1]:7.1f} Kbit/s" if isinstance(d[1], float) else str(d[1])
            freq_str = f"{d[2]:6.1f} Hz" if isinstance(d[2], float) else str(d[2])
            delay_str = f"{d[3]:6.3f} s" if isinstance(d[3], float) else str(d[3])
            self.get_logger().info(f"{topic_str} | {bw_str:>14} | {freq_str:>9} | {delay_str:>8}")
        self.get_logger().info(f"Total Bandwidth: {total_bw:7.1f} Kbit/s\n")

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting the program on KeyboardInterrupt...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
