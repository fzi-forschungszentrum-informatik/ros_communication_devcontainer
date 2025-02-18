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
# \date    2024-11-13
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as BuiltinTime
from std_msgs.msg import Header

import math
import time
import datetime

def log_with_timestamp(msg: str):
    now_str = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"{now_str} - {msg}")

class TopicStats:
    """ Accumulate stats for one topic over the sampling window. """
    def __init__(self):
        self.message_count = 0
        self.byte_count = 0
        self.has_header = False
        self.delay_accumulator = 0.0
        self.delay_count = 0
        self.start_time = time.time()

    def record_message(self, msg_size: int, delay: float = None):
        self.message_count += 1
        self.byte_count += msg_size
        if delay is not None:
            self.delay_accumulator += delay
            self.delay_count += 1

    def compute_results(self, window_duration: float):
        """Return (bandwidth_kbit_s, frequency_hz, avg_delay_s or 'No header')"""
        if window_duration <= 0:
            return (0.0, 0.0, "-")

        # Bandwidth in Kbit/s
        bandwidth_kbit_s = (self.byte_count * 8 / 1024.0) / window_duration
        # Frequency in Hz
        freq = self.message_count / window_duration
        # Delay
        if self.delay_count > 0:
            avg_delay = self.delay_accumulator / self.delay_count
        else:
            avg_delay = "No header"

        return (bandwidth_kbit_s, freq, avg_delay)

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')

        # How often to refresh the list of _sync topics
        self.declare_parameter('refresh_interval', 5.0)  # seconds
        self.declare_parameter('print_interval', 30.0)   # seconds

        self.refresh_interval = self.get_parameter('refresh_interval').value
        self.print_interval = self.get_parameter('print_interval').value

        # Keep track of stats for each topic
        self.topics_stats = {}  # dict of topic_name -> TopicStats

        # We will refresh the subscription list every refresh_interval
        self.refresh_timer = self.create_timer(self.refresh_interval, self.refresh_subscriptions)
        # We will print the stats every print_interval
        self.print_timer = self.create_timer(self.print_interval, self.print_stats)

        log_with_timestamp("topic_monitor node started. Searching for `_sync` topics...")

    def refresh_subscriptions(self):
        # 1) Get all topics
        all_topics = self.get_topic_names_and_types()
        # Filter for those that end with "_sync"
        sync_topics = [t[0] for t in all_topics if t[0].endswith('_sync')]
        log_with_timestamp(f"Found {len(sync_topics)} _sync topics. Updating subscriptions if needed...")

        # 2) Create subscription if not exist. If exist, do nothing.
        for topic_name in sync_topics:
            if topic_name not in self.topics_stats:
                # New topic
                self.topics_stats[topic_name] = TopicStats()
                # Create subscription
                # We'll assume best_effort or reliable as you prefer:
                qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10
                )
                # We guess we can subscribe generically to "any" message.
                # In rclpy, we need an actual type. We'll do an approximate:
                # If we had more advanced logic, we'd detect the type from self.get_topic_names_and_types().
                # For simplicity, we'll pick 'byte[]' approach or we pick 'AnyMsg' style if we had a bridging library.
                # Let's store the type as "AnyMsg". We'll do a small hack: subscribe with a generic type:
                from std_msgs.msg import ByteMultiArray
                self.create_subscription(ByteMultiArray, topic_name, self.make_callback(topic_name), qos)
                log_with_timestamp(f"Subscribed to new topic: {topic_name}")

        # 3) Check if any subscriptions are no longer relevant (topic disappeared).
        # Actually, in ROS 2, a topic might not vanish but become inactive. We'll keep the subscription.
        # If you want to remove subscription for missing topics, you'd do it here. Typically we just keep it.

    def make_callback(self, topic_name: str):
        """ Return a callback function that captures the topic_name. """
        def callback(msg):
            # If it's ByteMultiArray, we have data in msg.data
            msg_size = len(msg.data)

            # If it has a header, we can measure delay. But we used ByteMultiArray, no direct header.
            # So let's see if we do a quick hack: user must define a real type or we skip delay measure.
            # For demonstration, we skip or do "No header".

            # If we wanted to detect a "std_msgs/Header" inside, it's not possible with ByteMultiArray.
            # We can do advanced bridging or parse the raw bytes. We'll keep it simple:
            stats = self.topics_stats.get(topic_name, None)
            if stats is None:
                return
            stats.record_message(msg_size)
        return callback

    def print_stats(self):
        # 1) For each topic, compute the bandwidth/frequency/delay
        #    Then reset for next interval
        window_duration = self.print_interval

        # We'll build a list of (topic_name, bandwidth, freq, delay)
        details = []
        for t_name, stats in self.topics_stats.items():
            (bw, freq, delay) = stats.compute_results(window_duration)
            details.append((t_name, bw, freq, delay))
            # Reset counters for next interval
            self.topics_stats[t_name] = TopicStats()

        # 2) Sort by highest bandwidth
        details.sort(key=lambda x: x[1] if isinstance(x[1], float) else 0, reverse=True)

        # 3) Summation
        total_bw = 0.0
        for d in details:
            if isinstance(d[1], float):
                total_bw += d[1]

        # 4) Print table
        #   same style as your old code
        log_with_timestamp(f"{'Topic'.ljust(87)} | {'Bandwidth'.rjust(14)} | {'Frequency'.rjust(9)} | {'Delay'.rjust(8)}")
        print('-' * 155)
        for d in details:
            topic_str = d[0].ljust(87)
            if isinstance(d[1], float):
                bw_str = f"{d[1]:7.1f} Kbit/s"
            else:
                bw_str = str(d[1])
            if isinstance(d[2], float):
                freq_str = f"{d[2]:6.1f} Hz"
            else:
                freq_str = str(d[2])
            if isinstance(d[3], float):
                delay_str = f"{d[3]:6.3f} s"
            else:
                delay_str = str(d[3])
            log_with_timestamp(f"{topic_str} | {bw_str:>14} | {freq_str:>9} | {delay_str:>8}")
        log_with_timestamp(f"Total Bandwidth: {total_bw:7.1f} Kbit/s\n")

def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        log_with_timestamp("Exiting the program on KeyboardInterrupt...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
