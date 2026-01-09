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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import datetime
import re
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Float64
import subprocess
import shutil
import threading
from typing import Optional


def tshark_cmd(interface: str, host_ip: str, peer_ip: str, direction: str, duration: int):
    if direction == "up":
        flt = f"src host {host_ip} and dst host {peer_ip} and ip"
    elif direction == "down":
        flt = f"src host {peer_ip} and dst host {host_ip} and ip"
    else:
        raise ValueError("direction must be 'up' or 'down'")
    return [
        "sudo", "tshark", "-i", interface, "-n",
        "-f", flt,
        "-a", f"duration:{duration}",
        "-T", "fields",
        "-e", "ip.len",
    ]


def measure_peer_bytes(
    interface: str,
    host_ip: str,
    peer_ip: str,
    direction: str,
    print_interval: float,
    window_s: float,
) -> int:
    """
    Returns up_bytes (host_ip -> peer_ip) if direction='out',
    or down_bytes (peer_ip -> host_ip) if direction='in', over window_s using ip.len.
    """
    if shutil.which("tshark") is None:
        raise RuntimeError("tshark not found (sudo apt install tshark)")

    duration = max(1, int(round(window_s)))

    def run_and_sum(direction: str):
        cmd = tshark_cmd(interface, host_ip, peer_ip, direction, duration)
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=True,
            timeout=print_interval,
        )
        total = 0
        for line in proc.stdout.splitlines():
            ln = line.strip()
            try:
                ln_i = int(ln)
                total += ln_i
            except ValueError:
                continue
        return total

    if direction == "out":
        # host_ip -> peer_ip (up)
        return run_and_sum("up")
    elif direction == "in":
        # peer_ip -> host_ip (down)
        return run_and_sum("down")
    else:
        raise ValueError("direction must be 'in' or 'out'")


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
        self.declare_parameter("interface", "")      # optional override
        self.declare_parameter("link_duration_buffer_s", 1.0) # buffer subtracted from print_interval for tshark capture

        self.refresh_interval = self.get_parameter('refresh_interval').value
        self.print_interval = self.get_parameter('print_interval').value
        self.sourcename = self.get_parameter('sourcename').value
        self.direction = self.get_parameter('direction').value
        self.to_adressant = self.get_parameter('to_adressant').value
        self.host_ip = self.get_parameter("ip_local").value
        self.peer_ip = self.get_parameter("ip_remote").value
        self.interface = self.get_parameter("interface").value
        self.link_duration_buffer_s = float(self.get_parameter("link_duration_buffer_s").value)
        self.link_window_s = max(0.1, self.print_interval - self.link_duration_buffer_s)

        # Track actual elapsed time between prints (avoid timer drift issues)
        self._last_print_t = time.monotonic()

        # Link measurement async state
        self._link_lock = threading.Lock()
        self._link_event = threading.Event()
        self._link_inflight = False
        self._link_last_kbps: Optional[float] = None
        self._link_last_err: Optional[Exception] = None

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

        if not self.host_ip:
            raise ValueError("Parameter 'ip_local' is required for link bandwidth measurement")

        if not self.peer_ip:
            raise ValueError("Parameter 'ip_remote' is required for link bandwidth measurement")

        if not self.interface:
            raise ValueError("Parameter 'interface' is required for link bandwidth measurement")

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

        self.warned_topics = set()  # Track topics that triggered a warning

        # Publisher for total ROS topic bandwidth
        self.ros_topic_bandwidth_topic = f"/topic_monitor/{self.direction}/{self.sourcename}/ros_topic_bandwidth_kbps"
        if self.to_adressant:
            self.ros_topic_bandwidth_topic = f"/topic_monitor/{self.direction}/{self.sourcename}/to_{self.to_adressant}/ros_topic_bandwidth_kbps"
        self.ros_topic_bandwidth_pub = self.create_publisher(
            Float64, self.ros_topic_bandwidth_topic, qos_profile=QoSProfile(depth=10)
        )

        self.get_logger().info(f"topic_monitor node started. Searching for topics with prefix: {self.topic_prefix}")
        self.get_logger().info(f"Publishing ROS topic bandwidth to: {self.ros_topic_bandwidth_topic}")
        self.get_logger().info(f"Publishing link bandwidth to: {self.link_bandwidth_topic}")

        # Start the first link measurement immediately so the first print has a chance to show it
        self._kick_link_measurement()

    def refresh_subscriptions(self):
        all_topics = self.get_topic_names_and_types()
        topic_map = {tname: ttypes[0] for tname, ttypes in all_topics if ttypes}

        filtered_topics = [t for t in topic_map.keys() if t.startswith(self.topic_prefix)]

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

    def _kick_link_measurement(self):
        # Do not start a new measurement if one is still running (avoid overlap)
        if self._link_inflight:
            return

        self._link_inflight = True
        self._link_event.clear()

        def worker():
            self.get_logger().info(f"Starting link measurement ({self.direction}) with window {self.link_window_s} seconds.")
            link_kbps = None
            link_exception = None
            try:
                iface = self.interface
                if self.direction == 'out':
                    up_bytes = measure_peer_bytes(
                        interface=self.interface,
                        host_ip=self.host_ip,
                        peer_ip=self.peer_ip,
                        direction="out",
                        window_s=self.link_window_s,
                        print_interval=self.print_interval,
                    )
                    link_kbps = (up_bytes * 8.0 / 1024.0) / max(1e-9, self.link_window_s)
                else:  # direction == 'in'
                    down_bytes = measure_peer_bytes(
                        interface=iface,
                        host_ip=self.host_ip,
                        peer_ip=self.peer_ip,
                        direction="in",
                        window_s=self.link_window_s,
                        print_interval=self.print_interval,
                    )
                    link_kbps = (down_bytes * 8.0 / 1024.0) / max(1e-9, self.link_window_s)
            except Exception as e:
                link_exception = e
                self.get_logger().warn(f"Link bandwidth measurement failed: {e}")

            with self._link_lock:
                self._link_last_kbps = link_kbps
                self._link_last_err = link_exception

            self.get_logger().info(f"Link measurement ({self.direction}) finished. Publishing results later on print_stats()")
            self._link_inflight = False
            self._link_event.set()

        threading.Thread(target=worker, daemon=True).start()

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

        # Wait for the link measurement that was started after the previous print_stats()
        wait_timeout = 0.0
        if not self._link_event.wait(timeout=wait_timeout):
            self.get_logger().warn(f"Link Bandwidth: not ready yet (still measuring) after {wait_timeout} seconds. Skipping publication. Increase link_duration_buffer_s parameter to avoid this issue.")
        else:
            with self._link_lock:
                link_kbps = self._link_last_kbps
                link_exception = self._link_last_err

            if link_exception is not None:
                self.get_logger().warn(f"Link bandwidth measurement failed: {link_exception}")
            elif link_kbps is not None:
                self.get_logger().info(f"Link Bandwidth: {link_kbps:7.1f} Kbit/s")

                m = Float64()
                m.data = float(link_kbps)
                self.link_bandwidth_pub.publish(m)

        self.get_logger().info("")

        # Trigger next link measurement for the next print cycle
        self._kick_link_measurement()


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
