#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2024-11-13
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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

from array import array
import bz2
import re
import time
from threading import Lock
import zlib

import lz4.frame as lz4
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.executors import ExternalShutdownException
from rosidl_runtime_py.utilities import get_message

from com_py.qos import get_topic_qos, load_qos_config
# import zstandard as zstd
# zstandard is not available on the Ubuntu 26.04 / ROS 2 Lyrical setup.
# Reject zstd explicitly when a user configures it.

# Adjust the import below to your custom package and message name:
# e.g. from com_cpp.msg import CompressedData
from com_msgs.msg import CompressedData


def format_size(size: int) -> str:
    """Format a byte size into a human-readable string."""
    if size < 1024:
        return f"{size} B"
    elif size < 1048576:
        return f"{size / 1024:.1f} KB"
    elif size < 1073741824:
        return f"{size / 1048576:.1f} MB"
    else:
        return f"{size / 1073741824:.1f} GB"

def compress_data(data: bytes, algorithm: str) -> bytes:
    """
    Compress raw bytes according to the chosen algorithm.
    Recognized: 'bz2', 'zlib', 'lz4'.
    """
    if algorithm == 'bz2':
        return bz2.compress(data)
    elif algorithm == 'zlib':
        return zlib.compress(data)
    elif algorithm == 'lz4':
        return lz4.compress(data)
    elif algorithm == 'zstd':
        raise ValueError(
            "zstd compression is not supported on this Ubuntu 26.04 / ROS 2 Lyrical setup. "
            "Choose bz2, zlib, or lz4 instead."
        )
    else:
        raise ValueError(f"Unsupported compression algorithm: {algorithm}")


class UniversalCompressorNode(Node):
    """
    A ROS 2 node that dynamically searches for topics matching user-defined regex
    and publishes a compressed version of their messages to a new topic.

    Configuration is read from a YAML file specified via the 'config_file' parameter.
    The YAML has a 'compression:' list of dictionaries, each specifying:
      - topic_regex
      - algorithm (optional, default="bz2")
      - add_suffix (optional, default="/{algorithm}")
    """

    def __init__(self):
        super().__init__('universal_compressor')

        # Use the node's default callback group for timers and subscriptions.
        # On this Lyrical/Fast DDS setup, dynamically created entities in
        # custom callback groups were visible in the graph but did not keep
        # dispatching reliably.

        # 1) Declare + get parameter for the config file
        self.declare_parameter('config_file', 'compression_config.yaml')
        config_file = self.get_parameter('config_file').value

        # Default compression algorithm (used when a YAML rule does not specify one)
        self.declare_parameter('default_algorithm', 'bz2')
        self.default_algorithm = self.get_parameter('default_algorithm').value

        self.get_logger().info(
            f"[universal_compressor] Starting with config_file='{config_file}'"
        )
        self.get_logger().info(
            f"[universal_compressor] default_algorithm='{self.default_algorithm}'"
        )

        self.declare_parameter('stats_log_period_s', 10.0)
        self.stats_log_period_s = float(
            self.get_parameter('stats_log_period_s').value
        )
        self.declare_parameter('slow_callback_warn_ms', 1000.0)
        self.slow_callback_warn_ms = float(
            self.get_parameter('slow_callback_warn_ms').value
        )
        self.declare_parameter('no_input_warn_period_s', 15.0)
        self.no_input_warn_period_s = float(
            self.get_parameter('no_input_warn_period_s').value
        )

        # 3) Simple set to track topics we've already subscribed to
        self.subscribed_topics = set()
        # Keep our own references without touching rclpy.Node private fields.
        self._owned_subscriptions = []
        self._owned_publishers = []
        self._owned_output_topics = set()
        self._topic_stats = {}
        self._stats_lock = Lock()
        # 4) QoS config + roles (configured via qos.py YAML)
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value
        self.sub_role = 'compressor_sub'
        self.pub_role = 'compressor_pub'
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # 5) Load YAML config
        self.config = self.load_config(config_file)
        compression_rules = self.config.get('compression', [])
        if isinstance(compression_rules, list):
            self.get_logger().info(
                f"[universal_compressor] Loaded {len(compression_rules)} "
                f"compression rule(s). "
                f"stats_log_period_s={self.stats_log_period_s:.1f}, "
                f"slow_callback_warn_ms={self.slow_callback_warn_ms:.1f}"
            )
        self.get_logger().info(
            f"[universal_compressor] Node initialized with config file: {config_file}"
        )

        # 6) Set up a timer to periodically re-check the graph for new topics
        self.timer = self.create_timer(
            5.0,
            self.check_and_subscribe,
        )

        # Discovery is timer-driven once the executor is spinning. Calling
        # get_topic_names_and_types() before spin can leave this node with
        # graph entities that are visible but never dispatch callbacks on the
        # Lyrical/Fast DDS setup used here.

    def load_config(self, filename: str) -> dict:
        """Load compression configuration from a YAML file into a Python dict."""
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
                self.get_logger().debug(f"Configuration loaded: {data}")
                return data or {}
        except Exception as e:
            self.get_logger().error(f"Failed to load config from '{filename}': {e}")
            return {}

    def check_and_subscribe(self):
        """
        Checks the ROS graph for topics, matches them against config['compression'] entries,
        and sets up new subscribers (with corresponding publishers) if not already subscribed.
        """
        try:
            self._check_and_subscribe_impl()
        except Exception as e:
            self.get_logger().error(
                f"[universal_compressor] Discovery error: {type(e).__name__}: {e}"
            )

    def _check_and_subscribe_impl(self):
        self._warn_about_inactive_subscriptions()

        if 'compression' not in self.config:
            self.get_logger().warning(
                "[universal_compressor] No 'compression' section in config."
            )
            return

        # 1) Gather all active topics and their types
        #    e.g. [("/foo", ["std_msgs/msg/String"]), ...]
        graph_start = time.monotonic()
        all_topics = self.get_topic_names_and_types()
        graph_ms = (time.monotonic() - graph_start) * 1000.0
        if graph_ms > self.slow_callback_warn_ms:
            self.get_logger().warning(
                f"[universal_compressor] ROS graph lookup took "
                f"{graph_ms:.1f} ms "
                f"for {len(all_topics)} topic(s)."
            )

        # 2) Build a dictionary: topic_name -> first_type
        topic_map = {}
        for (tname, ttypes) in all_topics:
            if len(ttypes) > 0:
                topic_map[tname] = ttypes[0]

        # 3) For each rule in config['compression'], check regex
        for item in self.config['compression']:
            topic_pattern = item.get('topic_regex', '')
            # Default to node parameter if no algorithm provided:
            algorithm = item.get('algorithm', self.default_algorithm)
            add_suffix = item.get('add_suffix', f'/{algorithm}')

            # Pre-compile the regex for efficiency if you like:
            try:
                rx = re.compile(topic_pattern)
            except re.error as e:
                self.get_logger().error(
                    f"[universal_compressor] Invalid "
                    f"topic_regex='{topic_pattern}': {e}"
                )
                continue

            match_count = 0

            for tname, type_name in topic_map.items():
                if rx.search(tname):
                    match_count += 1
                    # Avoid recursively compressing our own compressed output topics.
                    if tname in self._owned_output_topics:
                        continue
                    if type_name == 'com_msgs/msg/CompressedData':
                        continue
                    if add_suffix and tname.endswith(add_suffix):
                        continue

                    # e.g. matched => /costmap/costmap.
                    out_topic = tname + add_suffix
                    subscription_key = (tname, out_topic)

                    if subscription_key not in self.subscribed_topics:
                        msg_class = self.get_message_class(type_name)
                        if msg_class is None:
                            # If we can't load or parse the type, skip
                            continue

                        # QoS from roles.
                        sub_qos = get_topic_qos(
                            self.get_logger(),
                            self.qos_config,
                            tname,
                            self.sub_role,
                        )
                        sub_qos = self._adapt_sub_qos_to_publishers(tname, sub_qos)
                        pub_qos = get_topic_qos(
                            self.get_logger(),
                            self.qos_config,
                            tname,
                            self.pub_role,
                        )

                        # Create publisher with CompressedData
                        pub = self.create_publisher(
                            CompressedData,
                            out_topic,
                            pub_qos,
                        )
                        self._owned_publishers.append(pub)
                        self._owned_output_topics.add(out_topic)

                        # Create subscription to the original message
                        # We'll pass a small lambda capturing the arguments
                        # so we know which publisher and algorithm to use.
                        # Pass 'tname' so logs identify the source topic.
                        sub = self.create_subscription(
                            msg_class,
                            tname,
                            self.make_compression_callback(
                                pub,
                                algorithm,
                                tname,
                                type_name,
                            ),
                            sub_qos,
                        )
                        self._owned_subscriptions.append(sub)

                        self.subscribed_topics.add(subscription_key)
                        self._init_topic_stats(
                            tname, out_topic, algorithm, type_name
                        )
                        self.get_logger().info(
                            f"[universal_compressor] Subscribed to '{tname}' "
                            f"(type={type_name}) => publishing compressed "
                            f"to '{out_topic}' "
                            f"with algorithm='{algorithm}'"
                        )

            if match_count == 0:
                self.get_logger().debug(
                    f"[universal_compressor] Rule topic_regex='{topic_pattern}' "
                    f"matched no topics "
                    f"among {len(topic_map)} discovered topic(s)."
                )

        self.get_logger().info(
            f"[universal_compressor] Discovery pass complete; "
            f"subscriptions={len(self.subscribed_topics)}"
        )

    def _qos_reliability_name(self, qos) -> str:
        try:
            return qos.reliability.name
        except Exception:
            return str(getattr(qos, 'reliability', 'unknown'))

    def _adapt_sub_qos_to_publishers(self, topic: str, sub_qos):
        """
        Make the compressor subscription compatible with discovered publishers.

        A RELIABLE subscription cannot receive from a BEST_EFFORT publisher. The
        graph still shows both endpoints, which looks like a silent hang. For a
        transparent compressor, prefer receiving data over enforcing reliability.
        """
        try:
            publishers = self.get_publishers_info_by_topic(topic)
        except Exception as e:
            self.get_logger().warning(
                f"[universal_compressor] Could not inspect publishers for "
                f"'{topic}' QoS: {e}. Using configured subscriber QoS "
                f"reliability={self._qos_reliability_name(sub_qos)}."
            )
            return sub_qos

        if not publishers:
            self.get_logger().info(
                f"[universal_compressor] No publisher QoS info available yet "
                f"for '{topic}'. Using configured subscriber QoS "
                f"reliability={self._qos_reliability_name(sub_qos)}."
            )
            return sub_qos

        offered = []
        saw_best_effort = False
        for info in publishers:
            qos = getattr(info, 'qos_profile', None)
            if qos is None:
                continue
            reliability = self._qos_reliability_name(qos)
            offered.append(reliability)
            if getattr(qos, 'reliability', None) == ReliabilityPolicy.BEST_EFFORT:
                saw_best_effort = True

        before = self._qos_reliability_name(sub_qos)
        if saw_best_effort and sub_qos.reliability != ReliabilityPolicy.BEST_EFFORT:
            sub_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.get_logger().warning(
                f"[universal_compressor] '{topic}' has BEST_EFFORT publisher(s); "
                f"using BEST_EFFORT compressor_sub QoS instead of {before}. "
                f"Offered publisher reliabilities={offered}."
            )
        else:
            self.get_logger().info(
                f"[universal_compressor] '{topic}' compressor_sub QoS "
                f"reliability={self._qos_reliability_name(sub_qos)}; "
                f"offered publisher reliabilities={offered}."
            )

        return sub_qos

    def get_message_class(self, ros1_style_path: str):
        """
        Convert 'package_name/msg/MessageName' into a Python class object.
        E.g. 'std_msgs/msg/String' -> <class 'std_msgs.msg._string.String'>
        """
        try:
            return get_message(ros1_style_path)
        except Exception as e:
            self.get_logger().error(
                f"Could not load message class '{ros1_style_path}': {e}"
            )
            return None

    def _init_topic_stats(
        self,
        topic: str,
        out_topic: str,
        algorithm: str,
        msg_type: str,
    ):
        now = time.monotonic()
        with self._stats_lock:
            self._topic_stats.setdefault(topic, {
                'out_topic': out_topic,
                'algorithm': algorithm,
                'msg_type': msg_type,
                'received': 0,
                'published': 0,
                'errors': 0,
                'created_time': now,
                'last_no_input_warn_time': now,
                'last_log_time': now,
                'last_rx_time': None,
                'last_publish_time': None,
                'total_original_bytes': 0,
                'total_compressed_bytes': 0,
                'total_compress_ms': 0.0,
            })

    def _record_receive(self, topic: str) -> int:
        now = time.monotonic()
        with self._stats_lock:
            stats = self._topic_stats.setdefault(topic, {
                'out_topic': '',
                'algorithm': '',
                'msg_type': '',
                'received': 0,
                'published': 0,
                'errors': 0,
                'created_time': now,
                'last_no_input_warn_time': now,
                'last_log_time': now,
                'last_rx_time': None,
                'last_publish_time': None,
                'total_original_bytes': 0,
                'total_compressed_bytes': 0,
                'total_compress_ms': 0.0,
            })
            stats['received'] += 1
            stats['last_rx_time'] = now
            return stats['received']

    def _warn_about_inactive_subscriptions(self):
        now = time.monotonic()
        warnings = []
        with self._stats_lock:
            for topic, stats in self._topic_stats.items():
                if stats['received'] > 0:
                    continue
                age_s = now - stats['created_time']
                since_warn_s = now - stats['last_no_input_warn_time']
                if age_s < self.no_input_warn_period_s:
                    continue
                if since_warn_s < self.no_input_warn_period_s:
                    continue
                stats['last_no_input_warn_time'] = now
                warnings.append((
                    topic,
                    stats['out_topic'],
                    stats['algorithm'],
                    stats['msg_type'],
                    age_s,
                ))

        for topic, out_topic, algorithm, msg_type, age_s in warnings:
            self.get_logger().warning(
                f"[{topic}] Subscribed {age_s:.1f}s ago but no input "
                f"callbacks have fired yet. If the topic is publishing, "
                f"check subscriber QoS role='{self.sub_role}', type='{msg_type}', "
                f"algorithm='{algorithm}', out='{out_topic}'."
            )

    def _record_publish(
        self,
        topic: str,
        original_size: int,
        compressed_size: int,
        compress_ms: float,
    ):
        now = time.monotonic()
        with self._stats_lock:
            stats = self._topic_stats[topic]
            stats['published'] += 1
            stats['last_publish_time'] = now
            stats['total_original_bytes'] += original_size
            stats['total_compressed_bytes'] += compressed_size
            stats['total_compress_ms'] += compress_ms

            due = (now - stats['last_log_time']) >= self.stats_log_period_s
            if not due and stats['published'] != 1:
                return None

            window_s = max(now - stats['last_log_time'], 1e-9)
            total_compressed = max(stats['total_compressed_bytes'], 1)
            ratio = (
                float(stats['total_original_bytes']) / float(total_compressed)
            )
            avg_ms = stats['total_compress_ms'] / max(stats['published'], 1)
            hz = stats['published'] / window_s if due else 0.0
            stats['last_log_time'] = now

        if due:
            return (
                f"[{topic}] compressed stats: rx={stats['received']}, "
                f"pub={stats['published']}, errors={stats['errors']}, "
                f"avg_time={avg_ms:.1f} ms, ratio={ratio:.2f}, "
                f"recent_pub_hz={hz:.2f}, out='{stats['out_topic']}'"
            )
        return (
            f"[{topic}] first message compressed: "
            f"original={format_size(original_size)}, "
            f"compressed={format_size(compressed_size)}, ratio={ratio:.2f}, "
            f"time={compress_ms:.1f} ms "
            f"(algo={stats['algorithm']}, type='{stats['msg_type']}')"
        )

    def _record_error(self, topic: str):
        with self._stats_lock:
            stats = self._topic_stats.get(topic)
            if stats is not None:
                stats['errors'] += 1

    def make_compression_callback(
        self,
        publisher,
        algorithm: str,
        original_topic: str,
        msg_type_str: str,
    ):
        def callback(msg):
            self.compression_callback(
                msg,
                publisher,
                algorithm,
                original_topic,
                msg_type_str,
            )

        return callback

    def compression_callback(
        self,
        msg,
        publisher,
        algorithm,
        original_topic,
        msg_type_str: str,
    ):
        """
        Callback that:
          1) Serializes the incoming message
          2) Compresses the bytes
          3) Publishes them as 'com_msgs/msg/CompressedData'
        """
        callback_start = time.monotonic()
        try:
            receive_count = self._record_receive(original_topic)
            if receive_count == 1:
                self.get_logger().info(
                    f"[{original_topic}] First input message received; "
                    f"serializing and compressing "
                    f"type='{msg_type_str}' with algorithm='{algorithm}'."
                )

            # 1) Serialize the message into raw bytes
            serialize_start = time.monotonic()
            serialized = serialize_message(msg)
            serialize_ms = (time.monotonic() - serialize_start) * 1000.0
            original_size = len(serialized)
            if serialize_ms > self.slow_callback_warn_ms:
                self.get_logger().warning(
                    f"[{original_topic}] Serialization took "
                    f"{serialize_ms:.1f} ms "
                    f"for {format_size(original_size)}."
                )

            # 2) Compress
            start_time = time.monotonic()
            compressed_bytes = compress_data(serialized, algorithm)
            elapsed_ms = (time.monotonic() - start_time) * 1000.0

            compressed_size = len(compressed_bytes)
            if elapsed_ms > self.slow_callback_warn_ms:
                self.get_logger().warning(
                    f"[{original_topic}] Compression took "
                    f"{elapsed_ms:.1f} ms "
                    f"for {format_size(original_size)} -> "
                    f"{format_size(compressed_size)} "
                    f"(algo={algorithm})."
                )

            # 3) Publish as custom CompressedData.
            out_msg = CompressedData()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.msg_type = msg_type_str
            # Avoid list(compressed_bytes): it expands each byte into a Python
            # int and can make large messages look like a hung compressor.
            out_msg.data = array('B', compressed_bytes)

            publisher.publish(out_msg)

            callback_ms = (time.monotonic() - callback_start) * 1000.0
            if callback_ms > self.slow_callback_warn_ms:
                self.get_logger().warning(
                    f"[{original_topic}] Full compression callback took "
                    f"{callback_ms:.1f} ms "
                    f"(serialize={serialize_ms:.1f} ms, "
                    f"compress={elapsed_ms:.1f} ms, "
                    f"publish/assign="
                    f"{(callback_ms - serialize_ms - elapsed_ms):.1f} ms)."
                )

            stats_log = self._record_publish(
                original_topic, original_size, compressed_size, elapsed_ms
            )
            if stats_log:
                self.get_logger().info(stats_log)

        except Exception as e:
            self._record_error(original_topic)
            self.get_logger().error(f"[{original_topic}] Compression error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UniversalCompressorNode()

    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
