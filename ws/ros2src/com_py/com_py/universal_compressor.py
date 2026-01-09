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
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from com_py.qos import load_qos_config, get_topic_qos

import yaml
import re
import bz2
import zlib
import lz4.frame as lz4
import zstandard as zstd
import time
from threading import Lock

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
    Recognized: 'bz2', 'zlib', 'lz4', 'zstd'.
    """
    if algorithm == 'bz2':
        return bz2.compress(data)
    elif algorithm == 'zlib':
        return zlib.compress(data)
    elif algorithm == 'lz4':
        return lz4.compress(data)
    elif algorithm == 'zstd':
        return zstd.ZstdCompressor().compress(data)
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

        # 1) Declare + get parameter for the config file
        self.declare_parameter('config_file', 'compression_config.yaml')
        config_file = self.get_parameter('config_file').value

        # Default compression algorithm (used when a YAML rule does not specify one)
        self.declare_parameter('default_algorithm', 'bz2')
        self.default_algorithm = self.get_parameter('default_algorithm').value

        self.get_logger().info(f"[universal_compressor] Starting with config_file='{config_file}'")
        self.get_logger().info(f"[universal_compressor] default_algorithm='{self.default_algorithm}'")

        # 2) Lock to protect subscription checks
        self.subscribe_lock = Lock()

        # 3) Simple set to track topics we've already subscribed to
        self.subscribed_topics = set()
        # Keep subscription objects alive
        self._subscriptions = []
        # 4) QoS config + roles (configured via qos.py YAML)
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value
        self.sub_role = 'compressor_sub'
        self.pub_role = 'compressor_pub'
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # 5) Load YAML config
        self.config = self.load_config(config_file)
        self.get_logger().info(f"[universal_compressor] Node initialized with config file: {config_file}")

        # 6) Set up a timer to periodically re-check the graph for new topics
        self.timer = self.create_timer(5.0, self.check_and_subscribe)

        # Also do an initial subscription check immediately
        self.check_and_subscribe()

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
        with self.subscribe_lock:
            if 'compression' not in self.config:
                self.get_logger().warn("[universal_compressor] No 'compression' section in config.")
                return

            # 1) Gather all active topics and their types
            #    e.g. [("/foo", ["std_msgs/msg/String"]), ("/costmap/costmap", ["nav_msgs/msg/OccupancyGrid"])]
            all_topics = self.get_topic_names_and_types()

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
                rx = re.compile(topic_pattern)

                for tname, type_name in topic_map.items():
                    if rx.search(tname):
                        # e.g. matched => /costmap/costmap, type=nav_msgs/msg/OccupancyGrid
                        out_topic = tname + add_suffix

                        if out_topic not in self.subscribed_topics:
                            msg_class = self.get_message_class(type_name)
                            if msg_class is None:
                                # If we can't load or parse the type, skip
                                continue

                            # QoS from roles (per-topic overrides supported via qos.yaml)
                            sub_qos = get_topic_qos(self.get_logger(), self.qos_config, tname, self.sub_role)
                            pub_qos = get_topic_qos(self.get_logger(), self.qos_config, tname, self.pub_role)

                            # Create publisher with CompressedData
                            pub = self.create_publisher(
                                CompressedData,
                                out_topic,
                                qos_profile=pub_qos
                            )

                            # Create subscription to the original message
                            # We'll pass a small lambda capturing the arguments
                            # so we know which publisher and algorithm to use.
                            # We'll also pass the 'tname' so we can log which topic we got.
                            sub = self.create_subscription(
                                msg_class,
                                tname,
                                lambda msg,
                                       publisher=pub,
                                       algo=algorithm,
                                       source_topic=tname,
                                       msg_type_str=type_name: self.compression_callback(
                                           msg, publisher, algo, source_topic, msg_type_str
                                       ),
                                qos_profile=sub_qos
                            )
                            self._subscriptions.append(sub)

                            self.subscribed_topics.add(out_topic)
                            self.get_logger().info(
                                f"[universal_compressor] Subscribed to '{tname}' "
                                f"(type={type_name}) => publishing compressed to '{out_topic}' "
                                f"with algorithm='{algorithm}'"
                            )

    def get_message_class(self, ros1_style_path: str):
        """
        Convert 'package_name/msg/MessageName' into a Python class object.
        E.g. 'std_msgs/msg/String' -> <class 'std_msgs.msg._string.String'>
        """
        try:
            return get_message(ros1_style_path)
        except Exception as e:
            self.get_logger().error(f"Could not load message class '{ros1_style_path}': {e}")
            return None

    def compression_callback(self, msg, publisher, algorithm, original_topic, msg_type_str: str):
        """
        Callback that:
          1) Serializes the incoming message
          2) Compresses the bytes
          3) Publishes them as 'com_msgs/msg/CompressedData'
        """
        try:
            # 1) Serialize the message into raw bytes
            serialized = serialize_message(msg)
            original_size = len(serialized)

            # 2) Compress
            start_time = time.time()
            compressed_bytes = compress_data(serialized, algorithm)
            elapsed_ms = (time.time() - start_time) * 1000.0

            compressed_size = len(compressed_bytes)

            # 3) Publish as custom CompressedData (include original type so decompressor can be type-agnostic)
            out_msg = CompressedData()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.msg_type = msg_type_str
            # Convert Python bytes -> list of uint8
            out_msg.data = list(compressed_bytes)

            publisher.publish(out_msg)

            # 4) Optional: Log stats
            ratio = 0.0
            if compressed_size > 0:
                ratio = float(original_size) / float(compressed_size)

            self.get_logger().info(
                f"[{original_topic}] original={format_size(original_size)}, "
                f"compressed={format_size(compressed_size)}, ratio={ratio:.2f}, "
                f"time={elapsed_ms:.1f} ms (algo={algorithm}, type='{msg_type_str}')"
            )

        except Exception as e:
            self.get_logger().error(f"[{original_topic}] Compression error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UniversalCompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
