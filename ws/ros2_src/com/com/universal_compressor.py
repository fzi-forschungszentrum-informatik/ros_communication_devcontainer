#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must retain the above copyright
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

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

import yaml
import importlib
import re
import zlib
import bz2
import lz4.frame as lz4
import zstandard as zstd
import io
import time
from threading import Lock

from std_msgs.msg import UInt8MultiArray

def format_size(size: int) -> str:
    """Format a byte size into human-readable form."""
    if size < 1024:
        return f"{size} B"
    elif size < 1048576:
        return f"{size/1024:.1f} KB"
    elif size < 1073741824:
        return f"{size/1048576:.1f} MB"
    return f"{size/1073741824:.1f} GB"

def compress_data(data: bytes, algorithm: str) -> bytes:
    """Compress raw bytes according to the chosen algorithm."""
    if algorithm == 'gzip':
        return zlib.compress(data)
    elif algorithm == 'bz2':
        return bz2.compress(data)
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
    """

    def __init__(self):
        super().__init__('universal_compressor')

        # Declare + get parameters
        self.declare_parameter('config_file', 'compression_config.yaml')
        config_file = self.get_parameter('config_file').value

        self.get_logger().info(f"[universal_compressor] Starting with config_file='{config_file}'")
        self.subscribed_topics = set()
        self.subscribe_lock = Lock()

        # Load config from YAML
        self.config = self.load_config(config_file)
        self.get_logger().info(f"[universal_compressor] Node initialized with config file: {config_file}")

        # Create a timer to periodically check for new or updated topics
        # (Ros2 approach: 5-second timer)
        self.timer = self.create_timer(5.0, self.check_and_subscribe)
        # Initial subscription attempt
        self.check_and_subscribe()

    def load_config(self, filename: str) -> dict:
        """Load compression configuration from a YAML file."""
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
                self.get_logger().debug(f"Configuration loaded: {data}")
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load config from '{filename}': {e}")
            return {}

    def get_message_class(self, ros1_style_path: str):
        """
        Convert a 'package_name/msg/MessageName' string into a ROS 2 message type.
        E.g. 'std_msgs/msg/String' -> get_message('std_msgs/msg/String').
        """
        try:
            return get_message(ros1_style_path)
        except Exception as e:
            self.get_logger().error(f"Could not load message class '{ros1_style_path}': {e}")
            return None

    def check_and_subscribe(self):
        """
        Periodically checks published topics, matching them against config['compression'] entries,
        and sets up subscribers if not already subscribed.
        """
        with self.subscribe_lock:
            if 'compression' not in self.config:
                self.get_logger().warn("[universal_compressor] No 'compression' section in config.")
                return

            # get_topic_names_and_types() returns list of (topic_name, [type_names])
            all_topics = self.get_topic_names_and_types()
            # Flatten out each topic name => type_name
            # Usually type_names is a list with one element
            # e.g. [("/foo", ["std_msgs/msg/String"])]
            # We only take the first if there's multiple.
            topic_map = {}
            for (tname, ttypes) in all_topics:
                if len(ttypes) > 0:
                    # just pick the first type
                    topic_map[tname] = ttypes[0]

            for item in self.config['compression']:
                topic_regex = item.get('topic_regex', '')
                algorithm = item.get('algorithm', 'bz2')
                add_suffix = item.get('add_suffix', '_compressed')

                for tname, msg_type in topic_map.items():
                    # If tname matches the regex
                    if re.search(topic_regex, tname):
                        compressed_topic = tname + add_suffix
                        if compressed_topic not in self.subscribed_topics:
                            # We'll subscribe => publish
                            msg_class = self.get_message_class(msg_type)
                            if msg_class is None:
                                continue

                            # Create publisher
                            pub = self.create_publisher(UInt8MultiArray, compressed_topic, 10)

                            # Create subscriber
                            # We'll pass (pub, algorithm) in partial arguments
                            self.create_subscription(
                                msg_class,
                                tname,
                                lambda msg, pub=pub, algo=algorithm: self.compression_callback(msg, pub, algo),
                                10
                            )

                            self.subscribed_topics.add(compressed_topic)
                            self.get_logger().info(
                                f"[universal_compressor] Subscribed to '{tname}' => publishing compressed to '{compressed_topic}' with algorithm='{algorithm}'"
                            )

    def compression_callback(self, msg, publisher, algorithm):
        """Callback that compresses the incoming message and publishes a UInt8MultiArray."""
        try:
            # Serialize the message
            serialized = serialize_message(msg)
            original_size = len(serialized)
            original_size_formatted = format_size(original_size)

            start_time = time.time()
            compressed_data = compress_data(serialized, algorithm)
            comp_duration = (time.time() - start_time) * 1000.0  # ms
            compressed_size = len(compressed_data)
            compressed_size_formatted = format_size(compressed_size)

            self.get_logger().info(
                f"[{publisher.topic_name}] orig. size: {original_size_formatted}, "
                f"comp. size: {compressed_size_formatted}, "
                f"comp. time: {comp_duration:.1f} ms"
            )

            compressed_msg = UInt8MultiArray(data=list(compressed_data))
            publisher.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Compression error in callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = UniversalCompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
