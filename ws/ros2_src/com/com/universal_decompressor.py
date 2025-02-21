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
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import yaml
import importlib
import re
import zlib
import bz2
import lz4.frame as lz4
import zstandard as zstd
import time
from threading import Lock

from std_msgs.msg import UInt8MultiArray

def format_size(size: int) -> str:
    """Format byte size into human-readable form."""
    if size < 1024:
        return f"{size} B"
    elif size < 1048576:
        return f"{size/1024:.1f} KB"
    elif size < 1073741824:
        return f"{size/1048576:.1f} MB"
    return f"{size/1073741824:.1f} GB"

def decompress_data(data: bytes, algorithm: str) -> bytes:
    """Decompress raw bytes according to the chosen algorithm."""
    if algorithm == 'gzip':
        return zlib.decompress(data)
    elif algorithm == 'bz2':
        return bz2.decompress(data)
    elif algorithm == 'lz4':
        return lz4.decompress(data)
    elif algorithm == 'zstd':
        dctx = zstd.ZstdDecompressor()
        return dctx.decompress(data)
    else:
        raise ValueError(f"Unsupported decompression algorithm: {algorithm}")

class UniversalDecompressorNode(Node):
    """
    A ROS 2 node that dynamically matches compressed topics and publishes the decompressed original message type.
    Configuration is read from a YAML file via 'config_file' param.
    """

    def __init__(self):
        super().__init__('universal_decompressor')

        self.declare_parameter('config_file', 'decompression_config.yaml')
        config_file = self.get_parameter('config_file').value

        self.get_logger().info(f"[universal_decompressor] Starting with config_file='{config_file}'")
        self.subscribed_topics = set()
        self.subscribe_lock = Lock()

        # Load config
        self.config = self.load_config(config_file)
        self.get_logger().info(f"[universal_decompressor] Node initialized with config file: {config_file}")

        # Timer to check for new topics every 5s
        self.timer = self.create_timer(5.0, self.check_and_subscribe)
        # Initial subscription attempt
        self.check_and_subscribe()

    def load_config(self, filename: str) -> dict:
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
        E.g. 'std_msgs/msg/String' -> get_message('std_msgs/msg/String').
        """
        try:
            return get_message(ros1_style_path)
        except Exception as e:
            self.get_logger().error(f"Could not load message class '{ros1_style_path}': {e}")
            return None

    def check_and_subscribe(self):
        """Periodically checks for compressed topics matching user config, sets up subscriptions if needed."""
        with self.subscribe_lock:
            if 'decompression' not in self.config:
                self.get_logger().warn("[universal_decompressor] No 'decompression' section in config.")
                return

            all_topics = self.get_topic_names_and_types()
            topic_map = {}
            for (tname, ttypes) in all_topics:
                if len(ttypes) > 0:
                    topic_map[tname] = ttypes[0]

            for item in self.config['decompression']:
                topic_regex = item.get('topic_regex', '')
                algorithm = item.get('algorithm', 'bz2')
                msg_type_str = item.get('msg_type', None)
                remove_suffix = item.get('remove_suffix', '')
                add_suffix = item.get('add_suffix', '')

                if not msg_type_str:
                    self.get_logger().error("[universal_decompressor] 'msg_type' missing in config for some item.")
                    continue

                target_msg_class = self.get_message_class(msg_type_str)
                if target_msg_class is None:
                    continue

                for tname, ttype in topic_map.items():
                    if re.search(topic_regex, tname):
                        # Check if topic ends with remove_suffix if so => produce a new name
                        if remove_suffix and tname.endswith(remove_suffix):
                            base_name = tname[:-len(remove_suffix)]
                            decompressed_topic = base_name + add_suffix
                        else:
                            # If we require remove_suffix but it doesn't match => skip
                            if remove_suffix:
                                self.get_logger().warn(
                                    f"Topic '{tname}' does not end with '{remove_suffix}', ignoring."
                                )
                                continue
                            decompressed_topic = tname + add_suffix

                        # Already subscribed?
                        if decompressed_topic not in self.subscribed_topics:
                            # We'll sub to tname (which is compressed, type=UInt8MultiArray),
                            # publish to decompressed_topic of type target_msg_class
                            pub = self.create_publisher(target_msg_class, decompressed_topic, 10)
                            self.create_subscription(
                                UInt8MultiArray,
                                tname,
                                lambda msg, pub=pub, cls=target_msg_class, algo=algorithm: self.decompress_callback(msg, pub, cls, algo),
                                10
                            )
                            self.subscribed_topics.add(decompressed_topic)
                            self.get_logger().info(
                                f"[universal_decompressor] Subscribed to '{tname}' => publishing decompressed to '{decompressed_topic}' with algorithm='{algorithm}'"
                            )

    def decompress_callback(self, msg: UInt8MultiArray, publisher, msg_class, algorithm: str):
        """Callback to decompress data from UInt8MultiArray and publish original message type."""
        compressed_data = bytes(msg.data)
        csize = len(compressed_data)
        csize_str = format_size(csize)
        self.get_logger().info(f"Received compressed msg, size: {csize_str}")

        start = time.time()
        try:
            raw_data = decompress_data(compressed_data, algorithm)
        except Exception as e:
            self.get_logger().error(f"Decompression error: {e}")
            return

        dtime = (time.time() - start) * 1000.0  # ms
        dsize = len(raw_data)
        dsize_str = format_size(dsize)

        self.get_logger().info(
            f"[{publisher.topic_name}] comp. size: {csize_str}, decomp. size: {dsize_str}, decomp. time: {dtime:.1f} ms"
        )

        # Attempt to deserialize
        try:
            # We can do: msg_class().deserialize(raw_data), but in rclpy we generally do:
            deserialized_obj = msg_class()
            # For rclpy, we can use rclpy.serialization.deserialize_message but it wants a known type
            # We'll just do the .deserialize method if the Python binding supports it
            # Alternatively:
            from rclpy.serialization import deserialize_message
            deserialized_obj = deserialize_message(raw_data, msg_class)

            # Publish the reconstructed message
            publisher.publish(deserialized_obj)
        except Exception as e:
            self.get_logger().error(f"Deserialization error: {e} - Possibly corrupt data?")

def main(args=None):
    rclpy.init(args=args)
    node = UniversalDecompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
