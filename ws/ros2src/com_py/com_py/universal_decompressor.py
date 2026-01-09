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
from rclpy.serialization import deserialize_message
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

from com_msgs.msg import CompressedData

def format_size(size: int) -> str:
    """Format a byte size into human-readable form."""
    if size < 1024:
        return f"{size} B"
    elif size < 1048576:
        return f"{size / 1024:.1f} KB"
    elif size < 1073741824:
        return f"{size / 1048576:.1f} MB"
    else:
        return f"{size / 1073741824:.1f} GB"

def decompress_data(input_bytes: bytes, algorithm: str) -> bytes:
    """
    Decompress raw bytes according to the chosen algorithm.
    Supported: 'bz2', 'zlib', 'lz4', 'zstd'.
    """
    if algorithm == 'bz2':
        return bz2.decompress(input_bytes)
    elif algorithm == 'zlib':
        return zlib.decompress(input_bytes)
    elif algorithm == 'lz4':
        return lz4.decompress(input_bytes)
    elif algorithm == 'zstd':
        return zstd.ZstdDecompressor().decompress(input_bytes)
    else:
        raise ValueError(f"Unsupported decompression algorithm: {algorithm}")

class UniversalDecompressorNode(Node):
    """
    A ROS 2 node that dynamically discovers compressed topics (type = com_msgs/msg/CompressedData),
    matches them to YAML 'decompression' rules with a regex, decompresses + deserializes,
    then publishes the uncompressed typed message.

    YAML structure (example):
      decompression:
        - topic_regex: "^/costmap/costmap_compressed$"
          # msg_type is OPTIONAL (legacy override). If omitted, the decompressor reads it from
          # the incoming com_msgs/msg/CompressedData.msg_type field.
          # msg_type: "nav_msgs/msg/OccupancyGrid"
          remove_suffix: "/{algorithm}"
          add_suffix: "/deco"
          algorithm: "bz2"
    """

    def __init__(self):
        super().__init__('universal_decompressor')

        # 1) Declare + get parameter for config file
        self.declare_parameter('config_file', 'decompression_config.yaml')
        config_file = self.get_parameter('config_file').value

        # Default decompression algorithm (used when a YAML rule does not specify one)
        self.declare_parameter('default_algorithm', 'bz2')
        self.default_algorithm = self.get_parameter('default_algorithm').value

        # QoS config + roles (configured via qos.py YAML)
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value
        self.sub_role = 'decompressor_sub'
        self.pub_role = 'decompressor_pub'
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        self.get_logger().info(f"[universal_decompressor] Starting with config_file='{config_file}'")
        self.get_logger().info(f"[universal_decompressor] default_algorithm='{self.default_algorithm}'")

        # 2) Load config
        self.config = self.load_config(config_file)

        # 3) Lock to synchronize subscription checks
        self.subscribe_lock = Lock()

        # 4) Keep track of what compressed topics we have already subscribed to
        self.subscribed_topics = set()
        # Keep subscription objects alive
        self._subscriptions = []

        # 4b) Cache per compressed topic: created typed publisher + message class.
        #     We create these lazily on the first received message because msg_type now comes
        #     with the CompressedData message itself.
        self._typed_pub_cache = {}  # compressed_topic -> dict(pub=..., typed_cls=..., msg_type=str, out_topic=str)
        self._cache_lock = Lock()

        # 5) Build a timer to re-check the graph for new topics every 5s
        #    (If you only want one-time subscription, remove the timer & call once.)
        self.timer = self.create_timer(5.0, self.check_and_subscribe)

        # 6) Do an initial subscription attempt
        self.check_and_subscribe()

    def load_config(self, filename: str) -> dict:
        """Load decompression config from YAML into a Python dict."""
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
        Periodically checks the ROS graph for topics of type com_msgs/msg/CompressedData,
        matches them against the 'decompression' config entries, and sets up typed
        subscriptions/publishers if not already subscribed.
        """
        with self.subscribe_lock:
            if 'decompression' not in self.config:
                self.get_logger().warn("[universal_decompressor] No 'decompression' section in config.")
                return

            # 1) Gather all active topics
            all_topics = self.get_topic_names_and_types()

            # 2) Filter for topics that have "com_msgs/msg/CompressedData" (or your custom name)
            #    (If there's multiple types, we check if any match.)
            compressed_map = {}  # topic_name -> True if it has compressed data
            for (tname, ttypes) in all_topics:
                for t in ttypes:
                    if t == "com_msgs/msg/CompressedData":
                        compressed_map[tname] = True
                        break

            # 3) For each config rule, check which topics match the regex
            for item in self.config['decompression']:
                topic_pattern = item.get('topic_regex', '')
                rx = re.compile(topic_pattern)

                # default algorithm = node parameter
                algorithm = item.get('algorithm', self.default_algorithm)
                # default remove_suffix = f'/{algorithm}', add_suffix = ""
                remove_suffix = item.get('remove_suffix', f'/{algorithm}')
                add_suffix = item.get('add_suffix', '')
                # Optional legacy override: allow msg_type in YAML, but it's no longer required.
                msg_type_hint = item.get('msg_type', '')

                # 4) Check each compressed topic to see if it matches
                for tname in compressed_map.keys():
                    if rx.search(tname):
                        # e.g. "/costmap/costmap_compressed" -> matches
                        if tname in self.subscribed_topics:
                            continue

                        # Build the output topic: remove the suffix, then add new one
                        out_topic = self.build_output_topic(tname, remove_suffix, add_suffix)

                        # QoS from roles (per-topic overrides supported via qos.yaml)
                        sub_qos = get_topic_qos(self.get_logger(), self.qos_config, tname, self.sub_role)

                        # Create subscription to the compressed topic
                        # We'll capture the needed info in a lambda
                        sub = self.create_subscription(
                            CompressedData,
                            tname,
                            lambda msg,
                                   algo=algorithm,
                                   original_topic=tname,
                                   out_topic=out_topic,
                                   msg_type_hint=msg_type_hint: self.decompression_callback(
                                       msg, algo, original_topic, out_topic, msg_type_hint
                                   ),
                            qos_profile=sub_qos
                        )
                        self._subscriptions.append(sub)

                        self.subscribed_topics.add(tname)
                        self.get_logger().info(
                            f"[universal_decompressor] Subscribed to '{tname}' -> publishing to '{out_topic}' "
                            f"(algo='{algorithm}', msg_type from message{' (yaml hint fallback)' if msg_type_hint else ''})"
                        )

    def build_output_topic(self, original_topic: str, remove_sfx: str, add_sfx: str) -> str:
        """Remove a suffix (if present) from the original_topic, then append a new suffix."""
        # 1) Remove suffix if the topic ends with it
        if remove_sfx and original_topic.endswith(remove_sfx):
            base_topic = original_topic[:-len(remove_sfx)]
        else:
            base_topic = original_topic
        # 2) Add the new suffix
        return base_topic + add_sfx

    def get_message_class(self, ros1_style_path: str):
        """Convert a 'pkg_name/msg/MessageName' string into a Python message class."""
        try:
            return get_message(ros1_style_path)
        except Exception as e:
            self.get_logger().error(f"Could not load message class '{ros1_style_path}': {e}")
            return None

    def _get_or_create_typed_publisher(self, compressed_topic: str, out_topic: str, msg_type: str):
        """
        Create (and cache) a typed publisher for `out_topic` based on `msg_type`.
        Returns (publisher, typed_cls).
        """
        with self._cache_lock:
            cached = self._typed_pub_cache.get(compressed_topic)
            if cached is not None:
                # If out_topic changes across rules for the same compressed topic, log once and keep first.
                if cached.get('out_topic') != out_topic:
                    self.get_logger().warn(
                        f"[universal_decompressor] '{compressed_topic}' out_topic changed from "
                        f"'{cached.get('out_topic')}' to '{out_topic}'. Keeping the first publisher."
                    )
                return cached['pub'], cached['typed_cls']

            typed_cls = self.get_message_class(msg_type)
            if typed_cls is None:
                return None, None

            pub_qos = get_topic_qos(self.get_logger(), self.qos_config, out_topic, self.pub_role)
            pub = self.create_publisher(typed_cls, out_topic, qos_profile=pub_qos)
            self._typed_pub_cache[compressed_topic] = {
                'pub': pub,
                'typed_cls': typed_cls,
                'msg_type': msg_type,
                'out_topic': out_topic,
            }
            self.get_logger().info(
                f"[universal_decompressor] Created typed publisher for '{compressed_topic}' -> '{out_topic}' "
                f"(type='{msg_type}')"
            )
            return pub, typed_cls

    def decompression_callback(self, compressed_msg, algorithm,
                               original_topic: str, out_topic: str, msg_type_hint: str):
        """
        Callback for a compressed message:
          1) Decompress
          2) Deserialize into typed_cls
          3) Publish
          4) Log stats
        """
        try:
            msg_type = getattr(compressed_msg, 'msg_type', '') or msg_type_hint
            if not msg_type:
                self.get_logger().error(
                    f"[{original_topic}] No msg_type in CompressedData and no msg_type hint in YAML; skipping."
                )
                return

            compressed_size = len(compressed_msg.data)
            if compressed_size == 0:
                self.get_logger().error(
                    f"[{original_topic}] Received empty compressed data. Skipping.")
                return

            publisher, typed_cls = self._get_or_create_typed_publisher(original_topic, out_topic, msg_type)
            if publisher is None or typed_cls is None:
                self.get_logger().error(
                    f"[{original_topic}] Cannot create publisher for msg_type='{msg_type}'. Skipping."
                )
                return

            start_time = time.time()

            # 1) Decompress
            decompressed_bytes = decompress_data(bytes(compressed_msg.data), algorithm)
            decompress_ms = (time.time() - start_time) * 1000.0

            decompressed_size = len(decompressed_bytes)

            # 2) Deserialize into typed message
            out_msg = deserialize_message(decompressed_bytes, typed_cls)

            # 3) Publish
            publisher.publish(out_msg)

            # 4) Optional: log stats
            ratio = 0.0
            if compressed_size > 0:
                ratio = float(decompressed_size) / float(compressed_size)

            self.get_logger().info(
                f"[{original_topic}] Decompressed from {format_size(compressed_size)} "
                f"to {format_size(decompressed_size)}, ratio={ratio:.2f}, "
                f"time={decompress_ms:.1f} ms (type='{msg_type}', algo='{algorithm}')"
            )

        except Exception as e:
            self.get_logger().error(
                f"[{original_topic}] Decompression or deserialization error: {e}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = UniversalDecompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
