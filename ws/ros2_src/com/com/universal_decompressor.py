#!/usr/bin/env python3
# best_effort_decompressor.py

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

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import concurrent.futures
import yaml
import re
import zlib
import bz2
import lz4.frame as lz4
import zstandard as zstd
import time

from std_msgs.msg import UInt8MultiArray


def format_size(size: int) -> str:
    """Format byte size into a human-readable string."""
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


class BestEffortDecompressor(Node):
    """
    A best-effort, multi-threaded decompressor node in ROS 2.

    - Subscribes to a compressed UInt8MultiArray topic (matching config).
    - Decompresses in parallel using a thread pool.
    - Publishes the deserialized message on a new topic.
    - QoS is set to BEST_EFFORT with depth=10 to store up to 10 messages.

    The configuration is loaded from a YAML file:
      'decompression' : [
         { 'topic_regex': '/compressed_topic.*', 'algorithm': 'zstd',
           'msg_type': 'nav_msgs/msg/OccupancyGrid',
           'remove_suffix': '_compressed', 'add_suffix': '_uncompressed' },
         ...
      ]

    The user can specify the final published type via 'msg_type'.
    """

    def __init__(self):
        super().__init__("best_effort_decompressor")

        # Declare & get parameters
        self.declare_parameter('config_file', 'decompression_config.yaml')
        config_file = self.get_parameter('config_file').value

        # Logging
        self.get_logger().info(f"[best_effort_decompressor] config_file='{config_file}'")

        # Load config from YAML
        self.config = self._load_config(config_file)
        self.get_logger().info("[best_effort_decompressor] Node initialized")

        # Create a ReentrantCallbackGroup so multiple callbacks can run concurrently
        self.callback_group = ReentrantCallbackGroup()

        # Create a thread pool for actual decompression tasks
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=8)

        # Create a timer to periodically re-check for new matching topics
        self.create_timer(5.0, self._check_and_subscribe, callback_group=self.callback_group)

        # Keep track of the decompressed (sub->pub) topics we’ve already subscribed to
        self.subscribed_topics = set()

        # Check topics once on startup
        self._check_and_subscribe()

    def _load_config(self, filename: str) -> dict:
        """Load decompression config from YAML."""
        try:
            with open(filename, 'r') as file:
                data = yaml.safe_load(file)
                self.get_logger().debug(f"Decompression config loaded: {data}")
                return data
        except Exception as e:
            self.get_logger().error(f"Failed to load config from '{filename}': {e}")
            return {}

    def _check_and_subscribe(self):
        """Check current published topics and subscribe if they match the config (topic_regex, etc.)."""
        if 'decompression' not in self.config:
            self.get_logger().warn("No 'decompression' section in config, skipping subscription check.")
            return

        # Collect current topics
        all_topics = self.get_topic_names_and_types()
        topic_map = {}
        for (tname, ttypes) in all_topics:
            if ttypes:
                topic_map[tname] = ttypes[0]

        # For each item in the config, see if we can match a topic
        for item in self.config['decompression']:
            topic_regex = item.get('topic_regex', '')
            algorithm = item.get('algorithm', 'bz2')
            msg_type_str = item.get('msg_type', None)
            remove_suffix = item.get('remove_suffix', '')
            add_suffix = item.get('add_suffix', '')

            if not msg_type_str:
                self.get_logger().error("Config item missing 'msg_type', skipping.")
                continue

            # Convert e.g. 'nav_msgs/msg/OccupancyGrid' to a Python type
            try:
                msg_cls = get_message(msg_type_str)
            except Exception as e:
                self.get_logger().error(f"Could not load message class '{msg_type_str}': {e}")
                continue

            # Attempt to find a matching compressed topic
            for tname in topic_map.keys():
                if re.search(topic_regex, tname):
                    # Potential match
                    if remove_suffix and tname.endswith(remove_suffix):
                        base_name = tname[:-len(remove_suffix)]
                        out_topic = base_name + add_suffix
                    else:
                        if remove_suffix:
                            # We require a suffix but it doesn't match
                            self.get_logger().warn(
                                f"Topic '{tname}' does not end with '{remove_suffix}', skipping."
                            )
                            continue
                        out_topic = tname + add_suffix

                    if out_topic not in self.subscribed_topics:
                        # Build a best-effort QoS with depth=10
                        qos_profile = QoSProfile(
                            reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST,
                            depth=10
                        )

                        # Create publisher for the uncompressed data
                        pub = self.create_publisher(
                            msg_cls,
                            out_topic,
                            qos_profile=qos_profile,
                            callback_group=self.callback_group
                        )

                        # Subscribe to the compressed data
                        self.create_subscription(
                            UInt8MultiArray,
                            tname,
                            lambda msg, p=pub, mcls=msg_cls, algo=algorithm: self._callback(msg, p, mcls, algo),
                            qos_profile=qos_profile,
                            callback_group=self.callback_group
                        )

                        self.subscribed_topics.add(out_topic)
                        self.get_logger().info(
                            f"BEST_EFFORT sub: '{tname}' => pub: '{out_topic}' with algorithm='{algorithm}'"
                        )

    def _callback(self, msg: UInt8MultiArray, publisher, msg_cls, algorithm: str):
        """
        Callback that spawns a decompression job in a thread pool.
        No blocking in the main callback so we can handle messages concurrently.
        """
        self.thread_pool.submit(self._decompress_and_publish, msg, publisher, msg_cls, algorithm)

    def _decompress_and_publish(self, msg: UInt8MultiArray, publisher, msg_cls, algorithm: str):
        """
        Actually do the decompress + deserialize in a background thread to maximize concurrency.
        """
        comp_data = bytes(msg.data)
        csize = len(comp_data)
        csize_fmt = format_size(csize)

        # Optionally measure time
        start = time.time()
        try:
            raw_data = decompress_data(comp_data, algorithm)
        except Exception as e:
            self.get_logger().error(f"Decompression error: {e}")
            return

        dtime_ms = (time.time() - start) * 1000.0
        dsize = len(raw_data)
        dsize_fmt = format_size(dsize)

        self.get_logger().debug(
            f"[{publisher.topic_name}] comp. size={csize_fmt}, decomp. size={dsize_fmt}, time={dtime_ms:.1f} ms"
        )

        try:
            # Convert bytes to the final message
            deserialized = deserialize_message(raw_data, msg_cls)
            publisher.publish(deserialized)
        except Exception as e:
            self.get_logger().error(f"Deserialization error: {e}")


def main(args=None):
    rclpy.init(args=args)

    # Use a MultiThreadedExecutor so multiple callbacks can run concurrently
    executor = MultiThreadedExecutor(num_threads=8)
    node = BestEffortDecompressor()

    executor.add_node(node)

    try:
        node.get_logger().info("BEST_EFFORT Decompressor: spinning with multi-threaded executor.")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
