#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

import array
import re
from threading import Lock

import rclpy
import yaml
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from com_msgs.msg import OtaStamped
from com_py.qos import get_topic_qos, load_qos_config


class UniversalOtaWrapperNode(Node):
    """
    Wrap arbitrary ROS 2 messages into com_msgs/msg/OtaStamped.

    Configuration is read from YAML via the ``config_file`` parameter:

      ota_wrapper:
        - topic_regex: "^/foo$"
          add_suffix: "/ota_stamped"   # optional
    """

    def __init__(self):
        super().__init__('universal_ota_wrapper')

        self.declare_parameter('config_file', 'ota_wrapper_config.yaml')
        config_file = self.get_parameter('config_file').value

        self.declare_parameter('default_suffix', '/ota_stamped')
        self.default_suffix = self.get_parameter('default_suffix').value

        self.qos_config_file = self.declare_parameter('qos_config_file', '').value
        # This is a local processing step before the actual OTA bridge.
        self.sub_role = 'ota_wrap_sub'
        self.pub_role = 'ota_wrap_pub'
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        self.subscribe_lock = Lock()
        self.sequence_lock = Lock()
        self.subscribed_topics = set()
        self.sequence_by_topic = {}
        # Keep our own references without shadowing rclpy.Node._subscriptions.
        # Node.create_subscription() owns and updates that private list itself.
        self._owned_subscriptions = []

        self.config = self.load_config(config_file)

        self.get_logger().info(
            f"[universal_ota_wrapper] Starting with config_file='{config_file}', "
            f"default_suffix='{self.default_suffix}'"
        )

        self.timer = self.create_timer(5.0, self.check_and_subscribe)
        self.check_and_subscribe()

    def load_config(self, filename: str) -> dict:
        try:
            with open(filename, 'r', encoding='utf-8') as file:
                return yaml.safe_load(file) or {}
        except Exception as exc:
            self.get_logger().error(f"Failed to load config from '{filename}': {exc}")
            return {}

    def check_and_subscribe(self):
        with self.subscribe_lock:
            if 'ota_wrapper' not in self.config:
                self.get_logger().warning("[universal_ota_wrapper] No 'ota_wrapper' section in config.")
                return

            topic_map = {}
            for topic_name, topic_types in self.get_topic_names_and_types():
                if topic_types:
                    topic_map[topic_name] = topic_types[0]

            for item in self.config['ota_wrapper']:
                topic_pattern = item.get('topic_regex', '')
                add_suffix = item.get('add_suffix', self.default_suffix)
                rx = re.compile(topic_pattern)

                for topic_name, type_name in topic_map.items():
                    if not rx.search(topic_name):
                        continue

                    out_topic = topic_name + add_suffix
                    if out_topic in self.subscribed_topics:
                        continue

                    msg_class = self.get_message_class(type_name)
                    if msg_class is None:
                        continue

                    sub_qos = get_topic_qos(self.get_logger(), self.qos_config, topic_name, self.sub_role)
                    pub_qos = get_topic_qos(self.get_logger(), self.qos_config, out_topic, self.pub_role)
                    publisher = self.create_publisher(OtaStamped, out_topic, qos_profile=pub_qos)

                    subscription = self.create_subscription(
                        msg_class,
                        topic_name,
                        lambda msg,
                               publisher=publisher,
                               source_topic=topic_name,
                               msg_type_str=type_name: self.wrapper_callback(
                                   msg, publisher, source_topic, msg_type_str
                               ),
                        qos_profile=sub_qos,
                    )
                    self._owned_subscriptions.append(subscription)
                    self.subscribed_topics.add(out_topic)

                    self.get_logger().info(
                        f"[universal_ota_wrapper] Subscribed to '{topic_name}' "
                        f"(type={type_name}) -> publishing '{out_topic}'"
                    )

    def get_message_class(self, ros_type: str):
        try:
            return get_message(ros_type)
        except Exception as exc:
            self.get_logger().error(f"Could not load message class '{ros_type}': {exc}")
            return None

    def _next_sequence(self, topic_name: str) -> int:
        with self.sequence_lock:
            seq = self.sequence_by_topic.get(topic_name, 0)
            self.sequence_by_topic[topic_name] = seq + 1
            return seq

    def wrapper_callback(self, msg, publisher, source_topic: str, msg_type_str: str):
        try:
            header = getattr(msg, 'header', None)

            serialized = serialize_message(msg)

            out_msg = OtaStamped()
            out_msg.header.stamp = self.get_clock().now().to_msg()
            if header is not None:
                out_msg.header.frame_id = header.frame_id
            out_msg.seq = self._next_sequence(source_topic)
            out_msg.msg_type = msg_type_str
            # array('B', ...) hits the generated setter's fast path and stores the
            # buffer as-is. A list would allocate one Python int per payload byte,
            # inside the interval that `header.stamp` above is meant to measure --
            # and rule out wrapping anything large (a ~100 kB encoded video packet
            # at 10 Hz is 1M allocations per second).
            out_msg.serialized_msg = array.array('B', serialized)

            publisher.publish(out_msg)
        except Exception as exc:
            self.get_logger().error(f"[{source_topic}] OTA wrapping error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = UniversalOtaWrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
