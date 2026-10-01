#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

import re
from threading import Lock

import rclpy
import yaml
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from com_msgs.msg import OtaStamped
from com_py.qos import get_topic_qos, load_qos_config
from com_py.status_overview_core import is_sequence_epoch_reset


class UniversalOtaUnwrapperNode(Node):
    """
    Unwrap com_msgs/msg/OtaStamped back into the original typed ROS 2 messages.

    Configuration is read from YAML via the ``config_file`` parameter:

      ota_unwrapper:
        - topic_regex: "^/foo/ota_stamped$"
          remove_suffix: "/ota_stamped"   # optional
          add_suffix: ""                  # optional
    """

    def __init__(self):
        super().__init__('universal_ota_unwrapper')

        self.declare_parameter('config_file', 'ota_unwrapper_config.yaml')
        config_file = self.get_parameter('config_file').value

        self.declare_parameter('default_suffix', '/ota_stamped')
        self.default_suffix = self.get_parameter('default_suffix').value

        self.qos_config_file = self.declare_parameter('qos_config_file', '').value
        # This is a local processing step after the actual OTA bridge.
        self.sub_role = 'ota_wrap_sub'
        self.pub_role = 'ota_wrap_pub'
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        self.config = self.load_config(config_file)
        self.subscribe_lock = Lock()
        self.cache_lock = Lock()
        self.subscribed_topics = set()
        # Keep our own references without shadowing rclpy.Node._subscriptions.
        # Node.create_subscription() owns and updates that private list itself.
        self._owned_subscriptions = []
        self._typed_pub_cache = {}
        self._last_seq = {}

        self.get_logger().info(
            f"[universal_ota_unwrapper] Starting with config_file='{config_file}', "
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
            if 'ota_unwrapper' not in self.config:
                self.get_logger().warning("[universal_ota_unwrapper] No 'ota_unwrapper' section in config.")
                return

            wrapped_topics = {}
            for topic_name, topic_types in self.get_topic_names_and_types():
                if 'com_msgs/msg/OtaStamped' in topic_types:
                    wrapped_topics[topic_name] = True

            for item in self.config['ota_unwrapper']:
                topic_pattern = item.get('topic_regex', '')
                remove_suffix = item.get('remove_suffix', self.default_suffix)
                add_suffix = item.get('add_suffix', '')
                rx = re.compile(topic_pattern)

                for topic_name in wrapped_topics.keys():
                    if not rx.search(topic_name):
                        continue
                    if topic_name in self.subscribed_topics:
                        continue

                    out_topic = self.build_output_topic(topic_name, remove_suffix, add_suffix)
                    sub_qos = get_topic_qos(self.get_logger(), self.qos_config, topic_name, self.sub_role)

                    subscription = self.create_subscription(
                        OtaStamped,
                        topic_name,
                        lambda msg,
                               wrapped_topic=topic_name,
                               out_topic=out_topic: self.unwrapper_callback(
                                   msg, wrapped_topic, out_topic
                               ),
                        qos_profile=sub_qos,
                    )
                    self._owned_subscriptions.append(subscription)
                    self.subscribed_topics.add(topic_name)

                    self.get_logger().info(
                        f"[universal_ota_unwrapper] Subscribed to '{topic_name}' -> publishing '{out_topic}'"
                    )

    def build_output_topic(self, original_topic: str, remove_suffix: str, add_suffix: str) -> str:
        if remove_suffix and original_topic.endswith(remove_suffix):
            base_topic = original_topic[:-len(remove_suffix)]
        else:
            base_topic = original_topic
        return base_topic + add_suffix

    def get_message_class(self, ros_type: str):
        try:
            return get_message(ros_type)
        except Exception as exc:
            self.get_logger().error(f"Could not load message class '{ros_type}': {exc}")
            return None

    def _get_or_create_typed_publisher(self, wrapped_topic: str, out_topic: str, msg_type: str):
        with self.cache_lock:
            cached = self._typed_pub_cache.get(wrapped_topic)
            if cached is not None:
                return cached['pub'], cached['typed_cls']

            typed_cls = self.get_message_class(msg_type)
            if typed_cls is None:
                return None, None

            pub_qos = get_topic_qos(self.get_logger(), self.qos_config, out_topic, self.pub_role)
            publisher = self.create_publisher(typed_cls, out_topic, qos_profile=pub_qos)
            self._typed_pub_cache[wrapped_topic] = {
                'pub': publisher,
                'typed_cls': typed_cls,
                'msg_type': msg_type,
                'out_topic': out_topic,
            }
            return publisher, typed_cls

    def _check_sequence(self, wrapped_topic: str, seq: int):
        """Say which of the three things a sequence break is.

        A backward jump is either reordering or the sending peer restarting, and
        this pane log was the last place still calling both "reordering". Three
        consumers now agree that a jump larger than the transport can reorder is
        a new epoch -- the live accounting, the transit join and the offline bag
        join -- and a breadcrumb that disagrees with them costs exactly the time
        it takes to trust it.
        """
        last_seq = self._last_seq.get(wrapped_topic)
        self._last_seq[wrapped_topic] = seq
        if last_seq is None:
            return
        expected = last_seq + 1
        if seq == expected:
            return
        if is_sequence_epoch_reset(seq, expected):
            self.get_logger().warning(
                f"[{wrapped_topic}] Sending peer restarted: sequence begins again at {seq} "
                f"after {last_seq}. Counts and delays are per epoch from here."
            )
        elif seq < last_seq:
            self.get_logger().warning(
                f"[{wrapped_topic}] Sequence reordering detected: received {seq} after {last_seq}"
            )
        else:
            self.get_logger().warning(
                f"[{wrapped_topic}] Sequence jump detected: expected {expected}, received {seq}"
            )

    def unwrapper_callback(self, wrapped_msg: OtaStamped, wrapped_topic: str, out_topic: str):
        try:
            msg_type = (wrapped_msg.msg_type or '').strip()
            if not msg_type:
                self.get_logger().error(f"[{wrapped_topic}] Missing msg_type in OtaStamped message.")
                return

            payload = bytes(wrapped_msg.serialized_msg)
            if not payload:
                self.get_logger().error(f"[{wrapped_topic}] Received empty serialized_msg payload.")
                return

            publisher, typed_cls = self._get_or_create_typed_publisher(wrapped_topic, out_topic, msg_type)
            if publisher is None or typed_cls is None:
                self.get_logger().error(
                    f"[{wrapped_topic}] Cannot create publisher for msg_type='{msg_type}'."
                )
                return

            self._check_sequence(wrapped_topic, wrapped_msg.seq)
            out_msg = deserialize_message(payload, typed_cls)
            publisher.publish(out_msg)
        except Exception as exc:
            self.get_logger().error(f"[{wrapped_topic}] OTA unwrapping error: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = UniversalOtaUnwrapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
