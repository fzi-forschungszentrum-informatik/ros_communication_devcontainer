#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from com.qos import get_topic_type, build_role_qos

VALID_QOS_ROLES = {
    "ota_sub", "ota_pub", 
    "forward_sub", "forward_pub",
    "relay_sub", "relay_pub",
    "local_sub", "local_pub"
    # etc. define all you want
}

class PubSubPair:
    """
    A single subscription->publication pair for a single base topic.
    sub_role, pub_role must be in VALID_QOS_ROLES or we error out.
    No fallback if missing in the config => We'll still build a minimal QoS from the merged defaults or empty => standard ROS 2 defaults?
    """

    def __init__(self, node: Node,
                 base_topic_name: str,
                 sub_topic: str,
                 pub_topic: str,
                 sub_role: str,
                 pub_role: str,
                 qos_config: dict):

        self.node = node
        self.base_topic_name = base_topic_name
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.sub_role = sub_role
        self.pub_role = pub_role
        self.first_msg = True
        self.is_valid = False
        self.logger = node.get_logger()

        # 1) Validate roles
        if sub_role not in VALID_QOS_ROLES:
            self.logger.error(f"[PubSubPair] sub_role='{sub_role}' not recognized. Must be in {VALID_QOS_ROLES}")
            return
        if pub_role not in VALID_QOS_ROLES:
            self.logger.error(f"[PubSubPair] pub_role='{pub_role}' not recognized. Must be in {VALID_QOS_ROLES}")
            return

        # 2) Get type from QoS config
        msg_type_str = get_topic_type(qos_config, base_topic_name)
        if not msg_type_str:
            self.logger.error(f"[PubSubPair] No 'type' found for base_topic='{base_topic_name}' in QoS config => cannot build pair.")
            return

        msg_type = get_message(msg_type_str)
        if not msg_type:
            self.logger.error(f"[PubSubPair] Could not load message type '{msg_type_str}' for '{base_topic_name}'!")
            return

        # 3) Build QoS
        sub_qos = build_role_qos(self.logger, qos_config, base_topic_name, sub_role)
        pub_qos = build_role_qos(self.logger, qos_config, base_topic_name, pub_role)

        # 4) Create subscription & publisher
        self.publisher = node.create_publisher(msg_type, self.pub_topic, pub_qos)
        self.subscription = node.create_subscription(
            msg_type,
            self.sub_topic,
            self._callback,
            sub_qos
        )

        self.logger.info(f"[PubSubPair] sub='{self.sub_topic}'(role={sub_role}), "
                         f"pub='{self.pub_topic}'(role={pub_role}), base='{base_topic_name}'")
        self.is_valid = True

    def _callback(self, msg):
        if self.first_msg:
            self.logger.info(f"[PubSubPair] FIRST msg on sub='{self.sub_topic}' => pub='{self.pub_topic}', base='{self.base_topic_name}'")
            self.first_msg = False
        self.publisher.publish(msg)
