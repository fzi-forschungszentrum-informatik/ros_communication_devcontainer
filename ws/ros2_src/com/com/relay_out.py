#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from com.topic_resolution import resolve_topics
from com.qos import load_qos_config
from com.pub_sub_pair import PubSubPair

# Define QoS roles for relay_out
SUB_ROLE = "local_sub"
PUB_ROLE = "forward_pub"  # Publishes to /com/out

class OtaRelayOut(Node):
    """
    - Reads base topics from files
    - Resolves final (sub_topic, pub_topic, base_topic)
    - Creates PubSubPair with QoS for each (sub, pub) topic pair

    Example:
      User publishes to `/foo` locally.
      Relay subscribes to `/foo` (`local_sub`) and republishes to `/com/out/<host>/foo` (`forward_pub`).
    """

    def __init__(self):
        super().__init__('ota_relay_out')

        self.node_role = "relay_out"

        # 1) Read parameters
        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('host_name', '').value
        self.target_names = self.declare_parameter('target_names', rclpy.Parameter.Type.STRING_ARRAY).value
        self.locally_used_source_name = bool(self.declare_parameter('locally_used_source_name', False).value)
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value

        # 2) Resolve final (sub_topic, pub_topic, base_topic)
        try:
            triplets = resolve_topics(self.node_role, self.base_topic_files, [self.host_name], self.target_names, self.locally_used_source_name)
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"[ota_relay_out] resolve_topics failed: {e}")
            return

        # 3) Load QoS configuration
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        self.pairs = []

        # 4) Create PubSubPairs (relay local => forward to /com/out)
        for (sub_t, pub_t, base_t) in triplets:
            pair = PubSubPair(
                node=self,
                base_topic_name=base_t,
                sub_topic=sub_t,
                pub_topic=pub_t,
                sub_role=SUB_ROLE,
                pub_role=PUB_ROLE,
                qos_config=self.qos_config
            )
            if pair.is_valid:
                self.pairs.append(pair)

        self.get_logger().info(f"[ota_relay_out] Created {len(self.pairs)} relay_out pairs.")


def main(args=None):
    rclpy.init(args=args)
    node = OtaRelayOut()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
