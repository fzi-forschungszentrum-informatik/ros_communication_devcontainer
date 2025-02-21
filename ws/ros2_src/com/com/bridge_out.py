#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from com.topic_resolution import resolve_topics
from com.qos import load_qos_config
from com.pub_sub_pair import PubSubPair

# bridging out => forward_sub => ota_pub
SUB_ROLE = "forward_sub"
PUB_ROLE = "ota_pub"

class OtaBridgeOut(Node):
    """
    node_role="bridge_out",
      => sub=/com/out/<host>/<topic>, pub=/ota/<host>/<topic>
      => QoS roles: sub_role=forward_sub, pub_role=ota_pub
    """

    def __init__(self):
        super().__init__('ota_bridge_out')

        self.node_role = self.declare_parameter('node_role','').value
        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('host_name','').value
        self.target_names = self.declare_parameter('target_names', rclpy.Parameter.Type.STRING_ARRAY).value
        self.qos_config_file = self.declare_parameter('qos_config_file','').value

        self.get_logger().info(f"[ota_bridge_out] node_role={self.node_role}, base_files={self.base_topic_files}, host={self.host_name}, targets={self.target_names}")
        self.get_logger().info(f"[ota_bridge_out] qos_config_file={self.qos_config_file}")

        try:
            triplets = resolve_topics(self.node_role, self.base_topic_files, self.host_name, self.target_names)
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"[ota_bridge_out] resolve_topics error: {e}")
            return

        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)
        self.pairs = []

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

        self.get_logger().info(f"[ota_bridge_out] Created {len(self.pairs)} bridging_out pairs.")


def main(args=None):
    rclpy.init(args=args)
    node = OtaBridgeOut()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
