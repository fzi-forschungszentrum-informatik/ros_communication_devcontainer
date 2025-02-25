#!/usr/bin/env python3
# bridging/bridge_in.py

import rclpy
from rclpy.node import Node
from com.topic_resolution import resolve_topics
from com.qos import load_qos_config
from com.pub_sub_pair import PubSubPair

# choose roles for bridging in => sub=ota_sub, pub=forward_pub
SUB_ROLE = "ota_sub"
PUB_ROLE = "forward_pub"

class OtaBridgeIn(Node):
    """
    Typically: node_role="bridge_in",
      => sub=/ota/<host>/<topic>, pub=/com/in/<host>/<topic>
      => QoS roles: sub_role=ota_sub, pub_role=forward_pub
    """

    def __init__(self):
        super().__init__('ota_bridge_in')

        self.node_role = "bridge_in"

        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('host_name','').value
        self.source_names = self.declare_parameter('source_names', rclpy.Parameter.Type.STRING_ARRAY).value
        self.explicitly_adressed = bool(self.declare_parameter('explicitly_adressed', False).value)
        self.qos_config_file = self.declare_parameter('qos_config_file','').value

        self.target_names = [self.host_name] if self.explicitly_adressed else []

        try:
            triplets = resolve_topics(self.node_role, self.base_topic_files, self.source_names, self.target_names)
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"[ota_bridge_in] resolve_topics error: {e}")
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

        self.get_logger().info(f"[ota_bridge_in] Created {len(self.pairs)} bridging_in pairs.")


def main(args=None):
    rclpy.init(args=args)
    node = OtaBridgeIn()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
