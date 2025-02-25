#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from com.topic_resolution import resolve_topics
from com.qos import load_qos_config
from com.pub_sub_pair import PubSubPair

# We define which QoS roles we want for sub->pub in relay_in
# e.g. sub_role='forward_sub', pub_role='local_pub', or 
# sub_role='relay_sub', pub_role='local_pub' 
SUB_ROLE = "forward_sub"
PUB_ROLE = "local_pub"

class OtaRelayIn(Node):
    """
    => will call resolve_topics("relay_in", base_topic_files, host_name, target_names)
       for each base topic => sub=/com/in/<host>/<topic>, pub=/<topic>
       create QoS roles sub=forward_sub, pub=local_pub
    """

    def __init__(self):
        super().__init__('ota_relay_in')

        self.node_role = "relay_in"

        # 1) read parameters
        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('host_name', '').value
        self.source_names = self.declare_parameter('source_names', rclpy.Parameter.Type.STRING_ARRAY).value
        self.explicitly_adressed = bool(self.declare_parameter('explicitly_adressed', False).value)
        self.locally_used_source_name = bool(self.declare_parameter('locally_used_source_name', False).value)
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value

        self.target_names = [self.host_name] if self.explicitly_adressed else []

        # 2) resolve final (sub_topic, pub_topic, base_topic)
        try:
            triplets = resolve_topics(self.node_role, self.base_topic_files, self.source_names, self.target_names, self.locally_used_source_name, False)
        except (ValueError, FileNotFoundError) as e:
            self.get_logger().error(f"[ota_relay_in] resolve_topics failed: {e}")
            return

        # 3) load QoS config
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

        self.get_logger().info(f"[ota_relay_in] Created {len(self.pairs)} relay_in pairs.")


def main(args=None):
    rclpy.init(args=args)
    node = OtaRelayIn()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
