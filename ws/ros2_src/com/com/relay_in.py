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
    Example usage:
      ros2 run bridging relay_in --ros-args
        -p node_role:="relay_in"
        -p base_topic_files:="[/path/to/some.txt]"
        -p host_name:="shuttle_ella"
        -p target_names:="[]"
        -p qos_config_file:="/path/to/qos.yaml"

    => will call resolve_topics("relay_in", base_topic_files, host_name, target_names)
       for each base topic => sub=/com/in/<host>/<topic>, pub=/<topic>
       create QoS roles sub=forward_sub, pub=local_pub
    """

    def __init__(self):
        super().__init__('ota_relay_in')

        # 1) read parameters
        self.node_role = self.declare_parameter('node_role', '').value
        self.base_topic_files = self.declare_parameter('base_topic_files', rclpy.Parameter.Type.STRING_ARRAY).value
        self.host_name = self.declare_parameter('host_name', '').value
        self.target_names = self.declare_parameter('target_names', rclpy.Parameter.Type.STRING_ARRAY).value
        self.qos_config_file = self.declare_parameter('qos_config_file', '').value

        self.get_logger().info(f"[ota_relay_in] node_role={self.node_role}, base_files={self.base_topic_files}, host={self.host_name}, targets={self.target_names}")
        self.get_logger().info(f"[ota_relay_in] qos_config_file={self.qos_config_file}")

        # 2) resolve final (sub_topic, pub_topic, base_topic)
        try:
            triplets = resolve_topics(self.node_role, self.base_topic_files, self.host_name, self.target_names)
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
