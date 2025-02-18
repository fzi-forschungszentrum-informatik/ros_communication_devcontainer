#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__("heartbeat_publisher")

        # Parameter lesen, Standardwert ist "/heartbeat_topic"
        self.declare_parameter("topic_names", "/heartbeat_topic")
        topic_list = self.get_parameter("topic_names").get_parameter_value().string_array_value

        # Falls ein einzelner String angegeben wurde, in eine Liste umwandeln
        if isinstance(topic_list, str):
            topic_list = [topic_list]

        self.publishers = {topic: self.create_publisher(PoseStamped, topic, 10) for topic in topic_list}
        self.timer = self.create_timer(1.0, self.publish_heartbeat)  # 1 Hz
        self.seq = 0

    def publish_heartbeat(self):
        heartbeat_msg = PoseStamped()
        heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        heartbeat_msg.header.frame_id = "heartbeat_frame"
        heartbeat_msg.header.set__seq(self.seq)  # Manuelle Sequenznummer setzen (optional)

        for topic, pub in self.publishers.items():
            self.get_logger().info(f"Publishing Heartbeat on {topic}: seq={self.seq}, stamp={heartbeat_msg.header.stamp.sec}.{heartbeat_msg.header.stamp.nanosec}")
            pub.publish(heartbeat_msg)

        self.seq += 1  # Sequenznummer erhöhen


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
