#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import traceback

class HeartbeatPublisher(Node):
    def __init__(self):
        super().__init__("heartbeat_publisher")
        
        # Declare the parameter "topic_names" with a default empty list.
        # If a single string is provided, we wrap it in a list.
        self.declare_parameter('topic_names', rclpy.Parameter.Type.STRING_ARRAY)
        topic_list = self.get_parameter('topic_names').value

        if isinstance(topic_list, str):
            topic_list = [topic_list]

        # Use a different name for our dictionary to avoid conflict with Node._publishers
        self.my_publishers = {topic: self.create_publisher(PoseStamped, topic, 10)
                              for topic in topic_list}
        
        # Create a timer to publish heartbeats at 1 Hz.
        self.timer = self.create_timer(1.0, self.publish_heartbeat)
        self.seq = 0

    def publish_heartbeat(self):
        heartbeat_msg = PoseStamped()
        heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        heartbeat_msg.header.frame_id = "heartbeat_frame"

        # In ROS2, the Header message no longer contains a 'seq' field.
        # We attempt to set it, and if not present, log a debug message.
        try:
            heartbeat_msg.header.seq = self.seq
        except AttributeError:
            self.get_logger().debug("Header does not support 'seq'; skipping assignment.")

        # Publish the same heartbeat message on all topics in our local publisher dictionary.
        for topic, pub in self.my_publishers.items():
            self.get_logger().info(
                f"Publishing Heartbeat on {topic}: seq={self.seq}, stamp={heartbeat_msg.header.stamp.sec}.{heartbeat_msg.header.stamp.nanosec}"
            )
            pub.publish(heartbeat_msg)

        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error("Exception during spin:\n" + traceback.format_exc())
    finally:
        try:
            node.destroy_node()
        except Exception:
            node.get_logger().error("Error during node destruction:\n" + traceback.format_exc())
        rclpy.shutdown()

if __name__ == "__main__":
    main()
