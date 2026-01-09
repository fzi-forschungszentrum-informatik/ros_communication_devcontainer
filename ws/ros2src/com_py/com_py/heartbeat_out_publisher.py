#!/usr/bin/env python3

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
# 
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
# 
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.                                                
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Martin Gontscharow <gontscharow@fzi.de>
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from com_msgs.msg import Heartbeat
from com_py.qos import load_qos_config, get_topic_qos
import traceback

class HeartbeatOutPublisher(Node):
    def __init__(self):
        super().__init__("heartbeat_out_publisher")
        
        # QoS configuration (mirrors pattern used in other com_py nodes)
        self.qos_config_file = self.declare_parameter("qos_config_file", "").value
        self.pub_role = "heartbeat_pub"
        self.qos_config = load_qos_config(self.get_logger(), self.qos_config_file)

        # Declare the parameter "topic_names" with a default empty list.
        # If a single string is provided, we wrap it in a list.
        self.declare_parameter('topic_names', rclpy.Parameter.Type.STRING_ARRAY)
        topic_list = self.get_parameter('topic_names').value

        if isinstance(topic_list, str):
            topic_list = [topic_list]

        # Use a different name for our dictionary to avoid conflict with Node._publishers.
        # Build per-topic QoS profiles using the configured QoS roles / YAML.
        self.my_publishers = {}
        for topic in topic_list:
            pub_qos = get_topic_qos(self.get_logger(), self.qos_config, topic, self.pub_role)
            self.my_publishers[topic] = self.create_publisher(
                Heartbeat,
                topic,
                qos_profile=pub_qos,
            )
        
        # Declare a parameter for the publish frequency (Hz) with default 10 Hz.
        self.declare_parameter('hz', 10.0)
        hz = self.get_parameter('hz').get_parameter_value().double_value
        if hz <= 0.0:
            self.get_logger().warn(f"Parameter 'hz' must be > 0.0, got {hz}. Falling back to 10.0 Hz.")
            hz = 10.0

        # Create a timer to publish heartbeats at the configured rate.
        period = 1.0 / hz
        self.timer = self.create_timer(period, self.publish_heartbeat)
        self.seq = 0

    def publish_heartbeat(self):
        heartbeat_msg = Heartbeat()
        heartbeat_msg.header.stamp = self.get_clock().now().to_msg()
        heartbeat_msg.header.frame_id = "map"

        # Set the sequence number field in the Heartbeat message.
        heartbeat_msg.seq = self.seq

        # Publish the same heartbeat message on all topics in our local publisher dictionary.
        for topic, pub in self.my_publishers.items():
            self.get_logger().info(
                f"Publishing Heartbeat on {topic}: seq={heartbeat_msg.seq}, stamp={heartbeat_msg.header.stamp.sec}.{heartbeat_msg.header.stamp.nanosec}"
            )
            pub.publish(heartbeat_msg)

        self.seq += 1

def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatOutPublisher()
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
