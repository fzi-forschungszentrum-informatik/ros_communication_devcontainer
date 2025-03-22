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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

class TopicHzMonitorNode(Node):
    def __init__(self):
        super().__init__('hz_monitor_node')

        # 1) Declare parameters (topic required)
        self.declare_parameter('topic', '')
        self.declare_parameter('use_reliable_qos', True)
        self.declare_parameter('print_period_s', 1)

        # 2) Read param values
        self.topic = self.get_parameter('topic').value
        self.use_reliable_qos = self.get_parameter('use_reliable_qos').value
        self.print_period_s = self.get_parameter('print_period_s').value

        # 3) Validate
        if not self.topic:
            self.get_logger().error(
                "Parameter 'topic' is REQUIRED and must not be empty.\n"
                "Example usage:\n"
                "  ros2 run your_package hz_monitor --ros-args -p topic:=/some_topic"
            )
            raise RuntimeError("Missing required parameter 'topic'.")

        # 4) Construct QoS
        if self.use_reliable_qos:
            self.get_logger().info("Using RELIABLE QoS")
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else:
            self.get_logger().info("Using BEST_EFFORT QoS")
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

        # 5) List all discovered topics
        all_topics_list = self.get_topic_names_and_types()
        self.get_logger().debug("==== DEBUG: Listing all discovered topics ====")
        for (name, types) in all_topics_list:
            self.get_logger().debug(f"  Topic '{name}' --> types {types}")

        # Convert that list into a dict for easier lookup
        all_topics_dict = dict(all_topics_list)

        # Now check if our topic is in the map
        if self.topic not in all_topics_dict:
            # The topic does not exist (yet) in the node's graph
            self.get_logger().error(
                f"Topic '{self.topic}' was not found in the ROS graph. Exiting."
            )
            raise RuntimeError("Topic not found in ROS graph.")

        # 6) Attempt to load one of the discovered types for this topic
        topic_types = all_topics_dict[self.topic]  # e.g. ["nav_msgs/msg/OccupancyGrid", "some_msgs/msg/AnotherType"]
        subscription_created = False
        for type_name in topic_types:
            msg_class = self.load_message_class(type_name)
            if msg_class is not None:
                # Create a subscription using that type
                self.subscription_ = self.create_subscription(
                    msg_class,
                    self.topic,
                    self.generic_callback,
                    qos_profile
                )
                self.get_logger().info(
                    f"Monitoring frequency of topic '{self.topic}' with type '{type_name}'"
                )
                subscription_created = True
                break

        if not subscription_created:
            all_type_str = ', '.join(topic_types)
            self.get_logger().error(
                f"Topic '{self.topic}' has types [{all_type_str}], but none could be loaded dynamically. Exiting."
            )
            raise RuntimeError("No recognized/loaded type for the requested topic.")

        # 7) Set up for frequency measurement
        self.msg_count = 0
        self.last_time = self.get_clock().now()

        self.timer_ = self.create_timer(float(self.print_period_s), self.timer_callback)

    def load_message_class(self, ros1_style_path: str):
        """
        Dynamically load a Python message class for 'nav_msgs/msg/OccupancyGrid', etc.
        Returns None if loading fails.
        """
        try:
            return get_message(ros1_style_path)
        except Exception as e:
            self.get_logger().warn(
                f"Could not load message class '{ros1_style_path}': {e}. Trying next type if available."
            )
            return None

    def generic_callback(self, msg):
        self.msg_count += 1

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed_ns = (current_time - self.last_time).nanoseconds
        dt = elapsed_ns / 1e9
        frequency = 0.0
        if dt > 0.0:
            frequency = self.msg_count / dt

        self.get_logger().info(
            f"Received {self.msg_count} messages in {dt:.2f} s => frequency: {frequency:.2f} Hz"
        )
        self.last_time = current_time
        self.msg_count = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TopicHzMonitorNode()
        rclpy.spin(node)
    except Exception as ex:
        import traceback
        print(f"Exception in {__file__}: {ex}")
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
