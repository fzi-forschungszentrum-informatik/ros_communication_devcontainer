// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
// 
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
// 
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.                                                
// -- END LICENSE BLOCK ------------------------------------------------
//
// ---------------------------------------------------------------------
// !\file
//
// \author  Martin Gontscharow <gontscharow@fzi.de>
// \date    2024-11-13
//
//
// ---------------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <unordered_set>
#include <string>
#include <chrono>

class TopicHzMonitorNode : public rclcpp::Node
{
public:
  TopicHzMonitorNode()
  : Node("hz_monitor_node"), msg_count_(0)
  {
    // 1) Declare parameters (topic has no default -> required)
    this->declare_parameter<std::string>("topic", "");
    this->declare_parameter<bool>("use_reliable_qos", true);
    this->declare_parameter<int>("print_period_s", 1);

    // 2) Retrieve parameters
    std::string topic         = this->get_parameter("topic").as_string();
    bool use_reliable_qos     = this->get_parameter("use_reliable_qos").as_bool();
    int print_period_s        = this->get_parameter("print_period_s").as_int();

    // 3) Validate that topic is non-empty
    if (topic.empty()) {
      RCLCPP_ERROR(get_logger(),
        "'topic' parameter is REQUIRED and must not be empty.\n"
        "Example usage:\n"
        "  ros2 run your_package hz_monitor --ros-args -p topic:=/some_topic");
      throw std::runtime_error("Missing required parameter 'topic'.");
    }

    // 4) Construct QoS
    rclcpp::QoS qos(10);
    if (use_reliable_qos) {
      qos.reliable();
      RCLCPP_INFO(get_logger(), "Using RELIABLE QoS");
    } else {
      qos.best_effort();
      RCLCPP_INFO(get_logger(), "Using BEST_EFFORT QoS");
    }

    // 5) Look up the topic in the ROS graph to discover its type(s)
    auto all_topics = this->get_topic_names_and_types();
    auto it = all_topics.find(topic);
    if (it == all_topics.end()) {
      // The requested topic does not exist at node startup
      RCLCPP_ERROR(get_logger(),
                   "Topic '%s' was not found in the ROS graph. Exiting.", topic.c_str());
      throw std::runtime_error("Topic not found in ROS graph.");
    }

    // Typically, there's either one type or multiple possible types for bridging.
    // We'll see if there's a recognized type among them:
    bool subscription_created = false;
    for (auto & type_name : it->second) {
      // Check if recognized
      if (type_name == "nav_msgs/msg/OccupancyGrid") {
        sub_occupancy_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          topic, qos,
          std::bind(&TopicHzMonitorNode::occupancyCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(get_logger(),
          "Monitoring frequency of OccupancyGrid on topic '%s'", topic.c_str());
        subscription_created = true;
        break;

      } else if (type_name == "visualization_msgs/msg/MarkerArray") {
        sub_marker_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
          topic, qos,
          std::bind(&TopicHzMonitorNode::markerArrayCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(get_logger(),
          "Monitoring frequency of MarkerArray on topic '%s'", topic.c_str());
        subscription_created = true;
        break;
      }
      // Add more else if(...) for other message types if needed
    }

    if (!subscription_created) {
      // The topic was found, but had no recognized message type
      std::string all_types_str;
      for (auto & t : it->second) {
        all_types_str += t + " ";
      }
      RCLCPP_ERROR(get_logger(),
        "Topic '%s' has types [%s], but none are recognized (OccupancyGrid / MarkerArray). Exiting.",
        topic.c_str(), all_types_str.c_str());
      throw std::runtime_error("No recognized type for the requested topic.");
    }

    // 6) Create a timer to report frequency
    timer_ = this->create_wall_timer(
      std::chrono::seconds(print_period_s),
      std::bind(&TopicHzMonitorNode::timerCallback, this)
    );

    last_time_ = this->now();
  }

private:
  void occupancyCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr /*msg*/)
  {
    msg_count_++;
  }

  void markerArrayCallback(const visualization_msgs::msg::MarkerArray::SharedPtr /*msg*/)
  {
    msg_count_++;
  }

  void timerCallback()
  {
    auto current_time = this->now();
    auto elapsed = current_time - last_time_;
    double dt = elapsed.seconds(); // in seconds

    double frequency = 0.0;
    if (dt > 1e-8) {
      frequency = static_cast<double>(msg_count_) / dt;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Received %d messages in %.2f s => frequency: %.2f Hz",
      msg_count_, dt, frequency
    );

    // Reset counters
    last_time_ = current_time;
    msg_count_ = 0;
  }

  // We only store a subscription for one recognized type
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_marker_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_time_;
  int msg_count_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    // If the topic is not found or no recognized type is found, an exception is thrown
    rclcpp::spin(std::make_shared<TopicHzMonitorNode>());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in TopicHzMonitorNode: %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
