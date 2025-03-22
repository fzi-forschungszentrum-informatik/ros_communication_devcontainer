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
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <com_msgs/msg/compressed_data.hpp>
#include <yaml-cpp/yaml.h>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/copy.hpp>

#include <regex>
#include <string>
#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <chrono>

static std::string format_size(size_t size) {
  if (size < 1024) {
    return std::to_string(size) + " B";
  } else if (size < 1048576) {
    return std::to_string(size / 1024.0) + " KB";
  } else if (size < 1073741824) {
    return std::to_string(size / 1048576.0) + " MB";
  }
  return std::to_string(size / 1073741824.0) + " GB";
}

class UniversalCompressorNode : public rclcpp::Node {
public:
  UniversalCompressorNode()
  : Node("universal_compressor_node")
  {
    // Load config param
    this->declare_parameter<std::string>("config_file", "compression_config.yaml");
    std::string config_file = this->get_parameter("config_file").as_string();
    RCLCPP_INFO(get_logger(), "Loading config from: %s", config_file.c_str());

    // Load YAML
    try {
      config_ = YAML::LoadFile(config_file);
    } catch(const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Failed to load YAML config: %s", e.what());
      return;
    }

    // Optionally re-check every few seconds to catch new topics
    // or changes in the graph. If you only want to do it once at startup,
    // call checkConfigAndSubscribe() just once.
    timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&UniversalCompressorNode::checkConfigAndSubscribe, this));

    // Or if you only want to do it once:
    // checkConfigAndSubscribe();
  }

private:
  void checkConfigAndSubscribe()
  {
    std::lock_guard<std::mutex> lock(subscribe_mutex_);

    if (!config_["compression"]) {
      RCLCPP_WARN(get_logger(), "No 'compression' node in YAML config.");
      return;
    }

    // 1) Retrieve the full list of topics + types
    auto all_topics = this->get_topic_names_and_types();

    // 2) For each config entry, do the regex matching
    for (auto && entry : config_["compression"]) {
      std::string topic_regex = entry["topic_regex"].as<std::string>();
      std::string algorithm   = entry["algorithm"]   ? entry["algorithm"].as<std::string>() : "bz2";
      std::string suffix      = entry["add_suffix"]  ? entry["add_suffix"].as<std::string>() : "_compressed";

      std::regex rx(topic_regex);

      // 3) Walk all discovered topics, see if they match
      for (const auto & [topic_name, type_vec] : all_topics) {
        if (!std::regex_search(topic_name, rx)) {
          continue; // skip if doesn't match
        }

        // If we've already subscribed to this topic with this config, skip
        // (especially if the user calls checkConfigAndSubscribe() repeatedly)
        if (subscribed_topics_.count(topic_name) > 0) {
          continue;
        }

        // 4) See if the discovered message type is recognized
        //    (some topics have multiple types, e.g. for ros1_bridge or multiple type supports)
        //    We'll just check if it includes "nav_msgs/msg/OccupancyGrid" or "MarkerArray", etc.
        for (const auto & t : type_vec) {
          // Create typed subscription if recognized
          if (t == "nav_msgs/msg/OccupancyGrid") {
            createOccupancyGridSubscription(topic_name, suffix, algorithm);
            subscribed_topics_.insert(topic_name);
            break; // Done with this topic

          } else if (t == "visualization_msgs/msg/MarkerArray") {
            createMarkerArraySubscription(topic_name, suffix, algorithm);
            subscribed_topics_.insert(topic_name);
            break; // Done with this topic
          }
          // Add more else if(...) for any other message types you want to support
        }
      }
    }
  }

  void createOccupancyGridSubscription(const std::string & topic_name,
                                       const std::string & suffix,
                                       const std::string & algorithm)
  {
    // Publisher
    auto pub_topic = topic_name + suffix;
    auto pub = this->create_publisher<com_msgs::msg::CompressedData>(pub_topic, 10);

    // Subscription
    auto sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      topic_name,
      10,
      [this, pub, algorithm, topic_name](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->typedCallback<nav_msgs::msg::OccupancyGrid>(msg, pub, algorithm, topic_name);
      }
    );

    subscriptions_.push_back(sub);
    RCLCPP_INFO(get_logger(),
      "Regex matched OccupancyGrid topic '%s'. Publishing on '%s' with algorithm '%s'",
      topic_name.c_str(), pub_topic.c_str(), algorithm.c_str());
  }

  void createMarkerArraySubscription(const std::string & topic_name,
                                     const std::string & suffix,
                                     const std::string & algorithm)
  {
    auto pub_topic = topic_name + suffix;
    auto pub = this->create_publisher<com_msgs::msg::CompressedData>(pub_topic, 10);

    auto sub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
      topic_name,
      10,
      [this, pub, algorithm, topic_name](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        this->typedCallback<visualization_msgs::msg::MarkerArray>(msg, pub, algorithm, topic_name);
      }
    );

    subscriptions_.push_back(sub);
    RCLCPP_INFO(get_logger(),
      "Regex matched MarkerArray topic '%s'. Publishing on '%s' with algorithm '%s'",
      topic_name.c_str(), pub_topic.c_str(), algorithm.c_str());
  }

  // Generic typed callback for any known message type
  template<typename MsgT>
  void typedCallback(
    const typename MsgT::SharedPtr & msg,
    const rclcpp::Publisher<com_msgs::msg::CompressedData>::SharedPtr & pub,
    const std::string & algorithm,
    const std::string & original_topic)
  {
    try {
      // 1) Serialize
      rclcpp::SerializedMessage serialized_msg;
      rclcpp::Serialization<MsgT> serializer;
      serializer.serialize_message(msg.get(), &serialized_msg);

      const uint8_t* raw_data = serialized_msg.get_rcl_serialized_message().buffer;
      size_t raw_size = serialized_msg.size();

      auto start = std::chrono::high_resolution_clock::now();

      // 2) Compress
      std::string compressed_str = compressData(raw_data, raw_size, algorithm);

      auto end = std::chrono::high_resolution_clock::now();
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      // 3) Publish
      com_msgs::msg::CompressedData out_msg;
      out_msg.header.stamp = this->get_clock()->now();
      out_msg.data.assign(compressed_str.begin(), compressed_str.end());
      pub->publish(out_msg);

      // Optional: log stats
      RCLCPP_INFO(
        get_logger(),
        "[%s] Original=%s, Compressed=%s, Ratio=%.2f, Time=%ld ms",
        original_topic.c_str(),
        format_size(raw_size).c_str(),
        format_size(compressed_str.size()).c_str(),
        (double)raw_size / (double)compressed_str.size(),
        duration_ms);
    }
    catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to compress msg on '%s': %s", original_topic.c_str(), e.what());
    }
  }

  // Compression with Boost iostreams
  std::string compressData(const uint8_t* data, size_t size, const std::string& algo) {
    std::stringstream compressed;
    boost::iostreams::filtering_ostream out;

    if (algo == "bz2") {
      out.push(boost::iostreams::bzip2_compressor());
    } else if (algo == "gzip") {
      out.push(boost::iostreams::gzip_compressor());
    } else if (algo == "zlib") {
      out.push(boost::iostreams::zlib_compressor());
    } else {
      throw std::runtime_error("Unknown compression algorithm: " + algo);
    }

    out.push(compressed);
    boost::iostreams::copy(
      boost::make_iterator_range(
        reinterpret_cast<const char*>(data),
        reinterpret_cast<const char*>(data + size)
      ),
      out);
    return compressed.str();
  }

  // Data
  YAML::Node config_;
  std::mutex subscribe_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_set<std::string> subscribed_topics_; // Keep track to avoid duplicates
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UniversalCompressorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
