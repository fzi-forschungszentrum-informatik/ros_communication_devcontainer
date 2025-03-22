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
#include <yaml-cpp/yaml.h>

// Adjust includes as needed for your message packages:
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <com_msgs/msg/compressed_data.hpp> 
// or #include <com_cpp/msg/compressed_data.hpp> if that's your type

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/copy.hpp>

#include <regex>
#include <string>
#include <mutex>
#include <unordered_set>
#include <vector>
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

class UniversalDecompressorNode : public rclcpp::Node {
public:
  UniversalDecompressorNode()
  : Node("universal_decompressor_node")
  {
    // 1) Load YAML config parameter
    this->declare_parameter<std::string>("config_file", "decompression_config.yaml");
    std::string config_file = this->get_parameter("config_file").as_string();

    RCLCPP_INFO(get_logger(), "Loading decompression config from: %s", config_file.c_str());

    try {
      config_ = YAML::LoadFile(config_file);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load YAML config: %s", e.what());
      return;
    }

    // 2) Optionally re-check every X seconds for new topics.
    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&UniversalDecompressorNode::checkConfigAndSubscribe, this)
    );

    // Or, if you only want to do it once at startup:
    // checkConfigAndSubscribe();
  }

private:
  void checkConfigAndSubscribe()
  {
    std::lock_guard<std::mutex> lock(subscribe_mutex_);

    if (!config_["decompression"]) {
      RCLCPP_WARN(get_logger(), "No 'decompression' node in YAML config.");
      return;
    }

    // Retrieve the full list of topics + types from the ROS graph
    auto all_topics = this->get_topic_names_and_types();

    // For each entry in the YAML
    for (auto && entry : config_["decompression"]) {
      std::string topic_regex   = entry["topic_regex"].as<std::string>();
      std::string msg_type_hint = entry["msg_type"].as<std::string>();  // e.g. "nav_msgs/msg/OccupancyGrid"
      std::string remove_suffix = entry["remove_suffix"] ? entry["remove_suffix"].as<std::string>() : "_compressed";
      std::string add_suffix    = entry["add_suffix"]    ? entry["add_suffix"].as<std::string>() : "";
      std::string algorithm     = entry["algorithm"]     ? entry["algorithm"].as<std::string>() : "bz2";

      std::regex rx(topic_regex);

      // Loop over all discovered topics
      for (const auto & [topic_name, type_vec] : all_topics) {
        // Check if topic name matches the regex
        if (!std::regex_search(topic_name, rx)) {
          continue;
        }

        // Check if we've already subscribed to this topic
        if (subscribed_topics_.count(topic_name) > 0) {
          continue;
        }

        // We only expect the compressed-data message type here
        // e.g. "com_msgs/msg/CompressedData"
        bool is_compressed_msg = false;
        for (auto & t : type_vec) {
          if (t == "com_msgs/msg/CompressedData") {
            is_compressed_msg = true;
            break;
          }
        }
        if (!is_compressed_msg) {
          continue;
        }

        // Determine the uncompressed output topic
        // 1) Remove `remove_suffix` if present at the end
        // 2) Append `add_suffix`
        std::string out_topic = topic_name;
        if (!remove_suffix.empty()) {
          if (out_topic.size() >= remove_suffix.size()) {
            if (out_topic.compare(out_topic.size() - remove_suffix.size(),
                                  remove_suffix.size(),
                                  remove_suffix) == 0)
            {
              out_topic.erase(out_topic.size() - remove_suffix.size());
            }
          }
        }
        out_topic += add_suffix;

        // We'll create a typed subscription to CompressedData,
        // and publish the final uncompressed message according to msg_type_hint
        if (msg_type_hint == "nav_msgs/msg/OccupancyGrid") {
          // Publisher
          auto pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(out_topic, 10);
          // Subscription
          auto sub = this->create_subscription<com_msgs::msg::CompressedData>(
            topic_name, 10,
            [this, pub, topic_name, msg_type_hint, algorithm](const com_msgs::msg::CompressedData::SharedPtr msg) {
              decompressCallback<nav_msgs::msg::OccupancyGrid>(msg, pub, topic_name, msg_type_hint, algorithm);
            }
          );
          subscriptions_.push_back(sub);
          RCLCPP_INFO(get_logger(),
            "Decompressor subscribed to '%s' -> publishing OccupancyGrid on '%s' (algo=%s)",
            topic_name.c_str(), out_topic.c_str(), algorithm.c_str());

        } else if (msg_type_hint == "visualization_msgs/msg/MarkerArray") {
          auto pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(out_topic, 10);
          auto sub = this->create_subscription<com_msgs::msg::CompressedData>(
            topic_name, 10,
            [this, pub, topic_name, msg_type_hint, algorithm](const com_msgs::msg::CompressedData::SharedPtr msg) {
              decompressCallback<visualization_msgs::msg::MarkerArray>(msg, pub, topic_name, msg_type_hint, algorithm);
            }
          );
          subscriptions_.push_back(sub);
          RCLCPP_INFO(get_logger(),
            "Decompressor subscribed to '%s' -> publishing MarkerArray on '%s' (algo=%s)",
            topic_name.c_str(), out_topic.c_str(), algorithm.c_str());

        } else {
          // Extend for any other message types you might want to handle
          RCLCPP_WARN(get_logger(),
            "Decompressor skipping topic '%s': unsupported msg_type hint '%s'",
            topic_name.c_str(), msg_type_hint.c_str());
        }

        // Mark the compressed topic as subscribed
        subscribed_topics_.insert(topic_name);
      }
    }
  }

  // Templated callback that:
  // 1. Decompresses the raw data from com_msgs::msg::CompressedData
  // 2. Deserializes it into the typed message T
  // 3. Publishes T
  template<typename T>
  void decompressCallback(
    const com_msgs::msg::CompressedData::SharedPtr & msg_in,
    const typename rclcpp::Publisher<T>::SharedPtr & pub,
    const std::string & compressed_topic,
    const std::string & msg_type_hint,
    const std::string & algorithm)
  {
    try {
      // 1) Decompress
      auto start = std::chrono::high_resolution_clock::now();

      std::vector<uint8_t> decompressed_bytes = decompressData(msg_in->data, algorithm);

      auto end = std::chrono::high_resolution_clock::now();
      auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

      // 2) Deserialize into a typed message T
      rclcpp::SerializedMessage serialized_msg;
      serialized_msg.reserve(decompressed_bytes.size());
      serialized_msg.get_rcl_serialized_message().buffer_length = decompressed_bytes.size();
      std::memcpy(serialized_msg.get_rcl_serialized_message().buffer,
                  decompressed_bytes.data(),
                  decompressed_bytes.size());

      T out_msg;
      rclcpp::Serialization<T> serializer;
      serializer.deserialize_message(&serialized_msg, &out_msg);

      // 3) Publish
      pub->publish(out_msg);

      // Optional: log stats
      RCLCPP_INFO(get_logger(),
        "[Decompress: %s -> %s] Decompressed %s to %s (ratio=%.2f) in %ld ms (algo=%s)",
        compressed_topic.c_str(), msg_type_hint.c_str(),
        format_size(msg_in->data.size()).c_str(),
        format_size(decompressed_bytes.size()).c_str(),
        (double)decompressed_bytes.size() / (double)msg_in->data.size(),
        duration_ms, algorithm.c_str());

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Failed to decompress/deserialize from '%s' (type '%s', algo '%s'): %s",
        compressed_topic.c_str(), msg_type_hint.c_str(), algorithm.c_str(), e.what());
    }
  }

  // Helper to decompress data with the specified algorithm
  std::vector<uint8_t> decompressData(const std::vector<uint8_t> & input,
                                      const std::string & algorithm)
  {
    // Convert input data to a stream
    std::stringstream compressed_stream;
    compressed_stream.write(
      reinterpret_cast<const char*>(input.data()),
      static_cast<std::streamsize>(input.size()));

    // Set up filter chain
    boost::iostreams::filtering_istream in;
    if (algorithm == "bz2") {
      in.push(boost::iostreams::bzip2_decompressor());
    } else if (algorithm == "gzip") {
      in.push(boost::iostreams::gzip_decompressor());
    } else if (algorithm == "zlib") {
      in.push(boost::iostreams::zlib_decompressor());
    } else {
      throw std::runtime_error("Unknown or unsupported decompression algorithm: " + algorithm);
    }

    in.push(compressed_stream);

    // Copy decompressed bytes into an output buffer
    std::stringstream decompressed_stream;
    boost::iostreams::copy(in, decompressed_stream);

    std::string decompressed_str = decompressed_stream.str();
    return std::vector<uint8_t>(decompressed_str.begin(), decompressed_str.end());
  }

  YAML::Node config_;
  std::mutex subscribe_mutex_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unordered_set<std::string> subscribed_topics_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UniversalDecompressorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
