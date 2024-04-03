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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Martin Gotscharow <gontscharow@fzi.de>
 * \date    2024-04-03
 *
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/copy.hpp>
#include <sstream>
#include <chrono>
#include <fstream>
#include <iomanip>

ros::Publisher compressed_pub;
std::string compression_algorithm;

std::string format_size(size_t size_in_bytes) {
    double size_kb = size_in_bytes / 1024.0;
    double size_mb = size_kb / 1024.0;
    std::ostringstream formatted_size;

    if (size_mb >= 1.0) {
        formatted_size << std::fixed << std::setprecision(2) << size_mb << " MB";
    } else if (size_kb >= 1.0) {
        formatted_size << std::fixed << std::setprecision(2) << size_kb << " KB";
    } else {
        formatted_size << size_in_bytes << " B";
    }

    return formatted_size.str();
}

void compressionCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    std::stringstream original_data;
    std::string costmap_as_string(reinterpret_cast<const char*>(&msg->data[0]), msg->data.size());

    uint32_t length = ros::serialization::serializationLength(*msg);
    boost::shared_array<uint8_t> buffer(new uint8_t[length]);
    ros::serialization::OStream stream(buffer.get(), length);
    ros::serialization::serialize(stream, *msg);
    original_data.write((const char*)buffer.get(), length);

    std::stringstream compressed_data;
    boost::iostreams::filtering_ostream out;

    if (compression_algorithm == "bzip2") {
        out.push(boost::iostreams::bzip2_compressor());
    } else if (compression_algorithm == "gzip") {
        out.push(boost::iostreams::gzip_compressor());
    } else if (compression_algorithm == "zlib") {
        out.push(boost::iostreams::zlib_compressor());
    }
    
    out.push(compressed_data);
    boost::iostreams::copy(original_data, out);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto compression_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    size_t original_size = costmap_as_string.size();
    size_t compressed_size = compressed_data.str().size();

    ROS_INFO("Original size: %s", format_size(original_size).c_str());
    ROS_INFO("Compressed size: %s", format_size(compressed_size).c_str());
    ROS_INFO("Compression ratio: %.2f", static_cast<double>(original_size) / compressed_size);
    ROS_INFO("Compression time: %.6f milliseconds", static_cast<double>(compression_time));

    std_msgs::String compressed_msg;
    compressed_msg.data = compressed_data.str();
    compressed_pub.publish(compressed_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_compressor");
    ros::NodeHandle nh("~");

    std::string compression_topic;
    std::string map_topic;
    std::string compression_identifier;

    nh.getParam("compression_algorithm", compression_algorithm);
    nh.getParam("map_topic", map_topic);
    nh.getParam("compression_topic", compression_topic);

    ros::Subscriber sub;
    sub = nh.subscribe(map_topic, 1000, compressionCallback);

    compressed_pub = nh.advertise<std_msgs::String>(compression_topic, 1000);

    ros::spin();

    return 0;
}
