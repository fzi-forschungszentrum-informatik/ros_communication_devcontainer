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
#include <std_msgs/String.h>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filter/bzip2.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <sstream>
#include <chrono>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher decompressed_pub;
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

void decompressionCallback(const std_msgs::String::ConstPtr& msg)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    size_t compressed_size = msg->data.size();

    std::stringstream compressed_data(msg->data);
    std::stringstream original_data;

    boost::iostreams::filtering_istream in;

    if (compression_algorithm == "bzip2") {
        in.push(boost::iostreams::bzip2_decompressor());
    } else if (compression_algorithm == "gzip") {
        in.push(boost::iostreams::gzip_decompressor());
    } else if (compression_algorithm == "zlib") {
        in.push(boost::iostreams::zlib_decompressor());
    }

    in.push(compressed_data);
    boost::iostreams::copy(in, original_data);

    std::string data_str = original_data.str();
    std::vector<uint8_t> vec(data_str.begin(), data_str.end());
    ros::serialization::IStream istream(vec.data(), vec.size());
    nav_msgs::OccupancyGrid original_msg;
    ros::serialization::deserialize(istream, original_msg);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto decompression_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    size_t original_size = data_str.size();

    ROS_INFO("Compressed size: %s", format_size(compressed_size).c_str());
    ROS_INFO("Decompressed size: %s", format_size(original_size).c_str());
    ROS_INFO("Decompression ratio: %.2f", static_cast<double>(original_size) / compressed_size);
    ROS_INFO("Decompression time: %.6f milliseconds", static_cast<double>(decompression_time));

    decompressed_pub.publish(original_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_decompressor");
    ros::NodeHandle nh("~");

    std::string compression_topic;
    std::string decompression_topic;

    nh.getParam("compression_algorithm", compression_algorithm);
    nh.getParam("compression_topic", compression_topic);
    nh.getParam("decompression_topic", decompression_topic);

    ros::Subscriber sub;
    sub = nh.subscribe(compression_topic, 1000, decompressionCallback);

    decompressed_pub = nh.advertise<nav_msgs::OccupancyGrid>(decompression_topic, 1000);

    ros::spin();

    return 0;
}
