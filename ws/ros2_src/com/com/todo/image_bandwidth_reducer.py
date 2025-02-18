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

import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

# Resolution Constants
resolutions = {
    "qqvga": "160x120", "qvga": "320x240", "custom": "480x360",
    "sd_vga": "640x480", "sd_svga": "800x600", "hd_720p": "1280x720",
    "hd": "1366x768", "fhd": "1920x1080", "qhd": "2560x1440",
    "res_4k": "3840x2160", "res_8k": "7680x4320"
}

def human_readable_size(size, precision=2):
    suffixes = ['B', 'KB', 'MB', 'GB', 'TB']
    suffix_index = 0
    while size > 1024 and suffix_index < 4:
        suffix_index += 1
        size /= 1024.0
    return f"{size:.{precision}f} {suffixes[suffix_index]}"

class ImageBandwidthReducer:
    def __init__(self, topics, width, height, jpeg_quality, frame_rate):
        self.width = width
        self.height = height
        self.jpeg_quality = jpeg_quality
        self.frame_rate = frame_rate
        self.last_time = rospy.Time.now()
        self.bridge = CvBridge()
        self.publishers = {}
        rospy.loginfo(f"Initialized with width: {width}, height: {height}, JPEG quality: {jpeg_quality}, frame rate: {frame_rate}")

        # Ensure topics is a list even if a single topic is provided
        if not isinstance(topics, list):
            topics = [topics]  # Make it a list if only one topic is provided

        # Create subscribers and publishers for each topic
        for topic in topics:
            reduced_topic = f'{topic}_reduced'
            self.publishers[reduced_topic] = rospy.Publisher(reduced_topic, CompressedImage, queue_size=1)
            
            # Check if the topic is likely a raw image or compressed image
            if 'compressed' in topic:
                rospy.Subscriber(topic, CompressedImage, self.callback, callback_args=reduced_topic, queue_size=1)
            else:
                rospy.Subscriber(topic, Image, self.callback, callback_args=reduced_topic, queue_size=1)
            
            rospy.loginfo(f"Subscribed to topic: {topic} and publishing to: {reduced_topic}")

    def callback(self, data, reduced_topic):
        start_time = time.time()
        current_time = rospy.Time.now()
        if (current_time - self.last_time).to_sec() > 1.0 / self.frame_rate:
            try:
                if isinstance(data, CompressedImage):
                    np_arr = np.frombuffer(data.data, np.uint8)
                    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                elif isinstance(data, Image):
                    image_np = self.bridge.imgmsg_to_cv2(data, "bgr8")
                else:
                    raise ValueError("Unsupported ROS message type")

                original_size = human_readable_size(len(data.data))
                resized_image = cv2.resize(image_np, (self.width, self.height), interpolation=cv2.INTER_LANCZOS4)
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                _, buffer = cv2.imencode('.jpg', resized_image, encode_param)
                compressed_size = human_readable_size(len(buffer))
                compression_ratio = len(data.data) / len(buffer) if len(buffer) > 0 else float('inf')

                compressed_image_msg = CompressedImage()
                compressed_image_msg.header = data.header
                compressed_image_msg.format = "jpeg"
                compressed_image_msg.data = np.array(buffer).tobytes()
                self.publishers[reduced_topic].publish(compressed_image_msg)
                self.last_time = current_time

                compression_time = (time.time() - start_time) * 1000  # Convert to milliseconds
                rospy.loginfo(f"[{reduced_topic}] orig. size: {original_size}, comp. size: {compressed_size} (ratio: {compression_ratio:.2f}), comp time: {compression_time:.2f} ms")
            except Exception as e:
                rospy.logerr(f"Failed to compress and publish data: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('image_bandwidth_reducer')
    resolution_param = rospy.get_param('~resolution', resolutions['custom'])
    width, height = map(int, resolution_param.split('x'))
    jpeg_quality = rospy.get_param('~jpeg_quality', 50)
    frame_rate = rospy.get_param('~frame_rate', 10)
    topics = rospy.get_param('~topics')

    ImageBandwidthReducer(topics, width, height, jpeg_quality, frame_rate)
    rospy.spin()
