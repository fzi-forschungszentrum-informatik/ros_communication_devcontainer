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
from rclpy.duration import Duration
import cv2
import numpy as np
import time
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

# Resolution constants (if needed for reference)
resolutions = {
    "qqvga": "160x120",
    "qvga": "320x240",
    "custom": "480x360",
    "sd_vga": "640x480",
    "sd_svga": "800x600",
    "hd_720p": "1280x720",
    "hd": "1366x768",
    "fhd": "1920x1080",
    "qhd": "2560x1440",
    "res_4k": "3840x2160",
    "res_8k": "7680x4320"
}

def human_readable_size(size, precision=2):
    """Return a human-readable string for a given byte size."""
    suffixes = ['B', 'KB', 'MB', 'GB', 'TB']
    suffix_index = 0
    while size > 1024 and suffix_index < len(suffixes)-1:
        suffix_index += 1
        size /= 1024.0
    return f"{size:.{precision}f} {suffixes[suffix_index]}"

class ImageBandwidthReducer(Node):
    def __init__(self):
        super().__init__('image_bandwidth_reducer')
        self.bridge = CvBridge()
        self.pubs = {}

        # Declare parameters with defaults
        self.declare_parameter('resolution', resolutions['custom'])
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('frame_rate', 10)
        self.declare_parameter('topics', rclpy.Parameter.Type.STRING_ARRAY)

        # Get parameter values
        resolution_param = self.get_parameter('resolution').value
        try:
            self.width, self.height = map(int, resolution_param.split('x'))
        except Exception as e:
            self.get_logger().error(f"Invalid resolution parameter '{resolution_param}': {e}")
            self.width, self.height = 480, 360
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.frame_rate = self.get_parameter('frame_rate').value
        topics_param = self.get_parameter('topics').value
        if not isinstance(topics_param, list):
            topics_param = [topics_param]
        self.topics = topics_param

        self.last_time = self.get_clock().now()
        self.get_logger().info(f"Initialized with resolution: {self.width}x{self.height}, "
                                f"JPEG quality: {self.jpeg_quality}, frame rate: {self.frame_rate} Hz")
        # Create subscribers and publishers for each topic
        for topic in self.topics:
            reduced_topic = f'{topic}_reduced'
            self.pubs[reduced_topic] = self.create_publisher(CompressedImage, reduced_topic, 1)

            # Check message type based on topic name: if 'compressed' appears in topic, assume CompressedImage;
            # otherwise, assume raw Image.
            if 'compressed' in topic:
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, rt=reduced_topic: self.callback(msg, rt),
                    1
                )
            else:
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, rt=reduced_topic: self.callback(msg, rt),
                    1
                )
            self.get_logger().info(f"Subscribed to topic: {topic} and publishing to: {reduced_topic}")

    def callback(self, data, reduced_topic):
        start_time = time.time()
        current_time = self.get_clock().now()
        # Only process if sufficient time has elapsed
        if (current_time - self.last_time).nanoseconds / 1e9 >= 1.0 / self.frame_rate:
            try:
                # Convert ROS message to OpenCV image
                if isinstance(data, CompressedImage):
                    np_arr = np.frombuffer(data.data, np.uint8)
                    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                elif isinstance(data, Image):
                    image_np = self.bridge.imgmsg_to_cv2(data, "bgr8")
                else:
                    raise ValueError("Unsupported message type received")

                # Get size of original data
                original_size = len(data.data)
                original_size_str = human_readable_size(original_size)

                # Resize image to desired resolution
                resized_image = cv2.resize(image_np, (self.width, self.height), interpolation=cv2.INTER_LANCZOS4)
                resized_image = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)

                # Compress resized image as JPEG
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
                ret, buffer = cv2.imencode('.jpg', resized_image, encode_param)
                if not ret:
                    raise RuntimeError("JPEG encoding failed")
                compressed_size = len(buffer)
                compressed_size_str = human_readable_size(compressed_size)
                compression_ratio = original_size / compressed_size if compressed_size > 0 else float('inf')

                compression_time = (time.time() - start_time) * 1000  # in milliseconds

                self.get_logger().info(
                    f"[{reduced_topic}] Original size: {original_size_str}, "
                    f"Compressed size: {compressed_size_str} (ratio: {compression_ratio:.2f}), "
                    f"Compression time: {compression_time:.2f} ms"
                )

                # Create and publish new CompressedImage message
                reduced_msg = CompressedImage()
                reduced_msg.header = data.header
                reduced_msg.format = "jpeg"
                reduced_msg.data = buffer.tobytes()
                self.pubs[reduced_topic].publish(reduced_msg)

                self.last_time = current_time
            except Exception as e:
                self.get_logger().error(f"Failed to process and publish reduced image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageBandwidthReducer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
