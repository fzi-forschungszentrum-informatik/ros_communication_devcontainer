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
import yaml
import importlib
import re
import zlib
import bz2
import lz4.frame as lz4
import zstandard as zstd
import io
import time
from std_msgs.msg import UInt8MultiArray
from threading import Lock

# Global set to track subscribed topics
subscribed_topics = set()
subscribe_lock = Lock()

def load_config(filename):
    rospy.logdebug(f"Loading configuration from {filename}")
    with open(filename, 'r') as file:
        config = yaml.safe_load(file)
        rospy.logdebug(f"Configuration loaded: {config}")
        return config

def get_message_class(package_path):
    rospy.logdebug(f"Getting message class for package path: {package_path}")
    package, message_name = package_path.split('/')
    mod = importlib.import_module(package + '.msg')
    msg_class = getattr(mod, message_name)
    rospy.logdebug(f"Message class {message_name} from {package} loaded")
    return msg_class

def compress_data(data, algorithm):
    rospy.logdebug(f"Compressing data with {algorithm} algorithm")
    if algorithm == 'gzip':
        return zlib.compress(data)
    elif algorithm == 'bz2':
        return bz2.compress(data)
    elif algorithm == 'lz4':
        return lz4.compress(data)
    elif algorithm == 'zstd':
        return zstd.ZstdCompressor().compress(data)
    else:
        rospy.logerr(f"Unsupported compression algorithm: {algorithm}")
        raise ValueError(f"Unsupported compression algorithm: {algorithm}")

def format_size(size):
    if size < 1024:
        return f"{size} B"
    elif size < 1048576:
        return f"{size / 1024:.1f} KB"
    elif size < 1073741824:
        return f"{size / 1048576:.1f} MB"
    return f"{size / 1073741824:.1f} GB"

def callback(msg, args):
    publisher, algorithm = args
    rospy.logdebug("Received message for compression")
    try:
        buffer = io.BytesIO()
        msg.serialize(buffer)
        original_data = buffer.getvalue()
        original_size = len(original_data)
        original_size_formatted = format_size(original_size)

        start_time = time.time()
        compressed_data = compress_data(original_data, algorithm)
        compression_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        compressed_size = len(compressed_data)
        compressed_size_formatted = format_size(compressed_size)

        rospy.loginfo(f"[{publisher.resolved_name}] orig. size: {original_size_formatted}, "
                      f"comp. size: {compressed_size_formatted}, "
                      f"comp. time: {compression_time:.1f} ms")

        compressed_msg = UInt8MultiArray(data=list(compressed_data))
        publisher.publish(compressed_msg)
    except Exception as e:
        rospy.logerr(f"Compression error: {str(e)}")

def setup_compressor_node():
    rospy.init_node('universal_compressor')
    config_file = rospy.get_param('~config', 'compression_config.yaml')
    config = load_config(config_file)
    rospy.loginfo(f"Node initialized with config file: {config_file}")

    check_and_subscribe(config)

    # Set up a timer to periodically check for new topics
    period = rospy.Duration(5)  # Check every 5 seconds
    rospy.Timer(period, lambda event: check_and_subscribe(config, update=True))
    
    rospy.spin()

def check_and_subscribe(config, update=False):
    global subscribed_topics
    with subscribe_lock:
        all_topics = rospy.get_published_topics()
        for item in config['compression']:
            topic_regex = item['topic_regex']
            algorithm = item.get('algorithm', "bz2")
            for topic, msg_type in all_topics:
                compressed_topic = topic + item.get('add_suffix', '_compressed')
                if re.search(topic_regex, topic) and (compressed_topic not in subscribed_topics or update):
                    rospy.logdebug(f"Matching topic found: {topic}")
                    if compressed_topic not in subscribed_topics:
                        msg_class = get_message_class(msg_type)
                        pub = rospy.Publisher(compressed_topic, UInt8MultiArray, queue_size=1)
                        rospy.Subscriber(topic, msg_class, callback, (pub, algorithm), queue_size=1)
                        subscribed_topics.add(compressed_topic)
                    else:
                        rospy.logwarn(f"Already subscribed to topic {topic}")

if __name__ == '__main__':
    setup_compressor_node()
