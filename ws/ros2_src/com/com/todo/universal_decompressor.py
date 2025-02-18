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
import time
from std_msgs.msg import UInt8MultiArray
from threading import Lock

# Global set to track subscribed topics and a lock for thread-safe operations
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

def decompress_data(data, algorithm):
    rospy.logdebug(f"Decompressing data with {algorithm} algorithm")
    if algorithm == 'gzip':
        return zlib.decompress(data)
    elif algorithm == 'bz2':
        return bz2.decompress(data)
    elif algorithm == 'lz4':
        return lz4.decompress(data)
    elif algorithm == 'zstd':
        return zstd.ZstdDecompressor().decompress(data)
    else:
        rospy.logerr(f"Unsupported decompression algorithm: {algorithm}")
        raise ValueError(f"Unsupported decompression algorithm: {algorithm}")

def format_size(size):
    if size < 1024:
        return f"{size} B"
    elif size < 1048576:
        return f"{size / 1024:.1f} KB"
    elif size < 1073741824:
        return f"{size / 1048576:.1f} MB"
    return f"{size / 1073741824:.1f} GB"

def callback(msg, args):
    publisher, msg_class, algorithm = args
    compressed_data_bytes = bytes(msg.data)
    compressed_size = len(compressed_data_bytes)
    compressed_size_formatted = format_size(compressed_size)

    rospy.loginfo(f"Received message with compressed size: {compressed_size_formatted}")

    start_time = time.time()
    try:
        decompressed_data = decompress_data(compressed_data_bytes, algorithm)
    except Exception as e:
        rospy.logerr(f"Decompression error: {str(e)}")
        return

    decompression_time = (time.time() - start_time) * 1000  # Convert to milliseconds
    decompressed_size = len(decompressed_data)
    decompressed_size_formatted = format_size(decompressed_size)

    rospy.loginfo(f"[{publisher.resolved_name}] comp. size: {compressed_size_formatted}, decomp. size: {decompressed_size_formatted}, decomp. time: {decompression_time:.1f} ms")

    try:
        deserialized_msg = msg_class().deserialize(decompressed_data)
        publisher.publish(deserialized_msg)
        rospy.logdebug("Message published after decompression and deserialization")
    except Exception as e:
        rospy.logerr(f"Deserialization error: {str(e)} - Data may be corrupt or incomplete.")

def setup_decompressor_node():
    rospy.init_node('universal_decompressor')
    config_file = rospy.get_param('~config', 'decompression_config.yaml')
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
        all_topics = [topic for topic, msg_type in rospy.get_published_topics()]
        for item in config['decompression']:
            topic_regex = item['topic_regex']
            algorithm = item.get('algorithm', "bz2")
            for topic in all_topics:
                if re.search(topic_regex, topic) and (topic not in subscribed_topics or update):
                    rospy.logdebug(f"Matching topic found: {topic}")
                    msg_class = get_message_class(item['msg_type'])
                    remove_suffix = item.get('remove_suffix', '')
                    if topic.endswith(remove_suffix):
                        decompressed_topic = topic[:-len(remove_suffix)] + item.get('add_suffix', '')
                        if decompressed_topic not in subscribed_topics:
                            pub = rospy.Publisher(decompressed_topic, msg_class, queue_size=1)
                            rospy.Subscriber(topic, UInt8MultiArray, callback, (pub, msg_class, algorithm), queue_size=1)
                            subscribed_topics.add(decompressed_topic)
                        else:
                            rospy.logwarn(f"Already subscribed to topic {decompressed_topic}")
                    else:
                        rospy.logwarn(f"The topic '{topic}' does not end with the remove-suffix '{remove_suffix}'. Ignoring topic.")

if __name__ == '__main__':
    setup_decompressor_node()
