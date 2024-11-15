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
import rostopic
import time
import sys

class DynamicRelayOutgoing:
    def __init__(self, original_topic, new_topic):
        self.original_topic = original_topic
        self.new_topic = new_topic
        self.published_messages = False
        self.msg_type = None
        self.publisher = None
        self.publisher_initialized = False

        self.initialize_publisher(log_message=True)

        self.subscriber = rospy.Subscriber(self.original_topic, rospy.AnyMsg, self.callback, queue_size=1)

    def initialize_publisher(self, log_message=False):
        try:
            self.msg_type = rostopic.get_topic_class(self.original_topic)[0]
            if self.msg_type is None:
                rospy.logwarn_once(f"Could not determine message type for topic: {self.original_topic}")
                self.publisher = None
            else:
                self.publisher = rospy.Publisher(self.new_topic, self.msg_type, queue_size=1)
                if log_message:
                    rospy.loginfo(f"Publisher initialized for {self.new_topic} on initialization.")
                self.publisher_initialized = True
        except rostopic.ROSTopicException as e:
            rospy.logerr_once(f"Failed to get message class for topic {self.original_topic}: {str(e)}")
            self.publisher = None
            self.publisher_initialized = False

    def update(self):
        if self.publisher is None:
            self.initialize_publisher(log_message=False)
            if self.publisher_initialized:
                rospy.loginfo(f"Publisher initialized for {self.new_topic} on update cycle.")

    def callback(self, msg):
        if self.publisher is None:
            self.initialize_publisher(log_message=False)
            if self.publisher is None:
                rospy.logerr_once(f"Could not initialize publisher for topic: {self.original_topic} on callback")
                return
        
            if self.publisher.get_num_connections() == 0:
                rospy.loginfo(f"No subscribers connected to '{self.new_topic}', waiting for connections...")
                while self.publisher.get_num_connections() == 0:
                    time.sleep(0.1)
                    rospy.loginfo(f"Still waiting for subscribers to connect to '{self.new_topic}'...")
            rospy.loginfo(f"Publisher for '{self.new_topic}' on callback after receiving message from '{self.original_topic}'.")
            rospy.loginfo(f"Subscriber found for {self.new_topic}.")
            self.publisher.publish(msg)
            return
        if not self.published_messages:
            rospy.loginfo(f"Publishing first relayed message from {self.original_topic} to {self.new_topic}")
            self.published_messages = True
        self.publisher.publish(msg)

def start_relay_nodes(topics_list, target_prefix='', topic_list_prefix='', sync_suffix=''):
    prefixed_topics_list = [topic_list_prefix + topic for topic in topics_list]
    
    relays = []
    for topic in prefixed_topics_list:
        new_topic = f"{target_prefix}{topic}{sync_suffix}"
        rospy.loginfo(f"Starting relay from '{topic}' to '{new_topic}'")
        relays.append(DynamicRelayOutgoing(topic, new_topic))
    return relays

if __name__ == "__main__":
    # Separate ROS parameters from command line arguments
    argv = rospy.myargv(argv=sys.argv)
    
    rospy.init_node("outgoing_relay", anonymous=True)
    
    target_prefix = rospy.get_param('~target_prefix', '')
    topic_list_prefixes = rospy.get_param('~topic_list_prefixes', '')
    topic_list_paths = rospy.get_param('~topic_list_paths')
    sync_suffix = rospy.get_param('~sync_suffix', '')  # Retrieve sync suffix from ROS parameter

    if isinstance(topic_list_paths, str):
        topic_list_paths = [topic_list_paths]  # Convert to list if only one path is provided
    all_topics_list = []
    for path in topic_list_paths:
        try:
            with open(path, 'r') as file:
                topics_list = [line.strip() for line in file if line.strip()]
                all_topics_list.extend(topics_list)  # Add topics from current file to the master list
        except FileNotFoundError:
            rospy.logerr(f"File not found: {path}")
            exit(1)

    if not isinstance(topic_list_prefixes, list):
        topic_list_prefixes = [topic_list_prefixes]  # Make it a list if only one prefix is provided

    # Map None values to empty strings
    topic_list_prefixes = [prefix if prefix != "None" else "" for prefix in topic_list_prefixes]
    if target_prefix == "None": target_prefix = ""

    relays = []
    for topic_list_prefix in topic_list_prefixes:
        relays = relays + start_relay_nodes(all_topics_list, target_prefix, topic_list_prefix, sync_suffix)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rate.sleep()    
        for relay in relays:
            relay.update()
