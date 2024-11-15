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
import subprocess

def start_fkie_master_sync(topics_list, sync_remote_nodes, check_host, topic_list_prefix, sync_suffix):
    # Generate a list of topics with optional prefix and synchronization flag
    sync_topics_list = [topic_list_prefix + topic + sync_suffix for topic in topics_list]
    sync_topics = ','.join(sync_topics_list)  # Convert list to a comma-separated string

    # Adjust the node name if a prefix is provided
    node_name = "fkie_sync"
    if topic_list_prefix:
        node_name += f"_{topic_list_prefix.replace('/', '_')}"

    rospy.loginfo(f"Starting {node_name}")

    command = [
        'rosrun',
        'fkie_master_sync',
        'master_sync',
        f'_sync_topics:=[{sync_topics}]',
        f'_sync_remote_nodes:={sync_remote_nodes}',
        f'_check_host:={check_host}',
        '__name:=' + node_name
    ]
   
    # Print the command
    print("Executing command:", ' '.join(command))
    
    # Start the synchronization process
    subprocess.Popen(command)

if __name__ == "__main__":
    rospy.init_node('start_fkie_master_sync')

    # Retrieve parameters from ROS parameter server
    topic_list_prefixes = rospy.get_param('~topic_list_prefixes', "")
    topic_list_paths = rospy.get_param('~topic_list_paths')
    sync_remote_nodes = rospy.get_param('~sync_remote_nodes')
    check_host = rospy.get_param('~check_host')
    sync_suffix = rospy.get_param('~sync_suffix', '')

    # Validate that topic_list_paths is a list
    if isinstance(topic_list_paths, str):
        topic_list_paths = [topic_list_paths]  # Convert to list if only one path is provided

    # Initialize a list to collect all topics from all files
    all_topics_list = []

    # Read topics from each file specified in the topic_list_paths
    for path in topic_list_paths:
        try:
            with open(path, 'r') as file:
                topics_list = [line.strip() for line in file if line.strip()]
                all_topics_list.extend(topics_list)  # Add topics from current file to the master list
        except FileNotFoundError:
            rospy.logerr(f"File not found: {path}")
            exit(1)

    # Ensure topic_list_prefixes is a list
    if not isinstance(topic_list_prefixes, list):
        topic_list_prefixes = [topic_list_prefixes]

    # Start synchronization for each prefix using the collected list of all topics
    for topic_list_prefix in topic_list_prefixes:
        start_fkie_master_sync(all_topics_list, sync_remote_nodes, check_host, topic_list_prefix, sync_suffix)

    rospy.spin()  # Keep the node running
