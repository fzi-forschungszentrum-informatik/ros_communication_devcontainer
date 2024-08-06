#!/usr/bin/env python3

import os
import rospy
import subprocess

def start_fkie_master_sync(topics_list, sync_remote_nodes, check_host, topic_list_prefix=''):
    # Generate a list of topics with optional prefix and synchronization flag
    sync_topics_list = [topic_list_prefix + topic + "_sync" for topic in topics_list]
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

    #         # f'_ignore_services:="[/*]"',
    #         # f'_sync_services:="[/not_existing_service]"',
    
    # Print the command
    print("Executing command:", ' '.join(command))
    
    # Start the synchronization process
    subprocess.Popen(command)

if __name__ == "__main__":
    rospy.init_node('start_fkie_master_sync')

    # Retrieve parameters from ROS parameter server
    topic_list_prefixes = rospy.get_param('~topic_list_prefixes', [])
    topic_list_paths = rospy.get_param('~topic_list_paths')
    sync_remote_nodes = rospy.get_param('~sync_remote_nodes')
    check_host = rospy.get_param('~check_host')

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
        start_fkie_master_sync(all_topics_list, sync_remote_nodes, check_host, topic_list_prefix)

    rospy.spin()  # Keep the node running
