#!/usr/bin/env python3

import rospy
import subprocess
import re
import os
from multiprocessing import Pool, cpu_count
import time  # Required to add sleep for periodic updates
from datetime import datetime

def log_with_timestamp(message):
    """ Helper function to log messages with a timestamp. """
    print(f"{datetime.now().strftime('%Y-%m-%d %H:%M:%S')} - {message}")

def has_publishers(topic):
    """ Check if a topic has any publishers. """
    info_process = subprocess.run(['rostopic', 'info', topic], stdout=subprocess.PIPE, text=True)
    return "Publishers: None" not in info_process.stdout

def convert_to_kbits(value, unit):
    """ Convert bandwidth to Kbit/s """
    if unit == "B":
        value = value / 1024  # Convert B/s to KB/s
    return value * 8

def get_bandwidth_and_frequency(topic):
    # Get bandwidth
    cmd_bw = ['rostopic', 'bw', topic]
    bw_process = subprocess.Popen(cmd_bw, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    bandwidth = "-"
    try:
        bw_output, _ = bw_process.communicate(timeout=4)
    except subprocess.TimeoutExpired:
        bw_process.terminate()  # Ensure the process is terminated after timeout
        # Find all matches and select the last one
        bw_output = bw_process.stdout.read()

        pattern = re.compile(r'average:\s([\d\.]+)([K|B]B?)/s', re.MULTILINE)
        bandwidth_matches = pattern.findall(bw_output)
        if bandwidth_matches:
            value, unit = bandwidth_matches[-1]
            bandwidth = convert_to_kbits(float(value), unit)

    # Get frequency
    cmd_hz = ['rostopic', 'hz', topic]
    hz_process = subprocess.Popen(cmd_hz, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    frequency = "-"
    try:
        hz_output, _ = hz_process.communicate(timeout=4)
    except subprocess.TimeoutExpired:
        hz_process.terminate()  # Ensure the process is terminated after timeout
        # Find all matches and select the last one
        hz_output = hz_process.stdout.read()

        # Define the regex pattern to extract average rate values
        pattern = re.compile(r'average rate:\s([\d\.]+)', re.MULTILINE)
        # Find all matches in the text
        frequency_matches = pattern.findall(hz_output)
        if frequency_matches:
            frequency = float(frequency_matches[-1])  # Select the last match

    return topic, bandwidth, frequency

def process_topics(sync_topics):
    if not sync_topics:
        return []
    with Pool(processes=cpu_count()) as pool:
        results = pool.map(get_bandwidth_and_frequency, sync_topics)
    return results

def find_sync_topics_with_publishers():
    log_with_timestamp("Finding synchronized topics with active publishers...")
    topics_process = subprocess.run(['rostopic', 'list'], stdout=subprocess.PIPE, text=True)
    topics = topics_process.stdout.split()
    # Filter topics to include only those ending with '_sync' and having publishers
    active_sync_topics = [topic for topic in topics if topic.endswith('_sync') and has_publishers(topic)]
    return active_sync_topics

if __name__ == "__main__":
    rospy.init_node('topic_monitor', anonymous=True)
    refresh_interval = 1  # seconds between updates

    try:
        while True:
            sync_topics = find_sync_topics_with_publishers()
            log_with_timestamp(f"Found {len(sync_topics)} sync topics with active publishers.")
            if not sync_topics:
                log_with_timestamp("No active sync topics found. Waiting for the next update...")
            else:
                # Gather and sort data
                topic_details = process_topics(sync_topics)

                # Only sum bandwidths that are numbers
                total_bandwidth = sum(details[1] for details in topic_details if isinstance(details[1], float))

                # Sort topics by bandwidth
                topic_details.sort(key=lambda x: x[1] if isinstance(x[1], float) else 0, reverse=True)

                # Print table header and details
                log_with_timestamp(f"{'Topic'.ljust(87)} | {'Bandwidth'.rjust(14)} | {'Frequency'.rjust(9)}")
                print('-' * 130)
                for details in topic_details:
                    bandwidth_display = f"{details[1]:>7.0f} Kbit/s" if isinstance(details[1], float) else details[1]
                    frequency_display = f"{details[2]:>6.1f} Hz" if isinstance(details[2], float) else details[2]
                    log_with_timestamp(f"{details[0].ljust(87)} | {bandwidth_display:>14} | {frequency_display:>9}")

                # Print total bandwidth in Kbit/s
                log_with_timestamp(f"Total Bandwidth: {total_bandwidth:.1f} Kbit/s")

            log_with_timestamp("\nWaiting for the next update...\n")
            time.sleep(refresh_interval)
    except KeyboardInterrupt:
        log_with_timestamp("Exiting the program...")
        rospy.signal_shutdown("User requested interruption.")
        exit(0)
