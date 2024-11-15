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
import re
from multiprocessing import Pool, cpu_count
import time
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
        bw_output, _ = bw_process.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        bw_process.terminate()
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
        hz_output, _ = hz_process.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        hz_process.terminate()
        hz_output = hz_process.stdout.read()

    pattern = re.compile(r'average rate:\s([\d\.]+)', re.MULTILINE)
    frequency_matches = pattern.findall(hz_output)
    if frequency_matches:
        frequency = float(frequency_matches[-1])

    # Get delay
    cmd_delay = ['rostopic', 'delay', topic]
    delay_process = subprocess.Popen(cmd_delay, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    delay = "-"
    try:
        delay_output, _ = delay_process.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        delay_process.terminate()
        delay_output = delay_process.stdout.read()

    #print("delay_output:" + str(delay_output))

    if "msg does not have header" in delay_output:
        delay = "No header"
    else:
        pattern = re.compile(r'average delay:\s([\d\.]+)', re.MULTILINE)
        delay_matches = pattern.findall(delay_output)
        if delay_matches:
            delay = float(delay_matches[-1])
    #print("delay:" + str(delay))

    return topic, bandwidth, frequency, delay

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
                topic_details = process_topics(sync_topics)

                total_bandwidth = sum(details[1] for details in topic_details if isinstance(details[1], float))

                topic_details.sort(key=lambda x: x[1] if isinstance(x[1], float) else 0, reverse=True)

                log_with_timestamp(f"{'Topic'.ljust(87)} | {'Bandwidth'.rjust(14)} | {'Frequency'.rjust(9)} | {'Delay'.rjust(8)}")
                print('-' * 140)
                for details in topic_details:
                    bandwidth_display = f"{details[1]:>7.0f} Kbit/s" if isinstance(details[1], float) else details[1]
                    frequency_display = f"{details[2]:>6.1f} Hz" if isinstance(details[2], float) else details[2]
                    delay_display = f"{details[3]:>6.3f} s" if isinstance(details[3], float) else details[3]
                    log_with_timestamp(f"{details[0].ljust(87)} | {bandwidth_display:>14} | {frequency_display:>9} | {delay_display:>8}")

                log_with_timestamp(f"Total Bandwidth: {total_bandwidth:.1f} Kbit/s")

            log_with_timestamp("\nWaiting for the next update...\n")
            time.sleep(refresh_interval)
    except KeyboardInterrupt:
        log_with_timestamp("Exiting the program...")
        rospy.signal_shutdown("User requested interruption.")
        exit(0)
