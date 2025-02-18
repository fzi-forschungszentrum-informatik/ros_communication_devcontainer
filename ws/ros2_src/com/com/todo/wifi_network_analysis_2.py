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

import os
import speedtest
import subprocess
import csv
from datetime import datetime
import re
import argparse

def measure_speed(num_measurements=5, test_location="Unknown", test_device="Unknown"):
    results = []
    download_speeds = []
    upload_speeds = []
    ping_times = []
    signal_levels = []
    wifi_ssid = get_wifi_ssid_linux()

    st = speedtest.Speedtest()

    print(f"Starting speed measurements ({num_measurements} measurements) for {test_device} at {test_location}...")

    # Get current date and time for file naming
    test_date = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

    # Ensure directory exists for saving results
    save_directory = "speed_test_results"
    os.makedirs(save_directory, exist_ok=True)

    # Generate unique filename for CSV based on test parameters
    csv_filename = f"{save_directory}/{test_location}_{test_date}_{test_device}.csv"

    # Initialize CSV file and write headers
    write_csv_header(csv_filename)

    for i in range(num_measurements):
        print(f"\nMeasurement {i+1}/{num_measurements}:")

        st.get_best_server()

        # Measure download speed
        download_speed = st.download() / 1_000_000  # Convert to Mbit/s
        print(f"Download Speed (Mbit/s): {download_speed:.2f}")
        download_speeds.append(download_speed)

        # Measure upload speed
        upload_speed = st.upload() / 1_000_000  # Convert to Mbit/s
        print(f"Upload Speed (Mbit/s): {upload_speed:.2f}")
        upload_speeds.append(upload_speed)

        # Measure ping time
        ping = st.results.ping  # Ping in ms
        print(f"Ping (ms): {ping:.2f}")
        ping_times.append(ping)

        server_host = st.results.server['host']
        server_name = st.results.server['name']
        print(f"Server Host: {server_host}")
        print(f"Server Name: {server_name}")

        signal_level, access_point = get_wifi_info()
        print(f"Signal Level (dBm): {signal_level}")
        signal_levels.append(signal_level)
        print(f"Access Point: {access_point}")

        # Get interface information
        interface_info = get_interface_info()
        print(f"Interface Information: {interface_info}")

        # Append results to list
        results.append({
            'Test Location': test_location,
            'Test Device': test_device,
            'Test Date': test_date,
            'Download Speed (Mbit/s)': download_speed,
            'Upload Speed (Mbit/s)': upload_speed,
            'Ping (ms)': ping,
            'Server Host': server_host,
            'Server Name': server_name,
            'Signal Level (dBm)': signal_level,
            'Access Point': access_point,
            'Wi-Fi SSID': wifi_ssid,
        })

        # Write latest result to CSV
        write_to_csv([results[-1]], csv_filename)

    # Print summary once all measurements are completed
    print_summary(download_speeds, upload_speeds, ping_times, signal_levels)

def write_csv_header(filename):
    keys = [
        'Test Location', 'Test Device', 'Test Date', 'Download Speed (Mbit/s)', 'Upload Speed (Mbit/s)',
        'Ping (ms)', 'Server Host', 'Server Name', 'Signal Level (dBm)', 'Access Point', 'Wi-Fi SSID'
    ]
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=keys)
        writer.writeheader()

def write_to_csv(results, filename):
    keys = results[0].keys()
    with open(filename, 'a', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=keys)
        for result in results:
            writer.writerow(result)

def print_summary(download_speeds, upload_speeds, ping_times, signal_levels):
    print("\nFinal Measurement Summary:")
    print(f"Avg Download Speed (Mbit/s): {average(download_speeds):.2f}")
    print(f"Min Download Speed (Mbit/s): {min(download_speeds):.2f}")
    print(f"Max Download Speed (Mbit/s): {max(download_speeds):.2f}")
    print(f"Avg Upload Speed (Mbit/s): {average(upload_speeds):.2f}")
    print(f"Min Upload Speed (Mbit/s): {min(upload_speeds):.2f}")
    print(f"Max Upload Speed (Mbit/s): {max(upload_speeds):.2f}")
    print(f"Avg Ping (ms): {average(ping_times):.2f}")
    print(f"Min Ping (ms): {min(ping_times):.2f}")
    print(f"Max Ping (ms): {max(ping_times):.2f}")
    print(f"Avg Signal Level (dBm): {average(signal_levels):.2f}")
    print(f"Min Signal Level (dBm): {min(signal_levels)}")
    print(f"Max Signal Level (dBm): {max(signal_levels)}")

def get_interface_info():
    try:
        iwconfig_output = subprocess.check_output(['iwconfig']).decode('utf-8').strip().split('\n')
        interfaces = []
        for line in iwconfig_output:
            if re.search(r'^\w+', line):
                interfaces.append(line)
        if interfaces:
            return ', '.join(interfaces)
        else:
            return "No wireless extensions"
    except subprocess.CalledProcessError:
        return "No wireless extensions"

def get_wifi_ssid_linux():
    try:
        output = subprocess.check_output(['iwgetid', '-r']).decode('utf-8').strip()
        return output
    except subprocess.CalledProcessError:
        print("Failed to get Wi-Fi SSID.")
        return "Unknown"

def get_wifi_info():
    try:
        iwconfig_output = subprocess.check_output(['iwconfig']).decode('utf-8')
        signal_level_match = re.search(r'Signal level=(-\d+)', iwconfig_output)
        if signal_level_match:
            signal_level = int(signal_level_match.group(1))
        else:
            signal_level = None

        access_point_match = re.search(r'Access Point: (\S+)', iwconfig_output)
        if access_point_match:
            access_point = access_point_match.group(1)
        else:
            access_point = None

        return signal_level, access_point

    except subprocess.CalledProcessError:
        print("Failed to get WiFi information.")
        return None, None
def average(lst):
    if len(lst) == 0:
        return 0
    return sum(lst) / len(lst)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Perform Wi-Fi speed measurements and save results to CSV.')
    parser.add_argument('test_location', type=str, help='Location where the test is performed')
    parser.add_argument('test_device', type=str, help='Device used for the test')
    parser.add_argument('--num_measurements', type=int, default=5, help='Number of measurements to perform (default: 5)')
    args = parser.parse_args()

    try:
        measure_speed(num_measurements=args.num_measurements, test_location=args.test_location, test_device=args.test_device)
    except KeyboardInterrupt:
        print("\n\nMeasurement interrupted. Final results saved up to this point.")
        print_summary(download_speeds, upload_speeds, ping_times, signal_levels)
