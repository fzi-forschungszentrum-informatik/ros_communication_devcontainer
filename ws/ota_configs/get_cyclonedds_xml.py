#!/usr/bin/env python3

import os
import sys
import argparse

ws_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(ws_dir)
from ota_configs.utils import process_template
from session.content.get_data_dict_entries import main as get_data_dict_entries

def main(host_ip, peers):

    host_ip_resolved_list = get_data_dict_entries(host_ip)
    if len(host_ip_resolved_list) != 1:
        raise ValueError("Host IP must resolve to exactly one IP address.")
    host_ip_resolved = host_ip_resolved_list[0]

    peers_resolved = get_data_dict_entries(peers)
    peers_str = ""
    for peer in peers_resolved:
        peers_str += f"<Peer Address=\"{peer}\"/>"

    return process_template(
        template_path="/ws/ota_configs/cyclonedds.xml.template", 
        substitutions={"#host_ip": host_ip_resolved, "#peers": peers_str})

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--host_ip', required=True)
    parser.add_argument('-p', '--peers', required=True)
    args = parser.parse_args()

    result = main(**{k: v for k, v in vars(args).items() if v is not None})

    print(result)  # This will output the value to stdout, which can be captured in a shell