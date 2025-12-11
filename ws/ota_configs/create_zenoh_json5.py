#!/usr/bin/env python3

import os
import sys
import argparse

ws_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(ws_dir)
from ota_configs.utils import process_template
from session.content.get_data_dict_entries import main as get_data_dict_entries

def main(zen_mode, zen_endpoint_role, zen_transport, zen_main_ip, zen_main_port, zen_config_file, zen_pub_allow="", zen_sub_allow="", zen_qos_pub=""):
    zen_main_ip_resolved_list = get_data_dict_entries(zen_main_ip)
    if len(zen_main_ip_resolved_list) != 1:
        raise ValueError("Host IP must resolve to exactly one IP address.")
    zen_main_ip_resolved = zen_main_ip_resolved_list[0]

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')

    config_content = process_template(
        template_path="/ws/ota_configs/zenoh.json5.template", 
        substitutions={
            "#zen_mode": zen_mode,
            "#zen_endpoint_role": zen_endpoint_role,
            "#zen_transport": zen_transport,
            "#zen_main_ip": zen_main_ip_resolved,
            "#zen_main_port": zen_main_port,
            "#ros_domain_id": ros_domain_id,
            "#zen_pub_allow": zen_pub_allow,
            "#zen_sub_allow": zen_sub_allow,
            "#zen_qos_pub": zen_qos_pub
        },
        remove_comments_flag=False,
        remove_empty_lines_flag=False)
    
    with open(zen_config_file, "w") as f:
        f.write(config_content) 

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--zen-mode', required=True)
    parser.add_argument('-e', '--zen-endpoint-role', required=True)
    parser.add_argument('-t', '--zen-transport', required=True)
    parser.add_argument('-i', '--zen-main-ip', required=True)
    parser.add_argument('-r', '--zen-main-port', required=True)
    parser.add_argument('-f', '--zen-config-file', required=True)
    parser.add_argument('-p', '--zen-pub-allow')
    parser.add_argument('-s', '--zen-sub-allow')
    parser.add_argument('-q', '--zen-qos-pub')
    args = parser.parse_args()

    main(**{k: v for k, v in vars(args).items() if v is not None and v != "None"})

    # print(result)  # This will output the value to stdout, which can be captured in a shell