#!/usr/bin/env python3

import os
import sys
import argparse

ws_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(ws_path)
unwanted_path = "/home/carpc/robot_folders/src/robot_folders"
if unwanted_path in sys.path: 
    sys.path.remove(unwanted_path)
from helpers.create_session import create_session
from helpers.ensure_environment import ensure_environment, deserialize_additional_args


def run(session_dir, connect_to_target=False, external_terminal=False):
    # TODO: Handle accounts and project path properly
    # Load machine data
    # machine_config = ws_utils.json_to_dict(f'{routine_dir}/machine.json')
    # target_project_path = machine_config['project_path']
    # target_account_key = machine_config['ssh_account_key'].split()
    # target_account = ws_utils.get_adress(target_account_key, ws_path)
    target_account = None
    target_project_path = None
    additional_args = {"session_dir":session_dir}

    ensure_environment(os.path.realpath(__file__), connect_to_target, target_account, target_project_path, external_terminal, **additional_args)

    create_session(session_dir)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=run.__doc__)
    parser.add_argument("-s", "--session_dir", help="Connect to provided target account")
    parser.add_argument("-c", "--connect_to_target", action="store_true", help="Connect to provided target account")
    parser.add_argument("-e", "--external_terminal", action="store_true", help="Launch in external (gnome-)terminal")
    parser.add_argument("-a", "--additional_arguments", help="Additional script specific arguments", type=str, default="")
    args = parser.parse_args()
    additional_args_dict = deserialize_additional_args(args.additional_arguments)

    # Use **vars(args) to integrate additional_args_dict, filtering out None values and replacing 'additional_arguments' string with a dict
    other_args = {k: v for k, v in vars(args).items() if v is not None and k != "additional_arguments"}

    # run(**other_args)
    run(**other_args, **additional_args_dict)
