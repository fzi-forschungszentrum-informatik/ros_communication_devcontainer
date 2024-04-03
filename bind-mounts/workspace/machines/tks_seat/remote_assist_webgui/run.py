#!/usr/bin/env python3

import os
import sys

routine_dir = os.path.dirname(os.path.realpath(__file__))
machine_dir = os.path.dirname(routine_dir)
workspace_path = os.path.dirname(os.path.dirname(machine_dir))
sys.path.append(workspace_path) # add workspace to sys
from utility.foxglove_webgui_docker.run import run
from utility.machine_utils import machine_main, ensure_machine_environment, get_project_path
from utility.workspace_utils import get_ip_or_account

def run_foxglove_web_app(connect_to_target=False, external_terminal=False):
    ensure_machine_environment(__file__, connect_to_target, external_terminal, routine_dir)

    with open(f'{routine_dir}/control_center_ip_key', 'r') as file:
        ip_key_str = file.read()
    control_center_ip = get_ip_or_account(ip_key_str.split(), workspace_path)

    container_name = "remote_assist_webgui"
    mount_prefix = get_project_path(connect_to_target, machine_dir) + "/bind-mounts"
    foxglove_layout_path = f"{routine_dir}/foxglove_layout.json"

    run(container_name, foxglove_layout_path, control_center_ip, mount_prefix)

if __name__ == "__main__":
    machine_main(run_foxglove_web_app)