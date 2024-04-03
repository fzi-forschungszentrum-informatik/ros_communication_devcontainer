#!/usr/bin/env python3

import os
import sys

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))
routine_dir = os.path.dirname(directory_of_this_script)
machine_dir = os.path.dirname(routine_dir)
workspace_path = os.path.dirname(os.path.dirname(machine_dir))
sys.path.append(workspace_path) # add workspace to sys
from utility.ros_catmux_docker.build_and_run import build_and_run
from utility.machine_utils import machine_main, ensure_machine_environment, get_project_path

def run_control_center(connect_to_target=False, external_terminal=False):
    ensure_machine_environment(__file__, connect_to_target, external_terminal, routine_dir)

    mount_prefix = get_project_path(connect_to_target, machine_dir) + "/bind-mounts"
    container_name = "control_center"
    catmux_parameter_file = f'{directory_of_this_script}/catmux_parameters.yaml'
    additional_run_arguments = f"-v {mount_prefix}{workspace_path}/lightweight_data/pose_msg.txt:/pose_msg.txt"

    build_and_run(container_name, catmux_parameter_file, mount_prefix, additional_run_arguments)

if __name__ == "__main__":
    machine_main(run_control_center)