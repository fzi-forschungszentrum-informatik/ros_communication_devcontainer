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
# \date    2024-04-03
#
#
# ---------------------------------------------------------------------

import argparse
import os
import subprocess
import sys

workspace_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(workspace_path)
from utility.workspace_utils import json_to_dict

def find_workspace(current_dir):
    """
    Find the workspace directory by iterating through parent directories.
    Stops when it finds a directory named 'workspace' or reaches the root directory.
    """
    root_directory = os.path.abspath(os.sep)
    while True:
        workspace_dir_candidate = os.path.join(current_dir, "workspace")
        if os.path.basename(current_dir) == "workspace":  # Check if the current_dir itself is the workspace
            return current_dir
        elif os.path.exists(workspace_dir_candidate):
            return workspace_dir_candidate
        elif current_dir == root_directory:  # Stop if we have reached the root directory
            break
        current_dir = os.path.dirname(current_dir)
    raise FileNotFoundError("Workspace directory not found.")


def arguments_to_string(connect_to_target=False, external_terminal=False, **additional_arguments):
    """
    Constructs a string of arguments for command line execution.
    Includes a leading space if there are any arguments.
    """
    arguments = []
    if connect_to_target == True:
        arguments.append("-c")
    if external_terminal == True:
        arguments.append("-e")
    if additional_arguments:
        additional_args_str = ' '.join([f"{k}={v}" for k, v in additional_arguments.items()])
        arguments.append(f"-a '{additional_args_str}'")
    
    arguments_string = " ".join(arguments)
    if arguments_string:
        return " " + arguments_string
    else:
        return ""

def deserialize_additional_args(additional_args_str):
    """
    Deserialize the 'additional_arguments' string into a dictionary.
    """
    args_dict = {}
    if additional_args_str:
        for item in additional_args_str.split(' '):
            key, value = item.split('=')
            args_dict[key] = value
    return args_dict

def ensure_environment(script_path, connect_to_target=False, target_account=None, target_project_path=None, external_terminal=False, **additional_arguments):
    # Find the workspace directory
    workspace_dir = find_workspace(script_path)

    # Determine the relative path to the script from the workspace
    workspace_to_script_path = os.path.relpath(script_path, start=os.path.dirname(workspace_dir))

    print(f"-- Ensuring Evironment for {script_path} --")

    if os.getenv("INSIDE_CONTAINER") != "true":
        print("Current status: Not ins container. Restarting Script inside container...")
        # Load project config
        project_dir = os.path.dirname(os.path.dirname(workspace_dir))
        project_config = json_to_dict(f"{project_dir}/config/project.json")

        print(f"Checking if the container '{project_config['dev_container_name']}' is running...")
        container_not_running = subprocess.run(
            f'docker ps -q -f name="{project_config["dev_container_name"]}" | grep -q .',
            shell=True,
            check=False
        ).returncode != 0

        if container_not_running:
            print("Container not running. Building and starting the container...")
            subprocess.run([f"{project_dir}/docker/build_up.py"], check=True)
        else:
            print("Container is already running.")

        container_script_path = f"/{workspace_to_script_path}"
        args = arguments_to_string(connect_to_target, external_terminal, **additional_arguments)
        docker_command = f"{container_script_path}{args}"
        print(f"Executing script in container with command: {docker_command}")
        subprocess.run([f"{project_dir}/docker/exec.py", '-c', docker_command], check=True)
        print("Script execution in container completed.")
        sys.exit()
    else:
        print("Current status: Inside container")

    if external_terminal:
        print("External Terminal requested. Restarting script inside new gnome-terminal...")
        args = arguments_to_string(connect_to_target, False, **additional_arguments)
        gnome_terminal_command = f'gnome-terminal -- bash -c "{script_path}{args}"'
        print(f"Executing in gnome-terminal with command: {gnome_terminal_command}")
        print(gnome_terminal_command)
        subprocess.run(gnome_terminal_command, shell=True, check=True)
        print("Script execution in external gnome-terminal initiated.")
        sys.exit()

    if connect_to_target:
        print("SSH connection to target requested. Restarting script inside ssh-connection...")
        target_script_path = os.path.join(target_project_path, "bind-mounts", workspace_to_script_path)
        args = arguments_to_string(False, False, **additional_arguments)
        ssh_command = f'ssh {target_account} -o ForwardAgent=yes -t "{target_script_path}{args}"'
        print(f"Executing SSH command: {ssh_command}")
        try:
            subprocess.run(ssh_command, shell=True, check=True)
            print("SSH command executed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to execute SSH command. Error: {e.stderr.decode()}")
        sys.exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Helper to ensure that the provided script is executed in the desired environment.')
    parser.add_argument("-c", "--connect_to_target", action="store_true", help="Connect to provided target account")
    parser.add_argument("-t", "--target_account", help="Target account to be connected to")
    parser.add_argument("-p", "--target_project_path", help="Path to project directory on target")
    parser.add_argument("-e", "--external_terminal", action="store_true", help="Launch in external (gnome-)terminal")
    parser.add_argument("-a", "--additional_arguments", help="Additional script specific arguments", type=str, default="")
    args = parser.parse_args()
    additional_args_dict = deserialize_additional_args(args.additional_arguments)

    # Use **vars(args) to integrate additional_args_dict, filtering out None values and replacing 'additional_arguments' string with a dict
    ensure_environment_args = {k: v for k, v in vars(args).items() if v is not None and k != "additional_arguments"}
    ensure_environment(**ensure_environment_args, **additional_args_dict)