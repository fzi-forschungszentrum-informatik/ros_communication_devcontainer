#!/usr/bin/env python3

import os
import sys
import subprocess
import shlex
import argparse

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.getters import *

def main(additional_run_arguments='-it', exec_command='bash'):
    # Use shlex.split to safely parse additional_run_arguments and run_command
    additional_run_arguments_parts = shlex.split(additional_run_arguments)

    # If exec_command is a single command (not already wrapped for bash), wrap it
    if isinstance(exec_command, str):
        exec_command_parts = shlex.split(exec_command)
    else:
        exec_command_parts = exec_command  # Assume it's already a list
    
    docker_command = [
        'docker',
        'exec',
        *additional_run_arguments_parts,
        "--user", f"{os.getuid()}:{os.getgid()}",
        get_container_name(),
        *exec_command_parts
    ]

    print("Executing Docker command:", ' '.join(docker_command))
    subprocess.run(docker_command, check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-a', '--additional_run_arguments', help='Docker run arguments')
    parser.add_argument('-c', '--exec_command', help='Command to exec in the Docker container')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
