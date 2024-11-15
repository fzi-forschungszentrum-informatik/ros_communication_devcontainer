#!/usr/bin/env python3

import os
import sys

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))
repository_path = os.path.dirname(os.path.dirname(os.path.dirname(directory_of_this_script)))
sys.path.append(repository_path)

from example.ros_catmux_docker.build_and_run import main as build_and_run

def run():
    container_name = "ros_master_and_listener"
    catmux_session_file = f'{directory_of_this_script}/master_session.yaml'

    build_and_run(container_name, catmux_session_file)

if __name__ == "__main__":
    run()