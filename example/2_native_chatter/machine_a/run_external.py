#!/usr/bin/env python3

import os
import sys

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))
repository_path = os.path.dirname(os.path.dirname(os.path.dirname(directory_of_this_script)))
sys.path.append(repository_path)

from ros2docker.build_run import main as build_run

def run():
    config = f'{directory_of_this_script}/external_ros2docker_config.json'
    build_run(config)

if __name__ == "__main__":
    run()