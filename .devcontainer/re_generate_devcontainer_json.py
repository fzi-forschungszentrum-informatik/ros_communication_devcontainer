#!/usr/bin/env python3

import os
import sys

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
sys.path.append(project_dir)

from ros2docker.utils.files import generate_file_from_template
from ros2docker.utils.getters import get_docker_run_args, get_image_name


def main():
    # Save the contents of the vsc-template-files
    devcontainer_template_file = os.path.join(script_dir, "devcontainer.json.template")
    devcontainer_file = os.path.join(script_dir, "devcontainer.json")

    substitutions = { 
        "#docker_run_args": str(get_docker_run_args()).replace("'", '"')[1:-1], 
        "#image_name": get_image_name(),
    }
    generate_file_from_template(devcontainer_template_file, devcontainer_file, substitutions)

if __name__ == "__main__":
    main()