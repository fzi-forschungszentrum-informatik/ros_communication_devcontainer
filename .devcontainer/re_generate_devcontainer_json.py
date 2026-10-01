#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

import os
import stat

script_dir = os.path.dirname(os.path.abspath(__file__))
project_dir = os.path.dirname(script_dir)
config_path = os.path.join(project_dir, "ros2docker.json")

from ros2docker.commands import make_run_command
from ros2docker.config import load_config, strip_json_comments


def generate_file_from_template(template_path, output_path, substitutions):
    with open(template_path, "r", encoding="utf-8") as file:
        content = file.read()
    for key, value in substitutions.items():
        content = content.replace(key, value)
    content = strip_json_comments(content)
    content = "\n".join(line for line in content.splitlines() if line.strip())
    if os.path.exists(output_path):
        os.remove(output_path)
    with open(output_path, "w", encoding="utf-8") as file:
        file.write(content)
    os.chmod(output_path, stat.S_IRUSR | stat.S_IRGRP | stat.S_IROTH)


def main():
    # Save the contents of the vsc-template-files
    devcontainer_template_file = os.path.join(script_dir, "devcontainer.json.template")
    devcontainer_file = os.path.join(script_dir, "devcontainer.json")

    local_config = load_config(config_path)
    run_command = make_run_command(config_path, override={"run_type": "up"})
    image_name = local_config.get("image_name", "ros2docker")
    image_index = run_command.index(image_name)
    docker_run_args = run_command[2:image_index]
    if docker_run_args and docker_run_args[-1] == "-d":
        docker_run_args = docker_run_args[:-1]

    substitutions = {
        "#docker_run_args": str(docker_run_args).replace("'", '"')[1:-1],
        "#image_name": str(image_name),
    }
    generate_file_from_template(devcontainer_template_file, devcontainer_file, substitutions)

if __name__ == "__main__":
    main()
