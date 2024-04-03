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
import shlex

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))
workspace_path = os.path.dirname(os.path.dirname(directory_of_this_script))
sys.path.append(workspace_path)

def build_and_run(container_name, catmux_parameter_file, mount_prefix, additional_run_arguments):

    # Docker build command to get image ID (quiet mode)
    docker_build_cmd = f"sudo docker build -q {directory_of_this_script}"
    print("Executing Docker command:", ' '.join(docker_build_cmd))
    image_id = subprocess.check_output(docker_build_cmd, shell=True).decode().strip()
    docker_run_cmd = [
        'sudo',
        'docker',
        'run',

        # basic settings
        '-it',
        '--rm',
        "--network", "host",
        "--name", container_name,
        "-e", f"CATMUX_SESSION_NAME={container_name}",

        # standard mounts
        "-v", f"{mount_prefix}{directory_of_this_script}/run_catmux_session.py:/run_catmux_session.py",
        "-v", f"{mount_prefix}{directory_of_this_script}/session.yaml:/session.yaml",
        "-v", f"{mount_prefix}{catmux_parameter_file}:/catmux_parameters.yaml",

        # user settings
        *shlex.split(additional_run_arguments),

        # Docker image and command
        image_id, "/run_catmux_session.py"
        # image_id, "tail", "-f", "/dev/null"
    ]

    print("Executing Docker command:", ' '.join(docker_run_cmd))
    subprocess.run(docker_run_cmd, check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--container_name', required=True)
    parser.add_argument('-f', '--catmux_parameter_file', required=True)
    parser.add_argument('-a', '--additional_run_arguments', help='Docker run arguments')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    build_and_run(**{k: v for k, v in vars(args).items() if v is not None})
