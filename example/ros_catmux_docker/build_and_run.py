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

def main(container_name, catmux_session_file, additional_run_arguments="", catmux_params=""):
    # Docker build command to get image ID (quiet mode)
    docker_build_cmd = f"docker build -q {directory_of_this_script}"
    print("Executing Docker command:", ' '.join(docker_build_cmd))
    image_id = subprocess.check_output(docker_build_cmd, shell=True).decode().strip()
    inner_command="source /opt/ros/noetic/setup.bash && catmux_create_session /session.yaml"
    if catmux_params: inner_command += f" --overwrite {catmux_params}"
    run_command=f'/bin/bash -c "{inner_command}"'
    docker_run_cmd = [
        'docker',
        'run',

        # basic settings
        '-it',
        '--rm',
        "--network", "host",
        "--name", container_name,

        # standard mounts
        "-v", f"{catmux_session_file}:/session.yaml",

        # user settings
        *shlex.split(additional_run_arguments),

        # Docker image and command
        image_id, *shlex.split(run_command)
        # image_id, "tail", "-f", "/dev/null"
    ]

    print("Executing Docker command:", ' '.join(docker_run_cmd))
    subprocess.run(docker_run_cmd, check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--container_name', required=True)
    parser.add_argument('-f', '--catmux_session_file', required=True)
    parser.add_argument('-a', '--additional_run_arguments', help='Docker run arguments')
    parser.add_argument('-p', '--catmux_params')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
