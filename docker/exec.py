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

import os
import sys
import argparse
import shlex
import subprocess

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.devcontainer_utils import json_to_dict

def exec(exec_arguments="-it", exec_command="bash"):
    project_config = json_to_dict(f"{project_dir}/config/project.json")

    # Safely split both exec_arguments and exec_command to handle complex shell syntax
    exec_arguments_parts = shlex.split(exec_arguments)
    command_parts = shlex.split(exec_command)

    # Build the full command list for subprocess.run
    full_command = ['docker', 'exec'] + exec_arguments_parts + [project_config['dev_container_name']] + command_parts

    subprocess.run(full_command, check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a Docker container with specified arguments.")
    parser.add_argument('-a', '--exec_arguments', help='Docker run arguments')
    parser.add_argument('-c', '--exec_command', help='Command to run in the Docker container')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    exec(**{k: v for k, v in vars(args).items() if v is not None})