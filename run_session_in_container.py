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
# \date    2024-11-13
#
#
# ---------------------------------------------------------------------

import os
import sys
import subprocess
import argparse

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from utils.getters import *
from docker_.build_run import main as build_run
from docker_.exec import main as exec

# hotfix where usage of robot folders leads to problems
unwanted_path = "/home/carpc/robot_folders/src/robot_folders"
if unwanted_path in sys.path: 
    sys.path.remove(unwanted_path)

def main(session_dir, external_terminal=False):
    container_name = get_container_name()
    print(f"Checking if the container '{container_name}' is running...")
    container_not_running = subprocess.run( f'docker ps -q -f name="{container_name}" | grep -q .', shell=True, check=False).returncode != 0

    if external_terminal: 
        script_path = f"/ws/session/creation/run_session_in_external_terminal.py"
    else: 
        script_path = f"/ws/session/creation/run_session.py"
    docker_command = f"{script_path} --session-dir {session_dir}"
    print(f"Command which will be run/executed in container: {docker_command}")

    if container_not_running:
        print("Container not running. Building and running the container with the command...")
        build_run(run_command=["bash", "-i", "-c", docker_command])
    else:
        print("Container is already running. Directly executing command...")
        exec(exec_command=["bash", "-i", "-c", docker_command])

    print("Script execution in container completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("-s", "--session-dir")
    parser.add_argument("-e", "--external-terminal", action="store_true", help="Launch in external (gnome-)terminal")
    args = parser.parse_args()
    main(**{k: v for k, v in vars(args).items() if v is not None})
