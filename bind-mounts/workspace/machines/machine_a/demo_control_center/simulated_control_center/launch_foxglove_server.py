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
import subprocess

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))
routine_dir = os.path.dirname(directory_of_this_script)
machine_dir = os.path.dirname(routine_dir)
workspace_path = os.path.dirname(os.path.dirname(machine_dir))
sys.path.append(workspace_path) # add workspace to sys
from utility.machine_utils import machine_main, ensure_machine_environment, get_project_path

def launch_foxglove_server(connect_to_target=False, external_terminal=False):
    ensure_machine_environment(__file__, connect_to_target, external_terminal, routine_dir)

    mount_prefix = get_project_path(connect_to_target, machine_dir) + "/bind-mounts"
    layout_path = f"{directory_of_this_script}/foxglove_layout.json"
    docker_command = f'sudo docker run --name foxglove_server_remote_assistance --rm -p "8080:8080" -v {mount_prefix}{layout_path}:/foxglove/default-layout.json ghcr.io/foxglove/studio:latest'
    # Execute the docker command
    try:
        subprocess.run(docker_command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to run Docker command: {e}")

if __name__ == "__main__":
    machine_main(launch_foxglove_server)