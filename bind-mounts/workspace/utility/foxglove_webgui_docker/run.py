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

directory_of_this_script = os.path.dirname(os.path.realpath(__file__))

def is_container_running(container_name):
    """Check if the specified Docker container is running."""
    result = subprocess.run(['sudo', 'docker', 'ps', '--filter', f"name={container_name}", '--format', '{{.Names}}'],
                            capture_output=True, text=True)
    return container_name in result.stdout.strip()


def run(container_name, foxglove_layout_path="", websocket_ip="", mount_prefix=""):
    if not is_container_running(container_name):
        # Ensure the path is absolute or correctly mapped to the Docker context

        # Construct the Docker command
        dind_run = [
            'sudo', 'docker', 'run',

            # basic settings
            '-it',
            '--rm',
            "--privileged",
            "--network", "host",
            "--name", container_name,
            "-d",

            # display
            "-v", "/tmp/.X11-unix:/tmp/.X11-unix",
            "-e", "DISPLAY",

            # parameters
            "-e", f"WEBSOCKET_IP={websocket_ip}",
            "-v", f"{mount_prefix}{foxglove_layout_path}:/foxglove-layout.json",

            # necessary for rviz for some clients
            "--security-opt", "apparmor=unconfined",

            # for using the google chrome browser inside the container
            # see: https://stackoverflow.com/a/53975412
            "--security-opt", "seccomp=unconfined",

            # '-v', '/dev/shm:/dev/shm',
            '-v', '/var/run/docker.sock:/var/run/docker.sock',
            "-v", f"{mount_prefix}{directory_of_this_script}/run_commands.sh:/run_commands.sh",

            # Docker image
            'docker:dind'
        ]

        # Execute the Docker command
        print("Executing Docker command:", ' '.join(dind_run))
        subprocess.run(dind_run, check=True)
    subprocess.run(["sudo", "docker", "exec", "-it", container_name, "sh", "/run_commands.sh"], check=True)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--container_name', required=True)
    parser.add_argument('-f', '--foxglove_layout_path')
    parser.add_argument('-i', '--websocket_ip')
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    run(**{k: v for k, v in vars(args).items() if v is not None})
