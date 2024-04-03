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
import json

from re_generate_docker_run_args import re_generate_docker_run_args

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

from utils.devcontainer_utils import json_to_dict

def re_generate_devcontainer_json():
    # Path to your files
    docker_run_args_path = f'{project_dir}/docker/run_args.json'
    devcontainer_template_path = f'{project_dir}/config/devcontainer.json.template'
    devcontainer_output_path = f'{project_dir}/.devcontainer/devcontainer.json'

    # Read the run_args.json
    re_generate_docker_run_args()
    docker_run_args_dict = json_to_dict(docker_run_args_path)

    # Rename Key
    docker_run_args_dict['runArgs'] = docker_run_args_dict['run_args']
    del docker_run_args_dict['run_args']
    # Convert the template JSON back to a string for manipulation
    docker_run_args = json.dumps(docker_run_args_dict, indent=4)
    # remove the outer brackets by removing the first and last line from the string
    docker_run_args_no_brackets = '\n'.join(docker_run_args.split('\n')[1:-1])

    # Read the devcontainer.json.template
    with open(devcontainer_template_path, 'r') as file:
        devcontainer_template = file.read()

    # Substitute the #runArgs placeholder with actual runArgs
    devcontainer_content = devcontainer_template.replace("#runArgs", docker_run_args_no_brackets)

    # Check if the file already exists
    file_exists = os.path.exists(devcontainer_output_path)

    if file_exists:
        os.remove(devcontainer_output_path)

    # Write the processed content to the new file
    with open(devcontainer_output_path, 'w') as file:
        file.write(devcontainer_content)

    # Inform the user about the action taken
    if file_exists:
        print("Existing 'devcontainer.json' was replaced in the '.devcontainer' directory.")
    else:
        print("'devcontainer.json' was created in the '.devcontainer' directory.")

    # Change the file mode to read-only
    os.chmod(devcontainer_output_path, 0o444)


if __name__ == "__main__":
    re_generate_devcontainer_json()