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

project_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(project_dir)

import utils.devcontainer_utils

def re_generate_docker_run_args():
    project_config = utils.devcontainer_utils.json_to_dict(f"{project_dir}/config/project.json")

    # Define the path to the template file
    template_file_path = f"{project_dir}/config/docker_run_args.json.template"

    # Define the output file path
    output_file_path = f"{project_dir}/docker/run_args.json"

    # Define the substitutions for the variables starting with #
    substitutions = {
        "#dev_container_name": project_config['dev_container_name'],
        "#project_dir": project_dir,
        "#ssh_auth_sock": os.getenv('SSH_AUTH_SOCK')
    }

    # Function to load template and perform substitutions
    def process_template(template_path, substitutions):
        # Read the template content
        with open(template_path, 'r') as file:
            content = file.read()
        # Perform substitutions
        for key, value in substitutions.items():
            content = content.replace(key, value)

        content = utils.devcontainer_utils.remove_comments(content)
        content = utils.devcontainer_utils.remove_empty_lines(content)
        
        return content

    # Process the template
    processed_content = process_template(template_file_path, substitutions)

    # Ensure the directory exists
    os.makedirs(os.path.dirname(output_file_path), exist_ok=True)

    # Check if the file already exists
    file_exists = os.path.exists(output_file_path)

    if file_exists:
        os.remove(output_file_path)

    # Write the processed content to the new file
    with open(output_file_path, 'w') as file:
        file.write(processed_content)

    # Inform the user about the action taken
    if file_exists:
        print("Existing 'run_args.json' was replaced in the 'docker' directory.")
    else:
        print("'run_args.json' was created in the 'docker' directory.")

    # Change the file mode to read-only
    os.chmod(output_file_path, 0o444)


if __name__ == "__main__":
    re_generate_docker_run_args()