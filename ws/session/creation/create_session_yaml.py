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

import yaml
import argparse
import os
import stat

def main(peer_dir):
    spec_file = os.path.join(peer_dir, 'session_specification.yaml')

    with open(spec_file, 'r') as f:
        spec = yaml.safe_load(f)

    plugins = spec['session_plugins']
    parameters = spec.get('parameters', [])


    # Initialize base configuration
    merged_config = {'parameters': {}, 'before-command': {}, 'windows': [], 'common': {}}

    # Reverse the order of plugins
    plugins.reverse()

    # Load and merge each plugin file
    for plugin in plugins:
        plugin_path = os.path.join(peer_dir, plugin)  # Adjust path relative to session peer directory
        with open(plugin_path, 'r') as f:
            plugin_data = yaml.safe_load(f) or {}

            # Merge 'parameters'
            if 'parameters' in plugin_data:
                merged_config['parameters'] = {**merged_config['parameters'], **plugin_data['parameters']}
            
            # Merge 'common'
            if 'common' in plugin_data:
                merged_config['common'] = {**merged_config['common'], **plugin_data['common']}
            
            # Append 'windows'
            if 'windows' in plugin_data:
                merged_config['windows'].extend(plugin_data['windows'])

    # Integrate parameters directly into the configuration
    if parameters:
        merged_config['parameters'] = {**merged_config['parameters'], **parameters}

    # Output file path
    output_file = os.path.join(peer_dir, '.session_readonly.yaml')

    # Remove the output file if it already exists to allow overwriting
    if os.path.exists(output_file):
        os.remove(output_file)

    # Write and make the output file read-only
    with open(output_file, 'w') as f_out:
        yaml.dump(merged_config, f_out, default_flow_style=False)
    os.chmod(output_file, stat.S_IREAD)

    print("Session configuration successfully created.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Merge multiple YAML plugin files into a full session configuration.")
    parser.add_argument('-p', '--peer-dir', required=True, help='Peer directory containing the session_specification.yaml file')
    args = parser.parse_args()

    main(args.peer_dir)
