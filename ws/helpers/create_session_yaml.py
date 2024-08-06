#!/usr/bin/env python3

import yaml
import argparse
import os
import stat

def create_session_yaml(session_dir):
    spec_file = os.path.join(session_dir, 'session_specification.yaml')

    with open(spec_file, 'r') as f:
        spec = yaml.safe_load(f)

    plugins = spec['session_plugins']
    parameters = spec.get('parameters', [])

    # # Convert the list of parameter dictionaries to a single dictionary
    # print(parameters_list)
    # parameters = {}
    # for param in parameters_list:
    #     print("param: " + str(param))
    #     parameters.update(param)

    # Initialize base configuration
    merged_config = {'parameters': {}, 'before-command': {}, 'windows': [], 'common': {}}

    # Reverse the order of plugins
    plugins.reverse()

    # Load and merge each plugin file
    for plugin in plugins:
        plugin_path = os.path.join(session_dir, plugin)  # Adjust path relative to session directory
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

    # Add the session_dir parameter
    merged_config['parameters']['session_dir'] = session_dir

    # Output file path
    output_file = os.path.join(session_dir, '.session_readonly.yaml')

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
    parser.add_argument('-s', '--session-dir', required=True, help='Session directory containing the session_specification.yaml file')
    args = parser.parse_args()

    create_session_yaml(args.session_dir)
