#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2024-11-13
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

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
#    * Neither the name of the copyright holder nor the names of its
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
import shlex
import stat


def _logging_before_command(instance_dir=None, config_dir=None, catmux_log_dir=None, rosbag_dir=None):
    exports = {
        "ROSOTACOM_INSTANCE_DIR": instance_dir,
        "ROSOTACOM_CONFIG_DIR": config_dir,
        "ROSOTACOM_CATMUX_LOG_DIR": catmux_log_dir,
        "ROSOTACOM_ROSBAG_DIR": rosbag_dir,
    }
    parts = [f"export {key}={shlex.quote(str(value))}" for key, value in exports.items() if value]
    parts.append("source /ws/session/creation/catmux_log_setup.sh")
    return "; ".join(parts)


def _inject_logging_command(merged_config, logging_command):
    common = merged_config.setdefault('common', {})
    before_commands = common.get('before_commands') or []
    if not isinstance(before_commands, list):
        before_commands = [before_commands]
    before_commands = [cmd for cmd in before_commands if cmd != logging_command]
    common['before_commands'] = [logging_command, *before_commands]


def main(peer_dir, instance_dir=None, config_dir=None, catmux_log_dir=None, rosbag_dir=None):
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

    _inject_logging_command(
        merged_config,
        _logging_before_command(
            instance_dir=instance_dir,
            config_dir=config_dir,
            catmux_log_dir=catmux_log_dir,
            rosbag_dir=rosbag_dir,
        ),
    )

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
    parser.add_argument('--instance-dir')
    parser.add_argument('--config-dir')
    parser.add_argument('--catmux-log-dir')
    parser.add_argument('--rosbag-dir')
    args = parser.parse_args()

    main(
        args.peer_dir,
        instance_dir=args.instance_dir,
        config_dir=args.config_dir,
        catmux_log_dir=args.catmux_log_dir,
        rosbag_dir=args.rosbag_dir,
    )
