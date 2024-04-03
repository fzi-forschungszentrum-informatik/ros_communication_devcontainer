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
import sys

workspace_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
sys.path.append(workspace_path)
import utility.workspace_utils as workspace_utils
from utility.ensure_environment import ensure_environment, deserialize_additional_args

def ensure_machine_environment(machine_script, connect_to_target=False, external_terminal=False, routine_dir=None, **additional_args):
    routine_dir = routine_dir or os.path.dirname(os.path.realpath(machine_script))
    machine_dir = os.path.dirname(routine_dir)

    # Load machine data
    machine_config = workspace_utils.json_to_dict(f'{machine_dir}/machine.json')
    target_project_path = machine_config['project_path']
    target_account_key = machine_config['ssh_account_key'].split()
    target_account = workspace_utils.get_ip_or_account(target_account_key, workspace_path)

    ensure_environment(os.path.realpath(machine_script), connect_to_target, target_account, target_project_path, external_terminal, **additional_args)

def machine_main(func):
    parser = argparse.ArgumentParser(description=func.__doc__)
    parser.add_argument("-c", "--connect_to_target", action="store_true", help="Connect to provided target account")
    parser.add_argument("-e", "--external_terminal", action="store_true", help="Launch in external (gnome-)terminal")
    parser.add_argument("-a", "--additional_arguments", help="Additional script specific arguments", type=str, default="")
    args = parser.parse_args()
    additional_args_dict = deserialize_additional_args(args.additional_arguments)

    # Use **vars(args) to integrate additional_args_dict, filtering out None values and replacing 'additional_arguments' string with a dict
    other_args = {k: v for k, v in vars(args).items() if v is not None and k != "additional_arguments"}
    func(**other_args, **additional_args_dict)

def get_machine_project_path(machine_dir):
    machine_config = workspace_utils.json_to_dict(f'{machine_dir}/machine.json')
    return machine_config['project_path']

def get_project_path(for_target_connection, machine_dir=None):
    if for_target_connection:
        return get_machine_project_path(machine_dir)
    else:
        return workspace_utils.get_host_project_path()