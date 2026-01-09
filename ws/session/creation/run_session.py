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

import argparse
import yaml
import subprocess
import sys
import os
from typing import Optional

ws_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(ws_path)

from session.creation.create_session_yaml import main as create_session_yaml
from session.creation.generate_session_files import func as generate_session_files


def _resolve_peer_dir(
    session_dir: str, identity: Optional[str], force: bool, rewrite_formatting: bool
) -> str:
    """
    Resolve a runnable session directory (the directory containing session_specification.yaml).

    Supported input:
    - session directory that contains one of:
      - session-definition.yaml (self-contained)
      - session-parametrization.yaml (template + parameters)
      plus: --identity <peer_key>
    """
    p = os.path.abspath(session_dir)
    if not os.path.isdir(p):
        raise RuntimeError(f"--session-dir must be a directory, got: {p}")

    if not identity:
        raise RuntimeError(
            "Missing --identity. Provide --identity <name of the peer>."
        )

    # Resolve which session config input file to use.
    #
    # Precedence:
    # - session-parametrization.yaml (if present): treated as the "primary" input even if a generated
    #   session-definition.yaml also exists.
    # - session-definition.yaml
    candidates = [
        "session-parametrization.yaml",
        "session-definition.yaml",
    ]
    param_yaml = None
    for name in candidates:
        fp = os.path.join(p, name)
        if os.path.exists(fp):
            param_yaml = fp
            break
    if not param_yaml:
        raise RuntimeError(
            "Missing session config input file in session dir. Expected one of: "
            f"{candidates}. Got dir: {p}"
        )

    generate_session_files(
        session_config_yaml=param_yaml,
        force=force,
        rewrite_formatting=rewrite_formatting,
    )
    peer_dir = os.path.join(p, identity)
    spec_file = os.path.join(peer_dir, "session_specification.yaml")
    if not os.path.exists(spec_file):
        # Try to help with a quick "what identities exist" hint.
        try:
            subdirs = sorted(
                d for d in os.listdir(p) if os.path.isdir(os.path.join(p, d)) and not d.startswith(".")
            )
        except Exception:
            subdirs = []
        hint = f" Available subdirs: {subdirs}" if subdirs else ""
        raise RuntimeError(f"Identity '{identity}' did not resolve to a runnable peer dir: missing {spec_file}.{hint}")
    return peer_dir


def main(
    session_dir: str,
    identity: Optional[str] = None,
    force: bool = False,
    rewrite_formatting: bool = False,
):

    peer_dir = _resolve_peer_dir(session_dir, identity, force, rewrite_formatting)

    # Ensure merged .session_readonly.yaml exists for catmux
    create_session_yaml(peer_dir)

    # Define the command and arguments
    command = "catmux_create_session"
    yaml_file_path = f"{peer_dir}/.session_readonly.yaml"
    session_name_arg = "--session_name"

    # get name
    spec_file = os.path.join(peer_dir, "session_specification.yaml")
    with open(spec_file, "r") as f:
        spec = yaml.safe_load(f)
    session_name = (spec or {}).get("name", "ros_communication")

    # Combine them into a single command
    full_command = [
        command,
        yaml_file_path,
        session_name_arg,
        session_name,
        "--overwrite",
        f"dir_path={peer_dir}",
    ]

    # Execute the command
    try:
        subprocess.run(full_command, check=True)
        print("Command executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"An error occurred while executing the command: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate peer session files from a session definition/parametrization and launch catmux for one identity."
    )
    parser.add_argument(
        "-s",
        "--session-dir",
        required=True,
        help="Directory containing session-definition.yaml or session-parametrization.yaml.",
    )
    parser.add_argument(
        "--identity",
        required=True,
        type=str,
        help="Which peer to launch.",
    )
    parser.add_argument(
        "-f",
        "--force",
        action="store_true",
        help="If generating session files: overwrite existing files even if they differ semantically.",
    )
    parser.add_argument(
        "--rewrite-formatting",
        action="store_true",
        help="If generating session files: rewrite files even when semantically equal (format-only differences).",
    )
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})