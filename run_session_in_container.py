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
import argparse
import re
import yaml
import importlib.util

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from ros2docker.build_run import main as build_run
from ros2docker.utils.getters import get_local_config, get_config_dir

ws_creation_dir = os.path.join(project_dir, "ws", "session", "creation")
session_gen_path = os.path.join(ws_creation_dir, "generate_session_files.py")
spec = importlib.util.spec_from_file_location("session_gen", session_gen_path)
session_gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session_gen)

# hotfix where usage of robot folders leads to problems
# unwanted_path = "/home/carpc/robot_folders/src/robot_folders"
# if unwanted_path in sys.path: 
#     sys.path.remove(unwanted_path)

def _resolve_host_session_dir(session_dir: str) -> str:
    p = os.path.abspath(session_dir)
    if os.path.isdir(p):
        return p

    local_config = get_local_config()
    run_args = local_config.get("run_args", [])
    config_dir = get_config_dir()

    resolved = _map_container_path_to_host(p, run_args, config_dir)
    if resolved and os.path.isdir(resolved):
        return resolved

    raise RuntimeError(
        f"--session-dir must be a directory, got: {p} "
        "and could not map container path via config run_args."
    )


def _map_container_path_to_host(container_path: str, run_args: list, config_dir: str):
    i = 0
    while i < len(run_args):
        arg = run_args[i]
        if arg == "-v" and i + 1 < len(run_args):
            volume = run_args[i + 1]
            host_path, container_mount = volume.split(":", 1)
            if host_path.startswith("../") or host_path.startswith("./"):
                host_path = os.path.realpath(os.path.join(config_dir, host_path))

            if container_path == container_mount or container_path.startswith(container_mount + "/"):
                suffix = container_path[len(container_mount):]
                return os.path.join(host_path, suffix.lstrip("/"))
            i += 2
        else:
            i += 1
    return None


def _load_session_config(session_dir: str) -> dict:
    p = _resolve_host_session_dir(session_dir)

    candidates = [
        "session-parametrization.yaml",
        "session-definition.yaml",
    ]
    for name in candidates:
        fp = os.path.join(p, name)
        if os.path.exists(fp):
            with open(fp, "r") as f:
                param = yaml.safe_load(f) or {}

            # If it's a parametrization, resolve the template so we can access peers.
            if isinstance(param, dict) and "load_template" in param:
                param_dir = os.path.dirname(fp)
                session_template_fs, provided_params = session_gen._parse_session_config_template_spec(
                    param, param_dir
                )
                cfg_raw = session_gen._load_yaml(session_template_fs)
                vars_map = session_gen._build_vars_map_from_template(cfg_raw, provided_params)
                return session_gen._substitute(cfg_raw, vars_map) or {}

            return param

    raise RuntimeError(
        "Missing session config input file in session dir. Expected one of: "
        f"{candidates}. Got dir: {p}"
    )


def _resolve_remote_peer_name(session_dir: str, identity: str) -> str:
    cfg = _load_session_config(session_dir)
    peers = (cfg or {}).get("peers")
    if not isinstance(peers, dict):
        raise RuntimeError("session-config must define a mapping 'peers: { <peer_key>: { ... } }'.")

    peer_keys = list(peers.keys())
    if len(peer_keys) != 2:
        raise RuntimeError(f"Expected exactly 2 peers, got peers={peer_keys}")
    if identity not in peer_keys:
        raise RuntimeError(f"--identity must be one of peers={peer_keys}")

    def _peer_com_name(peer_key: str) -> str:
        v = None
        try:
            v = (peers[peer_key] or {}).get("com-name")
        except Exception:
            v = None
        if v is None:
            return peer_key
        if isinstance(v, str):
            s = v.strip()
            return s if s else peer_key
        return str(v) if v else peer_key

    remote_peer_key = next(k for k in peer_keys if k != identity)
    return _peer_com_name(remote_peer_key)


def _sanitize_container_name(name: str) -> str:
    # Docker container name: allow [a-zA-Z0-9_.-]; replace others with "_"
    return re.sub(r"[^a-zA-Z0-9_.-]", "_", name)


def main(session_dir, identity=None, force=False, rewrite_formatting=False):
    script_path = f"/ws/session/creation/run_session.py"
    docker_command = f"{script_path} --session-dir {session_dir}"
    if identity is not None:
        docker_command += f" --identity {identity}"
    if force:
        docker_command += " --force"
    if rewrite_formatting:
        docker_command += " --rewrite-formatting"

    print(f"Command which will be run in container: {docker_command}")
    remote_peer_name = _resolve_remote_peer_name(session_dir, identity)
    container_name = _sanitize_container_name(f"com_to_{remote_peer_name}")
    build_run(
        override={
            "run_type": "command",
            "command": docker_command,
            "container_name": container_name,
        }
    )

    print("Script execution in container completed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument(
        "-s",
        "--session-dir",
        required=True,
        help="Directory containing a session config input file (session-definition.yaml / session-parametrization.yaml) and where generated files will be written.",
    )
    parser.add_argument("--identity", required=True)
    parser.add_argument("-f", "--force", action="store_true")
    parser.add_argument("--rewrite-formatting", action="store_true")
    args = parser.parse_args()
    main(**{k: v for k, v in vars(args).items() if v is not None})
