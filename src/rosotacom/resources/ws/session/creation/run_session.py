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

import argparse
import copy
import os
import re
import subprocess
import sys
from typing import Dict, Optional

import yaml

ws_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(ws_path)

from session.creation.create_session_yaml import main as create_session_yaml
from session.creation import generate_session_files as session_gen

_CATMUX_NO_SERVER_RE = re.compile(
    r"^error connecting to /tmp/tmux-\d+/catmux \(No such file or directory\)$"
)
# catmux's tmux wrapper swallows failed tmux calls: it prints this marker to
# stdout and keeps going (exit 0) instead of raising. Treat it as a launch
# failure so a half-built session surfaces loudly instead of silently.
_CATMUX_SWALLOWED_ERROR = "Error while calling"


def _report_catmux_failure(full_command, result, *, swallowed: bool, failure_log_path) -> None:
    reason = (
        "catmux swallowed a tmux error and left the session incomplete"
        if swallowed and result.returncode == 0
        else f"catmux_create_session exited with status {result.returncode}"
    )
    # Print to stdout: the stderr traceback can be captured/dropped by the layers
    # above (the smoke harness), so keep the cause next to the failure on stdout.
    detail = (result.stderr or "").strip() or (result.stdout or "").strip()
    print("=" * 72)
    print(f"catmux session launch FAILED: {reason}.")
    if detail:
        print("--- catmux output ---")
        print(detail)
        print("--- end catmux output ---")
    if failure_log_path:
        try:
            os.makedirs(os.path.dirname(failure_log_path) or ".", exist_ok=True)
            with open(failure_log_path, "w") as handle:
                handle.write("command: " + " ".join(str(part) for part in full_command) + "\n")
                handle.write(f"returncode: {result.returncode}\n\n")
                handle.write("===== stdout =====\n")
                handle.write(result.stdout or "")
                handle.write("\n===== stderr =====\n")
                handle.write(result.stderr or "")
            print(f"Full catmux launch log: {failure_log_path}")
        except OSError as exc:
            print(f"(could not write catmux launch log to {failure_log_path}: {exc})")
    print("=" * 72)


def _run_catmux(full_command, attach: bool, failure_log_path=None) -> None:
    if attach:
        subprocess.run(full_command, check=True)
        return

    result = subprocess.run(full_command, text=True, capture_output=True)
    if result.stdout:
        print(result.stdout, end="")

    stderr_lines = [
        line
        for line in (result.stderr or "").splitlines()
        if not _CATMUX_NO_SERVER_RE.match(line.strip())
    ]
    if stderr_lines:
        print("\n".join(stderr_lines), file=sys.stderr)

    swallowed = _CATMUX_SWALLOWED_ERROR in (result.stdout or "")
    if result.returncode != 0 or swallowed:
        _report_catmux_failure(
            full_command, result, swallowed=swallowed, failure_log_path=failure_log_path
        )
        raise subprocess.CalledProcessError(
            result.returncode or 1, full_command, output=result.stdout, stderr=result.stderr
        )


def _parse_peer_address_overrides(overrides) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for override in overrides or []:
        peer_key, sep, address_value = (override or "").partition("=")
        peer_key = peer_key.strip()
        address_value = address_value.strip()
        if sep != "=" or not peer_key or not address_value:
            raise RuntimeError(
                "--peer-address must use '<peer_key>=<address>', "
                "for example 'a=192.168.1.10'."
            )
        if peer_key in result:
            raise RuntimeError(f"Duplicate --peer-address override for peer '{peer_key}'.")
        result[peer_key] = address_value
    return result


def _load_session_config_input(session_config_yaml: str) -> Dict:
    param_dir = os.path.dirname(session_config_yaml)
    param = session_gen._load_yaml(session_config_yaml)
    if not isinstance(param, dict):
        raise RuntimeError(
            f"Session config input YAML must be a mapping, got {type(param)} for '{session_config_yaml}'"
        )

    if "load_template" in param:
        session_template_fs, provided_params = session_gen._parse_session_config_template_spec(param, param_dir)
        cfg_raw = session_gen._load_yaml(session_template_fs)
        vars_map = session_gen._build_vars_map_from_template(cfg_raw, provided_params)
        cfg = session_gen._substitute(cfg_raw, vars_map) or {}
    else:
        cfg = dict(param)

    session_gen._validate_session_template_cfg(cfg)
    return cfg


def _apply_link_trace_overrides(
    cfg: Dict,
    *,
    link_trace: Optional[bool] = None,
    link_trace_interval_s: Optional[float] = None,
    link_trace_modem_command: Optional[str] = None,
) -> Dict:
    if link_trace is None and link_trace_interval_s is None and link_trace_modem_command is None:
        return cfg
    updated = copy.deepcopy(cfg)
    shared = updated.setdefault("shared", {})
    if not isinstance(shared, dict):
        raise RuntimeError("shared must be a mapping before applying link trace overrides.")
    trace_cfg = shared.setdefault("link_trace", {})
    if not isinstance(trace_cfg, dict):
        raise RuntimeError("shared.link_trace must be a mapping before applying link trace overrides.")
    shared["use_status_overview"] = True
    trace_cfg["enabled"] = True if link_trace is None else bool(link_trace)
    if link_trace_interval_s is not None:
        trace_cfg["interval_s"] = float(link_trace_interval_s)
    if link_trace_modem_command is not None:
        trace_cfg["modem_metrics_command"] = link_trace_modem_command
    session_gen._validate_session_template_cfg(updated)
    return updated


def _resolve_peer_dir(
    session_dir: str,
    output_dir: Optional[str],
    identity: Optional[str],
    force: bool,
    rewrite_formatting: bool,
    peer_address=None,
    link_trace: Optional[bool] = None,
    link_trace_interval_s: Optional[float] = None,
    link_trace_modem_command: Optional[str] = None,
) -> str:
    """
    Resolve a runnable session directory (the directory containing session_specification.yaml).

    Supported input:
    - session directory that contains one of:
      - session-definition.yaml (self-contained)
      - session-parametrization.yaml (template + parameters)
      plus: --identity <peer_key>
    """
    if not os.path.isabs(session_dir):
        for base in (os.environ.get("SESSION_DEFINITIONS_DIR"), os.environ.get("SESSION_CONFIGS_DIR")):
            if not base:
                continue
            candidate = os.path.join(base, session_dir)
            if os.path.isdir(candidate):
                session_dir = candidate
                break

    p = os.path.abspath(session_dir)
    if not os.path.isdir(p):
        raise RuntimeError(f"--session-dir must be a directory, got: {p}")
    out = os.path.abspath(output_dir) if output_dir else p
    os.makedirs(out, exist_ok=True)

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

    cfg_for_generation = _apply_link_trace_overrides(
        _load_session_config_input(param_yaml),
        link_trace=link_trace,
        link_trace_interval_s=link_trace_interval_s,
        link_trace_modem_command=link_trace_modem_command,
    )
    peer_addresses = _parse_peer_address_overrides(peer_address)
    known_peers = set((cfg_for_generation.get("peers") or {}).keys())
    unknown = sorted(set(peer_addresses) - known_peers)
    if unknown:
        raise RuntimeError(
            f"--peer-address references unknown peer(s) {unknown}. Known peers: {sorted(known_peers)}"
        )
    session_gen.func(
        session_config_yaml=param_yaml,
        force=force,
        rewrite_formatting=rewrite_formatting,
        session_config_obj=cfg_for_generation,
        output_dir=out,
        write_resolved_definition=True,
        peer_addresses=peer_addresses,
    )
    peer_dir = os.path.join(out, identity)
    spec_file = os.path.join(peer_dir, "session_specification.yaml")
    if not os.path.exists(spec_file):
        # Try to help with a quick "what identities exist" hint.
        try:
            subdirs = sorted(d for d in os.listdir(out) if os.path.isdir(os.path.join(out, d)) and not d.startswith("."))
        except Exception:
            subdirs = []
        hint = f" Available subdirs: {subdirs}" if subdirs else ""
        raise RuntimeError(f"Identity '{identity}' did not resolve to a runnable peer dir: missing {spec_file}.{hint}")
    return peer_dir


def main(
    session_dir: str,
    identity: Optional[str] = None,
    output_dir: Optional[str] = None,
    instance_dir: Optional[str] = None,
    catmux_log_dir: Optional[str] = None,
    rosbag_dir: Optional[str] = None,
    force: bool = False,
    rewrite_formatting: bool = False,
    peer_address=None,
    link_trace: Optional[bool] = None,
    link_trace_interval_s: Optional[float] = None,
    link_trace_modem_command: Optional[str] = None,
    attach: Optional[bool] = None,
):

    peer_dir = _resolve_peer_dir(
        session_dir,
        output_dir,
        identity,
        force,
        rewrite_formatting,
        peer_address=peer_address,
        link_trace=link_trace,
        link_trace_interval_s=link_trace_interval_s,
        link_trace_modem_command=link_trace_modem_command,
    )

    # Ensure merged .session_readonly.yaml exists for catmux
    config_dir = output_dir or os.path.dirname(peer_dir)
    create_session_yaml(
        peer_dir,
        instance_dir=instance_dir or os.environ.get("ROSOTACOM_INSTANCE_DIR"),
        config_dir=config_dir,
        catmux_log_dir=catmux_log_dir or os.environ.get("ROSOTACOM_CATMUX_LOG_DIR"),
        rosbag_dir=rosbag_dir or os.environ.get("ROSOTACOM_ROSBAG_DIR"),
    )

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
    if attach is None:
        attach = sys.stdin.isatty() and sys.stdout.isatty()
    if not attach:
        full_command.append("--detach")

    resolved_catmux_log_dir = catmux_log_dir or os.environ.get("ROSOTACOM_CATMUX_LOG_DIR")
    failure_log_path = os.path.join(
        resolved_catmux_log_dir or peer_dir, "catmux-launch-failure.log"
    )

    # Execute the command
    try:
        _run_catmux(full_command, attach, failure_log_path=failure_log_path)
        print("Command executed successfully.")
    except subprocess.CalledProcessError as e:
        print(
            f"catmux_create_session failed (exit {e.returncode}); see the catmux "
            f"output above and {failure_log_path}.",
            file=sys.stderr,
        )
        raise SystemExit(e.returncode)

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
        "--output-dir",
        help="Directory where generated runtime session files are written.",
    )
    parser.add_argument(
        "--instance-dir",
        help="Runtime session instance directory.",
    )
    parser.add_argument(
        "--catmux-log-dir",
        help="Directory where catmux pane logs for this peer are written.",
    )
    parser.add_argument(
        "--rosbag-dir",
        help="Directory where rosbags for this peer should be written.",
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
    parser.add_argument(
        "--peer-address",
        action="append",
        default=[],
        metavar="PEER=ADDRESS",
        help="Resolved peer address. Pass once for every logical peer.",
    )
    parser.add_argument(
        "--link-trace",
        action="store_true",
        default=None,
        help="Enable link_trace.jsonl recording for this generated session instance.",
    )
    parser.add_argument(
        "--link-trace-interval",
        dest="link_trace_interval_s",
        type=float,
        help="Link trace sample interval in seconds; enables link tracing.",
    )
    parser.add_argument(
        "--link-trace-modem-command",
        dest="link_trace_modem_command",
        help="Shell command returning a JSON object to merge under each trace row's modem block.",
    )
    attach_group = parser.add_mutually_exclusive_group()
    attach_group.add_argument(
        "--attach",
        dest="attach",
        action="store_true",
        help="Attach to the catmux/tmux session after creating it.",
    )
    attach_group.add_argument(
        "--detach",
        dest="attach",
        action="store_false",
        help="Create the catmux/tmux session and return without attaching.",
    )
    parser.set_defaults(attach=None)
    args = parser.parse_args()

    # Use **vars(args) to convert argparse.Namespace to a dict, filtering out None values
    main(**{k: v for k, v in vars(args).items() if v is not None})
