#!/usr/bin/env python3

import argparse
import json
import os
import sys

project_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(project_dir)

from ros2docker.stop import main as stop_container
from run_session_in_container import _load_session_config, _sanitize_container_name


def _get_peer_com_names(session_dir: str) -> dict:
    cfg = _load_session_config(session_dir)
    peers = (cfg or {}).get("peers")
    if not isinstance(peers, dict):
        raise RuntimeError("session-config must define a mapping 'peers: { <peer_key>: { ... } }'.")

    peer_keys = list(peers.keys())
    if len(peer_keys) != 2:
        raise RuntimeError(f"Expected exactly 2 peers, got peers={peer_keys}")

    names = {}
    for peer_key in peer_keys:
        v = None
        try:
            v = (peers[peer_key] or {}).get("com-name")
        except Exception:
            v = None
        if v is None:
            name = peer_key
        elif isinstance(v, str):
            s = v.strip()
            name = s if s else peer_key
        else:
            name = str(v) if v else peer_key
        names[peer_key] = name
    return names


def _container_names_for_session(session_dir: str, identity: str | None) -> list[str]:
    peer_names = _get_peer_com_names(session_dir)
    if identity:
        if identity not in peer_names:
            raise RuntimeError(f"--identity must be one of peers={list(peer_names.keys())}")
        remote_peer_key = next(k for k in peer_names.keys() if k != identity)
        remote_peer_name = peer_names[remote_peer_key]
        return [_sanitize_container_name(f"com_to_{remote_peer_name}")]

    # If identity is not provided, try both possible container names.
    return sorted({_sanitize_container_name(f"com_to_{v}") for v in peer_names.values()})


def _merge_override(override, container_name: str) -> dict:
    if override is None:
        merged = {}
    elif isinstance(override, str):
        try:
            merged = json.loads(override)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"Invalid JSON for --override: {override}") from exc
    elif isinstance(override, dict):
        merged = dict(override)
    else:
        merged = dict(override)

    merged["container_name"] = container_name
    return merged


def main(session_dir, identity=None, config_file=None, override=None):
    container_names = _container_names_for_session(session_dir, identity)
    for container_name in container_names:
        print(f"Stopping container: {container_name}")
        stop_container(config_file=config_file, override=_merge_override(override, container_name))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Stop rosotacom containers derived from a session directory.")
    parser.add_argument(
        "-s",
        "--session-dir",
        required=True,
        help="Directory containing a session config input file (session-definition.yaml / session-parametrization.yaml).",
    )
    parser.add_argument("--identity", required=False)
    parser.add_argument("-f", "--config_file")
    parser.add_argument("-o", "--override")
    args = parser.parse_args()
    main(**{k: v for k, v in vars(args).items() if v is not None})

