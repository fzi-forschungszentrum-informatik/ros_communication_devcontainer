# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Deployment host configuration and peer binding resolution."""

from __future__ import annotations

import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

VALUE_PREFIX = "value:"


@dataclass(frozen=True)
class DeploymentHost:
    name: str
    address: str
    ssh: str | None


@dataclass(frozen=True)
class DeploymentConfig:
    path: Path
    hosts: dict[str, DeploymentHost]
    values: dict[str, Any]


@dataclass(frozen=True)
class PeerBinding:
    peer: str
    address: str
    ssh: str | None
    host: str | None = None


def _load_mapping(path: Path) -> dict[str, Any]:
    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise RuntimeError(f"Deployment file must contain a mapping: {path}")
    return loaded


def load_deployment(path: Path | None) -> DeploymentConfig | None:
    if path is None:
        return None
    raw = _load_mapping(path)
    unknown = sorted(set(raw) - {"hosts", "values"})
    if unknown:
        raise RuntimeError(f"Unsupported deployment keys {unknown}; allowed keys are ['hosts', 'values'].")

    hosts_raw = raw.get("hosts")
    values = raw.get("values")
    if hosts_raw is None:
        hosts_raw = {}
    if values is None:
        values = {}
    if not isinstance(hosts_raw, dict):
        raise RuntimeError("deployment.hosts must be a mapping.")
    if not isinstance(values, dict):
        raise RuntimeError("deployment.values must be a mapping.")

    hosts: dict[str, DeploymentHost] = {}
    for raw_name, raw_host in hosts_raw.items():
        name = str(raw_name).strip()
        if not name:
            raise RuntimeError("deployment host names must be non-empty.")
        if not isinstance(raw_host, dict):
            raise RuntimeError(f"deployment.hosts.{name} must be a mapping.")
        extra = sorted(set(raw_host) - {"address", "ssh"})
        if extra:
            raise RuntimeError(
                f"Unsupported deployment.hosts.{name} keys {extra}; allowed keys are ['address', 'ssh']."
            )
        address = raw_host.get("address")
        if not isinstance(address, str) or not address.strip():
            raise RuntimeError(f"deployment.hosts.{name}.address must be a non-empty string.")
        ssh = raw_host.get("ssh")
        if ssh is not None and (not isinstance(ssh, str) or not ssh.strip()):
            raise RuntimeError(f"deployment.hosts.{name}.ssh must be a non-empty string or null.")
        hosts[name] = DeploymentHost(name=name, address=address.strip(), ssh=ssh.strip() if ssh else None)

    return DeploymentConfig(path=path, hosts=hosts, values=copy.deepcopy(values))


def parse_assignments(raw_values: list[str] | None, *, option: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for raw in raw_values or []:
        key, separator, value = (raw or "").partition("=")
        key = key.strip()
        value = value.strip()
        if separator != "=" or not key or not value:
            raise RuntimeError(f"{option} must use '<peer>=<value>', for example '{option} a=machine-a'.")
        if key in result:
            raise RuntimeError(f"Duplicate {option} assignment for peer '{key}'.")
        result[key] = value
    return result


def resolve_value(expression: str, deployment: DeploymentConfig | None) -> str:
    value = str(expression).strip()
    if not value:
        raise RuntimeError("Deployment values must be non-empty strings.")
    if not value.startswith(VALUE_PREFIX):
        return value
    key = value[len(VALUE_PREFIX) :].strip()
    if not key:
        raise RuntimeError("value: references must include a key.")
    if deployment is None:
        raise RuntimeError(f"Cannot resolve '{value}' because no deployment file is configured.")

    current: Any = deployment.values
    for part in key.split("."):
        if not isinstance(current, dict) or part not in current:
            raise RuntimeError(f"Deployment value '{key}' was not found in {deployment.path}.")
        current = current[part]
    if isinstance(current, (dict, list)) or current is None:
        raise RuntimeError(f"Deployment value '{key}' must resolve to a scalar.")
    return str(current)


def resolve_peer_bindings(
    cfg: dict[str, Any],
    deployment: DeploymentConfig | None,
    *,
    peer_assignments: dict[str, str] | None = None,
    address_overrides: dict[str, str] | None = None,
    ssh_overrides: dict[str, str] | None = None,
    require_addresses: bool = True,
) -> dict[str, PeerBinding]:
    peers = cfg.get("peers")
    if not isinstance(peers, dict) or not peers:
        raise RuntimeError("Session config must define a non-empty peers mapping.")

    peer_assignments = peer_assignments or {}
    address_overrides = address_overrides or {}
    ssh_overrides = ssh_overrides or {}
    known = set(str(peer) for peer in peers)
    for option, assignments in (
        ("--peer", peer_assignments),
        ("--peer-address", address_overrides),
        ("--peer-ssh", ssh_overrides),
    ):
        unknown = sorted(set(assignments) - known)
        if unknown:
            raise RuntimeError(f"{option} references unknown peer(s) {unknown}. Known peers: {sorted(known)}")

    bindings: dict[str, PeerBinding] = {}
    missing: list[str] = []
    for raw_peer, raw_peer_cfg in peers.items():
        peer = str(raw_peer)
        peer_cfg = raw_peer_cfg or {}
        if not isinstance(peer_cfg, dict):
            raise RuntimeError(f"peers.{peer} must be a mapping.")
        host_name = peer_assignments.get(peer)
        if host_name is None:
            configured_host = peer_cfg.get("host")
            host_name = str(configured_host).strip() if configured_host is not None else None

        address: str | None = None
        ssh: str | None = None
        if host_name:
            if deployment is None:
                if peer in peer_assignments or peer not in address_overrides:
                    raise RuntimeError(
                        f"Peer '{peer}' selects deployment host '{host_name}', but no deployment file is configured."
                    )
                host_name = None
            else:
                try:
                    host = deployment.hosts[host_name]
                except KeyError as exc:
                    raise RuntimeError(
                        f"Unknown deployment host '{host_name}' for peer '{peer}'. "
                        f"Known hosts: {sorted(deployment.hosts)}"
                    ) from exc
                address = resolve_value(host.address, deployment)
                ssh = host.ssh

        if peer in address_overrides:
            address = resolve_value(address_overrides[peer], deployment)
        if peer in ssh_overrides:
            ssh_value = ssh_overrides[peer]
            ssh = None if ssh_value == "local" else ssh_value

        if address is None:
            missing.append(peer)
            continue
        bindings[peer] = PeerBinding(peer=peer, address=address, ssh=ssh, host=host_name)

    if missing and require_addresses:
        raise RuntimeError(
            "Missing deployment address for peer(s) "
            f"{missing}. Use --peer PEER=HOST with a deployment file, or --peer-address PEER=ADDRESS."
        )
    return bindings
