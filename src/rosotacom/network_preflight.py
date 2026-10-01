# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Host data-plane validation for resolved communication peers."""

from __future__ import annotations

import ipaddress
import re
import subprocess
from collections.abc import Iterable
from dataclasses import dataclass

from .deployment import PeerBinding


@dataclass(frozen=True)
class NetworkPreflightResult:
    identity: str
    local_address: str
    remote_identity: str
    remote_address: str
    interface: str
    source_address: str
    peer_reachable: bool | None


def _binding_ipv4(binding: PeerBinding, *, role: str) -> str:
    try:
        return str(ipaddress.IPv4Address(binding.address))
    except ipaddress.AddressValueError as exc:
        raise RuntimeError(
            f"Network preflight failed: {role} peer {binding.peer!r} has non-IPv4 address "
            f"{binding.address!r}. Communication peer addresses must be concrete IPv4 addresses."
        ) from exc


def run_network_preflight(
    bindings: dict[str, PeerBinding],
    identity: str,
    *,
    local_addresses: Iterable[str],
    require_peer_reachable: bool = False,
) -> NetworkPreflightResult:
    """Validate the selected local binding and kernel path to its peer."""
    if identity not in bindings:
        raise RuntimeError(f"Network preflight failed: identity {identity!r} is not one of peers={sorted(bindings)}.")
    if len(bindings) != 2:
        raise RuntimeError(f"Network preflight failed: expected exactly 2 peers, got peers={sorted(bindings)}.")

    local = bindings[identity]
    remote_identity = next(peer for peer in bindings if peer != identity)
    remote = bindings[remote_identity]
    local_address = _binding_ipv4(local, role="local")
    remote_address = _binding_ipv4(remote, role="remote")

    known_local_addresses = sorted(set(local_addresses))
    if not known_local_addresses:
        raise RuntimeError(
            "Network preflight failed: could not determine any local IPv4 addresses. "
            "Verify that the `ip` command is installed and the required data-plane interface is up."
        )
    if local_address not in known_local_addresses:
        raise RuntimeError(
            f"Network preflight failed for identity {identity!r}: configured local address {local_address} "
            f"is not assigned to this host (local IPv4s: {known_local_addresses}). "
            "Verify --identity and the peer bindings; if the address belongs to a VPN or other data-plane "
            "interface, bring that interface up before starting communication."
        )

    try:
        route = subprocess.run(
            ["ip", "-4", "route", "get", remote_address],
            text=True,
            capture_output=True,
            check=False,
        )
    except FileNotFoundError as exc:
        raise RuntimeError("Network preflight failed: required command `ip` was not found.") from exc
    route_output = (route.stdout or "").strip()
    if route.returncode != 0 or not route_output:
        detail = (route.stderr or route.stdout or "no route returned").strip()
        raise RuntimeError(
            f"Network preflight failed: no usable IPv4 route from {identity!r} ({local_address}) "
            f"to {remote_identity!r} ({remote_address}): {detail}"
        )
    route_line = route_output.splitlines()[0]
    source_match = re.search(r"\bsrc\s+(\d+\.\d+\.\d+\.\d+)\b", route_line)
    interface_match = re.search(r"\bdev\s+(\S+)", route_line)
    if source_match is None or interface_match is None:
        raise RuntimeError(
            f"Network preflight failed: could not determine source address and interface from route: {route_line}"
        )
    source_address = source_match.group(1)
    interface = interface_match.group(1)
    if source_address != local_address:
        raise RuntimeError(
            f"Network preflight failed: route to {remote_identity!r} ({remote_address}) uses source "
            f"{source_address} on {interface}, but identity {identity!r} is bound to {local_address}. "
            "Fix the route or peer binding before starting communication."
        )

    peer_reachable: bool | None = None
    if require_peer_reachable:
        try:
            ping = subprocess.run(
                ["ping", "-4", "-n", "-c", "3", "-W", "2", "-I", local_address, remote_address],
                text=True,
                capture_output=True,
                check=False,
                timeout=10,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                "Network preflight failed: strict peer reachability was requested, but `ping` was not found."
            ) from exc
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(
                f"Network preflight failed: ICMP check from {local_address} to {remote_address} exceeded 10 seconds."
            ) from exc
        if ping.returncode != 0:
            raise RuntimeError(
                f"Network preflight failed: peer {remote_identity!r} ({remote_address}) did not answer "
                f"3 ICMP echo requests sent from {local_address} via {interface}. This start requires an "
                "already-reachable peer; verify the data plane, VPN, and firewall."
            )
        peer_reachable = True

    return NetworkPreflightResult(
        identity=identity,
        local_address=local_address,
        remote_identity=remote_identity,
        remote_address=remote_address,
        interface=interface,
        source_address=source_address,
        peer_reachable=peer_reachable,
    )
