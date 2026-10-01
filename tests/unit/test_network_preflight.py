# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import argparse
import subprocess

import pytest

import rosotacom.cli as rosotacom
import rosotacom.network_preflight as network_preflight
from rosotacom.deployment import PeerBinding


def _bindings(
    local: str = "10.20.0.39",
    remote: str = "10.20.0.38",
) -> dict[str, PeerBinding]:
    return {
        "center": PeerBinding("center", local, None, "center-host"),
        "vehicle": PeerBinding("vehicle", remote, None, "vehicle-host"),
    }


def test_network_preflight_verifies_local_address_and_route_source(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[list[str], dict[str, object]]] = []
    monkeypatch.setattr(rosotacom, "_get_local_ipv4s", lambda: ["127.0.0.1", "10.20.0.39"])

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append((command, kwargs))
        return subprocess.CompletedProcess(
            command,
            0,
            "10.20.0.38 dev tun1 src 10.20.0.39 uid 1000\n",
            "",
        )

    monkeypatch.setattr(network_preflight.subprocess, "run", fake_run)

    result = rosotacom._network_preflight(_bindings(), "center")

    assert result == rosotacom.NetworkPreflightResult(
        identity="center",
        local_address="10.20.0.39",
        remote_identity="vehicle",
        remote_address="10.20.0.38",
        interface="tun1",
        source_address="10.20.0.39",
        peer_reachable=None,
    )
    assert calls == [
        (
            ["ip", "-4", "route", "get", "10.20.0.38"],
            {"text": True, "capture_output": True, "check": False},
        )
    ]


def test_network_preflight_rejects_missing_local_binding_before_route_lookup(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(rosotacom, "_get_local_ipv4s", lambda: ["10.0.11.18"])

    def fail_run(*args: object, **kwargs: object) -> subprocess.CompletedProcess[str]:
        raise AssertionError("route lookup must not run when the local address is absent")

    monkeypatch.setattr(network_preflight.subprocess, "run", fail_run)

    with pytest.raises(RuntimeError, match="configured local address 10.20.0.39 is not assigned") as excinfo:
        rosotacom._network_preflight(_bindings(), "center")

    assert "VPN or other data-plane interface" in str(excinfo.value)


def test_network_preflight_rejects_route_with_wrong_source(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(rosotacom, "_get_local_ipv4s", lambda: ["10.20.0.39", "192.168.10.12"])
    monkeypatch.setattr(
        network_preflight.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command,
            0,
            "10.20.0.38 via 192.168.10.1 dev eth0 src 192.168.10.12 uid 1000\n",
            "",
        ),
    )

    with pytest.raises(RuntimeError, match="uses source 192.168.10.12 on eth0") as excinfo:
        rosotacom._network_preflight(_bindings(), "center")

    assert "bound to 10.20.0.39" in str(excinfo.value)


def test_network_preflight_can_require_bounded_peer_reachability(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    calls: list[tuple[list[str], dict[str, object]]] = []
    monkeypatch.setattr(rosotacom, "_get_local_ipv4s", lambda: ["10.20.0.39"])

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append((command, kwargs))
        if command[0] == "ip":
            return subprocess.CompletedProcess(
                command,
                0,
                "10.20.0.38 dev tun1 src 10.20.0.39 uid 1000\n",
                "",
            )
        return subprocess.CompletedProcess(command, 0, "3 packets transmitted, 3 received\n", "")

    monkeypatch.setattr(network_preflight.subprocess, "run", fake_run)

    result = rosotacom._network_preflight(_bindings(), "center", require_peer_reachable=True)

    assert result.peer_reachable is True
    assert calls[1] == (
        ["ping", "-4", "-n", "-c", "3", "-W", "2", "-I", "10.20.0.39", "10.20.0.38"],
        {"text": True, "capture_output": True, "check": False, "timeout": 10},
    )


def test_network_preflight_reports_required_peer_that_does_not_answer(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(rosotacom, "_get_local_ipv4s", lambda: ["10.20.0.39"])

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        if command[0] == "ip":
            return subprocess.CompletedProcess(
                command,
                0,
                "10.20.0.38 dev tun1 src 10.20.0.39 uid 1000\n",
                "",
            )
        return subprocess.CompletedProcess(command, 1, "3 packets transmitted, 0 received\n", "")

    monkeypatch.setattr(network_preflight.subprocess, "run", fake_run)

    with pytest.raises(RuntimeError, match="did not answer 3 ICMP echo requests") as excinfo:
        rosotacom._network_preflight(_bindings(), "center", require_peer_reachable=True)

    assert "verify the data plane, VPN, and firewall" in str(excinfo.value)


def test_preflight_command_exposes_session_binding_and_reachability_args(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    seen: list[argparse.Namespace] = []
    monkeypatch.setattr(rosotacom, "preflight_session", lambda args: seen.append(args))

    result = rosotacom.main(
        [
            "preflight",
            "1_heartbeat",
            "--identity",
            "center",
            "--peer",
            "center=control-center",
            "--peer-address",
            "vehicle=10.20.0.38",
            "--require-peer-reachable",
        ]
    )

    assert result == 0
    assert len(seen) == 1
    assert seen[0].session_dir == "1_heartbeat"
    assert seen[0].identity == "center"
    assert seen[0].peer == ["center=control-center"]
    assert seen[0].peer_address == ["vehicle=10.20.0.38"]
    assert seen[0].require_peer_reachable is True
