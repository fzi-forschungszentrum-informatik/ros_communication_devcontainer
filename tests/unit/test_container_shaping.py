# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Container-mode network shaping: tc/ip without host sudo (issue #279).

The privileged half of network profiles reference gains a third privilege shape: every tc/ip
argv runs in a short-lived `docker run --rm --network host --cap-add
NET_ADMIN` container on the peer, so docker-group membership replaces the
sudoers entry. These tests pin the generated shell -- the part that must be
right without a bench pair at hand: the netns choice, the capability, the
watchdog's detachment and self-replacement, and the on-peer image resolution
with its actionable failure.
"""

from __future__ import annotations

import shlex

import pytest

from rosotacom.cli import (
    _SHAPING_WATCHDOG_NAME,
    _container_shaping_command,
    _validate_ota_sudo_mode,
)
from rosotacom.network_profiles import safety_teardown_command, teardown_command


def test_container_is_a_valid_sudo_mode() -> None:
    assert _validate_ota_sudo_mode("container") == "container"
    with pytest.raises(RuntimeError, match="Unsupported OTA sudo mode"):
        _validate_ota_sudo_mode("nsenter")


def test_tc_argv_runs_in_a_host_netns_net_admin_container() -> None:
    script = _container_shaping_command(teardown_command("tun0"), image="ros-communication-abc:latest")
    assert script.startswith("img=ros-communication-abc:latest; ")
    assert "docker run --rm --network host --cap-add NET_ADMIN" in script
    assert '--entrypoint tc "$img" qdisc del dev tun0 root' in script
    assert "sudo" not in script


def test_without_an_explicit_image_the_peer_resolves_its_own() -> None:
    script = _container_shaping_command(["tc", "qdisc", "show"], image=None)
    # Prefer the image of a running rosotacom container, then the newest local
    # com image; a machine that has neither gets an actionable error, not a
    # docker pull attempt.
    assert "docker ps --filter name=rosotacom_" in script
    assert "^ros-communication" in script
    assert "exit 69" in script
    assert "--shaping-image" in script


def test_the_watchdog_detaches_via_the_daemon_and_replaces_a_stale_one() -> None:
    argv = safety_teardown_command("tun0", 120.0)
    assert argv[:2] == ["sh", "-c"]  # the launcher contract this test relies on

    script = _container_shaping_command(argv, image="img:1", watchdog=True)
    assert f"docker rm -f {_SHAPING_WATCHDOG_NAME}" in script
    assert f"docker run -d --rm --name {_SHAPING_WATCHDOG_NAME} --network host --cap-add NET_ADMIN" in script
    assert f'--entrypoint sh "$img" -c {shlex.quote(argv[2])}' in script
    # `docker run -d` IS the detachment: no nohup, no sudo.
    assert "nohup" not in script
    assert "sudo" not in script


def test_the_watchdog_name_stays_out_of_the_conflict_scan() -> None:
    """The OTA conflict check flags containers named `rosotacom_*`; a sleeping
    watchdog from a finished run must not read as an active run."""
    assert not _SHAPING_WATCHDOG_NAME.startswith("rosotacom_")
