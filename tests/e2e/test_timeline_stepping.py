# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""network profiles reference validation — live seamless timeline stepping (issue #124).

The host unit tests assert the generated argv; this proves the kernel semantics
that argv relies on, live in a container: ``tc qdisc replace`` on an unchanged
tree changes it **in place** — the qdisc stats (and with them the queue) persist
across a step boundary and the netem child survives a tbf-root replace — so
stepping drops no queued packets and never leaves an unshaped window. It also
proves that a bare ``netem`` step resets previous delay/loss instead of
inheriting it, and that a reconnect outage's link-down is restored by the
following step.
"""

from __future__ import annotations

import os
import re
import subprocess
import uuid
from collections.abc import Iterator, Sequence

import pytest

from rosotacom.network_profiles import expand_timeline, parse_profile
from rosotacom.network_shaper import ProfileShaper

pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(
        os.environ.get("ROSOTACOM_RUN_E2E") != "1",
        reason="Docker-backed E2E tests require ROSOTACOM_RUN_E2E=1.",
    ),
]

# Any small image with a shell works; iproute2 is installed into it. The shaping
# itself runs against the container's eth0 with CAP_NET_ADMIN, exactly like the
# lab benchmark path.
IMAGE = "debian:stable-slim"


def _exec(container: str, argv: Sequence[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["docker", "exec", "-u", "root", container, *argv],
        capture_output=True,
        text=True,
        check=False,
    )


def _runner(container: str):
    def run(argv: Sequence[str]) -> None:
        result = _exec(container, argv)
        if result.returncode != 0:
            raise RuntimeError(f"{' '.join(argv)} -> {result.stderr or result.stdout}")

    return run


def _qdisc_show(container: str) -> str:
    result = _exec(container, ["tc", "-s", "qdisc", "show", "dev", "eth0"])
    assert result.returncode == 0, result.stderr
    return result.stdout


def _netem_sent_packets(container: str) -> int:
    match = re.search(r"qdisc netem 10:.*?Sent \d+ bytes (\d+) pkt", _qdisc_show(container), re.DOTALL)
    assert match, "netem 10: stage not found in qdisc show"
    return int(match.group(1))


def _operstate(container: str) -> str:
    return _exec(container, ["cat", "/sys/class/net/eth0/operstate"]).stdout.strip()


def _send_udp_burst(container: str) -> None:
    # Fire-and-forget datagrams towards TEST-NET-3 egress eth0 via the default
    # route; no reply is needed, only egress traffic through the qdisc tree.
    result = _exec(
        container,
        ["bash", "-c", "for i in $(seq 40); do echo probe > /dev/udp/203.0.113.1/9; done"],
    )
    assert result.returncode == 0, result.stderr


@pytest.fixture()
def shaping_container() -> Iterator[str]:
    name = f"rosotacom-stepping-{uuid.uuid4().hex[:8]}"
    subprocess.run(
        ["docker", "run", "-d", "--rm", "--name", name, "--cap-add", "NET_ADMIN", IMAGE, "sleep", "600"],
        check=True,
        capture_output=True,
    )
    try:
        install = _exec(name, ["sh", "-c", "apt-get update -qq && apt-get install -y -qq iproute2 >/dev/null"])
        if install.returncode != 0:
            pytest.fail(f"could not install iproute2 in {IMAGE}: {install.stderr}")
        netem_probe = _exec(name, ["tc", "qdisc", "replace", "dev", "eth0", "root", "handle", "10:", "netem"])
        if netem_probe.returncode != 0:
            pytest.skip(f"host kernel lacks sch_netem: {netem_probe.stderr.strip()}")
        _exec(name, ["tc", "qdisc", "del", "dev", "eth0", "root"])
        yield name
    finally:
        subprocess.run(["docker", "rm", "-f", name], capture_output=True, check=False)


def test_timeline_steps_change_the_qdisc_tree_in_place(shaping_container: str) -> None:
    profile = parse_profile(
        "stepping",
        {
            "timeline": [
                {"for": "1s", "uplink": {"rate": "8mbit", "delay": "50ms", "jitter": "15ms", "distribution": "normal"}},
                {"for": "1s", "uplink": {"rate": "3mbit", "delay": "80ms"}},
                {"for": "1s", "outage": "catchup"},
                {"for": "1s", "uplink": {"delay": "20ms"}},
                {"for": "1s", "outage": "reconnect"},
                {"for": "1s", "uplink": {"rate": "8mbit"}},
            ]
        },
    )
    steps = expand_timeline(profile, "eth0", direction="uplink")

    with ProfileShaper("eth0", _runner(shaping_container)) as shaper:
        shaper.arm([])

        shaper.apply(steps[0].commands)
        show = _qdisc_show(shaping_container)
        assert "8Mbit" in show and "delay 50ms" in show
        _send_udp_burst(shaping_container)
        sent_before = _netem_sent_packets(shaping_container)
        assert sent_before > 0

        # The step boundary must change the tree in place: a recreated qdisc
        # would reset its stats (and drop its queue) — they must persist.
        shaper.apply(steps[1].commands)
        show = _qdisc_show(shaping_container)
        assert "3Mbit" in show and "delay 80ms" in show
        assert _netem_sent_packets(shaping_container) >= sent_before

        # Catchup outage: in-place full loss, interface stays up.
        shaper.apply(steps[2].commands)
        assert "loss 100%" in _qdisc_show(shaping_container)
        assert _operstate(shaping_container) == "up"

        # Rate-less step: tbf stage stays as effectively-unlimited, loss is gone.
        shaper.apply(steps[3].commands)
        show = _qdisc_show(shaping_container)
        assert "10Gbit" in show and "delay 20ms" in show and "loss" not in show

        # Reconnect outage downs the link; the following step restores it first
        # and a netem-less segment leaves a bare pass-through netem stage.
        shaper.apply(steps[4].commands)
        assert _operstate(shaping_container) == "down"
        shaper.apply(steps[5].commands)
        assert _operstate(shaping_container) == "up"
        show = _qdisc_show(shaping_container)
        assert "8Mbit" in show and "delay" not in show

    # Context exit reverts: no shaping stages remain on the interface.
    show = _qdisc_show(shaping_container)
    assert "tbf" not in show and "netem" not in show
