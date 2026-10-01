# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unit tests for the link-byte sampler (expectation concepts reference 'link overhead').

link_bytes is pure Python (no rclpy), loaded by file path like the other
in-container modules so the host suite can exercise it.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[2]
LINK_BYTES_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py" / "link_bytes.py"


def _load(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


lb = _load(LINK_BYTES_PY, "rosotacom_link_bytes")

# /proc/net/dev: two header rows (no ":", skipped), then lo + a tunnel interface.
# Columns: rx_bytes rx_pkts rx_errs rx_drop rx_fifo rx_frame rx_comp rx_mcast
#          tx_bytes tx_pkts tx_errs tx_drop tx_fifo tx_colls tx_carrier tx_comp
PROC_NET_DEV = """\
Inter-|   Receive                |  Transmit
 face |bytes    packets ... |bytes    packets ...
    lo: 123456 900 0 0 0 0 0 0 123456 900 0 0 0 0 0 0
  tun1: 1000000 5000 0 0 0 0 0 0 2000000 6000 0 0 0 0 0 0
"""


def test_parse_proc_net_dev_extracts_rx_tx() -> None:
    stats = lb.parse_proc_net_dev(PROC_NET_DEV)
    assert stats["tun1"] == {
        "rx_bytes": 1000000,
        "rx_packets": 5000,
        "tx_bytes": 2000000,
        "tx_packets": 6000,
    }
    assert stats["lo"]["rx_bytes"] == 123456


def test_parse_skips_headers_and_garbage() -> None:
    stats = lb.parse_proc_net_dev("garbage\n  bad: not numbers here\n" + PROC_NET_DEV)
    assert set(stats) == {"lo", "tun1"}


def test_kbps_conversion() -> None:
    # 1024 bytes over 1s == 8 kbit/s
    assert lb.kbps(1024, 1.0) == 8.0
    assert lb.kbps(100, 0.0) == 0.0


def _sampler_over(frames: list[str], times: list[float]):
    """A sampler whose /proc read and clock are driven by fixtures."""
    state = {"i": 0}

    def fake_read(iface, proc_path):  # noqa: ARG001
        stats = lb.parse_proc_net_dev(frames[state["i"]])
        return stats.get(iface)

    clock = {"i": 0}

    def fake_clock():
        t = times[clock["i"]]
        return t

    s = lb.LinkByteSampler("tun1", clock=fake_clock)
    # Patch the module-level reader the sampler uses.
    orig = lb.read_iface_counter_row
    lb.read_iface_counter_row = fake_read
    try:
        results = []
        for _ in frames:
            results.append(s.sample())
            state["i"] += 1
            clock["i"] += 1
    finally:
        lb.read_iface_counter_row = orig
    return results


def test_sampler_first_call_primes_then_reports_rate() -> None:
    frame0 = PROC_NET_DEV
    # +1024 rx, +2048 tx after 1 second.
    frame1 = PROC_NET_DEV.replace("1000000 5000", "1001024 5001").replace("2000000 6000", "2002048 6001")
    results = _sampler_over([frame0, frame1], [10.0, 11.0])
    assert results[0] is None  # primed
    assert results[1]["rx_kbps"] == lb.kbps(1024, 1.0)
    assert results[1]["tx_kbps"] == lb.kbps(2048, 1.0)
    assert results[1]["rx_packets_delta"] == 1
    assert results[1]["tx_packets_delta"] == 1
    assert results[1]["interface"] == "tun1"


def test_sampler_counter_reset_is_skipped() -> None:
    frame0 = PROC_NET_DEV
    frame1 = PROC_NET_DEV.replace("1000000", "10").replace("2000000", "20")  # counters dropped (reset)
    results = _sampler_over([frame0, frame1], [10.0, 11.0])
    assert results[1] is None


IP_ADDR_OUT = (
    "1: lo    inet 127.0.0.1/8 scope host lo\\       valid_lft forever preferred_lft forever\n"
    "5: tun1    inet 10.20.0.32/24 scope global tun1\\       valid_lft forever preferred_lft forever\n"
    "2: eth0    inet 172.17.0.2/16 scope global eth0\\       valid_lft forever preferred_lft forever\n"
)


def test_parse_ip_addr_resolves_interface_by_address() -> None:
    assert lb.parse_ip_addr_for_ip(IP_ADDR_OUT, "10.20.0.32") == "tun1"
    assert lb.parse_ip_addr_for_ip(IP_ADDR_OUT, "127.0.0.1") == "lo"
    assert lb.parse_ip_addr_for_ip(IP_ADDR_OUT, "172.17.0.2") == "eth0"


def test_parse_ip_addr_unknown_or_empty() -> None:
    assert lb.parse_ip_addr_for_ip(IP_ADDR_OUT, "10.0.0.99") is None
    assert lb.parse_ip_addr_for_ip(IP_ADDR_OUT, "") is None


# --- OTA link interface resolution (#267) ----------------------------------


def test_explicit_interface_wins_and_says_so() -> None:
    assert lb.resolve_link_interface("tun3", "10.20.0.39", finder=lambda ip: "tun1") == (
        "tun3",
        "given explicitly",
    )


def test_interface_is_resolved_from_the_ota_address() -> None:
    interface, provenance = lb.resolve_link_interface("", "10.20.0.39", finder=lambda ip: "tun1")
    assert interface == "tun1"
    assert provenance == "resolved from 10.20.0.39"


def test_an_address_no_interface_owns_is_an_error() -> None:
    try:
        lb.resolve_link_interface("", "10.20.0.39", finder=lambda ip: None)
    except ValueError as exc:
        assert "10.20.0.39" in str(exc)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("expected a ValueError")


def test_loopback_is_refused_for_a_real_ota_address() -> None:
    """The #267 defect, as the number that made it visible.

    The session template's interface lookup collapsed to `lo` on both peers.
    Loopback carries this host's own ROS traffic, so on the vehicle -- which
    also ran a 36 GB `-a` recording -- `link_bandwidth_kbps` read ~28,000,000
    (~28 Gbit/s) for an OTA link delivering ~6 Mbit/s, near-constant, on both
    directions at once (rx and tx are the same bytes on loopback).
    """
    try:
        lb.resolve_link_interface("", "10.20.0.38", finder=lambda ip: "lo")
    except ValueError as exc:
        assert "loopback" in str(exc)
    else:  # pragma: no cover - the call must raise
        raise AssertionError("expected a ValueError")


def test_loopback_is_fine_when_the_ota_address_is_loopback() -> None:
    """Single-machine smoke sessions really do run both peers over loopback."""
    assert lb.resolve_link_interface("", "127.0.0.1", finder=lambda ip: "lo")[0] == "lo"


def test_is_loopback_address() -> None:
    assert lb.is_loopback_address("127.0.0.1")
    assert lb.is_loopback_address("127.0.1.1")
    assert lb.is_loopback_address("::1")
    assert not lb.is_loopback_address("10.20.0.39")
    assert not lb.is_loopback_address("")
