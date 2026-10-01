# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Lightweight link-byte measurement from the kernel's own interface counters.

The OTA link runs over a (typically dedicated) network interface -- a VPN tunnel
such as ``tun1`` in the cross-host setup. The kernel already counts every byte
that crosses it in ``/proc/net/dev`` (``rx_bytes`` / ``tx_bytes``), so measuring
link traffic needs no packet capture, no ``tshark``, and no privileges: we read
two counters, diff them over a window, and divide by elapsed time.

This replaces the ``sudo tshark`` path in ``topic_monitor`` (see expectation concepts reference
"link overhead"): the status overview already measures the *ROS payload*
bandwidth per stage, and this module supplies the *wire* bandwidth, so the two
together give the overhead ratio ``link / payload`` (=~1 healthy; >>1 means
retransmits, shadow connections, or a bad RMW/QoS).

Pure parsing is split out (``parse_proc_net_dev``) so it is unit-testable against
a fixture without touching the real filesystem.
"""

from __future__ import annotations

import subprocess
import time
from typing import Any, Dict, Optional, Tuple

PROC_NET_DEV = "/proc/net/dev"

# Field order on each data row of /proc/net/dev, after "<iface>:".
# Receive: bytes packets errs drop fifo frame compressed multicast
# Transmit: bytes packets errs drop fifo colls carrier compressed
_RX_BYTES_IDX = 0
_RX_PACKETS_IDX = 1
_TX_BYTES_IDX = 8
_TX_PACKETS_IDX = 9


def parse_proc_net_dev(text: str) -> Dict[str, Dict[str, int]]:
    """Parse /proc/net/dev contents -> {iface: {'rx_bytes', 'tx_bytes'}}."""
    out: Dict[str, Dict[str, int]] = {}
    for line in text.splitlines():
        if ":" not in line:
            continue  # the two header rows have no "<iface>:"
        name, _, rest = line.partition(":")
        iface = name.strip()
        fields = rest.split()
        if len(fields) <= _TX_BYTES_IDX:
            continue
        try:
            out[iface] = {
                "rx_bytes": int(fields[_RX_BYTES_IDX]),
                "rx_packets": int(fields[_RX_PACKETS_IDX]),
                "tx_bytes": int(fields[_TX_BYTES_IDX]),
                "tx_packets": int(fields[_TX_PACKETS_IDX]),
            }
        except ValueError:
            continue
    return out


def parse_ip_addr_for_ip(text: str, ip: str) -> Optional[str]:
    """Find the interface owning ``ip`` in `ip -o addr show` output.

    Each line looks like: ``5: tun1    inet 10.20.0.32/24 scope global tun1 ...``
    We resolve the interface by address rather than a brittle shell regex so the
    OTA link interface (e.g. the VPN tunnel) is found reliably from the peer's
    own ip_local, with no dependence on catmux/shell variable expansion.
    """
    if not ip:
        return None
    for line in text.splitlines():
        parts = line.split()
        if "inet" not in parts and "inet6" not in parts:
            continue
        for key in ("inet", "inet6"):
            if key in parts:
                idx = parts.index(key)
                if idx + 1 < len(parts) and parts[idx + 1].split("/")[0] == ip:
                    return parts[1]
    return None


#: Interface names that carry intra-host traffic only. Sampling one of these and
#: calling the result "link bandwidth" is the failure of issue #267: the number
#: is real, it is just not the link. On a machine that also runs a `-a` recorder
#: it reads ~28 Gbit/s next to a 6 Mbit/s OTA link.
LOOPBACK_INTERFACES = frozenset({"lo", "lo0"})


def is_loopback_address(ip: str) -> bool:
    """Is ``ip`` a loopback address (so a loopback interface would be correct)?"""
    ip = (ip or "").strip()
    return ip.startswith("127.") or ip in {"::1", "0:0:0:0:0:0:0:1"}


def find_interface_for_ip(ip: str) -> Optional[str]:
    """The interface owning ``ip``, via `ip -o addr show` (None if not found)."""
    if not ip:
        return None
    try:
        out = subprocess.run(
            ["ip", "-o", "addr", "show"], capture_output=True, text=True, timeout=5, check=True
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    return parse_ip_addr_for_ip(out, ip)


def resolve_link_interface(
    explicit: str,
    host_ip: str,
    finder=find_interface_for_ip,
) -> Tuple[str, str]:
    """The OTA link interface and where it came from, or raise.

    ``explicit`` wins when given; otherwise the interface is the one owning
    ``host_ip`` (the peer's own OTA address). A loopback interface for a
    non-loopback address is refused rather than reported: loopback carries this
    host's own ROS traffic, which on a machine running an ``-a`` recorder is
    three orders of magnitude above the link and looks like a working
    measurement. That mistake is issue #267; it survived a field session and an
    offline analysis before anyone doubted the number.
    """
    interface = (explicit or "").strip()
    provenance = "given explicitly"
    if not interface:
        interface = (finder(host_ip) or "").strip()
        provenance = f"resolved from {host_ip}"
    if not interface:
        raise ValueError(
            f"no interface owns the OTA address '{host_ip}'; "
            "pass ip_local (preferred) or interface"
        )
    if interface in LOOPBACK_INTERFACES and not is_loopback_address(host_ip):
        raise ValueError(
            f"refusing to report loopback interface '{interface}' as the link for OTA "
            f"address '{host_ip}' ({provenance}). Loopback carries this host's own ROS "
            "traffic, not the link."
        )
    return interface, provenance


def read_iface_counters(iface: str, proc_path: str = PROC_NET_DEV) -> Optional[Tuple[int, int]]:
    """Return (rx_bytes, tx_bytes) for ``iface`` now, or None if unavailable."""
    row = read_iface_counter_row(iface, proc_path)
    if row is None:
        return None
    return row["rx_bytes"], row["tx_bytes"]


def read_iface_counter_row(iface: str, proc_path: str = PROC_NET_DEV) -> Optional[Dict[str, int]]:
    """Return parsed /proc/net/dev counters for ``iface``, or None if unavailable."""
    try:
        with open(proc_path, encoding="utf-8") as fp:
            stats = parse_proc_net_dev(fp.read())
    except OSError:
        return None
    return stats.get(iface)


def kbps(delta_bytes: int, elapsed_s: float) -> float:
    """Bytes over a window -> kilobits per second (matches topic_monitor units)."""
    if elapsed_s <= 0:
        return 0.0
    return (delta_bytes * 8.0 / 1024.0) / elapsed_s


class LinkByteSampler:
    """Diffs interface byte counters between calls to report rx/tx Kbit/s.

    The first ``sample`` only primes the baseline (returns None); each later call
    returns the rate since the previous call. Counter wraps (negative deltas) are
    treated as no data for that window rather than reported as a spike.
    """

    def __init__(self, iface: str, proc_path: str = PROC_NET_DEV, clock=time.monotonic) -> None:
        self.iface = iface
        self._proc_path = proc_path
        self._clock = clock
        self._last_t: Optional[float] = None
        self._last_rx: Optional[int] = None
        self._last_tx: Optional[int] = None
        self._last_rx_packets: Optional[int] = None
        self._last_tx_packets: Optional[int] = None

    def sample(self) -> Optional[Dict[str, Any]]:
        counters = read_iface_counter_row(self.iface, self._proc_path)
        if counters is None:
            return None
        rx = counters["rx_bytes"]
        tx = counters["tx_bytes"]
        rx_packets = counters.get("rx_packets")
        tx_packets = counters.get("tx_packets")
        now = self._clock()
        prev_t = self._last_t
        prev_rx = self._last_rx
        prev_tx = self._last_tx
        prev_rx_packets = self._last_rx_packets
        prev_tx_packets = self._last_tx_packets
        self._last_t = now
        self._last_rx = rx
        self._last_tx = tx
        self._last_rx_packets = rx_packets
        self._last_tx_packets = tx_packets
        if prev_t is None or prev_rx is None or prev_tx is None:
            return None
        elapsed = now - prev_t
        d_rx, d_tx = rx - prev_rx, tx - prev_tx
        if d_rx < 0 or d_tx < 0 or elapsed <= 0:
            return None  # counter reset/wrap -- skip this window
        d_rx_packets = rx_packets - prev_rx_packets if rx_packets is not None and prev_rx_packets is not None else None
        d_tx_packets = tx_packets - prev_tx_packets if tx_packets is not None and prev_tx_packets is not None else None
        if d_rx_packets is not None and d_rx_packets < 0:
            d_rx_packets = None
        if d_tx_packets is not None and d_tx_packets < 0:
            d_tx_packets = None
        return {
            "interface": self.iface,
            "rx_kbps": kbps(d_rx, elapsed),
            "tx_kbps": kbps(d_tx, elapsed),
            "rx_bytes_delta": d_rx,
            "tx_bytes_delta": d_tx,
            "rx_packets_delta": d_rx_packets,
            "tx_packets_delta": d_tx_packets,
            "window_s": elapsed,
        }
