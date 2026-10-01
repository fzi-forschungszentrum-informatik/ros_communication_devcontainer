# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Header-only packet capture of the OTA link, beside the link trace.

Runs inside the communication container for the lifetime of one peer, started
by `status_overview` when a session definition asks for it:

    shared:
      use_status_overview: true
      ota_pcap:
        enabled: true

    python3 -m com_py.ota_pcap --iface tun0 --out .../ota.pcap [--peer 10.20.0.37]

WHAT IT ANSWERS THAT `link_trace.jsonl` CANNOT
----------------------------------------------
The link trace counts `/proc/net/dev` bytes per direction once a second, which
localises loss to a *direction* and no further. It cannot say which fragment of
which sample was lost, whether a retransmit followed, or how one sample's
sub-messages were spaced on the wire. Those are the questions a header-only
capture answers, at roughly 1 % of payload volume.

It is passive and local: nothing is transmitted, so it costs no link capacity,
and the interface is never put into promiscuous mode — everything of interest
is addressed to or from this host, and promiscuous mode on a control-centre LAN
port would collect other people's traffic. `CAP_NET_ADMIN`, which the
originating issue assumed was needed, buys only that.

WHY IT LIVES HERE AND NOT NEXT TO A RECORDER
--------------------------------------------
The first implementation of this sat beside the recording tool in the
downstream project, and had to rebuild what this container already knows:
which interface carries the link, and to which peer. `link_bytes` answers both
from the peer's own OTA address — including the refusal that matters most, a
loopback interface for a non-loopback address, which once read 28 Gbit/s next
to a 6 Mbit/s link and survived a field session (#267). A capture that has to
re-derive link knowledge is in the wrong place; this one asks the same
resolution the status snapshot's own link block uses.

WHAT IT NEEDS, MEASURED RATHER THAN ASSUMED
-------------------------------------------
`sudo`, and only until the socket is open. ros2docker's entrypoint runs this
container's processes as `containeruser`, whose `CapEff` is
0000000000000000 — a non-root process cannot open an `AF_PACKET` socket
however the container was started, so `--cap-add` buys nothing. `containeruser`
does have passwordless sudo here, which the NET pane's `sudo iftop` already
relies on, so the capture is launched through it and `--drop-to` hands the
process back to `containeruser` the moment the socket exists. The pcap in
`logs/<peer>/status/` therefore belongs to the same user as `link_trace.jsonl`
beside it.

That is the whole cost: no image change, no rebuild, no added capability and no
extra container. Two earlier attempts at this assumed root from a `docker run`
that bypassed the entrypoint, and both were wrong in the same way — check the
*running* container, not the image.

THE FILE
--------
`SOCK_DGRAM` rather than `SOCK_RAW`, which is what makes one code path work on
both link types the fleet uses: a VPN `tun` device is layer 3 and has no
link-layer header at all, while a LAN port is Ethernet. The kernel hands a
`SOCK_DGRAM` packet socket the network-layer packet either way, plus the
`sockaddr_ll` that says what the link layer was.

That is exactly the input `LINKTYPE_LINUX_SLL` (113) is defined over, so the
16-byte SLL pseudo-header is synthesized from the address tuple and the file
reads in Wireshark, tshark and scapy without a hint. It carries the field the
analysis is actually for: `pkttype` distinguishes PACKET_OUTGOING from
PACKET_HOST, so **every packet in the file is labelled uplink or downlink** —
which is the direction question the per-second counters answer only coarsely,
and the one a direction-blind `LINKTYPE_RAW` capture would have thrown away.

Timestamps come from the kernel (`SO_TIMESTAMPNS`), not from the moment python
got round to the packet, and are written at microsecond resolution — the
classic `0xa1b2c3d4` magic, because every reader takes it and a millisecond
phenomenon does not need nanoseconds.

HONESTY ABOUT WHAT IT MISSED
----------------------------
A capture that quietly drops packets is worse than none: the analysis would
read the gap as radio loss. So `PACKET_STATISTICS` is read from the kernel and
the sidecar `.json` reports `kernel_drops` next to `packets`. A non-zero drop
count means this process fell behind, not that the link did.

Neither file depends on a clean shutdown, and that is deliberate. A session
usually ends by the container being stopped, which does not reach this process
as a signal it can act on — the first bench run ended exactly that way and left
a pcap truncated at a buffer boundary and no sidecar at all. So the pcap is
flushed on an interval and the sidecar is written at the *start* and rewritten
on the same interval, with `stopped_because: "running"` until something stops
it. A killed capture therefore still leaves a readable pcap and a sidecar that
says what it was and how much it had seen, rather than an unexplained file.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import struct
import sys
import time
from pathlib import Path

#: Capture every ethertype; the protocol filter is applied below, on the
#: network-layer header the kernel hands a SOCK_DGRAM packet socket.
ETH_P_ALL = 0x0003

ETH_P_IP = 0x0800
ETH_P_IPV6 = 0x86DD
IPPROTO_UDP = 17

#: linux/if_packet.h. Only the outgoing one is load-bearing — it is what makes
#: a packet uplink rather than downlink — but all five are written through so a
#: reader sees what the kernel saw.
PACKET_OUTGOING = 4

SOL_PACKET = 263
PACKET_STATISTICS = 6

#: CPython does not export these on every build — 3.12 in the recorder image
#: has neither, which cost the first capture its kernel timestamps and left it
#: stamping packets when python got round to them. The numbers are the
#: asm-generic ones every architecture the fleet runs uses, and `SCM_` shares
#: the `SO_` value by definition (linux/socket.h).
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)
SCM_TIMESTAMPNS = getattr(socket, "SCM_TIMESTAMPNS", 35)

#: pcap, classic microsecond magic (LE). Every reader takes it.
PCAP_MAGIC = 0xA1B2C3D4
LINKTYPE_LINUX_SLL = 113

#: `-s 96` in the issue: enough for the link, network, UDP and RTPS sub-message
#: headers, and short of any payload.
DEFAULT_SNAPLEN = 96

#: Not part of the recorder's own disk budget, and deliberately far above what
#: a drive produces: header-only at 500 packets/s is ~6 MB per minute, so a
#: two-hour drive is well under a gigabyte. It exists to bound a mistake — a
#: capture pointed at a busy control-centre LAN port — not to shape a drive.
DEFAULT_MAX_MB = 2000

#: 4 MiB of kernel socket buffer. The default is a few hundred kilobytes, which
#: is where `kernel_drops` comes from on a burst.
RCVBUF_BYTES = 4 << 20

#: How often the pcap is flushed and the sidecar rewritten. A session that ends
#: by having its container stopped never reaches this process, so what is on
#: disk at any moment is what the run will be read from. One second costs a
#: ~300-byte write and bounds the loss to the packets of that second.
CHECKPOINT_S = 1.0


def log(message: str) -> None:
    print(f"[ota_pcap] {message}", flush=True)


# ------------------------------------------------------------------ the file


def pcap_header(snaplen: int) -> bytes:
    return struct.pack(
        "<IHHiIII",
        PCAP_MAGIC,
        2,  # version_major
        4,  # version_minor
        0,  # thiszone: timestamps are UTC
        0,  # sigfigs, as every writer leaves it
        snaplen,
        LINKTYPE_LINUX_SLL,
    )


def sll_header(pkttype: int, hatype: int, addr: bytes, protocol: int) -> bytes:
    """The 16-byte pseudo-header LINKTYPE_LINUX_SLL is defined over.

    Big-endian, unlike the pcap headers around it — that is the format, not an
    oversight. `addr` is padded to the fixed 8 bytes and its true length is
    written next to it, which is how a reader tells a 6-byte MAC from the empty
    address a layer-3 tunnel has.
    """
    address = (addr or b"")[:8]
    return struct.pack(
        "!HHH8sH",
        pkttype,
        hatype,
        len(address),
        address.ljust(8, b"\0"),
        protocol,
    )


def record_header(seconds: int, micros: int, incl_len: int, orig_len: int) -> bytes:
    return struct.pack("<IIII", seconds, micros, incl_len, orig_len)


# --------------------------------------------------------------- the filter


def is_udp(protocol: int, packet: bytes) -> bool:
    """UDP, judged from the network-layer header the packet socket handed us.

    Both branches read a fixed offset of the header the kernel guarantees is
    there, so a truncated or malformed packet is answered `False` rather than
    raising inside the capture loop.

    IPv4 fragments after the first carry no UDP header, and are still matched:
    the protocol field is in the IP header of every fragment, which is what
    keeps a fragmented sample whole in the file.
    """
    if protocol == ETH_P_IP:
        return len(packet) > 9 and packet[9] == IPPROTO_UDP
    if protocol == ETH_P_IPV6:
        # Next Header. An extension-header chain would hide the UDP behind it;
        # nothing on this link uses one, and guessing past it would be the kind
        # of cleverness that silently drops traffic.
        return len(packet) > 6 and packet[6] == IPPROTO_UDP
    return False


def addresses_of(protocol: int, packet: bytes) -> tuple[str, str] | None:
    """Source and destination, for the optional peer filter."""
    try:
        if protocol == ETH_P_IP and len(packet) >= 20:
            return (
                socket.inet_ntop(socket.AF_INET, packet[12:16]),
                socket.inet_ntop(socket.AF_INET, packet[16:20]),
            )
        if protocol == ETH_P_IPV6 and len(packet) >= 40:
            return (
                socket.inet_ntop(socket.AF_INET6, packet[8:24]),
                socket.inet_ntop(socket.AF_INET6, packet[24:40]),
            )
    except (ValueError, OSError):
        return None
    return None


def wanted(protocol: int, packet: bytes, peer: str | None) -> bool:
    if not is_udp(protocol, packet):
        return False
    if peer is None:
        return True
    pair = addresses_of(protocol, packet)
    return bool(pair) and peer in pair


# ---------------------------------------------------------------- capturing


def open_socket(iface: str) -> socket.socket:
    """A non-promiscuous packet socket, bound to one interface.

    `SOCK_DGRAM` normalises the two link types the fleet has — see the module
    header. No `PACKET_ADD_MEMBERSHIP`, so the interface is never put into
    promiscuous mode and only this host's own traffic is seen.
    """
    sock = socket.socket(socket.AF_PACKET, socket.SOCK_DGRAM, socket.htons(ETH_P_ALL))
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, RCVBUF_BYTES)
    except OSError:  # a smaller buffer is a worse capture, not a broken one
        pass
    try:
        sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
    except OSError:
        log("WARN the kernel refused SO_TIMESTAMPNS — timestamps are taken in "
            "userspace, so they carry this process's scheduling delay")
    sock.bind((iface, ETH_P_ALL))
    return sock


def drop_privileges(spec: str | None) -> str | None:
    """Hand the process back to the user that started it.

    The capture is launched through `sudo` — see the module header — because a
    non-root process cannot open an `AF_PACKET` socket. Nothing after that call
    needs root, and the artifacts in `logs/<peer>/status/` belong to
    `containeruser` like `link_trace.jsonl` beside them: a root-owned file in
    that directory is a `sudo rm` waiting to happen months later, and
    `gather_drive.py` would copy it without noticing.

    `spec` is "uid:gid". An argument rather than an environment variable
    because sudo does not pass the environment through. Returns what it did, or
    None when there was nothing to do.
    """
    if not spec:
        return None
    try:
        uid_text, _, gid_text = spec.partition(":")
        uid, gid = int(uid_text), int(gid_text or uid_text)
    except ValueError:
        log(f"WARN --drop-to {spec!r} is not uid:gid — staying as I am")
        return None
    if os.geteuid() != 0:
        return None
    try:
        os.setgroups([])
        os.setgid(gid)
        os.setuid(uid)
    except OSError as error:
        log(f"WARN could not drop to {uid}:{gid} ({error}) — the file will be root's")
        return None
    return f"{uid}:{gid}"


def kernel_timestamp(ancdata) -> tuple[int, int] | None:
    """(seconds, microseconds) out of SCM_TIMESTAMPNS, if the kernel sent it."""
    for level, kind, data in ancdata:
        if level == socket.SOL_SOCKET and kind == SCM_TIMESTAMPNS:
            if len(data) >= 16:
                seconds, nanos = struct.unpack("qq", data[:16])
                return int(seconds), int(nanos) // 1000
    return None


def packet_statistics(sock: socket.socket) -> tuple[int, int]:
    """(seen, dropped) since the last call — reading resets the counters."""
    try:
        raw = sock.getsockopt(SOL_PACKET, PACKET_STATISTICS, 8)
    except OSError:
        return 0, 0
    seen, dropped = struct.unpack("II", raw)
    return int(seen), int(dropped)


class Capture:
    """One pcap file, and the counts needed to trust it."""

    def __init__(self, iface: str, out: Path, *, snaplen: int = DEFAULT_SNAPLEN,
                 peer: str | None = None, max_bytes: int = DEFAULT_MAX_MB * 1000 * 1000,
                 drop_to: str | None = None):
        self.drop_to = drop_to
        self.iface = iface
        self.out = out
        self.snaplen = snaplen
        self.peer = peer
        self.max_bytes = max_bytes
        self.packets = 0
        self.bytes_written = 0
        self.kernel_drops = 0
        self.kernel_seen = 0
        self.first_ts: float | None = None
        self.last_ts: float | None = None
        self.stopped_because = "running"

    def sidecar(self) -> dict:
        """What the file is, and what it is missing. Written next to the pcap."""
        return {
            "interface": self.iface,
            "snaplen": self.snaplen,
            "linktype": "LINKTYPE_LINUX_SLL",
            "filter": "udp" + (f" and host {self.peer}" if self.peer else ""),
            "promiscuous": False,
            "packets": self.packets,
            "bytes": self.bytes_written,
            # Packets the kernel dropped because this process fell behind. A
            # non-zero value here is a gap in the FILE, not in the link, and
            # any loss analysis has to subtract it rather than report it.
            "kernel_drops": self.kernel_drops,
            "kernel_seen": self.kernel_seen,
            "first_timestamp": self.first_ts,
            "last_timestamp": self.last_ts,
            "stopped_because": self.stopped_because,
        }

    def write_sidecar(self) -> None:
        """The sidecar as of now. Cheap enough to call on every checkpoint."""
        try:
            self.out.with_suffix(".json").write_text(
                json.dumps(self.sidecar(), indent=2) + "\n", encoding="utf-8")
        except OSError as error:
            log(f"WARN could not write the sidecar: {error}")

    def run(self, stop: list) -> None:
        sock = open_socket(self.iface)
        # Root was needed for that one call and for nothing after it.
        dropped = drop_privileges(self.drop_to)
        if dropped:
            log(f"privileges dropped to {dropped}; the socket stays open")
        packet_statistics(sock)  # zero the counters; the run owns them from here
        ancillary = socket.CMSG_SPACE(16)
        self.out.parent.mkdir(parents=True, exist_ok=True)
        with open(self.out, "wb", buffering=1 << 20) as handle:
            handle.write(pcap_header(self.snaplen))
            self.bytes_written = 24
            handle.flush()
            # Written before a single packet arrives, so a capture that is
            # killed with its container still explains itself.
            self.write_sidecar()
            next_checkpoint = time.monotonic() + CHECKPOINT_S
            sock.settimeout(0.5)
            while not stop:
                # `tick`, not `now`: the timestamp fallback below rebinds `now`
                # to wall-clock time, and comparing that against a monotonic
                # deadline would checkpoint on every single packet.
                tick = time.monotonic()
                if tick >= next_checkpoint:
                    next_checkpoint = tick + CHECKPOINT_S
                    self._collect(sock)
                    handle.flush()
                    self.write_sidecar()
                try:
                    packet, ancdata, _flags, address = sock.recvmsg(65535, ancillary)
                except socket.timeout:
                    continue
                except OSError as error:
                    log(f"WARN capture socket: {error}")
                    self.stopped_because = f"socket error: {error}"
                    break
                # (ifname, protocol, pkttype, hatype, addr) — everything the
                # SLL header needs, which is why SOCK_DGRAM is enough.
                _name, protocol, pkttype, hatype, addr = address
                if not wanted(protocol, packet, self.peer):
                    continue
                stamp = kernel_timestamp(ancdata)
                if stamp is None:
                    now = time.time()
                    stamp = (int(now), int((now % 1) * 1_000_000))
                body = sll_header(pkttype, hatype, addr, protocol) + packet
                orig_len = len(body)
                clipped = body[: self.snaplen]
                handle.write(record_header(stamp[0], stamp[1], len(clipped), orig_len))
                handle.write(clipped)
                self.packets += 1
                self.bytes_written += 16 + len(clipped)
                seconds = stamp[0] + stamp[1] / 1_000_000
                if self.first_ts is None:
                    self.first_ts = seconds
                self.last_ts = seconds
                if self.bytes_written >= self.max_bytes:
                    log(f"stopping at the {self.max_bytes // 1000 // 1000} MB ceiling "
                        f"after {self.packets} packets — the recording is untouched")
                    self.stopped_because = "size ceiling"
                    break
            handle.flush()
        if self.stopped_because == "running":
            self.stopped_because = "signal"
        self._collect(sock)
        sock.close()

    def _collect(self, sock: socket.socket) -> None:
        seen, dropped = packet_statistics(sock)
        self.kernel_seen += seen
        self.kernel_drops += dropped


# --------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Header-only UDP capture on one interface, written as pcap.")
    parser.add_argument("--iface", required=True, help="interface to capture on")
    parser.add_argument("--out", required=True, type=Path, help="pcap file to write")
    parser.add_argument("--snaplen", type=int, default=DEFAULT_SNAPLEN,
                        help=f"bytes kept per packet (default {DEFAULT_SNAPLEN})")
    parser.add_argument("--peer", help="keep only packets to or from this address")
    parser.add_argument("--max-mb", type=int, default=DEFAULT_MAX_MB,
                        help=f"stop after this many MB (default {DEFAULT_MAX_MB})")
    parser.add_argument("--drop-to", metavar="UID:GID",
                        help="give root back once the socket is open, so the pcap "
                             "belongs to the user that started the session")
    args = parser.parse_args(argv)

    capture = Capture(args.iface, args.out, snaplen=args.snaplen, peer=args.peer,
                      max_bytes=args.max_mb * 1000 * 1000, drop_to=args.drop_to)

    stop: list = []
    for name in (signal.SIGINT, signal.SIGTERM):
        signal.signal(name, lambda *_: stop.append(True))

    log(f"{args.iface} -> {args.out} (-s {args.snaplen}, udp"
        f"{f' and host {args.peer}' if args.peer else ''})")
    try:
        capture.run(stop)
    except PermissionError:
        log("FAIL no permission to open a packet socket. This is meant to run "
            "through `sudo` from inside the communication container, where "
            "containeruser has passwordless sudo; a non-root process cannot "
            "open AF_PACKET whatever capabilities the container was given.")
        return 3
    except OSError as error:
        log(f"FAIL {error}")
        return 3

    capture.write_sidecar()
    sidecar = capture.sidecar()
    log(f"{sidecar['packets']} packets, {sidecar['bytes'] / 1e6:.1f} MB, "
        f"{sidecar['kernel_drops']} dropped by the kernel")
    if sidecar["kernel_drops"]:
        log("WARN the kernel dropped packets: that is a gap in THIS FILE, not "
            "on the link. Subtract it before reading any loss out of the pcap.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
