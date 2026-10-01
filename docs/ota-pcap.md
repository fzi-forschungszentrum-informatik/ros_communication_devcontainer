# Header-Only OTA Capture

A passive, header-only packet capture of the OTA link, written beside the
[link trace](link-trace.md) by the same node:

```text
session-instances/.../logs/<peer>/status/ota.pcap
session-instances/.../logs/<peer>/status/ota.json
```

Enable it in a session definition:

```yaml
shared:
  use_status_overview: true
  ota_pcap:
    enabled: true
```

Everything else is optional:

```yaml
  ota_pcap:
    enabled: true
    snaplen: 96        # bytes kept per packet; headers, no payload
    max_mb: 2000       # ceiling for one capture file
    peer_filter: true  # keep only packets to or from the remote peer
```

Off by default, and off is exactly the session that existed before: a
definition without the block renders the same peer files it always did, which
a test asserts by rendering both and diffing them.

## What it answers that the link trace cannot

`link_trace.jsonl` counts `/proc/net/dev` bytes per direction once a second.
That localises loss to a **direction** and no further. It cannot say which
fragment of which sample was lost, whether a retransmit followed, or how one
sample's sub-messages were spaced on the wire.

The capture answers those, at roughly 1 % of payload volume. Use the two
together: the trace is the cheap continuous record, the capture is what you
turn on for a measurement run.

## What the file is

`LINKTYPE_LINUX_SLL` (113), microsecond `pcap`, snaplen 96 by default. Every
reader takes it — Wireshark, `tshark`, `scapy`, `capinfos`.

The link type is the important choice. The capture uses a `SOCK_DGRAM` packet
socket, so one code path works on both link types the fleet has: a VPN `tun`
device is layer 3 and has no link-layer header at all, while a control-centre
LAN port is Ethernet. The kernel hands `SOCK_DGRAM` the network-layer packet
either way, plus the `sockaddr_ll` that `LINKTYPE_LINUX_SLL` is defined over.

That pseudo-header carries the field the analysis is for:

```bash
# uplink (this host sent it) against downlink, per packet
tshark -r ota.pcap -T fields -e sll.pkttype | sort | uniq -c
#   4 = PACKET_OUTGOING, 0 = PACKET_HOST
```

A `LINKTYPE_RAW` capture would have thrown that away.

Timestamps come from the kernel (`SO_TIMESTAMPNS`), not from the moment python
reached the packet.

## The sidecar, and the number that matters

`ota.json` is written when the capture stops:

```json
{
  "interface": "tun0",
  "snaplen": 96,
  "linktype": "LINKTYPE_LINUX_SLL",
  "filter": "udp and host 10.20.0.37",
  "promiscuous": false,
  "packets": 2434,
  "bytes": 272498,
  "kernel_drops": 0,
  "kernel_seen": 4638,
  "first_timestamp": 1787297717.008177,
  "last_timestamp": 1787297733.10892,
  "stopped_because": "signal"
}
```

`stopped_because` reads `running` until something stops the capture, and that
is deliberate: **neither file depends on a clean shutdown.** A session usually
ends by its container being stopped, which does not reach this process as a
signal it can act on — the first bench run ended exactly that way and left a
pcap truncated at a buffer boundary and no sidecar at all. The pcap is flushed
once a second and the sidecar is written at the start and rewritten on the same
interval, so a killed capture still leaves a readable file and a description of
it. Verified by `SIGKILL`ing a running capture: `capinfos` reads the pcap and
the sidecar reports the packets up to the last checkpoint.

**`kernel_drops` is a gap in the file, not on the link.** A capture that fell
behind looks exactly like radio loss to anyone reading the pcap months later,
so subtract it before reading any loss out of the capture. Zero is the normal
result on a link of a few Mbit/s; a non-zero value means this process, not the
radio.

## Which interface, and the refusal that matters

The interface is not configured. It is resolved from the peer's own OTA
address by `com_py.link_bytes.resolve_link_interface` — the same resolution the
status snapshot's `link` block uses, including its refusal to report a loopback
interface for a non-loopback address. That refusal is issue #267: sampling `lo`
on a machine that also runs an `-a` recorder reads about 28 Gbit/s next to a
6 Mbit/s link, and it survived a field session and an offline analysis before
anyone doubted the number. A capture on the wrong device would be the same
mistake in a bigger file.

If no interface resolves, the capture is skipped with a warning and the session
runs on. `shared.ota_pcap.enabled` therefore requires
`shared.use_status_overview: true`, which is where that resolution lives.

## Privilege, containers, and what the option costs

No image change, no rebuild, no added capability and no extra container in
`docker ps`. What it does need is `sudo`, and only until the socket is open.

ros2docker's entrypoint runs this container's processes as `containeruser`,
whose `CapEff` is `0000000000000000`. A non-root process cannot open an
`AF_PACKET` socket however the container was started, so `--cap-add NET_RAW`
would buy nothing. `containeruser` does have passwordless sudo here — the NET
pane's `sudo iftop` already depends on it — so the capture is launched through
it, and `--drop-to` hands the process back to `containeruser` the moment the
socket exists. The pcap therefore belongs to the same user as
`link_trace.jsonl` beside it, rather than to root.

Two earlier attempts at this assumed root from a `docker run` that bypassed the
entrypoint, and were wrong in the same way both times. If you change this,
check the **running** container:

```bash
docker exec <com-container> bash -lc 'id; grep CapEff /proc/self/status'
```

`CAP_NET_ADMIN` is *not* taken and is not needed: it buys promiscuous mode,
which this capture deliberately does not use. Everything of interest is
addressed to or from this host, and promiscuous mode on a control-centre LAN
port would collect other people's traffic.

The capture runs as a separate process rather than inside the status node's
thread, so a capture that misbehaves cannot reach the snapshot an operator is
watching. It is stopped with `SIGINT` and waited for, because that is what
writes the sidecar JSON.

It is launched by **file path**, not as `python3 -m com_py.ota_pcap`. sudo
resets the environment, so `PYTHONPATH` does not survive it and the package is
not importable on the other side — which is what a two-host run answered,
`No module named 'com_py'`, once the permission problem was out of the way.
`ota_pcap.py` imports nothing but the standard library, so a path needs no
package at all, and a test pins that it stays that way.

## Cost

Header-only at 96 bytes: about 6 MB per minute at 500 packets/s, so a two-hour
run is well under a gigabyte. `max_mb` bounds a mistake — a capture pointed at
a busy LAN port — rather than shaping a run; when it is reached the capture
stops and says so in `stopped_because`.

Nothing is transmitted, so the capture costs no link capacity.
