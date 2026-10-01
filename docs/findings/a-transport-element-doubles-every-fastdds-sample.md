# Any Transport Element In A Fast DDS Profile Doubles Every Sample On The Wire

## Claim

A Fast DDS participant profile that carries **any** transport element transmits
every sample twice. It is not a property of the transport that gets selected:
`<builtinTransports>DEFAULT</builtinTransports>` asks for exactly the built-in
set — a semantic no-op — and doubles the wire just as a full custom
`<userTransports>` descriptor does. Removing the element is the entire fix.

Measured against a 960 kbit/s payload, on the profile this repository ships:

| participant profile carries | interface tx |
|---|---|
| `<userTransports>` + `<useBuiltinTransports>false` (as shipped) | 1741 kbit/s |
| `<builtinTransports non_blocking="true">UDPv4` | 1741 kbit/s |
| `<builtinTransports>UDPv4` | 1741 kbit/s |
| `<builtinTransports>DEFAULT` | 1741 kbit/s |
| **no transport element** | **1007 kbit/s** |

Counted per datagram: **366** to the reader's data locator for **180** published
samples, against **183** without the element — each carrying a full 11 951 B of
UDP payload, to the same address and the same port. The reader discards the
second copy by sequence number, so it is delivered exactly once and costs half
the link.

The consequence for any published wire-volume comparison is direct: a Fast DDS
row measured with such a profile is a measurement of this defect, not of the
middleware.

## Setup

- Host pair / topology: one host, two containers on a private docker bridge
  (172.31.77.0/24), publisher and subscriber in separate containers; and the
  same profile on the host network of a multi-homed machine (38 interfaces:
  docker bridges, a VPN tun, WLAN) for the discovery checks.
- Session: a two-process rclpy probe, not the session pipeline — one
  `std_msgs/String` topic at 12000 B and 10 Hz, best_effort / KEEP_LAST depth 1,
  publisher started after the subscriber. Bytes are read from
  `/sys/class/net/eth0/statistics/tx_bytes` around the run; datagrams are counted
  from a `tshark` capture with `ip.defragment:FALSE`, so the packet count and the
  interface counter agree.
- rosotacom SHA: `fastdds_tuned.xml.template` as of this commit, rendered by
  `get_ota_xml.py`. ROS 2 Kilted, Fast DDS 3.2.4, `rmw_fastrtps_cpp` 9.3.4,
  image `ros-communication:latest`.
- Profile: no shaping for the wire-volume rows — the point is what the stack
  offers, not what a link does with it.
- Seed policy: deterministic; one run per cell, and the cells that share a
  configuration agree to the byte (4 996 526 B across three different descriptor
  shapes), which is what makes single runs readable here.

## Evidence

Evidence grade: interface counters plus a packet capture of the same runs, on
both a synthetic profile and the rendered shipped one.

The trigger is the element, not its contents. Four descriptor shapes, all 366
datagrams for 180 samples: a bare `<transport_descriptor>` with nothing but
`transport_id` and `type`; the same plus `maxInitialPeersRange`; the same with
`<useBuiltinTransports>` set to `true` instead of `false`; and the same
allowlisted to a single interface. The single-interface row also refutes the
obvious alternative explanation — that the participant sends one copy per bound
interface.

On the rendered `fastdds_tuned.xml`, correcting for the ~40 messages published
before discovery completes (a Fast DDS writer with no matched reader puts nothing
on the wire, and this profile takes 3.9–4.2 s to match):

| | interface tx | transmitted payload | ratio |
|---|---|---|---|
| as shipped | 6516 kB | 3109 kB | **2.10×** |
| with the element removed | 3270 kB | 3145 kB | **1.04×** |

Both delivered 245 of 245, at a sub-millisecond median age. Nothing is traded for
the halving. For scale, CycloneDDS on the same bench and payload costs 963 kbit/s
— **1.00×** — so removing the element brings Fast DDS from twice Cyclone's wire
need to within 4% of it.

Two properties the removed descriptor carried were checked rather than assumed,
on the multi-homed host the profile exists for:

- **The same-host path stays open.** 195 of 195 delivered, discovery in 4.13 s,
  across 38 interfaces. The pinned `defaultUnicastLocatorList` /
  `metatrafficUnicastLocatorList` already say what the interface allowlist said.
- **Discovery survives the lost `maxInitialPeersRange`.** With 12 extra
  participants started first, so the probe processes take high participant
  indices: 195 of 195, discovery in 4.24 s, unchanged.

What is genuinely given up: `<non_blocking_send>true</non_blocking_send>` goes
with the descriptor, so a full send buffer blocks the writer instead of dropping.
That is the smaller cost — a blocking send stalls only while the link is behind,
where the duplicate costs half the link at all times.

Verification: automated:
`pytest tests/unit/test_ota_link_configs.py::test_no_fastdds_profile_names_a_transport`
guards the deletion in every packaged Fast DDS profile. Manual, for the effect
itself: two containers on a private bridge, one 12 kB/10 Hz stream, read
`tx_bytes` around a 30 s run with and without a `<builtinTransports>DEFAULT</builtinTransports>`
line in the participant profile — the ratio must be 2:1.

## Status

confirmed, 2026-08-19.

## Publication notes

This is a Fast DDS 3.2.4 / `rmw_fastrtps_cpp` 9.3.4 behaviour and is worth
reporting upstream; the `DEFAULT` row is the reproducer to lead with, because it
removes every question about what the profile was asking for.

It retires the "Fast DDS's lower loss is bought with bandwidth" reading of this
project's wire-volume table: roughly half of the excess over CycloneDDS was this,
and the remainder is still unattributed. Pair with
[lifespan-is-enforced-by-one-reader-and-not-the-other.md](lifespan-is-enforced-by-one-reader-and-not-the-other.md)
— together they account for both columns on which Fast DDS looked like a
different kind of stack: it costs about what Cyclone costs, and it delivers less
because it refuses to deliver stale.
