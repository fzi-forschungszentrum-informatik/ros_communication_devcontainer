# A Fast DDS Datagram Cap Stops Every Message Larger Than It

## Claim

Setting `<maxMessageSize>` on a Fast DDS UDPv4 transport to a value below a
sample's serialized size makes that sample **never arrive** through the OTA
pipeline — not "arrive fragmented", not "arrive with loss". Messages that fit
inside the cap keep flowing at full rate on the same link, which is what makes
this dangerous: an 84 B heartbeat crossing at 10 Hz with 0.0% loss is a link
that looks alive from every direction except the one that matters.

The obvious reason to set it is symmetry with CycloneDDS, whose OTA profile
carries this link at a 1024 B fragment inside a 1200 B datagram and works. The
symmetry does not transfer: on ROS 2 Kilted with Fast DDS 3.2.4 the cap is the
single knob that breaks the link, at 1200 B and at 8192 B, with a synchronous
writer and with an asynchronous one.

## Setup

- Host pair / topology: one host, the packaged local smoke rig — two
  communication containers on their own Docker network, split ROS domains
  (peer 50, peer 51, OTA 52) with the stock `domain_bridge` between them. The
  same failure was first seen across two machines over a VPN; one host
  reproduces it, so nothing private is needed.
- Session: a two-peer session with a `com_msgs/msg/SizedPayload` stream at
  12000 B and 10 Hz on the loaded direction plus the echo heartbeat, OTA QoS
  best_effort / depth 1 / lifespan 0.7 (the deployed shape).
- rosotacom SHA: measured at `6e4b748`; re-runs record theirs.
- Profile: none for the bisect — the failure needs no impairment, which is what
  makes it attributable to the configuration rather than to the link.
- Seed policy: deterministic (no netem, no random load), `n=1` per variant; the
  effect is present/absent, not a distribution.

## Evidence

Evidence grade: single-host reproduction on packaged material, one variant per
run, read from the receiving peer's own `status.txt`.

Each knob of the tuned profile was added to the working `fastdds_unicast.xml`
one at a time (2026-08-19):

| added to fastdds_unicast.xml | 12 kB stream at the receiver |
|---|---|
| interface allowlist (127.0.0.1 + OTA address) | FLOWING 10.0 Hz, 0.0% loss |
| loopback locators (default / metatraffic / initial peers) | FLOWING 10.0 Hz |
| `maxInitialPeersRange` 10 | FLOWING 10.0 Hz |
| discovery lease / announcement | FLOWING 10.0 Hz |
| socket buffer sizes (1 MB / 4 MB) | FLOWING 10.0 Hz |
| `non_blocking_send` | FLOWING 10.0 Hz |
| `publishMode` ASYNCHRONOUS | FLOWING, degraded: 9.3 Hz at 285 ms |
| **`maxMessageSize` 1200** | **IDLE — nothing delivered** |
| **`maxMessageSize` 8192** | **IDLE — nothing delivered** |
| **`maxMessageSize` 1200 + ASYNCHRONOUS** | **IDLE — nothing delivered** |

The heartbeat crossed at 10 Hz with 0.0% loss in every one of those runs,
including the three that delivered no payload at all.

Two nearby observations, both of which cost time before the bisect narrowed it:

- Two Fast DDS participants in two containers, with the capped profile and
  nothing else running, DO exchange 12 kB and 38 kB samples (57–59 of 60). The
  cap only breaks the full pipeline, where the sample crosses a domain bridge
  and an OTA bridge, so an isolated transport probe will report the cap as
  harmless.
- Asynchronous publication does not rescue it, which rules out the documented
  "a synchronous writer cannot fragment" explanation as the whole story.

Verification: manual: run the packaged smoke for a session whose OTA side names
`fastdds_tuned.xml`, then again for one that adds `<maxMessageSize>1200` to it,
and read `logs/<receiver>/status/status.txt`; the payload stage is FLOWING in
the first and IDLE in the second while the heartbeat is FLOWING in both. The
shipped profiles are pinned against the cap by
`tests/unit/test_ota_link_configs.py::test_only_cyclone_caps_the_datagram_and_fast_dds_must_not`.

## Status

confirmed, 2026-08-19.

## Publication notes

This is why "the OTA middleware is a configuration choice" needs measuring
rather than reasoning: the two profiles that look like translations of each
other are not, and the failure mode of the wrong translation is a link that
passes a heartbeat check. Consequence for the transport comparison: Fast DDS
lets IP fragment a large sample here where CycloneDDS sends RTPS fragments, so
the two differ in what a lost packet costs and in what the tunnel has to
reassemble — the measured OpenVPN barrier on the operator's path is around
110 kB per message. Report Fast DDS numbers with that difference stated, not as
a like-for-like transport swap.
