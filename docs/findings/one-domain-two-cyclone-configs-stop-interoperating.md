# Two CycloneDDS Configurations on One Domain Stop Interoperating

## Claim

On CycloneDDS 11.0.1 two participants on the **same host and the same domain**
exchange no data when they are given different configurations. On 0.10.5 they
do. Discovery is unaffected in the failing combination — the topic appears in
`ros2 topic list`, both sides announce their endpoints, the reader builds its
proxy writers — and no sample is delivered.

This is what a session relies on without saying so. A peer renders two whole
configurations, a local one and an OTA one, and exports `CYCLONEDDS_URI` twice,
so the second replaces the first: processes started after it view the *local*
domain through the *OTA* profile while the relay and the application containers
view it through the local one. Under 0.10.5 that mixture worked. Under 11.0.1
the bridge subscribes on the local domain, receives nothing, forwards nothing
into the OTA domain, and the far peer receives nothing — with every stage
reporting itself healthy.

## Setup

- Host pair / topology: for the effect itself, one host is enough — two
  containers from the same communication image, `--network host`, one ROS
  domain, one publisher and one subscriber. The session it was found in runs
  across two hosts joined by a point-to-point VPN.
- Session: none for the isolated measurement; a `std_msgs/msg/String` at 2 Hz
  and `ros2 topic echo … --once`. The two configurations are the ones a session
  renders: `cyclonedds_local_participants.xml` and `cyclonedds_tuned.xml`.
- rosotacom SHA: measured on the packaged 2.5.dev61 images; re-runs record
  theirs.
- Profile: none — no impairment; the effect needs none.
- Seed policy: deterministic (no netem, no random load), `n=1` per cell; the
  effect is present/absent, not a distribution.
- Stacks: `ros-kilted-cyclonedds` 0.10.5-2noble against
  `ros-lyrical-cyclonedds` 11.0.1-4resolute.

## Evidence

| publisher / subscriber | 0.10.5 | 11.0.1 |
|---|---|---|
| local / local | 12 B | 12 B |
| local / **ota** | 12 B | **0** |
| **ota** / local | 12 B | **0** |
| ota / ota | 12 B | 12 B |

Inside a failing session, one topic the relay publishes on the local domain:

```
/com/out/<peer>/can/twist/max10hz/ota_stamped, domain 47
  subscriber with local_dds.xml : 512 B received
  subscriber with ota_dds.xml   :   0 B
domain 47 seen through local_dds.xml : 103 topics
domain 47 seen through ota_dds.xml   :  29 topics
```

Ruled out on the way to it, each measured with the real images on the same
hosts: cross-host delivery, same-host delivery under one profile,
`com_msgs/msg/OtaStamped`, the container's IPC namespace, `domain_bridge`
itself (minimal config, the session's full 27-topic config, with and without the
OTA profile, with and without `ROS_DOMAIN_ID` set), participant count (0, 6 and
12 extra participants), and adding the local profile's
`ParticipantIndex`/`MaxAutoParticipantIndex` to the OTA profile.

Verification: automated:
`pytest tests/unit/test_ota_xml.py::test_shipped_scoped_template_carries_both_domains_and_only_ota_restrictions`
holds the shape of the configuration that avoids the mixture — OTA restrictions
scoped to the OTA domain, the local domain left with default discovery. Manual,
for the effect itself: two containers on one host and one domain, one publisher
and one subscriber, each given one of the two rendered profiles, all four
combinations.

## Status

confirmed, 2026-08-20.

Addressed by `cyclonedds_scoped.xml.template`, which carries both domains in one
file with each section scoped by `Domain Id`, so one configuration can be shared
by every process on the host. It is an additional template: the two-file
arrangement is unchanged, and a session that does not name the new one behaves
exactly as before.

## Publication notes

The reportable part is not "a DDS version changed". It is that a configuration
which was *never* uniform became load-bearing without anybody choosing it: the
mixture had been there all along and was harmless, so nothing recorded it as a
property of the deployment. A measurement stack should state, beside the
middleware version, whether every process on a domain was configured
identically — because on one side of that version boundary the answer does not
matter and on the other it decides whether any data moves.

Second, it is another instance of the same trap as the datagram cap: a link that
passes discovery, endpoint matching and a heartbeat check while carrying
nothing. Neither check tests what the payload uses.
