# A Fragment Sized Exactly Like the Datagram Cap Stops Every Fragmented Sample

## Claim

Setting CycloneDDS's `FragmentSize` **equal to** `MaxMessageSize` makes every
sample large enough to fragment **never arrive** on CycloneDDS 11.0.1, while the
identical profile carries them on 0.10.5. Samples small enough to fit in one
fragment keep flowing on the same link, which is what makes it dangerous: the
endpoints match, neither side logs anything, and the reader simply never sees a
sample.

An RTPS message carries headers as well as the fragment, so a fragment of
exactly `MaxMessageSize` cannot be put into a message of `MaxMessageSize`.
0.10.5 evidently coped with that arithmetic; 11.0.1 does not, and drops
silently.

This is the CycloneDDS sibling of
[the Fast DDS datagram cap](fastdds-datagram-cap-drops-large-samples.md), found
from the opposite direction: there the cap was a value copied *from* this
profile, here the profile's own two values were equal to each other.

## Setup

- Host pair / topology: two hosts joined by a point-to-point VPN (`/24`,
  1.3 ms RTT, 0.0% loss measured with three probes each way), one container per
  host from the packaged communication image, `--network host --ipc host`, one
  ROS domain. Not the local smoke rig: on a single host nothing fragments far
  enough to show it.
- Session: none. A bare `ros2 topic pub -r 1` of a 64 kB
  `std_msgs/msg/String` against a subscriber given the type explicitly — with
  `EnableTopicDiscoveryEndpoints false` a bare `ros2 topic echo <topic>`
  resolves no type and reports nothing, in both versions, which is a trap worth
  naming because it looks exactly like the failure under test.
- rosotacom SHA: measured on the packaged 2.5.dev61 images; re-runs record
  theirs.
- Profile: none — no impairment. The failure needs none, which is what makes it
  attributable to the configuration rather than to the link.
- Seed policy: deterministic (no netem, no random load), `n=1` per variant; the
  effect is present/absent, not a distribution.
- Stacks: `ros-kilted-cyclonedds` 0.10.5-2noble against
  `ros-lyrical-cyclonedds` 11.0.1-4resolute.

## Evidence

| profile | 0.10.5 | 11.0.1 |
|---|---|---|
| `FragmentSize` 1200 B, `MaxMessageSize` 1200 B | arrives | **nothing** |
| no size limits at all | — | arrives |
| `FragmentSize` 1024 B, `MaxMessageSize` 1200 B | — | arrives |

- A 40 B string crosses on 11.0.1 with the 1200/1200 profile, so the trigger is
  fragmentation and not the link.
- The discovery trace of a failing run rules out everything above the data path:
  both participants discover each other across the hosts, both sides announce
  their endpoints, the reading side builds proxy writers for the remote
  participants, and neither side logs a QoS incompatibility or a rejection.

Verification: automated:
`pytest tests/unit/test_ota_link_configs.py::test_only_cyclone_caps_the_datagram_and_fast_dds_must_not`
asserts the fragment stays strictly below the datagram cap, which is the
invariant the two numbers exist for. Manual, for the effect itself: one
container per host from the communication image on one ROS domain, the shipped
OTA profile with only the three size elements varied, a 64 kB string at 1 Hz,
and `ros2 topic echo <topic> std_msgs/msg/String --once` on the other side.

## Status

confirmed, 2026-08-20.

Fixed by making `FragmentSize` strictly smaller than `MaxMessageSize` (1024 B
against 1200 B) in `cyclonedds_tuned.xml.template`. The datagram stays inside
the same MTU budget the 1200 B was chosen for — only the header room is added —
and the value works on both versions, so it is one number rather than a branch
on the distribution.

The 1200 B profile stays reachable, deliberately. It is what every 0.10.5 run
before 2026-08-20 carried, so a comparison against one of those runs has to be
able to reproduce it rather than approximate it:

```yaml
shared:
  rmw:
    ota:
      cyclone: {config: cyclonedds_tuned.xml, fragment_size: 1200B}
```

`get_ota_xml.py` warns on stderr whenever the rendered fragment reaches the
datagram cap, naming 11.0.1 — a profile worth choosing on purpose is not one
worth choosing by accident.

## Publication notes

A distribution bump silently changed what a profile *means*, and the profile
text did not move. That is the reportable part: measurements taken under one ROS
distribution cannot be presented as the other's on the strength of an identical
configuration file, because identical configuration is not identical behaviour
across a DDS major version. State the middleware version beside the profile
wherever OTA numbers are reported.

The second consequence is about what counts as evidence that a link works. A
heartbeat crossing at 10 Hz with 0.0% loss certified nothing here — the small
samples were exactly the ones that could not fail. A link check that does not
include a sample past the fragment threshold does not test the transport the
payload uses.
