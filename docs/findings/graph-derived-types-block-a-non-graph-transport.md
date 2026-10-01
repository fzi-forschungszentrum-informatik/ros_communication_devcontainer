# A Bridge That Learns Types From The Graph Cannot Use A Transport That Does Not Carry The Graph

## Claim

`PubSubPair` resolved a stage's message type from
`get_topic_names_and_types()`, so it could only create its endpoints once
somebody else had created the matching one. On a DDS OTA link that is invisible
— the remote writer is in the same domain, so the graph knows it. Over the
zenoh DDS bridge (`rmw.ota: zenoh_ros2dds`) it is a deadlock: the bridge
creates a subscriber route for a topic only once a **local DDS reader** exists,
and the reader was waiting for the bridge to route a publisher into the graph.
Neither ever happens, the payload topic never arrives, and the only trace is one
word in a log line — `Initialized 1 pair(s). Pending=1`.

The heartbeat escapes it because the receiving peer's bridge sees the remote
liveliness token for it early enough to create the route before the reader is
needed; the wrapped payload topic does not. So the observable is again a link
that carries the small topic and silently drops the big one.

## Setup

- Host pair / topology: two machines over a VPN, `rmw.ota: zenoh_ros2dds`
  (local cyclone), split ROS domains, one 12 kB `SizedPayload` stream at 10 Hz
  plus the echo heartbeat.
- rosotacom SHA: observed at `6e4b748` (before the fix), fixed in `48d84dc`.
- Profile: the drive-derived loss process; the effect is not a loss effect and
  reproduced identically in both runs under it.
- Seed policy: not applicable — present/absent, `n=2` runs, both failing the
  same way.

## Evidence

Evidence grade: two live two-machine runs, read from the receiving peer's node
logs and from the zenoh bridge's own routing log.

Receiving peer, `bridge_in` (2026-08-19):

```
[PubSubPair] Created: sub='/ota/b/heartbeat_b' ...
[PubSubPair] Created: sub='/ota/b/cam_like/ota_stamped' ...
[PubSubPair] Initialized: sub='/ota/b/heartbeat_b' type='com_msgs/msg/EchoHeartbeat'
[bridge_in] Initialized 1 pair(s). Pending=1
```

Receiving peer, `zenoh_bridge_ros2dds`, for the whole run:

```
routes_mgr: Route Subscriber (Zenoh:ota/b/heartbeat_b -> ROS:/ota/b/heartbeat_b) created
```

— and no such line for `ota/b/cam_like/ota_stamped`, while the sending peer had
created its publisher route for exactly that key. Delivered payload messages in
both runs: 0 of ~810 sent.

The fix removes the dependency rather than the symptom: every session now writes
`<peer>/topic_types.yaml` (`<stage topic>: <message type>` for every stage that
peer takes part in, derived from the same pipeline description the status node
uses), the bridge and relay nodes read it, and the graph is consulted only for
what the file does not name.

Verification: `python -m pytest -q tests/unit/test_pub_sub_pair_types.py` — a
pair with a declared type creates its endpoints against an empty graph, a pair
without one still reads the graph, and a pair with neither stays pending.
`tests/unit/test_ota_link_configs.py::test_every_peer_gets_the_stage_types_its_bridges_need`
pins that the file is generated for every peer of every session.

## Status

confirmed, 2026-08-19.

## Publication notes

Worth stating in any "the transport is a configuration choice" claim: a
pipeline can depend on a transport property nobody wrote down — here, that the
transport propagates the ROS graph and not only the data. DDS does, the zenoh
DDS bridge does not, and the dependency only shows up as a topic that never
arrives. The general lesson is that a contract known at generation time should
be handed to the runtime rather than rediscovered from the runtime, which is
also what makes bring-up order-independent.
