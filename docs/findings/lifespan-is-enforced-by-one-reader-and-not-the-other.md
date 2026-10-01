# Lifespan Is Enforced By One DDS Reader And Not The Other

## Claim

Under identical oversubscription the two DDS stacks queue identically — 472 and
433 delivered of ~490 offered, at medians of 1146 ms and 1220 ms. The stacks
diverge only when `lifespan` is switched on, and then completely: CycloneDDS is
unchanged (472 delivered, 1143 → 1146 ms, **9.1%** of samples inside the 0.7 s
bound they were published with), while Fast DDS drops from 433 delivered to
**4**, at a 344 ms median, and **never delivers a sample older than 570 ms**.

`rmw_cyclonedds` does not enforce lifespan at the reader; Fast DDS does. Neither
enforcement shortens the delay — the backlog is below DDS and unrecallable
either way. What lifespan decides is whether a late sample is **shown or
suppressed**, and for an operator picture those are different failures: 7.9 Hz of
video more than a second old, against 0.07 Hz of video that is current.

The corollary matters for reading any oversubscription result: **Fast DDS's
apparent collapse under a cap is not a transport failure.** With the policy off
it carries the same load as CycloneDDS. The missing messages are its reader
discarding what arrived too late.

## Setup

- Host pair / topology: one host, two containers on a private docker bridge
  (172.31.77.0/24), publisher and subscriber in separate containers. Single host
  on purpose: both sides read the same `CLOCK_REALTIME`, so a sample's age is
  measured without a clock-offset term.
- Session: a two-process rclpy probe, not the session pipeline — one
  `std_msgs/String` topic at 12000 B and 10 Hz (960 kbit/s payload), QoS
  best_effort / KEEP_LAST depth 1, publisher started after the subscriber so
  discovery is not inside the measurement. The payload carries its own sequence
  number and publish stamp.
- rosotacom SHA: not involved; ROS 2 Kilted, Fast DDS 3.2.4 /
  `rmw_fastrtps_cpp` 9.3.4, CycloneDDS via `rmw_cyclonedds_cpp`, image
  `ros-communication:latest`.
- Profile: `tc qdisc add dev eth0 root tbf rate 800kbit burst 64kb latency
  2000ms` on the publisher container's egress — 20% below the offered payload,
  with a 2 s buffer, i.e. a deliberate standing queue. 60 s per run.
- Both middlewares configured with explicit unicast discovery (the bridge
  carries unicast but not the DDS discovery multicast), so neither runs on
  defaults.
- Seed policy: no stochastic netem elements (rate only); one run per cell,
  reported as the four-cell A/B rather than as a distribution.

## Evidence

Evidence grade: single-host container runs, per-sample age computed from a
publish stamp inside the payload against one clock, sequence numbers giving loss
independently of the age.

| middleware | lifespan | delivered | age p50 | age p95 | age max | inside 0.7 s |
|---|---|---|---|---|---|---|
| CycloneDDS | 0.7 s | 472 | 1146 ms | 1449 ms | 1533 ms | **9.1%** |
| CycloneDDS | 86400 s (off) | 472 | 1143 ms | 1449 ms | 1533 ms | — |
| Fast DDS | 0.7 s | **4** | **344 ms** | 570 ms | **570 ms** | **100%** |
| Fast DDS | 86400 s (off) | **433** | 1220 ms | 1459 ms | 1557 ms | — |

Three readings:

1. **The two stacks queue the same.** With the policy off, 472 against 433 and
   1143 ms against 1220 ms. Interface accounting agrees: 811 and 812 kbit/s, both
   pinned to the 800 kbit/s cap. Whatever separates them under `lifespan: 0.7` is
   not capacity and not the transport.
2. **CycloneDDS's A/B is a null result, and that is the finding.** 472/472,
   1146/1143 ms — turning the policy off changes nothing measurable, because the
   policy was doing nothing. Only 9.1% of what it delivers is inside the bound it
   was published with.
3. **Fast DDS's A/B is the opposite.** 4 against 433, and the max age under the
   policy is 570 ms — below 700 ms, not approximately below it. The samples that
   vanish are the ones the shaper delayed past their lifespan.

The same pair of runs also shows the delay is a queueing discipline rather than a
property of the path: replacing `tbf` with `htb` + `fq_codel` at the same
800 kbit/s takes CycloneDDS from 472 delivered at 1146 ms to 8 delivered at
**124 ms**, with 100% inside the lifespan. An AQM converts staleness into loss;
it does not create capacity.

Verification: manual: two containers on a private bridge, `tbf` as above on the
publisher's egress, one 12 kB/10 Hz best_effort depth-1 stream, run once with
`lifespan: 0.7` and once with it effectively off, for each RMW. The Cyclone pair
must be indistinguishable and the Fast DDS pair must differ by two orders of
magnitude in delivered count, with the `lifespan: 0.7` arm's maximum age below
the lifespan.

## Status

confirmed, 2026-08-19.

## Publication notes

This corrects, from the reader side, the conclusion drawn in the operator
harness's middleware comparison that lifespan is "indistinguishable for both DDS
stacks" — that A/B's Fast DDS arm ran against a zero-byte profile. It is the
enforcement half of
[oversubscription-queues-not-losses.md](oversubscription-queues-not-losses.md):
that finding shows the excess becomes delay rather than loss, this one shows what
each reader then does with the delayed sample.

For papers, the useful framing is that "loss" and "delivered" are not
middleware-comparable numbers unless the lifespan enforcement is stated with
them. A table that puts Fast DDS's 4 next to CycloneDDS's 472 without that column
reads as a transport difference and is not one.

Practical consequence: neither behaviour is usable at 20% oversubscription, so
this is a choice about how to fail, not a fix. The fix is below DDS (offer fewer
bytes than the link carries) or above it (receiver-side age gating that can skip
rather than render).
