# Best-Effort Reordering Becomes Reader-Side Loss, Independent Of History Depth

## Claim

Under delay jitter large enough to reorder packets — with ZERO configured
loss — a CycloneDDS best-effort reader discards every overtaken sample to
preserve per-writer order: delivery is strictly monotonic, the discarded
samples count as loss, and subscription history depth does not recover any of
it (the drop happens in the reader proxy, before history). Variance is
therefore a loss mechanism of its own, and it is the one QoS depth cannot
help against — unlike bundled-publication overwrite, where depth does help.

## Setup

- Host pair / topology: two Docker containers on one machine (the repository's
  communication image; any image with `rclpy`, `rmw_cyclonedds_cpp` and `tc`
  works), unicast Cyclone peers (`AllowMulticast=false`, static IPs), one
  `std_msgs/Int64` stream at 100 Hz, publisher history depth 10, best-effort;
  the receiver subscribes the SAME topic twice: depth 1 and depth 50. netem on
  the publisher's egress only, delay+jitter, no loss, no rate limit.
- rosotacom SHA: measured at `3227daf`; re-runs record theirs.
- Profile: inline netem args (`delay 50ms 45ms` and `delay 50ms 20ms`), not a
  packaged profile — the experiment is below rosotacom, at the RMW layer.
- Seed policy: seedless single runs (`n=1` per jitter level); the effect size
  (tens of percent) dwarfs run-to-run noise.

## Evidence

Evidence grade: scripted container reproduction, committed next to this file.

```bash
python3 docs/findings/repro/reorder_drop_repro.py orchestrate --rate 100 --duration 40
```

2026-08-18 run (4000 messages sent per scenario, zero configured loss):

| netem | depth 1 delivered | depth 50 delivered | inversions | loss of span |
|---|---|---|---|---|
| `delay 50ms 45ms` | 1709 | 1727 | 0 | **57%** |
| `delay 50ms 20ms` | 2626 | 2659 | 0 | **34%** |

Three readings, one per column group: delivery is strictly monotonic (zero
inversions in every subscription), depth 50 loses the same as depth 1 (the
0.5–1 pp difference is executor noise, not recovery), and the loss tracks the
overtake probability — at 100 Hz the inter-message gap is 10 ms, so jitter
±45 ms makes "the next message arrives first" the common case.

Field context (2026-08-17 cellular drive): the operational 10 Hz streams have a
100 ms gap against ~18 ms steady jitter, so this channel is minor there
(neighbour-delay overtake proxy: 9.6% around steady losses vs 6.0% baseline).
It grows quadratically relevant as rates rise or jitter grows — any stream
whose inter-message gap approaches the link's jitter pays it.

Verification: manual: run the command above from a source checkout with
Docker (build the image via the packaged `ros2docker.json.example` or pass
`--image`); the script prints and writes `out/reorder_results.json`.

## Status

confirmed, 2026-08-18.

## Publication notes

This is the mechanism behind the jitter side of
[jitter-loss-bandwidth-latency.md](jitter-loss-bandwidth-latency.md): the
jitter boundary is not radio noise but the reader's monotonicity rule. For
papers: plot delivered share vs jitter/gap ratio, and state explicitly that
history depth is not a mitigation here — pacing (larger gaps), FEC, or
reliable QoS are.
