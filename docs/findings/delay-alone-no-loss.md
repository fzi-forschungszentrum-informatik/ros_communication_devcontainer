# Delay Alone Was Not The Loss Boundary For The 18 KB At 20 Hz Probe

## Claim

Pure uplink delay, without jitter, configured loss, or bandwidth limiting, was
not the observed loss boundary for the 18 KB at 20 Hz benchmark probe: the
300 ms delay-only run delivered all messages, while lower delay-only repeats
showed only residual single-message losses.

## Setup

- Host pair / topology: rosotacom local benchmark mode using the packaged example
  project, synthetic `bench_1_1_capacity` session, one `a->b:/bench_capacity`
  stream, uplink shaping only.
- rosotacom SHA: original migrated summary was measured at `9dfc157`; public
  re-runs record the current checkout SHA in `result.json.context`.
- Profiles: `finding-delay-5ms`, `finding-delay-50ms`, and
  `finding-delay-300ms` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml),
  with delay-only uplink shaping and empty downlink.
- Seed policy: delay-only profiles have no stochastic netem seed; lower-delay
  profiles use seedless repeats and `finding-delay-300ms` is a seedless single
  run in the migrated evidence.

## Evidence

Evidence grade: migrated benchmark summaries plus exact public re-run commands.

5 ms delay:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-delay-5ms --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 10 attempts, 5123 expected, 5123 delivered, 0 lost.

50 ms delay:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-delay-50ms --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 10 attempts, 5133 expected, 5123 delivered, 10 lost, max
attempt loss 0.196%, max p95 latency 56.817 ms. This run is not perfectly
zero-loss, but its low loss is not consistent with delay being the dominant loss
mechanism.

300 ms delay:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-delay-300ms --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 1 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 400 expected, 400 delivered, 0 lost, max p95 latency
306.927 ms.

Field recurrence (2026-08-17 cellular drive): the negative control held on a real
cellular link — five jam episodes ran one-way delay up to 401–459 ms with zero
losses, and the drive's soggy last minute held p90 ≈ 138 ms at 0.04 losses/s.
The positive counterpart (where the delay actually comes from when a link is
too slow rather than too far) is
[oversubscription-queues-not-losses.md](oversubscription-queues-not-losses.md).

Verification: manual: run the commands above from a source checkout with Docker
and `tc` privileges; this is a no-loss negative-control finding rather than a
must-fail boundary row, so it remains outside the performance bands reference boundary gate.

## Status

confirmed, 2026-08-18.

## Publication notes

Feeds the network-characterization paper as a negative control. The plot should
separate fixed latency from jitter and bandwidth constraints, and should mention
the small lower-delay losses as residual benchmark noise rather than a delay
boundary.
