# Jitter Causes Loss While Bandwidth Shortage Builds Latency

## Claim

For 18 KB at 20 Hz, injected jitter causes message loss once the 30 ms delay /
27 ms jitter profile is reached, while a slight bandwidth shortage at 3.1 Mbit/s
primarily builds latency before or alongside loss.

## Setup

- Host pair / topology: rosotacom local benchmark mode using the packaged example
  project, synthetic `bench_1_1_capacity` session, one `a->b:/bench_capacity`
  stream, uplink shaping only.
- rosotacom SHA: original migrated summaries were measured at `678be64` for the
  June 29 capacity probes and `2313a26` for the July 1 latency-buildup probe;
  public re-runs record the current checkout SHA in `result.json.context`.
- Profiles: `finding-jitter18`, `finding-jitter27`, `finding-3.2mbit`, and
  `finding-3.1mbit` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml).
- Seed policy: seedless netem runs; the boundary probes use seedless replicates
  (`n=9` or `n=10`) for generality, and the latency-buildup probe is a seedless
  single diagnostic run (`n=1`).

## Evidence

Evidence grade: migrated benchmark summaries plus exact public re-run commands.

Good jitter boundary:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-jitter18 --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 10 attempts, 4000 expected, 4000 delivered, 0 lost, max p95
latency 85.501 ms.

Bad jitter boundary:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-jitter27 --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 10 attempts, 4000 expected, 3957 delivered, 43 lost, max
attempt loss 1.5%, max p95 latency 107.689 ms.

Good bandwidth boundary:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-3.2mbit --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 9 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 9 attempts, 3600 expected, 3600 delivered, 0 lost, max p95
latency 45.942 ms.

Bad bandwidth boundary:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-3.1mbit --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 9 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 9 attempts, 3598 expected, 2897 delivered, 701 lost, max
attempt loss 20.0%, max p95 latency 114.336 ms.

Latency-buildup diagnostic:

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-3.1mbit --size 18000 --rate-hz 20 --duration 20 --repeats 1 --no-plot
```

Migrated summary: 406 expected, 406 delivered, 0 lost, p95 latency rose to
447.429 ms by the last time bin under the 3.1 Mbit/s profile.

Verification: nightly row `boundary-loss-18kb20hz-bandwidth-cyclone` re-proves
the deterministic bandwidth boundary; the jitter boundary and latency-buildup
diagnostic remain manual with the commands above because public runners cannot
install seeded random netem yet.

## Status

confirmed, 2026-07-07.

## Publication notes

Feeds the network-characterization paper as a contrast plot: jitter-boundary
loss vs bandwidth-boundary queue buildup. The paper adaptation should plot
per-attempt loss and p95 latency, and it should label the seed policy as
seedless replicates.
