# Zero-Loss Boundaries For 18 KB At 20 Hz

## Claim

For the 18 KB at 20 Hz synthetic stream, 3.2 Mbit/s bandwidth and 18 ms jitter
were observed as zero-loss good cases, while 3.1 Mbit/s bandwidth and 27 ms
jitter were observed as bad cases; the combined 3.2 Mbit/s plus 18 ms jitter case
was also zero-loss.

## Setup

- Host pair / topology: rosotacom local benchmark mode using the packaged example
  project, synthetic `bench_1_1_capacity` session, one `a->b:/bench_capacity`
  stream, uplink shaping only.
- rosotacom SHA: original migrated summaries were measured at `678be64`; public
  re-runs record the current checkout SHA in `result.json.context`.
- Profiles: `finding-3.2mbit`, `finding-3.1mbit`, `finding-jitter18`,
  `finding-jitter27`, and `finding-3.2mbit-jitter18` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml).
- Seed policy: seedless netem replicates (`n=9` for the two bandwidth-only
  boundary runs; `n=10` for the jitter and combined runs). No `seed` field is
  recorded in the profiles.

## Evidence

Evidence grade: migrated benchmark summaries plus exact public re-run commands.

Good bandwidth:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-3.2mbit --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 9 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 3600 expected, 3600 delivered, 0 lost.

Bad bandwidth:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-3.1mbit --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 9 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 3598 expected, 2897 delivered, 701 lost, max attempt loss
20.0%.

Good jitter:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-jitter18 --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 4000 expected, 4000 delivered, 0 lost.

Bad jitter:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-jitter27 --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 4000 expected, 3957 delivered, 43 lost.

Combined good case:

```bash
rosotacom benchmark capacity --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-3.2mbit-jitter18 --knob size --low 18000 --high 18000 --rate-hz 20 --duration 20 --repeats 10 --max-loss 100 --max-latency-ms 100000
```

Migrated summary: 4000 expected, 4000 delivered, 0 lost, max p95 latency
93.138 ms.

Verification: nightly row `boundary-loss-18kb20hz-bandwidth-cyclone` re-proves
the deterministic bandwidth pair (`finding-3.2mbit` good,
`finding-3.1mbit` bad) via `rosotacom benchmark row`; jitter cases remain
manual with the commands above because public runners cannot install seeded
random netem yet.

## Status

confirmed, 2026-06-29.

## Publication notes

Feeds the network-characterization paper boundary table and the benchmark-suite
paper's reference profile choice. The paper adaptation should present both the
independent boundaries and the combined good case, with the exact seedless
replicate counts.
