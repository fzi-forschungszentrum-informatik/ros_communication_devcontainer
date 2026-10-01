# Lighter Alternating Messages Can Lose More Than Steady Messages

## Claim

Under a tight emulated profile, replacing every second 18 KB message with a
0 KB message can increase loss even though the average payload bandwidth is
lower.

## Setup

- Host pair / topology: rosotacom local benchmark mode using the packaged example
  project, synthetic `bench_1_1_capacity` session, one `a->b:/bench_capacity`
  stream, uplink shaping only.
- rosotacom SHA: original migrated summary was measured at `4b11973`; public
  re-runs record the current checkout SHA in `result.json.context`.
- Profile: `finding-tight-irregular` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml),
  uplink `rate: 3.2mbit`, `delay: 50ms`, `jitter: 18ms`, `distribution: normal`.
- Seed policy: seedless single-run comparison (`n=1` for the alternating run and
  each steady comparison run); no `seed` field is recorded in the profile.

## Evidence

Evidence grade: migrated benchmark summaries plus exact public re-run commands.
This is a reproduced counterexample, but it still needs seedless `n >= 10`
confirmation before being used as a generality claim.

Alternating lighter-message run:

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-tight-irregular --size-pattern 1x18KB+1x0KB --rate-hz 20 --duration 20 --repeats 1 --no-plot
```

Migrated summary: 390 expected, 347 delivered, 43 lost, 11.026% loss, p50
latency 73.758 ms, p50 transit jitter 57.906 ms.

Steady 18 KB comparison:

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-tight-irregular --size-pattern 1x18KB+1x18KB --rate-hz 20 --duration 20 --repeats 1 --no-plot
```

Migrated summaries: one steady run had 405 expected, 404 delivered, 1 lost,
0.247% loss; a second steady run had 405 expected, 405 delivered, 0 lost. The
alternating run used mean payload 9000 bytes and offered payload bandwidth
1.44 Mbit/s, while the steady comparison used 18000 bytes at 20 Hz, about
2.88 Mbit/s.

Verification: manual: run the two commands above from a source checkout with
Docker and `tc` privileges; the public registry exists, but this finding still
uses a random-jitter profile that cannot become a deterministic public gate
until the runner class can seed netem.

## Status

confirmed, 2026-07-02.

## Publication notes

Feeds the benchmark-suite paper as the motivating anomaly for patterned
synthetic loads. The paper adaptation should show the lower average bandwidth of
`1x18KB+1x0KB` next to the higher loss rate, and should avoid calling this
statistically general until replicated.
