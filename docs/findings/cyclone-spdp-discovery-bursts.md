# Cyclone DDS SPDP Discovery Bursts Can Distort Tight-Link Tail Latency

## Claim

Cyclone DDS default `SPDPInterval=30s` discovery traffic shares the OTA link with
payload data and can produce periodic p99/max latency spikes on tight shaped
links. The traffic is real end-to-end DDS behavior, but payload-only benchmark
characterization should either annotate it or deliberately use a longer SPDP
interval.

## Setup

- Host pair / topology: host-deterministic diagnostic classification for
  rosotacom benchmark mode, plus public live re-run material using the packaged
  example project, synthetic `bench_1_1_capacity` session, one
  `a->b:/bench_capacity` stream, uplink shaping only.
- rosotacom SHA: seeded in the public ledger at the issue-173 implementation
  commit; public re-runs record the current checkout SHA in
  `result.json.context`.
- Profile: `finding-spdp-tight` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml),
  a 3.2 Mbit/s tight uplink that makes an 18 KB at 20 Hz stream consume about
  90% of the shaped rate.
- Seed policy: no stochastic netem seed is needed for the diagnostic; the live
  profile is rate-only and seedless.

## Evidence

Evidence grade: existing host unit test plus exact public re-run command.

The host test constructs a 3.2 Mbit/s tight profile, a Cyclone benchmark run, and
an 18 KB at 20 Hz payload. It asserts the diagnostic reports `risk: possible`,
offered/shaped-rate about `0.9`, and a warning containing `SPDPInterval=30s`.

Public live re-run:

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-spdp-tight --size 18000 --rate-hz 20 --duration 25 --repeats 1 --rmw cyclone --no-plot
```

Inspect the resulting `result.json` at
`context.diagnostics.cyclonedds_spdp`. A representative diagnostic contains the
RMW, interval, duration-to-interval ratio, profile rate limit, offered bandwidth,
offered-to-rate utilization, risk classification, and warning text.

Verification: `python -m pytest -q tests/unit/test_cli_benchmark.py::test_probe_spdp_diagnostics_warn_on_tight_cyclone_profile` checks diagnostic classification, not live tail latency. The command above is the separate live reproduction.

## Status

confirmed, 2026-07-08.

## Publication notes

Feeds benchmark-method text and result interpretation. Plots that show tight-link
p99/max latency under Cyclone DDS should state whether default discovery traffic
was included, annotated, or quieted with `--cyclone-spdp-interval`.
