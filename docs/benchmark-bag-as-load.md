# Bag replay as the benchmark load (`--target` / `--bag`)

`rosotacom benchmark requirements` and `benchmark loss-boundaries` search a
network profile for a load. Normally that load is the synthetic `sized_publisher`
stream (a size/rate pattern on `a_to_b`). This mode makes the load a **real
replay** instead — a whole session/scenario — so the search answers the closed-loop
vision question:

> *Which network conditions could carry **this bag** loss-free (or ≥95% complete,
> latency-bounded)?*

instead of the synthetic-stream version of it. The search drivers are unchanged
(same geometric/linear boundary search, same `result.json`); what changes is
**what runs at each probe point** and **how the verdict is judged**.

## What varies, what is held constant

| Held constant across every probe | Varied between probes |
|---|---|
| the load — the replay `--target` session's own contract publishers (both directions), bag-shaped payloads | the **network profile**: the generated candidate (bandwidth / jitter / latency / loss) the search is testing |
| the completeness ground truth (`--bag`) and the oracle thresholds | |
| the RMW and any run-wide QoS override (`--rmw`, `--qos-*`) | |

A probe point = **one full replay run** under the candidate profile. That is the
whole cost story below.

## The load source (`--target`)

`--target <name> --target-type <session|scenario>` selects the replay, mirroring
`ota-benchmark`'s target selection. The target's carried topics (across all
directions) become the **required contract**: the set the run must deliver.

- **Local (Docker) replay** — the default for `benchmark …` with a *session*
  target. rosotacom starts the target session's two peers on an isolated Docker
  network and drives every crossed topic with the same synthetic-but-typed
  publishers the local smoke uses (e.g. a `com_msgs/msg/CompressedData` payload
  for `/topic5`), shapes the link with the candidate profile, and collects
  transit records. This is what CI exercises. Local replay needs a two-peer
  (`a`/`b`) session target.
- **OTA replay** — `ota-benchmark … --target …` runs the real target
  session/scenario across deployment peers exactly as `ota-smoke` does, and feeds
  the same per-topic oracle. Use this for scenario targets, real payloads, and
  real links.

## The oracle: the whole contract, per topic

Each replay run's transit summary is judged **per topic** and aggregated to a run
verdict (`rosotacom.benchmark.evaluate_bag_run`, host-tested):

- **loss** — network loss ≤ the driver's loss bound (`requirements --max-loss`;
  `loss-boundaries` is zero-loss);
- **latency** — the topic's `p95` OTA-hop ≤ `--max-latency-ms`;
- **completeness** — when `--bag` supplies a rosbag2 `metadata.yaml`,
  `delivered / bag-count ≥ --oracle-min-completeness` (default `0.95`). Without a
  bag the completeness gate is skipped and only loss + latency apply, so the mode
  still works on a bench session with no bag file.

A run **passes iff every required topic passes**; otherwise the run is a fail and
the offending topics are listed. A required topic that never arrives counts as
fully lost. For `loss-boundaries`, an *incomplete* topic makes the run "lossy" —
so an under-delivered bag is a bad point, exactly as a dropped message would be.

Per-topic verdicts (`passes`, `completeness`, `loss_pct`, `latency_p95_ms`,
`reason`) are attached to every sample, and the union of failing topics is
attached to each row.

## Results

`result.json` keeps the normal shape and gains a `result.replay` block recording
the replay identity so a figure is self-describing:

```json
"replay": {
  "target": {"name": "15_remote_assist_anonymized_costmap", "type": "session"},
  "bag": {"metadata": "…/metadata.yaml", "topics": ["/topic5"]},
  "required_topics": ["/topic5"],
  "oracle_min_completeness": 0.95
}
```

Rows carry `failing_topics`; each sample carries `per_topic` verdicts. The
per-topic loss/latency/jitter table (`measurements.points[].samples[].topics`) is
the full contract, not a single stream.

## Cost guidance

A synthetic probe costs `duration` seconds; a **replay probe costs a full bag
loop** (the `remote_assist` bag is ~100 s), times `--probe-repeats`, times the
number of candidates the search visits. So:

- Keep the search budget small: few `--probe-repeats`, a coarse
  `--bandwidth-step`, one axis at a time (`--axes bandwidth`).
- The existing prior-knowledge seeding still applies — start near the
  offered-bandwidth estimate (`--bandwidth-high`/`--bandwidth-low`) and let the
  target bound exclude conditions that cannot help.
- Seed policy is unchanged (`--netem-seed` / `--netem-seeds`); on shared CI
  runners prefer the bandwidth axis (rate-limited, seed-free) over jitter.

## Owner smoke

Run the costmap replay (example 15) through `loss-boundaries` with a one-pair
budget, on existing packaged material — no new drive needed:

```bash
rosotacom benchmark loss-boundaries \
  --target 15_remote_assist_anonymized_costmap --target-type session \
  --axes bandwidth --bandwidth-low 0.5mbit --bandwidth-high 2mbit --bandwidth-step 0.5mbit \
  --rate-hz 10 --min-duration 20 --min-messages 1 \
  --probe-repeats 1 --good-clean-count 1 --bad-lossy-count 1 --max-latency-ms 1000
```

Expected: the run completes and prints `Benchmark result saved to …/result.json`;
that file has `genre: "loss-boundaries"`, a `result.replay.target.name` of
`15_remote_assist_anonymized_costmap`, and per-topic verdicts for `/topic5` on
every probe row.

## See also

- [Benchmark genres & CI (benchmark commands reference)](reference/benchmark-commands.md) — the
  `requirements` / `loss-boundaries` drivers and their oracle.
- [A/B tuning experiments](benchmark-ab.md) — the *fix*-step sibling (config A vs
  B on the same load).
- [Performance bands & the ratchet](performance-bands.md) — how gated rows assert
  against committed two-sided bands (performance bands reference).
