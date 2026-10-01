# A/B tuning experiments (`rosotacom benchmark ab`)

`rosotacom` is a tuning suite as much as a measurement one: the *fix* step of the
closed-loop workflow needs a first-class answer to **"does candidate config B
beat baseline config A on the same load and profile — what improved, what
regressed, what is unchanged?"** `benchmark ab` is that verdict.

It is the sibling of the regression gate (benchmark commands reference / performance bands reference). The gate asks
"is this run within a committed envelope?"; A/B asks "is B better or worse than
A, right now, on the same load?". Both are two-sided and both speak the same
**IMPROVED / WITHIN / REGRESSED** language — an A/B cell is exactly a band
verdict on the ephemeral `[baseline ± tolerance]` envelope.

## What varies, what is held constant

| Held constant across every run | Varied between configs |
|---|---|
| the load — the synthetic `sized_publisher` stream on `a_to_b` (`--size`/`--size-pattern`, `--rate-hz`, `--streams`) | the **session config**: the OTA pipeline knobs (QoS reliability/depth/lifespan, compression, ffmpeg gop/bitrate/crf, …) |
| the network profile and its seed policy (`--profile`) | |
| the RMW and any run-wide QoS override (`--rmw`, `--qos-*`) | |

Each `--baseline` and `--candidate` is a **whole session config** — a directory
or a `session-definition.yaml`, no patch format. Every config must carry the same
`a_to_b` load topic(s) (that is what the synthetic publisher drives); they should
differ *only* in the pipeline knob under test. The run records the YAML diff of
each candidate against the baseline (`configs/<label>.diff`) so what actually
changed is explicit and reviewable — and so the "same load" assumption is
auditable rather than assumed.

**Compare knobs that keep the topic name.** The verdict matches topics by name,
so the clean A/B knobs are the ones that leave the delivered topic identical
across configs: QoS (reliability/depth/lifespan), compression, ffmpeg settings.
`throttle_hz` and `drop` are implemented as a `topic_tools` republish to a
rate-suffixed topic (`/topic/max20hz`), so two throttle values produce two
*different* topic names — not a like-for-like comparison. To compare rate caps,
give both configs the same fixed suffix and vary elsewhere, or measure the rate
knob with `benchmark sensitivity`/`capacity` instead.

## Running it

```bash
rosotacom benchmark ab \
  --profile cellular-4g-degraded \
  --baseline configs/gop30 \
  --candidate gop15=configs/gop15 \
  --candidate gop15-crf28=configs/gop15_crf28 \
  --size 18000 --rate-hz 20 --duration 20 --repeats 5
```

`--candidate` is repeatable; the first (implicit) config is always the
`baseline`. Useful flags:

- `--repeats N` — runs per config (default 3). Higher N narrows the spread and
  lets smaller effects clear the tolerance band.
- `--metrics` — comma list of watched metrics (default
  `completeness_pct,loss_pct,latency_p95_ms,jitter_p95_ms`). Full set:
  `completeness_pct`, `loss_pct`, `latency_p50_ms`, `latency_p95_ms`,
  `jitter_p50_ms`, `jitter_p95_ms`.
- `--rel-tolerance` / `--abs-tolerance` — the unchanged band half-width around the
  baseline median is `max(abs, rel · |baseline|)`. Defaults: `rel = 0.10`; a
  small absolute floor on the latency/jitter tails (2 ms on p95) so sub-floor
  noise on a tiny baseline never reads as a change. Loss and completeness are
  exact percentages and get no floor.
- `--topic` — restrict the verdict to one topic.
- `--fail-on-regression` — exit non-zero when any candidate regressed (default:
  the verdict lives in `result.json`, exit 0), so a fix playbook can gate on it.

## Interleaving: guarding the verdict against drift

Runs are **interleaved**, not "all of A then all of B": each repeat runs every
config once, and the starting config rotates each repeat (`baseline, cand,
cand, baseline, …`). Interleaving spreads any slow host/thermal drift across
every config instead of loading it onto whichever ran last; the rotation removes
the residual first-slot warm-up bias. The schedule is deterministic, so a verdict
is reproducible from the same runs. The exact execution order is recorded in
`configuration.schedule`.

## The verdict

Per candidate, per topic, per metric, the candidate's **median** across repeats
is compared to the baseline's median (a median so one unlucky repeat cannot flip
a cell). The cell is `IMPROVED`, `WITHIN`, or `REGRESSED` by the metric's
better-direction and the tolerance band. A candidate **passes** iff nothing
regressed beyond tolerance and it did not drop a topic the baseline delivered;
the overall experiment passes iff every candidate passes.

Both spreads (min/median/max) travel with each cell, plus a `separated` flag —
`yes` when the baseline and candidate spreads do **not** overlap, i.e. the effect
is distinguishable at this repeat count.

## Statistical power — read small N honestly

A/B here is a **directional screen**, not a significance test. With a handful of
repeats you can only claim **large** effects: a candidate whose median moves by
less than the run-to-run spread is not really separated from the baseline
(`separated: no`), no matter which side of the tolerance band its median lands
on. Two practical rules:

- **Prefer the boundary regime.** Effects are sharpest where the system is near
  a cliff — a rate-limited profile at the capacity breakpoint, or one of the
  documented boundary pairs (performance bands reference) — so a real knob change produces a big,
  clearly-separated delta instead of a wiggle inside the noise. On a fat,
  unstressed link most knobs look "unchanged" simply because nothing is
  constrained.
- **Raise `--repeats` before you trust a marginal call.** A cell that is
  `IMPROVED`/`REGRESSED` but `separated: no` is a "maybe"; more repeats either
  separate it or reveal it as noise.

Latency/jitter tails are host-timing-dominated and noisier than the
bottleneck-dominated `loss_pct`/`completeness_pct` (which are counted off exact
sequence numbers) — weight the completeness/loss verdict accordingly, exactly as
the regression gate does.

## Outputs (self-describing — the publication byproduct)

Under the run directory:

- `result.json` — `genre: "ab"`: the full per-cell verdict (`result`), the
  per-run measurements (`measurements.runs`), the resolved
  `configuration` (baseline, configs, profile, load, tolerances, schedule), the
  rosotacom `sha` and runner fingerprint, and the overall `verdict`.
- `ab.md` — a markdown table per candidate (topic × metric, both spreads, Δ,
  `sep?`, verdict) — a paper-grade figure with no rework.
- `configs/<label>/…` + `configs/<label>.diff` — each materialized config and its
  YAML diff against the baseline.
- `ab.jsonl` — one row per run.

## Non-goals

- **No automatic parameter search / optimization.** The operator picks
  the candidate configs; `benchmark ab` only renders the verdict between them.
- **No live-drive A/B.** Replay / emulated-profile conditions only, so the
  comparison is reproducible and the only thing that changed is the config.

## Owner smoke

Two configs that differ only in OTA QoS **reliability**, on the packaged example
project, under an emulated 20% packet loss (QoS keeps the delivered topic name
identical across configs, which throttle/drop do not — they republish to a
Hz/N-renamed topic — so QoS is the clean knob for a like-for-like A/B):

```bash
rosotacom examples create /tmp/ab-demo && cd /tmp/ab-demo
printf 'profiles:\n  ab-loss:\n    uplink: { rate: 8mbit, loss: 20%% }\n    downlink: { rate: 8mbit }\n' > profiles.yaml
for rel in best_effort reliable; do d="cfg_$rel"; mkdir -p "$d"; cat > "$d/session-definition.yaml" <<YAML
peers: { a: {}, b: {} }
peer_settings: { a: { domain_id: 50 }, b: { domain_id: 51 } }
shared:
  use_status_overview: true
  ota_domain_id: 52
  rmw: cyclone
  qos: { for_role: { ota_pub: { depth: 1, reliability: $rel }, ota_sub: { depth: 1, reliability: $rel } } }
topics:
  a_to_b:
    - topic: "/bench_capacity"
      type: "com_msgs/msg/SizedPayload"
      processing: { use_ota_wrapper: true }
YAML
done
rosotacom benchmark ab --profile ab-loss --baseline cfg_best_effort --candidate reliable=cfg_reliable \
  --size 18000 --rate-hz 20 --duration 15 --repeats 1
```

Expected: both configs are measured on `/bench_capacity`, and the printed `ab.md`
table classifies the `reliable` candidate's `completeness_pct` as **IMPROVED** (it
recovers the dropped samples best_effort loses) or **WITHIN** — never REGRESSED.
The verdict JSON is written to `result.json`.

See also: [benchmark commands reference](reference/benchmark-commands.md) (benchmark genres),
[performance bands reference](reference/performance-bands.md) (the two-sided band / verdict language),
and [docs/performance-bands.md](performance-bands.md) (the committed-band gate
this shares its verdict vocabulary with).
