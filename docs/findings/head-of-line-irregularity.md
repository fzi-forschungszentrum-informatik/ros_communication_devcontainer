# Head-Of-Line Blocking Makes Irregularity The Dominant Failure Mode

## Claim

Irregular message sizes can cause effective loss through head-of-line blocking:
a heavy message delays the following light message, order is preserved, and the
delayed light message can be overwritten by the next update.

## Setup

- Host pair / topology: rosotacom local benchmark mode using the packaged example
  project, synthetic `bench_1_1_capacity` session, one `a->b:/bench_capacity`
  stream, uplink shaping only.
- rosotacom SHA: original migrated reproduction summary was measured at
  `4b11973`; public re-runs record the current checkout SHA in
  `result.json.context`.
- Profile: `finding-tight-irregular` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml),
  a tight emulated uplink with rate, delay, and jitter.
- Seed policy: seedless single-run reproduction (`n=1`) and still needs seedless
  `n >= 10` confirmation for a general claim.

## Evidence

Evidence grade: migrated benchmark summaries plus exact public re-run commands.

Current minimal reproduction:

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-tight-irregular --size-pattern 1x18KB+1x0KB --rate-hz 20 --duration 20 --repeats 1 --no-plot
```

Migrated summary: 390 expected, 347 delivered, 43 lost, 11.026% loss, p50
transit jitter 57.906 ms.

Steady comparison:

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile finding-tight-irregular --size-pattern 1x18KB+1x18KB --rate-hz 20 --duration 20 --repeats 1 --no-plot
```

Migrated summary: 405 expected, 405 delivered, 0 lost.

The mechanism is visible only per message: the stream carrying alternating
heavy/light samples can lose more than the steady stream despite lower mean
payload bandwidth.

Field recurrence (2026-08-17 cellular drive): the DELAY half of the mechanism
recurred and was measured per message — a small message with more than 30 KB
in flight ahead of it paid +17.4 ms p50 / +67 ms p90, a small message sent
within 40 ms after a video keyframe paid p50 42.1 vs 28.4 ms, and 18.4% of
cross-topic pairs sent within 2 ms flipped order after a >20 KB first message.
The LOSS half (delayed-then-overwritten) did not trigger there: it needs a
queue tight against the offered load, and that link had ~14 Mbit/s
serialization headroom at 2.5 Mbit/s offered. Both halves stay reproducible
with the tight profile above.

Verification: manual: run the two commands above from a source checkout with
Docker and `tc` privileges; automation waits for the performance bands reference regression matrix
row for patterned loads because the public row registry does not exist yet.

## Status

confirmed, 2026-08-18.

## Publication notes

Feeds both the benchmark-suite paper and the recovery/forensics paper as the
mechanism behind patterned synthetic loads. The paper adaptation should add a
per-message timeline plot before publication, because averages alone hide this
effect.
