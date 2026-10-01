# Performance bands & the ratchet

The deterministic slice of the benchmark suite gates against **committed
two-sided bands** ([performance bands reference](reference/performance-bands.md)). This page is the
day-to-day workflow; the RFC records the design and its rejected alternatives.

## The band store: `budgets.jsonl` (schema v2)

One JSON line per `(row, profile, metric)`:

```json
{"schema": 2, "row": "capacity-size", "profile": "rate-limited-capacity-ci",
 "metric": "capacity_size", "lo": 4900.0, "hi": 5100.0, "better": "higher",
 "provenance": {"fingerprint": "github-hosted-linux-x86_64", "window_s": 60.0,
                "repeats": 5, "sigma": 33.0, "floor": 100.0, "k": 3.0,
                "source_sha": "c44d4c5", "ratcheted_at": "2026-07-08T12:00:00",
                "note": "initial calibration"}}
```

- `[lo, hi]` is the accepted envelope; `better` says which way out of it is an
  improvement. The interval is closed: values on the edge are `WITHIN`.
- `provenance` is why the band is exactly this wide: the runner class that
  calibrated it, the measurement window, the K repeats behind `sigma`, and the
  half-width formula `max(k·sigma, floor)`.
- **Bands are never hand-edited.** They change only through
  `rosotacom benchmark ratchet`, so every move is a reviewed diff with a cause
  note. The file is written in a stable order — a ratchet is a one-line diff.

## Gating a run: `benchmark compare`

```bash
rosotacom benchmark compare <run-dir-or-result.json ...> --budgets budgets.jsonl
```

Per banded metric the verdict is `WITHIN`, `REGRESSED` (out on the worse side),
or `IMPROVED` (out on the better side). Exit codes:

| exit | meaning |
|---|---|
| 0 | all `WITHIN` (or `--monitor`, which reports without blocking) |
| 1 | `REGRESSED`, or a refusal (fingerprint mismatch, missing band/metric) |
| 2 | `IMPROVED` beyond the band — red too, but the fix is the printed ratchet command, not a revert |

`IMPROVED` is red so improvements get **banked**: the failure text prints the
exact `benchmark ratchet` invocation; run it and commit the tightened
`budgets.jsonl` in the same change. Without this, a later regression back to
the old level would sit invisibly inside a slack band.

`compare` **refuses** to judge a run against a band calibrated on a different
runner class (the `fingerprint` in every `result.json` vs the band's
provenance): a runner change must force a visible recalibration, never a
silent shift. Multiple result files are treated as repeats of one row: the
per-metric median is gated.

## Moving a band: `benchmark ratchet`

```bash
# bank an improvement (keeps the calibrated width, re-centers on the run):
rosotacom benchmark ratchet <run ...> --budgets budgets.jsonl --note "gop default changed"

# calibrate a new band, or deliberately move/widen one, from K fresh repeats:
rosotacom benchmark ratchet <run1> <run2> <run3> --budgets budgets.jsonl \
  --recalibrate --note "new runner class"
```

- A plain ratchet only turns one way: the worse edge may move toward better,
  never toward worse, and the calibrated width and provenance are preserved.
  Anything else — first bands, widening, accepting a worse level as a
  trade-off, a new runner class — is `--recalibrate`, which recomputes the
  half-width as `max(k·σ, floor)` from the given runs (`--k`, `--floor`,
  `--floor-frac`; defaults k=3, floor 2 % of the center).
- Every move takes a one-line `--note`; a band diff without an explanation is
  a review red flag.

Banded metrics today come from `probe` runs (`loss_pct`, plus
`latency_p50_ms`/`latency_p95_ms` — monitor-only on shared runners),
`replay` runs (the probe set plus `delivered_count` and `delivered_hz`),
`capacity` runs (`capacity_<knob>`) and `recovery` runs (`t_recover_s`,
`t_steady_s`, `recovery_burst`, `lost_during_outage_total`).

## The benched set: which rows gate

The curated registry
[`src/rosotacom/resources/benched-set.yaml`](../src/rosotacom/resources/benched-set.yaml)
is the whitelist of gated rows — one deliberate (RMW × profile × load × metric
set) each, with an operator-visible reason, committed seeds, and per-metric
width floors. The CI lanes *read* it; adding a row is a registry edit plus a
calibration, never a workflow edit (contract-tested in
`tests/contract/test_benched_set_registry.py`).

```bash
rosotacom benchmark rows                      # list the benched set
rosotacom benchmark row <id> \
  --rosotacom-config <project>/rosotacom.yaml # run + band-assert one row
```

`benchmark row` runs the row's genre with its committed parameters, gates the
result against `budgets.jsonl`, and writes a machine-readable
`verdict.json` (`WITHIN` / `REGRESSED` / `IMPROVED` / `EXPECT_FAILED` /
`REFUSED`, metrics, bands, sha, runner fingerprint) for downstream tooling.
`--monitor` reports without blocking; `--no-compare` is for calibration
repeats.

### Replay rows: gated by the bag, not only by a band

A `genre: replay` row's load is not invented in the registry — every parameter
(cadence, mean payload, interval jitter, window) is a measurement of the
recording named in its `replay` block, and the loader refuses a row whose load
has drifted away from it. The bags stay in the operator harness; what is public
is their measured shape and the whole-bag `expect` fragment generated from them
(`rosotacom expect from-bag`), which `benchmark row` asserts on every gating
run: delivered count against `min_count`, delivered/expected against
`completeness_min_ratio`, and delivered rate against `vs_bag_ratio × native_hz`.
A run that misses any of them is `EXPECT_FAILED` and red, with the per-threshold
reasons in `verdict.json` under `replay_expect`.

That assertion is bag-derived rather than runner-derived, which is why replay is
the one genre allowed to land a row with no banded metrics at all: it gates from
the day it lands, and the bands follow from a calibration run on the runner
class. Every other genre still needs at least one banded metric.

Lanes (performance bands reference):

- **merge gate** — the `merge-gate` rows run inside the benchmark-capacity E2E
  (`tests/e2e/test_benchmark_capacity.py::test_merge_gate_row_is_band_asserted`,
  lane `just test-e2e-slice benchmark-capacity`): minutes-scale, default RMW,
  one rate-limited profile.
- **nightly** — `.github/workflows/benchmark-gate.yml` runs every `nightly`
  row on schedule (and on dispatch), uploads per-row verdicts plus the
  aggregated `benchmark-gate-summary` artifact (`benchmark gate-summary`), and
  is red on any setup failure, `REGRESSED`, or unbanked `IMPROVED`.

## Calibration: how bands are minted

Bands are calibrated on the runner class that executes them, via
`.github/workflows/benchmark-calibrate.yml` (manual dispatch): K independent
repeats per row — one job per (row, repeat), so σ includes cross-instance
variance — then

```bash
rosotacom benchmark calibrate <row-id> <K run dirs...> --budgets budgets.jsonl \
  --report calibration/<row-id>.json
```

mints the bands for exactly the row's gated metrics (each with its committed
registry floor) and writes the spread evidence — monitor metrics included, so
the tighten-or-monitor decision is made from numbers, not taste. The workflow
uploads `budgets.jsonl` + reports as the `calibrated-bands` artifact; review
the widths and commit both. The committed calibration evidence lives in
[`calibration/`](../calibration/README.md).
