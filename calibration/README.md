# Calibration evidence

Per-row calibration reports minted by the **Benchmark Calibrate** workflow
(`.github/workflows/benchmark-calibrate.yml`) together with `budgets.jsonl` —
see [docs/performance-bands.md](../docs/performance-bands.md).

One JSON file per benched row (`<row-id>.json`): the K per-repeat metric
values, their median and run-to-run σ on the calibrated runner class, and the
width parameters used. Monitor metrics carry their spread here too, so the
decision to keep them monitor-only (performance bands reference metric policy) is reviewable
against committed numbers.

These files are evidence, not configuration: the gate reads only
`budgets.jsonl` and the registry. Recalibrating a row replaces its report in
the same change that moves its bands.
