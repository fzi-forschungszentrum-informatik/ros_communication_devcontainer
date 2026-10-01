# Benchmark commands and results

The benchmark CLI runs synthetic or recorded loads under named network
conditions and writes a machine-readable result with its configuration.

The commands include a fixed probe, capacity search, rate ramp, parameter
sweep, recovery measurement and paired comparison. See the command help and
[README](../../README.md) for concrete invocations.

A benchmark result records its measurement window, offered load, profile,
per-topic observations and verdict. Keep those fields with the result:
an aggregate loss percentage without its offered traffic does not specify a
repeatable experiment.

- [Bag-as-load](../benchmark-bag-as-load.md) documents recorded workload replay.
- [Paired comparison](../benchmark-ab.md) documents repeated A/B measurements.
- [Performance bands](../performance-bands.md) documents reference comparisons.
- [Network profiles](network-profiles.md) defines the emulated conditions.
- [Validation](../ci.md) lists the local test commands.

Host tests exercise driver behavior using controlled measurement inputs.
Docker tests exercise the live publisher, generated session and receiving
instrumentation. A host test cannot establish physical-link throughput.
