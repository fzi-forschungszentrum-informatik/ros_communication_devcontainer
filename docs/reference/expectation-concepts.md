# Expectation concepts

Expectations describe what a pipeline stage should observe under its configured
network condition. See [session configuration](../../session-configuration.md)
for accepted fields and examples.

## Live expectations

`mode: stream` assesses received rate, latency, loss and count when configured.
`mode: latched` requires an observed retained value, not a continuous rate.
`mode: existence` requires the topic to exist in the graph, not to deliver.
Topics are required unless `presence: optional` is explicit.

Per-profile overrides select expectations for a named emulated network profile.
Correctness checks remain meaningful across profiles; throughput and delay
bounds depend on the configured condition.

## Replay-only checks

Replay tests can compare delivered messages with a bag's known offered load.
Live tests cannot infer messages that never reached the instrumented publisher.
See [bag-as-load benchmarks](../benchmark-bag-as-load.md) for replay semantics
and [measurement records](measurement-records.md) for sequence accounting.

## QoS

QoS remains explicit session configuration. Automatic QoS derivation from live
publisher discovery is not an implemented feature of this snapshot.
