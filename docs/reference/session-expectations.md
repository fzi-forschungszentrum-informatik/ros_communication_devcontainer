# Session expectations

A session definition describes peers, topics, processing stages and expectations.
The generator builds the per-peer configuration and a pipeline specification.
The status evaluator checks observations against that specification.

Expectations attach to a named pipeline stage. A passing publisher or container
health check does not establish that the receiving application obtained data.
Stream checks use received rate, loss and latency where the corresponding
instrumentation is enabled.

- [Session configuration](../../session-configuration.md) defines the schema.
- [Expectation concepts](expectation-concepts.md) explains stream and latched data.
- [Measurement records](measurement-records.md) describes the observations.
- [Testing](../testing.md) distinguishes host contracts from Docker and physical-link tests.

The generated-topic contract test checks that configured node inputs have a
producer in the peer configuration or pipeline specification. This detects
topic-name mismatches without running ROS. Runtime delivery still requires an
end-to-end test.
