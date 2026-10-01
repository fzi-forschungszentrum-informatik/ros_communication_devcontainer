# Performance bands and findings

Benchmarks compare observations with committed performance bands.
A band records the metric, expected range and calibration provenance.
A result outside either side needs review: an apparent improvement may also
indicate that a stage stopped measuring.

The `benchmark ratchet` command proposes updated bands from measured runs.
Changing a bound requires a reason and retained evidence. A runner, dependency
or ROS distribution change can require recalibration rather than indicating
a transport regression.

See [performance bands](../performance-bands.md) for the schema, command
examples and verdicts, and [paired comparison](../benchmark-ab.md) for controlled
before/after measurements.

The [findings index](../findings/README.md) records measured effects with their
setup, evidence, verification command and scope. A synthetic reproduction
establishes the mechanism under its stated conditions; it does not establish
a field rate.

This snapshot ships local validation commands. It does not configure repository
branch protection, a merge queue or a remote promotion pipeline.
