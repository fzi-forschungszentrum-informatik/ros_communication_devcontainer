# Link Trace Recorder

The link trace recorder writes an append-only JSONL stream next to the live
status overview artifacts:

```text
session-instances/.../logs/<peer>/status/link_trace.jsonl
```

Enable it in a session definition:

```yaml
shared:
  use_status_overview: true
  link_trace:
    enabled: true
    interval_s: 1.0
    modem_metrics_command: "cat /run/modem-metrics.json"
    modem_metrics_timeout_s: 2.0
```

Or enable it for a single run:

```bash
rosotacom start 13_link_latency --identity a --link-trace
rosotacom smoke 13_link_latency --link-trace --link-trace-interval 0.5
rosotacom ota-smoke 13_link_latency --link-trace
```

`--link-trace-interval` and `--link-trace-modem-command` also enable the
recorder and force `shared.use_status_overview: true` for the generated instance.

`interval_s` is the period between rows, and the status-overview node is ticked
at the same period when the trace is enabled. Those two being equal is what
makes the trace continuous: each row's `passive_counter_delta` covers one tick,
so consecutive rows meet rather than leaving traffic between them. The recorder
therefore schedules against an advancing deadline with a small tolerance instead
of a stopwatch since the last row — without that, a tick landing a hair early is
rejected and waits a whole further period, which halves the trace and leaves
every second interval described by no row at all (measured on a 5 h drive:
1.984 s between rows carrying 1.002 s windows).

## Row Schema

Each line is one JSON object:

```json
{
  "schema_version": 1,
  "kind": "link_trace",
  "generated_at": "2026-07-07T10:00:00.000",
  "monotonic_s": 123.456,
  "peer": "a",
  "remote": "b",
  "passive_counter_delta": {
    "available": true,
    "provenance": "proc_net_dev_counter_delta",
    "interface": "eth0",
    "window_s": 1.0,
    "observed_not_available_bandwidth": true,
    "rx": {"bytes_delta": 1234, "packets_delta": 10, "observed_kbps": 9.872},
    "tx": {"bytes_delta": 5678, "packets_delta": 20, "observed_kbps": 45.424}
  },
  "peer_probe": {
    "available": true,
    "provenance": "echo_heartbeat_status_snapshot",
    "rtt_ms": 12.5,
    "peer_offset_ms": -1.2,
    "loss_pct": 0.0,
    "assumption": "symmetric_path"
  },
  "modem": {
    "available": true,
    "provenance": "modem_metrics_command",
    "command": "cat /run/modem-metrics.json",
    "metrics": {"rsrp_dbm": -88}
  }
}
```

`passive_counter_delta` uses `/proc/net/dev` counters for the resolved OTA
interface. Its `observed_kbps` fields are observed traffic during the sampling
window. They are not available-bandwidth estimates and cannot reveal unused link
capacity.

`peer_probe` reuses the status overview's echo heartbeat snapshot. RTT and clock
offset require `shared.use_heartbeat: true`; loss is copied from the first status
stage that has sequence-loss provenance.

`modem_metrics_command` is optional. It runs inside the communication container
at each sample interval, must print a JSON object on stdout, and is recorded under
`modem.metrics`. Non-zero exits, timeouts, invalid JSON, or non-object JSON are
captured as unavailable modem blocks instead of failing the session.
