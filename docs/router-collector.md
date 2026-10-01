# Milesight cellular status collector

[`tools/milesight_cellular_metrics.py`](../tools/milesight_cellular_metrics.py)
reads the Milesight UR75 status API and emits one JSON object for the
[link-trace recorder](link-trace.md). It retains radio measurements and cell
identifiers, while excluding SIM identifiers, login credentials and cookies.

The collector requires Python and OpenSSL. It accepts a local mode-0600 JSON
configuration with these fields:

| Field | Value |
|---|---|
| `base_url` | HTTPS origin of the router's private IPv4 address, port 443 |
| `username`, `password` | Router login, stored only in the local configuration |
| `expected_source_ipv4` | Local address the kernel must select to reach the router |
| `tls_certificate_sha256` | 64 hexadecimal digits of the verified router certificate fingerprint |

Keep the configuration outside the repository. Verify the certificate through
an independent trusted connection before recording its fingerprint. The client
checks it before sending a login and refuses a mismatching local route.
It accommodates the UR75's older TLS implementation; it is not a general HTTPS
client. Response sizes and request times are bounded.

Mount the collector and its configuration into the communication container,
then configure the session, using those container paths:

```yaml
shared:
  use_status_overview: true
  link_trace:
    enabled: true
    interval_s: 1.0
    modem_metrics_command: "python3 /opt/collectors/milesight_cellular_metrics.py --config /run/secrets/router.json"
    modem_metrics_timeout_s: 10.0
```

The timeout permits an initial login as well as a status request. Slow requests
can reduce the achieved sampling rate. The authentication cache is private and
kept in the container's temporary directory.

For offline inspection, `--normalize-response <file.json>` normalizes a saved
API response without contacting the router or reading credentials.

The status API may serve cached radio measurements. Polling it once per second
does not establish a one-second modem refresh rate. `sinr_raw` preserves the
router's reported value without assigning decibel units; older campaign traces
called this field `sinr_db`, which does not establish that the value was in dB.
The collector does not issue AT commands or change router configuration.
