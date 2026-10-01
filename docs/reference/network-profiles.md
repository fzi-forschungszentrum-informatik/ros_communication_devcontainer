# Network profiles

Profiles specify an emulated link independently of session topics. A project
selects its profiles file with the `profiles` entry in `rosotacom.yaml`.

```yaml
profiles:
  example-steady:
    uplink: {delay: 60ms, jitter: 10ms, distribution: normal, loss: 0.5%}
  example-events:
    timeline:
      - {for: 30s, uplink: {delay: 60ms, loss: 0.1%}}
      - {for: 2s, uplink: {delay: 180ms, loss: 3%}}
```

Static profiles support delay, jitter, built-in or custom delay distributions,
loss, Gilbert-Elliott loss, duplication, reordering, rate limits and seeds.
Timeline profiles apply successive conditions and may contain outages.
`src/rosotacom/network_profiles.py` validates the schema and generates
commands; `network_shaper.py` controls arming and teardown.

Shaping applies to each sending peer's outgoing interface. A physical two-host
run needs the correct interface and shaping privileges on both participating
peers. The control connection must remain reachable. Local Docker benchmarks
use their own network namespaces.

The shaper has bounded lifetime and teardown handling. Timeline transitions
preserve the queue rather than deleting it between segments. A rate cap may
queue or eventually drop packets; it is not equivalent to independent loss.

The [eleven fitted field windows](../../field-profiles/2026-08/README.md)
supply steady and event profiles with effective fragment-model loss, not
measured datagram-loss probabilities. Their passive
recordings do not identify spare capacity, so these profiles set no rate cap.
Do not add a loss-correlation parameter to reproduce a burst frequency:
use the event timeline. The [replay](../../field-profiles/2026-08/replay/replay.py)
checks fragment-to-frame loss on the published frame plan.

See [link trace](../link-trace.md), [benchmark commands](benchmark-commands.md)
and [testing](../testing.md). Emulation checks a declared condition; it does not
establish prediction accuracy for an unmeasured cellular link.
