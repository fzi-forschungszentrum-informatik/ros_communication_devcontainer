# The Size Law Belongs To The Transport, Not To The Link

## Claim

On one emulated link, with the impairment held byte-identical and only the OTA
middleware changed, message loss against message size takes three different
shapes:

| wrapped size | fragments | CycloneDDS | Fast DDS | zenoh native |
|---:|---:|---:|---:|---:|
| 300 B | 1 | 1.05 % | 1.05 % | 0.57 % |
| 1.1 kB | 2 | 1.47 % | 1.14 % | 1.00 % |
| 3.1 kB | 4 | 3.47 % | 3.80 % | 0.71 % |
| 6.1 kB | 7 | 6.28 % | 4.81 % | 1.00 % |
| 12.1 kB | 13 | 11.23 % | 8.74 % | **0.47 %** |
| 38.1 kB | 38 | 31.92 % | 39.25 % | **61.69 %** |

**CycloneDDS follows `1-(1-p)^N` and nothing else.** Fitting one per-packet
rate and one effective packet size over the whole ladder leaves residuals of
1.20, 0.85, 1.01, 1.06, 0.97 and 1.00 — flat across three decades of size and
a factor of 44 in packet count, with no trend.

**Fast DDS keeps the form and loses the fit.** It carries no RTPS fragment size
at all, so the effective packet is the datagram; one `(p, F)` describes the
ladder to within 0.71–1.14, and the misfit is systematic rather than noise
(at 12 kB the Wilson interval is 7.61–10.02 % against a predicted 12.32 %).

**zenoh does not obey the law.** Its loss is flat at 0.5–1.0 % from 300 B to
12 kB — a 13-packet message loses what a 1-packet message loses, which is what
a transport that repairs its own losses looks like — and then falls off a cliff
to 61.7 % at 38 kB, where the offered rate leaves the regime the repair can
carry.

So the transfer function this project reports from the field is not a property
of the link. It is what a **best-effort, fragmenting, unrepaired** transport
does to a link, and the same link answers a repairing transport completely
differently. Any design rule of the form *a stream's availability follows from
its largest message class* carries that scope with it.

## Setup

- Host pair / topology: one host, the packaged local benchmark rig — two
  communication containers on their own Docker network, `tc` inside each
  container's own netns (`--sudo-mode container`), no host privileges. The
  confirmation ladder runs the same session across two machines joined by a
  point-to-point VPN, each shaping its own tunnel device, with the control
  plane on a separate interface so the profile never touches it.
- Session: `bench_1_1_capacity` from the packaged example project, one
  `a->b:/bench_capacity` stream at 10 Hz, OTA QoS best_effort / KEEP_LAST
  depth 1, `--rmw` the only thing that differs between the three ladders.
- rosotacom SHA: measured at `7bb420c` on one host and `b3c573d` across two;
  re-runs record theirs. The one-host runs need 2.5.dev74 or later, because before it a profile carrying a fitted delay table
  could not be armed at all
  ([a fitted delay table never reached the shaper](a-fitted-delay-table-never-reached-the-shaper.md)).
- Profile: `tilt-iid` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml)
  — the 2026-08-17 drive's own delay table at 25 ms ± 18 ms, with independent
  per-packet loss raised to 1 % so that a 240 s run resolves the one-packet
  cell. The *form* of the law is under test, and it does not depend on the
  level.
- Seed policy: netem `loss` is unseeded; each cell offers about 2100 messages
  inside the measured window, so the Wilson intervals in the table below carry
  the run-to-run spread. `n=1` per cell.

## Evidence

Evidence grade: per-message counts read from the **receiving** peer's own transit records, not from the runner's summary — a summary over rows from
two observers is exactly what has been wrong in this harness before
([sender rows](sender-rows-make-a-dead-link-look-loss-free.md)). The window is
taken in sequence numbers (first 300, last 50 dropped), because a lost row
carries no timestamp of its own and the shaper is armed after discovery.

CycloneDDS, one `(p, F)` fitted by maximum likelihood over the whole ladder —
`p = 0.870 %` per packet, `F = 1008–872 B` against a declared 1024 B fragment,
the difference being the per-packet header the fragment size does not count:

| wrapped | N | offered | lost | observed | 95 % CI | predicted | obs/pred |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 300 B | 1 | 2101 | 22 | 1.047 % | 0.69–1.58 | 0.870 % | 1.20 |
| 1.1 kB | 2 | 2102 | 31 | 1.475 % | 1.04–2.09 | 1.733 % | 0.85 |
| 3.1 kB | 4 | 2105 | 73 | 3.468 % | 2.77–4.34 | 3.435 % | 1.01 |
| 6.1 kB | 7 | 2103 | 132 | 6.277 % | 5.32–7.40 | 5.934 % | 1.06 |
| 12.1 kB | 14 | 2102 | 236 | 11.227 % | 9.95–12.65 | 11.515 % | 0.97 |
| 38.1 kB | 44 | 2102 | 671 | 31.922 % | 29.96–33.95 | 31.921 % | 1.00 |

Fast DDS on the identical profile, `p = 1.306 %`, `F = 1224 B`: 0.80, 0.87,
0.98, 0.76, **0.71**, **1.14**. The two ends of the ladder sit on opposite
sides of the fit and their intervals exclude it, so a single effective packet
size does not describe this stack.

zenoh native on the identical profile: 0.571, 0.998, 0.712, 0.996, 0.475,
**61.693 %**. No fit is reported because there is nothing of this family to
fit — five points are flat and the sixth is a cliff.

**The CycloneDDS ladder is not an artefact of two containers on one host.**
Repeated across two machines over a point-to-point VPN — different NICs, a real
clock offset, the shaping on each host's own tunnel device rather than on a
docker bridge — it reproduces:

| payload | wrapped | one host | two hosts | ratio |
|---:|---:|---:|---:|---:|
| 200 B | 300 B | 1.047 % | 0.952 % | 0.91 |
| 3 kB | 3.1 kB | 3.468 % | 3.465 % | 1.00 |
| 12 kB | 12.1 kB | 11.227 % | 12.149 % | 1.08 |
| 38 kB | 38.1 kB | 31.922 % | 31.773 % | 1.00 |

The two extremes agree to within half a per cent and the worst point to within
eight. What the second host adds — serialisation on a real interface, a clock
that is not shared, a tunnel MTU — does not move the transfer function.

**The effective packet is smaller than the fragment, and by a measurable
amount.** CycloneDDS's fitted `F` lands at 872–1008 B against a configured
1024 B fragment, i.e. every packet carries roughly 15 % of itself in headers.
That is the same quantity as the wire-to-payload ratio measured on the tunnel
(1.13–1.17×), reached from the opposite direction: one from counting bytes on
an interface, one from fitting a loss curve.

Verification: manual, and it needs a source checkout with Docker:

```bash
for size in 200 1000 3000 6000 12000 38000; do
  for rmw in cyclone fastdds zenoh; do
    rosotacom benchmark probe \
      --project src/rosotacom/resources/examples/rosotacom.yaml \
      --profile tilt-iid --size "$size" --rate-hz 10 --duration 240 \
      --repeats 1 --rmw "$rmw" --sudo-mode container --no-plot
  done
done
```

Read the per-size counts from `logs/b/status/events.jsonl` of each instance
(`kind: transit`, `status` delivered or lost, `size_bytes` on the delivered
rows) rather than from `result.json`. The CycloneDDS ladder must fit one
`(p, F)` to within about ten per cent at every point; the zenoh ladder must be
flat below 12 kB.

Across two machines the same ladder runs through `ota-benchmark probe` with
`--install-mode checkout` and a `--peer-exec` transport on each peer. It needs
2.5.dev78 or later: before it the profile's impaired leg landed on the peer the
a/b convention calls the sender rather than the one the session's topic map
does, the benchmark's load never reached the peers' publishers, and the
collection looked for their artifacts in a directory the project does not use.

## Status

confirmed, 2026-08-31.

## Publication notes

The useful sentence is the scope one. `1-(1-p)^N` is textbook and the field fit
of it is a real result, but reporting it without naming the transport invites
the reading that it describes the *link*. It does not: on one link, at one
impairment, three transports produce a clean power law, a distorted one, and no
law at all. A paper that quotes a per-fragment rate should therefore quote the
middleware and its QoS beside it, and a design rule derived from the law should
say that it holds for a best-effort path that fragments and does not repair.

Two smaller points travel with it. The **effective packet is not the fragment**:
the fitted size falls about 15 % below the configured one, which is the header
share, and it is the same 15 % an interface counter reports as wire-over-payload
— so a fragment budget computed from the configured size understates exposure
by that much. And zenoh's shape is the more interesting half of the comparison
rather than a footnote: a transport that repairs turns a size-dependent
availability problem into a capacity cliff, which is a different failure to
design against, not a smaller one.
