# Oversubscription Queues — It Does Not Lose Until The Queue Itself Dies

## Claim

Offered load slightly above a rate cap turns entirely into queueing delay, not
loss: one-way delay climbs by hundreds of milliseconds per second of
oversubscription — into the seconds — while per-bin loss stays exactly zero.
Loss appears only when the queue itself is destroyed (in emulation: a
shape-changing timeline step; in the field: a bearer/modem reset), and then it
is the whole backlog at once. Consequences: loss statistics alone certify a
badly broken link as healthy, delay-based failure precedes loss-based failure,
and a recovering link replays a burst of stale messages unless a lifespan
bounds them.

## Setup

- Host pair / topology: rosotacom local benchmark mode using the packaged
  example project, synthetic `bench_1_1_capacity` session, one
  `a->b:/bench_capacity` stream, uplink shaping only (tc inside the peer
  containers' own netns — no host privileges).
- rosotacom SHA: measured at `73fa1de`; public re-runs record the current
  checkout SHA in `result.json.context`.
- Profile: `field-20260817-case3-delay-jam` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml)
  — 10 s clean, 15 s `rate: 900kbit` (the crunch), 20 s clean; fitted to the
  2026-08-17 cellular drive's case3 window (in-flight queueing to 1.4 s at almost
  no loss).
- Seed policy: no stochastic netem elements (rate/delay only), `n=1`.

## Evidence

Evidence grade: packaged-profile benchmark run plus per-bin timeline.

```bash
rosotacom benchmark probe --project src/rosotacom/resources/examples/rosotacom.yaml --profile field-20260817-case3-delay-jam --size-pattern 1x13KB --rate-hz 10 --duration 45 --repeats 1 --no-plot
```

2026-08-18 run (offered 1.04 Mbit/s against the 15 s 900 kbit crunch — 16%
oversubscription): twelve consecutive 1 s bins during the crunch show p95
one-way latency climbing 340 → 601 → 880 → 1117 → 1420 → 1684 → 1944 → 2208 →
2469 → 2704 → 3009 → **3250 ms with `lost = 0` in every single bin** — the
queue (netem child, default limit 1000 packets) absorbs the entire excess.
All 27 lost messages (5.9% of the run) sit in the three bins at the
crunch→clean boundary, where the timeline step replaces a tbf+netem tree with
a netem-only tree and the queued ~2.7 s backlog dies with the old queue.

Two readings beyond the headline: (a) a loss-based health verdict over the
crunch window reads "0% loss" while the operator's picture is three seconds
old and aging — delay-based failure criteria fire long before loss-based
ones; (b) the backlog-drop at the boundary is the emulated twin of a bearer
reset, and it is also the caveat's edge: `tc qdisc replace` steps are seamless
only between same-shape trees — a step that adds or removes `rate` swaps the
tree and drops its queue.

Field recurrence (2026-08-17 cellular drive): five jam episodes with one-way
delay 401–459 ms at zero losses; the case3 window queued to 1.4 s in-flight
with almost no loss; and the drive's last minute ran p90 ≈ 138 ms delay at
0.04 losses/s — the same regime at lower intensity. A 2026-08-18 container
replay of a 9% oversubscription reached 3.66 s delay at exactly 0% loss.

Verification: manual: run the command above from a source checkout with
Docker; read the per-bin `time-bins.jsonl` next to `result.json` — the crunch
bins must show monotonically climbing latency at zero loss, with the losses
(if any) confined to the boundary bins.

## Status

confirmed, 2026-08-18.

## Publication notes

This is the mechanism behind
[delay-alone-no-loss.md](delay-alone-no-loss.md)'s negative control seen from
the other side: delay alone does not lose, and oversubscription alone does not
lose either — it defers. Pair with
[reorder-becomes-reader-loss.md](reorder-becomes-reader-loss.md) for the two
ways "the network was fine, loss says so" misleads. For papers: plot per-bin
delay and loss on one timeline; the visual is a delay ramp under a flat zero
loss line, ending in a loss spike exactly when the queue dies. Lifespan QoS
(and its enforcement gap — writer-side only in CycloneDDS) decides what the
recovery burst does to the application.
