# A Shorter Group Does Not Lose Fewer Key Frames

## Claim

Shortening the group of pictures is the design lever the fragment law hands a
video stream: fewer bytes in the largest message class, so a smaller fragment
count, so a lower loss probability for the frame everything else depends on.
The lever works on that probability and **not on the rate at which key frames
go missing**, because the same change makes proportionally more of them.

Three size distributions at one cadence, each standing in for a group length,
under an unchanged link:

| link | pattern | large msg | per second | loss of it | **lost per second** | offered |
|---|---|---:|---:|---:|---:|---:|
| p = 1 % | 4 small + 1 large | 38.1 kB | 2.00 | 29.52 % | **0.590** | 0.936 Mbit/s |
| p = 1 % | 2 small + 1 large | 24.1 kB | 3.33 | 23.82 % | **0.794** | 0.915 Mbit/s |
| p = 1 % | 1 small + 1 large | 14.1 kB | 5.00 | 12.45 % | **0.623** | 0.768 Mbit/s |
| p = 2 % | 4 small + 1 large | 38.1 kB | 2.00 | 56.43 % | **1.127** | 0.935 Mbit/s |
| p = 2 % | 2 small + 1 large | 24.1 kB | 3.34 | 37.95 % | **1.266** | 0.915 Mbit/s |
| p = 2 % | 1 small + 1 large | 14.1 kB | 5.00 | 21.98 % | **1.099** | 0.768 Mbit/s |

Read the two bold columns together. **The loss of the large message falls by
more than half — 29.5 % to 12.5 % — while the rate at which large messages are
lost does not move: 0.590, 0.794, 0.623 per second.** The lever's two factors
cancel to within a few per cent, by construction rather than by accident: a
message 2.7× smaller occupies 2.7× fewer packets, and there are 2.5× more of
them.

**The factor that does move the answer is the one the designer does not set.**
Doubling the link's per-packet rate moves the same quantity by 1.8–1.9× in
every pattern.

The rule is therefore usable and its limit is exactly stated: it predicts
*which* term changes and by how much, and it does not predict that the outcome
improves. What a shorter group buys is the length of each hole, not their
number — and, worth having, about 18 % less offered bandwidth for the same
cadence.

## Setup

- Host pair / topology: one host, the packaged local benchmark rig — two
  communication containers on their own Docker network, `tc` inside each
  container's own netns (`--sudo-mode container`).
- Session: `bench_1_1_capacity` from the packaged example project, one
  `a->b:/bench_capacity` stream at 10 Hz with `--size-pattern`, CycloneDDS,
  OTA QoS best_effort / KEEP_LAST depth 1. The patterns are
  `4x5KB+1x38KB`, `2x5KB+1x24KB` and `1x5KB+1x14KB` — a large class against a
  fixed small one, in the three ratios a group length of five, three and two
  produces.
- rosotacom SHA: measured at `7bb420c`; needs 2.5.dev74 or later for the fitted
  delay table to arm.
- Profiles: `tilt-iid` and `tilt-iid-worse` in
  [`src/rosotacom/resources/examples/profiles.yaml`](../../src/rosotacom/resources/examples/profiles.yaml)
  — the 2026-08-17 drive's own delay table at 25 ms ± 18 ms with 1 % and 2 %
  independent per-packet loss. Nothing else differs between the two links.
- Seed policy: netem `loss` is unseeded, `n=1` per cell; each cell offers about
  2100 messages of which 420–1680 are of the large class, so the intervals on
  the large-class figures are 3–5 points wide and on the per-second rates
  correspondingly.

## Evidence

Evidence grade: per-message counts from the receiving peer's own measurement records reference
transit records. Lost rows carry no payload and therefore no size, so the size
classes are learnt from the delivered rows as the publisher's repeating pattern
against the sequence number, and every row — delivered or lost — is then
assigned by its position in that cycle. The window is taken in sequence numbers
(first 300, last 50 dropped).

The transfer function itself holds throughout, so the cancellation is not a
breakdown of the model but a consequence of it: fitting one `(p, F)` over the
mixed workload leaves residuals of 0.86–1.11 at p = 1 % and 0.99–1.03 at
p = 2 %.

Verification: manual, from a source checkout with Docker:

```bash
for prof in tilt-iid tilt-iid-worse; do
  for patt in 4x5KB+1x38KB 2x5KB+1x24KB 1x5KB+1x14KB; do
    rosotacom benchmark probe \
      --project src/rosotacom/resources/examples/rosotacom.yaml \
      --profile "$prof" --size-pattern "$patt" --rate-hz 10 --duration 240 \
      --repeats 1 --sudo-mode container --no-plot
  done
done
```

Split each run's transit rows by size class and divide the large class's losses
by the window in seconds. Within one link the three patterns must agree on that
rate to within about a quarter; between the two links it must roughly double.

## Status

confirmed, 2026-08-31.

## Publication notes

This is the bench form of a field result that reads, without it, like an
excuse. A campaign that shortened its group of pictures and then measured no
improvement in the aggregate can attribute that to the link having been worse
on the days that followed — which is true and unsatisfying, because it is
unfalsifiable from one campaign. Here the link is held still by construction
and the aggregate still does not move, so the cancellation is a property of the
lever rather than of those evenings.

Stated positively, it is the sentence that makes the design rule usable: the
fragment law tells you what will change and by how much, and the thing it
changes is not the thing an operator counts. For a video path the honest
formulation is that a shorter group trades hole *length* against hole *count*
at constant link, and pays for it in nothing — the offered bandwidth falls by
about a fifth at the same cadence and quality target.

The 2× row is the other half and belongs with it wherever the rule is offered
as advice: the term the designer does not control moved the outcome by more
than the term they do.
