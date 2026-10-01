# Fitted network profiles, 2026-08 cellular field campaign

[`netem_profiles.yaml`](netem_profiles.yaml) holds network profiles fitted to
eleven drive windows of a remote-assistance vehicle on a public 4G/5G uplink in
Karlsruhe, Germany, in August 2026. Each window was fitted from the passive
recording of its own application traffic, and each gives two profiles:

- `<key>-steady`: one-way delay, jitter and per-packet loss of the steady state;
- `<key>-events`: one cycle of the same steady state for the fitted mean time
  between link events, followed by one fitted short event.

| key | date | window | site | ROS 2 | DDS fragment |
|---|---|---|---|---|---|
| `r3` | 2026-08-17 | 14:57 | Oststadt | Kilted | 1200 B |
| `r4a` | 2026-08-19 | 13:20 | Campus Ost | Kilted | 1200 B |
| `r5b` | 2026-08-21 | run 2 | Oststadt | Lyrical | 1024 B |
| `r5c` | 2026-08-21 | last run | Oststadt | Lyrical | 1024 B |
| `r6a` | 2026-08-21 | 17:08 | Oststadt | Lyrical | 1024 B |
| `r6b` | 2026-08-21 | 17:45 | Oststadt | Lyrical | 1024 B |
| `r6c` | 2026-08-21 | 18:07 | Oststadt | Lyrical | 1024 B |
| `r6d` | 2026-08-21 | 18:35 | Oststadt | Lyrical | 1024 B |
| `r6e` | 2026-08-21 | 18:38 | Oststadt | Lyrical | 1024 B |
| `d0926` | 2026-08-24 | 09:26 | Oststadt | Lyrical | 1024 B |
| `d1136` | 2026-08-24 | 11:36 | Oststadt | Lyrical | 1024 B |

## Use

Point a project at the file and select a profile by name:

```yaml
# rosotacom.yaml
profiles: field-profiles/2026-08/netem_profiles.yaml
```

```bash
rosotacom benchmark probe --profile r6c-events
```

Profile syntax and the shaping they produce are described in
[network profiles reference](../../docs/reference/network-profiles.md).

## Replay check

`r3` (L1 in the paper) is the window used for this supplementary model check.
[`replay/replay.py`](replay/replay.py) sends synthetic frames with the recorded
camera frame sizes and times through netem and counts complete frames.
It contains no source pictures and is not the DDS/decoder experiment in the
current paper's laboratory section. Sender and receiver are two
disposable Docker containers on their own network; only the sender has
`NET_ADMIN`. Every frame is represented by UDP datagrams with up to 1188 B of
synthetic payload plus a replay header, at its recorded time. The datagram
count follows the 1200 B DDS-fragment model; this is not an RTPS wire replay.
For `r3-events` the timeline is
switched inside the sending container and every switch is logged, so the time
spent in the event state is measured (4.0 % realized, 3.8 % nominal).

```bash
python3 field-profiles/2026-08/replay/replay.py \
  --scenario none steady events --repeat 9 --work /tmp/replay
```

Use an empty or absent work directory for each experiment; existing results
are refused to prevent mixing runs. It needs Docker and PyYAML and sends the first 300 s of the plan (3 001
frames), so a run takes about five minutes and the 27 runs above about 35 at the
default four in parallel. It writes one JSON file per run and `summary.json`,
which puts each scenario beside the loss the profile implies for this plan,
1-(1-p)^n per frame of n fragments.

Frame loss in %, nine runs per scenario, against what the drive itself lost:

| | key frames | delta frames | all frames (range) | 1-(1-p)^n |
|---|---|---|---|---|
| no shaping | 0 | 0 | 0 | 0 |
| `r3-steady` | 3.08 | 0.51 | 1.03 (0.77 to 1.17) | 1.06 |
| `r3-events` | 5.79 | 1.15 | 2.08 (1.93 to 2.37) | 2.13 |
| the drive, same 300 s | 2.99 | 2.21 | 2.37 | |
| the drive, whole recording | 5.06 | 1.59 | 2.33 | |

The replay rows are reference runs of this supplementary UDP experiment,
not the current paper's DDS/decoder laboratory results. This script, rerun on
2026-09-29 with nine runs per scenario, gave 2.19 % (1.83 to 2.60) with
`r3-events` and 1.11 % with `r3-steady`. The difference between the two means
is smaller than the spread of single runs.

The event timeline raises loss relative to the steady-only model, but matching
an aggregate does not validate the mechanism or either frame class separately.
The drive's own loss over 300 s spans ranges from 0.73 % to
4.40 % across the recording, which is the scale the difference has to be read
against.

The frame plan, [`replay/r3_frame_plan.csv`](replay/r3_frame_plan.csv), describes
metadata of the H.264 stream emitted on 2026-08-17, not its pictures: one row
per frame with its time from the start of the vehicle-side recording (`t_s`),
its serialized size in bytes (`size`) and its key-frame flag (`kf`), 12 016
frames at 10 Hz. It is derived from the vehicle-side recording of that drive,
which is not part of a published dataset, so the plan is published here
(SHA-256 `dfdfb6118eea0fab9df8358e84533e9b7b361a4b8d05720bb0545a6c37a61a81`).

## Size law on transit records

[`transit/r6c.jsonl.gz`](transit/r6c.jsonl.gz) holds the uplink transit
records of window `r6c` (L6 in the paper) in the format the communication peers write
([measurement records reference](../../docs/reference/measurement-records.md)), already joined: one
record per wrapped message with its topic, sequence number, verdict
(`delivered` or `lost`), serialized size and send time `t_wrap`, here in
seconds from the start of the window. 78 909 messages on ten topics, 817 lost.
The public-road excerpt of the FZI-RECAP dataset is cut from the same drive
window. The one heartbeat topic that named the vehicle is renamed `/heartbeat`.

Every row has exactly `kind`, `source`, `target`, `topic`, `epoch`, `seq`,
`status`, `size_bytes` and `t_wrap`. No payloads, images, encoded video,
coordinates, IP addresses or host names are included. Stream names, sizes,
timing and the window's date remain public metadata. The access-label trace
is absent, so the example fits LTE and 5G NR together, as the campaign table
does, not the label-conditioned or held-out camera tests.

[`rosotacom.fragment_loss`](../../src/rosotacom/fragment_loss.py) fits the
per-fragment loss rate of any such records: it separates link events (seconds
in which three or more topics lose, with one second either side), fits the
rate of each layer by maximum likelihood over the fragment counts, and gives a
95 % interval from a bootstrap over 60 s blocks.

```bash
python -m rosotacom.fragment_loss field-profiles/2026-08/transit/r6c.jsonl.gz --fragment-bytes 1024
```

```text
78909 messages, 817 lost, 109 s in link events
steady     70225 messages    419 lost  per-fragment loss (1024 B) 0.143 % [0.039, 0.281]
event       8684 messages    398 lost  per-fragment loss (1024 B) 1.033 % [0.721, 1.516]
```

followed by the observed loss per fragment-count bin beside 1-(1-p)^n. The
steady rate is `r6c-steady`'s loss above and the upper end of the per-window
range in the campaign table (0.009 to 0.143 %). The 1.033 % event-layer rate
uses all messages inside the padded one-second episodes. It is not the 4.06 %
burst parameter in `r6c-events`, fitted on loss-bearing 100 ms slots.
The interval's bounds depend on the random draws: the paper's
evaluation, with another generator, reports 0.039 to 0.272 %.

## What the numbers are, and are not

- The fitted `p` is an effective parameter of the message-loss model
  1-(1-p)^n, not the measured datagram loss rate. netem applies that parameter
  independently per packet. Use the matching fragment count; real burst loss
  and receiver discards are not equivalent to this synthetic process. The
  current paper reports 77 % of size bins within a factor of two under LTE,
  but 44 % under 5G NR, outside multi-topic loss episodes. Those conditioned
  results are not reproduced by these pooled-label profiles. The timeline's
  burst rate is fitted on 100 ms slots with at least one lost message.
- No profile sets a loss correlation. netem draws a correlated loss from a
  mix of a fresh and the previous random number, which is almost never below
  a few percent, so `loss 4% 25%` drops about 0.1 % of packets instead of 4 %.
  Burst loss is expressed by the `<key>-events` timeline instead.
- Delay includes the operator-side bridge hop.
- No profile sets a rate: the link never saturated during these windows, so a
  passive recording cannot identify its capacity. The downlink was not fitted.

## License

The profiles, the frame plan and the transit records in this directory are licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). The rest of the
directory's code is [BSD-3-Clause](../../LICENSE.txt). Attribution: Martin
Gontscharow, FZI Research Center for Information Technology, 2026. Retain the
license link and indicate changes. See [license scope](../../docs/licenses.md)
for the repository's third-party exceptions.
