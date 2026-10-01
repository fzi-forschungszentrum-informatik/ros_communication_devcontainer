# Paper artifacts: per-message delivery in cellular remote assistance

Companion code for **Beyond Bitrate and Latency: Per-Message Delivery in
Cellular Remote Assistance**, Martin Gontscharow et al., 2026.
This page maps the paper's methods to the released files and separates
reproducible examples from results that require the full campaign data.

**Start here:** the [size-law example](#reproduce-one-window-without-ros-or-docker)
uses only Python and the included metadata. It reproduces the effective loss
parameter for **L6** in the paper's campaign table. No vehicle, ROS installation,
Docker daemon, credentials or camera images are needed.

## From the paper to the code

| Paper method or result | Released implementation and input | What can be checked here |
|---|---|---|
| Measurement setup: sequence numbers per offered message | [Wrapper](../src/rosotacom/resources/ws/ros2src/com_py/com_py/universal_ota_wrapper.py), [unwrapper](../src/rosotacom/resources/ws/ros2src/com_py/com_py/universal_ota_unwrapper.py), [message definition](../src/rosotacom/resources/ws/ros2src/com_msgs/msg/OtaStamped.msg) | Sender/receiver instrumentation |
| Delivery and clock-aware delay | [Transit join](../src/rosotacom/transit.py), [measurement reference](reference/measurement-records.md), [echo heartbeat](../src/rosotacom/resources/ws/ros2src/com_py/com_py/heartbeat_echo.py) | Join key, receiver verdicts and clock assumptions |
| Size law, Eq. (1), and the campaign table | [Fit and bootstrap](../src/rosotacom/fragment_loss.py), [L6 records](../field-profiles/2026-08/transit/r6c.jsonl.gz) | Pooled-label L6 fit, not camera-held-out or label-conditioned fits |
| Interface and router observations | [Link trace](link-trace.md), [collector](../tools/milesight_cellular_metrics.py), [configuration](router-collector.md) | Collection fields and limits; the HTTP collector is not the September direct-modem sampler |
| Packet-level evidence | [Capture code](../src/rosotacom/resources/ws/ros2src/com_py/com_py/ota_pcap.py), [reference](ota-pcap.md) | Header capture; campaign captures and their paper-specific classification are not included |
| Controlled queueing | [Shaper](../src/rosotacom/network_shaper.py), [profile schema](reference/network-profiles.md) | Rate caps, netem, bounded lifetime and teardown, not the full laboratory campaign |
| Diagnostic mechanisms | [Findings](findings/README.md), [forensics](forensics-report.md) | Scoped reproductions, not regeneration of every paper figure |
| Supplementary profile example | [Eleven profiles](../field-profiles/2026-08/README.md), [UDP replay](../field-profiles/2026-08/replay/replay.py), [L1 frame plan](../field-profiles/2026-08/replay/r3_frame_plan.csv) | Fragment-count model under netem, not the paper's DDS/decoder experiment |

## Reproduce one window without ROS or Docker

From the repository root, with Python 3.10 or newer:

```bash
PYTHONPATH=src python3 -m rosotacom.fragment_loss field-profiles/2026-08/transit/r6c.jsonl.gz --fragment-bytes 1024
```

This uses the standard library, performs no network access and writes to the
terminal. It fits both layers and computes 2,000 bootstrap draws over 60-second
blocks. Expected leading output:

```text
78909 messages, 817 lost, 109 s in link events
steady     70225 messages    419 lost  per-fragment loss (1024 B) 0.143 % [0.039, 0.281]
event       8684 messages    398 lost  per-fragment loss (1024 B) 1.033 % [0.721, 1.516]
```

`r6c` identifies **L6**, the 21 August 2026 evening lap starting at 18:07.
The campaign table reports the steady point estimate 0.143 %. Its interval is
[0.039, 0.272] %, using a different bootstrap random generator. This example
reproduces the point estimate, not bit-identical bootstrap bounds.
[Tests](../tests/unit/test_fragment_loss.py) check the rates and event selection.

A multi-topic loss episode is a second with losses on at least three topics,
padded by one second on either side. The fitted `p` is an **effective model
parameter**, not a measured datagram loss probability. The input has no
access-label trace, so this command pools LTE and 5G NR. It cannot reproduce
the paper's label-conditioned, camera-held-out tests or identify reader drops.
Its 1.033 % episode fit also differs from `r6c-events`' 4.06 % netem burst
parameter, fitted on loss-bearing 100 ms slots.

## What is in the data files?

The [L6 gzip file](../field-profiles/2026-08/transit/r6c.jsonl.gz) contains
78,909 JSON records with exactly nine fields: `kind`, `source`, `target`,
`topic`, `epoch`, `seq`, `status`, `size_bytes`, `t_wrap`. Source and target
are the generic roles `vehicle` and `center`; time is relative to the window
start. Topic names identify stream types. **No image pixels, encoded video,
message payloads, GNSS positions, IP addresses or host names are included.**
The vehicle-specific heartbeat name was replaced with `/heartbeat`.
The date, time, stream names, sizes and timing remain public metadata.

The [L1 CSV](../field-profiles/2026-08/replay/r3_frame_plan.csv) contains only
frame time, serialized size and key-frame flag. Its replay constructs synthetic
payloads, so neither file exposes source pictures. The
[data contract](../tests/contract/test_publication_data.py) checks the schemas
and rejects additional fields.

## Optional synthetic profile replay

For the larger codebase, see [installation](../README.md). For this experiment,
Python with PyYAML and a running Docker daemon are sufficient:

```bash
python3 field-profiles/2026-08/replay/replay.py --scenario none steady events --repeat 9 --work /tmp/rosotacom-field-replay
```

Use an empty or absent output directory. The script creates isolated networks,
caps each container at 256 MiB and runs at most four jobs in parallel.
The first run builds its image and needs Internet access. Each repetition
lasts 300 seconds; 27 repetitions take about 35 minutes after image preparation.
Each writes a result; `summary.json` collects them. Failures return nonzero.
See the [profile README](../field-profiles/2026-08/README.md) for reference
results and sampling variability.

This sends zero-filled UDP datagrams at recorded times and fragment counts.
It does not instantiate DDS, decode H.264 or compare picture quality. It cannot
reproduce the paper's **75.4 % to 90.6 % intact-picture result** for separate
writers plus order restoration. That experiment's runner, full input recording
and figure-generation pipeline are not included in this release.

## Release scope and provenance

The [FZI-RECAP dataset](https://url.fzi.de/fzi-recap) separately releases two
recording excerpts. Here are the communication stack, generic transit analysis,
netem shaper, one-window fit and supplementary profiles. Full campaign
recordings, vehicle deployments, route-model analysis, label-conditioned fits
and manuscript table/figure scripts are not included. The excerpts and this
example support inspection of the method, not recomputation of every result.

The runtime baseline is rosotacom **2.5.dev85**. The August campaign used
2.5.dev39 through 2.5.dev73, under Kilted and Lyrical. This is a maintained
snapshot, not a frozen copy of every field deployment. The L6 command supplies
its recorded 1024 B fragment size explicitly; the L1 replay uses 1200 B.
Current defaults must not replace a measurement's recorded configuration.

See [validation](ci.md) for local checks. Project-authored code is
**BSD-3-Clause**; three retained ROS test templates are **Apache-2.0**, and
field profile data are **CC BY 4.0**. See [license scope](licenses.md) before
redistributing a built environment. Cite a release-pinned version of this
page so its links keep pointing to the reviewed source.
