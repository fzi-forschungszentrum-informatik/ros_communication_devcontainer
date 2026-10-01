# Identifying keyframes in ffmpeg camera streams

An ffmpeg-transported camera topic
(`ffmpeg_image_transport_msgs/msg/FFMPEGPacket`) is not a uniform stream: the
encoder emits one **keyframe** (I-frame, self-contained) followed by
`gop_size - 1` small **delta frames** (P-frames), then repeats. The two
classes differ in size by one to two orders of magnitude — measured on the
remote-assist processed handoff trace (gop_size 5, 10 Mbit/s cap, 10 Hz):
keyframes 21–89 KiB (mean ~43 KiB), delta frames 0.3–15 KiB (mean ~4.5 KiB),
with per-GOP contrast up to ~180×. For network analysis this periodic burst
pattern *is* the stream: a link must absorb the keyframe spikes, not the
average bitrate. Frame classification is therefore the first step of any
GOP-aware bandwidth, loss, or latency analysis.

## Primary algorithm: the `flags` field

`FFMPEGPacket.flags` mirrors libav's packet flags; **bit 0 is
`AV_PKT_FLAG_KEY` and is set exactly on keyframes**:

```python
is_keyframe = bool(msg.flags & 0x1)   # AV_PKT_FLAG_KEY
```

This works on live subscriptions, recorded handoff traces, and anonymized
replay bags alike: `rosotacom anonymize` deliberately preserves `flags` and
`pts` when it zeroes FFMPEGPacket payloads (see
`src/rosotacom/resources/ws/session/creation/anonymize_bag.py`), because the
keyframe/delta structure is stream shape — like the sizes and timing the
anonymizer already keeps — not scene content.

Offline tooling can classify frames straight from serialized CDR bytes (as
stored in mcap files) without a ROS runtime via
[`rosotacom.ffmpeg_packet.parse_ffmpeg_packet`](../src/rosotacom/ffmpeg_packet.py):

```python
from rosotacom.ffmpeg_packet import parse_ffmpeg_packet

info = parse_ffmpeg_packet(serialized_message_bytes)
info.is_keyframe   # flags bit 0
info.pts           # encode order, also preserved by anonymization
info.payload_size  # encoded frame bytes
```

Grouping a stream into GOPs is then just splitting at keyframes. Note the
encoder may emit keyframes *early* (scene cuts), so GOPs can be shorter than
`gop_size` — assert spacing `<= gop_size`, not `== gop_size`.

## Transit records carry the flag

The receiving status overview applies the same parse live: when an inbound
`OtaStamped` envelope's `msg_type` names an FFMPEGPacket, it walks the wrapped
CDR bytes to `flags` (`com_py/ffmpeg_flags.py`, a dependency-free mirror of the
host parser) and writes `keyframe: true/false` on every delivered measurement records reference
transit row in `events.jsonl`. Synthesized lost rows have no payload and carry
no such field; a payload the walk cannot parse simply omits it. Downstream,
`rosotacom report` annotates a stream from these real flags whenever they cover
more than 90% of its delivered rows (provenance `ffmpeg_flags (transit
records)`), and only recordings without the field fall back to size bimodality.

## Automated stream-stage stats

`rosotacom stream-stats` applies the same packet parser and GOP grouping to
recorded stage bags, then lines the aggregate results up beside post-OTA
`events.jsonl` transit rows. This is the repeatable version of the manual
remote-assist camera analysis:

```bash
rosotacom stream-stats \
  --bag pre=logs/b/metrics/stages_20260702_172900:/sensors/camera/front_medium/resized/image_rect_color \
  --bag handoff=logs/b/metrics/stages_20260702_172900:/sensors/camera/front_medium/resized/image_rect_color/compressed/restamped/drop1of2/ffmpeg \
  --events post=logs/b/status/events.jsonl:/sensors/camera/front_medium/resized/image_rect_color \
  --out stream-stats
```

Inputs are explicit and repeatable:

- `--bag LABEL=PATH:/topic` reads one rosbag2 topic, using message timestamps and
  serialized sizes. When the bag metadata declares `FFMPEGPacket`, the command
  parses the CDR payload and reports encoded-frame payload sizes plus keyframes
  from `flags & 1`. MCAP bags are read through the Python `mcap` package; other
  rosbag2 storage backends fall back to `rosbag2_py` in a ROS environment.
- `--events LABEL=PATH:/topic` reads delivered transit rows from
  `events.jsonl`, using receiver-side `t_com_in` timestamps and `size_bytes`.
  Rows carrying the per-message `keyframe` field annotate GOPs from the real
  flags; older recordings without it use the documented size-bimodality
  fallback when the keyframe share looks like a real GOP.

The Markdown output starts with the comparison table:

```text
| stage | kind | topic | msgs | hz | mean B | p90 B | gap std ms | gaps within +/-10% | GOP spacing |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `pre` | bag | `/.../image_rect_color` | ... | 20 | ... | ... | ... | ... | - |
| `handoff` | bag | `/.../ffmpeg` | ... | 10 | ... | ... | ... | ... | 5 |
| `post` | events | `/.../image_rect_color` | ... | ... | ... | ... | ... | ... | 5 |
```

The JSON output (`stream-stats/stream-stats.json`) carries the same rows plus
the full size and interval distributions, per-GOP-position size table, keyframe
spacing distribution, and the most extreme GOP by keyframe-to-delta mean ratio.
The comparison is intentionally aggregate-only: it does not join individual
messages across decimating stages such as drop or throttle, matching the
index-join caveat in [measurement records reference](reference/measurement-records.md).

The July 2 remote-assist handoff traces used by examples 15/16 produce this
handoff-only comparison:

```text
| stage | kind | topic | msgs | hz | mean B | p90 B | gap std ms | gaps within +/-10% | GOP spacing |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `costmap` | bag | `/costmap/costmap/restamped/globalframe/bz2` | 1396 | 10.001 | 8332.97 | 10208 | 12.701 | 68.817% | - |
| `camera` | bag | `/sensors/camera/front_medium/resized/image_rect_color/compressed/restamped/drop1of2/ffmpeg` | 1396 | 9.999 | 12398.8 | 42300 | 19.691 | 48.459% | 5 |
```

The camera GOP-position table is the expected bimodal GOP-5 shape: position 0
averages 43,723 B, while positions 1-4 average 3,738 B, 4,512 B, 4,834 B, and
4,335 B respectively.

## Fallback: size bimodality

If `flags` is unavailable (foreign recordings, bags anonymized before flag
preservation existed, or `events.jsonl` written before transit rows carried
`keyframe`), the size distribution still separates the two classes: delta frames dominate the stream (a GOP of N contributes N−1), so
the median frame size sits on the delta mode, while keyframes are typically
5–100× larger. `rosotacom.ffmpeg_packet.keyframes_by_size` marks frames
above `3 × median` as keyframes. This misclassifies only when delta frames
get unusually large (heavy motion) or keyframes unusually small (static
scene at a very low bitrate) — prefer `flags` whenever present.

## Validation

- `tests/unit/test_ffmpeg_packet.py` — CDR parsing (alignment across
  odd-length strings), flag semantics, and the size fallback on a synthetic
  GOP pattern.
- `tests/unit/test_ffmpeg_flags.py` — the receiver-side keyframe-bit walk in
  `com_py/ffmpeg_flags.py`: agreement with the host parser on hand-built
  packets, and `None` (never an exception) on truncated or foreign bytes.
- `tests/unit/test_forensics.py` — `rosotacom report` prefers transit-row
  `keyframe` flags over size bimodality, and falls back below the coverage
  gate.
- `tests/unit/test_stream_stats.py` — exact stream statistics, per-GOP-position
  tables from hand-built CDR packets, events-row source loading, comparison
  table shape, and CLI output writing.
- `tests/unit/test_anonymize.py::test_anonymize_msg_preserves_ffmpeg_keyframe_structure`
  — anonymization keeps `flags`/`pts` while zeroing payloads.
- `tests/e2e/test_anonymize_e2e.py` — headless end-to-end: a trace bag written
  in the container, anonymized via the CLI, and re-read with keyframe flags
  intact.
