# A Lost Keyframe Silences Exactly The Next gop_size−2 Deltas

## Claim

In a best-effort libx264 stream (ABR, `tune: zerolatency`, fixed GOP), losing
a KEYFRAME makes exactly the next `gop_size − 2` arrived deltas undecodable —
libavcodec emits **no frame at all** for them — the last delta before the next
keyframe decodes with concealment artifacts, and the next keyframe restores
clean decode. Losing a DELTA never blocks decode: every following access unit
still produces a frame (artifacts until the next keyframe, never a gap). The
behaviour is deterministic — not timing, not decoder state noise — so the
display hole a keyframe loss tears is `gop_size × frame_period`, which is what
makes the GOP length the knob that decides whether one lost packet crosses an
operator-visible threshold.

## Setup

- Host pair / topology: single host, single process, below rosotacom at the
  codec layer — encode with PyAV/libx264 exactly as the ffmpeg transport does,
  delete single access units explicitly, feed the survivors to a fresh
  libavcodec h264 decoder (what a best-effort receiver does after a loss).
- rosotacom SHA: measured at `73fa1de`; re-runs record theirs.
- Profile: none — no network is involved; the deletion IS the loss, which is
  what makes the rule attributable to the codec rather than the transport.
- Seed policy: deterministic synthetic motion clip, `n=1` (the assertions are
  exact-set comparisons, not statistics).

## Evidence

Evidence grade: scripted single-host reproduction, committed next to this file.

```bash
python3 docs/findings/repro/decode_chain_repro.py --gop 5   # needs: pip install av numpy
```

2026-08-18 runs (synthetic clip, libx264 1 Mbit/s ABR, zerolatency, 10 Hz):

| gop | deleted | silent AUs after it | first output after the gap |
|---|---|---|---|
| 5 | keyframe | exactly 3 (= gop−2) | last delta of the GOP, with artifacts |
| 3 | keyframe | exactly 1 (= gop−2) | last delta of the GOP, with artifacts |
| 2 | keyframe | none (= gop−2 = 0) | the following delta, with artifacts |
| any | one delta | none | every AU keeps decoding, artifacts to next KF |

The same rule was first measured on real field material (2026-08-17 cellular
drive, camera FFMPEGPacket stream at gop 5: every lost keyframe silenced the
next 3 arrived deltas, ~400 ms holes at 10 Hz; a `refs=1` encode does NOT
shorten the chain — tested and rejected). The synthetic clip reproduces it
exactly, so the public repro carries the claim without any private bag.

Verification: manual: run the command above (per `--gop 5`, `--gop 3`,
`--gop 2`); the script asserts the exact silent set and exits non-zero on any
deviation.

## Status

confirmed, 2026-08-18.

## Publication notes

This is the quantitative bridge from "loss percent" to "operator-visible
hole": camera loss numbers mean nothing next to costmap loss numbers until
multiplied by the GOP geometry. It drove the 2026-08-17 field configuration
change gop 5 → 3 (hole ≤ 300 ms at equal measured bitrate). For papers: plot
hole duration vs GOP at fixed measured bitrate, and state that intra-refresh
trades the hole for a several-frame artifact window instead (no silent AUs,
lower clean-frame share). See also
[docs/ffmpeg-keyframes.md](../ffmpeg-keyframes.md) for reading keyframe flags
out of recorded transit records.
