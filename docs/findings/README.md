# Public Findings Ledger

This ledger records one measured communication effect per file when the effect is
reproducible from public material: packaged sessions, public example profiles,
synthetic loads, anonymized replay fixtures, or deterministic emulation. Findings
that require private bags, private hosts, real cellular/WLAN links, or operator
calibration data stay in the operator harness ledger.

Each finding keeps the export-oriented schema:

- Claim
- Setup
- Evidence
- `Verification:` entry
- Status
- Publication notes

Run the local schema and index check with:

```bash
python3 tests/contract/test_findings.py
python -m pytest -q tests/contract/test_findings.py
```

The check is intentionally stdlib-only when run as a script, so docs and
contract lanes can call the same logic. performance bands reference defines why findings need a
living verification hook.

## Findings

- [The size law belongs to the transport, not to the link](the-size-law-belongs-to-the-transport-not-the-link.md) - one emulated link, three OTA middlewares: CycloneDDS follows `1-(1-p)^N` with a flat residual over three decades of size, Fast DDS keeps the form and loses the fit, and zenoh is flat from 300 B to 12 kB and then falls off a cliff at 38 kB.
- [Neither a queue nor a burst tilts the size law](neither-a-queue-nor-a-burst-tilts-the-size-law.md) - the field's size-systematic residual is reproduced by none of the three candidates: independent per-packet loss fits flat, a token bucket a burst can fill changes nothing systematic, and a serialisation constraint does not tilt the curve but truncates it (38 kB goes from 32% to 97.5% while 1 kB pays 10% more).
- [A shorter group does not lose fewer key frames](a-shorter-group-does-not-lose-fewer-key-frames.md) - shortening the group cuts the large message's loss from 29.5% to 12.5% and leaves the rate at which large messages go missing unchanged (0.59, 0.79, 0.62 per second), while doubling the link's per-packet rate moves it by 1.8x.
- [A fitted delay table never reached the shaper](a-fitted-delay-table-never-reached-the-shaper.md) - every profile carrying a custom netem distribution failed at arming: the table was installed as `/usr/lib/tc/<name>.dist` while iproute2 on a multiarch image reads `/usr/lib/<triplet>/tc`, so the drive-fitted delay tail was never actually in use.
- [Sender-side transit rows made a dead link report zero loss](sender-rows-make-a-dead-link-look-loss-free.md) - `delivered = rows - lost` counted the sender's own `sent` rows as deliveries, so a topic the receiver never saw summarised as `delivered == expected` at 0.0% loss, with the publisher's timer reported as arrival spacing.
- [A bridge that learns types from the graph cannot use a transport that does not carry the graph](graph-derived-types-block-a-non-graph-transport.md) - over `zenoh_ros2dds` the payload topic never arrived in either run: the bridge routes a topic once a local reader exists and the reader waited for the bridge, leaving only `Pending=1` in a log.
- [A Fast DDS datagram cap stops every message larger than it](fastdds-datagram-cap-drops-large-samples.md) - copying CycloneDDS's 1200 B fragment size into the Fast DDS OTA profile makes every sample above the cap never arrive, at 1200 B and at 8192 B, sync and async — while the 84 B heartbeat keeps flowing at 10 Hz.
- [A fragment sized exactly like the datagram cap stops every fragmented sample](cyclone-fragment-equal-to-max-drops-fragmented-samples.md) - CycloneDDS 11.0.1 delivers nothing above one fragment when `FragmentSize` equals `MaxMessageSize`, where 0.10.5 delivers everything on the identical profile; a 40 B string still crosses, and discovery, endpoint announcement and proxy-writer matching all succeed while no sample arrives.
- [Two CycloneDDS configurations on one domain stop interoperating](one-domain-two-cyclone-configs-stop-interoperating.md) - on 11.0.1 a publisher and a subscriber on the same host and domain deliver nothing when one carries the local profile and the other the OTA profile, where 0.10.5 delivers in all four combinations; discovery, endpoint announcement and matching all succeed while no sample moves.
- [A lost keyframe silences exactly the next gop_size−2 deltas](keyframe-loss-kills-the-next-deltas.md) - libavcodec emits nothing for them, the last delta decodes with artifacts, the next keyframe restores clean decode; a lost delta never blocks decode — so the display hole is gop_size × frame period, deterministically.
- [Any transport element in a Fast DDS profile doubles every sample on the wire](a-transport-element-doubles-every-fastdds-sample.md) - 366 datagrams for 180 samples against 183 without it; `<builtinTransports>DEFAULT` is a no-op selection and doubles too, so the trigger is the element's presence — removing it takes the shipped OTA profile from 2.10x the payload to 1.04x at identical delivery.
- [Lifespan is enforced by one DDS reader and not the other](lifespan-is-enforced-by-one-reader-and-not-the-other.md) - under the same 20% oversubscription both stacks queue alike (472 and 433 delivered at ~1.1-1.2 s); with `lifespan: 0.7` CycloneDDS is unchanged and delivers 9.1% inside the bound, while Fast DDS drops to 4 delivered and never exceeds 570 ms — the policy decides shown-vs-suppressed, not how long the wait is.
- [A filling queue adds delay and no loss of its own](queueing-adds-delay-not-loss.md) - 18 bins of ramp from 0.5 to 5.6 s lose 0 of 558 on a clean link and 15 of 558 on one that drops 0.6 % of packets, which is the unshaped rate; the queue defers, the link loses, and the buffer ends it at 97 % of one bin.
- [Oversubscription queues — it does not lose until the queue itself dies](oversubscription-queues-not-losses.md) - 16% over an emulated cap climbed one-way delay 340 → 3250 ms across twelve bins with zero loss in every one; the losses arrived only when the queue died, all at once.
- [Best-effort reordering becomes reader-side loss, independent of history depth](reorder-becomes-reader-loss.md) - under pure delay jitter a Cyclone best-effort reader discards every overtaken sample (57% at 100 Hz under 50+-45 ms), delivery stays strictly monotonic, and depth 50 loses the same as depth 1.
- [Jitter causes loss while bandwidth shortage builds latency](jitter-loss-bandwidth-latency.md) - 18 KB at 20 Hz separates jitter-induced loss from bandwidth-induced queueing.
- [Lighter alternating messages can lose more than steady messages](lighter-message-loss.md) - a 1x18KB+1x0KB pattern lost under a tight emulated profile while the steady 18 KB comparison did not.
- [Delay alone was not the loss boundary for the 18 KB at 20 Hz probe](delay-alone-no-loss.md) - pure delay profiles showed no material loss, and the 300 ms delay-only probe delivered all messages.
- [Zero-loss boundaries for 18 KB at 20 Hz](zero-loss-boundaries-18kb-20hz.md) - 3.2 Mbit/s and 18 ms jitter were good boundaries; 3.1 Mbit/s and 27 ms jitter were bad.
- [Head-of-line blocking makes irregularity the dominant failure mode](head-of-line-irregularity.md) - alternating heavy/light payloads can lose more than a steady heavier stream.
- [Cyclone DDS SPDP discovery bursts can distort tight-link tail latency](cyclone-spdp-discovery-bursts.md) - default 30 s discovery traffic shares the OTA link with payload and is annotated in benchmark diagnostics.
