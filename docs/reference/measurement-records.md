# Per-message measurement records

The OTA wrapper adds a sequence number and timestamp. The status monitor
infers an epoch when the sequence resets; it is a log field, not a field of
the wire message. Sender and receiver observations join by
`(source, topic, epoch, seq)` without examining payload contents. Epochs keep
a restarted publisher's sequence space separate from the preceding run.

[transit.py](../../src/rosotacom/transit.py) loads, joins and summarizes records.
Receiver-side delivered, reordered and lost records establish observed delivery.
Sender-only records are not evidence of successful reception.

Delivered and reordered rows take precedence over a lost verdict for the same
key, for example when a message arrives after a gap was first reported. Sender
rows fill missing size and send-time fields. Counts concern the instrumented
application boundary, not radio packets or decoder output.

One-way timestamps require a clock reference. The echo heartbeat estimates RTT
and a peer clock offset under a symmetric-path assumption. Local stage timings
and corrected OTA timings answer different questions; an RTT estimate is not a
direct one-way measurement.

- [Session configuration](../../session-configuration.md) configures instrumentation.
- [Link trace](../link-trace.md) records interface counters and optional modem metrics.
- [Datagram capture](../ota-pcap.md) records packet headers at the OTA interface.
- [Forensics](../forensics-report.md) combines transit rows, link data and state transitions.
- [Paper artifacts](../paper-artifacts.md) maps these components to the paper.

Loss inferred from sequence gaps has finite observation-window boundaries.
Missing startup or trailing messages cannot always be distinguished from an
inactive sender without the corresponding sender record.
