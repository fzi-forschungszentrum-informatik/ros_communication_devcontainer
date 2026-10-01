# Sender-Side Transit Rows Made A Dead Link Report Zero Loss

## Claim

`summarize_transit_records` counted `delivered = rows − lost`. Sender-side
transit rows carry the status `sent` and are written whether or not anything
arrives, so a topic that the receiving peer never saw at all summarised as
`delivered == expected`, `loss_pct 0.0` — with the sender's own publication
cadence reported as the arrival spacing. The most dangerous shape a wrong number
can take: a link that delivered nothing scored better than one that delivered
96% of its messages.

Delivery is a receiver fact. Rows still `sent` after the join are `unconfirmed`;
`loss_pct` is `None` when no receiver accounted for the topic at all; and an
oracle reads that `None` as the worst case rather than as zero.

## Setup

- Host pair / topology: not a transport property — the summary is a pure
  function of collected records, so the reproduction is a unit test over
  synthetic rows. It was found on a two-machine run whose OTA transport
  delivered nothing.
- rosotacom SHA: present up to `6e4b748`, fixed in `730ddf5`.
- Profile: none — the defect is in the accounting, not in the link.
- Seed policy: not applicable, deterministic.

## Evidence

Evidence grade: unit-level, deterministic, plus the live run that exposed it.

Live (2026-08-19, `rmw.ota: zenoh_ros2dds` over two machines): the sending peer
wrote 816 `outbound`/`sent` rows, the receiving peer wrote 5 rows and no
delivery at all. The run's `result.json` reported `delivered 807, expected 807,
loss_pct 0.0` for that topic, with inter-arrival p05/p95 of 102/104 ms — the
publisher's own timer, not an arrival distribution. The receiver's `status.txt`
for the same run said `ota_recv ABSENT/GRAPH pub=0` and `com_in IDLE`.

The window mattered as much as the number: this only became reachable when
sender-side transit rows landed (issue #294). Before that, a topic with no
receiver rows had no rows at all and simply did not appear.

Verification: `python -m pytest -q tests/unit/test_transit.py -k sender_rows`
— sender rows alone report `delivered 0`, `unconfirmed 3` and `loss_pct None`,
and a mixed set counts only the receiver's verdicts.
`tests/unit/test_benchmark.py::test_an_unknown_loss_figure_is_read_as_the_worst_case`
pins that an oracle does not pass on the `None`.

## Status

confirmed, 2026-08-19.

## Publication notes

A measurement pipeline that gains a second observer gains a way to be
confidently wrong: the new rows were real, the join was correct, and the
aggregate stopped meaning what its name said. Worth citing whenever a loss
figure is quoted from an automated run — state which side observed it. The
general rule the fix encodes: an aggregate over rows from two observers must say
which observer each row came from before it counts anything.
