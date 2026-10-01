# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import json
import statistics
from pathlib import Path

from rosotacom.transit import (
    filter_transit_records_by_publish_window,
    join_transit_records,
    load_transit_records,
    summarize_transit_records,
)


def _record(seq: int, status: str, ota_ms: float | None = None, t_wrap: float | None = None) -> dict:
    return {
        "kind": "transit",
        "topic": "/x",
        "seq": seq,
        "status": status,
        "t_wrap": t_wrap,
        "sections": {"ota_hop_ms": ota_ms},
        "jitter_ms": 2.0 if ota_ms is not None else None,
    }


def test_load_join_and_summarize_transit_records(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    events = [
        {"kind": "state_transition", "topic": "/x"},
        _record(1, "lost"),
        _record(1, "delivered", 10.0),
        _record(2, "lost"),
        _record(3, "reordered", 30.0),
    ]
    path.write_text("\n".join(json.dumps(event) for event in events) + "\n", encoding="utf-8")

    records = load_transit_records([path])
    joined = join_transit_records(records)
    assert [(record["seq"], record["status"]) for record in joined] == [
        (1, "delivered"),
        (2, "lost"),
        (3, "reordered"),
    ]
    summary = summarize_transit_records(records)["topics"]["/x"]
    assert summary["expected"] == 3
    assert summary["delivered"] == 2
    assert summary["lost"] == 1
    assert summary["loss_pct"] == 33.333
    assert summary["ota_hop_ms"]["p95"] == 30.0


def test_join_keeps_same_topic_sequence_from_different_sources() -> None:
    a = {**_record(1, "delivered", 10.0), "source": "a", "target": "b"}
    b = {**_record(1, "delivered", 20.0), "source": "b", "target": "a"}
    joined = join_transit_records([a, b])
    assert len(joined) == 2
    summary = summarize_transit_records([a, b])["topics"]
    assert set(summary) == {"a->b:/x", "b->a:/x"}


def test_filter_transit_records_by_publish_window_keeps_only_bounded_losses() -> None:
    records = [
        {**_record(1, "delivered", 10.0, 9.9), "source": "a", "target": "b"},
        {**_record(2, "lost"), "source": "a", "target": "b"},
        {**_record(3, "delivered", 10.0, 10.1), "source": "a", "target": "b"},
        {**_record(4, "lost"), "source": "a", "target": "b"},
        {**_record(5, "delivered", 10.0, 10.5), "source": "a", "target": "b"},
        {**_record(6, "lost"), "source": "a", "target": "b"},
        {**_record(7, "delivered", 10.0, 11.2), "source": "a", "target": "b"},
    ]

    filtered = filter_transit_records_by_publish_window(records, start_s=10.0, end_s=11.0)

    assert [(record["seq"], record["status"]) for record in filtered] == [
        (3, "delivered"),
        (4, "lost"),
        (5, "delivered"),
    ]


def test_filter_transit_records_by_publish_window_keeps_delayed_tail_delivery() -> None:
    records = [
        {**_record(1, "delivered", 10.0, 10.0), "source": "a", "target": "b"},
        {**_record(2, "delivered", 50.0, 10.99), "source": "a", "target": "b"},
        {**_record(3, "delivered", 10.0, 11.1), "source": "a", "target": "b"},
    ]

    filtered = filter_transit_records_by_publish_window(records, start_s=10.0, end_s=11.0)
    summary = summarize_transit_records(filtered)["topics"]["a->b:/x"]

    assert [(record["seq"], record["status"]) for record in filtered] == [
        (1, "delivered"),
        (2, "delivered"),
    ]
    assert summary["expected"] == 2
    assert summary["lost"] == 0
    assert summary["ota_hop_ms"]["p95"] == 50.0


def test_summarize_reports_full_latency_distribution() -> None:
    records = [_record(seq, "delivered", ota_ms=10.0 + 5.0 * seq, t_wrap=10.0 + 0.1 * seq) for seq in range(12)]

    latency = summarize_transit_records(records)["topics"]["/x"]["ota_hop_ms"]

    assert latency["min"] == 10.0
    assert latency["max"] == 65.0
    assert latency["mean"] == 37.5
    assert latency["p50"] == 35.0
    assert latency["p95"] == 65.0
    assert latency["p99"] == 65.0
    assert latency["std"] == round(statistics.pstdev(10.0 + 5.0 * seq for seq in range(12)), 3)


def test_summarize_reports_latency_trend_between_first_and_last_third() -> None:
    # Latency ramps 10 -> 65 ms over 12 messages published at 100 ms cadence:
    # the first-third median stays low, the last-third median is high.
    records = [_record(seq, "delivered", ota_ms=10.0 + 5.0 * seq, t_wrap=10.0 + 0.1 * seq) for seq in range(12)]

    trend = summarize_transit_records(records)["topics"]["/x"]["ota_hop_trend_ms"]

    assert trend == {"first_third_p50": 15.0, "last_third_p50": 55.0, "delta": 40.0}


def test_summarize_latency_trend_needs_enough_stamped_points() -> None:
    records = [_record(seq, "delivered", ota_ms=10.0, t_wrap=10.0 + 0.1 * seq) for seq in range(5)]

    trend = summarize_transit_records(records)["topics"]["/x"]["ota_hop_trend_ms"]

    assert trend == {"first_third_p50": None, "last_third_p50": None, "delta": None}


def test_summarize_reports_arrival_spacing_and_bunching() -> None:
    # 12 messages sent equidistantly at 100 ms. Arrivals are on-cadence except one
    # stalled gap (300 ms) immediately followed by a bunched gap (5 ms) — the
    # head-of-line-blocking signature under test.
    gaps = [None, 100.0, 100.0, 100.0, 100.0, 100.0, 300.0, 5.0, 100.0, 100.0, 100.0, 100.0]
    records = [
        {**_record(seq, "delivered", ota_ms=20.0, t_wrap=10.0 + 0.1 * seq), "inter_arrival_ms": gaps[seq]}
        for seq in range(12)
    ]

    spacing = summarize_transit_records(records)["topics"]["/x"]["inter_arrival_ms"]

    assert spacing["nominal_period_ms"] == 100.0
    assert spacing["min"] == 5.0
    assert spacing["max"] == 300.0
    assert spacing["p50"] == 100.0
    assert spacing["p05"] == 5.0
    assert spacing["bunched_pct"] == round(100.0 / 11.0, 3)
    assert spacing["stalled_pct"] == round(100.0 / 11.0, 3)
    assert spacing["stall_then_bunch_pct"] == 100.0


def test_summarize_arrival_spacing_without_timestamps_has_no_bunching_verdict() -> None:
    records = [{**_record(seq, "delivered", ota_ms=20.0), "inter_arrival_ms": 100.0} for seq in range(4)]

    spacing = summarize_transit_records(records)["topics"]["/x"]["inter_arrival_ms"]

    assert spacing["p50"] == 100.0
    assert spacing["nominal_period_ms"] is None
    assert spacing["bunched_pct"] is None
    assert spacing["stall_then_bunch_pct"] is None


def test_join_keeps_sequence_numbers_from_different_epochs_apart() -> None:
    """A peer restart replays seq 0..N; the join must not collapse the second run.

    Field shape (2026-08-13): one centre instance outlived four vehicle
    restarts, and joining on ``(source, topic, seq)`` dropped 123,253 of
    304,876 raw lines -- silently, and concentrated in the mission window.
    """
    before = [{**_record(seq, "delivered", 10.0, t_wrap=float(seq)), "epoch": 0} for seq in range(5)]
    after = [{**_record(seq, "delivered", 12.0, t_wrap=100.0 + seq), "epoch": 1} for seq in range(5)]

    joined = join_transit_records(before + after)

    assert len(joined) == 10
    assert [record["epoch"] for record in joined] == [0] * 5 + [1] * 5
    assert summarize_transit_records(before + after)["topics"]["/x"]["epochs"] == 2


def test_records_without_an_epoch_are_one_epoch() -> None:
    """Instances recorded before the field learned this carry no epoch field."""
    records = [_record(seq, "delivered", 10.0, t_wrap=float(seq)) for seq in range(3)]

    assert len(join_transit_records(records)) == 3
    assert summarize_transit_records(records)["topics"]["/x"]["epochs"] == 1


def test_lost_records_are_bounded_within_their_own_epoch() -> None:
    """A post-restart seq must not vouch for a pre-restart seq's loss.

    Both epochs number their messages 0..3. Only epoch 1 publishes inside the
    window, so only epoch 1's gap may be kept.
    """
    epoch0 = [
        {**_record(0, "delivered", 10.0, t_wrap=1.0), "epoch": 0},
        {**_record(1, "lost"), "epoch": 0},
        {**_record(2, "delivered", 10.0, t_wrap=3.0), "epoch": 0},
    ]
    epoch1 = [
        {**_record(0, "delivered", 10.0, t_wrap=101.0), "epoch": 1},
        {**_record(1, "lost"), "epoch": 1},
        {**_record(2, "delivered", 10.0, t_wrap=103.0), "epoch": 1},
    ]

    kept = filter_transit_records_by_publish_window(epoch0 + epoch1, start_s=100.0, end_s=110.0)

    assert [(record["epoch"], record["seq"], record["status"]) for record in kept] == [
        (1, 0, "delivered"),
        (1, 1, "lost"),
        (1, 2, "delivered"),
    ]


def test_nominal_send_period_ignores_the_step_across_a_restart() -> None:
    """10 Hz before and after a 100 s outage is still a 10 Hz stream."""
    from rosotacom.transit import _nominal_send_period_ms

    records = [{**_record(seq, "delivered", 5.0, t_wrap=seq * 0.1), "epoch": 0} for seq in range(10)]
    records += [{**_record(seq, "delivered", 5.0, t_wrap=100.0 + seq * 0.1), "epoch": 1} for seq in range(10)]

    assert _nominal_send_period_ms(records) == 100.0


def test_sender_rows_can_never_mask_a_receiver_loss() -> None:
    """The sending peer's com_out rows join with the receiver's rows by
    (source, topic, epoch, seq). `sent` ranks below every receiver verdict in
    `join_transit_records`, so a message the link lost stays lost -- while the
    joined record still gains the sender's timestamps."""
    sent = {
        **_record(1, "sent", t_wrap=100.0),
        "stage": "com_out",
        "direction": "outbound",
        "t_com_out": 100.005,
        "sections": {"wrap_to_com_out_ms": 5.0},
    }
    lost = {**_record(1, "lost"), "stage": "com_in", "direction": "inbound"}

    for ordering in ([sent, lost], [lost, sent]):
        (joined,) = join_transit_records(ordering)
        assert joined["status"] == "lost"
        assert joined["t_wrap"] == 100.0
        assert joined["t_com_out"] == 100.005

    delivered = {**_record(1, "delivered", 12.0, t_wrap=100.0), "stage": "com_in", "direction": "inbound"}
    (joined,) = join_transit_records([sent, delivered])
    assert joined["status"] == "delivered"
    assert joined["sections"]["ota_hop_ms"] == 12.0
    assert joined["sections"]["wrap_to_com_out_ms"] == 5.0


def test_sender_rows_alone_report_nothing_delivered_and_no_loss_figure() -> None:
    """Only the receiving peer can say a message arrived.

    Sender gaps are still not loss — the far side may simply not have been
    observed yet — but neither are sender rows a delivery. A link that carried
    nothing used to report `delivered == expected, loss 0%` off its own
    sender-side rows, which is the most dangerous shape a wrong number can take.
    """
    records = [
        {**_record(0, "sent", t_wrap=10.0), "direction": "outbound", "stage": "com_out"},
        {**_record(1, "unobserved"), "direction": "outbound", "stage": "com_out"},
        {**_record(2, "sent", t_wrap=10.2), "direction": "outbound", "stage": "com_out"},
    ]
    summary = summarize_transit_records(records)["topics"]["/x"]
    assert summary["lost"] == 0
    assert summary["loss_pct"] is None
    assert summary["delivered"] == 0
    assert summary["unconfirmed"] == 3


def test_a_receiver_row_is_what_makes_a_message_delivered() -> None:
    records = [
        {**_record(0, "sent", t_wrap=10.0), "direction": "outbound", "stage": "com_out"},
        {**_record(0, "delivered", 12.0, t_wrap=10.0), "direction": "inbound", "stage": "com_in"},
        {**_record(1, "sent", t_wrap=10.1), "direction": "outbound", "stage": "com_out"},
        {**_record(1, "lost"), "direction": "inbound", "stage": "com_in"},
        {**_record(2, "sent", t_wrap=10.2), "direction": "outbound", "stage": "com_out"},
    ]
    summary = summarize_transit_records(records)["topics"]["/x"]
    assert summary["delivered"] == 1
    assert summary["lost"] == 1
    assert summary["unconfirmed"] == 1  # seq 2 was sent and never accounted for
    assert summary["loss_pct"] == 50.0  # of what the receiver could account for
