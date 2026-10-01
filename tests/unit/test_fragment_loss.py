# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The size law: fit, link-event layer, block bootstrap and the published example."""

from __future__ import annotations

import gzip
import json
import math
import random
from pathlib import Path

import yaml

import rosotacom.fragment_loss as fl

ROOT = Path(__file__).resolve().parents[2]
FIELD = ROOT / "field-profiles" / "2026-08"


def _message(t_s: float, nfrag: int = 1, lost: bool = False, topic: str = "/a") -> fl.Message:
    return fl.Message(topic=topic, t_s=t_s, size_bytes=nfrag * 1024, nfrag=nfrag, lost=lost)


def test_fit_recovers_the_rate_that_generated_the_counts():
    p = 0.0013
    table = {n: (1_000_000, round(1_000_000 * (1 - (1 - p) ** n))) for n in (1, 2, 5, 13, 34)}
    assert math.isclose(fl.fit_rate(table), p, rel_tol=1e-3)


def test_fit_is_undefined_without_losses():
    assert math.isnan(fl.fit_rate({1: (500, 0), 8: (40, 0)}))


def test_three_topics_losing_in_one_second_make_a_padded_link_event():
    messages = [_message(10.2, lost=True, topic=t) for t in ("/a", "/b", "/c")]
    messages += [_message(20.5, lost=True, topic=t) for t in ("/a", "/b")]
    assert fl.link_event_seconds(messages) == {9, 10, 11}
    steady, event, seconds = fl.split_layers([*messages, _message(9.9), _message(12.0)])
    assert seconds == 3
    assert len(event) == 4 and len(steady) == 3


def test_joined_records_give_a_lost_message_the_senders_size_and_time():
    records = [
        {
            "kind": "transit",
            "source": "v",
            "topic": "/cam",
            "epoch": 0,
            "seq": 1,
            "status": "sent",
            "size_bytes": 3000,
            "t_wrap": 5.25,
        },
        {
            "kind": "transit",
            "source": "v",
            "topic": "/cam",
            "epoch": 0,
            "seq": 1,
            "status": "lost",
            "size_bytes": None,
            "t_wrap": None,
        },
        {
            "kind": "transit",
            "source": "v",
            "topic": "/cam",
            "epoch": 0,
            "seq": 2,
            "status": "delivered",
            "size_bytes": 100,
            "t_wrap": 5.35,
        },
        {
            "kind": "transit",
            "source": "v",
            "topic": "/cam",
            "epoch": 0,
            "seq": 3,
            "status": "lost",
            "size_bytes": None,
            "t_wrap": None,
        },
        {
            "kind": "transit",
            "source": "c",
            "topic": "/cmd",
            "epoch": 0,
            "seq": 1,
            "status": "delivered",
            "size_bytes": 64,
            "t_wrap": 5.0,
        },
    ]
    messages, skipped = fl.messages_from_records(records, fragment_bytes=1024, source="v")
    assert skipped == 1  # seq 3: a verdict without the sender's size or time
    assert [(m.nfrag, m.lost, m.t_s) for m in messages] == [(3, True, 5.25), (1, False, 5.35)]


def test_records_are_read_from_gzip_and_non_transit_lines_ignored(tmp_path):
    path = tmp_path / "records.jsonl.gz"
    lines = [{"kind": "status"}, {"kind": "transit", "seq": 1, "topic": "/a", "status": "delivered"}]
    with gzip.open(path, "wt", encoding="utf-8") as handle:
        handle.write("\n".join(json.dumps(line) for line in lines) + "\n")
    assert [r["seq"] for r in fl.read_records([path])] == [1]


def test_block_bootstrap_brackets_the_rate_and_needs_enough_blocks():
    rng = random.Random(1)
    p = 0.01
    messages = []
    for i in range(20 * 60 * 10):
        n = rng.choice((1, 4, 16))
        messages.append(_message(i / 10, nfrag=n, lost=rng.random() < 1 - (1 - p) ** n))
    rate = fl.fit_rate(fl.counts(messages))
    lo, hi = fl.bootstrap_interval(messages, draws=200)
    assert lo < rate < hi
    assert lo < p < hi
    few_blocks = [m for m in messages if m.t_s < 5 * 60]
    assert all(math.isnan(v) for v in fl.bootstrap_interval(few_blocks, draws=50))


def test_command_line_prints_json(tmp_path, capsys):
    path = tmp_path / "records.jsonl"
    rows = []
    for seq in range(400):
        lost = seq % 40 == 0
        rows.append(
            {
                "kind": "transit",
                "source": "v",
                "topic": "/a",
                "epoch": 0,
                "seq": seq,
                "status": "lost" if lost else "delivered",
                "size_bytes": 2048,
                "t_wrap": seq * 0.5,
            }
        )
    path.write_text("\n".join(json.dumps(r) for r in rows) + "\n")
    assert fl.main([str(path), "--fragment-bytes", "1024", "--draws", "20", "--json"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["lost"] == 10 and result["event_seconds"] == 0
    assert math.isclose(result["steady"]["rate_pct"], 100 * (1 - math.sqrt(1 - 10 / 400)), rel_tol=1e-3)


def test_published_r6c_records_reproduce_the_papers_rates():
    """0.143 % steady and 1.03 % in link events, as the paper and `r6c-steady` report."""
    messages, skipped = fl.messages_from_records(
        fl.read_records([FIELD / "transit" / "r6c.jsonl.gz"]), fragment_bytes=1024
    )
    assert (len(messages), sum(m.lost for m in messages), skipped) == (78909, 817, 0)
    steady, event, seconds = fl.split_layers(messages)
    assert seconds == 109
    p_steady = 100 * fl.fit_rate(fl.counts(steady))
    p_event = 100 * fl.fit_rate(fl.counts(event))
    assert round(p_steady, 3) == 0.143
    assert round(p_event, 2) == 1.03
    profiles = yaml.safe_load((FIELD / "netem_profiles.yaml").read_text())["profiles"]
    assert profiles["r6c-steady"]["uplink"]["loss"] == f"{p_steady:.3f}%"
