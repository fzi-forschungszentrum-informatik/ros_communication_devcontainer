# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Public field examples contain only their declared measurement metadata."""

import csv
import gzip
import json
import math
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FIELD = ROOT / "field-profiles" / "2026-08"
FIELDS = {"kind", "source", "target", "topic", "epoch", "seq", "status", "size_bytes", "t_wrap"}
TOPICS = {
    "/sensors/camera/front_medium/resized/camera_info",
    "/sensors/camera/front_medium/resized/image_rect_color/compressed",
    "/costmap/costmap",
    "/heartbeat",
    "/visualization/planning/pso/submitted_visualization",
    "/can/twist",
    "/can/steering_angle",
    "/visualization/maneuver/announcements_barriers",
    "/visualization/map/sliding_driving_area",
    "/mission/progress",
}


def test_r6c_contains_only_public_metadata():
    counts: Counter[str] = Counter()
    topics = set()
    with gzip.open(FIELD / "transit/r6c.jsonl.gz", "rt") as handle:
        for line in handle:
            row = json.loads(line)
            assert set(row) == FIELDS
            assert row["kind"] == "transit"
            assert (row["source"], row["target"], row["epoch"]) == ("vehicle", "center", 0)
            assert row["topic"] in TOPICS
            assert row["status"] in {"delivered", "lost"}
            assert type(row["seq"]) is int and 0 <= row["seq"] < 100_000
            assert type(row["size_bytes"]) is int and 0 < row["size_bytes"] < 100_000
            assert type(row["t_wrap"]) is float and 0 <= row["t_wrap"] <= 993
            counts[row["status"]] += 1
            topics.add(row["topic"])
    assert counts == {"delivered": 78092, "lost": 817}
    assert topics == TOPICS


def test_frame_plan_has_no_picture_payload():
    with (FIELD / "replay/r3_frame_plan.csv").open() as handle:
        reader = csv.DictReader(handle)
        assert reader.fieldnames == ["t_s", "size", "kf"]
        rows = list(reader)
    assert len(rows) == 12016
    previous = -math.inf
    for row in rows:
        assert set(row) == {"t_s", "size", "kf"}
        stamp = float(row["t_s"])
        assert math.isfinite(stamp) and stamp >= previous
        assert 0 < int(row["size"]) < 1_000_000
        assert row["kf"] in {"True", "False"}
        previous = stamp
