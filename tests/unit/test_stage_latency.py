# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

CORE = (
    Path(__file__).resolve().parents[2]
    / "src"
    / "rosotacom"
    / "resources"
    / "ws"
    / "ros2src"
    / "com_py"
    / "com_py"
    / "stage_latency_core.py"
)
spec = importlib.util.spec_from_file_location("rosotacom_stage_latency_core", CORE)
assert spec and spec.loader
stage_latency_core = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = stage_latency_core
spec.loader.exec_module(stage_latency_core)


def test_join_stage_timestamps_by_message_index() -> None:
    rows = stage_latency_core.join_stage_timestamps(
        {"/native": [1.0, 2.0], "/wrapped": [1.01, 2.02], "/com": [1.05, 2.08]},
        ["/native", "/wrapped", "/com"],
    )
    assert rows[0]["sections_ms"] == {
        "/native -> /wrapped": 10.0,
        "/wrapped -> /com": 40.0,
    }
    assert rows[1]["sections_ms"]["/native -> /wrapped"] == 20.0
