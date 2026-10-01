#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Generate the heartbeat RMW test-config matrix.

The matrix is intentionally data-first: adding a transport combination should be
one new entry in RMW_CASES, not another hand-maintained session directory.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml

MATRIX_DIR = Path(__file__).resolve().parent / "rmw_matrix"

HEARTBEAT_EXPECT: dict[str, Any] = {
    "hz": {"min": 5, "max": 20},
    "latency_ms": {"max": 1000},
    "loss_pct": {"max": 5},
}

RMW_CASES: list[dict[str, Any]] = [
    {
        "name": "1_heartbeat_fastdds",
        "rmw": {
            "local": {"fastdds": {"config": "fastdds_unicast.xml"}},
            "ota": {"fastdds": {"config": "fastdds_unicast.xml"}},
        },
    },
    {
        # What `rmw: fastdds` means when a session names nothing else: the OTA
        # side defaults to `fastdds_tuned.xml`. The shipped default is the one
        # combination a matrix must not leave to a session author to discover.
        "name": "1_heartbeat_fastdds-default",
        "rmw": "fastdds",
    },
    {
        "name": "1_heartbeat_cyclone-ota",
        "rmw": {
            "local": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
            "ota": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
        },
    },
    {
        "name": "1_heartbeat_zen-endpoints",
        "rmw": {"local": "zenoh", "ota": "zenoh_connect_endpoints"},
    },
    {
        "name": "1_heartbeat_cyclone-local_zenoh-ros2dds-ota",
        "rmw": {"local": "cyclone", "ota": "zenoh_ros2dds"},
    },
    {
        "name": "1_heartbeat_cyclone-ota-tuned",
        "local_check": False,
        "rmw": {
            "local": "cyclone",
            "ota": {"cyclone": {"config": "cyclonedds_tuned.xml"}},
        },
    },
]


def _base_config() -> dict[str, Any]:
    return {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {
            "a": {"domain_id": 46},
            "b": {"domain_id": 47},
        },
        "shared": {
            "use_heartbeat": True,
            "use_status_overview": True,
            "heartbeat": {"expect": deepcopy(HEARTBEAT_EXPECT)},
            "ota_domain_id": 48,
        },
    }


def session_config(case: dict[str, Any]) -> dict[str, Any]:
    cfg = _base_config()
    if case.get("local_check") is False:
        cfg.pop("peer_settings")
        cfg["local_check"] = False
    cfg["shared"]["rmw"] = deepcopy(case["rmw"])
    return cfg


def render_session(case: dict[str, Any]) -> str:
    return yaml.safe_dump(session_config(case), sort_keys=False, default_flow_style=False)


def write_matrix() -> None:
    if MATRIX_DIR.exists():
        shutil.rmtree(MATRIX_DIR)
    MATRIX_DIR.mkdir(parents=True)
    for case in RMW_CASES:
        session_dir = MATRIX_DIR / case["name"]
        session_dir.mkdir(parents=True)
        (session_dir / "session-definition.yaml").write_text(render_session(case), encoding="utf-8")


def check_matrix() -> list[str]:
    problems: list[str] = []
    expected_names = {case["name"] for case in RMW_CASES}
    actual_names = {p.name for p in MATRIX_DIR.iterdir() if p.is_dir()} if MATRIX_DIR.is_dir() else set()
    for missing in sorted(expected_names - actual_names):
        problems.append(f"missing generated session: {missing}")
    for extra in sorted(actual_names - expected_names):
        problems.append(f"unexpected generated session: {extra}")
    for case in RMW_CASES:
        path = MATRIX_DIR / case["name"] / "session-definition.yaml"
        expected = render_session(case)
        if path.exists() and path.read_text(encoding="utf-8") != expected:
            problems.append(f"out of date: {path.relative_to(MATRIX_DIR.parent)}")
    return problems


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="Fail if generated files are not up to date.")
    args = parser.parse_args(argv)
    if args.check:
        problems = check_matrix()
        if problems:
            print("RMW matrix is not up to date:", file=sys.stderr)
            for problem in problems:
                print(f"  - {problem}", file=sys.stderr)
            return 1
        return 0
    write_matrix()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
