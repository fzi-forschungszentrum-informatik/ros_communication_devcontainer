# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unit tests for the cross-environment benchmark delta (harness #36).

Fixtures, not runs. The point of the tool is what it says when two sets differ,
and a real pair of environments is the one input that cannot be made to differ
on purpose.
"""

from __future__ import annotations

import json
from pathlib import Path

from rosotacom.benchmark_delta import collect, compare, interesting_metrics, render_markdown


def write_result(
    directory: Path,
    *,
    genre: str = "capacity",
    profile: str = "cellular-4g-typical",
    rmw: str = "cyclone",
    session: str = "bench_1_1_capacity",
    result: dict | None = None,
    command: str = "rosotacom benchmark capacity",
) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    payload = {
        "schema_version": 1,
        "genre": genre,
        "created_at": "2026-08-12T09:00:00",
        "context": {
            "command": command,
            "profile": {"name": profile},
            "rmw": {"requested": rmw},
            "session": {"name": session, "rmw": rmw},
            "paths": {"profiles_file": "profiles/benchmark-profiles.yaml"},
        },
        "result": result if result is not None else {"capacity": 43000, "slice": {"loss_pct": 0.5}},
        "verdict": {"passed": True, "status": "capacity_found"},
    }
    path = directory / "result.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def test_metrics_are_selected_not_swept_up() -> None:
    metrics = interesting_metrics(
        {
            "capacity": 43000,
            "slice": {"loss_pct": 0.5, "knob": "size"},
            "run_id": 17,
            "passed": True,
        }
    )
    assert metrics == {"capacity": 43000.0, "slice.loss_pct": 0.5}
    assert "run_id" not in metrics, "an id's delta means nothing"
    assert "passed" not in metrics, "a bool delta is a category error dressed as -1"


def test_matching_rows_produce_a_delta(tmp_path: Path) -> None:
    write_result(tmp_path / "emu" / "run1", result={"capacity": 43000, "slice": {"loss_pct": 0.5}})
    write_result(tmp_path / "ota" / "run1", result={"capacity": 21000, "slice": {"loss_pct": 2.0}})

    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )

    assert report["counts"]["matched"] == 1
    metrics = {m["metric"]: m for m in report["rows"][0]["metrics"]}
    assert metrics["capacity"]["delta"] == -22000
    assert metrics["capacity"]["relative"] < 0
    assert metrics["slice.loss_pct"]["delta"] == 1.5


def test_a_row_on_one_side_only_is_never_a_delta(tmp_path: Path) -> None:
    """ "It got faster" and "it did not run" must not look the same."""
    write_result(tmp_path / "emu" / "a", session="row_a")
    write_result(tmp_path / "emu" / "b", session="row_b")
    write_result(tmp_path / "ota" / "a", session="row_a")

    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )

    assert report["counts"]["matched"] == 1
    assert report["counts"]["only_emulated"] == 1
    assert any("row_b" in name for name in report["unmatched"]["emulated"])


def test_rows_differing_only_in_profile_do_not_pair(tmp_path: Path) -> None:
    """A tight profile is not a delta against a nominal one."""
    write_result(tmp_path / "emu" / "a", profile="cellular-4g-typical")
    write_result(tmp_path / "ota" / "a", profile="cellular-4g-degraded")

    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )
    assert report["counts"]["matched"] == 0


def test_rows_differing_only_in_rmw_do_not_pair(tmp_path: Path) -> None:
    write_result(tmp_path / "emu" / "a", rmw="cyclone")
    write_result(tmp_path / "ota" / "a", rmw="fastdds")

    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )
    assert report["counts"]["matched"] == 0


def test_the_report_says_it_is_monitor_only(tmp_path: Path) -> None:
    write_result(tmp_path / "emu" / "a")
    write_result(tmp_path / "ota" / "a")
    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )

    assert report["monitor_only"] is True
    markdown = render_markdown(report)
    assert "Monitor-only" in markdown.splitlines()[2]
    assert "never a verdict" in markdown


def test_the_report_carries_its_own_provenance(tmp_path: Path) -> None:
    write_result(tmp_path / "emu" / "a", command="rosotacom benchmark capacity --profile nominal")
    write_result(tmp_path / "ota" / "a", command="rosotacom ota-benchmark row --row s3")
    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )
    markdown = render_markdown(report)
    assert "--profile nominal" in markdown
    assert "ota-benchmark row --row s3" in markdown


def test_a_metric_present_on_one_side_only_is_flagged(tmp_path: Path) -> None:
    write_result(tmp_path / "emu" / "a", result={"capacity": 1000, "slice": {"loss_pct": 0.0}})
    write_result(tmp_path / "ota" / "a", result={"capacity": 900})
    report = compare(
        collect([tmp_path / "emu"]),
        collect([tmp_path / "ota"]),
        label_reference="emulated",
        label_measured="ota",
    )
    metrics = {m["metric"]: m for m in report["rows"][0]["metrics"]}
    assert metrics["slice.loss_pct"]["delta"] is None
    assert metrics["slice.loss_pct"]["note"] == "present on one side only"


def test_non_result_json_files_are_ignored(tmp_path: Path) -> None:
    write_result(tmp_path / "emu" / "a")
    (tmp_path / "emu" / "noise").mkdir()
    (tmp_path / "emu" / "noise" / "result.json").write_text('{"not": "a benchmark"}', encoding="utf-8")

    assert len(collect([tmp_path / "emu"])) == 1
