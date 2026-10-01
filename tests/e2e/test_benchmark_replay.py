# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Docker-backed e2e: benchmark search with a bag-replay session as the load.

Exercises the bag-as-load path end to end — a replay ``--target`` session drives
the link (its own contract publishers, both directions) under generated candidate
profiles, and the per-topic oracle judges the whole contract. Runs in the slow
lane only (``ROSOTACOM_RUN_E2E=1``); the search logic and oracle math are covered
deterministically by the host tests in ``tests/unit``.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_TIMEOUT_S = 900
SMOKE_NETWORK_NAME = "rosotacom-smoke"
REPLAY_SESSION = "15_remote_assist_anonymized_costmap"
CONTRACT_TOPIC = "/topic5"
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(
        os.environ.get("ROSOTACOM_RUN_E2E") != "1",
        reason="Docker-backed E2E tests require ROSOTACOM_RUN_E2E=1.",
    ),
]


def _cleanup_smoke_network() -> None:
    inspect = subprocess.run(
        ["docker", "network", "inspect", SMOKE_NETWORK_NAME, "--format", "{{json .Containers}}"],
        text=True,
        capture_output=True,
        check=False,
    )
    if inspect.returncode == 0 and inspect.stdout.strip():
        try:
            containers = json.loads(inspect.stdout)
        except json.JSONDecodeError:
            containers = {}
        for container_id in containers:
            subprocess.run(["docker", "rm", "-f", container_id], text=True, capture_output=True, check=False)
    subprocess.run(["docker", "network", "rm", SMOKE_NETWORK_NAME], text=True, capture_output=True, check=False)


def _run(command: list[str], *, timeout: int) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            command,
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        _cleanup_smoke_network()
        raise AssertionError(f"Command timed out after {timeout}s: {' '.join(command)}") from exc
    if result.returncode != 0:
        raise AssertionError(
            f"Command failed with {result.returncode}: {' '.join(command)}\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return result


def _benchmark_result_path(stdout: str) -> Path:
    prefix = "Benchmark result saved to "
    paths = [Path(line.removeprefix(prefix)) for line in stdout.splitlines() if line.startswith(prefix)]
    if not paths:
        raise AssertionError(f"Benchmark output did not report a result.json path.\nSTDOUT:\n{stdout}")
    return paths[-1]


@pytest.fixture(scope="module")
def copied_example_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom") / "examples"
    _run([sys.executable, "-m", "rosotacom", "examples", "create", str(project)], timeout=60)
    return project


def test_loss_boundaries_against_costmap_replay(copied_example_project: Path) -> None:
    """✅ loss-boundaries with example 15 (costmap replay) as the load completes."""
    cmd = [
        sys.executable,
        "-m",
        "rosotacom",
        "benchmark",
        "loss-boundaries",
        "--rosotacom-config",
        str(copied_example_project / "rosotacom.yaml"),
        "--target",
        REPLAY_SESSION,
        "--target-type",
        "session",
        # A tiny iteration budget: one bandwidth pair, one sample each side.
        "--axes",
        "bandwidth",
        "--bandwidth-low",
        "500kbit",
        "--bandwidth-high",
        "2mbit",
        "--bandwidth-step",
        "500kbit",
        "--rate-hz",
        "10",
        "--min-duration",
        "8",
        "--min-messages",
        "1",
        "--probe-repeats",
        "1",
        "--good-clean-count",
        "1",
        "--bad-lossy-count",
        "1",
        "--max-latency-ms",
        "2000",
        "--oracle-min-completeness",
        "0.5",
        "--drain-s",
        "2",
        "--rmw",
        "cyclone",
    ]
    result = _run(cmd, timeout=BENCHMARK_TIMEOUT_S)

    result_path = _benchmark_result_path(result.stdout)
    assert result_path.is_file(), f"Benchmark result file does not exist: {result_path}"
    doc = json.loads(result_path.read_text(encoding="utf-8"))

    assert doc["genre"] == "loss-boundaries"
    # The replay load identity is recorded.
    replay = doc["result"]["replay"]
    assert replay["target"]["name"] == REPLAY_SESSION
    assert replay["target"]["type"] == "session"
    assert CONTRACT_TOPIC in (replay["required_topics"] or [])

    # The whole-contract oracle ran: every classified row carries per-topic
    # verdicts, and the costmap contract topic was evaluated.
    rows = doc["result"]["rows"]
    assert rows, "loss-boundaries produced no probe rows"
    evaluated_topics = {
        entry["topic"] for row in rows for sample in row.get("samples", []) for entry in (sample.get("per_topic") or [])
    }
    assert CONTRACT_TOPIC in evaluated_topics, f"contract topic not evaluated; saw {sorted(evaluated_topics)}"
