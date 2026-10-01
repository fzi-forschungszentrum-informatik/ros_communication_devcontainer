# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Slow-lane E2E for ``rosotacom benchmark ab`` (#22).

Two configs that differ only in the OTA **QoS reliability** on the synthetic load
topic, run under an emulated lossy link. QoS is the knob that keeps the topic
name identical across configs (throttle/drop republish to a renamed topic via
`topic_tools`, so they are not 1:1 comparable), which is exactly what an A/B
verdict needs. The baseline uses `best_effort`; the candidate uses `reliable`,
which recovers dropped samples by retransmission and therefore must never have
*worse* completeness than best_effort under loss.

The point is the end-to-end proof the pure-logic unit tests cannot give: on a
real graph, under real shaping, `benchmark ab` materializes both configs, runs
them interleaved, measures the *same* topic for both, and renders a coherent
verdict. Docker-backed, so gated behind ``ROSOTACOM_RUN_E2E=1`` like the other
e2e lanes.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
BENCHMARK_TIMEOUT_S = 900
SMOKE_NETWORK_NAME = "rosotacom-smoke"
AB_PROFILE = "ab-loss-ci"

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


def _ab_config(reliability: str) -> dict[str, object]:
    """A bench_1_1-style single-stream config; only the OTA QoS reliability varies.

    QoS is pub/sub metadata, so the delivered topic stays ``/bench_capacity`` for
    every config — the property the A/B topic-by-topic comparison relies on.
    """
    return {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 50}, "b": {"domain_id": 51}},
        "shared": {
            "use_status_overview": True,
            "ota_domain_id": 52,
            "rmw": "cyclone",
            "qos": {
                "defaults": {"depth": 1},
                "for_role": {
                    "ota_sub": {"depth": 1, "reliability": reliability},
                    "ota_pub": {"depth": 1, "reliability": reliability},
                },
            },
        },
        "topics": {
            "a_to_b": [
                {
                    "topic": "/bench_capacity",
                    "type": "com_msgs/msg/SizedPayload",
                    "processing": {"use_ota_wrapper": True},
                }
            ]
        },
    }


@pytest.fixture(scope="session")
def ab_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom") / "examples"
    _run([sys.executable, "-m", "rosotacom", "examples", "create", str(project)], timeout=60)

    # Ample bandwidth (no congestion) with a plain per-packet loss — deterministic
    # enough for CI without a netem seed (GitHub tc rejects `seed`), and it is the
    # loss that best_effort cannot recover but reliable can.
    (project / "profiles.yaml").write_text(
        f"profiles:\n  {AB_PROFILE}:\n    uplink:   {{ rate: 8mbit, loss: 20% }}\n    downlink: {{ rate: 8mbit }}\n",
        encoding="utf-8",
    )
    baseline = project / "ab-configs" / "baseline"
    candidate = project / "ab-configs" / "reliable"
    baseline.mkdir(parents=True)
    candidate.mkdir(parents=True)
    (baseline / "session-definition.yaml").write_text(yaml.safe_dump(_ab_config("best_effort")), encoding="utf-8")
    (candidate / "session-definition.yaml").write_text(yaml.safe_dump(_ab_config("reliable")), encoding="utf-8")
    return project


def _result_path(stdout: str) -> Path:
    prefix = "Benchmark result saved to "
    paths = [Path(line.removeprefix(prefix)) for line in stdout.splitlines() if line.startswith(prefix)]
    if not paths:
        raise AssertionError(f"benchmark ab did not report a result.json path.\nSTDOUT:\n{stdout}")
    return paths[-1]


def test_benchmark_ab_reliability_verdict(ab_project: Path) -> None:
    """End-to-end: both configs are measured on the same topic and the reliable
    candidate never regresses completeness vs best_effort under loss."""
    result = _run(
        [
            sys.executable,
            "-m",
            "rosotacom",
            "benchmark",
            "ab",
            "--rosotacom-config",
            str(ab_project / "rosotacom.yaml"),
            "--profile",
            AB_PROFILE,
            "--baseline",
            str(ab_project / "ab-configs" / "baseline"),
            "--candidate",
            f"reliable={ab_project / 'ab-configs' / 'reliable'}",
            "--size",
            "18000",
            "--rate-hz",
            "20",
            "--duration",
            "15",
            "--repeats",
            "1",
            "--rmw",
            "cyclone",
        ],
        timeout=BENCHMARK_TIMEOUT_S,
    )

    doc = json.loads(_result_path(result.stdout).read_text(encoding="utf-8"))
    assert doc["genre"] == "ab"
    assert len(doc["measurements"]["runs"]) == 2  # baseline + candidate, 1 repeat each
    assert len(doc["result"]["candidates"]) == 1

    candidate = doc["result"]["candidates"][0]
    assert candidate["config"] == "reliable"
    # The core end-to-end guarantee: the reliable run kept the same topic as the
    # baseline (no topic-rename), so both were measured and none dropped.
    assert candidate["dropped_topics"] == [], candidate
    measured = [
        cell for cell in candidate["cells"] if cell["metric"] == "completeness_pct" and cell["verdict"] is not None
    ]
    assert measured, f"completeness was not measured for both configs: {candidate['cells']}"
    # Directional claim that holds regardless of how effective retransmission is
    # on this runner: reliable recovers losses, so it never *regresses*
    # completeness vs best_effort (it improves it or leaves it unchanged).
    completeness_regressed = [pair for pair in candidate["regressed"] if pair[1] == "completeness_pct"]
    assert not completeness_regressed, f"reliable regressed completeness vs best_effort: {candidate}"
