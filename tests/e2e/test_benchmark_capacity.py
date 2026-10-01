# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

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
# The committed public gate profile (performance bands reference): a 1 Mbit/s uplink bottleneck.
CAPACITY_PROFILE = "gate-tight"
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
    result_paths = [Path(line.removeprefix(prefix)) for line in stdout.splitlines() if line.startswith(prefix)]
    if not result_paths:
        raise AssertionError(f"Benchmark output did not report a result.json path.\nSTDOUT:\n{stdout}")
    return result_paths[-1]


def _assert_capacity_result(
    stdout: str,
    *,
    expected_capacity: int | None,
    expected_probe_value: int,
    expected_passed: bool,
) -> None:
    result_path = _benchmark_result_path(stdout)
    assert result_path.is_file(), f"Benchmark result file does not exist: {result_path}"
    doc = json.loads(result_path.read_text(encoding="utf-8"))

    assert doc["genre"] == "capacity"
    assert doc["result"]["slice"]["knob"] == "size"
    assert doc["result"]["capacity"] == expected_capacity
    assert doc["verdict"]["passed"] is expected_passed
    probes = doc["measurements"]["probes"]
    assert len(probes) == 1
    assert probes[0]["value"] == expected_probe_value
    assert probes[0]["passed"] is expected_passed
    assert probes[0]["load"]["offered_bandwidth_bps"] is not None
    if expected_passed:
        assert probes[0]["topics"]


@pytest.fixture(scope="session")
def copied_example_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """The examples project as shipped — the gate profiles are committed, not
    synthesized here, so these tests exercise exactly what the gate lanes run."""
    project = tmp_path_factory.mktemp("rosotacom") / "examples"

    _run([sys.executable, "-m", "rosotacom", "examples", "create", str(project)], timeout=60)
    return project


def test_benchmark_capacity_good_case(copied_example_project: Path) -> None:
    """✅ Good case (1B@20Hz, --max-loss 30): PASS"""
    cmd = [
        sys.executable,
        "-m",
        "rosotacom",
        "benchmark",
        "capacity",
        "--rosotacom-config",
        str(copied_example_project / "rosotacom.yaml"),
        "--profile",
        CAPACITY_PROFILE,
        "--knob",
        "size",
        "--low",
        "1",
        "--high",
        "1",
        "--max-loss",
        "30",
        "--max-latency-ms",
        "1000",
        "--duration",
        "10",
        "--repeats",
        "1",
        "--rmw",
        "cyclone",
    ]
    result = _run(cmd, timeout=BENCHMARK_TIMEOUT_S)
    _assert_capacity_result(result.stdout, expected_capacity=1, expected_probe_value=1, expected_passed=True)


def test_benchmark_capacity_bad_case(copied_example_project: Path) -> None:
    """✅ Bad case (18KB@20Hz = 2.88Mbit/s on 1Mbit/s, --max-loss 30): FAIL"""
    cmd = [
        sys.executable,
        "-m",
        "rosotacom",
        "benchmark",
        "capacity",
        "--rosotacom-config",
        str(copied_example_project / "rosotacom.yaml"),
        "--profile",
        CAPACITY_PROFILE,
        "--knob",
        "size",
        "--low",
        "18000",
        "--high",
        "18000",
        "--max-loss",
        "30",
        "--max-latency-ms",
        "1000",
        "--duration",
        "10",
        "--repeats",
        "1",
        "--rmw",
        "cyclone",
    ]
    result = _run(cmd, timeout=BENCHMARK_TIMEOUT_S)
    _assert_capacity_result(result.stdout, expected_capacity=None, expected_probe_value=18000, expected_passed=False)


def test_benchmark_probe_camera_load(copied_example_project: Path) -> None:
    """✅ Probe with camera load: GOP sequence + seeded interval jitter"""
    cmd = [
        sys.executable,
        "-m",
        "rosotacom",
        "benchmark",
        "probe",
        "--rosotacom-config",
        str(copied_example_project / "rosotacom.yaml"),
        "--profile",
        CAPACITY_PROFILE,
        "--size-pattern",
        "1x43KB+1x3KB+3x4KB",
        "--interval-jitter-ms",
        "20.0",
        "--interval-jitter-seed",
        "42",
        "--duration",
        "5",
        "--repeats",
        "1",
        "--rmw",
        "cyclone",
    ]
    result = _run(cmd, timeout=BENCHMARK_TIMEOUT_S)
    result_path = _benchmark_result_path(result.stdout)
    assert result_path.is_file()
    doc = json.loads(result_path.read_text(encoding="utf-8"))

    # Assert load configuration is correctly propagated
    load_params = doc["configuration"]["load"]["parameters"]
    assert load_params["sizes"] == [43000, 3000, 4000, 4000, 4000]
    assert load_params["interval_jitter_ms"] == 20.0
    assert load_params["interval_jitter_seed"] == 42


def _merge_gate_row_ids() -> list[str]:
    from rosotacom.benched_set import load_registry, rows_for_lane

    return [row.id for row in rows_for_lane(load_registry(), "merge-gate")]


@pytest.mark.parametrize("row_id", _merge_gate_row_ids())
def test_merge_gate_row_is_band_asserted(row_id: str, copied_example_project: Path, tmp_path: Path) -> None:
    """The performance bands reference merge-gate row: run the benched row end-to-end and assert
    the committed two-sided band. REGRESSED and IMPROVED both fail — the
    IMPROVED failure text carries the exact ratchet command to bank it in this
    same change. On a host that is not the calibrated runner class the compare
    refuses by design; that principled refusal is a skip here (the gate lanes
    always run on the calibrated class)."""
    verdict_path = tmp_path / f"verdict-{row_id}.json"
    cmd = [
        sys.executable,
        "-m",
        "rosotacom",
        "benchmark",
        "row",
        row_id,
        "--rosotacom-config",
        str(copied_example_project / "rosotacom.yaml"),
        "--budgets",
        str(PACKAGE_ROOT / "budgets.jsonl"),
        "--artifacts-dir",
        str(tmp_path / "gate-artifacts"),
        "--verdict-file",
        str(verdict_path),
    ]
    try:
        result = subprocess.run(
            cmd, cwd=PACKAGE_ROOT, text=True, capture_output=True, timeout=BENCHMARK_TIMEOUT_S, check=False
        )
    except subprocess.TimeoutExpired as exc:
        _cleanup_smoke_network()
        raise AssertionError(f"Benched row {row_id} timed out after {BENCHMARK_TIMEOUT_S}s") from exc

    output = f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
    if result.returncode == 1 and "REFUSED" in result.stdout and "runner class" in result.stdout:
        if os.environ.get("RUNNER_ENVIRONMENT") != "github-hosted":
            pytest.skip(f"bands are calibrated for another runner class; refusal is by design:\n{result.stdout}")
        raise AssertionError(f"gate refused on its own runner class — recalibrate the bands.\n{output}")
    if result.returncode == 2:
        raise AssertionError(
            f"benched row {row_id} IMPROVED beyond its band — bank it with the printed ratchet "
            f"command and commit budgets.jsonl in this same change.\n{output}"
        )
    assert result.returncode == 0, f"benched row {row_id} failed its band assert.\n{output}"

    verdict = json.loads(verdict_path.read_text(encoding="utf-8"))
    assert verdict["verdict"] == "WITHIN"
    assert verdict["row"] == row_id
    assert verdict["metrics"], "a gated row must record its banded metrics"
