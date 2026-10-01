# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""E2E half of the concurrency verification plan (issue #136).

Two independent local smoke tests must run concurrently to completion, and a
second smoke of the *same* target in the same workspace must abort immediately
with a clean conflict message while leaving the running test intact.
"""

from __future__ import annotations

import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(
        os.environ.get("ROSOTACOM_RUN_E2E") != "1",
        reason="Docker-backed E2E tests require ROSOTACOM_RUN_E2E=1.",
    ),
]

SESSION_INSTANCES_DIR = PACKAGE_ROOT / "session-instances"
SMOKE_TIMEOUT_S = 1800
CONFLICT_ABORT_TIMEOUT_S = 120


@pytest.fixture(scope="module")
def example_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom-parallel") / "examples"
    subprocess.run(
        [sys.executable, "-m", "rosotacom", "examples", "create", str(project)],
        cwd=PACKAGE_ROOT,
        text=True,
        capture_output=True,
        timeout=60,
        check=True,
    )
    return project


def _smoke_command(project: Path, session_name: str) -> list[str]:
    return [
        sys.executable,
        "-m",
        "rosotacom",
        "smoke",
        session_name,
        "--rosotacom-config",
        str(project / "rosotacom.yaml"),
        "--session-instances-dir",
        str(SESSION_INSTANCES_DIR),
        "--local-ip",
        "127.0.0.1",
    ]


def _run_smoke(project: Path, session_name: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        _smoke_command(project, session_name),
        cwd=PACKAGE_ROOT,
        text=True,
        capture_output=True,
        timeout=SMOKE_TIMEOUT_S,
        check=False,
    )


def _active_smoke_containers(target_key: str) -> list[str]:
    networks = subprocess.run(
        [
            "docker",
            "network",
            "ls",
            "--filter",
            "label=rosotacom.kind=smoke",
            "--filter",
            f"label=rosotacom.target={target_key}",
            "--format",
            "{{.Name}}",
        ],
        text=True,
        capture_output=True,
        check=False,
    ).stdout.splitlines()
    containers: list[str] = []
    for network in networks:
        if not network:
            continue
        result = subprocess.run(
            ["docker", "ps", "--filter", f"network={network}", "--format", "{{.Names}}"],
            text=True,
            capture_output=True,
            check=False,
        )
        containers.extend(name for name in result.stdout.splitlines() if name)
    return containers


def test_independent_local_smoke_tests_run_in_parallel(example_project: Path) -> None:
    with ThreadPoolExecutor(max_workers=2) as pool:
        heartbeat = pool.submit(_run_smoke, example_project, "1_heartbeat")
        chatter = pool.submit(_run_smoke, example_project, "2_native_chatter")
        heartbeat_result = heartbeat.result()
        chatter_result = chatter.result()

    for name, result in (("1_heartbeat", heartbeat_result), ("2_native_chatter", chatter_result)):
        assert result.returncode == 0, (
            f"parallel smoke {name} failed with {result.returncode}:\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    assert "OK: a->b final heartbeat (/heartbeat_a)" in heartbeat_result.stdout
    assert "OK: b->a final topic (/chatter)" in chatter_result.stdout


def test_second_same_target_smoke_aborts_and_leaves_first_intact(example_project: Path, tmp_path: Path) -> None:
    # Write the long-running first smoke's output to a file so the unread pipe
    # cannot fill up and stall it while this test polls docker.
    first_log = tmp_path / "first-smoke.log"
    with first_log.open("w", encoding="utf-8") as log_handle:
        first = subprocess.Popen(
            _smoke_command(example_project, "1_heartbeat"),
            cwd=PACKAGE_ROOT,
            text=True,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )
    try:
        deadline = time.time() + 600
        while time.time() < deadline:
            if first.poll() is not None:
                raise AssertionError(
                    "first smoke exited before becoming active:\n" + first_log.read_text(encoding="utf-8")
                )
            if _active_smoke_containers("session_1_heartbeat"):
                break
            time.sleep(2)
        else:
            raise AssertionError("first smoke never showed active containers on its labelled network")

        second = subprocess.run(
            _smoke_command(example_project, "1_heartbeat"),
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=CONFLICT_ABORT_TIMEOUT_S,
            check=False,
        )
        assert second.returncode != 0, f"second same-target smoke unexpectedly passed:\n{second.stdout}"
        combined = second.stdout + second.stderr
        assert "already active in this workspace" in combined
        assert "--skip-conflict-check" in combined

        assert first.wait(timeout=SMOKE_TIMEOUT_S) == 0, (
            "first smoke was disturbed by the aborted second run:\n" + first_log.read_text(encoding="utf-8")
        )
        assert "OK: a->b final heartbeat (/heartbeat_a)" in first_log.read_text(encoding="utf-8")
    finally:
        if first.poll() is None:
            first.kill()
            first.communicate()
        # Discovery-based cleanup removes any leftover containers/networks of
        # this target, including from a killed first run.
        subprocess.run(
            [
                sys.executable,
                "-m",
                "rosotacom",
                "smoke",
                "1_heartbeat",
                "--stop",
                "--target-type",
                "session",
                "--rosotacom-config",
                str(example_project / "rosotacom.yaml"),
                "--session-instances-dir",
                str(SESSION_INSTANCES_DIR),
            ],
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=120,
            check=False,
        )
