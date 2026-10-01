# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
SESSION_INSTANCES_DIR = PACKAGE_ROOT / "session-instances"
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(
        os.environ.get("ROSOTACOM_RUN_E2E") != "1",
        reason="Docker-backed E2E tests require ROSOTACOM_RUN_E2E=1.",
    ),
]


def _run(command: list[str], *, timeout: int) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        command,
        cwd=PACKAGE_ROOT,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )
    if result.returncode != 0:
        raise AssertionError(
            f"Command failed with {result.returncode}: {' '.join(command)}\n"
            f"STDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return result


def _artifact_dir(stdout: str) -> Path:
    matches = re.findall(r"Smoke artifacts: (.+)", stdout)
    assert matches, f"no 'Smoke artifacts:' line in smoke output:\n{stdout}"
    return Path(matches[-1].strip())


@pytest.fixture(scope="session")
def copied_example_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom-quality") / "examples"
    _run([sys.executable, "-m", "rosotacom", "examples", "create", str(project)], timeout=60)
    return project


def test_synthetic_camera_pipeline_records_quality_metrics(copied_example_project: Path, tmp_path: Path) -> None:
    smoke = _run(
        [
            sys.executable,
            "-m",
            "rosotacom",
            "smoke",
            "17_synthetic_camera_quality",
            "--rosotacom-config",
            str(copied_example_project / "rosotacom.yaml"),
            "--session-instances-dir",
            str(SESSION_INSTANCES_DIR),
            "--local-ip",
            "127.0.0.1",
        ],
        timeout=900,
    )
    artifact_dir = _artifact_dir(smoke.stdout)
    reference_bag = next((artifact_dir / "logs" / "b" / "metrics").glob("stages_*"))
    degraded_bag = next((artifact_dir / "logs" / "a" / "metrics").glob("stages_*"))
    report_path = tmp_path / "videoquality.json"

    quality = _run(
        [
            sys.executable,
            "-m",
            "rosotacom",
            "videoquality",
            str(reference_bag),
            str(degraded_bag),
            "--ref-topic",
            "/camera/image",
            "--degraded-topic",
            "/camera/image/ffmpeg/raw",
            "--align",
            "index",
            "--out",
            str(report_path),
            "--min-mean-psnr",
            "20",
            "--min-mean-ssim",
            "0.70",
            "--max-loss-pct",
            "30",
        ],
        timeout=120,
    )

    assert "VIDEOQUALITY OK" in quality.stdout
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["delivery"]["compared_frames"] > 0
    assert report["delivery"]["lost_frames"] <= report["delivery"]["reference_frames"] * 0.30
