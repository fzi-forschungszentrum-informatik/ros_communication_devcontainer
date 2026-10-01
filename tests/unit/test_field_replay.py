# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Failure handling and numerical contracts of the published frame replay."""

from __future__ import annotations

import argparse
import importlib.util
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("field_replay", ROOT / "field-profiles/2026-08/replay/replay.py")
assert spec and spec.loader
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def test_docker_wait_checks_container_exit_not_only_cli_exit(monkeypatch):
    def run(cmd, **kwargs):
        return subprocess.CompletedProcess(cmd, 0, "23\n" if cmd[1] == "wait" else "sender failed", "")

    monkeypatch.setattr(replay, "run", run)
    with pytest.raises(RuntimeError, match="exited 23"):
        replay.wait_container("test-sender", 1)


def test_existing_results_are_not_mixed_into_a_new_experiment(tmp_path):
    (tmp_path / "result_none_1.json").write_text("{}")
    args = argparse.Namespace(
        work=tmp_path,
        duration=1,
        repeat=1,
        parallel=1,
        plan=replay.HERE.parent / "r3_frame_plan.csv",
        scenario=["none"],
        key="r3",
    )
    with pytest.raises(ValueError, match="must be empty"):
        replay.mode_orchestrate(args)


def test_published_plan_and_profile_prediction():
    plan = replay.read_plan(replay.HERE.parent / "r3_frame_plan.csv", 300)
    assert len(plan) == 3001
    profiles = replay.load_profiles()
    assert len(profiles) == 22
    assert replay.expectation("steady", profiles, "r3", plan)["overall_loss_pct"] == pytest.approx(1.06, abs=0.01)
    assert replay.expectation("events", profiles, "r3", plan)["overall_loss_pct"] == pytest.approx(2.13, abs=0.01)


def test_empty_frame_class_does_not_crash_short_smoke():
    result = replay.loss_pct([(0, 1188, True)], [True])
    assert result["overall_loss_pct"] == 0
    assert result["delta_loss_pct"] is None


def test_invalid_frame_plan_is_rejected(tmp_path):
    path = tmp_path / "plan.csv"
    path.write_text("t_s,size,kf\n0,0,1\n")
    with pytest.raises(ValueError, match="positive sizes"):
        replay.read_plan(path, 1)
