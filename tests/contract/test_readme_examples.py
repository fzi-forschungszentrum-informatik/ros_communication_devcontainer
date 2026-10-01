# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import argparse
from pathlib import Path

import pytest
import yaml

import rosotacom.cli as cli
from rosotacom.deployment import load_deployment

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def test_readme_documents_current_entrypoints_and_workflow_files() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    assert "rosotacom --version" in readme
    assert "python -m rosotacom --version" in readme
    assert "CONTRIBUTING.md" in readme
    assert "docs/ci.md" in readme


def test_source_checkout_rosotacom_yaml_loads(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ROSOTACOM_PROFILES", str(cli.EXAMPLE_PROJECT_DIR / "profiles.yaml"))
    runtime = cli._load_runtime_config(argparse.Namespace(rosotacom_config=str(PACKAGE_ROOT / "rosotacom.yaml")))

    assert runtime.ros2docker_config == cli.EXAMPLE_PROJECT_DIR / "ros2docker.json"
    assert runtime.session_configs_dir == (cli.EXAMPLE_PROJECT_DIR / "sessions",)
    assert runtime.scenario_configs_dir == (cli.EXAMPLE_PROJECT_DIR / "scenarios",)
    assert runtime.session_instances_dir == PACKAGE_ROOT / "session-instances"
    assert runtime.deployment is None
    assert runtime.profiles_file == cli.EXAMPLE_PROJECT_DIR / "profiles.yaml"
    assert runtime.benchmarks_dir == (PACKAGE_ROOT.parent / "artifacts" / "benchmarks").resolve()


def test_packaged_example_setup_paths_are_relative_to_example_root() -> None:
    setup = yaml.safe_load((cli.EXAMPLE_PROJECT_DIR / "rosotacom.yaml").read_text(encoding="utf-8"))

    assert setup == {
        "ros2docker_config": "ros2docker.json",
        "session_configs_dir": ["sessions"],
        "scenario_configs_dir": ["scenarios"],
        "session_instances_dir": "session-instances",
        "profiles": "profiles.yaml",
    }


def test_packaged_deployment_example_loads_through_public_loader() -> None:
    deployment = load_deployment(cli.EXAMPLE_PROJECT_DIR / "deployment.example.yaml")

    assert deployment is not None
    assert set(deployment.hosts) == {"machine-a", "machine-b"}
    assert deployment.hosts["machine-a"].ssh is None
    assert deployment.hosts["machine-b"].ssh == "robot-b"
    assert deployment.values["example_router"] == "192.0.2.1"


def test_project_setup_rejects_unknown_keys(tmp_path: Path) -> None:
    config = tmp_path / "rosotacom.yaml"
    config.write_text("legacy_inventory: inventory.yaml\n", encoding="utf-8")

    with pytest.raises(RuntimeError, match="Unsupported rosotacom.yaml keys"):
        cli._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))


def test_packaged_example_project_contains_documented_heartbeat_session() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    assert "`1_heartbeat`" in readme
    assert "rosotacom scenario start 2_native_chatter" in readme
    assert "rosotacom smoke 2_native_chatter --interactive" in readme
    assert "rosotacom smoke 14_remote_assist_anonymized" in readme
    assert "rosotacom smoke 15_remote_assist_anonymized_costmap" in readme
    assert "rosotacom smoke 16_remote_assist_anonymized_camera" in readme
    assert "17_synthetic_camera_quality" in readme
    assert "rosotacom ota-smoke 2_native_chatter \\" in readme
    assert "--peer-ssh b=robot-b" in readme
    assert (cli.EXAMPLE_PROJECT_DIR / "sessions" / "1_heartbeat" / "session-definition.yaml").is_file()
    assert (cli.EXAMPLE_PROJECT_DIR / "scenarios" / "2_native_chatter" / "scenario-definition.yaml").is_file()
    assert (cli.EXAMPLE_PROJECT_DIR / "scripts" / "1_heartbeat" / "run_machine_a.sh").is_file()


def test_packaged_native_chatter_scenario_and_application_configs_load() -> None:
    runtime = cli._load_runtime_config(
        argparse.Namespace(rosotacom_config=str(cli.EXAMPLE_PROJECT_DIR / "rosotacom.yaml"))
    )
    resolved = cli._resolve_scenario("2_native_chatter", runtime)

    definition = cli._load_scenario_definition(resolved)

    assert definition.session == "2_native_chatter"
    assert set(definition.applications) == {"a", "b"}
