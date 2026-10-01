# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import re
import struct
import subprocess
import sys
import tarfile
import zlib
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

import rosotacom.cli as rosotacom
from rosotacom.deployment import PeerBinding


@pytest.fixture(autouse=True)
def clear_config_env(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    for key in (
        "ROSOTACOM_CONFIG",
        "ROSOTACOM_ROS2DOCKER_CONFIG",
        "ROSOTACOM_SESSION_CONFIGS_DIR",
        "ROSOTACOM_SCENARIO_CONFIGS_DIR",
        "ROSOTACOM_SESSION_INSTANCES_DIR",
        "ROSOTACOM_DEPLOYMENT",
    ):
        monkeypatch.delenv(key, raising=False)
    # Keep the global user config and the built-in example's tmpfs instances dir
    # off the real $HOME / runtime dir so tests stay hermetic.
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path / ".config"))
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / ".run"))


def test_cwd_rosotacom_yaml_is_auto_discovered(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "cwd"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text("ros2docker_config: ros2docker.json\n", encoding="utf-8")
    monkeypatch.chdir(tmp_path)

    runtime = rosotacom._load_runtime_config(argparse.Namespace())

    assert runtime.rosotacom_config == config
    assert runtime.project_source == "local"
    assert runtime.ros2docker_config == tmp_path / "ros2docker.json"
    assert runtime.session_instances_dir == tmp_path / "session-instances"


def test_global_user_config_project_is_used(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    proj = tmp_path / "proj"
    proj.mkdir()
    (proj / "ros2docker.json").write_text('{"image_name": "global"}\n', encoding="utf-8")
    config = proj / "rosotacom.yaml"
    config.write_text("ros2docker_config: ros2docker.json\n", encoding="utf-8")
    rosotacom._write_user_project(config.resolve())

    work = tmp_path / "work"
    work.mkdir()
    monkeypatch.chdir(work)

    runtime = rosotacom._load_runtime_config(argparse.Namespace())

    assert runtime.rosotacom_config == config.resolve()
    assert runtime.project_source == "global"


def test_builtin_example_used_when_nothing_configured(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    work = tmp_path / "work"
    work.mkdir()
    monkeypatch.chdir(work)

    runtime = rosotacom._load_runtime_config(argparse.Namespace())

    assert runtime.project_source == "built-in"
    # The packaged example is used in place (read-only), not copied into $HOME.
    assert runtime.rosotacom_config == (rosotacom.EXAMPLE_PROJECT_DIR / "rosotacom.yaml").resolve()
    assert runtime.session_configs_dir  # ships runnable sessions
    # Only the writable runtime output is redirected, to tmpfs (no $HOME writes).
    assert runtime.session_instances_dir == rosotacom._builtin_instances_dir()
    assert str(runtime.session_instances_dir).startswith(str(tmp_path / ".run"))


def test_config_set_project_global_roundtrip(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    config = tmp_path / "rosotacom.yaml"
    config.write_text("ros2docker_config: ros2docker.json\n", encoding="utf-8")

    rc = rosotacom.config_command(
        argparse.Namespace(config_action="set", key="project", value=str(config), scope="global")
    )

    assert rc == 0
    assert rosotacom._user_config_project() == config.resolve()

    rosotacom.config_command(argparse.Namespace(config_action="unset", key="project", scope="global"))
    assert rosotacom._user_config_project() is None


def test_config_set_project_shell_prints_export(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    config = tmp_path / "rosotacom.yaml"
    config.write_text("ros2docker_config: ros2docker.json\n", encoding="utf-8")

    rosotacom.config_command(argparse.Namespace(config_action="set", key="project", value=str(config), scope="shell"))

    assert capsys.readouterr().out.strip() == f"export ROSOTACOM_CONFIG={rosotacom.shlex.quote(str(config))}"


def test_rosotacom_yaml_relative_paths_resolve_from_config_dir(tmp_path: Path) -> None:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "test"}\n', encoding="utf-8")
    (tmp_path / "deployment.yaml").write_text(
        "hosts:\n  local: {address: 127.0.0.1, ssh: null}\n",
        encoding="utf-8",
    )
    (tmp_path / "sessions").mkdir()
    (tmp_path / "scenarios").mkdir()
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "\n".join(
            [
                "ros2docker_config: ros2docker.json",
                "session_configs_dir:",
                "  - sessions",
                "scenario_configs_dir:",
                "  - scenarios",
                "session_instances_dir: session-instances",
                "deployment: deployment.yaml",
                "",
            ]
        ),
        encoding="utf-8",
    )

    runtime = rosotacom._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))

    assert runtime.rosotacom_config == config
    assert runtime.ros2docker_config == tmp_path / "ros2docker.json"
    assert runtime.session_configs_dir == (tmp_path / "sessions",)
    assert runtime.scenario_configs_dir == (tmp_path / "scenarios",)
    assert runtime.session_instances_dir == tmp_path / "session-instances"
    assert runtime.deployment == tmp_path / "deployment.yaml"


def test_rosotacom_yaml_rejects_removed_data_dict_key(tmp_path: Path) -> None:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "test"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "ros2docker_config: ros2docker.json\ndata_dict: data_dict.json\n",
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="Unsupported rosotacom.yaml keys.*data_dict"):
        rosotacom._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))


def test_rosotacom_yaml_accepts_com_container_prefix(tmp_path: Path) -> None:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "test"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "ros2docker_config: ros2docker.json\ncom_container_prefix: remote-assist\n",
        encoding="utf-8",
    )

    runtime = rosotacom._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))

    assert runtime.com_container_prefix == "remote-assist"


def test_rosotacom_yaml_rejects_invalid_com_container_prefix(tmp_path: Path) -> None:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "test"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "ros2docker_config: ros2docker.json\ncom_container_prefix: 'has space'\n",
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="com_container_prefix must be a valid docker name"):
        rosotacom._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))


def test_session_definition_rejects_physical_address_field() -> None:
    with pytest.raises(RuntimeError, match="Unsupported keys in peers.a.*address"):
        rosotacom.session_gen._validate_session_template_cfg({"peers": {"a": {"address": "10.0.0.1"}, "b": {}}})


def test_environment_config_is_used_when_no_config_arg(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "env"}\n', encoding="utf-8")
    (tmp_path / "sessions").mkdir()
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "ros2docker_config: ros2docker.json\nsession_configs_dir:\n  - sessions\n",
        encoding="utf-8",
    )
    monkeypatch.setenv("ROSOTACOM_CONFIG", str(config))

    runtime = rosotacom._load_runtime_config(argparse.Namespace())

    assert runtime.rosotacom_config == config
    assert runtime.ros2docker_config == tmp_path / "ros2docker.json"
    assert runtime.session_configs_dir == (tmp_path / "sessions",)
    assert runtime.session_instances_dir == tmp_path / "session-instances"


def test_examples_create_copies_project_and_refuses_overwrite(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    target = tmp_path / "rosotacom_examples"
    args = argparse.Namespace(target=str(target), force=False)

    rosotacom.examples_create_command(args)

    assert (target / "rosotacom.yaml").is_file()
    assert (target / "ros2docker.json").is_file()
    assert (target / "ros2docker.lyrical.json").is_file()
    assert (target / "deployment.example.yaml").is_file()
    assert "session-instances/" in (target / ".gitignore").read_text(encoding="utf-8")
    assert (target / "sessions" / "1_heartbeat" / "session-definition.yaml").is_file()
    assert (target / "scenarios" / "2_native_chatter" / "scenario-definition.yaml").is_file()
    assert not (target / "__init__.py").exists()
    assert "Copied rosotacom examples" in capsys.readouterr().out

    with pytest.raises(RuntimeError, match="Target already exists"):
        rosotacom.examples_create_command(args)

    rosotacom.examples_create_command(argparse.Namespace(target=str(target), force=True))
    assert (target / "scripts" / "1_heartbeat" / "run_machine_a.sh").is_file()


def test_session_name_resolves_through_configured_sessions_dir(tmp_path: Path) -> None:
    session = tmp_path / "sessions" / "1_heartbeat"
    session.mkdir(parents=True)
    (session / "session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    runtime = rosotacom.RuntimeConfig(
        rosotacom_config=tmp_path / "rosotacom.yaml",
        ros2docker_config=rosotacom.DEFAULT_ROS2DOCKER_CONFIG,
        session_configs_dir=(tmp_path / "sessions",),
        deployment=None,
        install_id="test",
    )

    resolved = rosotacom._resolve_session("1_heartbeat", runtime)

    assert resolved.host_dir == session.resolve()
    assert resolved.container_dir == "/session/definitions/1_heartbeat"
    assert resolved.source == "session_configs"


def test_configured_session_and_scenario_paths_are_ordered_search_lists(tmp_path: Path) -> None:
    local_sessions = tmp_path / "sessions"
    example_sessions = tmp_path / "example-sessions"
    local_scenarios = tmp_path / "scenarios"
    example_scenarios = tmp_path / "example-scenarios"
    for root in (local_sessions, example_sessions, local_scenarios, example_scenarios):
        root.mkdir()

    local_session = local_sessions / "local"
    local_session.mkdir()
    local_session.joinpath("session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    example_session = example_sessions / "integrated"
    example_session.mkdir()
    example_session.joinpath("session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    local_scenario = local_scenarios / "local_scenario"
    local_scenario.mkdir()
    local_scenario.joinpath("scenario-definition.yaml").write_text(
        "schema_version: 1\nsession: local\napplications: {}\n",
        encoding="utf-8",
    )
    example_scenario = example_scenarios / "integrated_scenario"
    example_scenario.mkdir()
    example_scenario.joinpath("scenario-definition.yaml").write_text(
        "schema_version: 1\nsession: integrated\napplications: {}\n",
        encoding="utf-8",
    )
    (tmp_path / "ros2docker.json").write_text('{"image_name": "multi"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "\n".join(
            [
                "ros2docker_config: ros2docker.json",
                "session_configs_dir:",
                "  - sessions",
                f"  - {example_sessions}",
                "scenario_configs_dir:",
                "  - scenarios",
                f"  - {example_scenarios}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    runtime = rosotacom._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))
    resolved_session = rosotacom._resolve_session("integrated", runtime)
    resolved_scenario = rosotacom._resolve_scenario("integrated_scenario", runtime)

    assert runtime.session_configs_dir == (local_sessions, example_sessions)
    assert runtime.scenario_configs_dir == (local_scenarios, example_scenarios)
    assert rosotacom._session_names(runtime) == ["local", "integrated"]
    assert rosotacom._scenario_names(runtime) == ["local_scenario", "integrated_scenario"]
    assert resolved_session.host_dir == example_session.resolve()
    assert resolved_session.container_dir == "/session/definitions-2/integrated"
    assert resolved_scenario.definition_path == (example_scenario / "scenario-definition.yaml").resolve()


def test_session_name_completion_uses_active_project_and_prefix(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    sessions = tmp_path / "sessions"
    for name in ("1_heartbeat", "1_heartbeat_status", "2_native_chatter"):
        session = sessions / name
        session.mkdir(parents=True)
        (session / "session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    (sessions / "not_a_session").mkdir()
    (tmp_path / "ros2docker.json").write_text('{"image_name": "completion"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "ros2docker_config: ros2docker.json\nsession_configs_dir:\n  - sessions\n",
        encoding="utf-8",
    )
    monkeypatch.chdir(tmp_path)

    completions = rosotacom._session_name_completer("1_", argparse.Namespace())

    assert completions == {
        "1_heartbeat": "configured session",
        "1_heartbeat_status": "configured session",
    }

    external = tmp_path / "external_session"
    external.mkdir()
    path_completions = rosotacom._session_name_completer("./ext", argparse.Namespace())

    assert path_completions["./external_session/"] == "session directory"


def _write_test_scenario_project(tmp_path: Path) -> tuple[rosotacom.RuntimeConfig, rosotacom.ResolvedScenario]:
    sessions = tmp_path / "sessions"
    session = sessions / "demo"
    session.mkdir(parents=True)
    session.joinpath("session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    app_dir = tmp_path / "apps"
    app_dir.mkdir()
    for identity in ("a", "b"):
        app_dir.joinpath(f"{identity}.json").write_text(
            "\n".join(
                [
                    "{",
                    f'  "container_name": "{identity}_app",',
                    '  "run_type": "command",',
                    '  "command": ["true"]',
                    "}",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        app_dir.joinpath(f"{identity}.lyrical.json").write_text(
            json.dumps(
                {
                    "image_name": "lyrical-app",
                    "run_type": "command",
                    "command": ["true"],
                    "build_args": {"BASE_IMAGE": "ros:lyrical-ros-base-resolute"},
                }
            ),
            encoding="utf-8",
        )
    scenario_dir = tmp_path / "scenarios" / "demo"
    scenario_dir.mkdir(parents=True)
    scenario_dir.joinpath("scenario-definition.yaml").write_text(
        "\n".join(
            [
                "schema_version: 1",
                "session: demo",
                "applications:",
                "  a:",
                "    - name: local_app",
                "      ros2docker_config: ../../apps/a.json",
                "      ros2docker_config_by_distro:",
                "        lyrical: ../../apps/a.lyrical.json",
                "  b:",
                "    - name: local_app",
                "      ros2docker_config: ../../apps/b.json",
                "      ros2docker_config_by_distro:",
                "        lyrical: ../../apps/b.lyrical.json",
                "",
            ]
        ),
        encoding="utf-8",
    )
    runtime = rosotacom.RuntimeConfig(
        rosotacom_config=tmp_path / "rosotacom.yaml",
        ros2docker_config=rosotacom.DEFAULT_ROS2DOCKER_CONFIG,
        session_configs_dir=(sessions,),
        deployment=None,
        install_id="test",
        session_instances_dir=tmp_path / "session-instances",
        scenario_configs_dir=(tmp_path / "scenarios",),
    )
    return runtime, rosotacom._resolve_scenario("demo", runtime)


def _ota_plan(tmp_path: Path) -> rosotacom.OtaSmokePlan:
    return rosotacom.OtaSmokePlan(
        state_path=tmp_path / "ota-deployment.yaml",
        workdir="/tmp/rosotacom_ota",
        rosotacom="source/.venv/bin/rosotacom",
        project="project/rosotacom.yaml",
        peers={
            "a": rosotacom.OtaSmokePeer("a", None, "10.0.0.10"),
            "b": rosotacom.OtaSmokePeer("b", "robot-b", "10.0.0.11"),
        },
    )


def test_scenario_definition_resolves_and_validates_strictly(tmp_path: Path) -> None:
    runtime, resolved = _write_test_scenario_project(tmp_path)

    definition = rosotacom._load_scenario_definition(resolved)

    assert definition.session == "demo"
    assert definition.applications["a"][0].name == "local_app"
    assert definition.applications["a"][0].ros2docker_config == tmp_path / "apps" / "a.json"
    assert definition.applications["a"][0].ros2docker_config_by_distro == (
        ("lyrical", tmp_path / "apps" / "a.lyrical.json"),
    )
    assert rosotacom._scenario_names(runtime) == ["demo"]
    assert rosotacom._scenario_container_name(runtime, "demo", "a", "local_app", "run1") == (
        "rosotacom_test_run1_scenario_demo_a_local_app"
    )
    assert rosotacom._scenario_application_image_name(runtime, definition.applications["a"][0]) == "ros2docker-test"

    lyrical_config = tmp_path / "ros2docker.lyrical.json"
    lyrical_config.write_text(
        json.dumps({"build_args": {"BASE_IMAGE": "ros:lyrical-ros-base-resolute"}}),
        encoding="utf-8",
    )
    lyrical_runtime = replace(runtime, ros2docker_config=lyrical_config)
    application = definition.applications["a"][0]
    assert rosotacom._scenario_application_config(lyrical_runtime, application) == (
        tmp_path / "apps" / "a.lyrical.json"
    )
    assert rosotacom._scenario_application_image_name(lyrical_runtime, application) == "lyrical-app-test"

    resolved.definition_path.write_text(
        "schema_version: 1\nsession: demo\napplications: {}\nunknown: true\n",
        encoding="utf-8",
    )
    with pytest.raises(RuntimeError, match="Unsupported keys"):
        rosotacom._load_scenario_definition(resolved)


def test_scenario_definition_ignores_foreign_peers_host_paths(tmp_path: Path) -> None:
    """Loading a definition must not require other peers' machine-local mounts.

    Application b mounts a host path that exists only on b's machine (a staged
    replay bag). Peer a still has to be able to parse the definition and start
    its own applications; the mount is validated where b actually starts
    (#249).
    """
    runtime, resolved = _write_test_scenario_project(tmp_path)
    foreign_config = tmp_path / "apps" / "b.json"
    foreign_config.write_text(
        "\n".join(
            [
                "{",
                '  "container_name": "b_app",',
                '  "run_type": "command",',
                '  "command": ["true"],',
                '  "run_args": ["-v", "./only-on-machine-b:/bag:ro"]',
                "}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    definition = rosotacom._load_scenario_definition(resolved)
    assert definition.applications["b"][0].name == "local_app"

    # The start path still refuses the missing mount on the machine that runs
    # the application: full resolution stays where it is meaningful.
    with pytest.raises(FileNotFoundError, match="Host path does not exist"):
        rosotacom.load_config(foreign_config)


def test_scenario_name_completion_uses_active_project(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    (tmp_path / "ros2docker.json").write_text('{"image_name": "completion"}\n', encoding="utf-8")
    runtime.rosotacom_config.write_text(
        "\n".join(
            [
                "ros2docker_config: ros2docker.json",
                "session_configs_dir:",
                "  - sessions",
                "scenario_configs_dir:",
                "  - scenarios",
                "",
            ]
        ),
        encoding="utf-8",
    )
    monkeypatch.chdir(tmp_path)

    assert rosotacom._scenario_name_completer("de", argparse.Namespace()) == {"demo": "configured scenario"}


def test_active_scenarios_are_discovered_from_tmux_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        assert command[:3] == ["tmux", "-L", "rosotacom-test"]
        return subprocess.CompletedProcess(command, 0, "demo-a\tdemo\ta\ndemo-b\tdemo\tb\n", "")

    monkeypatch.setattr(rosotacom.shutil, "which", lambda name: "/usr/bin/tmux")
    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)

    assert rosotacom._active_scenario_runs(runtime) == [
        rosotacom.ActiveScenarioRun("demo", "a", "demo-a"),
        rosotacom.ActiveScenarioRun("demo", "b", "demo-b"),
    ]
    assert rosotacom._format_scenario_listing(runtime) == "\n".join(
        [
            "Configured scenarios:",
            "  - demo (active: a, b)",
            "Active scenarios:",
            "  - demo --identity a",
            "  - demo --identity b",
        ]
    )


def test_scenario_attach_selector_infers_the_only_active_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(
        rosotacom,
        "_active_scenario_runs",
        lambda runtime: [rosotacom.ActiveScenarioRun("demo", "a", "demo-a")],
    )
    args = argparse.Namespace(scenario=None, identity=None)

    rosotacom._infer_active_scenario_selector(args, require_active=True)

    assert (args.scenario, args.identity) == ("demo", "a")
    assert "Auto-selected active scenario: demo" in capsys.readouterr().out


def test_scenario_selector_requires_choice_for_multiple_active_identities(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(
        rosotacom,
        "_active_scenario_runs",
        lambda runtime: [
            rosotacom.ActiveScenarioRun("demo", "a", "demo-a"),
            rosotacom.ActiveScenarioRun("demo", "b", "demo-b"),
        ],
    )

    with pytest.raises(RuntimeError, match="multiple active identities"):
        rosotacom._infer_active_scenario_selector(
            argparse.Namespace(scenario="demo", identity=None),
            require_active=True,
        )


def test_scenario_and_identity_completion_follow_active_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(
        rosotacom,
        "_active_scenario_runs",
        lambda runtime: [rosotacom.ActiveScenarioRun("demo", "a", "demo-a")],
    )

    attach_args = argparse.Namespace(command="scenario", scenario_command="attach", scenario=None)
    assert rosotacom._active_scenario_name_completer("d", attach_args) == {"demo": "active scenario"}
    assert rosotacom._identity_completer("", attach_args) == {"a": "peer identity"}

    start_args = argparse.Namespace(command="scenario", scenario_command="start", scenario="demo")
    assert rosotacom._identity_completer("", start_args) == {
        "a": "peer identity",
        "b": "peer identity",
    }

    session_args = argparse.Namespace(command="start", session_dir_positional="demo", session_dir=None)
    assert rosotacom._identity_completer("", session_args) == {
        "a": "peer identity",
        "b": "peer identity",
    }


def test_scenario_attach_cli_accepts_no_selector(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[argparse.Namespace] = []
    monkeypatch.setattr(rosotacom, "attach_scenario", lambda args: calls.append(args) or 0)

    assert rosotacom.main(["scenario", "attach"]) == 0
    assert calls[0].scenario is None
    assert calls[0].identity is None


def test_scenario_tmux_commands_keep_ctrl_b_and_use_full_windows(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, resolved = _write_test_scenario_project(tmp_path)
    definition = rosotacom._load_scenario_definition(resolved)
    session = rosotacom._resolve_session("demo", runtime)
    instance = rosotacom._resolve_session_instance(runtime, session, "tmux")
    calls: list[list[str]] = []
    pane_number = 0

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        nonlocal pane_number
        calls.append(command)
        if "new-session" in command or "new-window" in command or "split-window" in command:
            pane_number += 1
            return subprocess.CompletedProcess(command, 0, f"%{pane_number}\n", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)

    tmux_session = rosotacom._create_scenario_tmux(
        runtime,
        resolved,
        definition,
        instance,
        "a",
        definition.applications["a"],
        "rosotacom_test_com_to_b",
        argparse.Namespace(
            force=True,
            rewrite_formatting=False,
            overwrite_peers_via_remote_peer=None,
            peer_address=[],
            network_name="scenario-net",
            network_ip="10.139.0.2",
        ),
    )

    assert tmux_session == "demo-a"
    assert all(command[:3] == ["tmux", "-L", "rosotacom-test"] for command in calls)
    assert any(command[-2:] == ["prefix", "C-b"] for command in calls if "set-option" in command)
    assert any(command[-3:] == ["prefix", "C-b", "send-prefix"] for command in calls if "bind-key" in command)
    assert any(command[-2:] == ["@rosotacom_scenario", "demo"] for command in calls)
    assert any(command[-2:] == ["@rosotacom_identity", "a"] for command in calls)
    assert any("inner catmux: C-b C-b" in part for command in calls for part in command)
    joined = "\n".join(" ".join(command) for command in calls)
    assert "rosotacom start demo --identity a --mode attach" in joined
    assert "--network-name scenario-net --network-ip 10.139.0.2" in joined
    assert "scenario _run-application demo" in joined
    assert "--network-name container:rosotacom_test_com_to_b" in joined
    assert "waiting for container readiness: rosotacom_test_com_to_b" in joined
    assert sum("new-window" in command for command in calls) == 1
    assert not any("split-window" in command for command in calls)
    assert any(
        command[command.index("-n") : command.index("-n") + 2] == ["-n", "local_app"]
        for command in calls
        if "new-window" in command
    )
    assert any(command[-1] == "demo-a:communication" for command in calls if "select-window" in command)
    assert (instance.logs_host_dir / "a" / "scenario").is_dir()


def test_interactive_smoke_target_resolution_prefers_scenario_and_peer_ips(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)

    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    session_target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "session")

    assert target.target_type == "scenario"
    assert target.name == "demo"
    assert session_target.target_type == "session"
    assert rosotacom._smoke_peer_ips_for_subnet(["robot_b", "robot_a"], "10.137.42.0/24") == {
        "robot_a": "10.137.42.2",
        "robot_b": "10.137.42.3",
    }


def test_interactive_smoke_tmux_uses_full_windows_and_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "interactive")
    calls: list[list[str]] = []
    pane_number = 0

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        nonlocal pane_number
        calls.append(command)
        if "new-session" in command or "new-window" in command or "split-window" in command:
            pane_number += 1
            return subprocess.CompletedProcess(command, 0, f"%{pane_number}\n", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)

    tmux_session = rosotacom._create_interactive_smoke_tmux(
        runtime,
        target,
        instance,
        {"a": "10.137.42.2", "b": "10.137.42.3"},
        "smoke-net",
        link_trace_parts=["--link-trace", "--link-trace-interval", "0.5"],
    )

    assert tmux_session == "smoke-scenario-demo"
    window_names = [
        command[command.index("-n") + 1]
        for command in calls
        if ("new-session" in command or "new-window" in command) and "-n" in command
    ]
    assert window_names == [
        "a_communication",
        "b_communication",
        "a_local_app",
        "b_local_app",
        "verification",
    ]
    assert any(command[-2:] == ["@rosotacom_smoke_target", "demo"] for command in calls)
    assert any(command[-2:] == ["@rosotacom_smoke_target_type", "scenario"] for command in calls)
    assert any(command[-2:] == ["@rosotacom_smoke_instance", "interactive"] for command in calls)
    assert any("inner catmux: C-b C-b" in part for command in calls for part in command)
    joined = "\n".join(" ".join(command) for command in calls)
    assert "--smoke-managed" in joined
    assert "--link-trace --link-trace-interval 0.5" in joined
    assert "--network-name smoke-net --network-ip 10.137.42.2" in joined
    assert "--peer-address a=10.137.42.2 --peer-address b=10.137.42.3" in joined
    assert "scenario _run-application demo --identity a --application local_app --instance-id interactive" in joined
    assert "--network-name container:rosotacom_test_interactive_com_to_b" in joined
    assert "waiting for generated config" in joined
    assert "waiting for container readiness" in joined
    assert any(command[-1] == "smoke-scenario-demo:verification" for command in calls if "select-window" in command)


def test_run_scenario_application_can_override_network(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    (tmp_path / "apps" / "a.json").write_text(
        "\n".join(
            [
                "{",
                '  "container_name": "a_app",',
                '  "image_name": "app-image",',
                '  "run_type": "command",',
                '  "command": ["true"],',
                '  "run_args": ["--network", "host", "-e", "ROS_DOMAIN_ID=46"]',
                "}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    runs: list[dict[str, object]] = []
    monkeypatch.setattr(rosotacom, "_require_ros2docker", lambda: None)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_container_exists", lambda name: False)
    monkeypatch.setattr(rosotacom, "ros2docker_build", lambda **kwargs: None)
    monkeypatch.setattr(
        rosotacom,
        "ros2docker_run",
        lambda **kwargs: runs.append(dict(kwargs)),
    )

    assert (
        rosotacom.run_scenario_application(
            argparse.Namespace(
                scenario="demo",
                identity="a",
                application="local_app",
                instance_id="run1",
                network_name="smoke-net",
                network_ip=None,
            )
        )
        == 0
    )

    override = runs[0]["override"]
    assert isinstance(override, dict)
    assert override["container_name"] == "rosotacom_test_run1_scenario_demo_a_local_app"
    assert override["image_name"] == "app-image-test"
    assert override["run_args"] == ["-e", "ROS_DOMAIN_ID=46", "--network", "smoke-net", "--cap-add", "NET_ADMIN"]


def test_active_interactive_smoke_runs_are_listed_from_tmux_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        assert command[:3] == ["tmux", "-L", "rosotacom-test"]
        return subprocess.CompletedProcess(command, 0, "smoke-scenario-demo\tdemo\tscenario\trun1\tnet1\n", "")

    monkeypatch.setattr(rosotacom.shutil, "which", lambda name: "/usr/bin/tmux")
    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)

    runs = rosotacom._active_interactive_smoke_runs(runtime)

    assert runs == [rosotacom.ActiveInteractiveSmokeRun("demo", "scenario", "smoke-scenario-demo", "run1", "net1")]
    assert "demo (scenario) instance=run1 network=net1" in rosotacom._format_active_interactive_smoke_runs(runs)


def test_ota_smoke_plan_writes_state_and_manifest(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_plan(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota")
    plan = rosotacom._ota_write_state(instance, plan)
    rosotacom._ota_write_manifest(
        instance,
        target,
        runtime,
        plan,
        tmux_session="ota-smoke-scenario-demo",
        interactive=True,
        phase="running",
    )
    manifest = yaml.safe_load((instance.host_dir / "manifest.yaml").read_text(encoding="utf-8"))
    run = manifest["ota_smoke_runs"]["scenario:demo"]

    assert target.target_type == "scenario"
    assert plan.peers["b"].ssh == "robot-b"
    assert run["deployment_state"] == str(instance.host_dir / "ota-deployment.yaml")
    assert run["peers"]["b"] == {
        "ssh_configured": True,
        "exec_configured": False,
        "address": "10.0.0.11",
    }
    assert rosotacom._ota_load_state(str(plan.state_path)) == plan


def test_ota_smoke_command_building_and_remote_wrapping(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_plan(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    peer_args = rosotacom._ota_peer_address_args(plan)

    start = rosotacom._ota_rosotacom_command(
        plan,
        rosotacom._ota_start_parts(target, "a", "run1", peer_args, mode="detached"),
        plan.peers["a"],
    )
    stop = rosotacom._ota_rosotacom_command(
        plan, rosotacom._ota_stop_parts(target, "b", "run1", peer_args), plan.peers["b"]
    )

    assert start.startswith("cd /tmp/rosotacom_ota && source/.venv/bin/rosotacom scenario start demo")
    assert "--identity a --mode detached --instance-id run1 --force" in start
    assert "--peer-address a=10.0.0.10 --peer-address b=10.0.0.11" in start
    assert "--rosotacom-config project/rosotacom.yaml" in start
    assert "scenario stop demo --identity b --instance-id run1" in stop
    assert rosotacom._ota_remote_argv(plan.peers["a"], "true") == ["bash", "-lc", "true"]
    assert rosotacom._ota_remote_argv(plan.peers["b"], "true", tty=True, batch=True) == [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-t",
        "robot-b",
        "true",
    ]


def test_ota_workdir_validation_refuses_dangerous_paths() -> None:
    with pytest.raises(RuntimeError, match="dangerous OTA prepare workdir"):
        rosotacom._ota_validate_prepare_workdir("/")
    rosotacom._ota_validate_prepare_workdir("/tmp/rosotacom_ota")


def test_active_ota_smoke_runs_are_listed_from_tmux_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        assert command[:3] == ["tmux", "-L", "rosotacom-test"]
        return subprocess.CompletedProcess(
            command,
            0,
            "ota-smoke-scenario-demo\tdemo\tscenario\trun1\t/tmp/ota-state.yaml\n",
            "",
        )

    monkeypatch.setattr(rosotacom.shutil, "which", lambda name: "/usr/bin/tmux")
    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)

    runs = rosotacom._active_ota_smoke_runs(runtime)

    assert runs == [
        rosotacom.ActiveOtaSmokeRun("demo", "scenario", "ota-smoke-scenario-demo", "run1", "/tmp/ota-state.yaml")
    ]
    assert "demo (scenario) instance=run1" in rosotacom._format_active_ota_smoke_runs(runs)


def test_interactive_ota_smoke_tmux_uses_control_windows_and_metadata(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_plan(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota-interactive")
    calls: list[list[str]] = []
    pane_number = 0

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        nonlocal pane_number
        calls.append(command)
        if "new-session" in command or "new-window" in command or "split-window" in command:
            pane_number += 1
            return subprocess.CompletedProcess(command, 0, f"%{pane_number}\n", "")
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(rosotacom.shutil, "which", lambda name: "/usr/bin/tmux")
    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)

    tmux_session = rosotacom._ota_create_tmux(runtime, target, plan, instance)

    assert tmux_session == "ota-smoke-scenario-demo"
    window_names = [
        command[command.index("-n") + 1]
        for command in calls
        if ("new-session" in command or "new-window" in command) and "-n" in command
    ]
    assert window_names == ["a_communication", "b_communication", "a_local_app", "b_local_app", "verification"]
    assert any(command[-2:] == ["@rosotacom_ota_smoke_target", "demo"] for command in calls)
    assert any(command[-2:] == ["@rosotacom_ota_smoke_target_type", "scenario"] for command in calls)
    assert any(command[-2:] == ["@rosotacom_ota_smoke_instance", "ota-interactive"] for command in calls)
    assert any(command[-2:] == ["@rosotacom_ota_smoke_state", str(plan.state_path)] for command in calls)
    joined = "\n".join(" ".join(command) for command in calls)
    assert "start demo --identity a --mode attach --instance-id ota-interactive --smoke-managed" in joined
    assert "start demo --identity b --mode attach --instance-id ota-interactive --smoke-managed" in joined
    assert "scenario _run-application demo --identity a --application local_app" in joined
    assert "scenario _run-application demo --identity b --application local_app" in joined
    assert "waiting for running container" in joined
    assert "docker attach" not in joined
    assert "scenario start demo --identity a --mode attach" not in joined
    assert "scenario start demo --identity b --mode attach" not in joined
    # Communication windows are full-screen now: no status-watch split pane.
    assert "--watch" not in joined
    assert not any("split-window" in command for command in calls)
    assert "ota-smoke demo --state-file" in joined
    assert "--verify-only" in joined


def _make_session_instance(host_dir: Path, instance_id: str) -> rosotacom.SessionInstance:
    return rosotacom.SessionInstance(
        instance_id=instance_id,
        host_dir=host_dir,
        container_dir="/c",
        config_host_dir=host_dir / "config",
        config_container_dir="/c/config",
        logs_host_dir=host_dir / "logs",
        logs_container_dir="/c/logs",
        rosbags_host_dir=host_dir / "rosbags",
        rosbags_container_dir="/c/rosbags",
    )


def test_ota_extract_peer_artifacts_merges_full_layout_and_skips_manifest(tmp_path: Path) -> None:
    host_dir = tmp_path / "inst"
    host_dir.mkdir()
    # The orchestration manifest must survive peer collection.
    (host_dir / "manifest.yaml").write_text("orchestration: keep\n", encoding="utf-8")
    instance = _make_session_instance(host_dir, "abcd")
    peer = rosotacom.OtaSmokePeer("a", None, "10.0.0.10")

    # The remote instance dir carries its own per-host timestamp (suffix matches only).
    prefix = "session-instances/2026-06-22/sess_2026-06-22_00-00-09_abcd"
    members = {
        f"{prefix}/manifest.yaml": b"peer: should-be-skipped\n",
        f"{prefix}/config/a/plugin.yaml": b"plugin: a\n",
        f"{prefix}/logs/a/launcher.log": b"launcher\n",
        f"{prefix}/logs/a/catmux/00-COM/0.log": b"com\n",
        f"{prefix}/logs/a/status/events.jsonl": b'{"e":1}\n',
    }
    buffer = io.BytesIO()
    with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
        for name, data in members.items():
            info = tarfile.TarInfo(name)
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
        evil = b"evil\n"
        escape = tarfile.TarInfo(f"{prefix}/../../escape.txt")
        escape.size = len(evil)
        archive.addfile(escape, io.BytesIO(evil))
    encoded = base64.b64encode(buffer.getvalue()).decode("ascii")

    rosotacom._ota_extract_peer_artifacts(instance, peer, encoded)

    assert (host_dir / "config" / "a" / "plugin.yaml").read_text(encoding="utf-8") == "plugin: a\n"
    assert (host_dir / "logs" / "a" / "launcher.log").read_text(encoding="utf-8") == "launcher\n"
    assert (host_dir / "logs" / "a" / "catmux" / "00-COM" / "0.log").read_text(encoding="utf-8") == "com\n"
    assert (host_dir / "logs" / "a" / "status" / "events.jsonl").read_text(encoding="utf-8") == '{"e":1}\n'
    # Per-peer manifest skipped so the orchestration manifest stays intact.
    assert (host_dir / "manifest.yaml").read_text(encoding="utf-8") == "orchestration: keep\n"
    # Path-traversal members are refused.
    assert not (tmp_path / "escape.txt").exists()
    assert not (tmp_path.parent / "escape.txt").exists()


def test_ota_collect_logs_extracts_each_peer_into_instance(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    host_dir = tmp_path / "inst"
    host_dir.mkdir()
    instance = _make_session_instance(host_dir, "abcd")
    plan = _ota_plan(tmp_path)

    def encoded_for(identity: str) -> str:
        prefix = f"session-instances/2026-06-22/sess_{identity}_abcd"
        buffer = io.BytesIO()
        with tarfile.open(fileobj=buffer, mode="w:gz") as archive:
            data = f"{identity}-launcher\n".encode()
            info = tarfile.TarInfo(f"{prefix}/logs/{identity}/launcher.log")
            info.size = len(data)
            archive.addfile(info, io.BytesIO(data))
        return base64.b64encode(buffer.getvalue()).decode("ascii")

    def fake_ota_run(
        peer: rosotacom.OtaSmokePeer, *_args: object, **_kwargs: object
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess([], 0, encoded_for(peer.name), "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)
    rosotacom._ota_collect_logs(instance, plan, dry_run=False)

    assert (host_dir / "logs" / "a" / "launcher.log").read_text(encoding="utf-8") == "a-launcher\n"
    assert (host_dir / "logs" / "b" / "launcher.log").read_text(encoding="utf-8") == "b-launcher\n"
    # No opaque base64 blob is left behind.
    assert not list(host_dir.glob("**/*.tar.gz.b64"))


def _wait_script_env(tmp_path: Path, docker_stdout: str) -> dict[str, str]:
    """PATH with a stub docker that prints `docker_stdout` for any invocation."""
    stub = tmp_path / "bin" / "docker"
    stub.parent.mkdir(exist_ok=True)
    stub.write_text(f"#!/bin/sh\nprintf '%s\\n' {rosotacom.shlex.quote(docker_stdout)}\n", encoding="utf-8")
    stub.chmod(0o755)
    return {**os.environ, "PATH": f"{stub.parent}:{os.environ['PATH']}"}


def test_ota_wait_script_is_unbounded_without_timeout() -> None:
    script = rosotacom._ota_wait_for_running_container_suffix_script("_com_to_b", "a:communication")

    assert "exit 1" not in script
    assert "waiting for running container:" in script


def test_ota_bounded_wait_script_fails_clearly_when_the_bound_is_hit(tmp_path: Path) -> None:
    script = rosotacom._ota_wait_for_running_container_suffix_script(
        "_scenario_demo_a_local_app", "a:application:local_app", timeout_s=2
    )

    result = subprocess.run(
        ["bash", "-c", script], env=_wait_script_env(tmp_path, "unrelated_container"), text=True, capture_output=True
    )

    assert result.returncode == 1
    assert "[ERROR] no running container after 2s: a:application:local_app" in result.stderr


def test_ota_bounded_wait_script_succeeds_once_the_container_runs(tmp_path: Path) -> None:
    script = rosotacom._ota_wait_for_running_container_suffix_script(
        "_scenario_demo_a_local_app", "a:application:local_app", timeout_s=2
    )
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    container = rosotacom._scenario_container_name(runtime, "demo", "a", "local_app", "run1")

    result = subprocess.run(
        ["bash", "-c", script], env=_wait_script_env(tmp_path, container), text=True, capture_output=True
    )

    assert result.returncode == 0
    assert f"[INFO] found running container: {container}" in result.stdout


def _runtime_with_prefix(tmp_path: Path, prefix_line: str = "") -> rosotacom.RuntimeConfig:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "test"}\n', encoding="utf-8")
    config = tmp_path / "rosotacom.yaml"
    config.write_text(f"ros2docker_config: ros2docker.json\n{prefix_line}", encoding="utf-8")
    return rosotacom._load_runtime_config(argparse.Namespace(rosotacom_config=str(config)))


def test_ota_application_wait_matches_the_name_the_start_gives(tmp_path: Path) -> None:
    """The wait and the start must make the same naming decision, in both modes.

    When `com_container_prefix` was introduced (#247), the start named the com
    container `<prefix>_com-to-<peer>` while the application windows kept
    waiting for the instance-scoped `_com_to_<peer>` suffix — so every scenario
    application waited forever on a healthy machine (#254).
    """
    for prefix_line in ("", "com_container_prefix: remote-assist\n"):
        runtime = _runtime_with_prefix(tmp_path, prefix_line)
        actual_name = rosotacom._container_name("center", runtime, "run1")
        suffix = rosotacom._ota_communication_wait_suffix(runtime, "center")
        assert actual_name.endswith(suffix), (
            f"wait suffix {suffix!r} does not match the started container {actual_name!r}"
        )


def test_ota_application_wait_script_finds_the_prefixed_container(tmp_path: Path) -> None:
    runtime = _runtime_with_prefix(tmp_path, "com_container_prefix: remote-assist\n")
    container = rosotacom._container_name("center", runtime, "run1")
    assert container == "remote-assist_com-to-center"

    script = rosotacom._ota_wait_for_running_container_suffix_script(
        rosotacom._ota_communication_wait_suffix(runtime, "center"),
        "vehicle:communication",
        timeout_s=2,
    )
    result = subprocess.run(
        ["bash", "-c", script], env=_wait_script_env(tmp_path, container), text=True, capture_output=True
    )

    assert result.returncode == 0
    assert f"[INFO] found running container: {container}" in result.stdout


def test_ota_wait_for_scenario_applications_waits_per_declared_application(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    plan = _ota_plan(tmp_path)
    ran: list[tuple[str, str]] = []

    def fake_ota_run(peer: rosotacom.OtaSmokePeer, script: str, **_kwargs: object) -> subprocess.CompletedProcess[str]:
        ran.append((peer.name, script))
        return subprocess.CompletedProcess([], 0, "", "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)

    assert rosotacom._ota_wait_for_scenario_applications(target, plan, dry_run=False) == []

    assert [name for name, _script in ran] == ["a", "b"]
    for identity, script in ran:
        assert f"_scenario_demo_{identity}_local_app" in script
        # The bound: verification must not wait forever, unlike the interactive panes.
        assert "exit 1" in script


def test_ota_wait_for_scenario_applications_reports_the_bound_not_the_topics(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    plan = _ota_plan(tmp_path)

    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        lambda peer, script, **kwargs: subprocess.CompletedProcess([], 0 if peer.name == "a" else 1, "", ""),
    )

    errors = rosotacom._ota_wait_for_scenario_applications(target, plan, dry_run=False)

    assert len(errors) == 1
    assert "b: application 'local_app' has no running container" in errors[0]
    assert "never came up" in errors[0]


def test_ota_verify_only_stops_before_checks_when_applications_never_come_up(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    plan = _ota_plan(tmp_path)
    checks: list[str] = []

    monkeypatch.setattr(rosotacom, "_resolve_ota_smoke_context", lambda args: (runtime, plan, target))
    monkeypatch.setattr(
        rosotacom,
        "_ota_wait_for_scenario_applications",
        lambda *args, **kwargs: ["b: application 'local_app' has no running container after 600s"],
    )
    monkeypatch.setattr(rosotacom, "_ota_start_session_publishers", lambda *args, **kwargs: checks.append("publishers"))
    monkeypatch.setattr(rosotacom, "_ota_verify_delivery", lambda *args, **kwargs: checks.append("delivery") or [])
    monkeypatch.setattr(rosotacom, "_ota_verify_isolation", lambda *args, **kwargs: checks.append("isolation") or [])

    rc = rosotacom._ota_verify_only(argparse.Namespace(instance_id="run1", dry_run=False))

    assert rc == 1
    # The point of #218: no status checks against a scenario that is not up.
    assert checks == []


def test_ota_wait_for_scenario_applications_ignores_session_targets(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "session")
    plan = _ota_plan(tmp_path)
    monkeypatch.setattr(rosotacom, "_ota_run", lambda *args, **kwargs: pytest.fail("session targets must not wait"))

    assert rosotacom._ota_wait_for_scenario_applications(target, plan, dry_run=False) == []


def test_ota_smoke_parser_accepts_stop_without_peer_arguments(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[argparse.Namespace] = []
    monkeypatch.setattr(rosotacom, "ota_smoke", lambda args: calls.append(args) or 0)

    assert rosotacom.main(["ota-smoke", "--stop"]) == 0

    assert calls[0].target is None
    assert calls[0].stop is True
    assert calls[0].state_file is None


def test_ota_smoke_stop_dry_run_does_not_kill_local_tmux(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_plan(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    calls: list[str] = []
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_resolve_ota_smoke_context", lambda args: (runtime, plan, target))
    monkeypatch.setattr(rosotacom, "_ota_stop_peers", lambda *args, **kwargs: calls.append("remote-dry-run"))
    monkeypatch.setattr(rosotacom, "_kill_scenario_tmux", lambda *args, **kwargs: calls.append("tmux") or True)

    assert (
        rosotacom._stop_ota_smoke(
            argparse.Namespace(
                target="demo",
                target_type="scenario",
                state_file=str(plan.state_path),
                instance_id="run1",
                dry_run=True,
                keep_workdir=True,
            )
        )
        == 0
    )

    assert calls == ["remote-dry-run"]
    assert "Would stop OTA smoke tmux session: ota-smoke-scenario-demo" in capsys.readouterr().out


def test_ota_smoke_stop_with_target_falls_back_to_peer_arguments(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_plan(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    calls: list[str] = []

    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_active_ota_smoke_runs", lambda runtime: [])
    monkeypatch.setattr(rosotacom, "_manifest_ota_smoke_runs", lambda runtime: [])
    monkeypatch.setattr(rosotacom, "_resolve_ota_smoke_context", lambda args: (runtime, plan, target))
    monkeypatch.setattr(rosotacom, "_ota_stop_peers", lambda *args, **kwargs: calls.append("remote-stop"))
    monkeypatch.setattr(rosotacom, "_kill_scenario_tmux", lambda *args, **kwargs: calls.append("tmux") or False)
    monkeypatch.setattr(rosotacom, "_ota_cleanup_hosts", lambda *args, **kwargs: calls.append("cleanup"))

    assert (
        rosotacom._stop_ota_smoke(
            argparse.Namespace(
                target="demo",
                target_type="scenario",
                state_file=None,
                instance_id=None,
                dry_run=False,
                keep_workdir=False,
            )
        )
        == 0
    )

    assert calls == ["remote-stop", "tmux", "cleanup"]
    assert "No active OTA smoke run found" not in capsys.readouterr().err


def test_noninteractive_ota_smoke_lifecycle_uses_generic_runner(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_plan(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota-lifecycle")
    calls: list[str] = []

    monkeypatch.setattr(rosotacom, "_resolve_ota_smoke_context", lambda args: (runtime, plan, target))
    monkeypatch.setattr(rosotacom, "_resolve_session_instance", lambda *args, **kwargs: instance)
    monkeypatch.setattr(rosotacom, "_ota_preflight", lambda *args, **kwargs: calls.append("preflight"))
    monkeypatch.setattr(rosotacom, "_ota_prepare_hosts", lambda *args, **kwargs: calls.append("prepare"))
    monkeypatch.setattr(rosotacom, "_ota_start_peers", lambda *args, **kwargs: calls.append("start"))
    monkeypatch.setattr(
        rosotacom, "_ota_wait_for_scenario_applications", lambda *args, **kwargs: calls.append("wait-apps") or []
    )
    monkeypatch.setattr(rosotacom, "_ota_start_session_publishers", lambda *args, **kwargs: calls.append("publishers"))
    monkeypatch.setattr(rosotacom, "_ota_verify_delivery", lambda *args, **kwargs: calls.append("test") or [])
    monkeypatch.setattr(rosotacom, "_ota_verify_isolation", lambda *args, **kwargs: calls.append("isolation") or [])
    monkeypatch.setattr(rosotacom, "_ota_collect_logs", lambda *args, **kwargs: calls.append("collect"))
    monkeypatch.setattr(rosotacom, "_ota_stop_peers", lambda *args, **kwargs: calls.append("stop"))
    monkeypatch.setattr(rosotacom, "_ota_cleanup_hosts", lambda *args, **kwargs: calls.append("cleanup"))
    monkeypatch.setattr(rosotacom, "_ota_write_state", lambda instance, plan: plan)
    monkeypatch.setattr(rosotacom.time, "sleep", lambda seconds: calls.append(f"sleep:{seconds}"))

    assert (
        rosotacom._start_noninteractive_ota_smoke(
            argparse.Namespace(
                skip_preflight=False,
                check_peer_reachability=False,
                dry_run=False,
                instance_id="ota-lifecycle",
                keep_running=False,
                keep_workdir=False,
            )
        )
        == 0
    )

    assert calls == [
        "preflight",
        "prepare",
        "start",
        "sleep:12",
        "wait-apps",
        "publishers",
        "test",
        "isolation",
        "collect",
        "stop",
        "cleanup",
    ]


def test_ota_smoke_dry_run_exercises_generic_remote_flow(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_ota_source_checkout", lambda: tmp_path)

    assert (
        rosotacom.ota_smoke(
            argparse.Namespace(
                target="demo",
                target_type="scenario",
                peer=[],
                peer_address=["a=10.0.0.10", "b=10.0.0.11"],
                peer_ssh=["b=robot-b"],
                deployment=None,
                interactive=False,
                stop=False,
                list=False,
                verify_only=False,
                reuse=False,
                workdir="/tmp/rosotacom_ota",
                keep_workdir=False,
                skip_preflight=False,
                check_peer_reachability=True,
                dry_run=True,
                instance_id="ota-dry",
                keep_running=False,
                mode="detached",
                state_file=None,
            )
        )
        == 0
    )

    out = capsys.readouterr().out
    assert "stage rosotacom source" in out
    assert "install rosotacom" in out
    assert "required command tmux" in out
    assert "Python venv support" in out
    assert "reach peer b (10.0.0.11): running remote command" in out
    # Named for what it checks rather than for one client: a peer can be
    # reached by something other than ssh.
    assert "b: reachable: running remote command" in out
    assert "a: start demo: running remote command" in out
    assert "b: start demo: running remote command" in out
    assert "a: wait for application local_app: running remote command" in out
    assert "b: wait for application local_app: running remote command" in out
    assert "a: rosotacom test: running remote command" in out
    assert "a: publish isolation probe: running remote command" in out
    assert "b: check isolation probe absent: running remote command" in out
    assert "a: stop demo: running remote command" in out
    assert "a: collect session-instances: running remote command" in out
    assert "OTA SMOKE OK" in out


def test_interactive_smoke_stop_infers_active_run_and_cleans_components(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "scenario")
    calls: list[str] = []

    monkeypatch.setattr(rosotacom, "_require_ros2docker", lambda: None)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(
        rosotacom,
        "_active_interactive_smoke_runs",
        lambda runtime: [
            rosotacom.ActiveInteractiveSmokeRun("demo", "scenario", "smoke-scenario-demo", "run1", "net1")
        ],
    )
    monkeypatch.setattr(rosotacom, "_resolve_interactive_smoke_target", lambda *args, **kwargs: target)
    monkeypatch.setattr(
        rosotacom,
        "_stop_scenario_application",
        lambda *args, **kwargs: calls.append("app") or True,
    )
    monkeypatch.setattr(
        rosotacom,
        "_stop_container_name",
        lambda *args, **kwargs: calls.append("communication") or True,
    )
    monkeypatch.setattr(rosotacom, "_kill_scenario_tmux", lambda *args, **kwargs: calls.append("tmux") or True)
    monkeypatch.setattr(rosotacom, "_remove_smoke_network", lambda *args, **kwargs: calls.append("network"))
    monkeypatch.setattr(rosotacom, "_find_latest_interactive_smoke_instance", lambda *args, **kwargs: None)

    assert (
        rosotacom._stop_interactive_smoke(argparse.Namespace(session_dir=None, target_type="auto", instance_id=None))
        == 0
    )

    assert calls == ["app", "app", "communication", "communication", "tmux", "network"]


def test_start_and_stop_scenario_manage_manifest_and_component_order(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, resolved = _write_test_scenario_project(tmp_path)
    definition = rosotacom._load_scenario_definition(resolved)
    session = rosotacom._resolve_session("demo", runtime)
    cfg = {"peers": {"a": {}, "b": {}}}
    instance = rosotacom._resolve_session_instance(runtime, session, "managed")
    calls: list[str] = []

    context = (runtime, resolved, definition, session, cfg, "a", definition.applications["a"])
    monkeypatch.setattr(rosotacom, "_require_ros2docker", lambda: None)
    monkeypatch.setattr(rosotacom, "_require_tmux", lambda: None)
    monkeypatch.setattr(rosotacom, "_resolve_scenario_context", lambda args, **kwargs: context)
    monkeypatch.setattr(rosotacom, "_tmux_session_exists", lambda runtime, name: False)
    monkeypatch.setattr(rosotacom, "_resolve_session_instance", lambda *args, **kwargs: instance)
    monkeypatch.setattr(rosotacom, "_remote_peer_name", lambda cfg, identity: "b")
    monkeypatch.setattr(
        rosotacom, "_create_scenario_tmux", lambda *args, **kwargs: calls.append("tmux-start") or "demo-a"
    )
    monkeypatch.setattr(rosotacom, "_resolve_mode", lambda mode: "detached")
    # Detached scenario start now blocks until the comm container is ready
    # (parity with start_session); the readiness probe is irrelevant here.
    monkeypatch.setattr(rosotacom, "_wait_for_container_ready", lambda *args, **kwargs: None)

    args = argparse.Namespace(
        scenario="demo",
        identity="a",
        force=True,
        mode="detached",
        instance_id="managed",
    )
    assert rosotacom.start_scenario(args) == 0
    manifest = yaml.safe_load((instance.host_dir / "manifest.yaml").read_text(encoding="utf-8"))
    run = manifest["scenario_runs"]["demo:a"]
    assert run["tmux_session"] == "demo-a"
    assert run["communication_container"] == "rosotacom_test_managed_com_to_b"
    assert run["applications"][0]["container_name"] == "rosotacom_test_managed_scenario_demo_a_local_app"
    assert run["applications"][0]["image_name"] == "ros2docker-test"
    assert calls == ["tmux-start"]

    monkeypatch.setattr(
        rosotacom,
        "_stop_scenario_application",
        lambda *args, **kwargs: calls.append("application-stop") or True,
    )
    monkeypatch.setattr(
        rosotacom,
        "_matching_com_containers",
        lambda runtime, remote, all_states=False: ["rosotacom_test_managed_com_to_b"],
    )
    monkeypatch.setattr(
        rosotacom,
        "_stop_container_name",
        lambda *args, **kwargs: calls.append("communication-stop") or True,
    )
    monkeypatch.setattr(
        rosotacom,
        "_kill_scenario_tmux",
        lambda *args, **kwargs: calls.append("tmux-stop") or True,
    )
    monkeypatch.setattr(rosotacom, "_find_latest_scenario_instance", lambda *args, **kwargs: instance.host_dir)

    assert rosotacom.stop_scenario(args) == 0
    assert calls[-3:] == ["application-stop", "communication-stop", "tmux-stop"]
    stopped_manifest = yaml.safe_load((instance.host_dir / "manifest.yaml").read_text(encoding="utf-8"))
    assert stopped_manifest["scenario_runs"]["demo:a"]["stopped_at"]


def test_completion_command_emits_shell_registration(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setenv("SHELL", "/usr/bin/zsh")
    monkeypatch.setattr(sys, "argv", ["/tmp/bin/rosotacom@2.2.0"])

    assert rosotacom.completion_command(argparse.Namespace(shell=None)) == 0

    output = capsys.readouterr().out
    assert "_python_argcomplete_global" in output
    assert "compdef _python_argcomplete_global rosotacom@2.2.0" in output


def test_module_completion_registers_the_public_command(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(sys, "argv", ["/tmp/site-packages/rosotacom/__main__.py"])

    assert rosotacom.completion_command(argparse.Namespace(shell="bash")) == 0

    assert "_python_argcomplete_global rosotacom" in capsys.readouterr().out


def test_argcomplete_protocol_returns_session_prefix_matches() -> None:
    line = "rosotacom smoke 1_"
    src_dir = str((Path(__file__).parent.parent.parent / "src").resolve())
    env = {
        **os.environ,
        "PYTHONPATH": os.path.pathsep.join([src_dir, os.environ.get("PYTHONPATH", "")]),
        "ROSOTACOM_PROFILES": str(Path(src_dir) / "rosotacom" / "resources" / "examples" / "profiles.yaml"),
        "ROSOTACOM_ROS2DOCKER_CONFIG": str(Path(src_dir) / "rosotacom" / "resources" / "examples" / "ros2docker.json"),
        "_ARGCOMPLETE": "1",
        "_ARGCOMPLETE_IFS": "\v",
        "_ARGCOMPLETE_SUPPRESS_SPACE": "1",
        "COMP_LINE": line,
        "COMP_POINT": str(len(line)),
    }

    result = subprocess.run(
        ["bash", "-c", f"{rosotacom.shlex.quote(sys.executable)} -m rosotacom 8>&1"],
        check=True,
        capture_output=True,
        env=env,
        text=True,
    )

    assert result.stdout.split("\v") == [
        "1_heartbeat",
        "1_heartbeat_status",
    ], f"STDOUT: {result.stdout!r}, STDERR: {result.stderr!r}"


def test_argcomplete_protocol_returns_identity_matches() -> None:
    line = "rosotacom start 1_heartbeat --identity "
    src_dir = str((Path(__file__).parent.parent.parent / "src").resolve())
    env = {
        **os.environ,
        "PYTHONPATH": os.path.pathsep.join([src_dir, os.environ.get("PYTHONPATH", "")]),
        "ROSOTACOM_PROFILES": str(Path(src_dir) / "rosotacom" / "resources" / "examples" / "profiles.yaml"),
        "ROSOTACOM_ROS2DOCKER_CONFIG": str(Path(src_dir) / "rosotacom" / "resources" / "examples" / "ros2docker.json"),
        "_ARGCOMPLETE": "1",
        "_ARGCOMPLETE_IFS": "\v",
        "_ARGCOMPLETE_SUPPRESS_SPACE": "1",
        "COMP_LINE": line,
        "COMP_POINT": str(len(line)),
    }

    result = subprocess.run(
        ["bash", "-c", f"{rosotacom.shlex.quote(sys.executable)} -m rosotacom 8>&1"],
        check=True,
        capture_output=True,
        env=env,
        text=True,
    )

    assert result.stdout.split("\v") == [
        "a",
        "b",
    ], f"STDOUT: {result.stdout!r}, STDERR: {result.stderr!r}"


def test_peer_completion_returns_logical_peers_hosts_and_values(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    deployment = tmp_path / "deployment.yaml"
    deployment.write_text(
        "\n".join(
            [
                "hosts:",
                "  workstation: {address: 10.0.0.1, ssh: null}",
                "  robot: {address: 10.0.0.2, ssh: robot-b}",
                "values:",
                "  vpn:",
                "    gateway: 10.0.0.254",
                "",
            ]
        ),
        encoding="utf-8",
    )
    runtime = rosotacom.RuntimeConfig(
        rosotacom_config=runtime.rosotacom_config,
        ros2docker_config=runtime.ros2docker_config,
        session_configs_dir=runtime.session_configs_dir,
        deployment=deployment,
        install_id=runtime.install_id,
        session_instances_dir=runtime.session_instances_dir,
        scenario_configs_dir=runtime.scenario_configs_dir,
    )
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    parsed = argparse.Namespace(command="start", session_dir_positional="demo", session_dir=None)

    assert rosotacom._peer_address_completer("", parsed) == {
        "a=": "peer address override",
        "b=": "peer address override",
    }
    assert rosotacom._peer_address_completer("a=value:vpn.", parsed) == {
        "a=value:vpn.gateway": "deployment value",
    }
    assert rosotacom._peer_host_completer("a=", parsed) == {
        "a=robot": "deployment host",
        "a=workstation": "deployment host",
    }


def test_smoke_interactive_parser_accepts_stop_without_target(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[argparse.Namespace] = []
    monkeypatch.setattr(rosotacom, "smoke", lambda args: calls.append(args) or 0)

    assert rosotacom.main(["smoke", "--interactive", "--stop"]) == 0

    assert calls[0].session_dir is None
    assert calls[0].interactive is True
    assert calls[0].interactive_stop is True


def test_local_check_derives_from_domains_and_allows_opt_out(tmp_path: Path) -> None:
    sessions = tmp_path / "sessions"
    derived = sessions / "derived"
    derived.mkdir(parents=True)
    derived.joinpath("session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "peer_settings:",
                "  a: { domain_id: 46 }",
                "  b: { domain_id: 47 }",
                "",
            ]
        ),
        encoding="utf-8",
    )
    opted_out = sessions / "opted_out"
    opted_out.mkdir()
    opted_out.joinpath("session-definition.yaml").write_text(
        "\n".join(
            [
                "local_check: false",
                "peers:",
                "  a: {}",
                "  b: {}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    shared_domain = sessions / "shared_domain"
    shared_domain.mkdir()
    shared_domain.joinpath("session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "shared:",
                "  local_domain_id: 46",
                "",
            ]
        ),
        encoding="utf-8",
    )

    assert rosotacom.session_local_checks(sessions) == {
        "derived": True,
        "opted_out": False,
        "shared_domain": False,
    }
    assert rosotacom.local_check_sessions(sessions) == ["derived"]
    assert set(rosotacom.ota_suite_sessions(sessions)) == {"derived", "opted_out", "shared_domain"}


def test_examples_have_logical_peers_without_default_deployment() -> None:
    runtime = rosotacom.RuntimeConfig(
        rosotacom_config=rosotacom.EXAMPLE_PROJECT_DIR / "rosotacom.yaml",
        ros2docker_config=rosotacom.EXAMPLE_PROJECT_DIR / "ros2docker.json",
        session_configs_dir=(rosotacom.EXAMPLE_PROJECT_DIR / "sessions",),
        deployment=None,
        install_id="test",
    )
    cfg = rosotacom._effective_session_config(
        rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "1_heartbeat",
        runtime,
    )
    assert cfg["peers"] == {"a": {}, "b": {}}
    with pytest.raises(RuntimeError, match="Missing deployment address"):
        rosotacom._resolve_bindings(cfg, runtime)


def test_version_flag_prints_package_version(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(SystemExit) as excinfo:
        rosotacom.main(["--version"])

    assert excinfo.value.code == 0
    assert capsys.readouterr().out.startswith("rosotacom ")


def test_positional_session_defaults_to_start(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[argparse.Namespace] = []

    def fake_start_session(args: argparse.Namespace) -> str:
        calls.append(args)
        return "container"

    monkeypatch.setattr(rosotacom, "start_session", fake_start_session)

    result = rosotacom.main(["1_heartbeat", "--identity", "a", "--require-peer-reachable"])

    assert result == 0
    assert calls
    assert calls[0].session_dir == "1_heartbeat"
    assert calls[0].identity == "a"
    assert calls[0].require_peer_reachable is True


def test_probe_verbs_dispatch_not_wrapped_in_start(monkeypatch: pytest.MonkeyPatch) -> None:
    # Regression guard: probe-* must be in main()'s command set, otherwise
    # the "bare positional -> start" shim rewrites them as `start <verb> ...`.
    seen: list[tuple[str, argparse.Namespace]] = []

    def record(name: str) -> Callable[[argparse.Namespace], int]:
        def _cmd(args: argparse.Namespace) -> int:
            seen.append((name, args))
            return 0

        return _cmd

    for name in ("probe_publish_command", "probe_check_command", "publish_test_topics_command"):
        monkeypatch.setattr(rosotacom, name, record(name))

    assert rosotacom.main(["probe-publish", "1_heartbeat", "--identity", "a"]) == 0
    assert rosotacom.main(["probe-check", "1_heartbeat", "--identity", "b", "--expect", "absent"]) == 0
    assert rosotacom.main(["publish-test-topics", "1_heartbeat", "--identity", "b"]) == 0

    assert [n for n, _ in seen] == [
        "probe_publish_command",
        "probe_check_command",
        "publish_test_topics_command",
    ]
    assert all(args.session_dir == "1_heartbeat" for _, args in seen)
    assert seen[1][1].expect == "absent"


def test_probe_publish_parser_advertise_timeout(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[argparse.Namespace] = []
    monkeypatch.setattr(rosotacom, "probe_publish_command", lambda args: calls.append(args) or 0)

    assert rosotacom.main(["probe-publish", "1_heartbeat", "--identity", "a"]) == 0
    assert calls[0].advertise_timeout == rosotacom._PROBE_ADVERTISE_TIMEOUT_S
    assert calls[0].duration == rosotacom._PROBE_DURATION_S

    assert rosotacom.main(["probe-publish", "1_heartbeat", "--identity", "a", "--advertise-timeout", "90"]) == 0
    assert calls[1].advertise_timeout == 90.0


def _probe_publish_args(**overrides: object) -> argparse.Namespace:
    args = argparse.Namespace(
        identity="center",
        stop=False,
        topic="/local_only",
        rate=5.0,
        duration=rosotacom._PROBE_DURATION_S,
        advertise_timeout=rosotacom._PROBE_ADVERTISE_TIMEOUT_S,
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def test_probe_publish_waits_past_slow_polls_until_advertised(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(rosotacom, "_resolve_running_peer", lambda args, identity: ("box", "setup", {}))
    monkeypatch.setattr(rosotacom, "_publish_isolation_probe", lambda *a, **k: None)
    monkeypatch.setattr(rosotacom, "_isolation_probe_running", lambda *a: True)
    monkeypatch.setattr(rosotacom.time, "sleep", lambda s: None)
    present = iter([False] * 15 + [True])
    monkeypatch.setattr(rosotacom, "_topic_present", lambda *a: next(present))

    assert rosotacom.probe_publish_command(_probe_publish_args()) == 0
    assert "advertised." in capsys.readouterr().out


def test_probe_publish_failure_names_slow_graph_not_publisher(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    # Publisher alive but the deadline passes: the message must blame graph
    # discovery, not the publisher (issue #219).
    monkeypatch.setattr(rosotacom, "_resolve_running_peer", lambda args, identity: ("box", "setup", {}))
    monkeypatch.setattr(rosotacom, "_publish_isolation_probe", lambda *a, **k: None)
    monkeypatch.setattr(rosotacom, "_isolation_probe_running", lambda *a: True)
    monkeypatch.setattr(rosotacom, "_topic_present", lambda *a: False)

    assert rosotacom.probe_publish_command(_probe_publish_args(advertise_timeout=0.0)) == 1
    err = capsys.readouterr().err
    assert "is running" in err
    assert "did not report it within" in err
    assert "--advertise-timeout" in err


def test_probe_publish_failure_names_dead_publisher(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(rosotacom, "_resolve_running_peer", lambda args, identity: ("box", "setup", {}))
    monkeypatch.setattr(rosotacom, "_publish_isolation_probe", lambda *a, **k: None)
    monkeypatch.setattr(rosotacom, "_isolation_probe_running", lambda *a: False)
    monkeypatch.setattr(rosotacom, "_topic_present", lambda *a: False)

    assert rosotacom.probe_publish_command(_probe_publish_args()) == 1
    assert "exited or never started" in capsys.readouterr().err


def test_wait_for_probe_advertisement_outcomes(monkeypatch: pytest.MonkeyPatch) -> None:
    sleeps: list[float] = []
    monkeypatch.setattr(rosotacom.time, "sleep", lambda s: sleeps.append(s))
    monkeypatch.setattr(rosotacom, "_isolation_probe_running", lambda *a: True)
    present = iter([False, False, True])
    monkeypatch.setattr(rosotacom, "_topic_present", lambda *a: next(present))
    assert rosotacom._wait_for_probe_advertisement("box", "setup", "/local_only", timeout_s=60.0) == "advertised"
    assert len(sleeps) == 2

    monkeypatch.setattr(rosotacom, "_topic_present", lambda *a: False)
    assert rosotacom._wait_for_probe_advertisement("box", "setup", "/local_only", timeout_s=0.0) == "timeout"

    monkeypatch.setattr(rosotacom, "_isolation_probe_running", lambda *a: False)
    assert rosotacom._wait_for_probe_advertisement("box", "setup", "/local_only", timeout_s=60.0) == "publisher-gone"


def test_verify_isolation_inconclusive_distinguishes_causes(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(rosotacom, "_publish_isolation_probe", lambda *a, **k: None)
    stopped: list[str] = []
    monkeypatch.setattr(rosotacom, "_stop_isolation_probe", lambda container, topic: stopped.append(container))

    monkeypatch.setattr(rosotacom, "_wait_for_probe_advertisement", lambda *a, **k: "timeout")
    errors = rosotacom._verify_isolation("pub", "ps", "chk", "cs", "/local_only", log_line=lambda _: None)
    assert len(errors) == 1
    assert "is running in pub" in errors[0]
    assert "did not report it within" in errors[0]

    monkeypatch.setattr(rosotacom, "_wait_for_probe_advertisement", lambda *a, **k: "publisher-gone")
    errors = rosotacom._verify_isolation("pub", "ps", "chk", "cs", "/local_only", log_line=lambda _: None)
    assert errors == ["isolation check inconclusive: probe publisher for /local_only exited or never started in pub"]
    assert stopped == ["pub", "pub"]


def test_expect_from_bag_fragment_passes_rosotacom_test(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    project = tmp_path / "project"
    session_dir = project / "sessions" / "whole_bag"
    bag_dir = project / "bags" / "fixture_bag"
    instance_dir = project / "session-instances" / "2026-01-01" / "whole_bag_2026-01-01_00-00-00_run1"
    session_dir.mkdir(parents=True)
    bag_dir.mkdir(parents=True)
    (project / "ros2docker.json").write_text('{"image_name": "unit"}\n', encoding="utf-8")
    (project / "rosotacom.yaml").write_text(
        "ros2docker_config: ros2docker.json\n"
        "session_configs_dir:\n"
        "  - sessions\n"
        "session_instances_dir: session-instances\n",
        encoding="utf-8",
    )
    (bag_dir / "metadata.yaml").write_text(
        """\
rosbag2_bagfile_information:
  duration: {nanoseconds: 10000000000}
  topics_with_message_count:
    - topic_metadata:
        name: /stream
        type: std_msgs/msg/String
        offered_qos_profiles:
          - {reliability: reliable, durability: volatile}
      message_count: 101
""",
        encoding="utf-8",
    )
    session_cfg = {
        "peers": {"a": {}, "b": {}},
        "shared": {"use_status_overview": True},
        "topics": {
            "b_to_a": [
                {
                    "topic": "/stream",
                    "type": "std_msgs/msg/String",
                    "processing": {"drop": {"drop_count": 1, "window_size": 2}},
                }
            ]
        },
    }
    session_path = session_dir / "session-definition.yaml"
    session_path.write_text(yaml.safe_dump(session_cfg, sort_keys=False), encoding="utf-8")

    generated_path = tmp_path / "whole-bag-expect.yaml"
    rc = rosotacom.main(
        [
            "expect",
            "from-bag",
            str(bag_dir),
            "--session",
            "whole_bag",
            "--project",
            str(project / "rosotacom.yaml"),
            "--out",
            str(generated_path),
            "--min-ratio",
            "0.8",
        ]
    )

    assert rc == 0
    assert "Wrote 1 generated expect block" in capsys.readouterr().out
    generated = yaml.safe_load(generated_path.read_text(encoding="utf-8"))
    session_cfg["topics"]["b_to_a"][0]["expect"] = generated["topics"]["b_to_a"][0]["expect"]
    session_path.write_text(yaml.safe_dump(session_cfg, sort_keys=False), encoding="utf-8")

    status_dir = instance_dir / "logs" / "a" / "status"
    status_dir.mkdir(parents=True)
    status = {
        "peer": "a",
        "topics": [
            {
                "base": "/stream",
                "direction": "inbound",
                "overall": "OK",
                "stages": [
                    {"stage": "com_in", "state": "FLOWING", "hz": 5.0, "messages_total": 50, "publishers": 1},
                    {"stage": "app_in", "state": "FLOWING", "hz": 5.0, "messages_total": 45, "publishers": 1},
                ],
            }
        ],
    }
    (status_dir / "status.json").write_text(json.dumps(status), encoding="utf-8")

    assert (
        rosotacom.main(
            [
                "test",
                "whole_bag",
                "--project",
                str(project / "rosotacom.yaml"),
                "--instance-id",
                "run1",
                "--bag",
                str(bag_dir),
                "--timeout",
                "0",
            ]
        )
        == 0
    )
    assert "TEST OK" in capsys.readouterr().out


def test_the_retired_compat_entrypoints_are_gone() -> None:
    """Removed on 2026-08-14 — they were `start`/`stop` under a second name.

    Asserting their absence is the point: a re-added alias would otherwise pass
    every other test in this file silently.
    """
    assert not hasattr(rosotacom, "start_compat_main")
    assert not hasattr(rosotacom, "stop_compat_main")


def test_path_yaml_and_network_helpers(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    invalid_yaml = tmp_path / "list.yaml"
    invalid_yaml.write_text("- not\n- mapping\n", encoding="utf-8")

    assert rosotacom._load_yaml_file(None) == {}
    assert rosotacom._load_yaml_file(tmp_path / "missing.yaml") == {}
    with pytest.raises(RuntimeError, match="must contain a mapping"):
        rosotacom._load_yaml_file(invalid_yaml)
    assert rosotacom._first_value(None, "", "chosen") == "chosen"
    assert rosotacom._resolve_path("~/missing", tmp_path, must_exist=False).is_absolute()
    with pytest.raises(FileNotFoundError):
        rosotacom._resolve_path("missing", tmp_path, must_exist=True)
    assert rosotacom._sanitize_docker_name("a/b c") == "a_b_c"
    assert len(rosotacom._install_id(tmp_path)) == 8

    def fake_check_output(command: list[str], *, text: bool) -> str:
        if command[:4] == ["ip", "-o", "-4", "addr"]:
            return "1: lo inet 127.0.0.1/8\n2: eth0 inet 10.0.0.5/24 brd 10.0.0.255 scope global eth0\n"
        return "1.1.1.1 via 10.0.0.1 dev eth0 src 10.0.0.5 uid 1000"

    monkeypatch.setattr(rosotacom.subprocess, "check_output", fake_check_output)

    assert rosotacom._get_local_ipv4s() == ["127.0.0.1", "10.0.0.5"]
    assert rosotacom._default_local_ip() == "10.0.0.5"


def test_peer_binding_identity_and_command_helpers(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    deployment = tmp_path / "deployment.yaml"
    deployment.write_text(
        "\n".join(
            [
                "hosts:",
                "  local_box: {address: 10.0.0.1, ssh: null}",
                "  remote_box: {address: 10.0.0.2, ssh: remote}",
                "",
            ]
        ),
        encoding="utf-8",
    )
    runtime = rosotacom.RuntimeConfig(
        rosotacom_config=tmp_path / "rosotacom.yaml",
        ros2docker_config=rosotacom.DEFAULT_ROS2DOCKER_CONFIG,
        session_configs_dir=(),
        deployment=deployment,
        install_id="abc",
    )
    cfg = {
        "peers": {
            "a": {"com-name": "local unit"},
            "b": {"com-name": "remote/unit"},
        }
    }

    bindings = rosotacom._resolve_bindings(
        cfg,
        runtime,
        peer=["a=local_box", "b=remote_box"],
    )
    assert bindings["a"] == rosotacom.PeerBinding("a", "10.0.0.1", None, "local_box")
    assert bindings["b"] == rosotacom.PeerBinding("b", "10.0.0.2", "remote", "remote_box")
    monkeypatch.setattr(rosotacom, "_get_local_ipv4s", lambda: ["10.0.0.1"])
    assert rosotacom._auto_identity(bindings) == "a"
    assert rosotacom._remote_peer_name(cfg, "a") == "remote/unit"
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            ("rosotacom_abc_run1_com_to_remote_unit", ["bridge"]),
            ("rosotacom_abc_run2_com_to_remote_unit", ["bridge"]),
            ("rosotacom_other_run1_com_to_remote_unit", ["bridge"]),
            ("rosotacom_abc_run1_com_to_elsewhere", ["bridge"]),
        ],
    )
    assert rosotacom._identity_container_names(cfg, runtime, "a") == [
        "rosotacom_abc_run1_com_to_remote_unit",
        "rosotacom_abc_run2_com_to_remote_unit",
    ]

    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    instance = rosotacom.SessionInstance(
        instance_id="run1",
        host_dir=tmp_path / "instances" / "2026-01-01" / "1_heartbeat_run1",
        container_dir="/session/instances/2026-01-01/1_heartbeat_run1",
        config_host_dir=tmp_path / "instances" / "2026-01-01" / "1_heartbeat_run1" / "config",
        config_container_dir="/session/instances/2026-01-01/1_heartbeat_run1/config",
        logs_host_dir=tmp_path / "instances" / "2026-01-01" / "1_heartbeat_run1" / "logs",
        logs_container_dir="/session/instances/2026-01-01/1_heartbeat_run1/logs",
        rosbags_host_dir=tmp_path / "instances" / "2026-01-01" / "1_heartbeat_run1" / "rosbags",
        rosbags_container_dir="/session/instances/2026-01-01/1_heartbeat_run1/rosbags",
    )
    assert rosotacom._session_command(
        session,
        instance,
        "a",
        force=True,
        rewrite_formatting=True,
        peer_address_overrides={"a": "10.0.0.1", "b": "10.0.0.2"},
        attach_mode="attach",
    ) == [
        "python3",
        "/ws/session/creation/run_session.py",
        "--session-dir",
        "/session/current",
        "--output-dir",
        "/session/instances/2026-01-01/1_heartbeat_run1/config",
        "--instance-dir",
        "/session/instances/2026-01-01/1_heartbeat_run1",
        "--catmux-log-dir",
        "/session/instances/2026-01-01/1_heartbeat_run1/logs/a/catmux",
        "--rosbag-dir",
        "/session/instances/2026-01-01/1_heartbeat_run1/rosbags/a",
        "--identity",
        "a",
        "--force",
        "--rewrite-formatting",
        "--peer-address",
        "a=10.0.0.1",
        "--peer-address",
        "b=10.0.0.2",
        "--attach",
    ]

    traced_command = rosotacom._session_command(
        session,
        instance,
        "a",
        force=False,
        rewrite_formatting=False,
        peer_address_overrides={},
        attach_mode="detached",
        link_trace=True,
        link_trace_interval_s=0.5,
        link_trace_modem_command="cat /tmp/modem.json",
    )
    assert "--link-trace" in traced_command
    assert traced_command[traced_command.index("--link-trace-interval") + 1] == "0.5"
    assert traced_command[traced_command.index("--link-trace-modem-command") + 1] == "cat /tmp/modem.json"

    with pytest.raises(RuntimeError, match="Duplicate"):
        rosotacom._parse_peer_address_overrides(["a=1.1.1.1", "a=2.2.2.2"])


def test_container_scripts_run_through_the_interpreter() -> None:
    """No `docker exec` may depend on a file's executable bit.

    A wheel does not carry executable bits for package data, so a bare script
    path works from a checkout and fails from an install with

        exec: "/ws/session/creation/run_session.py": permission denied

    and exit 126. That broke `rosotacom start` on every packaged machine —
    bench, control centre, vehicle — while the development laptop's editable
    install kept working, which is why it survived to a two-host bring-up on
    2026-08-09 before anyone saw it.

    Asserted as a property rather than as one literal command, so a second
    script added later fails here instead of on a machine.

    The first version of this test looked only at `cli.py`, and the session
    templates carried two more of exactly the same call — `get_ota_xml.py` and
    `create_zenoh_json5.py`. Those failed the same way and cost the same
    bring-up a second time: the redirect made the failure silent, leaving a
    zero-byte DDS profile, so the symptom was an unexplained XML parser error
    and an OTA layer that never came up. Anything that runs inside a container
    is in scope here, not just what `cli.py` spawns.
    """
    assert rosotacom.RUN_SESSION_CONTAINER_ARGV[0] == "python3"
    assert rosotacom.RUN_SESSION_CONTAINER_ARGV[1].endswith("/run_session.py")

    # The session content templates run commands in the container too, and a
    # failure there is worse than an exec error: the call is redirected into a
    # config file, so a non-executable script leaves an empty one and the next
    # component reports a parse error instead of a missing interpreter.
    content = Path(rosotacom.__file__).parent / "resources" / "ws" / "session" / "content"
    for template in sorted(content.rglob("*.yaml")):
        text = template.read_text(encoding="utf-8")
        for match in re.finditer(r"(?<![\w/])(/ws/[\w/.-]+\.py)", text):
            preceding = text[max(0, match.start() - 40) : match.start()]
            assert "python3 " in preceding, (
                f"{template.name}: {match.group(1)} is run by path. In a wheel "
                "install it is not executable, and here the output is "
                "redirected into a config file, so the failure is silent."
            )

    # And the same for any container script added later. Comment lines are
    # dropped first: the explanation above quotes the failing path, and a test
    # that reads prose as code would fail on the wrong thing.
    code = "\n".join(
        line
        for line in Path(rosotacom.__file__).read_text(encoding="utf-8").splitlines()
        if not line.lstrip().startswith("#")
    )
    for match in re.finditer(r'"(/ws/[^"]*\.py)"', code):
        preceding = code[max(0, match.start() - 200) : match.start()]
        assert '"python3"' in preceding, (
            f"{match.group(1)} is handed to the container without an interpreter; "
            "a wheel install has no executable bit and this fails with exit 126"
        )


def test_resolve_session_and_base_extra_run_args(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    session = tmp_path / "sessions" / "1_heartbeat"
    session.mkdir(parents=True)
    (session / "session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    runtime = rosotacom.RuntimeConfig(
        rosotacom_config=tmp_path / "rosotacom.yaml",
        ros2docker_config=tmp_path / "ros2docker.json",
        session_configs_dir=(tmp_path / "sessions",),
        deployment=None,
        install_id="id",
        session_instances_dir=tmp_path / "session-instances",
    )

    resolved = rosotacom._resolve_session("1_heartbeat", runtime)
    instance = rosotacom._resolve_session_instance(runtime, resolved, "test-run")
    monkeypatch.setattr(rosotacom, "load_config", lambda config: {"mount_ws": False}, raising=False)
    args = rosotacom._base_extra_run_args(
        runtime,
        resolved,
        {"peers": {"a": {}, "b": {}}},
        instance,
    )

    assert resolved.container_dir == "/session/definitions/1_heartbeat"
    assert f"{rosotacom.WS_DIR.resolve()}:/ws" in args
    assert f"{runtime.session_configs_dir[0]}:/session/definitions:ro" in args
    assert "--cpuset-cpus" not in args

    # A cpuset confines every container of the session. Without this the CPUs a
    # shared workstation reserved are merely idle, not used by the run.
    pinned = rosotacom._base_extra_run_args(runtime, resolved, {"peers": {"a": {}, "b": {}}}, instance, cpuset=" 0-3 ")
    assert pinned[pinned.index("--cpuset-cpus") + 1] == "0-3"

    # It is validated here too, not only where it is parsed: this is the last
    # place before the string becomes part of a docker command line.
    with pytest.raises(RuntimeError):
        rosotacom._base_extra_run_args(
            runtime, resolved, {"peers": {"a": {}, "b": {}}}, instance, cpuset="0-3; rm -rf /"
        )
    assert f"{runtime.session_instances_dir}:/session/instances" in args
    assert "Configured sessions:" in rosotacom._format_available_sessions(runtime)
    # A peer shares the host IPC namespace, so it can exchange local-domain
    # data with application containers on the same machine. See
    # `_local_ipc_run_args` for what happens without it.
    assert args[:2] == ["--ipc", "host"]


def test_a_peer_shares_the_host_ipc_namespace_unless_isolated() -> None:
    """Without it, a peer receives nothing from a local application container.

    `--network host` with `ipc=private` gives every container its own
    /dev/shm. FastDDS sees two participants on one host, chooses shared memory,
    and the reader never maps the writer's segment. Discovery succeeds, delivery
    does not, and nothing in the output says so: the link comes up, the
    heartbeat crosses at 10 Hz with 0% loss, and no payload ever enters it.

    Isolated on 2026-08-09 with two containers from one image on one host:
    `ipc=private` received 0 messages, `--ipc host` received 200.
    """
    assert rosotacom._local_ipc_run_args(network_isolated=False) == ["--ipc", "host"]
    # Under network isolation it must not be shared: SHM would carry data
    # straight past the boundary the isolation exists to simulate, and a test
    # that passes through a hole in its own isolation is worse than none.
    assert rosotacom._local_ipc_run_args(network_isolated=True) == []


def test_container_helpers_use_docker_and_ros2docker_stop(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    runtime = rosotacom.RuntimeConfig(None, tmp_path / "ros2docker.json", (), None, "id")
    calls: list[object] = []

    monkeypatch.setattr(rosotacom, "_require_ros2docker", lambda: None)
    monkeypatch.setattr(rosotacom, "_container_exists", lambda name: False)
    assert not rosotacom._stop_container_name("missing", runtime, quiet_missing=True)

    monkeypatch.setattr(rosotacom, "_container_exists", lambda name: True)
    monkeypatch.setattr(rosotacom, "ros2docker_stop", lambda **kwargs: calls.append(kwargs), raising=False)
    assert rosotacom._stop_container_name("running", runtime)
    assert calls == [{"config_file": runtime.ros2docker_config, "override": {"container_name": "running"}}]

    def fake_run(command: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(command)
        if command[:2] == ["docker", "logs"]:
            return subprocess.CompletedProcess(command, 0, "Sourced ROS 2 workspace overlay", "")
        return subprocess.CompletedProcess(command, 0, "true\n", "")

    monkeypatch.setattr(rosotacom.subprocess, "run", fake_run)
    rosotacom._wait_for_container_ready("ready", timeout_s=1)
    assert ["docker", "logs", "ready"] in calls


def test_start_session_detached_dispatches_ros2docker(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    runtime = rosotacom.RuntimeConfig(None, tmp_path / "ros2docker.json", (), None, "id", tmp_path / "instances")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {"com-name": "remote"}}}
    calls: list[tuple[str, dict[str, object]]] = []

    monkeypatch.setattr(rosotacom, "_require_ros2docker", lambda: None)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_resolve_session", lambda session_dir, runtime: session)
    monkeypatch.setattr(rosotacom, "_effective_session_config", lambda *args, **kwargs: cfg)
    monkeypatch.setattr(
        rosotacom,
        "_network_preflight",
        lambda *args, **kwargs: rosotacom.NetworkPreflightResult(
            "a", "127.0.0.1", "b", "127.0.0.2", "lo", "127.0.0.1", None
        ),
    )
    monkeypatch.setattr(rosotacom, "_print_network_preflight", lambda result: None)
    monkeypatch.setattr(rosotacom, "_scoped_image_name", lambda runtime: "image:id")
    monkeypatch.setattr(
        rosotacom,
        "_base_extra_run_args",
        lambda runtime, session, cfg, instance, **kwargs: ["--network", "host"],
    )
    monkeypatch.setattr(rosotacom, "_resolve_mode", lambda mode: "detached")
    monkeypatch.setattr(rosotacom, "_list_docker_containers", lambda all_states=False: [])
    monkeypatch.setattr(rosotacom, "_stop_container_name", lambda *args, **kwargs: True)
    monkeypatch.setattr(rosotacom, "_wait_for_container_ready", lambda name: None)
    monkeypatch.setattr(rosotacom, "_write_docker_log", lambda *args, **kwargs: None)
    monkeypatch.setattr(rosotacom, "ros2docker_build", lambda **kwargs: calls.append(("build", kwargs)), raising=False)
    monkeypatch.setattr(rosotacom, "ros2docker_run", lambda **kwargs: calls.append(("run", kwargs)), raising=False)
    monkeypatch.setattr(rosotacom, "ros2docker_exec", lambda **kwargs: calls.append(("exec", kwargs)), raising=False)

    container = rosotacom.start_session(
        argparse.Namespace(
            session_dir="1_heartbeat",
            peer=[],
            peer_address=["a=127.0.0.1", "b=127.0.0.2"],
            identity="a",
            auto_identity=True,
            mode="detached",
            force=True,
            rewrite_formatting=False,
            instance_id="unit",
        )
    )

    assert container == "rosotacom_id_unit_com_to_remote"
    assert [name for name, _ in calls] == ["build", "run", "exec"]
    assert calls[1][1]["override"]["run_type"] == "up"
    assert calls[2][1]["interactive"] is False
    assert "--output-dir" in calls[2][1]["command"]


def test_start_session_attach_dispatches_command_run(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    runtime = rosotacom.RuntimeConfig(None, tmp_path / "ros2docker.json", (), None, "id", tmp_path / "instances")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {}}}
    calls: list[tuple[str, dict[str, object]]] = []

    monkeypatch.setattr(rosotacom, "_require_ros2docker", lambda: None)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_resolve_session", lambda session_dir, runtime: session)
    monkeypatch.setattr(rosotacom, "_effective_session_config", lambda *args, **kwargs: cfg)
    monkeypatch.setattr(rosotacom, "_auto_identity", lambda *args: "a")
    monkeypatch.setattr(
        rosotacom,
        "_network_preflight",
        lambda *args, **kwargs: rosotacom.NetworkPreflightResult(
            "a", "127.0.0.1", "b", "127.0.0.2", "lo", "127.0.0.1", None
        ),
    )
    monkeypatch.setattr(rosotacom, "_print_network_preflight", lambda result: None)
    monkeypatch.setattr(rosotacom, "_scoped_image_name", lambda runtime: "image:id")
    monkeypatch.setattr(rosotacom, "_base_extra_run_args", lambda runtime, session, cfg, instance, **kwargs: [])
    monkeypatch.setattr(rosotacom, "_resolve_mode", lambda mode: "attach")
    monkeypatch.setattr(rosotacom, "_list_docker_containers", lambda all_states=False: [])
    monkeypatch.setattr(rosotacom, "_stop_container_name", lambda *args, **kwargs: True)
    monkeypatch.setattr(rosotacom, "_write_docker_log", lambda *args, **kwargs: None)
    monkeypatch.setattr(rosotacom, "ros2docker_build", lambda **kwargs: calls.append(("build", kwargs)), raising=False)
    monkeypatch.setattr(rosotacom, "ros2docker_run", lambda **kwargs: calls.append(("run", kwargs)), raising=False)

    rosotacom.start_session(
        argparse.Namespace(
            session_dir="1_heartbeat",
            peer=[],
            peer_address=["a=127.0.0.1", "b=127.0.0.2"],
            identity=None,
            auto_identity=True,
            mode="attach",
            force=True,
            rewrite_formatting=False,
            instance_id="unit",
        )
    )

    assert [name for name, _ in calls] == ["build", "run"]
    assert calls[1][1]["override"]["run_type"] == "command"
    assert calls[1][1]["override"]["tty"] is True


def test_stop_list_doctor_and_smoke_host_flows(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    session_dir = tmp_path / "sessions" / "1_heartbeat"
    (session_dir / "a").mkdir(parents=True)
    (session_dir / "b").mkdir()
    (session_dir / "session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    (session_dir / "a" / "plugin.yaml").write_text("127.0.0.1\n", encoding="utf-8")
    (session_dir / "b" / "plugin.yaml").write_text("127.0.0.1\n", encoding="utf-8")
    runtime = rosotacom.RuntimeConfig(
        None,
        tmp_path / "ros2docker.json",
        (tmp_path / "sessions",),
        None,
        "id",
        tmp_path / "session-instances",
    )
    session = rosotacom.ResolvedSession(session_dir, "/session/definitions/1_heartbeat", "session_configs")
    instance = rosotacom.SessionInstance(
        instance_id="smoke",
        host_dir=tmp_path / "session-instances" / "2026-01-01" / "1_heartbeat_smoke",
        container_dir="/session/instances/2026-01-01/1_heartbeat_smoke",
        config_host_dir=tmp_path / "session-instances" / "2026-01-01" / "1_heartbeat_smoke" / "config",
        config_container_dir="/session/instances/2026-01-01/1_heartbeat_smoke/config",
        logs_host_dir=tmp_path / "session-instances" / "2026-01-01" / "1_heartbeat_smoke" / "logs",
        logs_container_dir="/session/instances/2026-01-01/1_heartbeat_smoke/logs",
        rosbags_host_dir=tmp_path / "session-instances" / "2026-01-01" / "1_heartbeat_smoke" / "rosbags",
        rosbags_container_dir="/session/instances/2026-01-01/1_heartbeat_smoke/rosbags",
    )
    smoke_network = rosotacom._noninteractive_smoke_network_config(runtime, session, instance.instance_id)
    (instance.config_host_dir / "a").mkdir(parents=True)
    (instance.config_host_dir / "b").mkdir()
    (instance.config_host_dir / "a" / "plugin.yaml").write_text(f"{smoke_network.peer_ips['a']}\n", encoding="utf-8")
    (instance.config_host_dir / "b" / "plugin.yaml").write_text(f"{smoke_network.peer_ips['b']}\n", encoding="utf-8")
    cfg = {"peers": {"a": {}, "b": {}}}
    stopped: list[str] = []

    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_resolve_session", lambda session_dir, runtime: session)
    monkeypatch.setattr(rosotacom, "_resolve_session_instance", lambda runtime, session, instance_id=None: instance)
    monkeypatch.setattr(rosotacom, "_effective_session_config", lambda *args, **kwargs: cfg)
    monkeypatch.setattr(
        rosotacom,
        "_identity_container_names",
        lambda cfg, runtime, identity=None, all_states=False: ["c1", "c2"],
    )
    monkeypatch.setattr(rosotacom, "_stop_container_name", lambda name, runtime, **kwargs: stopped.append(name) or True)
    monkeypatch.setattr(rosotacom, "_ROS2DOCKER_IMPORT_ERROR", None)
    fake_ros2docker = type("FakeRos2Docker", (), {"__version__": "test", "__file__": __file__})
    monkeypatch.setattr(rosotacom, "ros2docker", fake_ros2docker)
    monkeypatch.setattr(rosotacom, "load_config", lambda config: {"mount_ws": True}, raising=False)
    monkeypatch.setattr(rosotacom, "_scoped_image_name", lambda runtime: "image:id")

    rosotacom.stop_session(argparse.Namespace(session_dir="1_heartbeat", identity=None, auto_identity=False))
    rosotacom.list_sessions(argparse.Namespace())
    assert stopped == ["c1", "c2"]
    assert "Configured sessions:" in capsys.readouterr().out

    assert rosotacom.doctor(argparse.Namespace()) == 0
    assert "OK: ros2docker validation: config loads" in capsys.readouterr().out

    started: list[argparse.Namespace] = []
    networks_created: list[tuple[str, str]] = []
    networks_removed: list[str] = []
    monkeypatch.setattr(rosotacom, "start_session", lambda args: started.append(args) or f"container_{args.identity}")
    monkeypatch.setattr(
        rosotacom,
        "_ensure_smoke_network",
        lambda name, subnet, labels=None: networks_created.append((name, subnet)),
    )
    monkeypatch.setattr(rosotacom, "_remove_smoke_network", lambda name: networks_removed.append(name))
    monkeypatch.setattr(rosotacom, "_matching_smoke_networks", lambda runtime, target_key: [])
    # Smoke's per-container delivery + isolation verification is exercised
    # end-to-end in tests/e2e; this unit test only drives the host flow
    # (start/stop/network), so stub the shared verification helpers as passing.
    monkeypatch.setattr(rosotacom, "_verify_received_topics", lambda *args, **kwargs: [])
    monkeypatch.setattr(rosotacom, "_verify_isolation", lambda *args, **kwargs: [])
    monkeypatch.setattr(rosotacom, "test_command", lambda args: 0)
    publisher_durations: list[float] = []

    def fake_start_publishers(*args: object, **kwargs: object) -> list[rosotacom.SmokeTopicSpec]:
        publisher_durations.append(float(kwargs["duration"]))
        return []

    monkeypatch.setattr(rosotacom, "_start_smoke_topic_publishers", fake_start_publishers)
    assert (
        rosotacom.smoke(
            argparse.Namespace(
                local=True,
                local_ip="127.0.0.1",
                session_dir="1_heartbeat",
                rosotacom_config=None,
                ros2docker_config=None,
                session_configs_dir=None,
                session_instances_dir=None,
                deployment=None,
                instance_id="smoke",
                keep_running=False,
            )
        )
        == 0
    )
    assert publisher_durations == [rosotacom.SMOKE_PUBLISHER_DURATION_S]
    assert rosotacom.SMOKE_PUBLISHER_DURATION_S == 3600.0
    assert networks_created == [(smoke_network.name, smoke_network.subnet)]
    assert [(args.identity, args.network_name, args.network_ip) for args in started] == [
        ("a", smoke_network.name, smoke_network.peer_ips["a"]),
        ("b", smoke_network.name, smoke_network.peer_ips["b"]),
    ]
    assert all(args.peer_address == rosotacom._smoke_peer_address_args(smoke_network.peer_ips) for args in started)
    assert networks_removed == [smoke_network.name]
    assert stopped[-2:] == ["container_a", "container_b"]


def test_smoke_peer_addresses_use_isolated_bridge_ips() -> None:
    # Smoke isolates the two peers in their own network namespaces on a dedicated
    # docker bridge with distinct IPs, instead of sharing the host loopback.
    assert rosotacom._smoke_peer_address_args() == ["a=10.137.0.2", "b=10.137.0.3"]
    assert rosotacom.SMOKE_PEER_IPS == {"a": "10.137.0.2", "b": "10.137.0.3"}


def test_noninteractive_smoke_network_config_is_instance_scoped(tmp_path: Path) -> None:
    sessions_root = tmp_path / "sessions"
    session_dir = sessions_root / "1_heartbeat"
    session_dir.mkdir(parents=True)
    runtime = rosotacom.RuntimeConfig(
        None,
        tmp_path / "ros2docker.json",
        (sessions_root,),
        None,
        "id",
        tmp_path / "session-instances",
    )
    session = rosotacom.ResolvedSession(session_dir, "/session/definitions/1_heartbeat", "session_configs")

    first = rosotacom._noninteractive_smoke_network_config(runtime, session, "first")
    second = rosotacom._noninteractive_smoke_network_config(runtime, session, "second")

    assert first.name != second.name
    assert first.subnet != second.subnet
    assert first.name.startswith("rosotacom_smoke_id_1_heartbeat_first")
    assert first.subnet.startswith("10.137.")
    assert first.subnet.endswith("/29")
    assert first.peer_ips == rosotacom._smoke_peer_ips_for_subnet(["a", "b"], first.subnet)


def test_smoke_peer_ips_for_subnet_supports_small_run_subnets() -> None:
    assert rosotacom._smoke_peer_ips_for_subnet(["b", "a"], "10.137.42.16/29") == {
        "a": "10.137.42.18",
        "b": "10.137.42.19",
    }


def test_isolated_network_run_args_swaps_host_networking() -> None:
    base = ["-e", "ROS_DOMAIN_ID=48", "--network", "host"]
    swapped = rosotacom._isolated_network_run_args(base, "rosotacom-smoke", "10.137.0.2")
    assert "host" not in swapped
    assert swapped == [
        "-e",
        "ROS_DOMAIN_ID=48",
        "--network",
        "rosotacom-smoke",
        "--cap-add",
        "NET_ADMIN",
        "--ip",
        "10.137.0.2",
    ]
    # A config that already pins --network=... form is also replaced.
    assert rosotacom._isolated_network_run_args(["--network=host"], "net", None) == [
        "--network",
        "net",
        "--cap-add",
        "NET_ADMIN",
    ]


def test_smoke_crossed_topics_include_native_chatter_direction() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "2_native_chatter" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )

    specs_a = rosotacom._received_crossed_topics(cfg, "a")
    specs_b = rosotacom._received_crossed_topics(cfg, "b")

    assert [(s.topic, s.label, s.publish_topic, s.publish_type) for s in specs_a] == [
        ("/com/in/b/chatter", "b->a inbound bridge topic", None, None),
        ("/chatter", "b->a final topic", "/chatter", "std_msgs/msg/String"),
    ]
    assert specs_b == []
    assert [(s.source_peer_key, s.receiver_peer_key, s.publish_topic) for s in rosotacom._smoke_publish_specs(cfg)] == [
        ("b", "a", "/chatter")
    ]
    assert [
        (s.source_peer_key, s.receiver_peer_key, s.publish_topic)
        for s in rosotacom._smoke_publish_specs(cfg, source_peer_key="b")
    ] == [("b", "a", "/chatter")]
    assert rosotacom._smoke_publish_specs(cfg, source_peer_key="a") == []
    setup_a = rosotacom._smoke_ros_setup("/config", cfg, "a")
    setup_b = rosotacom._smoke_ros_setup("/config", cfg, "b")
    assert "export ROS_DOMAIN_ID=46" in setup_a
    assert "export ROS_DOMAIN_ID=47" in setup_b
    assert "export CYCLONEDDS_URI=" in setup_a
    assert "MaxAutoParticipantIndex>99<" in setup_a


def test_smoke_ros_setup_keeps_explicit_cyclonedds_config() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "1_heartbeat_status" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )

    setup = rosotacom._smoke_ros_setup("/config", cfg, "a")

    assert "export CYCLONEDDS_URI=file:///config/a/local_dds.xml" in setup
    assert "MaxAutoParticipantIndex" not in setup


def test_smoke_ros_setup_on_a_scoped_cyclone_arrangement_is_the_environment_measured_to_deliver() -> None:
    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 50}, "b": {"domain_id": 51}},
        "shared": {"ota_domain_id": 52, "rmw": {"ota": {"cyclone": {"config": "cyclonedds_scoped.xml"}}}},
    }

    setup = rosotacom._smoke_ros_setup("/config", cfg, "a")

    # The session renders no local profile, so the publisher keeps the inline
    # default; on cyclone beside the session that delivered 1486 of 1486 messages
    # vehicle -> center under Lyrical (#346). The OTA file is not the local one.
    assert "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in setup
    assert "export ROS_DOMAIN_ID=50" in setup
    assert "MaxAutoParticipantIndex>99<" in setup
    assert "ota_dds.xml" not in setup


def test_smoke_ros_setup_keeps_the_inline_default_beside_an_unscoped_ota_profile() -> None:
    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 50}, "b": {"domain_id": 51}},
        "shared": {"ota_domain_id": 52, "rmw": "cyclone"},
    }

    setup = rosotacom._smoke_ros_setup("/config", cfg, "a")

    assert "ota_dds.xml" not in setup
    assert "MaxAutoParticipantIndex>99<" in setup


def test_smoke_ros_setup_puts_the_publisher_on_the_rmw_the_session_resolves_locally() -> None:
    """A session that names only its OTA side still runs cyclone on a bridged local
    domain. The publisher was given no RMW_IMPLEMENTATION there and ran on the
    image's default beside it (#346)."""
    scoped = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 50}, "b": {"domain_id": 51}},
        "shared": {"ota_domain_id": 52, "rmw": {"ota": {"cyclone": {"config": "cyclonedds_scoped.xml"}}}},
    }
    unbridged = {
        "peers": {"a": {}, "b": {}},
        "shared": {"rmw": {"ota": "cyclone"}},
    }

    for peer in ("a", "b"):
        assert "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" in rosotacom._smoke_ros_setup("/config", scoped, peer)
    # Without a bridge the generator renders no local RMW either, so there is
    # nothing to match and the publisher keeps the image's default.
    assert "RMW_IMPLEMENTATION" not in rosotacom._smoke_ros_setup("/config", unbridged, "a")


def test_smoke_publish_specs_use_source_target_prefix_and_qos() -> None:
    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {
            "a": {
                "domain_id": 46,
                "inbound": {"keep_source_prefix": True},
                "outbound": {"target_prefix": {"use_target_prefix": True}},
            },
            "b": {"domain_id": 47},
        },
        "shared": {"use_heartbeat": True},
        "topics": {
            "a_to_b": [
                {
                    "topic": "/topic1",
                    "type": "geometry_msgs/msg/PoseStamped",
                    "qos": {"reliability": "reliable", "durability": "transient_local", "for_role": {}},
                }
            ]
        },
    }

    received = rosotacom._received_crossed_topics(cfg, "b")
    received_a = rosotacom._received_crossed_topics(cfg, "a")
    specs = [s for s in rosotacom._smoke_publish_specs(cfg) if s.publish_topic]

    assert received[0].topic == "/com/in/a/to_b/heartbeat_a"
    assert received_a[1].topic == "/b/heartbeat_b"
    assert len(specs) == 1
    assert specs[0].publish_topic == "/to_b/topic1"
    assert specs[0].topic == "/topic1"
    assert specs[0].publish_qos == {"reliability": "reliable", "durability": "transient_local"}
    command = rosotacom._smoke_publisher_command(specs[0], "source ros", 180.0)
    assert "--qos-reliability reliable" in command
    assert "--qos-durability transient_local" in command


def test_smoke_global_to_local_publishes_and_asserts_generated_pipeline_topics() -> None:
    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {
            "a": {"domain_id": 46, "outbound": {"target_prefix": {"use_target_prefix": True}}},
            "b": {"domain_id": 47},
        },
        "shared": {"processing_suffixes": {"framebridge_global": "/globalframe"}},
        "topics": {
            "a_to_b": [
                {
                    "topic": "/move_base_free/goal",
                    "type": "geometry_msgs/msg/PoseStamped",
                    "processing": {"framebridge": "global_to_local"},
                    "expect": {"mode": "latched", "presence": "required"},
                }
            ]
        },
    }

    specs = rosotacom._received_crossed_topics(cfg, "b")

    assert [(s.topic, s.label, s.publish_topic) for s in specs] == [
        ("/com/in/a/to_b/move_base_free/goal/globalframe", "a->b inbound bridge topic", None),
        ("/move_base_free/goal", "a->b final topic", "/to_b/move_base_free/goal/globalframe"),
    ]
    assert [s.delivery_mode for s in specs] == ["latched", "latched"]
    assert not specs[-1].enforce_bounds


def test_verify_received_topics_checks_latched_presence_not_rate(monkeypatch: pytest.MonkeyPatch) -> None:
    spec = rosotacom.SmokeTopicSpec(
        source_peer_key="b",
        receiver_peer_key="a",
        topic="/b/site/latched",
        label="b->a final topic",
        enforce_bounds=False,
        delivery_mode="latched",
    )
    log_lines: list[str] = []
    present_checks: list[str] = []

    monkeypatch.setattr(rosotacom, "_received_crossed_topics", lambda cfg, receiver: [spec])

    def fake_present(container: str, ros_setup: str, topic: str) -> bool:
        present_checks.append(topic)
        return True

    monkeypatch.setattr(rosotacom, "_topic_present", fake_present)
    monkeypatch.setattr(
        rosotacom,
        "_wait_for_topic_hz",
        lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError("latched topics must not use topic hz")),
    )

    errors = rosotacom._verify_received_topics(
        "container_a",
        "source ros",
        {},
        "a",
        log_line=log_lines.append,
    )

    assert errors == []
    assert present_checks == ["/b/site/latched"]
    assert any("is present" in line and "latched" in line for line in log_lines)


def test_smoke_probe_false_keeps_topic_out_of_synthetic_runner() -> None:
    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 46}, "b": {"domain_id": 47}},
        "topics": {
            "b_to_a": [
                {"topic": "/active", "type": "std_msgs/msg/String"},
                {"topic": "/contract_only", "type": "std_msgs/msg/String", "expect": {"smoke_probe": False}},
            ]
        },
    }

    assert [s.publish_topic for s in rosotacom._smoke_publish_specs(cfg)] == ["/active"]
    assert [s.topic for s in rosotacom._received_crossed_topics(cfg, "a") if s.publish_topic] == ["/active"]


def test_smoke_native_publish_rate_override() -> None:
    # A rate-changing feature drives its source from expect.smoke_native_hz, not
    # the (lower) asserted received bounds.
    assert rosotacom._smoke_native_publish_rate({"smoke_native_hz": 10, "hz": {"min": 3, "max": 7}}) == 10.0
    # Without the override it falls back to the derived rate (midpoint of bounds).
    assert rosotacom._smoke_native_publish_rate({"hz": {"min": 4, "max": 6}}) == 5.0


def test_smoke_source_publish_rate_compensates_derived_drop_rate() -> None:
    pipe = {"drop_count": 1, "window_size": 2}

    assert rosotacom._smoke_source_publish_rate({"hz": {"min": 2}}, pipe) == 6.0
    assert rosotacom._smoke_source_publish_rate({"smoke_native_hz": 10, "hz": {"min": 2}}, pipe) == 10.0


def test_drop_example_source_publishes_at_native_rate() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "8_drop" / "session-definition.yaml").read_text(encoding="utf-8")
    )
    specs = [s for s in rosotacom._received_crossed_topics(cfg, "a") if s.publish_topic]
    assert len(specs) == 1
    spec = specs[0]
    assert spec.publish_topic == "/drop_demo"
    # Source runs at the declared native 10 Hz so drop 1-of-2 yields the asserted ~5 Hz.
    assert spec.publish_rate == 10.0
    assert (spec.hz_min, spec.hz_max) == (3, 7)


def test_restamp_example_uses_stale_stamped_header_source() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "10_restamp" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )
    specs = [s for s in rosotacom._received_crossed_topics(cfg, "a") if s.publish_topic]
    assert len(specs) == 1
    assert specs[0].publish_topic == "/restamp_demo"
    assert specs[0].publish_type == "geometry_msgs/msg/PointStamped"
    # The synthetic message carries a stale (1970) stamp so restamp has an effect.
    msg = rosotacom._smoke_publish_message("geometry_msgs/msg/PointStamped")
    assert "sec: 1000" in msg and "point" in msg


@pytest.mark.parametrize(
    "msg_type",
    [
        "std_msgs/msg/Empty",
        "std_msgs/msg/Float32",
        "std_msgs/msg/Float64",
        "geometry_msgs/msg/PoseStamped",
        "geometry_msgs/msg/TwistStamped",
        "tf2_msgs/msg/TFMessage",
        "visualization_msgs/msg/MarkerArray",
        "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/CompressedImage",
        "sensor_msgs/msg/NavSatFix",
        "gps_msgs/msg/GPSFix",
        "com_msgs/msg/CompressedData",
        "ffmpeg_image_transport_msgs/msg/FFMPEGPacket",
    ],
)
def test_smoke_publish_message_supports_remote_assist_anonymized_types(msg_type: str) -> None:
    assert rosotacom._smoke_publish_message(msg_type)


def test_smoke_compressed_image_payload_is_valid_png_for_ffmpeg_path() -> None:
    msg = rosotacom._smoke_publish_message("sensor_msgs/msg/CompressedImage")
    match = re.search(r"data: \[([^\]]+)\]", msg)
    assert match is not None
    png = bytes(int(item.strip()) for item in match.group(1).split(","))

    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    chunks: list[tuple[bytes, bytes]] = []
    offset = 8
    while offset < len(png):
        length = struct.unpack(">I", png[offset : offset + 4])[0]
        chunk_type = png[offset + 4 : offset + 8]
        chunk_data = png[offset + 8 : offset + 8 + length]
        expected_crc = struct.unpack(">I", png[offset + 8 + length : offset + 12 + length])[0]
        actual_crc = zlib.crc32(chunk_type + chunk_data) & 0xFFFFFFFF
        assert actual_crc == expected_crc
        chunks.append((chunk_type, chunk_data))
        offset += 12 + length

    assert offset == len(png)
    assert chunks[0][0] == b"IHDR"
    assert struct.unpack(">II", chunks[0][1][:8]) == (32, 32)
    assert any(chunk_type == b"IDAT" for chunk_type, _ in chunks)
    assert chunks[-1][0] == b"IEND"


def test_named_stage_latency_is_left_to_status_oracle() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "13_link_latency" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )
    specs = [
        spec for spec in rosotacom._received_crossed_topics(cfg, "a") if spec.publish_topic == "/link_latency_demo"
    ]
    assert len(specs) == 1
    assert specs[0].hz_min == 2.0
    assert specs[0].max_delay_s is None


def test_videoquality_cli_generates_synthetic_report_and_enforces_thresholds(
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    synthetic_dir = tmp_path / "frames"

    rc = rosotacom.main(
        [
            "videoquality",
            "--make-synthetic",
            str(synthetic_dir),
            "--synthetic-frames",
            "4",
            "--synthetic-width",
            "4",
            "--synthetic-height",
            "4",
            "--synthetic-quantization-step",
            "4",
        ]
    )

    assert rc == 0
    assert "Synthetic reference manifest" in capsys.readouterr().out
    report_path = tmp_path / "quality.json"
    rc = rosotacom.main(
        [
            "videoquality",
            str(synthetic_dir / "reference-frames.json"),
            str(synthetic_dir / "degraded-frames.json"),
            "--out",
            str(report_path),
            "--min-mean-psnr",
            "20",
            "--max-loss-pct",
            "0",
        ]
    )

    assert rc == 0
    assert "VIDEOQUALITY OK" in capsys.readouterr().out
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["delivery"]["compared_frames"] == 4
    assert report["delivery"]["lost_frames"] == 0

    rc = rosotacom.main(
        [
            "videoquality",
            str(synthetic_dir / "reference-frames.json"),
            str(synthetic_dir / "degraded-frames.json"),
            "--min-mean-psnr",
            "80",
        ]
    )

    assert rc == 1
    assert "VIDEOQUALITY FAIL" in capsys.readouterr().err


def test_trickle_example_asserts_the_trickle_output_stage() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "11_trickle" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )
    specs = [s for s in rosotacom._received_crossed_topics(cfg, "a") if s.publish_topic]
    assert len(specs) == 1
    spec = specs[0]
    # Source publishes the base topic at 1 Hz...
    assert spec.publish_topic == "/trickle_demo"
    assert spec.publish_rate == 1.0
    # ...but the asserted received stage is the receiver-side trickle re-publish.
    assert spec.topic == "/trickle_demo/trickle"
    assert (spec.hz_min, spec.hz_max) == (2, 8)


def test_publish_test_topics_command_starts_and_stops_identity_publishers(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "2_native_chatter" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )
    started: list[tuple[str, float | None]] = []
    stopped: list[str] = []

    monkeypatch.setattr(rosotacom, "_resolve_running_peer", lambda args, identity: ("container_b", "source ros", cfg))

    def fake_start(
        containers: dict[str, str],
        ros_setups: dict[str, str],
        cfg: dict[str, object],
        **kwargs: object,
    ) -> list[rosotacom.SmokeTopicSpec]:
        started.append((containers["b"], kwargs.get("duration") if isinstance(kwargs.get("duration"), float) else None))
        return rosotacom._smoke_publish_specs(cfg, source_peer_key="b")

    def fake_stop(containers: dict[str, str], specs: list[rosotacom.SmokeTopicSpec]) -> None:
        stopped.extend([containers[s.source_peer_key] for s in specs])

    monkeypatch.setattr(rosotacom, "_start_smoke_topic_publishers", fake_start)
    monkeypatch.setattr(rosotacom, "_stop_smoke_topic_publishers", fake_stop)

    assert (
        rosotacom.publish_test_topics_command(
            argparse.Namespace(identity="b", duration=42.0, stop=False, session_dir="2_native_chatter")
        )
        == 0
    )
    assert started == [("container_b", 42.0)]
    assert "Started 1 test topic publisher" in capsys.readouterr().out

    assert (
        rosotacom.publish_test_topics_command(
            argparse.Namespace(identity="b", duration=42.0, stop=True, session_dir="2_native_chatter")
        )
        == 0
    )
    assert stopped == ["container_b"]


def test_smoke_crossed_topics_keep_heartbeat_labels() -> None:
    cfg = {"peers": {"a": {}, "b": {}}, "shared": {"use_heartbeat": True}}

    assert [(s.topic, s.label, s.enforce_bounds) for s in rosotacom._received_crossed_topics(cfg, "b")] == [
        ("/com/in/a/heartbeat_a", "a->b inbound bridge heartbeat", False),
        ("/heartbeat_a", "a->b final heartbeat", True),
    ]


def test_smoke_crossed_topics_include_compressed_occupancy_grid_pipeline() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "3_comp_occ_grid" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )

    specs = rosotacom._received_crossed_topics(cfg, "a")
    costmap_specs = [s for s in specs if "costmap" in s.topic]

    assert [(s.topic, s.label) for s in costmap_specs] == [
        ("/com/in/b/costmap/costmap/restamped/bz2", "b->a inbound bridge topic"),
        ("/costmap/costmap/restamped", "b->a final topic"),
    ]
    final = costmap_specs[-1]
    assert final.publish_topic == "/costmap/costmap"
    assert final.publish_type == "nav_msgs/msg/OccupancyGrid"
    assert final.publish_rate == 3.0
    assert (final.hz_min, final.hz_max, final.max_delay_s) == (1.0, 5.0, 0.5)
    assert final.enforce_bounds
    assert "width: 4" in rosotacom._smoke_publish_message(final.publish_type)


def test_smoke_crossed_topics_include_zenoh_compressed_occupancy_grid_pipeline() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "4_comp_occ_grid_zen" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )

    specs = [s for s in rosotacom._received_crossed_topics(cfg, "a") if "costmap" in s.topic]

    assert [(s.topic, s.label) for s in specs] == [
        ("/com/in/b/costmap/costmap/restamped/bz2", "b->a inbound bridge topic"),
        ("/costmap/costmap/restamped", "b->a final topic"),
    ]
    assert specs[-1].publish_type == "nav_msgs/msg/OccupancyGrid"


@pytest.mark.parametrize(
    ("session_name", "receiver", "expected_topics"),
    [
        (
            "5_sized_payload",
            "b",
            ["/com/in/a/size_test_a/ota_stamped", "/size_test_a"],
        ),
        (
            "6_sized_payload_zen",
            "b",
            ["/com/in/a/size_test_a", "/size_test_a"],
        ),
    ],
)
def test_smoke_crossed_topics_include_sized_payload_pipelines(
    session_name: str,
    receiver: str,
    expected_topics: list[str],
) -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / session_name / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )

    specs = [s for s in rosotacom._received_crossed_topics(cfg, receiver) if "size_test" in s.topic]

    assert [s.topic for s in specs] == expected_topics
    final = specs[-1]
    assert final.publish_topic == "/size_test_a"
    assert final.publish_type == "com_msgs/msg/SizedPayload"
    assert final.expected_size == 66000
    command = rosotacom._smoke_publisher_command(final, "source ros", 180.0)
    assert "ros2 run com_py sized_publisher" in command
    assert "-p topic:=/size_test_a" in command
    assert "-p size:=66000" in command


def test_run_session_generates_into_instance_config_without_touching_static_source(tmp_path: Path) -> None:
    from session.creation import run_session

    source = tmp_path / "sessions" / "1_heartbeat"
    output = tmp_path / "session-instances" / "2026-01-01" / "1_heartbeat_run" / "config"
    source.mkdir(parents=True)
    (source / "session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "",
            ]
        ),
        encoding="utf-8",
    )

    peer_dir = run_session._resolve_peer_dir(
        str(source),
        str(output),
        "a",
        force=True,
        rewrite_formatting=False,
        peer_address=["a=127.0.0.1", "b=127.0.0.2"],
        link_trace=True,
        link_trace_interval_s=0.5,
        link_trace_modem_command="cat /tmp/modem.json",
    )

    assert Path(peer_dir) == output / "a"
    assert (output / "session-definition.yaml").is_file()
    assert (output / "a" / "plugin.yaml").is_file()
    plugin = yaml.safe_load((output / "a" / "plugin.yaml").read_text(encoding="utf-8"))
    assert plugin["parameters"]["status_overview"] is True
    assert plugin["parameters"]["link_trace"] is True
    assert plugin["parameters"]["link_trace_interval_s"] == 0.5
    assert plugin["parameters"]["link_trace_modem_command"] == "cat /tmp/modem.json"
    assert (output / "a" / "pipeline_spec.yaml").is_file()
    assert not (source / "a").exists()


def test_run_session_passes_cyclone_spdp_interval_to_plugin(tmp_path: Path) -> None:
    from session.creation import run_session

    source = tmp_path / "sessions" / "spdp"
    output = tmp_path / "session-instances" / "2026-01-01" / "spdp_run" / "config"
    source.mkdir(parents=True)
    (source / "session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "shared:",
                "  rmw:",
                "    local: cyclone",
                "    ota:",
                "      cyclone:",
                "        spdp_interval: 150s",
                "topics:",
                "  a_to_b:",
                "    - topic: /x",
                "      type: std_msgs/msg/String",
                "",
            ]
        ),
        encoding="utf-8",
    )

    peer_dir = run_session._resolve_peer_dir(
        str(source),
        str(output),
        "a",
        force=True,
        rewrite_formatting=False,
        peer_address=["a=127.0.0.1", "b=127.0.0.2"],
    )

    plugin = yaml.safe_load((Path(peer_dir) / "plugin.yaml").read_text(encoding="utf-8"))
    assert plugin["parameters"]["ota_spdp_interval"] == "150s"


def test_run_session_passes_cyclone_fragment_size_to_plugin(tmp_path: Path) -> None:
    """The number reaches the peer that renders the profile, not just the YAML."""
    from session.creation import run_session

    source = tmp_path / "sessions" / "fragment"
    output = tmp_path / "session-instances" / "2026-01-01" / "fragment_run" / "config"
    source.mkdir(parents=True)
    (source / "session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "shared:",
                "  rmw:",
                "    local: cyclone",
                "    ota:",
                "      cyclone:",
                "        fragment_size: 1200B",
                "topics:",
                "  a_to_b:",
                "    - topic: /x",
                "      type: std_msgs/msg/String",
                "",
            ]
        ),
        encoding="utf-8",
    )

    peer_dir = run_session._resolve_peer_dir(
        str(source),
        str(output),
        "a",
        force=True,
        rewrite_formatting=False,
        peer_address=["a=127.0.0.1", "b=127.0.0.2"],
    )

    plugin = yaml.safe_load((Path(peer_dir) / "plugin.yaml").read_text(encoding="utf-8"))
    assert plugin["parameters"]["ota_fragment_size"] == "1200B"


def test_run_session_rejects_a_fragment_size_no_template_can_use(tmp_path: Path) -> None:
    """An unusable size is a session error, not a profile that renders wrong."""
    from session.creation import run_session

    source = tmp_path / "sessions" / "bad_fragment"
    output = tmp_path / "session-instances" / "2026-01-01" / "bad_fragment_run" / "config"
    source.mkdir(parents=True)
    (source / "session-definition.yaml").write_text(
        "\n".join(
            [
                "peers:",
                "  a: {}",
                "  b: {}",
                "shared:",
                "  rmw:",
                "    local: cyclone",
                "    ota:",
                "      cyclone:",
                "        fragment_size: plenty",
                "topics:",
                "  a_to_b:",
                "    - topic: /x",
                "      type: std_msgs/msg/String",
                "",
            ]
        ),
        encoding="utf-8",
    )

    with pytest.raises(RuntimeError, match="positive byte size"):
        run_session._resolve_peer_dir(
            str(source),
            str(output),
            "a",
            force=True,
            rewrite_formatting=False,
            peer_address=["a=127.0.0.1", "b=127.0.0.2"],
        )


def test_create_session_yaml_injects_catmux_logging_command(tmp_path: Path) -> None:
    from session.creation import create_session_yaml

    peer_dir = tmp_path / "config" / "a"
    peer_dir.mkdir(parents=True)
    (peer_dir / "plugin.yaml").write_text(
        "\n".join(
            [
                "parameters:",
                "  example: true",
                "common:",
                "  before_commands:",
                "    - echo existing",
                "windows:",
                "  - name: TEST",
                "    splits:",
                "      - commands:",
                "        - echo run",
                "",
            ]
        ),
        encoding="utf-8",
    )
    (peer_dir / "session_specification.yaml").write_text("session_plugins:\n  - ./plugin.yaml\n", encoding="utf-8")

    create_session_yaml.main(
        str(peer_dir),
        instance_dir="/session/instances/run",
        config_dir="/session/instances/run/config",
        catmux_log_dir="/session/instances/run/logs/a/catmux",
        rosbag_dir="/session/instances/run/rosbags/a",
    )

    merged = yaml.safe_load((peer_dir / ".session_readonly.yaml").read_text(encoding="utf-8"))
    before_commands = merged["common"]["before_commands"]
    assert "catmux_log_setup.sh" in before_commands[0]
    assert "ROSOTACOM_CATMUX_LOG_DIR=/session/instances/run/logs/a/catmux" in before_commands[0]
    assert before_commands[1] == "echo existing"


# --- content integrity (expectation concepts reference replay-only) ---------------------------------


def test_content_matches_normalizes_quotes_and_separator() -> None:
    assert rosotacom.content_matches("rosotacom smoke\n", "rosotacom smoke")
    assert rosotacom.content_matches("'rosotacom smoke'\n---", "rosotacom smoke")
    assert rosotacom.content_matches('"hello"', "hello")
    assert not rosotacom.content_matches("corrupted", "rosotacom smoke")
    assert not rosotacom.content_matches("", "rosotacom smoke")


def test_smoke_expected_field_extracts_string_payload() -> None:
    assert rosotacom._smoke_expected_field("std_msgs/msg/String", "data") == "rosotacom smoke"
    # A field the synthetic payload doesn't have -> None (skip the check).
    assert rosotacom._smoke_expected_field("std_msgs/msg/String", "nope") is None


def test_content_integrity_specs_selects_passthrough_string() -> None:
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "12_content_integrity" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )
    specs = rosotacom._content_integrity_specs(cfg, "a")
    assert specs == [("/integrity_demo", "std_msgs/msg/String", "data", "rosotacom smoke")]
    # b receives nothing in this one-directional example.
    assert rosotacom._content_integrity_specs(cfg, "b") == []


def test_content_integrity_skips_transformed_topics() -> None:
    # 10_restamp transforms (restamp) so the received topic != base -> not byte-equal.
    cfg = yaml.safe_load(
        (rosotacom.EXAMPLE_PROJECT_DIR / "sessions" / "10_restamp" / "session-definition.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert rosotacom._content_integrity_specs(cfg, "a") == []


def test_resources_path_prints_an_absolute_existing_path(capsys: pytest.CaptureFixture[str]) -> None:
    # External tooling cannot `import rosotacom` (pipx/venv isolation), so this
    # command is the only supported way to locate packaged resources. It must
    # print exactly one absolute path and nothing else, to stay shell-composable.
    for name in rosotacom.NAMED_RESOURCES:
        assert rosotacom.resources_path_command(argparse.Namespace(name=name)) == 0
        out = capsys.readouterr().out
        assert out.endswith("\n")
        printed = Path(out.strip())
        assert printed.is_absolute()
        assert printed.exists()
        assert len(out.splitlines()) == 1

    assert Path(rosotacom.NAMED_RESOURCES["ros2docker-lyrical"]).is_file()


def test_resources_path_rejects_an_unknown_resource_name() -> None:
    with pytest.raises(SystemExit):
        rosotacom.main(["resources", "path", "no_such_resource"])


def test_resources_path_reports_an_incomplete_installation(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setitem(rosotacom.NAMED_RESOURCES, "ws", tmp_path / "gone")
    with pytest.raises(RuntimeError, match="missing from this rosotacom installation"):
        rosotacom.resources_path_command(argparse.Namespace(name="ws"))


def _ota_bindings() -> dict[str, PeerBinding]:
    return {
        "a": PeerBinding("a", "10.0.0.10", None, "workstation"),
        "b": PeerBinding("b", "10.0.0.11", "robot-b", "robot"),
    }


def test_ota_source_mode_stages_a_checkout_and_owns_the_default() -> None:
    plan = rosotacom._ota_plan_from_bindings(_ota_bindings(), workdir="/tmp/rosotacom_ota")

    assert plan.install_mode == "source"
    assert plan.install_pin is None
    assert plan.rosotacom == "source/.venv/bin/rosotacom"


def test_ota_pin_mode_defaults_to_the_running_version() -> None:
    # `--install-mode pin` alone means "rehearse what I am running, as it
    # ships", so it must not need a second flag to be useful.
    plan = rosotacom._ota_plan_from_bindings(_ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="pin")

    assert plan.install_pin == rosotacom.__version__
    assert plan.rosotacom == "venv/bin/rosotacom"


def test_ota_pin_mode_installs_the_published_distribution() -> None:
    plan = rosotacom._ota_plan_from_bindings(
        _ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="pin", install_pin="2.4"
    )

    script = rosotacom._ota_install_pin_script(plan)

    assert "python3 -m venv /tmp/rosotacom_ota/venv" in script
    assert f"pip install {rosotacom._ota_distribution_name()}==2.4" in script
    # Nothing is staged in pin mode: the peer pulls the artefact from the index.
    assert "source" not in script


def test_ota_pin_mode_rejects_a_version_no_index_can_serve() -> None:
    # A local build ("0+unknown", or anything with a local segment) would make
    # the peers install some *other* version while the manifest claims this one.
    for unusable in ("0+unknown", "2.4.dev3+g1234567"):
        with pytest.raises(RuntimeError, match="not something an index can serve"):
            rosotacom._ota_plan_from_bindings(
                _ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="pin", install_pin=unusable
            )


def test_ota_install_pin_requires_pin_mode() -> None:
    with pytest.raises(RuntimeError, match="only applies to --install-mode pin"):
        rosotacom._ota_plan_from_bindings(
            _ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="source", install_pin="2.4"
        )


def test_ota_rejects_an_unknown_install_mode() -> None:
    with pytest.raises(RuntimeError, match="Unsupported OTA install mode"):
        rosotacom._ota_plan_from_bindings(_ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="wheel")


def test_ota_state_round_trips_the_install_mode(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = rosotacom._ota_plan_from_bindings(
        _ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="pin", install_pin="2.4"
    )
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota")

    plan = rosotacom._ota_write_state(instance, plan)

    # A stop or a --state-file resume must reach the same executable the start
    # installed; losing the mode here would look for a source tree that is
    # not there.
    assert rosotacom._ota_load_state(str(plan.state_path)) == plan


def test_ota_manifest_records_what_the_peers_ran(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = rosotacom._ota_plan_from_bindings(
        _ota_bindings(), workdir="/tmp/rosotacom_ota", install_mode="pin", install_pin="2.4"
    )
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota")
    plan = rosotacom._ota_write_state(instance, plan)

    rosotacom._ota_write_manifest(
        instance, target, runtime, plan, tmux_session=None, interactive=False, phase="running"
    )

    manifest = yaml.safe_load((instance.host_dir / "manifest.yaml").read_text(encoding="utf-8"))
    run = manifest["ota_smoke_runs"]["scenario:demo"]

    assert run["install_mode"] == "pin"
    assert run["install_pin"] == "2.4"


def test_pin_mode_does_not_query_a_self_contained_project(tmp_path: Path) -> None:
    # The peer lookup exists only for projects that reuse rosotacom's packaged
    # configs. Running it unconditionally would make pin mode depend on a
    # command that older pinned versions do not have, for an answer it would
    # then discard.
    runtime, _resolved = _write_test_scenario_project(tmp_path)

    assert rosotacom._ota_project_uses_packaged_configs(runtime) is False


def test_pin_mode_queries_a_project_reusing_packaged_configs(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    config = rosotacom._load_yaml_file(runtime.rosotacom_config)
    config["session_configs_dir"] = str(rosotacom.EXAMPLE_PROJECT_DIR / "sessions")
    runtime.rosotacom_config.write_text(yaml.safe_dump(config), encoding="utf-8")

    assert rosotacom._ota_project_uses_packaged_configs(runtime) is True


def _ota_checkout_plan(project_config: Path) -> rosotacom.OtaSmokePlan:
    _root, project = rosotacom._ota_checkout_project_path(project_config)
    return rosotacom._ota_plan_from_bindings(
        _ota_bindings(),
        workdir="/tmp/rosotacom_ota",
        install_mode="checkout",
        checkouts={"a": str(project_config.parent.parent), "b": "/home/other/fleet_mgmt"},
        project=project,
    )


def _write_checkout_project(tmp_path: Path) -> Path:
    """A project inside a git checkout, one directory below its root."""
    root = tmp_path / "fleet_mgmt"
    (root / ".git").mkdir(parents=True)
    (root / "session").mkdir()
    config = root / "session" / "rosotacom.yaml"
    config.write_text("session_configs_dir:\n  - sessions\n", encoding="utf-8")
    return config


def test_ota_checkout_mode_runs_each_peer_from_its_own_checkout(tmp_path: Path) -> None:
    # The whole point: two machines, one repository, two absolute paths. A
    # single plan-wide workdir would be right on at most one of them.
    plan = _ota_checkout_plan(_write_checkout_project(tmp_path))

    assert plan.install_mode == "checkout"
    assert plan.rosotacom == "rosotacom"
    assert plan.project == "session/rosotacom.yaml"
    assert plan.peers["a"].checkout == str(tmp_path / "fleet_mgmt")
    assert plan.peers["b"].checkout == "/home/other/fleet_mgmt"

    command = rosotacom._ota_rosotacom_command(plan, ["test"], plan.peers["b"])

    assert command.startswith('export PATH="$HOME/.local/bin:$PATH"; cd /home/other/fleet_mgmt && rosotacom test')
    assert "--rosotacom-config session/rosotacom.yaml" in command


def test_ota_checkout_mode_names_the_shared_repository_when_a_peer_lacks_the_project(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    # The path is split once here and asserted on every peer, so a peer whose
    # checkout is a *different* repository fails this probe -- and used to fail
    # it as a bare "exit code 1", which reads as a broken machine rather than as
    # the contract it is. Two runs were spent on that in one afternoon: a bench
    # driven from the harness checkout, with a peer that only has the fleet
    # repository, cannot resolve one relative path on both sides.
    plan = _ota_checkout_plan(_write_checkout_project(tmp_path))

    def fake_run(peer, script, *, label, **kwargs):
        code = 1 if script.startswith("test -f") and peer.name == "b" else 0
        return subprocess.CompletedProcess([], code, "", "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_run)

    with pytest.raises(RuntimeError) as error:
        rosotacom._ota_verify_checkouts(plan, dry_run=False)

    message = str(error.value)
    assert "/home/other/fleet_mgmt/session/rosotacom.yaml does not exist" in message
    assert "the same repository" in message
    assert "it is behind: sync it" in message


def test_ota_checkout_mode_puts_pipx_on_path_because_ssh_is_not_a_login_shell(tmp_path: Path) -> None:
    # `ssh host 'cmd'` skips the login profile, so a pipx install in
    # ~/.local/bin is invisible and the peer answers "command not found" —
    # which reads as a broken machine. The staged modes never meet this
    # because they call an absolute path.
    plan = _ota_checkout_plan(_write_checkout_project(tmp_path))
    staged = _ota_plan(tmp_path)

    assert "$HOME/.local/bin" in rosotacom._ota_rosotacom_command(plan, ["test"], plan.peers["a"])
    assert "PATH" not in rosotacom._ota_rosotacom_command(staged, ["test"], staged.peers["a"])


def test_ota_checkout_mode_requires_a_checkout_for_every_peer(tmp_path: Path) -> None:
    _root, project = rosotacom._ota_checkout_project_path(_write_checkout_project(tmp_path))
    with pytest.raises(RuntimeError, match="--peer-checkout b=<path>"):
        rosotacom._ota_plan_from_bindings(
            _ota_bindings(),
            workdir="/tmp/rosotacom_ota",
            install_mode="checkout",
            checkouts={"a": "/home/me/fleet_mgmt"},
            project=project,
        )


def test_ota_checkout_mode_refuses_a_relative_or_dangerous_checkout(tmp_path: Path) -> None:
    _root, project = rosotacom._ota_checkout_project_path(_write_checkout_project(tmp_path))
    for bad, message in ((".", "absolute path"), ("/home", "broad top-level")):
        with pytest.raises(RuntimeError, match=message):
            rosotacom._ota_plan_from_bindings(
                _ota_bindings(),
                workdir="/tmp/rosotacom_ota",
                install_mode="checkout",
                checkouts={"a": bad, "b": bad},
                project=project,
            )


def test_ota_peer_checkout_is_rejected_outside_checkout_mode() -> None:
    with pytest.raises(RuntimeError, match="only applies to --install-mode checkout"):
        rosotacom._ota_plan_from_bindings(
            _ota_bindings(),
            workdir="/tmp/rosotacom_ota",
            install_mode="pin",
            checkouts={"a": "/home/me/fleet_mgmt", "b": "/home/other/fleet_mgmt"},
        )


def test_ota_checkout_project_path_needs_a_repository(tmp_path: Path) -> None:
    loose = tmp_path / "session" / "rosotacom.yaml"
    loose.parent.mkdir(parents=True)
    loose.write_text("{}\n", encoding="utf-8")
    with pytest.raises(RuntimeError, match="git checkout"):
        rosotacom._ota_checkout_project_path(loose)


def test_ota_cleanup_never_removes_a_checkout(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    # `--cleanup` exists to undo staging. In checkout mode the workdir is the
    # user's repository, so the same `rm -rf` would delete a checkout — with
    # its bags and untracked work — to remove files that were never created.
    plan = replace(_ota_checkout_plan(_write_checkout_project(tmp_path)), state_path=tmp_path / "state.yaml")
    calls: list[str] = []

    original = rosotacom._ota_run
    try:
        rosotacom._ota_run = lambda *args, **kwargs: calls.append(str(args))  # type: ignore[assignment]
        rosotacom._ota_cleanup_hosts(plan, dry_run=False)
    finally:
        rosotacom._ota_run = original  # type: ignore[assignment]

    assert calls == []
    assert "staged nothing" in capsys.readouterr().out


def test_ota_checkout_state_round_trips_the_peer_checkouts(tmp_path: Path) -> None:
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_checkout_plan(_write_checkout_project(tmp_path))
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota")

    plan = rosotacom._ota_write_state(instance, plan)

    # A --stop or --state-file resume must reach the same checkout the start
    # used; losing it would run the stop in the staging directory instead.
    assert rosotacom._ota_load_state(str(plan.state_path)) == plan


def _ota_exec_plan(project_config: Path, command: str = "remote-rosotacom center-host") -> rosotacom.OtaSmokePlan:
    _root, project = rosotacom._ota_checkout_project_path(project_config)
    return rosotacom._ota_plan_from_bindings(
        _ota_bindings(),
        workdir="/tmp/rosotacom_ota",
        install_mode="checkout",
        checkouts={"a": str(project_config.parent.parent), "b": "/home/other/fleet_mgmt"},
        project=project,
        exec_commands={"a": command},
    )


def test_named_resources_reach_ros2docker_through_the_environment(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # A ros2docker config cannot compute a path into an isolated install, so it
    # writes ${ROSOTACOM_COM_MSGS}. Until this, every caller had to export that
    # itself -- which works right up to the point where the caller is not a
    # shell on the same machine. A two-host run died on the far side with
    # "Could not expand environment variables in host path" while the identical
    # single-host run passed, because there the peer inherited the variable from
    # the orchestrator. The single-host run could not have caught it.
    for name in rosotacom.NAMED_RESOURCES:
        monkeypatch.delenv(f"ROSOTACOM_{name.upper()}", raising=False)

    rosotacom._export_named_resource_paths()

    assert os.environ["ROSOTACOM_COM_MSGS"] == str(rosotacom.NAMED_RESOURCES["com_msgs"])
    assert Path(os.environ["ROSOTACOM_COM_MSGS"]).name == "com_msgs"


def test_an_explicit_resource_path_is_not_overridden(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    # setdefault, not assignment: a caller pointing somewhere else on purpose --
    # a checkout under development, say -- still wins.
    monkeypatch.setenv("ROSOTACOM_COM_MSGS", "/somewhere/else")

    rosotacom._export_named_resource_paths()

    assert os.environ["ROSOTACOM_COM_MSGS"] == "/somewhere/else"


def test_ota_peer_exec_replaces_the_ssh_client(tmp_path: Path) -> None:
    # The point of the flag: a peer can be reached by something that is not a
    # shell. The script arrives as the final argument, which is the contract
    # `ssh host <script>` already has, so a forced command on the far side sees
    # exactly one argument and can decide whether to run it.
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))

    assert rosotacom._ota_remote_argv(plan.peers["a"], "true") == [
        "remote-rosotacom",
        "center-host",
        "true",
    ]


def test_ota_peer_exec_ignores_batch_mode_which_belongs_to_ssh(tmp_path: Path) -> None:
    # `-o BatchMode=yes` is an option of one client. Passing it to an arbitrary
    # transport would be guessing at an interface it never agreed to, and the
    # failure would look like a broken peer rather than a wrong flag.
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))

    assert rosotacom._ota_remote_argv(plan.peers["a"], "true", batch=True) == [
        "remote-rosotacom",
        "center-host",
        "true",
    ]


def test_ota_peer_exec_carries_the_tty_request(tmp_path: Path) -> None:
    # Dropping this cost a two-host run: the peer windows start their container
    # with `docker run -i -t`, which refuses outright when the far side has no
    # terminal, so a transport that cannot be told "this one is interactive"
    # carries the probes and not the experiment.
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))

    assert rosotacom._ota_remote_argv(plan.peers["a"], "true", tty=True) == [
        "env",
        "ROSOTACOM_PEER_TTY=1",
        "remote-rosotacom",
        "center-host",
        "true",
    ]


def test_ota_peer_exec_tty_is_a_variable_so_an_older_transport_still_works(
    tmp_path: Path,
) -> None:
    # An environment variable rather than a flag, so a transport that does not
    # understand it behaves exactly as before instead of dying on an argument it
    # never agreed to parse. The argv contract is what makes any command usable
    # as a transport at all.
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))
    argv = rosotacom._ota_remote_argv(plan.peers["a"], "true", tty=True)

    assert argv[-1] == "true"
    assert not any(part.startswith("-") for part in argv)


def test_ota_peer_exec_makes_a_peer_remote(tmp_path: Path) -> None:
    # Everything that used to ask `peer.ssh` meant "is it over there". A peer
    # behind a command transport is, and a caller that still reads `.ssh` would
    # take the local branch for a machine on another host.
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))

    assert rosotacom._ota_peer_is_remote(plan.peers["a"]) is True
    assert rosotacom._ota_peer_is_remote(plan.peers["b"]) is True
    assert rosotacom._ota_peer_target(plan.peers["a"]) == "remote-rosotacom center-host"


def test_ota_peer_exec_requires_checkout_mode() -> None:
    # source and pin push a tree and a file to the peer by taking the ssh
    # client's argv apart. A transport that only promises "run this script"
    # cannot carry either.
    with pytest.raises(RuntimeError, match="requires --install-mode checkout"):
        rosotacom._ota_plan_from_bindings(
            _ota_bindings(),
            workdir="/tmp/rosotacom_ota",
            install_mode="source",
            exec_commands={"a": "remote-rosotacom center-host"},
        )


def test_ota_peer_exec_refuses_to_share_a_peer_with_peer_ssh(tmp_path: Path) -> None:
    _root, project = rosotacom._ota_checkout_project_path(_write_checkout_project(tmp_path))
    with pytest.raises(RuntimeError, match="a peer is reached one way"):
        rosotacom._ota_plan_from_bindings(
            _ota_bindings(),
            workdir="/tmp/rosotacom_ota",
            install_mode="checkout",
            checkouts={"a": "/home/me/fleet_mgmt", "b": "/home/other/fleet_mgmt"},
            project=project,
            # "b" already carries ssh robot-b in the fixture bindings.
            exec_commands={"b": "remote-rosotacom center-host"},
        )


def test_ota_peer_exec_names_a_peer_the_session_has(tmp_path: Path) -> None:
    _root, project = rosotacom._ota_checkout_project_path(_write_checkout_project(tmp_path))
    with pytest.raises(RuntimeError, match="names a peer the session does not have: c"):
        rosotacom._ota_plan_from_bindings(
            _ota_bindings(),
            workdir="/tmp/rosotacom_ota",
            install_mode="checkout",
            checkouts={"a": "/home/me/fleet_mgmt", "b": "/home/other/fleet_mgmt"},
            project=project,
            exec_commands={"c": "remote-rosotacom center-host"},
        )


def test_ota_peer_exec_survives_a_stop(tmp_path: Path) -> None:
    # A stop reaches the peers the start reached. Losing the transport would
    # send the stop down the local branch and leave the far side running, which
    # is the shape of the defect --install-mode checkout introduced once
    # already.
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota")

    plan = rosotacom._ota_write_state(instance, plan)

    assert rosotacom._ota_load_state(str(plan.state_path)) == plan
    assert rosotacom._ota_load_state(str(plan.state_path)).peers["a"].exec_command == "remote-rosotacom center-host"


def test_ota_staging_refuses_a_command_transport(tmp_path: Path) -> None:
    # Defence in depth for the check above: if the mode validation is ever
    # loosened, the local branch of these helpers would copy the peer's tree
    # onto the orchestrator and report success.
    plan = _ota_exec_plan(_write_checkout_project(tmp_path))
    peer = plan.peers["a"]

    with pytest.raises(RuntimeError, match="over a command transport"):
        rosotacom._ota_stage_text(peer, "x", "/tmp/x", dry_run=True, label="stage")


def test_ota_stop_recovers_the_recorded_plan_instead_of_guessing(tmp_path: Path) -> None:
    # A stop that reconstructs its plan from the command line guesses the
    # install mode, and the default guess points at a staged interpreter that a
    # checkout-mode run never installed: every peer stop then fails silently
    # (they are best-effort) while the containers stay up.
    runtime, _resolved = _write_test_scenario_project(tmp_path)
    plan = _ota_checkout_plan(_write_checkout_project(tmp_path))
    target = rosotacom._resolve_interactive_smoke_target("demo", runtime, "auto")
    instance = rosotacom._resolve_session_instance(runtime, target.session, "ota")
    written = rosotacom._ota_write_state(instance, plan)

    found = rosotacom._ota_latest_state_for(runtime, target, "ota")

    assert found == str(written.state_path)
    assert rosotacom._ota_load_state(found).install_mode == "checkout"
    assert rosotacom._ota_latest_state_for(runtime, target, "no-such-instance") is None


def test_ota_cleanup_refuses_to_act_on_a_guessed_plan(capsys: pytest.CaptureFixture[str]) -> None:
    # Without recorded state the install mode is a default, not a fact. Removing
    # a workdir on that basis is a destructive act taken on a guess.
    plan = rosotacom._ota_plan_from_bindings(_ota_bindings(), workdir="/tmp/rosotacom_ota")
    calls: list[str] = []

    original = rosotacom._ota_run
    try:
        rosotacom._ota_run = lambda *args, **kwargs: calls.append(str(args))  # type: ignore[assignment]
        rosotacom._ota_cleanup_hosts(plan, dry_run=False)
    finally:
        rosotacom._ota_run = original  # type: ignore[assignment]

    assert calls == []
    assert "no deployment state" in capsys.readouterr().out


def _git_repo(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    env = {
        **os.environ,
        "GIT_AUTHOR_NAME": "t",
        "GIT_AUTHOR_EMAIL": "t@example.invalid",
        "GIT_COMMITTER_NAME": "t",
        "GIT_COMMITTER_EMAIL": "t@example.invalid",
    }
    for args in (
        ["init", "--initial-branch", "main"],
        ["config", "commit.gpgsign", "false"],
    ):
        subprocess.run(["git", "-C", str(path), *args], check=True, capture_output=True, env=env)
    (path / "session-definition.yaml").write_text("shared: {}\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(path), "add", "-A"], check=True, capture_output=True, env=env)
    subprocess.run(["git", "-C", str(path), "commit", "-m", "seed"], check=True, capture_output=True, env=env)


def test_the_manifest_records_the_revision_of_the_session_it_was_built_from(
    tmp_path: Path,
) -> None:
    """rosotacom_version pins the tool; this pins its input.

    Without it the only recoverable commit is whatever the machine holds when
    somebody looks, which is a different fact — and a wrong one as soon as the
    machine has been synced in between.
    """
    session = tmp_path / "sessions" / "demo"
    _git_repo(session)

    revision = rosotacom._source_revision(session)

    assert revision is not None
    assert revision["vcs"] == "git"
    assert re.fullmatch(r"[0-9a-f]{40}", revision["commit"])
    assert revision["branch"] == "main"
    assert revision["dirty"] is False
    assert Path(revision["root"]).resolve() == session.resolve()


def test_a_session_source_that_differs_from_its_commit_says_so(tmp_path: Path) -> None:
    session = tmp_path / "sessions" / "demo"
    _git_repo(session)
    (session / "session-definition.yaml").write_text("shared: {changed: true}\n", encoding="utf-8")

    assert rosotacom._source_revision(session)["dirty"] is True


def test_untracked_files_beside_a_session_do_not_make_it_dirty(tmp_path: Path) -> None:
    """A deployment checkout carries recordings, logs and scratch files.

    Counting those would leave every real deployment permanently dirty, which is
    the same as reporting nothing.
    """
    session = tmp_path / "sessions" / "demo"
    _git_repo(session)
    (session / "rosbag2_2026_01_01-00_00_00").mkdir()
    (session / "rosbag2_2026_01_01-00_00_00" / "metadata.yaml").write_text("x\n", encoding="utf-8")

    assert rosotacom._source_revision(session)["dirty"] is False


@pytest.mark.parametrize("case", ["not_a_repository", "not_a_directory", "missing"])
def test_a_source_without_a_revision_is_null_and_not_an_error(tmp_path: Path, case: str) -> None:
    """An instance must never fail to start over a provenance read."""
    if case == "not_a_repository":
        target = tmp_path / "plain"
        target.mkdir()
    elif case == "not_a_directory":
        target = tmp_path / "file"
        target.write_text("x\n", encoding="utf-8")
    else:
        target = tmp_path / "gone"

    assert rosotacom._source_revision(target) is None
