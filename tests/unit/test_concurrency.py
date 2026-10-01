# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Concurrency redesign: instance-scoped naming and fail-safe conflict preflights.

Covers the unit half of the concurrency verification plan; the
parallel/conflict end-to-end half lives in tests/e2e/test_parallel_smoke.py.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
from pathlib import Path

import pytest

import rosotacom.cli as rosotacom
from rosotacom import cli_benchmark


def _runtime(
    tmp_path: Path,
    install_id: str = "abc",
    com_container_prefix: str | None = None,
) -> rosotacom.RuntimeConfig:
    return rosotacom.RuntimeConfig(
        None,
        tmp_path / "ros2docker.json",
        (),
        None,
        install_id,
        tmp_path / "instances",
        com_container_prefix=com_container_prefix,
    )


def test_container_name_is_instance_scoped(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    first = rosotacom._container_name("remote", runtime, "run1")
    second = rosotacom._container_name("remote", runtime, "run2")

    assert first == "rosotacom_abc_run1_com_to_remote"
    assert second == "rosotacom_abc_run2_com_to_remote"
    assert first != second
    # Underscores in user-provided instance ids are folded away so the instance
    # component of a name stays parseable.
    assert rosotacom._container_name("remote", runtime, "my_run") == "rosotacom_abc_my-run_com_to_remote"


def test_container_name_is_fixed_with_com_container_prefix(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path, com_container_prefix="remote-assist")
    first = rosotacom._container_name("center", runtime, "run1")
    second = rosotacom._container_name("center", runtime, "run2")

    # No install id, no instance token: the name is stable across runs and
    # readable next to colleagues' containers in `docker ps`.
    assert first == "remote-assist_com-to-center"
    assert second == first
    assert rosotacom._container_name("vehicle", runtime, "run1") == "remote-assist_com-to-vehicle"


def test_matching_com_containers_includes_fixed_names(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path, com_container_prefix="remote-assist")
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            ("remote-assist_com-to-remote", ["bridge"]),
            ("remote-assist_com-to-other", ["bridge"]),
            ("rosotacom_abc_run1_com_to_remote", ["bridge"]),
            ("unrelated", ["bridge"]),
        ],
    )

    # Both schemes match, so a leftover instance-scoped container is still
    # found after the project switched to fixed names (and vice versa).
    assert rosotacom._matching_com_containers(runtime, "remote") == [
        "remote-assist_com-to-remote",
        "rosotacom_abc_run1_com_to_remote",
    ]


def test_scenario_container_name_is_instance_scoped(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    first = rosotacom._scenario_container_name(runtime, "demo", "a", "app", "run1")
    second = rosotacom._scenario_container_name(runtime, "demo", "a", "app", "run2")

    assert first == "rosotacom_abc_run1_scenario_demo_a_app"
    assert first != second


def test_split_workspace_container_extracts_instance_token(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)

    assert rosotacom._split_workspace_container("rosotacom_abc_run1_com_to_remote", runtime) == (
        "run1",
        "com_to_remote",
    )
    assert rosotacom._split_workspace_container("rosotacom_other_run1_com_to_remote", runtime) is None
    assert rosotacom._split_workspace_container("unrelated", runtime) is None


def test_matching_com_containers_filters_by_workspace_and_peer(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path)
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            ("rosotacom_abc_run1_com_to_remote", ["bridge"]),
            ("rosotacom_abc_run2_com_to_remote", ["rosotacom_smoke_abc_x"]),
            ("rosotacom_abc_run1_com_to_other", ["bridge"]),
            ("rosotacom_zzz_run1_com_to_remote", ["bridge"]),
            ("rosotacom_abc_run1_scenario_demo_a_app", ["bridge"]),
            ("unrelated", ["bridge"]),
        ],
    )

    assert rosotacom._matching_com_containers(runtime, "remote") == [
        "rosotacom_abc_run1_com_to_remote",
        "rosotacom_abc_run2_com_to_remote",
    ]


def test_matching_scenario_containers_reconstructs_truncated_names(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path)
    long_app = "app" * 50
    truncated = rosotacom._scenario_container_name(runtime, "demo", "a", long_app, "run1")
    assert len(truncated) == 120

    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            (truncated, ["bridge"]),
            (rosotacom._scenario_container_name(runtime, "demo", "a", "app", "run2"), ["bridge"]),
            ("rosotacom_abc_run1_com_to_remote", ["bridge"]),
        ],
    )

    assert rosotacom._matching_scenario_containers(runtime, "demo", "a", long_app) == [truncated]
    assert rosotacom._matching_scenario_containers(runtime, "demo", "a", "app") == [
        "rosotacom_abc_run2_scenario_demo_a_app"
    ]


def test_interactive_smoke_network_config_is_instance_scoped(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    first_name, first_subnet = rosotacom._interactive_smoke_network_config(runtime, "session", "demo", "run1")
    second_name, second_subnet = rosotacom._interactive_smoke_network_config(runtime, "session", "demo", "run2")

    assert first_name != second_name
    assert first_subnet != second_subnet
    assert first_subnet.startswith("10.137.")
    assert first_subnet.endswith("/29")


def _start_session_args(**overrides: object) -> argparse.Namespace:
    args = argparse.Namespace(
        session_dir="1_heartbeat",
        peer=[],
        peer_address=["a=127.0.0.1", "b=127.0.0.2"],
        identity="a",
        auto_identity=True,
        mode="detached",
        force=False,
        rewrite_formatting=False,
        instance_id="unit",
    )
    for key, value in overrides.items():
        setattr(args, key, value)
    return args


def _patch_start_session_collaborators(
    monkeypatch: pytest.MonkeyPatch,
    runtime: rosotacom.RuntimeConfig,
    session: rosotacom.ResolvedSession,
    cfg: dict[str, object],
    calls: list[tuple[str, dict[str, object]]],
) -> None:
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
    monkeypatch.setattr(rosotacom, "_base_extra_run_args", lambda runtime, session, cfg, instance, **kwargs: [])
    monkeypatch.setattr(rosotacom, "_resolve_mode", lambda mode: "detached")
    monkeypatch.setattr(rosotacom, "_container_exists", lambda name: False)
    monkeypatch.setattr(rosotacom, "_wait_for_container_ready", lambda name: None)
    monkeypatch.setattr(rosotacom, "_write_docker_log", lambda *args, **kwargs: None)
    monkeypatch.setattr(rosotacom, "ros2docker_build", lambda **kwargs: calls.append(("build", kwargs)), raising=False)
    monkeypatch.setattr(rosotacom, "ros2docker_run", lambda **kwargs: calls.append(("run", kwargs)), raising=False)
    monkeypatch.setattr(rosotacom, "ros2docker_exec", lambda **kwargs: calls.append(("exec", kwargs)), raising=False)


def test_start_session_aborts_on_identity_conflict_without_force(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path, install_id="id")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {"com-name": "remote"}}}
    calls: list[tuple[str, dict[str, object]]] = []
    _patch_start_session_collaborators(monkeypatch, runtime, session, cfg, calls)
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [("rosotacom_id_other_com_to_remote", ["bridge"])],
    )

    with pytest.raises(RuntimeError) as excinfo:
        rosotacom.start_session(_start_session_args())

    message = str(excinfo.value)
    assert "already running in this workspace" in message
    assert "rosotacom_id_other_com_to_remote" in message
    assert "rosotacom stop" in message
    # The abort happens before any docker build/run/exec is dispatched.
    assert calls == []


def test_start_session_force_replaces_conflicting_identity_container(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path, install_id="id")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {"com-name": "remote"}}}
    calls: list[tuple[str, dict[str, object]]] = []
    stopped: list[str] = []
    events: list[str] = []
    _patch_start_session_collaborators(monkeypatch, runtime, session, cfg, calls)
    monkeypatch.setattr(
        rosotacom,
        "_network_preflight",
        lambda *args, **kwargs: (
            events.append("preflight")
            or rosotacom.NetworkPreflightResult("a", "127.0.0.1", "b", "127.0.0.2", "lo", "127.0.0.1", None)
        ),
    )
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [("rosotacom_id_other_com_to_remote", ["bridge"])],
    )

    def record_stop(name: str, runtime: rosotacom.RuntimeConfig, **kwargs: object) -> bool:
        events.append("stop")
        stopped.append(name)
        return True

    monkeypatch.setattr(
        rosotacom,
        "_stop_container_name",
        record_stop,
    )

    container = rosotacom.start_session(_start_session_args(force=True))

    assert container == "rosotacom_id_unit_com_to_remote"
    # The conflicting other-instance container is stopped first; force also
    # keeps its pre-existing own-name replace for --instance-id rejoins.
    assert stopped == ["rosotacom_id_other_com_to_remote", "rosotacom_id_unit_com_to_remote"]
    assert events == ["preflight", "stop", "stop"]
    assert [name for name, _ in calls] == ["build", "run", "exec"]


def test_start_session_fixed_name_conflicts_with_itself_without_force(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path, install_id="id", com_container_prefix="remote-assist")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {"com-name": "remote"}}}
    calls: list[tuple[str, dict[str, object]]] = []
    _patch_start_session_collaborators(monkeypatch, runtime, session, cfg, calls)
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [("remote-assist_com-to-remote", ["bridge"])],
    )

    with pytest.raises(RuntimeError) as excinfo:
        rosotacom.start_session(_start_session_args())

    # A fixed name cannot coexist with itself, so the running container is a
    # conflict rather than a rejoinable instance.
    assert "remote-assist_com-to-remote" in str(excinfo.value)
    assert calls == []


def test_start_session_fixed_name_force_replaces_running_container(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path, install_id="id", com_container_prefix="remote-assist")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {"com-name": "remote"}}}
    calls: list[tuple[str, dict[str, object]]] = []
    stopped: list[str] = []
    _patch_start_session_collaborators(monkeypatch, runtime, session, cfg, calls)
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [("remote-assist_com-to-remote", ["bridge"])],
    )
    monkeypatch.setattr(
        rosotacom,
        "_stop_container_name",
        lambda name, runtime, **kwargs: stopped.append(name) or True,
    )

    container = rosotacom.start_session(_start_session_args(force=True))

    assert container == "remote-assist_com-to-remote"
    assert stopped[0] == "remote-assist_com-to-remote"
    assert [name for name, _ in calls] == ["build", "run", "exec"]


def test_start_session_skips_identity_conflicts_on_isolated_networks(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path, install_id="id")
    session = rosotacom.ResolvedSession(tmp_path, "/session/current", "absolute")
    cfg = {"peers": {"a": {}, "b": {"com-name": "remote"}}}
    calls: list[tuple[str, dict[str, object]]] = []
    _patch_start_session_collaborators(monkeypatch, runtime, session, cfg, calls)
    monkeypatch.setattr(rosotacom, "load_config", lambda config: {"run_args": []}, raising=False)

    def fail_scan(all_states: bool = False) -> list[tuple[str, list[str]]]:
        raise AssertionError("isolated-network starts must not scan for conflicts")

    monkeypatch.setattr(rosotacom, "_list_docker_containers", fail_scan)

    container = rosotacom.start_session(
        _start_session_args(network_name="rosotacom_smoke_id_x", network_ip="10.137.1.2")
    )

    assert container == "rosotacom_id_unit_com_to_remote"
    assert [name for name, _ in calls] == ["build", "run", "exec"]


def test_smoke_aborts_when_same_target_is_active(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = _runtime(tmp_path)
    session = rosotacom.ResolvedSession(tmp_path / "1_heartbeat", "/session/current", "absolute")
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_resolve_session", lambda session_dir, runtime: session)
    monkeypatch.setattr(rosotacom, "_matching_smoke_networks", lambda runtime, target_key: ["net-active", "net-idle"])
    monkeypatch.setattr(
        rosotacom,
        "_smoke_network_active_containers",
        lambda network_name: ["rosotacom_abc_run1_com_to_a"] if network_name == "net-active" else [],
    )

    def fail_alloc(*args: object, **kwargs: object) -> None:
        raise AssertionError("conflicting smoke must abort before allocating anything")

    monkeypatch.setattr(rosotacom, "_resolve_session_instance", fail_alloc)
    monkeypatch.setattr(rosotacom, "_ensure_smoke_network", fail_alloc)

    with pytest.raises(RuntimeError) as excinfo:
        rosotacom.smoke(
            argparse.Namespace(
                local=True,
                session_dir="1_heartbeat",
                verify_only=False,
                interactive=False,
                interactive_stop=False,
                interactive_list=False,
            )
        )

    message = str(excinfo.value)
    assert "already active in this workspace" in message
    assert "rosotacom_abc_run1_com_to_a" in message
    assert "--skip-conflict-check" in message

    # An idle leftover network alone is not a conflict.
    monkeypatch.setattr(rosotacom, "_smoke_network_active_containers", lambda network_name: [])
    with pytest.raises(AssertionError, match="abort before allocating"):
        # Passing the conflict check proceeds into (failing) allocation stubs.
        rosotacom.smoke(
            argparse.Namespace(
                local=True,
                session_dir="1_heartbeat",
                verify_only=False,
                interactive=False,
                interactive_stop=False,
                interactive_list=False,
                instance_id=None,
            )
        )


def test_ps_command_classifies_active_containers(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime = _runtime(tmp_path)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            ("rosotacom_abc_run1_com_to_a", ["rosotacom_smoke_abc_session_1_heartbeat_run1"]),
            ("rosotacom_abc_run2_com_to_b", ["bridge"]),
            ("rosotacom_zzz_run1_com_to_a", ["bridge"]),
            ("onedrive", ["bridge"]),
        ],
    )

    assert rosotacom.ps_command(argparse.Namespace()) == 0
    output = capsys.readouterr().out

    assert "Workspace abc" in output
    assert "Smoke-isolated" in output
    assert "rosotacom_abc_run1_com_to_a (network: rosotacom_smoke_abc_session_1_heartbeat_run1)" in output
    assert "Host-shared" in output
    assert "rosotacom_abc_run2_com_to_b" in output
    assert "other rosotacom workspaces: 1" in output
    assert "onedrive" not in output


def test_ps_command_lists_fixed_named_com_containers(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime = _runtime(tmp_path, com_container_prefix="remote-assist")
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            ("remote-assist_com-to-center", ["bridge"]),
            ("rosotacom_abc_run2_com_to_b", ["bridge"]),
            ("onedrive", ["bridge"]),
        ],
    )

    assert rosotacom.ps_command(argparse.Namespace()) == 0
    output = capsys.readouterr().out

    assert "Host-shared" in output
    assert "remote-assist_com-to-center" in output
    assert "rosotacom_abc_run2_com_to_b" in output
    assert "onedrive" not in output


def test_ps_command_reports_empty_workspace(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    runtime = _runtime(tmp_path)
    monkeypatch.setattr(rosotacom, "_load_runtime_config", lambda args: runtime)
    monkeypatch.setattr(rosotacom, "_list_docker_containers", lambda all_states=False: [("onedrive", ["bridge"])])

    assert rosotacom.ps_command(argparse.Namespace()) == 0
    assert "(none — you can start anything)" in capsys.readouterr().out


def _fake_ota_run_factory(outputs: dict[str, str]):
    """Map a substring of the remote script to canned stdout."""

    def fake_ota_run(peer: object, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        for fragment, stdout in outputs.items():
            if fragment in script:
                return subprocess.CompletedProcess([script], 0, stdout, "")
        return subprocess.CompletedProcess([script], 0, "", "")

    return fake_ota_run


def test_ota_conflict_check_aborts_on_remote_rosotacom_containers(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan = rosotacom.OtaSmokePlan(
        state_path=tmp_path / "ota-deployment.yaml",
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="rosotacom.yaml",
        peers={"a": rosotacom.OtaSmokePeer("a", None, "10.0.0.10")},
    )
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory({"docker ps": "rosotacom_abc_run1_com_to_b\nonedrive\n"}),
    )

    with pytest.raises(RuntimeError) as excinfo:
        rosotacom._ota_conflict_check(plan, dry_run=False)

    message = str(excinfo.value)
    assert "exclusive hosts" in message
    assert "rosotacom_abc_run1_com_to_b" in message
    assert "onedrive" not in message


def test_ota_conflict_check_aborts_on_active_shaping(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan = rosotacom.OtaSmokePlan(
        state_path=tmp_path / "ota-deployment.yaml",
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="rosotacom.yaml",
        peers={
            "a": rosotacom.OtaSmokePeer("a", None, "10.0.0.10"),
            "b": rosotacom.OtaSmokePeer("b", "robot-b", "10.0.0.11"),
        },
    )
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory(
            {
                "ip route get": "10.0.0.11 dev tun0 src 10.0.0.10 uid 1000\n",
                "tc qdisc show": "qdisc netem 8001: root refcnt 2 limit 1000 delay 100ms\n",
            }
        ),
    )

    with pytest.raises(RuntimeError, match="Active network shaping detected on a \\(tun0\\)"):
        rosotacom._ota_conflict_check(plan, dry_run=False)


def test_ota_conflict_check_passes_on_quiet_peers(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    plan = rosotacom.OtaSmokePlan(
        state_path=tmp_path / "ota-deployment.yaml",
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="rosotacom.yaml",
        peers={
            "a": rosotacom.OtaSmokePeer("a", None, "10.0.0.10"),
            "b": rosotacom.OtaSmokePeer("b", "robot-b", "10.0.0.11"),
        },
    )
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory(
            {
                "docker ps": "onedrive\n",
                "ip route get": "10.0.0.11 dev tun0 src 10.0.0.10 uid 1000\n",
                "tc qdisc show": "qdisc fq_codel 0: root refcnt 2 limit 10240p\n",
            }
        ),
    )

    rosotacom._ota_conflict_check(plan, dry_run=False)


def test_local_benchmark_aborts_on_any_active_rosotacom_container(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        rosotacom,
        "_list_docker_containers",
        lambda all_states=False: [
            ("rosotacom_zzz_run1_com_to_a", ["bridge"]),
            ("onedrive", ["bridge"]),
        ],
    )

    args = argparse.Namespace()
    with pytest.raises(RuntimeError) as excinfo:
        cli_benchmark._abort_on_local_benchmark_conflicts(args)

    message = str(excinfo.value)
    assert "exclusive resources" in message
    # Containers from *other* workspaces conflict too: shared host contention.
    assert "rosotacom_zzz_run1_com_to_a" in message

    cli_benchmark._abort_on_local_benchmark_conflicts(argparse.Namespace(skip_conflict_check=True))


def test_local_benchmark_passes_on_quiet_host(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(rosotacom, "_list_docker_containers", lambda all_states=False: [("onedrive", ["bridge"])])
    cli_benchmark._abort_on_local_benchmark_conflicts(argparse.Namespace())


def _load_plan(tmp_path: Path) -> rosotacom.OtaSmokePlan:
    return rosotacom.OtaSmokePlan(
        state_path=tmp_path / "ota-deployment.yaml",
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="rosotacom.yaml",
        peers={"a": rosotacom.OtaSmokePeer("a", None, "10.0.0.10")},
    )


def test_ota_load_check_refuses_a_peer_someone_else_is_saturating(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A busy peer must stop the run, not quietly distort it.

    Measured on a shared 32-CPU workstation on 2026-08-19: load 78 on 32 CPUs while a colleague
    trained models. Runs there did not fail — they completed and reported
    numbers that described the contention.
    """
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory({"/proc/loadavg": "78.24 81.86 81.28\n32\n"}),
    )

    with pytest.raises(RuntimeError) as excinfo:
        rosotacom._ota_load_check(_load_plan(tmp_path), dry_run=False)

    message = str(excinfo.value)
    assert "unrelated load" in message
    assert "--skip-conflict-check" in message


def test_ota_load_check_warns_but_proceeds_on_moderate_load(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory({"/proc/loadavg": "9.0 8.0 7.0\n8\n"}),
    )
    rosotacom._ota_load_check(_load_plan(tmp_path), dry_run=False)
    out = capsys.readouterr().out
    assert "interleaved" in out


def test_ota_load_check_is_quiet_on_an_idle_peer(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """The reading is printed even when it passes: a result should carry the
    conditions it was measured under, not only the ones that stopped it."""
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory({"/proc/loadavg": "0.21 0.08 0.02\n8\n"}),
    )
    rosotacom._ota_load_check(_load_plan(tmp_path), dry_run=False)
    out = capsys.readouterr().out
    assert "0.21" in out and "8 CPUs" in out


def test_sigterm_unwinds_so_cleanup_runs() -> None:
    """SIGTERM must reach the run's `finally`, not end the process outright.

    `timeout`, systemd, and a cancelled CI step all send SIGTERM. Under the
    default disposition Python exits immediately, so the OTA run's cleanup —
    revert shaping, collect logs, stop both peers — never happens. On
    2026-08-19 a run killed by `timeout` left containers on both peers, which
    then blocked the next run's exclusivity check.
    """
    cleaned: list[str] = []

    with pytest.raises(SystemExit) as excinfo:
        with rosotacom._terminate_via_exception():
            try:
                os.kill(os.getpid(), signal.SIGTERM)
            finally:
                cleaned.append("teardown ran")

    assert cleaned == ["teardown ran"]
    assert excinfo.value.code == 128 + signal.SIGTERM


def test_sigterm_disposition_is_restored() -> None:
    """The handler must not leak into whatever called us."""
    before = signal.getsignal(signal.SIGTERM)
    with rosotacom._terminate_via_exception():
        assert signal.getsignal(signal.SIGTERM) is not before
    assert signal.getsignal(signal.SIGTERM) is before


def _reach_peer() -> rosotacom.OtaSmokePeer:
    return rosotacom.OtaSmokePeer("a", "robot-a", "10.0.0.10")


def test_reach_check_retries_a_transport_that_flaked(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    """An ssh handshake that times out under load is not a verdict about the peer.

    The probe is `true`, so a non-zero exit can only be the transport. That is
    what makes retrying safe here and nowhere else in the preflight.
    """
    attempts: list[str] = []

    def fake_ota_run(peer: object, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        attempts.append(script)
        if len(attempts) < 3:
            return subprocess.CompletedProcess([script], 255, "", "ssh: connect to host port 22: Connection timed out")
        return subprocess.CompletedProcess([script], 0, "", "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)
    monkeypatch.setattr(rosotacom.time, "sleep", lambda _seconds: None)

    rosotacom._ota_reach_check(_reach_peer(), dry_run=False)

    assert len(attempts) == 3
    out = capsys.readouterr().out
    # The run must not read as a clean first-try connection.
    assert "attempt 3 of 3" in out
    assert "flaky" in out


def test_reach_check_retries_a_timeout_too(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []

    def fake_ota_run(peer: object, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(1)
        if len(calls) == 1:
            raise subprocess.TimeoutExpired(cmd=["ssh"], timeout=20.0)
        return subprocess.CompletedProcess([script], 0, "", "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)
    monkeypatch.setattr(rosotacom.time, "sleep", lambda _seconds: None)

    rosotacom._ota_reach_check(_reach_peer(), dry_run=False)
    assert len(calls) == 2


def test_reach_check_gives_up_and_says_how_often_it_tried(monkeypatch: pytest.MonkeyPatch) -> None:
    calls: list[int] = []

    def fake_ota_run(peer: object, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(1)
        return subprocess.CompletedProcess([script], 255, "", "ssh: Could not resolve hostname robot-a")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)
    monkeypatch.setattr(rosotacom.time, "sleep", lambda _seconds: None)

    with pytest.raises(RuntimeError) as excinfo:
        rosotacom._ota_reach_check(_reach_peer(), dry_run=False)

    assert len(calls) == rosotacom._REACH_ATTEMPTS
    message = str(excinfo.value)
    assert "after 3 attempts" in message
    assert "Could not resolve hostname" in message


def test_the_load_check_does_not_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Only the reachability probe retries.

    A peer somebody else is saturating is still saturated a moment later, and a
    container that is already running is still running. Retrying those would
    turn a real refusal into a delayed one, which is the failure mode the load
    check exists to prevent.
    """
    calls: list[str] = []

    def fake_ota_run(peer: object, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        calls.append(script)
        return subprocess.CompletedProcess([script], 0, "78.24 81.86 81.28\n32\n", "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)
    monkeypatch.setattr(rosotacom.time, "sleep", lambda _seconds: pytest.fail("the load check must not sleep"))

    with pytest.raises(RuntimeError):
        rosotacom._ota_load_check(_load_plan(tmp_path), dry_run=False)

    assert len(calls) == 1


def test_validate_cpuset_accepts_a_cpu_list_and_nothing_else() -> None:
    assert rosotacom.validate_cpuset("0-3") == "0-3"
    assert rosotacom.validate_cpuset(" 0,2,4-7 ") == "0,2,4-7"
    assert rosotacom.parse_cpuset_list("0-3,8") == [0, 1, 2, 3, 8]

    # The string reaches `docker run` and travels over ssh, so it is validated
    # rather than trusted.
    for bad in ("0-3; rm -rf /", "$(id)", "0..3", "-1", "", "0-", "a-b"):
        with pytest.raises(RuntimeError):
            rosotacom.validate_cpuset(bad)
    with pytest.raises(RuntimeError) as excinfo:
        rosotacom.validate_cpuset("7-2")
    assert "backwards" in str(excinfo.value)


def test_cpu_busy_is_read_from_two_snapshots() -> None:
    """A busy fraction needs the delta; the absolute counters describe boot onward."""
    stdout = (
        "cpu0 100 0 100 800 0 0 0 0 0 0\n"
        "cpu1 100 0 100 800 0 0 0 0 0 0\n"
        "---\n"
        # cpu0 gains 100 busy and 900 idle -> 10% busy.
        "cpu0 150 0 150 1700 0 0 0 0 0 0\n"
        # cpu1 gains 900 busy and 100 idle -> 90% busy.
        "cpu1 550 0 550 900 0 0 0 0 0 0\n"
    )
    busy = rosotacom._parse_cpu_busy(stdout, [0, 1])
    assert busy is not None
    assert busy[0] == pytest.approx(0.10, abs=0.01)
    assert busy[1] == pytest.approx(0.90, abs=0.01)

    # iowait does not count as competition for the runqueue: a peer writing a
    # bag must not read as a peer that cannot be measured on.
    io = "cpu0 0 0 0 100 0 0 0 0 0 0\n---\ncpu0 0 0 0 100 900 0 0 0 0 0\n"
    busy_io = rosotacom._parse_cpu_busy(io, [0])
    assert busy_io is not None
    assert busy_io[0] == pytest.approx(0.0, abs=0.01)

    assert rosotacom._parse_cpu_busy("no separator here", [0]) is None
    assert rosotacom._parse_cpu_busy(stdout, [9]) is None  # a CPU the peer does not have


def _pinned_peer(cpuset: str = "0-3") -> rosotacom.OtaSmokePeer:
    return rosotacom.OtaSmokePeer("a", "robot-a", "10.0.0.10", cpuset=cpuset)


def _busy_probe(fraction: float) -> str:
    """Two snapshots in which every CPU is `fraction` busy."""
    lines = [f"cpu{cpu} 0 0 0 0 0 0 0 0 0 0" for cpu in range(4)]
    after = [f"cpu{cpu} {int(1000 * fraction)} 0 0 {int(1000 * (1 - fraction))} 0 0 0 0 0 0" for cpu in range(4)]
    return "\n".join(lines) + "\n---\n" + "\n".join(after) + "\n"


def test_cpuset_check_refuses_when_the_reserved_cpus_are_not_free(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """A reservation that is not in force must not read as a quiet machine."""
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        lambda peer, script, **kw: subprocess.CompletedProcess([script], 0, _busy_probe(0.95), ""),
    )
    with pytest.raises(RuntimeError) as excinfo:
        rosotacom._ota_cpuset_load_check(_pinned_peer(), dry_run=False)
    message = str(excinfo.value)
    assert "not free" in message
    assert "0-3" in message


def test_cpuset_check_passes_on_reserved_cpus_that_are_idle(
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        lambda peer, script, **kw: subprocess.CompletedProcess([script], 0, _busy_probe(0.02), ""),
    )
    assert rosotacom._ota_cpuset_load_check(_pinned_peer(), dry_run=False) is True
    assert "CPUs 0-3 are 2% busy" in capsys.readouterr().out


def test_a_pinned_peer_is_not_judged_by_the_machine_wide_load(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The whole point: a reserved workstation reads high and is still measurable.

    Measured on a shared 32-CPU workstation on 2026-08-19: 68-75 on 32 CPUs with four cores reserved
    away from the account carrying that load. loadavg cannot see the reservation
    — the other account's threads all stayed runnable, they just queue on fewer
    cores — so judging the machine refuses a run the reservation made possible.
    """
    scripts: list[str] = []

    def fake_ota_run(peer: object, script: str, **kwargs: object) -> subprocess.CompletedProcess[str]:
        scripts.append(script)
        if "/proc/stat" in script:
            return subprocess.CompletedProcess([script], 0, _busy_probe(0.01), "")
        return subprocess.CompletedProcess([script], 0, "75.68 68.49 67.20\n32\n", "")

    monkeypatch.setattr(rosotacom, "_ota_run", fake_ota_run)
    plan = rosotacom.OtaSmokePlan(
        state_path=tmp_path / "ota-deployment.yaml",
        workdir="/tmp/rosotacom_ota",
        rosotacom="rosotacom",
        project="rosotacom.yaml",
        peers={"a": _pinned_peer()},
    )

    rosotacom._ota_load_check(plan, dry_run=False)  # must not raise

    assert any("/proc/stat" in s for s in scripts)
    assert not any("loadavg" in s for s in scripts), "a pinned peer must not fall back to loadavg"


def test_an_unpinned_peer_still_uses_the_machine_wide_load(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        rosotacom,
        "_ota_run",
        _fake_ota_run_factory({"/proc/loadavg": "78.24 81.86 81.28\n32\n"}),
    )
    with pytest.raises(RuntimeError) as excinfo:
        rosotacom._ota_load_check(_load_plan(tmp_path), dry_run=False)
    assert "--peer-cpuset" in str(excinfo.value), "the refusal should name the way out"


def test_a_pinned_peer_confines_its_containers(tmp_path: Path) -> None:
    """The reservation only buys anything if the containers actually land on it."""
    target = rosotacom.InteractiveSmokeTarget(
        name="s",
        target_type="session",
        session=rosotacom.ResolvedSession(host_dir=tmp_path, container_dir="/s", source="test"),
        cfg={},
    )
    parts = rosotacom._ota_start_parts(target, "a", "id-1", [], mode="detached", cpuset="0-3")
    assert "--cpuset" in parts
    assert parts[parts.index("--cpuset") + 1] == "0-3"

    # And without one, nothing is added: the flag must not appear on a peer that
    # was never pinned, or every unpinned run changes shape too.
    assert "--cpuset" not in rosotacom._ota_start_parts(target, "a", "id-1", [], mode="detached")


# --------------------------------------------------- instance resolution by name
#
# A session's instances are found by globbing its slug. The slug of one session
# can be a prefix of another's — `vehicle_link` and
# `vehicle_link_kilted_dds` are the pair that exposed it — and a pattern
# that ends the slug with `*` then matches both. The matches are ordered by
# name, so the answer is whichever sorts last, and a letter sorts after a digit:
# the *other* session's run wins over this session's own newest one.
#
# What that cost: on 2026-08-21 `rosotacom status vehicle_link` reported a
# `vehicle_link_kilted_dds` run that had been shut down 45 minutes
# earlier. Three two-host runs waited ten minutes each and concluded their link
# never came up, while their own status file said it had been OK the whole time.


def _session(tmp_path: Path, name: str) -> rosotacom.ResolvedSession:
    host_dir = tmp_path / "sessions" / name
    host_dir.mkdir(parents=True, exist_ok=True)
    return rosotacom.ResolvedSession(host_dir, f"/session/{name}", "test")


def _instance(root: Path, slug: str, stamp: str, instance_id: str) -> Path:
    day, _, _ = stamp.partition("_")
    path = root / day / f"{slug}_{stamp}_{instance_id}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def test_a_longer_session_name_does_not_capture_this_session_s_instances(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    root = tmp_path / "instances"
    mine = _instance(root, "vehicle_link", "2026-08-21_21-42-01", "35285739")
    # Older, another session, and its name sorts after mine because 'k' > '2'.
    _instance(root, "vehicle_link_kilted_dds", "2026-08-21_20-57-05", "52a752d6")

    found = rosotacom._find_latest_instance_dir(runtime, _session(tmp_path, "vehicle_link"), None)

    assert found == mine.resolve()


def test_the_longer_name_still_finds_its_own_newest(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    root = tmp_path / "instances"
    _instance(root, "vehicle_link", "2026-08-21_21-42-01", "35285739")
    _instance(root, "vehicle_link_kilted_dds", "2026-08-21_20-48-37", "643792ab")
    newest = _instance(root, "vehicle_link_kilted_dds", "2026-08-21_20-57-05", "52a752d6")

    found = rosotacom._find_latest_instance_dir(runtime, _session(tmp_path, "vehicle_link_kilted_dds"), None)

    assert found == newest.resolve()


def test_instances_are_ordered_by_their_stamp(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    root = tmp_path / "instances"
    _instance(root, "link", "2026-08-20_23-59-59", "aaaaaaaa")
    newest = _instance(root, "link", "2026-08-21_00-00-01", "00000000")

    found = rosotacom._find_latest_instance_dir(runtime, _session(tmp_path, "link"), None)

    # The id sorts first among these two; the stamp is what has to decide.
    assert found == newest.resolve()


def test_a_named_instance_of_another_session_is_not_answered(tmp_path: Path) -> None:
    runtime = _runtime(tmp_path)
    root = tmp_path / "instances"
    _instance(root, "vehicle_link_kilted_dds", "2026-08-21_20-57-05", "52a752d6")

    with pytest.raises(RuntimeError, match="No session instance found"):
        rosotacom._find_latest_instance_dir(runtime, _session(tmp_path, "vehicle_link"), "52a752d6")
