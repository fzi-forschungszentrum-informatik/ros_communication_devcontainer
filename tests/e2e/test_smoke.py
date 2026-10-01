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
import time
from collections.abc import Iterator
from pathlib import Path

import pytest
import yaml

from rosotacom.cli import (
    EXAMPLE_PROJECT_DIR,
    SMOKE_HZ_MAX,
    SMOKE_HZ_MIN,
    SMOKE_MAX_DELAY_S,
    local_check_sessions,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(
        os.environ.get("ROSOTACOM_RUN_E2E") != "1",
        reason="Docker-backed E2E tests require ROSOTACOM_RUN_E2E=1.",
    ),
]


# The single-machine smoke matrix is derived from local-check eligibility so it
# cannot drift from the source of truth. Transport combinations live in
# tests/sessions/rmw_matrix and are opt-in for the manual full-suite gate.
def _smoke_id(session_name: str) -> str:
    return session_name.removeprefix("1_heartbeat_").replace("_", "-")


RMW_MATRIX_DIR = PACKAGE_ROOT / "tests" / "sessions" / "rmw_matrix"

SMOKE_SESSIONS = local_check_sessions()
HEARTBEAT_SMOKE_SESSIONS = [
    pytest.param(name, id=_smoke_id(name)) for name in SMOKE_SESSIONS if name.startswith("1_heartbeat")
]
NATIVE_CHATTER_SMOKE_SESSIONS = [
    pytest.param(name, id="native-chatter") for name in SMOKE_SESSIONS if name == "2_native_chatter"
]
COMP_OCC_GRID_SMOKE_SESSIONS = [
    pytest.param(name, id="compressed-occupancy-grid") for name in SMOKE_SESSIONS if name == "3_comp_occ_grid"
]
ZEN_COMP_OCC_GRID_SMOKE_SESSIONS = [
    pytest.param(name, id="compressed-occupancy-grid-zenoh") for name in SMOKE_SESSIONS if name == "4_comp_occ_grid_zen"
]
SIZED_PAYLOAD_SMOKE_SESSIONS = [
    pytest.param(name, id="sized-payload-fastdds") for name in SMOKE_SESSIONS if name == "5_sized_payload"
]
ZEN_SIZED_PAYLOAD_SMOKE_SESSIONS = [
    pytest.param(name, id="sized-payload-zenoh") for name in SMOKE_SESSIONS if name == "6_sized_payload_zen"
]
LINK_LATENCY_SMOKE_SESSIONS = [
    pytest.param(name, id="link-latency") for name in SMOKE_SESSIONS if name == "13_link_latency"
]
REMOTE_ASSIST_ANON_SMOKE_SESSIONS = [
    pytest.param(name, id="remote-assist-anonymized")
    for name in SMOKE_SESSIONS
    if name == "14_remote_assist_anonymized"
]
# Single-stream cuts of the anonymized remote-assist contract: one heavy
# stream per session so its delivery can be measured in isolation.
SINGLE_STREAM_ANON_SMOKE_SESSIONS = [
    pytest.param(name, topic, msg_type, id=f"remote-assist-anonymized-{label}")
    for name, topic, msg_type, label in (
        (
            "15_remote_assist_anonymized_costmap",
            "/topic5",
            "com_msgs/msg/CompressedData",
            "costmap",
        ),
        (
            "16_remote_assist_anonymized_camera",
            "/topic9",
            "ffmpeg_image_transport_msgs/msg/FFMPEGPacket",
            "camera",
        ),
    )
    if name in SMOKE_SESSIONS
]

EXPECTED_HEARTBEAT_CHECKS = (
    "OK: generated plugin.yaml files use literal CLI addresses",
    "OK: a->b inbound bridge heartbeat (/com/in/a/heartbeat_a)",
    "OK: a->b final heartbeat (/heartbeat_a)",
    "OK: b->a inbound bridge heartbeat (/com/in/b/heartbeat_b)",
    "OK: b->a final heartbeat (/heartbeat_b)",
    # Isolation is now asserted in smoke too (local-only topic must not cross);
    # container names vary, so match the stable prefix.
    "OK: isolation holds (/local_only",
)
EXPECTED_NATIVE_CHATTER_CHECKS = (
    "OK: generated plugin.yaml files use literal CLI addresses",
    "OK: smoke publisher b->a /chatter (std_msgs/msg/String) is advertising",
    "OK: b->a inbound bridge topic (/com/in/b/chatter)",
    "OK: b->a final topic (/chatter)",
    "OK: isolation holds (/local_only",
)
EXPECTED_COMP_OCC_GRID_CHECKS = (
    *EXPECTED_HEARTBEAT_CHECKS,
    "OK: smoke publisher b->a /costmap/costmap (nav_msgs/msg/OccupancyGrid) is advertising",
    "OK: b->a inbound bridge topic (/com/in/b/costmap/costmap/restamped/bz2)",
    "OK: b->a final topic (/costmap/costmap/restamped)",
)
EXPECTED_WRAPPED_SIZED_PAYLOAD_CHECKS = (
    "OK: generated plugin.yaml files use literal CLI addresses",
    "OK: smoke publisher a->b /size_test_a (com_msgs/msg/SizedPayload) is advertising",
    "OK: smoke publisher b->a /size_test_b (com_msgs/msg/SizedPayload) is advertising",
    "OK: a->b inbound bridge topic (/com/in/a/size_test_a/ota_stamped)",
    "OK: a->b final topic (/size_test_a)",
    "OK: a->b final topic (/size_test_a) preserves SizedPayload size 66000",
    "OK: b->a inbound bridge topic (/com/in/b/size_test_b/ota_stamped)",
    "OK: b->a final topic (/size_test_b)",
    "OK: b->a final topic (/size_test_b) preserves SizedPayload size 66000",
    "OK: isolation holds (/local_only",
)
EXPECTED_ZEN_SIZED_PAYLOAD_CHECKS = (
    *EXPECTED_HEARTBEAT_CHECKS,
    "OK: smoke publisher a->b /size_test_a (com_msgs/msg/SizedPayload) is advertising",
    "OK: smoke publisher b->a /size_test_b (com_msgs/msg/SizedPayload) is advertising",
    "OK: a->b inbound bridge topic (/com/in/a/size_test_a)",
    "OK: a->b final topic (/size_test_a)",
    "OK: a->b final topic (/size_test_a) preserves SizedPayload size 66000",
    "OK: b->a inbound bridge topic (/com/in/b/size_test_b)",
    "OK: b->a final topic (/size_test_b)",
    "OK: b->a final topic (/size_test_b) preserves SizedPayload size 66000",
)
EXPECTED_REMOTE_ASSIST_ANON_CHECKS = (
    "OK: smoke publisher a->b /to_b/topic1 (geometry_msgs/msg/PoseStamped) is advertising",
    "OK: smoke publisher a->b /to_b/topic2 (std_msgs/msg/Empty) is advertising",
    "OK: smoke publisher b->a /topic9 (ffmpeg_image_transport_msgs/msg/FFMPEGPacket) is advertising",
    "OK: a->b inbound bridge topic (/com/in/a/to_b/topic1)",
    "OK: a->b final topic (/topic1)",
    "OK: b->a inbound bridge topic (/com/in/b/topic3)",
    "OK: b->a final topic (/b/topic3)",
)

# Heartbeat publishers emit at 10 Hz; received rate should stay close to that.
# The bounds are the shared single source of truth from rosotacom.
EXPECTED_HEARTBEAT_HZ = 10.0
HEARTBEAT_HZ_MIN = SMOKE_HZ_MIN
HEARTBEAT_HZ_MAX = SMOKE_HZ_MAX
MAX_HEARTBEAT_DELAY_S = SMOKE_MAX_DELAY_S

_ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]")
_ERROR_PATTERNS = (
    re.compile(r"\[ERROR\]"),
    re.compile(r"\[FATAL\]"),
    re.compile(r"Traceback \(most recent call last\)"),
    re.compile(r"terminate called"),
    re.compile(r"Segmentation fault"),
    re.compile(r"core dumped"),
)
_METRIC_RE = re.compile(
    r"SMOKE_METRIC topic=(?P<topic>\S+) container=(?P<container>\S+) "
    r"hz=(?P<hz>\S+) delay_s=(?P<delay>\S+) label=(?P<label>.+)"
)


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


@pytest.fixture(scope="session")
def copied_example_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom") / "examples"

    _run([sys.executable, "-m", "rosotacom", "examples", "create", str(project)], timeout=60)
    return project


@pytest.fixture(scope="session")
def rmw_matrix_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom") / "rmw-matrix"
    project.mkdir()
    (project / "rosotacom.yaml").write_text(
        "\n".join(
            [
                f"ros2docker_config: {EXAMPLE_PROJECT_DIR / 'ros2docker.json'}",
                "session_configs_dir:",
                f"  - {RMW_MATRIX_DIR}",
                "session_instances_dir: session-instances",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return project


# Force smoke artifacts into the repo workspace instead of the pytest tmp dir so
# CI's "Upload smoke session artifacts" step (path: session-instances/) actually
# captures catmux/domain-bridge logs when a smoke check fails.
SESSION_INSTANCES_DIR = PACKAGE_ROOT / "session-instances"
SCENARIO_NETWORK_SUBNET = "10.139.0.0/24"
SCENARIO_PEER_IPS = {"a": "10.139.0.2", "b": "10.139.0.3"}


@pytest.fixture
def scenario_network() -> Iterator[str]:
    network_name = "rosotacom-scenario-e2e"
    subprocess.run(
        ["docker", "network", "rm", network_name],
        text=True,
        capture_output=True,
        check=False,
    )
    _run(
        ["docker", "network", "create", "--subnet", SCENARIO_NETWORK_SUBNET, network_name],
        timeout=30,
    )
    try:
        yield network_name
    finally:
        subprocess.run(
            ["docker", "network", "rm", network_name],
            text=True,
            capture_output=True,
            check=False,
        )


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


def _scenario_command(
    project: Path,
    action: str,
    identity: str,
    instance_id: str,
    network_name: str | None = None,
) -> list[str]:
    return [
        sys.executable,
        "-m",
        "rosotacom",
        "scenario",
        action,
        "2_native_chatter",
        "--identity",
        identity,
        "--rosotacom-config",
        str(project / "rosotacom.yaml"),
        "--session-instances-dir",
        str(SESSION_INSTANCES_DIR),
        "--instance-id",
        instance_id,
        "--peer-address",
        f"a={SCENARIO_PEER_IPS['a']}",
        "--peer-address",
        f"b={SCENARIO_PEER_IPS['b']}",
        *(
            [
                "--mode",
                "detached",
                "--network-name",
                network_name,
                "--network-ip",
                SCENARIO_PEER_IPS[identity],
            ]
            if action == "start" and network_name
            else []
        ),
    ]


def _interactive_smoke_command(project: Path, *extra: str) -> list[str]:
    return [
        sys.executable,
        "-m",
        "rosotacom",
        "smoke",
        "2_native_chatter",
        "--interactive",
        "--rosotacom-config",
        str(project / "rosotacom.yaml"),
        "--session-instances-dir",
        str(SESSION_INSTANCES_DIR),
        *extra,
    ]


def _artifact_dir(stdout: str) -> Path:
    matches = re.findall(r"Smoke artifacts: (.+)", stdout)
    assert matches, f"no 'Smoke artifacts:' line in smoke output:\n{stdout}"
    return Path(matches[-1].strip())


def _strip_ansi(text: str) -> str:
    return _ANSI_RE.sub("", text)


def _collect_log_files(artifact_dir: Path) -> list[Path]:
    logs_dir = artifact_dir / "logs"
    files = list(logs_dir.glob("*/catmux/*/*.log"))
    files += list(logs_dir.glob("*/docker.log"))
    return sorted(files)


def _scan_logs_for_errors(artifact_dir: Path) -> list[str]:
    hits: list[str] = []
    for path in _collect_log_files(artifact_dir):
        text = _strip_ansi(path.read_text(encoding="utf-8", errors="replace"))
        for lineno, line in enumerate(text.splitlines(), start=1):
            if any(pattern.search(line) for pattern in _ERROR_PATTERNS):
                rel = path.relative_to(artifact_dir)
                hits.append(f"{rel}:{lineno}: {line.strip()}")
    return hits


def _parse_metrics(stdout: str) -> list[dict[str, object]]:
    metrics: list[dict[str, object]] = []
    for line in stdout.splitlines():
        match = _METRIC_RE.search(line)
        if not match:
            continue
        metrics.append(
            {
                "topic": match["topic"],
                "container": match["container"],
                "hz": None if match["hz"] == "nan" else float(match["hz"]),
                "delay_s": None if match["delay"] == "nan" else float(match["delay"]),
                "label": match["label"].strip().strip("'\""),
            }
        )
    return metrics


def _assert_no_ros_or_catmux_errors(session_name: str, artifact_dir: Path) -> None:
    log_files = _collect_log_files(artifact_dir)
    assert log_files, f"no catmux/docker logs found under {artifact_dir}"

    errors = _scan_logs_for_errors(artifact_dir)
    assert not errors, f"ROS/catmux logs for {session_name} contain errors:\n" + "\n".join(errors)


def _assert_heartbeat_rate_and_latency_within_bounds(session_name: str, stdout: str) -> None:
    metrics = _parse_metrics(stdout)
    final_heartbeat_metrics = [m for m in metrics if "final heartbeat" in str(m["label"])]
    assert final_heartbeat_metrics, f"no final heartbeat metrics in smoke output for {session_name}:\n{stdout}"

    hz_problems: list[str] = []
    delay_problems: list[str] = []
    for metric in final_heartbeat_metrics:
        hz = metric["hz"]
        delay_s = metric["delay_s"]
        if not isinstance(hz, float) or not (HEARTBEAT_HZ_MIN <= hz <= HEARTBEAT_HZ_MAX):
            hz_problems.append(f"{metric['label']} ({metric['topic']}): hz={hz}")
        if not isinstance(delay_s, float) or delay_s >= MAX_HEARTBEAT_DELAY_S:
            delay_problems.append(f"{metric['label']} ({metric['topic']}): delay_s={delay_s}")

    assert not hz_problems, (
        f"{session_name} heartbeat rate outside [{HEARTBEAT_HZ_MIN}, {HEARTBEAT_HZ_MAX}] Hz "
        f"(expected ~{EXPECTED_HEARTBEAT_HZ} Hz):\n" + "\n".join(hz_problems)
    )
    assert not delay_problems, f"{session_name} heartbeat latency >= {MAX_HEARTBEAT_DELAY_S}s:\n" + "\n".join(
        delay_problems
    )


def _assert_metric_present(
    session_name: str,
    stdout: str,
    *,
    topic: str,
    label: str,
    require_hz: bool = True,
) -> None:
    matches = [m for m in _parse_metrics(stdout) if m["topic"] == topic and m["label"] == label]
    assert matches, f"no metric for {label} ({topic}) in smoke output for {session_name}:\n{stdout}"
    if require_hz:
        assert any(isinstance(m["hz"], float) for m in matches), (
            f"no publishing rate for {label} ({topic}) in smoke output for {session_name}:\n{stdout}"
        )


def _assert_metric_within_bounds(
    session_name: str,
    stdout: str,
    *,
    topic: str,
    label: str,
    hz_min: float,
    hz_max: float,
    max_delay_s: float,
) -> None:
    matches = [m for m in _parse_metrics(stdout) if m["topic"] == topic and m["label"] == label]
    assert matches, f"no metric for {label} ({topic}) in smoke output for {session_name}:\n{stdout}"
    metric = matches[-1]
    assert isinstance(metric["hz"], float) and hz_min <= metric["hz"] <= hz_max, (
        f"{label} ({topic}) rate {metric['hz']} outside [{hz_min}, {hz_max}] for {session_name}"
    )
    assert isinstance(metric["delay_s"], float) and metric["delay_s"] < max_delay_s, (
        f"{label} ({topic}) delay {metric['delay_s']} >= {max_delay_s}s for {session_name}"
    )


def _load_status_json(artifact_dir: Path, peer: str) -> dict[str, object]:
    path = artifact_dir / "logs" / peer / "status" / "status.json"
    assert path.is_file(), f"no status.json for peer {peer} under {artifact_dir}"
    return json.loads(path.read_text(encoding="utf-8"))


def _find_inbound_stage(status: dict[str, object], *, base_suffix: str, stage: str) -> dict[str, object] | None:
    """The named pipeline stage of the inbound topic whose base ends with base_suffix."""
    for topic in status.get("topics", []):  # type: ignore[union-attr]
        if topic.get("direction") != "inbound" or not str(topic.get("base", "")).endswith(base_suffix):
            continue
        for st in topic.get("stages", []):
            if st.get("stage") == stage:
                return st
    return None


def _read_transit_records(artifact_dir: Path, peer: str) -> list[dict[str, object]]:
    path = artifact_dir / "logs" / peer / "status" / "events.jsonl"
    assert path.is_file(), f"no events.jsonl for peer {peer} under {artifact_dir}"
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        event = json.loads(line)
        if event.get("kind") == "transit":
            rows.append(event)
    return rows


def _read_link_trace_rows(artifact_dir: Path, peer: str) -> list[dict[str, object]]:
    path = artifact_dir / "logs" / peer / "status" / "link_trace.jsonl"
    assert path.is_file(), f"no link_trace.jsonl for peer {peer} under {artifact_dir}"
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


@pytest.mark.parametrize("session_name", LINK_LATENCY_SMOKE_SESSIONS)
def test_local_link_latency_smoke_exposes_metric_backbone(
    copied_example_project: Path,
    session_name: str,
) -> None:
    """measurement records reference end-to-end on the running nodes: the echo heartbeat yields a
    clock-sync estimate, offset correction fires on the wrapped headerless topic
    at com_in, and per-(topic, seq) transit records reach events.jsonl and the
    `rosotacom metrics` digest. Guards the live wiring that the unit tests, which
    run on synthetic rows, cannot see."""
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    artifact_dir = _artifact_dir(result.stdout)
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))
    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)

    # /link_latency_demo is published b->a, so peer 'a' is the receiver that
    # observes the wrapped OtaStamped at com_in and writes the metric backbone.
    status_a = _load_status_json(artifact_dir, "a")

    # Mechanism 2: the symmetric echo produced a min-RTT clock-sync estimate.
    clock_sync = status_a.get("clock_sync")
    assert isinstance(clock_sync, dict), f"no clock_sync block in status.json for {session_name}:\n{status_a}"
    assert clock_sync.get("method") == "echo_min_rtt"
    assert isinstance(clock_sync.get("peer_offset_ms"), (int, float)), f"no peer offset: {clock_sync}"
    assert isinstance(clock_sync.get("rtt_ms"), (int, float)), f"no RTT: {clock_sync}"

    # Mechanism 3: offset-corrected OTA latency fired at com_in for a headerless
    # topic, and the uncorrected value remains a separate field (no silent fallback).
    com_in = _find_inbound_stage(status_a, base_suffix="link_latency_demo", stage="com_in")
    assert com_in is not None, f"no inbound com_in stage for /link_latency_demo in {session_name}:\n{status_a}"
    assert com_in.get("latency_ms") is not None, f"corrected com_in latency missing (offset not applied): {com_in}"
    assert com_in.get("latency_uncorrected_ms") is not None, f"uncorrected com_in latency missing: {com_in}"

    # Mechanism 1 + forensics: events.jsonl carries per-(topic, seq) transit rows.
    transit_rows = _read_transit_records(artifact_dir, "a")
    assert transit_rows, f"no transit records in events.jsonl for {session_name}"
    assert any(row.get("status") == "delivered" for row in transit_rows), (
        f"no delivered transit record for {session_name}: {transit_rows[:3]}"
    )

    link_trace_rows = _read_link_trace_rows(artifact_dir, "a")
    assert link_trace_rows, f"no link trace rows for {session_name}"
    assert any(
        row.get("kind") == "link_trace"
        and row.get("schema_version") == 1
        and isinstance(row.get("passive_counter_delta"), dict)
        and row["passive_counter_delta"].get("provenance") == "proc_net_dev_counter_delta"
        and row["passive_counter_delta"].get("observed_not_available_bandwidth") is True
        and isinstance(row.get("peer_probe"), dict)
        and row["peer_probe"].get("provenance") == "echo_heartbeat_status_snapshot"
        for row in link_trace_rows
    ), f"link_trace.jsonl rows do not expose expected provenance for {session_name}: {link_trace_rows[:3]}"

    # The `rosotacom metrics` digest joins those rows into a non-empty summary.
    events_path = artifact_dir / "logs" / "a" / "status" / "events.jsonl"
    metrics_result = _run(
        [sys.executable, "-m", "rosotacom", "metrics", str(events_path)],
        timeout=60,
    )
    summary = json.loads(metrics_result.stdout)
    assert summary.get("topics"), f"empty metrics digest for {session_name}:\n{metrics_result.stdout}"
    assert any("link_latency_demo" in label for label in summary["topics"]), (
        f"link_latency_demo missing from metrics digest for {session_name}:\n{metrics_result.stdout}"
    )

    # `rosotacom report` turns the same instance into a forensics report: stream
    # summary from the live transit rows, self-describing provenance, and the
    # link-trace context wired through (this run recorded link_trace.jsonl).
    report_result = _run(
        [sys.executable, "-m", "rosotacom", "report", str(artifact_dir), "--json", "--no-figures"],
        timeout=60,
    )
    report = json.loads(report_result.stdout)
    assert any("link_latency_demo" in label for label in report["streams"]), (
        f"link_latency_demo missing from forensics report for {session_name}:\n{report_result.stdout[:2000]}"
    )
    assert report["provenance"]["inputs"]["link_trace"], f"report did not discover link_trace.jsonl: {artifact_dir}"
    assert (artifact_dir / "report" / "report.md").is_file()


@pytest.mark.parametrize("session_name", HEARTBEAT_SMOKE_SESSIONS)
def test_local_heartbeat_smoke_matrix_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_HEARTBEAT_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    assert (artifact_dir / "config" / "a" / "plugin.yaml").is_file()
    assert (artifact_dir / "config" / "b" / "plugin.yaml").is_file()
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)


@pytest.mark.skipif(
    os.environ.get("ROSOTACOM_RUN_FULL_E2E") != "1",
    reason="Full RMW matrix smoke requires ROSOTACOM_RUN_FULL_E2E=1.",
)
@pytest.mark.parametrize(
    "session_name",
    [pytest.param(name, id=_smoke_id(name)) for name in local_check_sessions(RMW_MATRIX_DIR)],
)
def test_full_rmw_heartbeat_smoke_matrix(
    rmw_matrix_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(rmw_matrix_project, session_name), timeout=900)

    for expected in EXPECTED_HEARTBEAT_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    assert (artifact_dir / "config" / "a" / "plugin.yaml").is_file()
    assert (artifact_dir / "config" / "b" / "plugin.yaml").is_file()
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)


@pytest.mark.parametrize("session_name", NATIVE_CHATTER_SMOKE_SESSIONS)
def test_local_native_chatter_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_NATIVE_CHATTER_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    assert (artifact_dir / "config" / "a" / "plugin.yaml").is_file()
    assert (artifact_dir / "config" / "b" / "plugin.yaml").is_file()
    assert (artifact_dir / "config" / "b_to_a_topics.txt").read_text(encoding="utf-8").strip() == "/chatter"
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_metric_present(session_name, result.stdout, topic="/chatter", label="b->a final topic")


@pytest.mark.parametrize("session_name", REMOTE_ASSIST_ANON_SMOKE_SESSIONS)
def test_local_remote_assist_anonymized_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_REMOTE_ASSIST_ANON_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    config_dir = artifact_dir / "config"
    assert (config_dir / "a_to_b_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_a",
        "/topic1",
        "/topic2",
    ]
    assert (config_dir / "b_to_a_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_b",
        *[f"/topic{i}" for i in range(3, 21)],
    ]
    qos = yaml.safe_load((config_dir / "qos.yaml").read_text(encoding="utf-8"))
    assert qos["topics"]["/topic4"]["durability"] == "transient_local"
    assert qos["topics"]["/topic12"]["roles"]["ota_pub"]["lifespan"] == 86400
    pipeline_a = yaml.safe_load((config_dir / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    topic1 = next(topic for topic in pipeline_a["topics"] if topic["base"] == "/topic1")
    assert topic1["stages"][0]["topic"] == "/to_b/topic1"
    assert topic1["stages"][-1]["topic"] == "/ota/a/to_b/topic1"

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)
    _assert_metric_present(session_name, result.stdout, topic="/topic1", label="a->b final topic", require_hz=False)
    _assert_metric_present(session_name, result.stdout, topic="/b/topic3", label="b->a final topic")


@pytest.mark.parametrize(("session_name", "topic", "msg_type"), SINGLE_STREAM_ANON_SMOKE_SESSIONS)
def test_local_single_stream_anonymized_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
    topic: str,
    msg_type: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    assert f"OK: smoke publisher b->a {topic} ({msg_type}) is advertising" in result.stdout
    assert f"OK: b->a inbound bridge topic (/com/in/b{topic})" in result.stdout
    assert f"OK: b->a final topic (/b{topic})" in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    config_dir = artifact_dir / "config"
    # The session is a true single-stream cut: nothing but the heartbeat and the
    # singled-out topic crosses the link.
    assert (config_dir / "a_to_b_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_a",
    ]
    assert (config_dir / "b_to_a_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_b",
        topic,
    ]

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)
    _assert_metric_present(session_name, result.stdout, topic=f"/b{topic}", label="b->a final topic")


def test_native_chatter_scenario_starts_apps_and_communication_together(
    copied_example_project: Path,
    scenario_network: str,
) -> None:
    instance_id = f"scenario-pilot-{time.time_ns()}"
    try:
        for identity in ("a", "b"):
            result = _run(
                _scenario_command(
                    copied_example_project,
                    "start",
                    identity,
                    instance_id,
                    scenario_network,
                ),
                timeout=900,
            )
            assert "rosotacom scenario started: 2_native_chatter" in result.stdout
            assert "inner catmux prefix with Ctrl-b Ctrl-b" in result.stdout

        listing = _run(
            [
                sys.executable,
                "-m",
                "rosotacom",
                "scenario",
                "list",
                "--rosotacom-config",
                str(copied_example_project / "rosotacom.yaml"),
                "--session-instances-dir",
                str(SESSION_INSTANCES_DIR),
            ],
            timeout=30,
        )
        assert "2_native_chatter (active: a, b)" in listing.stdout
        assert "2_native_chatter --identity a" in listing.stdout
        assert "2_native_chatter --identity b" in listing.stdout

        _run(
            [
                sys.executable,
                "-m",
                "rosotacom",
                "test",
                "2_native_chatter",
                "--rosotacom-config",
                str(copied_example_project / "rosotacom.yaml"),
                "--session-instances-dir",
                str(SESSION_INSTANCES_DIR),
                "--instance-id",
                instance_id,
                "--timeout",
                "120",
            ],
            timeout=180,
        )

        manifests = sorted(SESSION_INSTANCES_DIR.glob(f"*/*_{instance_id}/manifest.yaml"))
        assert manifests
        manifest = manifests[-1].read_text(encoding="utf-8")
        assert "2_native_chatter:a" in manifest
        assert "2_native_chatter:b" in manifest
        assert "rosotacom_" in manifest
        scenario_logs = list(manifests[-1].parent.glob("logs/*/scenario/*.log"))
        assert scenario_logs

        tmux_socket = re.search(r"tmux_socket: (.+)", manifest)
        assert tmux_socket
        windows = _run(
            ["tmux", "-L", tmux_socket.group(1), "list-windows", "-t", "2_native_chatter-a", "-F", "#{window_name}"],
            timeout=30,
        )
        assert windows.stdout.splitlines() == ["communication", "native_application"]

        _run(_scenario_command(copied_example_project, "stop", "b", instance_id), timeout=60)
        inferred_stop = _run(
            [
                sys.executable,
                "-m",
                "rosotacom",
                "scenario",
                "stop",
                "--rosotacom-config",
                str(copied_example_project / "rosotacom.yaml"),
                "--session-instances-dir",
                str(SESSION_INSTANCES_DIR),
            ],
            timeout=60,
        )
        assert "Auto-selected active scenario: 2_native_chatter" in inferred_stop.stdout
        assert "Auto-selected active identity: a" in inferred_stop.stdout
    finally:
        for identity in ("a", "b"):
            _run(_scenario_command(copied_example_project, "stop", identity, instance_id), timeout=60)


def test_interactive_native_chatter_smoke_starts_full_local_debug_rig(
    copied_example_project: Path,
) -> None:
    instance_id = f"interactive-smoke-{time.time_ns()}"
    try:
        result = _run(
            _interactive_smoke_command(
                copied_example_project,
                "--mode",
                "detached",
                "--instance-id",
                instance_id,
            ),
            timeout=900,
        )
        assert "rosotacom interactive smoke started: 2_native_chatter (scenario)" in result.stdout
        assert "Smoke peers isolated on docker network" in result.stdout

        listing = _run(
            _interactive_smoke_command(copied_example_project, "--list"),
            timeout=30,
        )
        assert "2_native_chatter (scenario)" in listing.stdout
        assert instance_id in listing.stdout

        manifests = sorted(SESSION_INSTANCES_DIR.glob(f"*/*_{instance_id}/manifest.yaml"))
        assert manifests
        manifest = yaml.safe_load(manifests[-1].read_text(encoding="utf-8"))
        run = manifest["interactive_smoke_runs"]["scenario:2_native_chatter"]
        peer_addresses = set(run["peer_address"])
        assert all(address.startswith(("a=10.137.", "b=10.137.")) for address in peer_addresses)
        plugin_a = manifests[-1].parent / "config" / "a" / "plugin.yaml"
        plugin_b = manifests[-1].parent / "config" / "b" / "plugin.yaml"
        deadline = time.time() + 180
        while time.time() < deadline and not (plugin_a.is_file() and plugin_b.is_file()):
            time.sleep(2)
        assert plugin_a.is_file()
        assert plugin_b.is_file()
        assert "data:" not in plugin_a.read_text(encoding="utf-8")
        assert "data:" not in plugin_b.read_text(encoding="utf-8")

        windows = _run(
            [
                "tmux",
                "-L",
                run["tmux_socket"],
                "list-windows",
                "-t",
                run["tmux_session"],
                "-F",
                "#{window_name}",
            ],
            timeout=30,
        )
        assert windows.stdout.splitlines() == [
            "a_communication",
            "b_communication",
            "a_native_application",
            "b_native_application",
            "verification",
        ]

        _run(
            [
                sys.executable,
                "-m",
                "rosotacom",
                "test",
                "2_native_chatter",
                "--rosotacom-config",
                str(copied_example_project / "rosotacom.yaml"),
                "--session-instances-dir",
                str(SESSION_INSTANCES_DIR),
                "--instance-id",
                instance_id,
                "--timeout",
                "180",
            ],
            timeout=240,
        )

        _run(_interactive_smoke_command(copied_example_project, "--stop"), timeout=120)
        stopped_listing = _run(_interactive_smoke_command(copied_example_project, "--list"), timeout=30)
        assert "(none)" in stopped_listing.stdout
        tmux_check = subprocess.run(
            ["tmux", "-L", run["tmux_socket"], "has-session", "-t", run["tmux_session"]],
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=30,
            check=False,
        )
        assert tmux_check.returncode != 0
        owned_containers = set(run["communication_containers"].values()) | {
            application["container_name"] for application in run["applications"]
        }
        running_containers = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}"],
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=30,
            check=False,
        )
        assert owned_containers.isdisjoint(set(running_containers.stdout.splitlines()))
        network_inspect = subprocess.run(
            ["docker", "network", "inspect", run["network_name"]],
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=30,
            check=False,
        )
        assert network_inspect.returncode != 0
    finally:
        subprocess.run(
            _interactive_smoke_command(copied_example_project, "--stop"),
            cwd=PACKAGE_ROOT,
            text=True,
            capture_output=True,
            timeout=120,
            check=False,
        )


@pytest.mark.parametrize("session_name", COMP_OCC_GRID_SMOKE_SESSIONS)
def test_local_compressed_occupancy_grid_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_COMP_OCC_GRID_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    config_dir = artifact_dir / "config"
    assert (config_dir / "b_to_a_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_b",
        "/costmap/costmap/restamped/bz2",
    ]
    assert "costmap/costmap/restamped" in (config_dir / "b" / "compression.yaml").read_text(encoding="utf-8")
    assert "costmap/costmap/restamped/bz2" in (config_dir / "a" / "decompression.yaml").read_text(encoding="utf-8")
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)
    _assert_metric_within_bounds(
        session_name,
        result.stdout,
        topic="/costmap/costmap/restamped",
        label="b->a final topic",
        hz_min=1.0,
        hz_max=5.0,
        max_delay_s=0.5,
    )


@pytest.mark.parametrize("session_name", ZEN_COMP_OCC_GRID_SMOKE_SESSIONS)
def test_local_zenoh_compressed_occupancy_grid_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_COMP_OCC_GRID_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    config_dir = artifact_dir / "config"
    assert (config_dir / "b_to_a_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_b",
        "/costmap/costmap/restamped/bz2",
    ]
    assert "use_zenoh_ros2dds: true" in (config_dir / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert 'priority: "data"' in (config_dir / "b" / "plugin.yaml").read_text(encoding="utf-8")
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)
    _assert_metric_present(
        session_name,
        result.stdout,
        topic="/costmap/costmap/restamped",
        label="b->a final topic",
    )


@pytest.mark.parametrize("session_name", SIZED_PAYLOAD_SMOKE_SESSIONS)
def test_local_wrapped_sized_payload_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_WRAPPED_SIZED_PAYLOAD_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    config_dir = artifact_dir / "config"
    assert (config_dir / "a_to_b_topics.txt").read_text(encoding="utf-8").strip() == "/size_test_a/ota_stamped"
    assert (config_dir / "b_to_a_topics.txt").read_text(encoding="utf-8").strip() == "/size_test_b/ota_stamped"
    assert "size_test_a" in (config_dir / "a" / "ota_wrapper.yaml").read_text(encoding="utf-8")
    assert "size_test_b/ota_stamped" in (config_dir / "a" / "ota_unwrapper.yaml").read_text(encoding="utf-8")
    assert "size_test_b" in (config_dir / "b" / "ota_wrapper.yaml").read_text(encoding="utf-8")
    assert "size_test_a/ota_stamped" in (config_dir / "b" / "ota_unwrapper.yaml").read_text(encoding="utf-8")
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_metric_present(session_name, result.stdout, topic="/size_test_a", label="a->b final topic")
    _assert_metric_present(session_name, result.stdout, topic="/size_test_b", label="b->a final topic")


@pytest.mark.parametrize("session_name", ZEN_SIZED_PAYLOAD_SMOKE_SESSIONS)
def test_local_zenoh_sized_payload_smoke_from_copied_example_project(
    copied_example_project: Path,
    session_name: str,
) -> None:
    result = _run(_smoke_command(copied_example_project, session_name), timeout=900)

    for expected in EXPECTED_ZEN_SIZED_PAYLOAD_CHECKS:
        assert expected in result.stdout

    artifact_dir = _artifact_dir(result.stdout)
    config_dir = artifact_dir / "config"
    assert (config_dir / "a_to_b_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_a",
        "/size_test_a",
    ]
    assert (config_dir / "b_to_a_topics.txt").read_text(encoding="utf-8").splitlines() == [
        "/heartbeat_b",
        "/size_test_b",
    ]
    assert "use_zenoh_ros2dds: true" in (config_dir / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert 'priority: "data"' in (config_dir / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert list((artifact_dir / "logs").glob("*/catmux/*/*.log"))

    _assert_no_ros_or_catmux_errors(session_name, artifact_dir)
    _assert_heartbeat_rate_and_latency_within_bounds(session_name, result.stdout)
    _assert_metric_present(session_name, result.stdout, topic="/size_test_a", label="a->b final topic")
    _assert_metric_present(session_name, result.stdout, topic="/size_test_b", label="b->a final topic")
