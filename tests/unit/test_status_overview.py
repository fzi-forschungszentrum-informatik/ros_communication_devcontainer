# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unit tests for the rosotacom status / debugging overview.

Covers three layers:
  * pipeline_spec.yaml generation (generate_session_files._build_status_pipeline_spec
    exercised end-to-end via func()),
  * state classification + rollup (status_overview_core, ROS-independent),
  * the `rosotacom status` host CLI rendering.

The in-container modules live under resources/ws and are loaded by file path so
the host test suite (which has no rclpy) can import the pure parts.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path
from types import ModuleType

import pytest
import yaml

import rosotacom.cli as rosotacom

REPO_ROOT = Path(__file__).resolve().parents[2]
WS = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws"
GENERATOR_PY = WS / "session" / "creation" / "generate_session_files.py"
STATUS_CORE_PY = WS / "ros2src" / "com_py" / "com_py" / "status_overview_core.py"
STATUS_NODE_PY = WS / "ros2src" / "com_py" / "com_py" / "status_overview.py"


def _load_module(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    # Register before exec so dataclasses can resolve the module's namespace.
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


generator = _load_module(GENERATOR_PY, "rosotacom_generate_session_files")
core = _load_module(STATUS_CORE_PY, "rosotacom_status_overview_core")


# ---------------------------------------------------------------------------
# pipeline_spec generation
# ---------------------------------------------------------------------------


def _heartbeat_cfg() -> dict:
    return {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 46}, "b": {"domain_id": 47}},
        "shared": {
            "use_heartbeat": True,
            "use_status_overview": True,
            "rmw": {
                "local": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
                "ota": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
            },
            "ota_domain_id": 48,
        },
    }


def _generate(cfg: dict, output_dir: Path) -> None:
    generator.func(
        session_config_obj=cfg,
        output_dir=str(output_dir),
        force=True,
        peer_addresses={"a": "127.0.0.1", "b": "127.0.0.2"},
    )


def test_pipeline_spec_generated_for_heartbeat(tmp_path: Path) -> None:
    import yaml

    _generate(_heartbeat_cfg(), tmp_path)

    spec_path = tmp_path / "a" / "pipeline_spec.yaml"
    assert spec_path.exists(), "pipeline_spec.yaml should be generated for peer a"
    spec = yaml.safe_load(spec_path.read_text(encoding="utf-8"))

    assert spec["peer"] == "a"
    assert spec["remote"] == "b"
    assert spec["local_domain_id"] == 46
    assert spec["ota_domain_id"] == 48
    assert spec["uses_domain_bridge"] is True

    by_dir = {(t["base"], t["direction"]): t for t in spec["topics"]}
    out = by_dir[("/heartbeat_a", "outbound")]
    out_stages = {s["stage"]: s for s in out["stages"]}
    assert out_stages["native"]["topic"] == "/heartbeat_a"
    assert out_stages["native"]["domain"] == "local"
    assert out_stages["com_out"]["topic"] == "/com/out/a/heartbeat_a"
    assert out_stages["ota_sent"]["topic"] == "/ota/a/heartbeat_a"
    assert out_stages["ota_sent"]["domain"] == "ota"

    inb = by_dir[("/heartbeat_b", "inbound")]
    in_stages = {s["stage"]: s for s in inb["stages"]}
    assert in_stages["ota_recv"]["topic"] == "/ota/b/heartbeat_b"
    assert in_stages["ota_recv"]["domain"] == "ota"
    assert in_stages["com_in"]["topic"] == "/com/in/b/heartbeat_b"
    assert in_stages["app_in"]["topic"] == "/heartbeat_b"


def test_pipeline_spec_absent_when_flag_off(tmp_path: Path) -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_status_overview"] = False
    _generate(cfg, tmp_path)
    assert not (tmp_path / "a" / "pipeline_spec.yaml").exists()


def test_link_trace_config_generates_status_recorder_params(tmp_path: Path) -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["link_trace"] = {
        "enabled": True,
        "interval_s": 0.5,
        "modem_metrics_command": "cat /tmp/modem.json",
        "modem_metrics_timeout_s": 1.5,
    }
    _generate(cfg, tmp_path)

    plugin = yaml.safe_load((tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8"))
    params = plugin["parameters"]

    assert params["status_overview"] is True
    assert params["status_spec_file"] == "${peer_dir}/pipeline_spec.yaml"
    assert params["status_write_interval_s"] == 0.5
    assert params["link_trace"] is True
    assert params["link_trace_interval_s"] == 0.5
    assert params["link_trace_modem_command"] == "cat /tmp/modem.json"
    assert params["link_trace_modem_timeout_s"] == 1.5
    assert (tmp_path / "a" / "pipeline_spec.yaml").is_file()


def test_link_trace_requires_status_overview() -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_status_overview"] = False
    cfg["shared"]["link_trace"] = {"enabled": True}

    with pytest.raises(RuntimeError, match="shared.link_trace.enabled requires"):
        generator.func(
            session_config_obj=cfg,
            output_dir="/tmp/unused",
            force=True,
            peer_addresses={"a": "127.0.0.1", "b": "127.0.0.2"},
        )


def test_use_status_overview_must_be_bool() -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_status_overview"] = "yes"
    with pytest.raises(RuntimeError):
        generator._validate_session_template_cfg(cfg)


def test_ota_observer_is_graph_only() -> None:
    """Regression: status monitoring must never create an OTA DataReader."""
    source = STATUS_NODE_PY.read_text(encoding="utf-8")
    assert "subscribe_to_messages=False" in source
    assert "if not self._subscribe_to_messages:" in source
    assert 'local_topics.update(by_domain.get("ota"' not in source


def test_observation_qos_matches_publisher_durability() -> None:
    """Regression: the observer must adopt the publisher's QoS so it can read a
    TRANSIENT_LOCAL writer's held sample. A hardcoded volatile observer races the
    single startup publish of a static latched topic -> intermittent false STALLED.
    """
    source = STATUS_NODE_PY.read_text(encoding="utf-8")
    # Observation subscription QoS is derived per-topic from the live publisher.
    assert "def _observation_qos" in source
    assert "self.get_publishers_info_by_topic(topic)" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
    # The old unconditional volatile/best-effort observer QoS is gone.
    assert "qos = self._observation_qos(topic)" in source


def test_observation_qos_does_not_force_transient_local_for_latched_roles() -> None:
    """#231 regression: the observer must NOT force TRANSIENT_LOCAL for every
    stage of a latched topic. A latched topic's native source stage is published
    VOLATILE (the ROS 1 bridge offers no durability), and a TRANSIENT_LOCAL
    reader is incompatible with a VOLATILE writer -- forcing it made the native
    stage receive nothing and read a false STALLED (broke e2e-remote-assist).
    The per-publisher QoS match already requests transient_local on the delivered
    stages that offer it, which is the correct per-stage form of Option A.
    """
    source = STATUS_NODE_PY.read_text(encoding="utf-8")
    assert "self._latched_topics" not in source
    assert "collect_latched_stage_topics" not in source


# ---------------------------------------------------------------------------
# state classification + rollup
# ---------------------------------------------------------------------------


class _FakeObserver:
    def __init__(self) -> None:
        self.observations: dict = {}


def _outbound_spec() -> dict:
    return {
        "peer": "a",
        "remote": "b",
        "local_domain_id": 46,
        "ota_domain_id": 48,
        "uses_domain_bridge": True,
        "topics": [
            {
                "base": "/heartbeat_a",
                "direction": "outbound",
                "source": "a",
                "target": "b",
                "type": "com_msgs/msg/EchoHeartbeat",
                "expected_hz": None,
                "stages": [
                    {"stage": "native", "topic": "/heartbeat_a", "domain": "local", "produced_by": "heartbeat_echo"},
                    {
                        "stage": "com_out",
                        "topic": "/com/out/a/heartbeat_a",
                        "domain": "local",
                        "produced_by": "relay_out",
                    },
                    {"stage": "ota_sent", "topic": "/ota/a/heartbeat_a", "domain": "ota", "produced_by": "bridge_out"},
                ],
            }
        ],
    }


def _build(observers: dict, output_dir: Path) -> core.StatusAggregator:
    return core.StatusAggregator(
        logger=None,
        spec=_outbound_spec(),
        output_dir=str(output_dir),
        observers_by_domain=observers,
        liveness_window_s=3.0,
        stale_after_s=3.0,
    )


def _obs(
    pub: int = 0,
    last_msg_at: float | None = None,
    *,
    graph_only: bool = False,
) -> core.StageObservation:
    o = core.StageObservation(graph_only=graph_only)
    o.pub_count = pub
    o.sub_count = 1 if pub else 0
    if last_msg_at is not None:
        o.record(size=64, delay_s=0.01, now_mono=last_msg_at, now_wall=1_700_000_000.0)
    return o


def test_rollup_ok_when_all_stages_flow(tmp_path: Path) -> None:
    now = 100.0
    local = _FakeObserver()
    ota = _FakeObserver()
    local.observations["/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    local.observations["/com/out/a/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=1, graph_only=True)

    agg = _build({"local": local, "ota": ota}, tmp_path)
    snap = agg.build_snapshot(now_mono=now)
    topic = snap["topics"][0]
    assert topic["overall"] == core.OK
    assert topic["reached_stage"] == "ota_sent"
    assert topic["blocked_at"] is None
    assert "Phase 1" in topic["diagnosis"]
    ota_stage = {s["stage"]: s for s in topic["stages"]}["ota_sent"]
    assert ota_stage["state"] == core.FLOWING
    assert ota_stage["observation"] == "graph"
    assert ota_stage["inferred_from"] == "/com/out/a/heartbeat_a"
    assert ota_stage["messages_total"] == 1
    assert snap["summary"]["OK"] == 1


def _sr(stage: str, topic: str, state: str, publishers: int = 1) -> dict:
    return {"stage": stage, "topic": topic, "state": state, "publishers": publishers, "produced_by": "relay"}


def test_rollup_latched_delivered_then_stale_is_ok(tmp_path: Path) -> None:
    # mode: latched -- a static topic that delivered its value and now idles is
    # OK, not STALLED (mirrors status_eval; expectation concepts reference).
    agg = _build({}, tmp_path)
    spec = {"direction": "inbound", "expect": {"mode": "latched"}}
    stages = [
        _sr("ota_recv", "/ota/b/site/latched", core.STALE),
        _sr("com_in", "/com/in/b/site/latched", core.STALE),
        _sr("app_in", "/b/site/latched", core.STALE),
    ]
    roll = agg.rollup(spec, stages)
    assert roll["overall"] == core.OK
    assert roll["blocked_at"] is None
    assert "latched" in roll["diagnosis"]


def test_rollup_latched_never_delivered_still_stalled(tmp_path: Path) -> None:
    agg = _build({}, tmp_path)
    spec = {"direction": "inbound", "expect": {"mode": "latched"}}
    stages = [_sr("ota_recv", "/ota/b/site/latched", core.IDLE), _sr("app_in", "/b/site/latched", core.IDLE)]
    assert agg.rollup(spec, stages)["overall"] == core.STALLED


def test_stage_diagnosis_names_the_three_latched_states(tmp_path: Path) -> None:
    """#231: a latched topic that is not delivering reads as one of three
    distinct states. The new one -- a publisher present but no retained value
    held -- must not collapse into 'no publisher', which is the difference
    between a broken latch and a genuinely silent producer."""
    agg = _build({}, tmp_path)
    topic = "/b/site/latched"
    no_publisher = agg._stage_diagnosis({"state": core.ABSENT, "topic": topic, "produced_by": "relay"})
    stopped = agg._stage_diagnosis({"state": core.STALE, "topic": topic, "produced_by": "relay"})
    no_retained = agg._stage_diagnosis({"state": core.IDLE, "topic": topic, "produced_by": "relay", "latched": True})
    plain_idle = agg._stage_diagnosis({"state": core.IDLE, "topic": "/heartbeat_a", "produced_by": "relay"})
    assert "no publisher" in no_publisher
    assert "stopped" in stopped
    assert "retained latched" in no_retained
    # all three name-distinct, and the latched case is not the plain-idle text
    assert len({no_publisher, stopped, no_retained}) == 3
    assert no_retained != plain_idle


def test_classify_stage_marks_latched_role(tmp_path: Path) -> None:
    """classify_stage carries the latched flag so _stage_diagnosis can name the
    third state without re-reading the spec."""
    agg = _build({}, tmp_path)
    latched = agg.classify_stage(
        {"stage": "app_in", "topic": "/b/site/latched"},
        None,
        0.0,
        {"mode": "latched"},
    )
    streamed = agg.classify_stage({"stage": "native", "topic": "/b/twist"}, None, 0.0, {"mode": "stream"})
    assert latched["latched"] is True
    assert streamed["latched"] is False


def test_rollup_latched_outbound_produced_is_ok(tmp_path: Path) -> None:
    # On the sender a one-shot held value shows at native/processed; the inferred
    # send stage may idle. Producing+latching is enough (receiver confirms delivery).
    agg = _build({}, tmp_path)
    spec = {"direction": "outbound", "expect": {"mode": "latched"}}
    stages = [
        _sr("native", "/site", core.FLOWING),
        _sr("processed", "/site/latched", core.STALE),
        _sr("ota_sent", "/ota/b/site/latched", core.IDLE),
    ]
    roll = agg.rollup(spec, stages)
    assert roll["overall"] == core.OK
    assert "produced" in roll["diagnosis"]


def test_rollup_existence_present_is_ok(tmp_path: Path) -> None:
    agg = _build({}, tmp_path)
    spec = {"direction": "inbound", "expect": {"mode": "existence"}}
    roll = agg.rollup(spec, [_sr("app_in", "/b/diag", core.IDLE, publishers=1)])
    assert roll["overall"] == core.OK
    assert "existence" in roll["diagnosis"]


def test_rollup_without_mode_keeps_stalled(tmp_path: Path) -> None:
    # No mode (default stream): a delivered-then-stale topic is STALLED as before.
    agg = _build({}, tmp_path)
    assert agg.rollup({"direction": "inbound"}, [_sr("app_in", "/b/x", core.STALE)])["overall"] == core.STALLED


def test_inbound_ota_receipt_is_inferred_from_com_in(tmp_path: Path) -> None:
    now = 100.0
    spec = {
        "peer": "a",
        "remote": "b",
        "topics": [
            {
                "base": "/heartbeat_b",
                "direction": "inbound",
                "stages": [
                    {
                        "stage": "ota_recv",
                        "topic": "/ota/b/heartbeat_b",
                        "domain": "ota",
                    },
                    {
                        "stage": "com_in",
                        "topic": "/com/in/b/heartbeat_b",
                        "domain": "local",
                    },
                    {
                        "stage": "app_in",
                        "topic": "/heartbeat_b",
                        "domain": "local",
                    },
                ],
            }
        ],
    }
    local = _FakeObserver()
    ota = _FakeObserver()
    ota.observations["/ota/b/heartbeat_b"] = _obs(pub=1, graph_only=True)
    local.observations["/com/in/b/heartbeat_b"] = _obs(pub=1, last_msg_at=now)
    local.observations["/heartbeat_b"] = _obs(pub=1, last_msg_at=now)

    agg = core.StatusAggregator(
        logger=None,
        spec=spec,
        output_dir=str(tmp_path),
        observers_by_domain={"local": local, "ota": ota},
    )
    topic = agg.build_snapshot(now_mono=now)["topics"][0]
    ota_stage = {s["stage"]: s for s in topic["stages"]}["ota_recv"]
    assert topic["overall"] == core.OK
    assert ota_stage["state"] == core.FLOWING
    assert ota_stage["inferred_from"] == "/com/in/b/heartbeat_b"


def test_rollup_partial_identifies_first_broken_stage(tmp_path: Path) -> None:
    now = 100.0
    local = _FakeObserver()
    ota = _FakeObserver()
    local.observations["/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    # com_out has a publisher but no messages observed -> IDLE
    local.observations["/com/out/a/heartbeat_a"] = _obs(pub=1, last_msg_at=None)
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=0, last_msg_at=None)

    agg = _build({"local": local, "ota": ota}, tmp_path)
    snap = agg.build_snapshot(now_mono=now)
    topic = snap["topics"][0]
    assert topic["overall"] == core.PARTIAL
    assert topic["reached_stage"] == "native"
    assert topic["blocked_at"] == "com_out"
    stages = {s["stage"]: s for s in topic["stages"]}
    assert stages["com_out"]["state"] == core.IDLE
    assert stages["ota_sent"]["state"] == core.ABSENT


def test_rollup_absent_when_nothing_publishes(tmp_path: Path) -> None:
    now = 100.0
    local = _FakeObserver()
    ota = _FakeObserver()
    local.observations["/heartbeat_a"] = _obs(pub=0)
    local.observations["/com/out/a/heartbeat_a"] = _obs(pub=0)
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=0)

    agg = _build({"local": local, "ota": ota}, tmp_path)
    snap = agg.build_snapshot(now_mono=now)
    topic = snap["topics"][0]
    assert topic["overall"] == core.ABSENT
    assert topic["reached_stage"] is None
    assert topic["blocked_at"] == "native"


def test_rollup_stalled_when_last_stage_goes_stale(tmp_path: Path) -> None:
    now = 100.0
    local = _FakeObserver()
    ota = _FakeObserver()
    local.observations["/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    local.observations["/com/out/a/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    # ota stage last received 10s ago -> STALE (stale_after=3)
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=1, last_msg_at=now - 10.0)

    agg = _build({"local": local, "ota": ota}, tmp_path)
    snap = agg.build_snapshot(now_mono=now)
    topic = snap["topics"][0]
    assert topic["overall"] == core.STALLED
    assert topic["reached_stage"] == "ota_sent"
    stages = {s["stage"]: s for s in topic["stages"]}
    assert stages["ota_sent"]["state"] == core.STALE


def test_write_produces_artifacts_and_transition_events(tmp_path: Path) -> None:
    now = 100.0
    local = _FakeObserver()
    ota = _FakeObserver()
    local.observations["/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    local.observations["/com/out/a/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=0, graph_only=True)

    agg = _build({"local": local, "ota": ota}, tmp_path)
    # First write establishes baseline (no transition events emitted).
    assert agg.write(now_mono=now) == 0
    assert (tmp_path / "status.json").exists()
    assert (tmp_path / "status.txt").exists()
    assert not (tmp_path / "events.jsonl").exists()

    # Now the ota stage starts flowing -> overall changes -> one event row.
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=1, graph_only=True)
    assert agg.write(now_mono=now) == 1
    events_text = (tmp_path / "events.jsonl").read_text(encoding="utf-8").strip().splitlines()
    assert len(events_text) == 1


# ---------------------------------------------------------------------------
# `rosotacom status` CLI rendering
# ---------------------------------------------------------------------------


def _make_project(tmp_path: Path) -> tuple[argparse.Namespace, Path]:
    (tmp_path / "ros2docker.json").write_text('{"image_name": "test"}\n', encoding="utf-8")
    sessions = tmp_path / "sessions" / "mysess"
    sessions.mkdir(parents=True)
    (sessions / "session-definition.yaml").write_text("peers: {}\n", encoding="utf-8")
    (tmp_path / "session-instances").mkdir()
    config = tmp_path / "rosotacom.yaml"
    config.write_text(
        "ros2docker_config: ros2docker.json\n"
        "session_configs_dir:\n"
        "  - sessions\n"
        "session_instances_dir: session-instances\n",
        encoding="utf-8",
    )
    instance = tmp_path / "session-instances" / "2026-01-01" / "mysess_2026-01-01_00-00-00_abcd1234"
    status_dir = instance / "logs" / "a" / "status"
    status_dir.mkdir(parents=True)
    import json

    snapshot = {
        "schema_version": 1,
        "phase": 1,
        "generated_at": "2026-01-01T00:00:01.000",
        "peer": "a",
        "remote": "b",
        "summary": {"OK": 1, "PARTIAL": 0, "STALLED": 0, "ABSENT": 0},
        "topics": [
            {
                "base": "/heartbeat_a",
                "direction": "outbound",
                "overall": "OK",
                "reached_stage": "ota_sent",
                "blocked_at": None,
                "diagnosis": "all observable stages flowing",
                "stages": [],
            }
        ],
    }
    (status_dir / "status.json").write_text(json.dumps(snapshot), encoding="utf-8")
    (status_dir / "status.txt").write_text("RENDERED STATUS TEXT\n", encoding="utf-8")

    args = argparse.Namespace(
        rosotacom_config=str(config),
        ros2docker_config=None,
        session_configs_dir=None,
        session_instances_dir=None,
        deployment=None,
        session_dir="mysess",
        identity=None,
        instance_id=None,
        json=False,
        watch=False,
        watch_interval=2.0,
    )
    return args, instance


def test_status_cli_human_output(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    args, _ = _make_project(tmp_path)
    rc = rosotacom.status(args)
    assert rc == 0
    out = capsys.readouterr().out
    assert "RENDERED STATUS TEXT" in out


def test_status_cli_json_output(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    import json

    args, _ = _make_project(tmp_path)
    args.json = True
    rc = rosotacom.status(args)
    assert rc == 0
    out = capsys.readouterr().out
    payload = json.loads(out)
    assert "a" in payload
    assert payload["a"]["topics"][0]["base"] == "/heartbeat_a"


# ---------------------------------------------------------------------------
# per-topic `expect` -> live classification + spec
# ---------------------------------------------------------------------------


def _quality(tmp_path: Path, m: dict, expected_hz, expect) -> tuple:
    agg = _build({"local": _FakeObserver(), "ota": _FakeObserver()}, tmp_path)
    return agg._classify_quality(m, expected_hz, expect)


def test_classify_quality_hz_outside_expect_range_is_bad(tmp_path: Path) -> None:
    assert _quality(tmp_path, {"hz": 3.0, "last_delay_s": 0.01}, None, {"hz": {"min": 5, "max": 20}}) == (
        core.BAD,
        "hz",
    )
    assert _quality(tmp_path, {"hz": 25.0, "last_delay_s": 0.01}, None, {"hz": {"max": 20}}) == (core.BAD, "hz")


def test_classify_quality_latency_over_expect_max_is_bad(tmp_path: Path) -> None:
    assert _quality(tmp_path, {"hz": 10.0, "last_delay_s": 0.25}, None, {"latency_ms": {"max": 200}}) == (
        core.BAD,
        "latency",
    )


def test_classify_quality_within_expect_is_good(tmp_path: Path) -> None:
    expect = {"hz": {"min": 5, "max": 20}, "latency_ms": {"max": 200}}
    assert _quality(tmp_path, {"hz": 10.0, "last_delay_s": 0.01}, None, expect) == (core.GOOD, None)


def test_classify_quality_without_expect_uses_expected_hz_heuristic(tmp_path: Path) -> None:
    assert _quality(tmp_path, {"hz": 4.0, "last_delay_s": 0.01}, 10.0, None) == (core.BAD, "hz")


def test_build_snapshot_applies_per_topic_expect(tmp_path: Path) -> None:
    # Full node path: a spec topic carrying `expect` is classified against it and
    # the contract is surfaced in the snapshot.
    now = 100.0
    spec = _outbound_spec()
    spec["topics"][0]["expect"] = {"hz": {"min": 5, "max": 20}}
    local = _FakeObserver()
    ota = _FakeObserver()
    local.observations["/heartbeat_a"] = _obs(pub=1, last_msg_at=now)  # ~0.33 Hz, below min 5
    local.observations["/com/out/a/heartbeat_a"] = _obs(pub=1, last_msg_at=now)
    ota.observations["/ota/a/heartbeat_a"] = _obs(pub=1, graph_only=True)

    agg = core.StatusAggregator(
        logger=None,
        spec=spec,
        output_dir=str(tmp_path),
        observers_by_domain={"local": local, "ota": ota},
        liveness_window_s=3.0,
        stale_after_s=3.0,
    )
    snap = agg.build_snapshot(now_mono=now)
    assert snap["topics"][0]["expect"] == {"hz": {"min": 5, "max": 20}}
    native = {s["stage"]: s for s in snap["topics"][0]["stages"]}["native"]
    assert native["quality"] == core.BAD
    assert native["quality_reason"] == "hz"


def test_pipeline_spec_carries_per_topic_expect(tmp_path: Path) -> None:
    import yaml

    cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 46}, "b": {"domain_id": 47}},
        "shared": {
            "use_status_overview": True,
            "rmw": {
                "local": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
                "ota": {"cyclone": {"config": "cyclonedds_minimal.xml"}},
            },
            "ota_domain_id": 48,
        },
        "topics": {
            "b_to_a": [
                {
                    "topic": "/costmap",
                    "type": "nav_msgs/msg/OccupancyGrid",
                    "expect": {"hz": {"min": 1, "max": 5}, "latency_ms": {"max": 500}},
                }
            ]
        },
    }
    _generate(cfg, tmp_path)
    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    by_dir = {(t["base"], t["direction"]): t for t in spec["topics"]}
    assert by_dir[("/costmap", "inbound")]["expect"] == {"hz": {"min": 1, "max": 5}, "latency_ms": {"max": 500}}


def test_pipeline_spec_uses_postprocessed_topics_and_stage_specific_types(tmp_path: Path) -> None:
    import yaml

    examples = Path(__file__).resolve().parents[2] / "src" / "rosotacom" / "resources" / "examples" / "sessions"

    occupancy_cfg = yaml.safe_load(
        (examples / "3_comp_occ_grid" / "session-definition.yaml").read_text(encoding="utf-8")
    )
    _generate(occupancy_cfg, tmp_path / "occupancy")
    occupancy_spec = yaml.safe_load((tmp_path / "occupancy" / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    occupancy_in = next(
        topic
        for topic in occupancy_spec["topics"]
        if topic["base"] == "/costmap/costmap" and topic["direction"] == "inbound"
    )
    occupancy_stages = {stage["stage"]: stage for stage in occupancy_in["stages"]}
    assert occupancy_stages["app_in"]["topic"] == "/costmap/costmap/restamped/bz2"
    assert occupancy_stages["app_in"]["type"] == "com_msgs/msg/CompressedData"
    assert occupancy_stages["native_in"]["topic"] == "/costmap/costmap/restamped"
    assert occupancy_stages["native_in"]["type"] == "nav_msgs/msg/OccupancyGrid"

    payload_cfg = yaml.safe_load((examples / "5_sized_payload" / "session-definition.yaml").read_text(encoding="utf-8"))
    _generate(payload_cfg, tmp_path / "payload")
    payload_spec = yaml.safe_load((tmp_path / "payload" / "b" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    payload_out = next(
        topic
        for topic in payload_spec["topics"]
        if topic["base"] == "/size_test_b" and topic["direction"] == "outbound"
    )
    payload_out_stages = {stage["stage"]: stage for stage in payload_out["stages"]}
    assert payload_out_stages["native"]["type"] == "com_msgs/msg/SizedPayload"
    assert payload_out_stages["processed"]["type"] == "com_msgs/msg/OtaStamped"

    collected = core.collect_stage_topics(payload_spec)
    assert collected["local"]["/size_test_b"] == "com_msgs/msg/SizedPayload"
    assert collected["local"]["/size_test_b/ota_stamped"] == "com_msgs/msg/OtaStamped"


def test_pipeline_spec_exposes_decoded_reverse_transport_stage(tmp_path: Path) -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_heartbeat"] = False
    cfg["shared"]["metric_backbone"] = {"record_stages": True}
    cfg["topics"] = {
        "b_to_a": [
            {
                "topic": "/camera/image",
                "type": "sensor_msgs/msg/Image",
                "processing": {
                    "transport": {
                        "type": "ffmpeg",
                        "local_republish": "raw",
                        "remote_republish": "raw",
                        "gop_size": 4,
                        "bit_rate": 500000,
                    }
                },
            }
        ]
    }

    _generate(cfg, tmp_path)

    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    inbound = next(topic for topic in spec["topics"] if topic["base"] == "/camera/image")
    stages = {stage["stage"]: stage for stage in inbound["stages"]}
    assert stages["app_in"]["topic"] == "/camera/image/ffmpeg"
    assert stages["app_in"]["type"] == "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"
    assert stages["native_in"]["topic"] == "/camera/image/ffmpeg/raw"
    assert stages["native_in"]["type"] == "sensor_msgs/msg/Image"

    plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert "irt_1_out_transport: raw" in plugin
    assert "metric_stage_topics: /com/in/b/camera/image/ffmpeg,/camera/image/ffmpeg,/camera/image/ffmpeg/raw" in plugin


def test_a_compressed_receiver_decode_names_the_stage_and_its_type(tmp_path: Path) -> None:
    """#276: the decoded twin follows `remote_republish`, on the name and the type.

    An uncompressed `sensor_msgs/Image` twin is ~3 MB per frame at 1280x800 and
    dominates any `record -a` on the receiving machine; a session that asks for
    `compressed` must get a `CompressedImage` under `.../ffmpeg/compressed`, and
    the status overview has to report that stage, not the one it used to assume.
    """
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_heartbeat"] = False
    cfg["topics"] = {
        "b_to_a": [
            {
                "topic": "/camera/image",
                "type": "sensor_msgs/msg/Image",
                "processing": {
                    "transport": {
                        "type": "ffmpeg",
                        # Deliberately asymmetric: the sender keeps pixels for a
                        # local look, the receiver takes the small copy.
                        "local_republish": "raw",
                        "remote_republish": "compressed",
                        "gop_size": 4,
                        "bit_rate": 500000,
                    }
                },
            }
        ]
    }

    _generate(cfg, tmp_path)

    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    inbound = next(topic for topic in spec["topics"] if topic["base"] == "/camera/image")
    stages = {stage["stage"]: stage for stage in inbound["stages"]}
    assert stages["native_in"]["topic"] == "/camera/image/ffmpeg/compressed"
    assert stages["native_in"]["type"] == "sensor_msgs/msg/CompressedImage"

    receiver_plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert "irt_1_out_transport: compressed" in receiver_plugin

    # The sender's own preview is a separate decision and keeps its own format.
    sender_plugin = (tmp_path / "b" / "plugin.yaml").read_text(encoding="utf-8")
    assert "irt_1_out_transport: raw" in sender_plugin


def test_a_receiver_without_a_decode_gets_no_reverse_transport(tmp_path: Path) -> None:
    """A sender-side preview must not put a decoder on the receiving peer."""
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_heartbeat"] = False
    cfg["topics"] = {
        "b_to_a": [
            {
                "topic": "/camera/image",
                "type": "sensor_msgs/msg/Image",
                "processing": {
                    "transport": {
                        "type": "ffmpeg",
                        "local_republish": "raw",
                        "gop_size": 4,
                        "bit_rate": 500000,
                    }
                },
            }
        ]
    }

    _generate(cfg, tmp_path)

    receiver_plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert "irt_1_topic" not in receiver_plugin
    sender_plugin = (tmp_path / "b" / "plugin.yaml").read_text(encoding="utf-8")
    assert "irt_1_out_transport: raw" in sender_plugin

    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    inbound = next(topic for topic in spec["topics"] if topic["base"] == "/camera/image")
    stages = {stage["stage"] for stage in inbound["stages"]}
    assert "native_in" not in stages


def test_reverse_transport_raw_stage_uses_sensor_image_type_for_compressed_source(tmp_path: Path) -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["use_heartbeat"] = False
    cfg["shared"]["metric_backbone"] = {"record_stages": True}
    cfg["topics"] = {
        "b_to_a": [
            {
                "topic": "/camera/image/compressed",
                "type": "sensor_msgs/msg/CompressedImage",
                "processing": {
                    "restamp_if": True,
                    "drop": {"drop_count": 1, "window_size": 2},
                    "transport": {
                        "type": "ffmpeg",
                        "local_republish": "raw",
                        "remote_republish": "raw",
                        "gop_size": 4,
                        "bit_rate": 500000,
                    },
                },
            }
        ]
    }

    _generate(cfg, tmp_path)

    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    inbound = next(topic for topic in spec["topics"] if topic["base"] == "/camera/image/compressed")
    stages = {stage["stage"]: stage for stage in inbound["stages"]}
    assert stages["native_in"]["topic"] == "/camera/image/compressed/restamped/drop1of2/ffmpeg/raw"
    assert stages["native_in"]["type"] == "sensor_msgs/msg/Image"

    plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert "irt_1_out_transport: raw" in plugin
    assert (
        "metric_stage_topics: /com/in/b/camera/image/compressed/restamped/drop1of2/ffmpeg,"
        "/camera/image/compressed/restamped/drop1of2/ffmpeg,"
        "/camera/image/compressed/restamped/drop1of2/ffmpeg/raw"
    ) in plugin

    sender_plugin = (tmp_path / "b" / "plugin.yaml").read_text(encoding="utf-8")
    assert "/camera/image/compressed/restamped/drop1of2" in sender_plugin
    assert "/camera/image/compressed/restamped/drop1of2/ffmpeg/raw" in sender_plugin


def test_pipeline_spec_carries_heartbeat_expect(tmp_path: Path) -> None:
    import yaml

    cfg = _heartbeat_cfg()
    contract = {"hz": {"min": 8, "max": 12}, "latency_ms": {"max": 200}}
    cfg["shared"]["heartbeat"] = {"expect": contract}
    _generate(cfg, tmp_path)
    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    by_dir = {(t["base"], t["direction"]): t for t in spec["topics"]}
    # The contract applies to both the locally-published and the received heartbeat.
    assert by_dir[("/heartbeat_a", "outbound")]["expect"] == contract
    assert by_dir[("/heartbeat_b", "inbound")]["expect"] == contract


def test_heartbeat_monitor_thresholds_overridden_from_expect(tmp_path: Path) -> None:
    # latency/loss from shared.heartbeat.expect override heartbeat_echo
    # status thresholds (delay_bad_ms / loss3_bad_pct).
    cfg = _heartbeat_cfg()
    cfg["shared"]["heartbeat"] = {"expect": {"latency_ms": {"max": 150}, "loss_pct": {"max": 3}}}
    _generate(cfg, tmp_path)
    plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert "heartbeat_delay_bad_ms: 150.0" in plugin
    assert "heartbeat_loss3_bad_pct: 3.0" in plugin
    assert "heartbeat_local_topic: /heartbeat_a" in plugin
    assert "heartbeat_remote_topic: /heartbeat_b" in plugin


def test_heartbeat_monitor_thresholds_default_without_expect(tmp_path: Path) -> None:
    _generate(_heartbeat_cfg(), tmp_path)
    plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    # No override emitted -> the plugin base template defaults apply.
    assert "heartbeat_delay_bad_ms" not in plugin
    assert "heartbeat_loss3_bad_pct" not in plugin


# ---------------------------------------------------------------------------
# link overhead (compute_link_overview, ROS-independent)
# ---------------------------------------------------------------------------


def _stage(stage: str, state: str, size: float, hz: float) -> dict:
    return {"stage": stage, "state": state, "mean_size_bytes": size, "hz": hz}


def test_compute_link_overview_none_without_sample() -> None:
    assert core.compute_link_overview([], None) is None


def test_compute_link_overview_ratios() -> None:
    # outbound payload at com_out: 1024 B x 10 Hz = 80 kbit/s; wire tx 160 -> ratio 2.0
    # inbound  payload at com_in:  512 B  x 10 Hz = 40 kbit/s; wire rx 40  -> ratio 1.0
    topics = [
        {"direction": "outbound", "stages": [_stage("com_out", "FLOWING", 1024, 10.0)]},
        {"direction": "inbound", "stages": [_stage("com_in", "FLOWING", 512, 10.0)]},
    ]
    link = core.compute_link_overview(topics, {"interface": "tun1", "tx_kbps": 160.0, "rx_kbps": 40.0, "window_s": 2.0})
    assert link["interface"] == "tun1"
    assert link["ros_payload_out_kbps"] == 80.0
    assert link["ros_payload_in_kbps"] == 40.0
    assert link["overhead_ratio_out"] == 2.0
    assert link["overhead_ratio_in"] == 1.0


def test_compute_link_overview_ratio_none_when_no_payload() -> None:
    # No flowing payload in a direction -> ratio is None (not a div-by-zero).
    topics = [{"direction": "outbound", "stages": [_stage("com_out", "IDLE", 1024, 0.0)]}]
    link = core.compute_link_overview(topics, {"tx_kbps": 50.0, "rx_kbps": 0.0})
    assert link["ros_payload_out_kbps"] == 0.0
    assert link["overhead_ratio_out"] is None


# ---------------------------------------------------------------------------
# measurement records reference metric backbone
# ---------------------------------------------------------------------------


def test_sequence_loss_reorder_and_transit_records() -> None:
    obs = core.StageObservation(type_str="com_msgs/msg/OtaStamped")
    transit = {
        "topic": "/wrapped",
        "direction": "inbound",
        "stage": "com_in",
        "t_wrap": 1.01,
        "t_com_in": 1.05,
    }
    obs.record(
        100,
        0.04,
        now_mono=10.0,
        now_wall=1.05,
        seq=10,
        raw_delay_s=0.04,
        clock_offset_s=0.0,
        transit=transit,
    )
    obs.record(
        100,
        0.05,
        now_mono=11.0,
        now_wall=1.10,
        seq=13,
        raw_delay_s=0.05,
        clock_offset_s=0.0,
        transit={**transit, "t_com_in": 1.10},
    )
    obs.record(100, 0.06, now_mono=11.5, now_wall=1.11, seq=12, transit=transit)

    metrics = obs.metrics(12.0, 3.0)
    assert metrics["loss_pct"] == 50.0
    assert metrics["reordered"] == 1
    assert metrics["max_burst_missing"] == 2
    records = obs.drain_transit_records()
    assert [(record["seq"], record["status"]) for record in records] == [
        (10, "delivered"),
        (11, "lost"),
        (12, "lost"),
        (13, "delivered"),
        (12, "reordered"),
    ]
    assert records[1]["t_wrap"] is None
    assert records[3]["sections"]["ota_hop_ms"] == 50.0


def test_clock_offset_estimator_uses_minimum_rtt_sample() -> None:
    estimator = core.ClockOffsetEstimator(window_s=60.0)
    assert estimator.update(t1_s=10.0, t2_s=10.12, t3_s=10.13, t4_s=10.06, now_mono=1.0)
    assert estimator.update(t1_s=20.0, t2_s=20.20, t3_s=20.21, t4_s=20.21, now_mono=2.0)
    estimate = estimator.estimate(now_mono=2.0)
    assert estimate is not None
    assert estimate["rtt_s"] == pytest.approx(0.05)
    assert estimate["offset_s"] == pytest.approx(0.095)


def test_sequence_zero_starts_new_epoch_after_publisher_restart() -> None:
    obs = core.StageObservation()
    obs.record(1, None, now_mono=1.0, seq=40)
    obs.record(1, None, now_mono=2.0, seq=41)
    obs.record(1, None, now_mono=3.0, seq=0)
    obs.record(1, None, now_mono=4.0, seq=1)
    metrics = obs.metrics(4.0, 10.0)
    assert metrics["reordered"] == 0
    assert metrics["loss_pct"] == 0.0
    assert metrics["epoch"] == 1
    assert metrics["epoch_resets"] == 1


def test_restart_is_recognised_when_seq_zero_never_arrives() -> None:
    """The field case: the reader matches after the first tens of messages.

    ``/can/twist`` on 2026-08-13 restarted at seq 28983 and the first arrival of
    the new run carried seq 37. Under the seq==0 rule that was `reordered`, and
    so was every message for the remaining 105 minutes of the instance.
    """
    obs = core.StageObservation()
    obs.record(1, None, now_mono=1.0, seq=28982)
    obs.record(1, None, now_mono=2.0, seq=28983)
    obs.record(1, None, now_mono=110.0, seq=37)
    obs.record(1, None, now_mono=110.1, seq=38)

    metrics = obs.metrics(111.0, 200.0)
    assert metrics["reordered"] == 0
    assert metrics["epoch_resets"] == 1
    assert metrics["loss_pct"] == 0.0


def test_a_small_backward_jump_is_still_reordering() -> None:
    """Restart detection must not swallow the reordering it exists next to."""
    obs = core.StageObservation()
    obs.record(1, None, now_mono=1.0, seq=1000)
    obs.record(1, None, now_mono=2.0, seq=1002)
    obs.record(1, None, now_mono=2.1, seq=1001)

    metrics = obs.metrics(3.0, 10.0)
    assert metrics["reordered"] == 1
    assert metrics["epoch_resets"] == 0


def test_transit_records_carry_the_epoch_they_were_counted_in() -> None:
    obs = core.StageObservation(type_str="com_msgs/msg/OtaStamped")
    transit = {
        "topic": "/wrapped",
        "direction": "inbound",
        "stage": "com_in",
        "t_wrap": 1.01,
        "t_com_in": 1.05,
    }
    obs.record(100, 0.04, now_mono=10.0, now_wall=1.05, seq=5000, transit=transit)
    obs.record(100, 0.04, now_mono=200.0, now_wall=2.05, seq=40, transit=transit)

    records = obs.drain_transit_records()
    assert [(record["seq"], record["epoch"]) for record in records] == [(5000, 0), (40, 1)]


def test_transit_records_carry_the_ffmpeg_keyframe_flag() -> None:
    obs = core.StageObservation(type_str="com_msgs/msg/OtaStamped")
    transit = {
        "topic": "/cam",
        "direction": "inbound",
        "stage": "com_in",
        "t_wrap": 1.01,
        "t_com_in": 1.05,
        "keyframe": True,
    }
    obs.record(40000, 0.04, now_mono=10.0, now_wall=1.05, seq=0, transit=transit)
    obs.record(3000, 0.04, now_mono=10.1, now_wall=1.15, seq=1, transit={**transit, "keyframe": False})
    # seq 2 never arrives: the synthesized lost row has no payload to parse.
    obs.record(3000, 0.04, now_mono=10.3, now_wall=1.35, seq=3, transit={**transit, "keyframe": False})
    # A non-FFMPEG stream's transit dict carries keyframe=None -> field absent.
    obs.record(3000, 0.04, now_mono=10.4, now_wall=1.45, seq=4, transit={**transit, "keyframe": None})

    records = obs.drain_transit_records()
    assert [(record["seq"], record.get("keyframe", "absent")) for record in records] == [
        (0, True),
        (1, False),
        (2, "absent"),
        (3, False),
        (4, "absent"),
    ]


def test_loss_expect_classifies_wrapped_stage_bad(tmp_path: Path) -> None:
    agg = _build({"local": _FakeObserver(), "ota": _FakeObserver()}, tmp_path)
    metrics = {"hz": 10.0, "last_delay_s": 0.01, "loss_pct": 4.0}
    assert agg._classify_quality(metrics, None, {"loss_pct": {"max": 3.0}}) == (
        core.BAD,
        "loss",
    )


def test_metric_stage_bag_is_generated_from_local_pipeline(tmp_path: Path) -> None:
    cfg = _heartbeat_cfg()
    cfg["shared"]["metric_backbone"] = {"record_stages": True}
    _generate(cfg, tmp_path)
    plugin = (tmp_path / "a" / "plugin.yaml").read_text(encoding="utf-8")
    assert "metric_stage_bag: true" in plugin
    assert "metric_stage_topics: /heartbeat_a,/com/out/a/heartbeat_a" in plugin


# ---------------------------------------------------------------------------
# Startup check (#266): a peer that did not bring up its own publishers
# ---------------------------------------------------------------------------


def _startup_aggregator(observers: dict, output_dir: Path, *, started_mono: float) -> core.StatusAggregator:
    return core.StatusAggregator(
        logger=None,
        spec=_outbound_spec(),
        output_dir=str(output_dir / "status"),
        observers_by_domain=observers,
        liveness_window_s=3.0,
        stale_after_s=3.0,
        startup_grace_s=20.0,
        started_mono=started_mono,
    )


def test_no_startup_verdict_before_the_grace_period(tmp_path: Path) -> None:
    """Nothing publishes yet at t+5s, and that is not news."""
    agg = _startup_aggregator({"local": _FakeObserver(), "ota": _FakeObserver()}, tmp_path, started_mono=0.0)

    agg.write(now_mono=5.0)

    assert json.loads((tmp_path / "status" / "status.json").read_text())["startup_check"] is None
    assert not (tmp_path / "status" / "startup_check.json").exists()


def test_a_peer_that_publishes_nothing_fails_its_startup_check(tmp_path: Path) -> None:
    """The 2026-08-13 centre: heartbeat_echo died, so /heartbeat_a never existed.

    Two writes: the verdict is only committed once the same defect has survived
    a second look one grace period later. A node that exited never recovers, so
    this costs the report 20 s and nothing else.
    """
    agg = _startup_aggregator({"local": _FakeObserver(), "ota": _FakeObserver()}, tmp_path, started_mono=0.0)

    agg.write(now_mono=25.0)
    assert not (tmp_path / "status" / "startup_check.json").exists()
    agg.write(now_mono=46.0)

    verdict = json.loads((tmp_path / "status" / "startup_check.json").read_text())
    assert verdict["verdict"] == "incomplete"
    assert verdict["checked_after_s"] == 46.0
    # Only the first break, not the cascade behind it.
    assert [gap["topic"] for gap in verdict["missing_outbound_stages"]] == ["/heartbeat_a"]
    assert verdict["missing_outbound_stages"][0]["produced_by"] == "heartbeat_echo"
    assert "STARTUP CHECK FAILED" in (tmp_path / "status" / "status.txt").read_text()


def test_a_healthy_peer_passes_its_startup_check(tmp_path: Path) -> None:
    local = _FakeObserver()
    local.observations = {
        "/heartbeat_a": _obs(pub=1, last_msg_at=24.9),
        "/com/out/a/heartbeat_a": _obs(pub=1, last_msg_at=24.9),
    }
    ota = _FakeObserver()
    ota.observations = {"/ota/a/heartbeat_a": _obs(pub=1, last_msg_at=24.9, graph_only=True)}
    agg = _startup_aggregator({"local": local, "ota": ota}, tmp_path, started_mono=0.0)

    agg.write(now_mono=25.0)

    verdict = json.loads((tmp_path / "status" / "startup_check.json").read_text())
    assert verdict["verdict"] == "ok"
    assert verdict["missing_outbound_stages"] == []
    assert "STARTUP CHECK FAILED" not in (tmp_path / "status" / "status.txt").read_text()


def test_the_verdict_quotes_the_panes_that_exited(tmp_path: Path) -> None:
    """A dead pane and a missing publisher are the same fact seen twice."""
    local = _FakeObserver()
    local.observations = {
        "/heartbeat_a": _obs(pub=1, last_msg_at=24.9),
        "/com/out/a/heartbeat_a": _obs(pub=1, last_msg_at=24.9),
    }
    ota = _FakeObserver()
    ota.observations = {"/ota/a/heartbeat_a": _obs(pub=1, last_msg_at=24.9, graph_only=True)}
    (tmp_path / "pane_failures.log").write_text(
        "2026-08-13T14:49:50+02:00\tpane=05-BEAT/0\texit=1\tcommand=ros2 run com_py heartbeat_echo\n",
        encoding="utf-8",
    )
    agg = _startup_aggregator({"local": local, "ota": ota}, tmp_path, started_mono=0.0)

    agg.write(now_mono=25.0)

    verdict = json.loads((tmp_path / "status" / "startup_check.json").read_text())
    assert verdict["verdict"] == "incomplete"
    assert verdict["pane_failures"] == [
        "2026-08-13T14:49:50+02:00\tpane=05-BEAT/0\texit=1\tcommand=ros2 run com_py heartbeat_echo"
    ]


def test_the_verdict_is_computed_once(tmp_path: Path) -> None:
    agg = _startup_aggregator({"local": _FakeObserver(), "ota": _FakeObserver()}, tmp_path, started_mono=0.0)

    agg.write(now_mono=25.0)
    agg.write(now_mono=99.0)
    agg.write(now_mono=140.0)

    verdict = json.loads((tmp_path / "status" / "startup_check.json").read_text())
    assert verdict["checked_after_s"] == 99.0


def test_a_stage_that_was_only_late_does_not_fail_the_check(tmp_path: Path) -> None:
    """CI, 2026-08-14: `relay_out` had no publisher for /topic13 at t+20 s.

    It creates one on discovering its input, and the input had just started.
    Seconds later the relay was working. The first version of this check called
    that a peer failure and turned the smoke red -- a check that fires on the
    order of events is a check that gets ignored.
    """
    observers = {"local": _FakeObserver(), "ota": _FakeObserver()}
    agg = _startup_aggregator(observers, tmp_path, started_mono=0.0)

    agg.write(now_mono=25.0)
    assert not (tmp_path / "status" / "startup_check.json").exists()

    observers["local"].observations = {
        "/heartbeat_a": _obs(pub=1, last_msg_at=45.9),
        "/com/out/a/heartbeat_a": _obs(pub=1, last_msg_at=45.9),
    }
    observers["ota"].observations = {"/ota/a/heartbeat_a": _obs(pub=1, last_msg_at=45.9, graph_only=True)}
    agg.write(now_mono=46.0)

    verdict = json.loads((tmp_path / "status" / "startup_check.json").read_text())
    assert verdict["verdict"] == "ok"
    assert verdict["missing_outbound_stages"] == []


def test_inbound_gaps_are_not_this_peers_startup_problem() -> None:
    snapshot = {
        "topics": [
            {
                "base": "/camera",
                "direction": "inbound",
                "stages": [{"stage": "ota_recv", "topic": "/ota/b/camera", "state": core.ABSENT}],
            }
        ]
    }
    assert core.outbound_startup_gaps(snapshot) == []


def _application_spec() -> dict:
    """An outbound topic whose publisher is the user's application, not ours."""
    spec = _outbound_spec()
    topic = spec["topics"][0]
    topic["base"] = "/costmap"
    topic["stages"] = [
        {"stage": "native", "topic": "/costmap", "domain": "local", "produced_by": "application"},
        {"stage": "com_out", "topic": "/com/out/a/costmap", "domain": "local", "produced_by": "relay_out"},
        {"stage": "ota_sent", "topic": "/ota/a/costmap", "domain": "ota", "produced_by": "bridge_out"},
    ]
    return spec


def test_an_application_that_is_not_publishing_is_not_a_peer_failure(tmp_path: Path) -> None:
    """A smoke run exercises the peers without the data source, and that is fine.

    The session declares the topic; whether the AD stack (or a replay) is
    publishing right now is not something this peer promised. Reporting it as a
    peer defect would make the verdict fire on every partial run, which is how a
    check stops being read.
    """
    agg = core.StatusAggregator(
        logger=None,
        spec=_application_spec(),
        output_dir=str(tmp_path / "status"),
        observers_by_domain={"local": _FakeObserver(), "ota": _FakeObserver()},
        liveness_window_s=3.0,
        stale_after_s=3.0,
        startup_grace_s=20.0,
        started_mono=0.0,
    )

    agg.write(now_mono=25.0)

    verdict = json.loads((tmp_path / "status" / "startup_check.json").read_text())
    assert verdict["verdict"] == "ok"
    assert verdict["missing_outbound_stages"] == []
    assert [gap["topic"] for gap in verdict["waiting_for_input"]] == ["/costmap"]
    assert "STARTUP CHECK FAILED" not in (tmp_path / "status" / "status.txt").read_text()


def test_only_the_first_break_of_a_topic_is_reported() -> None:
    """A cascade is one finding: relay_out is running, it just has no input."""
    snapshot = {
        "topics": [
            {
                "base": "/x",
                "direction": "outbound",
                "stages": [
                    {"stage": "native", "topic": "/x", "state": core.FLOWING, "produced_by": "application"},
                    {"stage": "com_out", "topic": "/com/out/a/x", "state": core.ABSENT, "produced_by": "relay_out"},
                    {"stage": "ota_sent", "topic": "/ota/a/x", "state": core.ABSENT, "produced_by": "bridge_out"},
                ],
            }
        ]
    }

    gaps = core.outbound_startup_gaps(snapshot)

    assert [(gap["stage"], gap["owner"]) for gap in gaps] == [("com_out", "rosotacom")]


def test_the_heartbeat_native_stage_is_owned_by_rosotacom(tmp_path: Path) -> None:
    """The generator must mark it, or the startup check blames the application."""
    _generate(_heartbeat_cfg(), tmp_path)

    spec = yaml.safe_load((tmp_path / "a" / "pipeline_spec.yaml").read_text(encoding="utf-8"))
    outbound = next(t for t in spec["topics"] if t["direction"] == "outbound" and t["base"] == "/heartbeat_a")
    native = next(s for s in outbound["stages"] if s["stage"] == "native")
    assert native["produced_by"] == "heartbeat_echo"


def test_a_stage_with_no_input_yet_is_waiting_not_broken() -> None:
    """`5_sized_payload`: the wrapper's publisher appears when its input does.

    `universal_ota_wrapper` creates `…/ota_stamped` on discovering the topic it
    wraps, so before anything publishes `/size_test_a` the absence is the order
    of events, not a defect. Calling it one made the check fire on a smoke that
    starts its payload publisher after the peers — which is how a check stops
    being read.
    """
    snapshot = {
        "topics": [
            {
                "base": "/size_test_a",
                "direction": "outbound",
                "stages": [
                    {"stage": "native", "topic": "/size_test_a", "state": core.IDLE, "produced_by": "application"},
                    {
                        "stage": "processed",
                        "topic": "/size_test_a/ota_stamped",
                        "state": core.ABSENT,
                        "produced_by": "preprocessing",
                    },
                ],
            }
        ]
    }

    (gap,) = core.outbound_startup_gaps(snapshot)

    assert gap["owner"] == "upstream"
    assert "nothing has arrived on /size_test_a" in gap["reason"]


def test_a_stage_whose_input_is_flowing_and_publishes_nothing_is_broken() -> None:
    snapshot = {
        "topics": [
            {
                "base": "/size_test_a",
                "direction": "outbound",
                "stages": [
                    {"stage": "native", "topic": "/size_test_a", "state": core.FLOWING, "produced_by": "application"},
                    {
                        "stage": "processed",
                        "topic": "/size_test_a/ota_stamped",
                        "state": core.ABSENT,
                        "produced_by": "preprocessing",
                    },
                ],
            }
        ]
    }

    (gap,) = core.outbound_startup_gaps(snapshot)

    assert gap["owner"] == "rosotacom"
    assert gap["reason"] == "preprocessing did not come up"


# ---------------------------------------------------------------------------
# a stage the session delays on purpose (#301)
#
# `transport.playout` holds every packet before the decode, so the decoded
# stage's age is the link plus a hold this session asked for. Judging that sum
# against a threshold calibrated for a link reads a healthy paced stream as a
# fault: measured on the cellular link on 2026-08-19, a flat 10 Hz picture at 286 ms
# sat permanently on FLOWING/BAD against the 200 ms default. A red panel that
# means nothing is how an operator learns to stop reading the panel.
#
# The pacer reports what it actually held, and that is the number taken back
# out -- never the configured budget, because a late message passes straight
# through and would otherwise be credited a buffer it never received.
# ---------------------------------------------------------------------------


PACED_STAGE = {
    "stage": "native_in",
    "topic": "/b/camera/ffmpeg/compressed",
    "domain": "local",
    "produced_by": "postprocessing",
    "paced_hold_topic": "/b/camera/ffmpeg/paced/hold_ms",
}


def _paced_aggregator(tmp_path: Path, *, latency_s: float, hold_ms: float | None) -> core.StatusAggregator:
    local = _FakeObserver()
    decoded = core.StageObservation()
    decoded.pub_count, decoded.sub_count = 1, 1
    decoded.record(size=1024, delay_s=latency_s, now_mono=1000.0, now_wall=1_700_000_000.0)
    local.observations[PACED_STAGE["topic"]] = decoded
    if hold_ms is not None:
        hold = core.StageObservation(type_str=core.SCALAR_MSG_TYPE)
        hold.pub_count, hold.sub_count = 1, 1
        hold.last_value = hold_ms
        local.observations[PACED_STAGE["paced_hold_topic"]] = hold
    return _build({"local": local, "ota": _FakeObserver()}, tmp_path)


def test_the_hold_topic_is_observed_even_though_it_is_not_a_stage() -> None:
    spec = {"topics": [{"type": "sensor_msgs/msg/CompressedImage", "stages": [PACED_STAGE]}]}

    by_domain = core.collect_stage_topics(spec)

    assert by_domain["local"][PACED_STAGE["paced_hold_topic"]] == core.SCALAR_MSG_TYPE
    assert PACED_STAGE["paced_hold_topic"] not in by_domain["ota"]


def test_a_paced_stage_is_judged_on_the_link_and_reports_both_numbers(tmp_path: Path) -> None:
    """286 ms on screen, 46 of them the link's: GOOD, and the split is visible."""
    agg = _paced_aggregator(tmp_path, latency_s=0.286, hold_ms=240.0)

    result = agg.classify_stage(PACED_STAGE, None, 1000.5)

    assert result["latency_ms"] == 286.0
    assert result["pacing_hold_ms"] == 240.0
    assert result["link_latency_ms"] == 46.0
    assert result["quality"] == core.GOOD


def test_the_same_age_without_a_pacer_is_still_bad(tmp_path: Path) -> None:
    """The threshold has not moved; only what it is applied to has."""
    stage = {key: value for key, value in PACED_STAGE.items() if key != "paced_hold_topic"}
    local = _FakeObserver()
    decoded = core.StageObservation()
    decoded.pub_count, decoded.sub_count = 1, 1
    decoded.record(size=1024, delay_s=0.286, now_mono=1000.0, now_wall=1_700_000_000.0)
    local.observations[stage["topic"]] = decoded

    result = _build({"local": local, "ota": _FakeObserver()}, tmp_path).classify_stage(stage, None, 1000.5)

    assert result["quality"] == core.BAD
    assert result["pacing_hold_ms"] is None


def test_a_late_message_is_credited_nothing_and_still_flags(tmp_path: Path) -> None:
    """The pacer passes a late packet straight through: hold 0, no excuse."""
    agg = _paced_aggregator(tmp_path, latency_s=0.480, hold_ms=0.0)

    result = agg.classify_stage(PACED_STAGE, None, 1000.5)

    assert result["link_latency_ms"] == 480.0
    assert result["quality"] == core.BAD


def test_before_the_pacer_reports_nothing_is_assumed(tmp_path: Path) -> None:
    """No hold observed yet -- neither zero nor the budget is a safe guess."""
    agg = _paced_aggregator(tmp_path, latency_s=0.286, hold_ms=None)

    result = agg.classify_stage(PACED_STAGE, None, 1000.5)

    assert result["pacing_hold_ms"] is None
    assert result["link_latency_ms"] is None
    assert result["quality"] == core.BAD


def test_the_text_report_shows_what_the_link_cost_and_what_was_added(tmp_path: Path) -> None:
    agg = _paced_aggregator(tmp_path, latency_s=0.286, hold_ms=240.0)
    stage = agg.classify_stage(PACED_STAGE, None, 1000.5)

    line = agg.render_text(
        {
            "peer": "a",
            "remote": "b",
            "generated_at": "2026-08-19T12:00:00",
            "summary": {"OK": 1, "PARTIAL": 0, "STALLED": 0, "ABSENT": 0},
            "phase": 1,
            "topics": [
                {
                    "base": "/camera",
                    "direction": "inbound",
                    "overall": core.OK,
                    "reached_stage": "native_in",
                    "blocked_at": None,
                    "diagnosis": "",
                    "stages": [stage],
                }
            ],
        }
    )

    assert "286ms(link 46+paced 240)" in line
