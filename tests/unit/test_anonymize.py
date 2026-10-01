# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import array
import importlib.util
import json
import sys
from pathlib import Path
from unittest.mock import MagicMock

import yaml

# Mock ROS 2 host-side missing packages for unit tests
sys.modules["rosbag2_py"] = MagicMock()
sys.modules["rclpy"] = MagicMock()
sys.modules["rclpy.serialization"] = MagicMock()
sys.modules["rosidl_runtime_py"] = MagicMock()
sys.modules["rosidl_runtime_py.utilities"] = MagicMock()

import pytest  # noqa: E402

import rosotacom.anonymize as anonymize_lib  # noqa: E402
import rosotacom.cli as cli  # noqa: E402

# Load anonymize_bag.py dynamically to test its recursive anonymizer on mock structures
ws_creation_dir = Path(__file__).parents[2] / "src" / "rosotacom" / "resources" / "ws" / "session" / "creation"
anonymize_bag_path = ws_creation_dir / "anonymize_bag.py"
spec = importlib.util.spec_from_file_location("anonymize_bag", anonymize_bag_path)
assert spec is not None and spec.loader is not None
anonymize_bag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(anonymize_bag)


class Time:
    __slots__ = ["sec", "nanosec"]

    def __init__(self, sec: int, nanosec: int):
        self.sec = sec
        self.nanosec = nanosec


Time.__module__ = "builtin_interfaces.msg._time"


class MockInner:
    __slots__ = ["inner_str", "inner_int"]

    def __init__(self, s: str, i: int):
        self.inner_str = s
        self.inner_int = i


class MockMessage:
    __slots__ = [
        "stamp",
        "payload_str",
        "payload_bytes",
        "payload_array",
        "payload_list",
        "payload_int",
        "payload_bool",
        "nested_msg",
        "nested_list",
    ]

    def __init__(self):
        self.stamp = Time(12345, 67890)
        self.payload_str = "sensitive_data"
        self.payload_bytes = b"binary_image_data"
        self.payload_array = array.array("i", [1, 2, 3])
        self.payload_list = [4, 5, 6]
        self.payload_int = 42
        self.payload_bool = True
        self.nested_msg = MockInner("secret", 7)
        self.nested_list = [MockInner("subsecret", 8)]


class MockFFMPEGPacket:
    """Mimics ffmpeg_image_transport_msgs FFMPEGPacket (ROS 2 slots carry a
    leading underscore); only the class NAME drives the preserve rules."""

    __slots__ = ["_header", "_width", "_height", "_encoding", "_pts", "_flags", "_is_bigendian", "_data"]

    def __init__(self):
        self._header = Time(1, 2)
        self._width = 640
        self._height = 480
        self._encoding = "h264"
        self._pts = 987
        self._flags = 1
        self._is_bigendian = False
        self._data = b"\x01\x02\x03"


MockFFMPEGPacket.__name__ = "FFMPEGPacket"


def test_anonymize_msg_recursive_replacement() -> None:
    msg = MockMessage()
    anonymize_bag.anonymize_msg(msg)

    # Time fields must not be touched
    assert msg.stamp.sec == 12345
    assert msg.stamp.nanosec == 67890

    # Payload values must be zeroed/mocked, but keep identical types and lengths
    assert msg.payload_str == "x" * 14
    assert msg.payload_bytes == b"\x00" * 17
    assert msg.payload_array == array.array("i", [0, 0, 0])
    assert msg.payload_list == [0, 0, 0]
    assert msg.payload_int == 0
    assert msg.payload_bool is False
    assert msg.nested_msg.inner_str == "xxxxxx"
    assert msg.nested_msg.inner_int == 0
    assert msg.nested_list[0].inner_str == "xxxxxxxxx"
    assert msg.nested_list[0].inner_int == 0


def test_anonymize_msg_preserves_ffmpeg_keyframe_structure() -> None:
    """flags (bit 0 = AV_PKT_FLAG_KEY) and pts survive anonymization so GOP
    analysis works on anonymized replay bags; everything else is still zeroed."""
    msg = MockFFMPEGPacket()
    anonymize_bag.anonymize_msg(msg)

    assert msg._flags == 1
    assert msg._pts == 987
    assert msg._data == b"\x00\x00\x00"
    assert msg._encoding == "xxxx"
    assert msg._width == 0
    assert msg._height == 0


class FakeTopicMetadata:
    def __init__(self, id, name, type, serialization_format, offered_qos_profiles):
        self.id = id
        self.name = name
        self.type = type
        self.serialization_format = serialization_format
        self.offered_qos_profiles = offered_qos_profiles


def _fake_reader_cls(topics: list[FakeTopicMetadata], messages: list[tuple[str, bytes, int]]):
    class FakeReader:
        def __init__(self) -> None:
            self._remaining = list(messages)

        def open(self, *args, **kwargs) -> None:
            pass

        def get_all_topics_and_types(self):
            return topics

        def has_next(self) -> bool:
            return bool(self._remaining)

        def read_next(self):
            return self._remaining.pop(0)

    return FakeReader


def test_anonymize_bag_writer_reuses_reader_topic_metadata(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Output topics must be registered from the reader's own typed
    TopicMetadata (incl. its QoS objects) — re-parsing metadata.yaml into dicts
    breaks the rosbag2_py TopicMetadata constructor."""
    bag_dir = tmp_path / "processed_bag"
    bag_dir.mkdir()
    offered_qos = ["<opaque rosbag2 QoS object>"]
    input_metadata = FakeTopicMetadata(1, "/processed", "std_msgs/msg/String", "cdr", offered_qos)

    created_topics = []
    writes = []

    class FakeWriter:
        def open(self, *args, **kwargs) -> None:
            pass

        def create_topic(self, metadata) -> None:
            created_topics.append(metadata)

        def write(self, *args) -> None:
            writes.append(args)

    monkeypatch.setattr(
        anonymize_bag.rosbag2_py,
        "SequentialReader",
        _fake_reader_cls([input_metadata], [("/processed", b"serialized", 123)]),
    )
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "SequentialWriter", FakeWriter)
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "TopicMetadata", FakeTopicMetadata)
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "StorageOptions", lambda **kwargs: kwargs)
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "ConverterOptions", lambda *args: args)
    monkeypatch.setattr(anonymize_bag, "get_message", lambda msg_type: MockMessage)
    monkeypatch.setattr(anonymize_bag, "deserialize_message", lambda data, msg_cls: MockMessage())
    monkeypatch.setattr(anonymize_bag, "serialize_message", lambda msg: b"anonymized")
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "anonymize_bag.py",
            "--input-bag",
            str(bag_dir),
            "--output-bag",
            str(tmp_path / "out_bag"),
            "--topics-map",
            json.dumps({"/processed": "/topic1"}),
        ],
    )

    assert anonymize_bag.main() == 0
    assert created_topics[0].name == "/topic1"
    assert created_topics[0].type == "std_msgs/msg/String"
    assert created_topics[0].serialization_format == "cdr"
    assert created_topics[0].offered_qos_profiles is offered_qos
    assert writes == [("/topic1", b"anonymized", 123)]


def test_anonymize_bag_fails_when_mapped_topic_is_missing(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    bag_dir = tmp_path / "processed_bag"
    bag_dir.mkdir()
    present = FakeTopicMetadata(1, "/present", "std_msgs/msg/String", "cdr", [])
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "SequentialReader", _fake_reader_cls([present], []))
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "StorageOptions", lambda **kwargs: kwargs)
    monkeypatch.setattr(anonymize_bag.rosbag2_py, "ConverterOptions", lambda *args: args)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "anonymize_bag.py",
            "--input-bag",
            str(bag_dir),
            "--output-bag",
            str(tmp_path / "out_bag"),
            "--topics-map",
            json.dumps({"/present": "/topic1", "/missing": "/topic2"}),
        ],
    )

    assert anonymize_bag.main() == 1


def test_anonymize_cli_subcommand(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    # 1. Create a dummy session definition
    session_dir = tmp_path / "sessions" / "test_session"
    session_dir.mkdir(parents=True)
    session_def = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {
            "a": {"domain_id": 46},
            "b": {"domain_id": 47},
        },
        "topics": {"b_to_a": [{"topic": "/sensitive/topic", "type": "std_msgs/msg/String"}]},
    }
    with open(session_dir / "session-definition.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(session_def, f)

    # 2. Create a dummy rosbag metadata.yaml
    bag_dir = tmp_path / "test_bag"
    bag_dir.mkdir()
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "topics_with_message_count": [
                {
                    "message_count": 3,
                    "topic_metadata": {
                        "name": "/sensitive/topic",
                        "type": "std_msgs/msg/String",
                        "serialization_format": "cdr",
                        "offered_qos_profiles": [
                            {"reliability": "reliable", "durability": "transient_local", "depth": 1}
                        ],
                    },
                }
            ],
        }
    }
    with open(bag_dir / "metadata.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(metadata, f)

    # 3. Mock runtime environment config and resolution functions
    from rosotacom.cli import ResolvedSession, RuntimeConfig

    runtime = RuntimeConfig(
        rosotacom_config=tmp_path / "rosotacom.yaml",
        ros2docker_config=tmp_path / "ros2docker.json",
        session_configs_dir=(tmp_path / "sessions",),
        deployment=None,
        install_id="test_install",
        session_instances_dir=tmp_path / "session-instances",
        scenario_configs_dir=(),
        profiles_file=None,
        benchmarks_dir=tmp_path / "benchmarks",
    )

    # Write a dummy ros2docker.json
    with open(tmp_path / "ros2docker.json", "w", encoding="utf-8") as f:
        json.dump({"image_name": "ros2docker-test-image"}, f)

    # Mock resolved session
    monkeypatch.setattr(cli, "_load_runtime_config", lambda *args: runtime)
    monkeypatch.setattr(
        cli,
        "_resolve_session",
        lambda name, rt: ResolvedSession(session_dir, str(session_dir), "session_configs"),
    )

    # Mock docker running api
    container_runs = []
    image_builds = []

    def mock_ros2docker_run(config_file, override, extra_run_args):
        container_runs.append((override, extra_run_args))

    monkeypatch.setattr(cli, "ros2docker_run", mock_ros2docker_run, raising=False)
    monkeypatch.setattr(
        cli,
        "_build_image",
        lambda config_file, override: image_builds.append((config_file, override)),
    )
    monkeypatch.setattr(cli, "_stop_container_name", lambda *args, **kwargs: None)
    monkeypatch.setattr(cli, "_require_ros2docker", lambda: None)

    # 4. Run anonymize subcommand
    output_dir = tmp_path / "anonymized_output"
    argv = ["anonymize", str(bag_dir), "-s", "test_session", "-o", str(output_dir), "--output-name", "anon_session"]

    exit_code = cli.main(argv)
    assert exit_code == 0

    # 5. Verify the files generated in output_dir
    assert (output_dir / "ros2docker.json").exists()
    assert (output_dir / "rosotacom.yaml").exists()

    # Verify anonymized session definition
    anon_session_def_path = output_dir / "sessions" / "anon_session" / "session-definition.yaml"
    assert anon_session_def_path.exists()
    with open(anon_session_def_path, encoding="utf-8") as f:
        anon_session = yaml.safe_load(f)
    assert anon_session["topics"]["b_to_a"][0]["topic"] == "/topic1"
    assert "processing" not in anon_session["topics"]["b_to_a"][0]

    # Verify anonymized scenario definition
    anon_scenario_def_path = output_dir / "scenarios" / "anon_session" / "scenario-definition.yaml"
    assert anon_scenario_def_path.exists()
    with open(anon_scenario_def_path, encoding="utf-8") as f:
        anon_scenario = yaml.safe_load(f)
    assert anon_scenario["session"] == "anon_session"
    assert "b" in anon_scenario["applications"]

    # Verify play_bag_b.ros2docker.json config
    play_cfg_path = output_dir / "scenarios" / "anon_session" / "play_bag_b.ros2docker.json"
    assert play_cfg_path.exists()
    with open(play_cfg_path, encoding="utf-8") as f:
        play_cfg = json.load(f)
    assert play_cfg["command"] == (
        "ros2 bag play --loop /bag/anonymized_bag "
        "--qos-profile-overrides-path /scenario/qos-overrides.yaml --topics /topic1"
    )
    assert "ROS_DOMAIN_ID=47" in play_cfg["run_args"]
    assert "./qos-overrides.yaml:/scenario/qos-overrides.yaml:ro" in play_cfg["run_args"]

    qos_path = output_dir / "scenarios" / "anon_session" / "qos-overrides.yaml"
    with open(qos_path, encoding="utf-8") as f:
        qos = yaml.safe_load(f)
    assert qos["/topic1"]["durability"] == "transient_local"
    assert qos["/topic1"]["reliability"] == "reliable"

    manifest_path = output_dir / "scenarios" / "anon_session" / "anonymization-manifest.yaml"
    with open(manifest_path, encoding="utf-8") as f:
        manifest = yaml.safe_load(f)
    assert manifest["mode"] == "processed_handoff_replay"
    assert manifest["topics"][0]["handoff_topic"] == "/sensitive/topic"

    # Verify container run arguments
    assert len(container_runs) == 1
    override, extra_args = container_runs[0]
    assert override["container_name"] == "rosotacom_anonymizer"
    assert override["image_name"] == "ros2docker-test-image-test_install"
    assert image_builds == [
        (
            runtime.ros2docker_config,
            {
                "container_name": "rosotacom_anonymizer",
                "image_name": "ros2docker-test-image-test_install",
            },
        )
    ]
    assert play_cfg["image_name"] == "ros2docker-test-image"
    # Batch job must stay headless-safe: a TTY aborts without a terminal.
    assert override["tty"] is False
    assert override["stdin_open"] is False
    assert "anonymize_bag.py" in override["command"]
    assert f"/input_parent_dir/{bag_dir.name}" in override["command"]

    # Ensure mapping of topics is passed to the container command
    assert '"/sensitive/topic": "/topic1"' in override["command"]


def test_handoff_planner_selects_processed_compressed_topic() -> None:
    session_cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 46}, "b": {"domain_id": 47}},
        "topics": {
            "b_to_a": [
                {
                    "topic": "/costmap/costmap",
                    "type": "nav_msgs/msg/OccupancyGrid",
                    "processing": {"compress": True},
                    "qos": {"reliability": "reliable"},
                    "expect": {"hz": {"min": 1, "max": 5}, "latency_ms": {"max": 500}},
                }
            ]
        },
    }

    plan = anonymize_lib.plan_handoff_topics(session_cfg, cli.session_gen)

    assert len(plan) == 1
    item = plan[0]
    assert item.source_peer == "b"
    assert item.direction == "b_to_a"
    assert item.source_topic == "/costmap/costmap"
    assert item.handoff_topic == "/costmap/costmap/bz2"
    assert item.handoff_type == "com_msgs/msg/CompressedData"
    assert item.generic_topic == "/topic1"

    replay_cfg = anonymize_lib.build_replay_session_config(session_cfg, plan)
    replay_entry = replay_cfg["topics"]["b_to_a"][0]
    assert replay_entry["topic"] == "/topic1"
    assert replay_entry["type"] == "com_msgs/msg/CompressedData"
    assert "processing" not in replay_entry
    assert replay_entry["qos"] == {"reliability": "reliable"}
    assert replay_entry["expect"] == {"hz": {"min": 1, "max": 5}}


def test_playback_topics_remap_target_prefixed_replay_sources() -> None:
    session_cfg = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {
            "a": {"domain_id": 46, "outbound": {"target_prefix": {"use_target_prefix": True}}},
            "b": {"domain_id": 47},
        },
        "topics": {
            "a_to_b": [
                {
                    "topic": "/move_base_free/goal",
                    "type": "geometry_msgs/msg/PoseStamped",
                    "processing": {"framebridge": "global_to_local"},
                }
            ],
            "b_to_a": [{"topic": "/status", "type": "std_msgs/msg/String"}],
        },
    }

    plan = anonymize_lib.plan_handoff_topics(session_cfg, cli.session_gen)
    replay_cfg = anonymize_lib.build_replay_session_config(session_cfg, plan)
    topics_by_peer = anonymize_lib.playback_topics_by_peer(replay_cfg, plan, cli.session_gen)

    assert [(topic.bag_topic, topic.publish_topic) for topic in topics_by_peer["a"]] == [("/topic1", "/to_b/topic1")]
    assert [(topic.bag_topic, topic.publish_topic) for topic in topics_by_peer["b"]] == [("/topic2", "/topic2")]


def test_handoff_planner_missing_processed_topic_fails_closed(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    session_dir = tmp_path / "sessions" / "compressed"
    session_dir.mkdir(parents=True)
    session_def = {
        "peers": {"a": {}, "b": {}},
        "peer_settings": {"a": {"domain_id": 46}, "b": {"domain_id": 47}},
        "topics": {
            "b_to_a": [
                {
                    "topic": "/costmap/costmap",
                    "type": "nav_msgs/msg/OccupancyGrid",
                    "processing": {"compress": True},
                }
            ]
        },
    }
    with open(session_dir / "session-definition.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(session_def, f)

    bag_dir = tmp_path / "raw_bag"
    bag_dir.mkdir()
    metadata = {
        "rosbag2_bagfile_information": {
            "storage_identifier": "mcap",
            "topics_with_message_count": [
                {
                    "message_count": 1,
                    "topic_metadata": {
                        "name": "/costmap/costmap",
                        "type": "nav_msgs/msg/OccupancyGrid",
                        "serialization_format": "cdr",
                    },
                }
            ],
        }
    }
    with open(bag_dir / "metadata.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(metadata, f)

    from rosotacom.cli import ResolvedSession, RuntimeConfig

    runtime = RuntimeConfig(
        rosotacom_config=tmp_path / "rosotacom.yaml",
        ros2docker_config=tmp_path / "ros2docker.json",
        session_configs_dir=(tmp_path / "sessions",),
        deployment=None,
        install_id="test_install",
        session_instances_dir=tmp_path / "session-instances",
        scenario_configs_dir=(),
        profiles_file=None,
        benchmarks_dir=tmp_path / "benchmarks",
    )
    with open(tmp_path / "ros2docker.json", "w", encoding="utf-8") as f:
        json.dump({"image_name": "ros2docker-test-image"}, f)

    monkeypatch.setattr(cli, "_load_runtime_config", lambda *args: runtime)
    monkeypatch.setattr(
        cli,
        "_resolve_session",
        lambda name, rt: ResolvedSession(session_dir, str(session_dir), "session_configs"),
    )

    exit_code = cli.main(["anonymize", str(bag_dir), "-s", "compressed", "-o", str(tmp_path / "out")])

    assert exit_code == 1
    assert not (tmp_path / "out").exists()
