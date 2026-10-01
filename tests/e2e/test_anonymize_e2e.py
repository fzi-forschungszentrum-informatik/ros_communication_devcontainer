# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Headless end-to-end check of `rosotacom anonymize`.

Guards the three ways this flow has actually broken on a real trace:
a TTY-enabled anonymizer container aborting without a terminal, writer topic
registration failing against current rosbag2_py, and FFMPEGPacket keyframe
flags being zeroed away. A trace bag is written inside the project container,
anonymized through the real CLI (headless subprocess), and re-read inside the
container; the anonymized bag must keep sizes, timestamps, `flags`, and `pts`
while zeroing payloads and strings.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
pytestmark = [
    pytest.mark.e2e,
    pytest.mark.skipif(
        os.environ.get("ROSOTACOM_RUN_E2E") != "1",
        reason="Docker-backed E2E tests require ROSOTACOM_RUN_E2E=1.",
    ),
]

TRACE_TOPIC = "/topic9"  # the camera stream's handoff name in packaged example 16
FLAG_PATTERN = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]
PAYLOAD_SIZES = [40000, 3000, 3500, 3200, 3100, 42000, 2900, 3300, 3600, 3400, 41000, 3000]

MAKE_TRACE_SCRIPT = f"""
import rosbag2_py
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

FFMPEGPacket = get_message("ffmpeg_image_transport_msgs/msg/FFMPEGPacket")
writer = rosbag2_py.SequentialWriter()
writer.open(
    rosbag2_py.StorageOptions(uri="/work/trace", storage_id="mcap"),
    rosbag2_py.ConverterOptions("", ""),
)
writer.create_topic(
    rosbag2_py.TopicMetadata(
        id=0,
        name={TRACE_TOPIC!r},
        type="ffmpeg_image_transport_msgs/msg/FFMPEGPacket",
        serialization_format="cdr",
    )
)
for i, (flags, size) in enumerate(zip({FLAG_PATTERN!r}, {PAYLOAD_SIZES!r})):
    msg = FFMPEGPacket()
    msg.header.stamp.sec = 100 + i
    msg.header.frame_id = "front_medium"
    msg.width = 640
    msg.height = 480
    msg.encoding = "h264"
    msg.pts = i
    msg.flags = flags
    msg.data = bytes((j % 251) + 1 for j in range(size))  # nonzero payload
    writer.write({TRACE_TOPIC!r}, serialize_message(msg), (100 + i) * 100_000_000)
del writer
print("TRACE_WRITTEN")
"""

READ_BAG_SCRIPT = """
import json
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

FFMPEGPacket = get_message("ffmpeg_image_transport_msgs/msg/FFMPEGPacket")
reader = rosbag2_py.SequentialReader()
bag_uri = "/work/anon/scenarios/anonymized_16_remote_assist_anonymized_camera/anonymized_bag"
reader.open(
    rosbag2_py.StorageOptions(uri=bag_uri, storage_id="mcap"),
    rosbag2_py.ConverterOptions("", ""),
)
rows = []
while reader.has_next():
    topic, data, stamp = reader.read_next()
    msg = deserialize_message(data, FFMPEGPacket)
    rows.append({
        "topic": topic,
        "stamp": stamp,
        "flags": msg.flags,
        "pts": msg.pts,
        "size": len(msg.data),
        "payload_zeroed": not any(msg.data),
        "frame_id": msg.header.frame_id,
        "encoding": msg.encoding,
    })
with open("/work/result.json", "w") as f:
    json.dump(rows, f)
print("BAG_READ")
"""


@pytest.fixture(scope="session")
def copied_example_project(tmp_path_factory: pytest.TempPathFactory) -> Path:
    project = tmp_path_factory.mktemp("rosotacom") / "examples"
    subprocess.run(
        [sys.executable, "-m", "rosotacom", "examples", "create", str(project)],
        cwd=PACKAGE_ROOT,
        text=True,
        capture_output=True,
        timeout=60,
        check=True,
    )
    return project


def _run_in_container(project: Path, work_dir: Path, name: str, script: str) -> None:
    """Run a python script inside the project image; raises on non-zero exit."""
    from ros2docker.api import run as ros2docker_run
    from ros2docker.config import load_config

    script_path = work_dir / f"{name}.py"
    script_path.write_text(script, encoding="utf-8")
    image_name = load_config(project / "ros2docker.json").get("image_name", "ros2docker")
    ros2docker_run(
        config_file=project / "ros2docker.json",
        override={
            "container_name": f"rosotacom_anonymize_e2e_{name}",
            "image_name": image_name,
            "run_type": "command",
            "command": f"python3 /work/{name}.py",
            "tty": False,
            "stdin_open": False,
        },
        extra_run_args=["-v", f"{work_dir}:/work"],
    )


def test_anonymize_headless_end_to_end_preserves_keyframe_structure(
    copied_example_project: Path,
    tmp_path: Path,
) -> None:
    from ros2docker.api import build as ros2docker_build

    work_dir = tmp_path
    work_dir.chmod(0o777)  # container user must write the bag and result.json

    ros2docker_build(config_file=copied_example_project / "ros2docker.json")
    _run_in_container(copied_example_project, work_dir, "make_trace", MAKE_TRACE_SCRIPT)

    # The real CLI, headless: no TTY on stdin — this is exactly the environment
    # in which a tty-enabled anonymizer container refuses to start.
    anonymize = subprocess.run(
        [
            sys.executable,
            "-m",
            "rosotacom",
            "anonymize",
            str(work_dir / "trace"),
            "-s",
            "16_remote_assist_anonymized_camera",
            "--rosotacom-config",
            str(copied_example_project / "rosotacom.yaml"),
            "-o",
            str(work_dir / "anon"),
        ],
        cwd=PACKAGE_ROOT,
        text=True,
        capture_output=True,
        timeout=600,
        stdin=subprocess.DEVNULL,
    )
    assert anonymize.returncode == 0, f"anonymize failed:\nSTDOUT:\n{anonymize.stdout}\nSTDERR:\n{anonymize.stderr}"

    out_name = "anonymized_16_remote_assist_anonymized_camera"
    session_def = yaml.safe_load(
        (work_dir / "anon" / "sessions" / out_name / "session-definition.yaml").read_text(encoding="utf-8")
    )
    assert session_def["topics"]["b_to_a"][0]["topic"] == "/topic1"
    assert session_def["topics"]["b_to_a"][0]["type"] == "ffmpeg_image_transport_msgs/msg/FFMPEGPacket"
    scenario_dir = work_dir / "anon" / "scenarios" / out_name
    assert (scenario_dir / "anonymized_bag" / "metadata.yaml").is_file()
    assert (scenario_dir / "scenario-definition.yaml").is_file()
    play_cfg = json.loads((scenario_dir / "play_bag_b.ros2docker.json").read_text(encoding="utf-8"))
    assert "ros2 bag play --loop /bag/anonymized_bag" in play_cfg["command"]

    _run_in_container(copied_example_project, work_dir, "read_bag", READ_BAG_SCRIPT)
    rows = json.loads((work_dir / "result.json").read_text(encoding="utf-8"))
    rows.sort(key=lambda row: row["stamp"])

    assert [row["topic"] for row in rows] == ["/topic1"] * len(FLAG_PATTERN)
    # Keyframe structure survives anonymization ...
    assert [row["flags"] for row in rows] == FLAG_PATTERN
    assert [row["pts"] for row in rows] == list(range(len(FLAG_PATTERN)))
    assert [row["size"] for row in rows] == PAYLOAD_SIZES
    assert [row["stamp"] for row in rows] == [(100 + i) * 100_000_000 for i in range(len(FLAG_PATTERN))]
    # ... while content does not.
    assert all(row["payload_zeroed"] for row in rows)
    assert all(row["frame_id"] == "x" * len("front_medium") for row in rows)
    assert all(row["encoding"] == "xxxx" for row in rows)
