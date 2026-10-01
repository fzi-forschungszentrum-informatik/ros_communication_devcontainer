# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Unit tests for the OTA wrapper's envelope construction.

The wrapper is the only stage that can measure per-message loss and true OTA
transit time, so what it costs per message is part of what it measures. These
tests drive `wrapper_callback` directly: the node module imports rclpy, which
the host suite does not have, so the ROS surface is stubbed before the module is
loaded by file path (same approach as `tests/unit/test_status_overview.py`).
"""

from __future__ import annotations

import array
import importlib.util
import sys
import threading
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
WRAPPER_PY = (
    REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py" / "universal_ota_wrapper.py"
)


#: Modules the wrapper imports at load time and the host suite does not have.
_STUBBED = (
    "rclpy",
    "rclpy.node",
    "rclpy.serialization",
    "rosidl_runtime_py",
    "rosidl_runtime_py.utilities",
    "com_msgs",
    "com_msgs.msg",
    "com_py",
    "com_py.qos",
)


class _FakeOtaStamped:
    """Stand-in for com_msgs/msg/OtaStamped that records what was assigned."""

    def __init__(self) -> None:
        self.header = SimpleNamespace(stamp=None, frame_id="")
        self.seq = 0
        self.msg_type = ""
        self.serialized_msg: object = None


class _FakeNode:
    pass


def _stub_module(name: str, **attrs: object) -> ModuleType:
    module = ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


@pytest.fixture(scope="module")
def wrapper_module() -> Iterator[ModuleType]:
    """Load the wrapper node with its ROS imports stubbed out."""
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    _stub_module("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None, spin=lambda *a, **k: None)
    _stub_module("rclpy.node", Node=_FakeNode)
    # The source message carries its own payload so a test can choose the bytes.
    _stub_module("rclpy.serialization", serialize_message=lambda msg: msg.payload)
    _stub_module("rosidl_runtime_py", utilities=None)
    _stub_module("rosidl_runtime_py.utilities", get_message=lambda type_str: None)
    _stub_module("com_msgs", msg=None)
    _stub_module("com_msgs.msg", OtaStamped=_FakeOtaStamped)
    _stub_module("com_py", qos=None)
    _stub_module("com_py.qos", get_topic_qos=lambda *a, **k: None, load_qos_config=lambda *a, **k: {})

    spec = importlib.util.spec_from_file_location("rosotacom_universal_ota_wrapper", WRAPPER_PY)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    yield module

    sys.modules.pop(spec.name, None)
    for name, previous in saved.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


class _RecordingPublisher:
    def __init__(self) -> None:
        self.published: list[_FakeOtaStamped] = []

    def publish(self, msg: _FakeOtaStamped) -> None:
        self.published.append(msg)


def _node(wrapper_module: ModuleType):
    """A wrapper node with only the state `wrapper_callback` touches."""
    node = object.__new__(wrapper_module.UniversalOtaWrapperNode)
    node.sequence_lock = threading.Lock()
    node.sequence_by_topic = {}
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "stamp"))
    node.get_logger = lambda: SimpleNamespace(error=lambda *a, **k: None, info=lambda *a, **k: None)
    return node


def _source(payload: bytes, frame_id: str | None = "camera"):
    if frame_id is None:
        return SimpleNamespace(payload=payload)
    return SimpleNamespace(payload=payload, header=SimpleNamespace(frame_id=frame_id, stamp=None))


def test_payload_is_a_buffer_not_a_per_byte_list(wrapper_module: ModuleType) -> None:
    """#257 regression: `list(serialized)` allocates one Python int per byte.

    That copy sits inside the interval `header.stamp` is taken to measure, and it
    is what rules out wrapping an encoded video packet.
    """
    node = _node(wrapper_module)
    publisher = _RecordingPublisher()
    payload = bytes(range(256)) * 8

    node.wrapper_callback(_source(payload), publisher, "/cam", "sensor_msgs/msg/CompressedImage")

    (sent,) = publisher.published
    assert not isinstance(sent.serialized_msg, list)
    assert isinstance(sent.serialized_msg, array.array)
    assert sent.serialized_msg.typecode == "B"
    assert bytes(sent.serialized_msg) == payload


def test_envelope_carries_type_frame_and_stamp(wrapper_module: ModuleType) -> None:
    node = _node(wrapper_module)
    publisher = _RecordingPublisher()

    node.wrapper_callback(_source(b"\x01\x02"), publisher, "/cam", "sensor_msgs/msg/CompressedImage")

    (sent,) = publisher.published
    assert sent.msg_type == "sensor_msgs/msg/CompressedImage"
    assert sent.header.frame_id == "camera"
    assert sent.header.stamp == "stamp"


def test_headerless_messages_are_wrapped_too(wrapper_module: ModuleType) -> None:
    node = _node(wrapper_module)
    publisher = _RecordingPublisher()

    node.wrapper_callback(_source(b"\x07", frame_id=None), publisher, "/tf", "tf2_msgs/msg/TFMessage")

    (sent,) = publisher.published
    assert bytes(sent.serialized_msg) == b"\x07"
    assert sent.header.frame_id == ""


def test_sequence_counts_per_source_topic(wrapper_module: ModuleType) -> None:
    """`loss_pct` is gap counting on this number, so it must be per topic."""
    node = _node(wrapper_module)
    publisher = _RecordingPublisher()

    for _ in range(3):
        node.wrapper_callback(_source(b"a"), publisher, "/twist", "geometry_msgs/msg/TwistStamped")
    node.wrapper_callback(_source(b"b"), publisher, "/progress", "std_msgs/msg/Float64")

    assert [m.seq for m in publisher.published] == [0, 1, 2, 0]
