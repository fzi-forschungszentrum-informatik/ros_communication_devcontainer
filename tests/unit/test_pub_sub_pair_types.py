# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Where a bridge or relay pair learns what a stage carries.

Reading the type from the ROS graph means an endpoint can only be created after
somebody else has created the matching one. Over a transport that carries data
but not the graph — the zenoh DDS bridge — that is a deadlock: the bridge routes
a topic once a local reader exists, and the reader waited for the bridge to
route a publisher into the graph. Measured on the bench pair (2026-08-18): the
receiving peer logged `Initialized 1 pair(s). Pending=1` and the payload topic
never arrived, while the heartbeat — whose publisher the graph did know — flowed.

The node imports rclpy, which the host suite does not have, so its ROS surface
is stubbed and the module is loaded by file path (same approach as
`tests/unit/test_ota_unwrapper.py`).
"""

from __future__ import annotations

import importlib.util
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
COM_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py"
PAIR_PY = COM_PY / "pub_sub_pair.py"

_STUBBED = ("rclpy", "rclpy.node", "rosidl_runtime_py", "rosidl_runtime_py.utilities", "com_py", "com_py.qos")


class _Logger:
    def __init__(self) -> None:
        self.messages: list[str] = []

    def info(self, message: str) -> None:
        self.messages.append(message)

    def error(self, message: str) -> None:
        self.messages.append(message)

    def warning(self, message: str) -> None:
        self.messages.append(message)


class _Node:
    """Only what PubSubPair uses: a logger, a graph, and endpoint factories."""

    def __init__(self, graph: dict[str, list[str]]) -> None:
        self._graph = graph
        self._logger = _Logger()
        self.created: list[tuple[str, str, object]] = []

    def get_logger(self):
        return self._logger

    def get_topic_names_and_types(self):
        return list(self._graph.items())

    def create_publisher(self, msg_type, topic, qos):
        self.created.append(("pub", topic, msg_type))
        return object()

    def create_subscription(self, msg_type, topic, callback, qos):
        self.created.append(("sub", topic, msg_type))
        return object()


def _stub_module(name: str, **attributes: object) -> ModuleType:
    module = ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


@pytest.fixture()
def pair_module() -> Iterator[ModuleType]:
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    _stub_module("rclpy")
    _stub_module("rclpy.node", Node=_Node)
    _stub_module("rosidl_runtime_py")
    _stub_module("rosidl_runtime_py.utilities", get_message=lambda name: f"<type {name}>")
    _stub_module("com_py")
    _stub_module("com_py.qos", get_topic_qos=lambda *a, **k: object())

    spec = importlib.util.spec_from_file_location("rosotacom_pub_sub_pair", PAIR_PY)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules["rosotacom_pub_sub_pair"] = module
    spec.loader.exec_module(module)
    try:
        yield module
    finally:
        sys.modules.pop("rosotacom_pub_sub_pair", None)
        for name, original in saved.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


def _pair(module: ModuleType, node: _Node, topic_types: dict[str, str] | None):
    return module.PubSubPair(
        node=node,
        base_topic_name="/cam",
        sub_topic="/ota/b/cam/ota_stamped",
        pub_topic="/com/in/b/cam/ota_stamped",
        sub_role="ota_sub",
        pub_role="forward_pub",
        qos_config={},
        topic_types=topic_types,
    )


def test_a_declared_type_creates_the_endpoints_with_nothing_in_the_graph(pair_module: ModuleType) -> None:
    node = _Node(graph={})

    pair = _pair(pair_module, node, {"/ota/b/cam/ota_stamped": "com_msgs/msg/OtaStamped"})

    assert pair.is_valid
    assert node.created == [
        ("pub", "/com/in/b/cam/ota_stamped", "<type com_msgs/msg/OtaStamped>"),
        ("sub", "/ota/b/cam/ota_stamped", "<type com_msgs/msg/OtaStamped>"),
    ]


def test_without_a_declaration_the_graph_still_answers(pair_module: ModuleType) -> None:
    node = _Node(graph={"/ota/b/cam/ota_stamped": ["com_msgs/msg/OtaStamped"]})

    pair = _pair(pair_module, node, None)

    assert pair.is_valid
    assert [entry[0] for entry in node.created] == ["pub", "sub"]


def test_with_neither_the_pair_stays_pending(pair_module: ModuleType) -> None:
    node = _Node(graph={})

    pair = _pair(pair_module, node, {"/some/other/topic": "std_msgs/msg/Bool"})

    assert not pair.is_valid
    assert node.created == []
