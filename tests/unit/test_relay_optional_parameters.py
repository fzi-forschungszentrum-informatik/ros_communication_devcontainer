# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Optional relay parameters must read as empty, not as unset.

`source_names` and `target_names` are optional: a peer that names no sources
simply has none. They were declared with a type and no value, which leaves them
PARAMETER_NOT_SET. The node itself never noticed, because it reads them once
through `.value or []`. Every *other* reader did: reading a not-set parameter
raises, and rclpy's parameter service logs one warning per read against the
node, so anything that enumerates parameters on a timer — a Foxglove bridge with
a parameter panel open is the usual one — filled the pane with

    [WARN] [ota_relay_out]: Failed to get parameters: The parameter
    'source_names' is not initialized: source_names

An empty list cannot be a plain default: `declare_parameter(name, [])` infers
the type from the value and an empty sequence satisfies the BYTE_ARRAY test
first, so the parameter ends up typed BYTE_ARRAY and a real string array is
refused later. The fakes below reproduce that rclpy behaviour, which was
confirmed against a live rclpy node before this was written.

rclpy is not available to the host suite, so the ROS surface is stubbed and the
module is loaded by file path (same approach as `tests/unit/test_ota_unwrapper.py`).
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
RELAY_PY = COM_PY / "base_bridge_relay.py"

_STUBBED = (
    "rclpy",
    "rclpy.node",
    "com_py",
    "com_py.topic_resolution",
    "com_py.qos",
    "com_py.pub_sub_pair",
    "com_py.pair_management",
)


class _Type:
    NOT_SET = "not_set"
    STRING_ARRAY = "string_array"
    STRING = "string"


class _Parameter:
    Type = _Type

    def __init__(self, name: str, type_: str = _Type.NOT_SET, value: object = None) -> None:
        self.name = name
        self.type_ = type_
        self.value = value


class _FakeNode:
    """rclpy's declare/get/set semantics, only as far as this method uses them."""

    def __init__(self, overrides: dict[str, list[str]] | None = None) -> None:
        self._overrides = overrides or {}
        self._parameters: dict[str, _Parameter] = {}
        self.set_calls: list[tuple[str, str, object]] = []

    def declare_parameter(self, name: str, type_: str) -> _Parameter:
        if name in self._overrides:
            parameter = _Parameter(name, type_, self._overrides[name])
        else:
            parameter = _Parameter(name, _Type.NOT_SET, None)
        self._parameters[name] = parameter
        return parameter

    def get_parameter_or(self, name: str) -> _Parameter:
        return self._parameters.get(name, _Parameter(name))

    def get_parameter(self, name: str) -> _Parameter:
        parameter = self._parameters[name]
        if parameter.type_ == _Type.NOT_SET:
            raise RuntimeError(f"The parameter '{name}' is not initialized: {name}")
        return parameter

    def set_parameters(self, parameters: list[_Parameter]) -> None:
        for parameter in parameters:
            self.set_calls.append((parameter.name, parameter.type_, parameter.value))
            self._parameters[parameter.name] = parameter


def _stub_module(name: str, **attrs: object) -> ModuleType:
    module = ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


@pytest.fixture(scope="module")
def relay_module() -> Iterator[ModuleType]:
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    _stub_module("rclpy", Parameter=_Parameter)
    # Distinct classes: `BaseBridgeRelay(Node, PairRefreshMixin)` cannot be built
    # from one base twice.
    _stub_module("rclpy.node", Node=type("Node", (), {}))
    _stub_module("com_py")
    _stub_module("com_py.topic_resolution", resolve_topics=lambda *a, **k: [])
    _stub_module("com_py.qos", load_qos_config=lambda *a, **k: {}, load_topic_types=lambda *a, **k: {})
    _stub_module("com_py.pub_sub_pair", PubSubPair=type("PubSubPair", (), {}))
    _stub_module("com_py.pair_management", PairRefreshMixin=type("PairRefreshMixin", (), {}))

    spec = importlib.util.spec_from_file_location("rosotacom_base_bridge_relay", RELAY_PY)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules["rosotacom_base_bridge_relay"] = module
    spec.loader.exec_module(module)
    yield module

    sys.modules.pop("rosotacom_base_bridge_relay", None)
    for name, previous in saved.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


def _declare(module: ModuleType, node: _FakeNode, name: str = "source_names") -> object:
    """The method under test, unbound — the surrounding __init__ needs a live ROS graph."""
    return module.BaseBridgeRelay.declare_optional_string_array(node, name)


def test_an_unsupplied_parameter_becomes_an_empty_string_array(relay_module: ModuleType) -> None:
    node = _FakeNode()

    value = _declare(relay_module, node)

    assert value == []
    assert node.set_calls == [("source_names", _Type.STRING_ARRAY, [])]


def test_the_parameter_is_readable_afterwards(relay_module: ModuleType) -> None:
    """The whole point: a read no longer raises, so nothing logs a warning."""
    node = _FakeNode()

    _declare(relay_module, node)

    assert node.get_parameter("source_names").value == []


def test_supplied_names_are_left_alone(relay_module: ModuleType) -> None:
    node = _FakeNode(overrides={"source_names": ["center", "vehicle"]})

    value = _declare(relay_module, node)

    assert value == ["center", "vehicle"]
    assert node.set_calls == [], "an override must not be overwritten by the default"


def test_it_stays_a_string_array(relay_module: ModuleType) -> None:
    """Not BYTE_ARRAY, which is what a plain `[]` default would have inferred."""
    node = _FakeNode()

    _declare(relay_module, node)

    assert node.get_parameter("source_names").type_ == _Type.STRING_ARRAY


def test_both_optional_parameters_use_it(relay_module: ModuleType) -> None:
    source = RELAY_PY.read_text(encoding="utf-8")

    for name in ("source_names", "target_names"):
        assert f"self.declare_optional_string_array('{name}')" in source
