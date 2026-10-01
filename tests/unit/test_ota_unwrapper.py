# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""What the unwrapper calls a break in the sequence.

A backward jump is either the transport reordering two messages or the sending
peer restarting, and they mean opposite things: one is a property of the link,
the other resets the numbering everything downstream keys on. Three consumers
already distinguish them — the live status accounting, the transit join and the
offline bag join — and this pane log was the last place still calling both
"reordering".

The node imports rclpy, which the host suite does not have, so its ROS surface
is stubbed before the module is loaded by file path (same approach as
`tests/unit/test_ota_wrapper.py`).
"""

from __future__ import annotations

import importlib.util
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
COM_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py"
UNWRAPPER_PY = COM_PY / "universal_ota_unwrapper.py"
CORE_PY = COM_PY / "status_overview_core.py"

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
    "com_py.status_overview_core",
)


class _FakeNode:
    pass


def _stub_module(name: str, **attrs: object) -> ModuleType:
    module = ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


def _load(path: Path, name: str) -> ModuleType:
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def unwrapper_module() -> Iterator[ModuleType]:
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    _stub_module("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None, spin=lambda *a, **k: None)
    _stub_module("rclpy.node", Node=_FakeNode)
    _stub_module("rclpy.serialization", deserialize_message=lambda *a, **k: None)
    _stub_module("rosidl_runtime_py", utilities=None)
    _stub_module("rosidl_runtime_py.utilities", get_message=lambda type_str: None)
    _stub_module("com_msgs", msg=None)
    _stub_module("com_msgs.msg", OtaStamped=object)
    _stub_module("com_py", qos=None)
    _stub_module("com_py.qos", get_topic_qos=lambda *a, **k: None, load_qos_config=lambda *a, **k: {})
    # The real rule, not a stub: the point of this test is that the unwrapper
    # and the accounting agree about what a restart is.
    core = _load(CORE_PY, "rosotacom_status_overview_core_unwrapper")
    _stub_module("com_py.status_overview_core", is_sequence_epoch_reset=core.is_sequence_epoch_reset)

    module = _load(UNWRAPPER_PY, "rosotacom_universal_ota_unwrapper")
    yield module

    sys.modules.pop("rosotacom_universal_ota_unwrapper", None)
    sys.modules.pop("rosotacom_status_overview_core_unwrapper", None)
    for name, previous in saved.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


def _node(unwrapper_module: ModuleType, warnings: list[str]):
    node = object.__new__(unwrapper_module.UniversalOtaUnwrapperNode)
    node._last_seq = {}
    node.get_logger = lambda: SimpleNamespace(
        warning=warnings.append, error=lambda *a, **k: None, info=lambda *a, **k: None
    )
    return node


def _feed(unwrapper_module: ModuleType, sequence: list[int]) -> list[str]:
    warnings: list[str] = []
    node = _node(unwrapper_module, warnings)
    for value in sequence:
        node._check_sequence("/x/ota_stamped", value)
    return warnings


def test_an_uninterrupted_stream_says_nothing(unwrapper_module: ModuleType) -> None:
    assert _feed(unwrapper_module, list(range(20))) == []


def test_a_restart_is_called_a_restart(unwrapper_module: ModuleType) -> None:
    """The field shape: seq 28983, then the new run's first arrival at 37."""
    (warning,) = _feed(unwrapper_module, [28982, 28983, 37, 38])

    assert "Sending peer restarted" in warning
    assert "37" in warning and "28983" in warning


def test_a_reset_to_zero_is_a_restart_too(unwrapper_module: ModuleType) -> None:
    (warning,) = _feed(unwrapper_module, [40, 41, 0, 1])
    assert "Sending peer restarted" in warning


def test_reordering_is_still_reordering(unwrapper_module: ModuleType) -> None:
    # 1002 after 1000 is a forward gap (one warning); 1001 after 1002 is the
    # reordering (the second). Both are real and both keep their old words.
    gap, reorder = _feed(unwrapper_module, [1000, 1002, 1001])

    assert "Sequence jump" in gap
    assert "reordering" in reorder.lower()
    assert "restarted" not in reorder


def test_a_forward_gap_is_a_jump(unwrapper_module: ModuleType) -> None:
    (warning,) = _feed(unwrapper_module, [10, 15])
    assert "Sequence jump" in warning
