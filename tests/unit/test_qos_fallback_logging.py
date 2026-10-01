# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""At which level `get_topic_qos` reports a base-topic fallback.

A session declares QoS on the base topic (`/tf_static`), and every processing
stage appends a suffix to the name that is actually published
(`/tf_static/globalframe`). Resolving the suffixed name through its base is
therefore the designed path, not a degradation — and it was logged as a warning,
twice per topic per peer, at every bring-up. That is the noise a real message has
to compete with while a peer is starting, which is exactly when somebody is
reading the pane.

The module imports rclpy, which the host suite does not have, so its ROS surface
is stubbed before the module is loaded by file path (same approach as
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
QOS_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py" / "qos.py"

_STUBBED = ("rclpy", "rclpy.node", "rclpy.qos", "rclpy.duration", "rosidl_runtime_py", "rosidl_runtime_py.utilities")


class _QoSProfile:
    def __init__(self, depth: int = 10) -> None:
        self.depth = depth


class _Policy:
    BEST_EFFORT = "best_effort"
    RELIABLE = "reliable"
    KEEP_ALL = "keep_all"
    KEEP_LAST = "keep_last"
    TRANSIENT_LOCAL = "transient_local"
    VOLATILE = "volatile"
    AUTOMATIC = "automatic"
    MANUAL_BY_TOPIC = "manual_by_topic"


def _stub_module(name: str, **attrs: object) -> ModuleType:
    module = ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


class _RecordingLogger:
    """Counts what was said and at which level."""

    def __init__(self) -> None:
        self.warnings: list[str] = []
        self.debugs: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def debug(self, message: str) -> None:
        self.debugs.append(message)

    def info(self, message: str) -> None:
        pass

    def error(self, message: str) -> None:
        pass


@pytest.fixture(scope="module")
def qos_module() -> Iterator[ModuleType]:
    saved = {name: sys.modules.get(name) for name in _STUBBED}
    _stub_module("rclpy")
    _stub_module("rclpy.node", Node=object)
    _stub_module(
        "rclpy.qos",
        QoSProfile=_QoSProfile,
        ReliabilityPolicy=_Policy,
        HistoryPolicy=_Policy,
        DurabilityPolicy=_Policy,
        LivelinessPolicy=_Policy,
    )
    _stub_module("rclpy.duration", Duration=lambda **kwargs: kwargs)
    _stub_module("rosidl_runtime_py", utilities=None)
    _stub_module("rosidl_runtime_py.utilities", get_message=lambda type_str: None)

    spec = importlib.util.spec_from_file_location("rosotacom_qos_fallback", QOS_PY)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules["rosotacom_qos_fallback"] = module
    spec.loader.exec_module(module)
    yield module

    sys.modules.pop("rosotacom_qos_fallback", None)
    for name, previous in saved.items():
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


CONFIG = {
    "default": {"depth": 10},
    "topics": {"/tf_static": {"reliability": "reliable", "durability": "transient_local"}},
}


def test_a_processed_name_resolves_through_its_base_without_warning(qos_module: ModuleType) -> None:
    logger = _RecordingLogger()

    qos_module.get_topic_qos(logger, CONFIG, "/tf_static/globalframe", "ota_pub")

    assert logger.warnings == []
    assert any("Falling back to base topic='/tf_static'" in line for line in logger.debugs)


def test_the_fallback_still_delivers_the_base_topic_settings(qos_module: ModuleType) -> None:
    """Quiet must not mean ignored: the resolved profile is the base topic's."""
    logger = _RecordingLogger()

    profile = qos_module.get_topic_qos(logger, CONFIG, "/tf_static/globalframe", "ota_pub")

    assert profile.reliability == _Policy.RELIABLE
    assert profile.durability == _Policy.TRANSIENT_LOCAL


def test_an_exact_match_says_nothing_at_all(qos_module: ModuleType) -> None:
    logger = _RecordingLogger()

    qos_module.get_topic_qos(logger, CONFIG, "/tf_static", "ota_pub")

    assert logger.warnings == []
    assert not any("Falling back" in line for line in logger.debugs)
