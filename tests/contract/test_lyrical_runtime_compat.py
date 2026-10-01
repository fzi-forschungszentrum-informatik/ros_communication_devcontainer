# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Contracts for command-line changes made by ROS 2 Lyrical."""

from __future__ import annotations

import re
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION_TEMPLATE = (
    REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "session" / "content" / "base" / "session_plugin_base.yaml"
)


def test_metric_bag_uses_explicit_topics_option() -> None:
    template = SESSION_TEMPLATE.read_text(encoding="utf-8")
    assert "ros2 bag record -s mcap" in template
    assert "--topics ${topics//,/ }" in template


def test_image_transport_sets_kilted_and_lyrical_parameter_namespaces() -> None:
    template = SESSION_TEMPLATE.read_text(encoding="utf-8")
    parameter_commands = "\n".join(
        line for line in template.splitlines() if line.lstrip().startswith("- 'if") and "args+=(-p" in line
    )
    kilted = Counter(re.findall(r'"ut\.([a-z_.]+):=', parameter_commands))
    lyrical = Counter(re.findall(r'"out\.([a-z_.]+):=', parameter_commands))

    assert kilted
    assert lyrical == kilted
