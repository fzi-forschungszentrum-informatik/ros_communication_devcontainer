# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Guard against shadowing rclpy.Node's private ROS entity collections."""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
COM_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py"

# rclpy.Node.create_*() appends entities to these collections itself. Assigning
# either name in a subclass discards Node-owned state, and appending the returned
# entity again makes it appear twice in the executor wait set on ROS 2 Lyrical.
RCLPY_NODE_ENTITY_FIELDS = {"_subscriptions", "_publishers"}


@pytest.mark.parametrize("source", sorted(COM_PY.glob("*.py")), ids=lambda path: path.name)
def test_nodes_do_not_shadow_rclpy_entity_collections(source: Path) -> None:
    tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))

    collisions = []
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        for target in targets:
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id == "self"
                and target.attr in RCLPY_NODE_ENTITY_FIELDS
            ):
                collisions.append((target.attr, target.lineno))

    assert collisions == [], (
        "rclpy.Node owns these private entity collections; use a component-specific "
        f"_owned_* list instead: {collisions}"
    )
