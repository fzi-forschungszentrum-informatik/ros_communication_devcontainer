# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import ast
import os
import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
TESTS_ROOT = PACKAGE_ROOT / "tests"


def _test_python_files() -> list[Path]:
    return sorted(TESTS_ROOT.rglob("test_*.py"))


def test_pytest_collects_every_test_module() -> None:
    env = os.environ.copy()
    src_path = str(PACKAGE_ROOT / "src")
    pythonpath = env.get("PYTHONPATH")
    env["PYTHONPATH"] = f"{src_path}{os.pathsep}{pythonpath}" if pythonpath else src_path
    result = subprocess.run(
        [sys.executable, "-m", "pytest", "--collect-only", "-q"],
        cwd=PACKAGE_ROOT,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    collected = result.stdout.splitlines()
    missing = [
        str(path.relative_to(PACKAGE_ROOT))
        for path in _test_python_files()
        if not any(nodeid.startswith(f"{path.relative_to(PACKAGE_ROOT)}::") for nodeid in collected)
    ]
    assert not missing, "Uncollected test modules:\n" + "\n".join(missing)


def test_skip_and_xfail_marks_have_explicit_reasons() -> None:
    missing_reasons: list[str] = []

    for path in sorted(TESTS_ROOT.rglob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not _is_skip_or_xfail_mark(node.func):
                continue
            if not any(keyword.arg == "reason" for keyword in node.keywords):
                rel_path = path.relative_to(PACKAGE_ROOT)
                missing_reasons.append(f"{rel_path}:{node.lineno}")

    assert not missing_reasons, "pytest skip/xfail marks need reason=:\n" + "\n".join(missing_reasons)


def _is_skip_or_xfail_mark(node: ast.AST) -> bool:
    return (
        isinstance(node, ast.Attribute)
        and node.attr in {"skip", "skipif", "xfail"}
        and isinstance(node.value, ast.Attribute)
        and node.value.attr == "mark"
        and isinstance(node.value.value, ast.Name)
        and node.value.value.id == "pytest"
    )
