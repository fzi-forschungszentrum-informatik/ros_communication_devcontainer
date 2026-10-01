# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""Source notices, retained third-party exceptions and distribution metadata."""

import ast
import runpy
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ROS = ROOT / "src/rosotacom/resources/ws/ros2src/com_py"


def test_code_has_license_author_and_date():
    paths = subprocess.check_output(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard"], cwd=ROOT, text=True
    )
    for name in paths.splitlines():
        path = ROOT / name
        if not path.is_file() or not (
            path.suffix in {".py", ".cpp", ".sh", ".msg"} or path.name in {"CMakeLists.txt", "justfile"}
        ):
            continue
        text = path.read_text()
        header = "\n".join(text.splitlines()[:25])
        expected = "Apache-2.0" if path.parent == ROS / "test" else "BSD-3-Clause"
        assert f"SPDX-License-Identifier: {expected}" in header, name
        assert "Copyright" in header and "Author:" in header and "Date:" in header, name
        assert "{copyright_holder}" not in header, name


def test_third_party_notices_and_data_license_are_explicit():
    for name in ("test_copyright.py", "test_flake8.py", "test_pep257.py"):
        text = (ROS / "test" / name).read_text()
        assert "Open Source Robotics Foundation, Inc." in text
        assert "Licensed under the Apache License, Version 2.0" in text
    assert "TERMS AND CONDITIONS" in (ROOT / "LICENSES/Apache-2.0.txt").read_text()
    assert (ROS / "LICENSES/Apache-2.0.txt").read_text().strip() == (
        ROOT / "LICENSES/Apache-2.0.txt"
    ).read_text().strip()
    assert 'license = "BSD-3-Clause AND Apache-2.0 AND CC-BY-4.0"' in (ROOT / "pyproject.toml").read_text()
    assert '"NOTICE.txt"' in (ROOT / "pyproject.toml").read_text()
    assert (ROOT / "LICENSE.txt").read_text().startswith("Copyright")


def test_ros_console_entry_points_resolve_to_source_modules():
    tree = ast.parse((ROS / "setup.py").read_text())
    setup = next(
        n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "setup"
    )
    entries = ast.literal_eval(next(k.value for k in setup.keywords if k.arg == "entry_points"))
    for entry in entries["console_scripts"]:
        target = entry.split("=", 1)[1].strip()
        module, function = target.split(":")
        path = ROS / (module.replace(".", "/") + ".py")
        assert path.is_file(), entry
        assert any(
            isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)) and n.name == function
            for n in ast.parse(path.read_text()).body
        ), entry


def test_ros_package_copyright_check_runs_without_ros():
    runpy.run_path(str(ROS / "test/test_copyright.py"))["test_copyright"]()
