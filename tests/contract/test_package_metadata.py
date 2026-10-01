# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def test_package_version_is_derived_from_scm_metadata() -> None:
    pyproject = (PACKAGE_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    package_init = (PACKAGE_ROOT / "src" / "rosotacom" / "__init__.py").read_text(encoding="utf-8")

    assert '\nversion = "0.1.0"' not in pyproject
    assert 'dynamic = ["version"]' in pyproject
    assert "setuptools-scm" in pyproject
    assert '__version__ = "0.1.0"' not in package_init


def test_rosotacom_is_the_only_console_script() -> None:
    """One installed command, so a shim in ~/.local/bin cannot be mistaken for it.

    `start_rosotacom` / `stop_rosotacom` were removed on 2026-08-14: they were
    `rosotacom start` / `rosotacom stop` under a second name (see
    DEVELOPMENT_PRINCIPLES.md, Compatibility Policy). This pins that no second
    name comes back.
    """
    pyproject = (PACKAGE_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    scripts = pyproject.split("[project.scripts]", 1)[1].split("\n[", 1)[0]
    entries = [line.split("=", 1)[0].strip() for line in scripts.splitlines() if "=" in line]

    assert 'rosotacom = "rosotacom.cli:main"' in pyproject
    assert entries == ["rosotacom"], entries


def test_videoquality_reader_dependencies_are_packaged() -> None:
    pyproject = (PACKAGE_ROOT / "pyproject.toml").read_text(encoding="utf-8")

    assert '"mcap>=1.4,<2"' in pyproject
    assert '"Pillow>=10,<13"' in pyproject


def test_checkout_installer_enables_completion_during_venv_activation() -> None:
    installer = (PACKAGE_ROOT / "install.sh").read_text(encoding="utf-8")

    assert "# >>> rosotacom shell completion >>>" in installer
    assert '"$VIRTUAL_ENV/bin/rosotacom" completion zsh' in installer
    assert '"$VIRTUAL_ENV/bin/rosotacom" completion bash' in installer
    assert "case $- in" in installer  # Do not register completion in non-interactive scripts.


def test_python_support_contract_is_consistent() -> None:
    pyproject = (PACKAGE_ROOT / "pyproject.toml").read_text(encoding="utf-8")
    contributing = (PACKAGE_ROOT / "CONTRIBUTING.md").read_text(encoding="utf-8")

    assert 'requires-python = ">=3.10"' in pyproject
    assert "3.10 through 3.14" in contributing
    for version in ("3.10", "3.11", "3.12", "3.13", "3.14"):
        assert f"Programming Language :: Python :: {version}" in pyproject
