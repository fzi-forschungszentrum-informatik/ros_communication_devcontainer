# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The OTA source bundle must not restate packaging metadata.

`ota-smoke --install-mode source` stages a source tree to each peer. When it is
invoked from an installed rosotacom there is no checkout to stage, so the bundle
is reconstructed from the installed package and given a generated
`pyproject.toml`.

That generated file used to carry a hand-written copy of the project's
dependencies, name and entry points. The copies drifted: `mcap` and `Pillow`
were added to the project and never to the literal, and the name still said
the old distribution name after a rename. A peer prepared from
an installed rosotacom therefore got an environment that imported fine and then
failed inside the first MCAP or image code path — remotely only, under OTA only,
and only when the run started from an install rather than a checkout.

These tests pin the metadata to a single source of truth.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

import rosotacom.cli as cli

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
PYPROJECT_PATH = PACKAGE_ROOT / "pyproject.toml"

# PEP 440 public version, optionally with a local segment. "0+unknown" — what an
# unresolvable distribution name degrades to — deliberately does not match.
VERSION_RE = re.compile(r"^\d+(\.\d+)*((a|b|rc|\.post|\.dev)\d+)*(\+[a-zA-Z0-9.]+)?$")


def _project_dependency_names() -> set[str]:
    """Runtime dependency names declared in pyproject.toml's [project] table."""
    text = PYPROJECT_PATH.read_text(encoding="utf-8")
    block = re.search(r"^dependencies = \[(.*?)^\]", text, re.MULTILINE | re.DOTALL)
    assert block is not None, "pyproject.toml has no [project] dependencies array"
    return {_normalise(item) for item in re.findall(r"\"([^\"]+)\"", block.group(1))}


def _normalise(requirement: str) -> str:
    """Distribution name of a requirement, lowercased with separators unified."""
    name = re.split(r"[<>=!~;\[\s]", requirement.strip(), maxsplit=1)[0]
    return re.sub(r"[-_.]+", "-", name).lower()


@pytest.fixture(scope="module")
def metadata() -> cli.OtaBundleMetadata:
    distribution = cli._ota_installed_distribution()
    if distribution is None:
        pytest.skip("rosotacom is not installed in this environment; the bundle cannot be derived")
    return cli._ota_bundle_metadata()


def test_bundle_dependencies_cover_every_runtime_dependency(metadata: cli.OtaBundleMetadata) -> None:
    """The regression: mcap and Pillow were declared but never bundled."""
    bundled = {_normalise(item) for item in metadata.dependencies}

    assert _project_dependency_names() <= bundled


def test_bundle_excludes_optional_dependencies(metadata: cli.OtaBundleMetadata) -> None:
    """Peers need the runtime stack, not the dev toolchain."""
    bundled = {_normalise(item) for item in metadata.dependencies}

    assert "pytest" not in bundled
    assert "ruff" not in bundled
    assert "matplotlib" not in bundled


def test_bundle_name_matches_the_published_distribution(metadata: cli.OtaBundleMetadata) -> None:
    text = PYPROJECT_PATH.read_text(encoding="utf-8")
    declared = re.search(r"^name = \"([^\"]+)\"", text, re.MULTILINE)
    assert declared is not None

    assert _normalise(metadata.name) == _normalise(declared.group(1))


def test_bundle_version_is_installable(metadata: cli.OtaBundleMetadata) -> None:
    """`0+unknown` in the generated pyproject makes the remote build fail."""
    assert metadata.version != "0+unknown"
    assert VERSION_RE.match(metadata.version), f"not a PEP 440 version: {metadata.version!r}"


def test_bundle_console_scripts_match_the_project(metadata: cli.OtaBundleMetadata) -> None:
    text = PYPROJECT_PATH.read_text(encoding="utf-8")
    block = re.search(r"^\[project\.scripts\](.*?)^\[", text, re.MULTILINE | re.DOTALL)
    assert block is not None
    declared = dict(re.findall(r"^(\S+) = \"([^\"]+)\"", block.group(1), re.MULTILINE))

    assert dict(metadata.console_scripts) == declared


def test_generated_pyproject_declares_the_derived_metadata(metadata: cli.OtaBundleMetadata, tmp_path: Path) -> None:
    """End to end: the file the peer builds from carries the derived values."""
    generated = (cli._ota_packaged_source_bundle(tmp_path) / "pyproject.toml").read_text(encoding="utf-8")

    assert f'name = "{metadata.name}"' in generated
    assert f'version = "{metadata.version}"' in generated
    for requirement in metadata.dependencies:
        assert f'"{requirement}"' in generated


def test_the_historical_literal_would_fail_this_guard() -> None:
    """Proof that the guard bites: the hand-written list that shipped up to 2.3."""
    shipped_until_2_3 = ("argcomplete>=3.6,<4", "PyYAML>=6", "ros2docker>=0.1.4,<0.2")
    bundled = {_normalise(item) for item in shipped_until_2_3}

    assert not _project_dependency_names() <= bundled


def test_requirement_extras_are_recognised() -> None:
    assert cli._ota_requirement_is_optional('pytest>=8.3; extra == "dev"')
    assert cli._ota_requirement_is_optional('matplotlib>=3.8; extra == "plots"')
    assert not cli._ota_requirement_is_optional("PyYAML>=6")
    # An environment marker is not an extra: the peer needs this one.
    assert not cli._ota_requirement_is_optional('tomli; python_version < "3.11"')
