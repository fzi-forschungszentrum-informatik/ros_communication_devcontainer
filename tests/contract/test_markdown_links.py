# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import re
from pathlib import Path
from urllib.parse import unquote, urlparse

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
README = (PACKAGE_ROOT / "README.md").resolve()
# session-instances holds gitignored runtime output; `rosotacom report` writes
# a generated report.md into instances, which is not public documentation.
IGNORED_DIRS = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    ".ruff_cache",
    ".venv",
    "build",
    "dist",
    "session-instances",
}
LINK_RE = re.compile(r"(?<!!)\[[^\]]+\]\((?P<target>[^)]+)\)")
REACHABILITY_ALLOWLIST_GLOBS = (
    # GitHub renders these through issue / PR UI flows; they are executable
    # templates, not standalone public documentation.
    ".github/ISSUE_TEMPLATE/*",
    ".github/PULL_REQUEST_TEMPLATE.md",
    # Per-release notes and copy-me templates are surfaced from releases or by
    # explicit release work, not as normal README-starting documentation.
    "docs/release-notes/*",
    "*TEMPLATE*",
)


def _markdown_files() -> list[Path]:
    return sorted(
        path
        for path in PACKAGE_ROOT.rglob("*.md")
        if not any(part in IGNORED_DIRS or part.endswith(".egg-info") for part in path.relative_to(PACKAGE_ROOT).parts)
    )


def _local_link_target(raw_target: str) -> str | None:
    target = raw_target.strip()
    if not target or target.startswith("#"):
        return None
    parsed = urlparse(target)
    if parsed.scheme or parsed.netloc:
        return None
    return unquote(target.split("#", 1)[0])


def _internal_markdown_links(markdown_path: Path) -> list[Path]:
    targets: list[Path] = []
    for match in LINK_RE.finditer(markdown_path.read_text(encoding="utf-8")):
        target = _local_link_target(match.group("target"))
        if target is None or not target.endswith(".md"):
            continue
        resolved = (markdown_path.parent / target).resolve()
        if resolved.exists() and resolved.is_relative_to(PACKAGE_ROOT):
            targets.append(resolved)
    return targets


def _reachable_from_readme() -> set[Path]:
    seen = {README}
    stack = [README]
    while stack:
        current = stack.pop()
        for target in _internal_markdown_links(current):
            if target not in seen:
                seen.add(target)
                stack.append(target)
    return seen


def _is_reachability_allowlisted(path: Path) -> bool:
    rel_path = path.relative_to(PACKAGE_ROOT)
    return any(rel_path.match(glob) for glob in REACHABILITY_ALLOWLIST_GLOBS)


def test_internal_markdown_links_resolve() -> None:
    failures: list[str] = []

    for markdown_path in _markdown_files():
        for match in LINK_RE.finditer(markdown_path.read_text(encoding="utf-8")):
            target = _local_link_target(match.group("target"))
            if target is None:
                continue
            resolved = (markdown_path.parent / target).resolve()
            if not resolved.exists():
                rel_path = markdown_path.relative_to(PACKAGE_ROOT)
                failures.append(f"{rel_path}: missing link target {target!r}")

    assert not failures, "\n".join(failures)


def test_public_docs_are_reachable_from_readme() -> None:
    reachable = _reachable_from_readme()
    orphans = [
        str(markdown_path.relative_to(PACKAGE_ROOT))
        for markdown_path in _markdown_files()
        if not _is_reachability_allowlisted(markdown_path) and markdown_path.resolve() not in reachable
    ]

    assert not orphans, (
        "These Markdown docs are not reachable by following internal .md links "
        "from README.md. Link them into the README-starting docs graph, or add "
        "a narrow allowlist entry with a reason:\n" + "\n".join(orphans)
    )
