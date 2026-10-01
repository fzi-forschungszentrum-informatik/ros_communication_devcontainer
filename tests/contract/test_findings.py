# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

from __future__ import annotations

import re
import sys
from pathlib import Path
from urllib.parse import unquote

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
FINDINGS_DIR = PACKAGE_ROOT / "docs" / "findings"
INDEX_FILE = FINDINGS_DIR / "README.md"
ROOT_README = PACKAGE_ROOT / "README.md"

REQUIRED_SECTIONS = ["Claim", "Setup", "Evidence", "Status", "Publication notes"]
SECTION_RE = re.compile(r"^## (?P<section>.+?)\s*$", re.MULTILINE)
LINK_RE = re.compile(r"\[[^\]]+\]\((?P<target>[^)]+)\)")
STATUS_RE = re.compile(r"^(confirmed|refuted|superseded-by\s+\S+),\s+\d{4}-\d{2}-\d{2}\.", re.IGNORECASE)
VERIFICATION_RE = re.compile(r"^Verification:\s+\S", re.MULTILINE)


def _markdown_links(path: Path) -> set[str]:
    text = path.read_text(encoding="utf-8")
    links: set[str] = set()
    for match in LINK_RE.finditer(text):
        target = unquote(match.group("target").split("#", 1)[0])
        if target:
            links.add(target)
    return links


def _section_spans(text: str) -> dict[str, str]:
    matches = list(SECTION_RE.finditer(text))
    spans: dict[str, str] = {}
    for index, match in enumerate(matches):
        start = match.end()
        end = matches[index + 1].start() if index + 1 < len(matches) else len(text)
        spans[match.group("section")] = text[start:end].strip()
    return spans


def _check_finding(path: Path) -> str | None:
    text = path.read_text(encoding="utf-8")
    if not re.search(r"^# .+", text, re.MULTILINE):
        return "missing H1 title"

    headings = [match.group("section") for match in SECTION_RE.finditer(text)]
    if headings != REQUIRED_SECTIONS:
        return f"headings {headings!r} != {REQUIRED_SECTIONS!r}"

    sections = _section_spans(text)
    for section in REQUIRED_SECTIONS:
        if not sections[section]:
            return f"empty section {section!r}"

    if not VERIFICATION_RE.search(text):
        return "missing Verification: entry"

    setup = sections["Setup"].lower()
    for required in ("host pair / topology", "rosotacom sha", "profile", "seed policy"):
        if required not in setup:
            return f"setup missing {required!r}"

    status = sections["Status"].splitlines()[0]
    if not STATUS_RE.match(status):
        return "status must be 'confirmed|refuted|superseded-by <link>, YYYY-MM-DD.'"

    return None


def _ledger_errors() -> list[str]:
    errors: list[str] = []
    if not FINDINGS_DIR.is_dir():
        return [f"missing {FINDINGS_DIR.relative_to(PACKAGE_ROOT)}"]
    if not INDEX_FILE.is_file():
        return [f"missing {INDEX_FILE.relative_to(PACKAGE_ROOT)}"]

    finding_files = sorted(path for path in FINDINGS_DIR.glob("*.md") if path.name != "README.md")
    if len(finding_files) < 6:
        errors.append(f"expected at least 6 finding files, found {len(finding_files)}")

    for path in finding_files:
        error = _check_finding(path)
        if error:
            errors.append(f"{path.relative_to(PACKAGE_ROOT)}: {error}")

    index_links = {Path(target).name for target in _markdown_links(INDEX_FILE)}
    missing_from_index = [path.name for path in finding_files if path.name not in index_links]
    if missing_from_index:
        errors.append(f"findings missing from index: {', '.join(missing_from_index)}")

    extra_index_links = sorted(
        name for name in index_links if name.endswith(".md") and not (FINDINGS_DIR / name).exists()
    )
    if extra_index_links:
        errors.append(f"index links missing files: {', '.join(extra_index_links)}")

    root_links = _markdown_links(ROOT_README)
    if "docs/findings/README.md" not in root_links:
        errors.append("README.md must link docs/findings/README.md")

    return errors


def test_findings_ledger_contract() -> None:
    assert not _ledger_errors()


def test_finding_without_verification_entry_fails(tmp_path: Path) -> None:
    finding = tmp_path / "finding.md"
    finding.write_text(
        """# Missing Verification

## Claim

Claim text.

## Setup

- Host pair / topology: public local benchmark.
- rosotacom SHA: test.
- Profile: test.
- Seed policy: seedless.

## Evidence

Evidence text.

## Status

confirmed, 2026-07-08.

## Publication notes

Notes.
""",
        encoding="utf-8",
    )

    assert _check_finding(finding) == "missing Verification: entry"


def main() -> int:
    errors = _ledger_errors()
    if errors:
        for error in errors:
            print(f"findings: FAIL: {error}", file=sys.stderr)
        return 1
    print("findings: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
