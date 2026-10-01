# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""`session-configuration.md` says "Only these keys are allowed". Make it true.

The reference listed eight `processing` keys while the generator accepted
eleven. `latch`, `trickle_hz` and `use_ota_wrapper` were live, used by real
sessions, and absent — which is worse than merely undocumented, because the
sentence above the list tells a reader there is nothing else to find.
"""

from __future__ import annotations

import re
from pathlib import Path

from rosotacom.resources.ws.session.creation.generate_session_files import KNOWN_PROCESSING_KEYS

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
REFERENCE = PACKAGE_ROOT / "session-configuration.md"

#: The fenced block under the "Allowed `processing` keys" heading.
BLOCK = re.compile(
    r"#### Allowed `processing` keys.*?```yaml\n(?P<body>.*?)```",
    re.DOTALL,
)

#: A top-level key inside that block: two spaces of indent, then `name:`.
TOP_LEVEL_KEY = re.compile(r"^  ([A-Za-z_][A-Za-z0-9_]*):", re.MULTILINE)


def documented_processing_keys() -> set[str]:
    match = BLOCK.search(REFERENCE.read_text(encoding="utf-8"))
    assert match, "no fenced `processing` example under the allowed-keys heading"
    return set(TOP_LEVEL_KEY.findall(match.group("body")))


def test_documented_processing_keys_match_the_generator() -> None:
    documented = documented_processing_keys()

    undocumented = sorted(KNOWN_PROCESSING_KEYS - documented)
    assert not undocumented, (
        f"the generator accepts these `processing` keys and {REFERENCE.name} does not list them: {undocumented}"
    )

    invented = sorted(documented - KNOWN_PROCESSING_KEYS)
    assert not invented, f"{REFERENCE.name} documents `processing` keys the generator rejects: {invented}"
