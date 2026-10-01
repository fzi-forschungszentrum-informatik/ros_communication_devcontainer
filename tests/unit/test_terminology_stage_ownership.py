# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The stage-ownership rule in `terminology.md` §9.1, against the code.

The rule is normative — it decides what may be added to this project — and it
argues from concrete pairs: decompression undoes compression, the unwrapper
undoes the wrapper, `global_to_local` undoes `local_to_global`, `trickle`
undoes `latch`, and the playout pacer compensates for the medium. A rule that
reasons from examples is only as good as the examples, and a renamed node
leaves it quietly arguing from things that are not there any more.

This is a documentation test, not a behaviour test: there is no behaviour to
run. What it pins is that every node the rule names still exists, that the two
tests it offers still name assertions the session schema accepts, and that the
section it belongs to is still where §7 sends the reader.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
TERMINOLOGY = REPO_ROOT / "terminology.md"
COM_PY = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "ros2src" / "com_py" / "com_py"
STATUS_EVAL = REPO_ROOT / "src" / "rosotacom" / "status_eval.py"
GENERATOR_PY = (
    REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "session" / "creation" / "generate_session_files.py"
)

#: The nodes §9.1 argues from, as module names in `com_py`.
CITED_NODES = (
    "universal_compressor",
    "universal_decompressor",
    "universal_ota_wrapper",
    "universal_ota_unwrapper",
    "local_global_frame_bridge",
    "trickle",
    "playout_pacer",
)

#: The processing keys §9.1's inverse pairs are stated in. A session has to
#: still be able to declare each of them, or the pair argues from nothing.
CITED_STAGE_KEYS = ("latch", "local_to_global", "global_to_local")

#: The assertion keys §9.1 offers as its falsifiable test. They are only a
#: test if a session may actually declare them, so each is checked against the
#: evaluator that reads them. `vs_bag_ratio` sits under `completeness` and is
#: named that way in the text for the same reason.
CITED_EXPECTATIONS = ("hz", "loss_pct", "latency_ms", "completeness.vs_bag_ratio")


#: `## 9.` and `### App Relay` are both headings, so a section has to run to
#: the next heading of its own level or higher, not to the next heading.
def section(title: str) -> str:
    """The text of one `terminology.md` section, up to the next peer heading."""
    text = TERMINOLOGY.read_text(encoding="utf-8")
    match = re.search(
        rf"^(#+) {re.escape(title)}\s*$",
        text,
        re.MULTILINE,
    )
    assert match is not None, f"terminology.md has no section '{title}'"
    depth = len(match.group(1))
    rest = text[match.end() :]
    end = re.search(rf"^#{{1,{depth}}} ", rest, re.MULTILINE)
    return rest[: end.start()] if end else rest


def test_the_rule_is_where_the_stage_definition_sends_the_reader():
    # §7 defers the ownership question rather than answering it twice.
    assert "§9.1" in section("7. Processing and transformations")
    assert section("9.1 Stage ownership (which transformations belong to this project)")


def test_every_node_the_rule_argues_from_still_exists():
    missing = [name for name in CITED_NODES if not (COM_PY / f"{name}.py").is_file()]

    assert not missing, f"terminology.md §9.1 cites nodes that are gone: {missing}"


def test_the_stage_keywords_the_rule_argues_from_are_still_session_vocabulary():
    # The rule reasons from inverse pairs, so it breaks quietly when a session
    # key is renamed: the prose keeps arguing from a `latch` or a
    # `local_to_global` that no definition can declare any more.
    body = section("9.1 Stage ownership (which transformations belong to this project)")
    generator = GENERATOR_PY.read_text(encoding="utf-8")

    for key in CITED_STAGE_KEYS:
        assert f"`{key}`" in body, f"§9.1 no longer argues from '{key}'"
        assert f'"{key}"' in generator, (
            f"§9.1 argues from '{key}', which the session generator no longer knows as a processing key"
        )


def test_the_falsifiable_test_names_assertions_a_session_may_declare():
    body = section("9.1 Stage ownership (which transformations belong to this project)")
    evaluator = STATUS_EVAL.read_text(encoding="utf-8")

    for key in CITED_EXPECTATIONS:
        assert f"`{key}`" in body, f"§9.1 no longer offers '{key}'"
        for part in key.split("."):
            assert f'"{part}"' in evaluator, (
                f"§9.1 offers '{key}' as an expectation, but status_eval does not know '{part}'"
            )
