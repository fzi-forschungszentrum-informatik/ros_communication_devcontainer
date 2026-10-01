# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""catmux substitutes ``${name}`` and nothing else.

`catmux.session.Session._replace_parameters` walks the loaded YAML and, for
every declared parameter, does ``re.sub(r"\\${%s}" % key, value, text)``. The
pattern requires the closing brace immediately after the name, so any bash
parameter-expansion form written around a *catmux parameter* survives into the
pane verbatim, where bash expands an unset shell variable to the empty string.

That is not hypothetical. Two of them shipped:

* ``${ip_local//./\\.}`` inside topic_monitor's interface lookup. The awk
  program collapsed to "first address line with a prefix length", i.e. ``lo``,
  and `/topic_monitor/*/link_bandwidth_kbps` reported this host's own ROS
  traffic as link bandwidth -- 28 Gbit/s next to a 6 Mbit/s link (#267).
* ``${status_write_interval_s:-2.0}`` in the status watcher, which quietly
  ignored a configured interval and always slept the literal default.

Neither failed; both reported. So the rule is checked rather than remembered.

Bash forms around variables the pane itself sets (``${topics//,/ }`` after
``topics=...``) or around environment variables (``${ROSOTACOM_CATMUX_LOG_DIR:-...}``)
are fine and stay allowed -- catmux never touches those names.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SESSION_CONTENT = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "session" / "content"
PLUGIN_BASE = SESSION_CONTENT / "base" / "session_plugin_base.yaml"

#: Any ``${...}`` group, including the ones catmux will not substitute.
_BRACED = re.compile(r"\$\{([^}]*)\}")
#: What catmux does substitute: the bare name, closing brace right after it.
_PLAIN_NAME = re.compile(r"^[A-Za-z_][A-Za-z_0-9]*$")


def _declared_parameters() -> set[str]:
    parameters = yaml.safe_load(PLUGIN_BASE.read_text(encoding="utf-8"))["parameters"]
    return set(parameters)


def _template_files() -> list[Path]:
    return sorted(path for path in SESSION_CONTENT.rglob("*.yaml") if "__pycache__" not in path.parts)


def test_the_plugin_base_is_where_parameters_are_declared() -> None:
    assert PLUGIN_BASE.is_file()
    assert "ip_local" in _declared_parameters()


@pytest.mark.parametrize("path", _template_files(), ids=lambda path: path.name)
def test_catmux_parameters_are_used_in_the_only_form_catmux_substitutes(path: Path) -> None:
    declared = _declared_parameters()
    offenders: list[str] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        for match in _BRACED.finditer(line):
            inner = match.group(1)
            if _PLAIN_NAME.match(inner):
                continue
            # A non-plain form is only a defect when it wraps a catmux
            # parameter; around a shell or environment variable it is correct.
            name = re.match(r"[A-Za-z_][A-Za-z_0-9]*", inner)
            if name and name.group(0) in declared:
                offenders.append(f"{path.name}:{line_number}: ${{{inner}}}")

    assert not offenders, (
        "catmux only substitutes ${name}; these wrap a declared parameter in a form it "
        "leaves literal, so bash expands an unset variable instead:\n  " + "\n  ".join(offenders)
    )
