# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

"""The pane-exit reporter in `catmux_log_setup.sh`.

A catmux pane whose command dies drops back to a shell prompt and keeps looking
alive. On 2026-08-13 that hid two dead nodes on the centre for two hours, so the
script now records every non-zero exit into one file per peer, which the status
overview's startup check reads.

The script needs `tmux` and `TMUX_PANE`, so the test provides a fake `tmux` that
answers the three `display-message` queries and swallows `pipe-pane`. bash runs
`PROMPT_COMMAND` when it is interactive, which it is when reading a script from
a pipe -- exactly how the assertions below reach the hook.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
SETUP_SH = REPO_ROOT / "src" / "rosotacom" / "resources" / "ws" / "session" / "creation" / "catmux_log_setup.sh"

_FAKE_TMUX = """#!/usr/bin/env bash
# Only what catmux_log_setup.sh asks for:
#   tmux display-message -p -t <pane> '#{window_name}'
if [ "$1" = display-message ]; then
    case "$5" in
        '#{window_name}')  echo BEAT ;;
        '#{window_index}') echo 5 ;;
        '#{pane_index}')   echo 0 ;;
        *) echo "fake tmux: unexpected format $5" >&2; exit 64 ;;
    esac
    exit 0
fi
exit 0
"""


@pytest.fixture
def pane_env(tmp_path: Path) -> tuple[Path, dict[str, str]]:
    """A catmux log directory plus an environment whose `tmux` is the fake one."""
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    tmux = fake_bin / "tmux"
    tmux.write_text(_FAKE_TMUX, encoding="utf-8")
    tmux.chmod(0o755)

    catmux_dir = tmp_path / "logs" / "center" / "catmux"
    catmux_dir.mkdir(parents=True)

    env = dict(os.environ)
    env["PATH"] = f"{fake_bin}:{env['PATH']}"
    env["TMUX_PANE"] = "%7"
    env["ROSOTACOM_CATMUX_LOG_DIR"] = str(catmux_dir)
    env.pop("PROMPT_COMMAND", None)
    return catmux_dir, env


def _run(script: str, env: dict[str, str]) -> subprocess.CompletedProcess[str]:
    bash = shutil.which("bash")
    assert bash, "bash is required for this test"
    return subprocess.run(
        [bash, "-i"],
        input=f"source {SETUP_SH}\n{script}\nexit 0\n",
        env=env,
        text=True,
        capture_output=True,
        timeout=60,
        check=False,
    )


def test_a_failing_pane_command_is_recorded_and_announced(pane_env: tuple[Path, dict[str, str]]) -> None:
    catmux_dir, env = pane_env

    result = _run("ros2-run-that-does-not-exist", env)

    failure_log = catmux_dir.parent / "pane_failures.log"
    assert failure_log.is_file(), result.stderr
    line = failure_log.read_text(encoding="utf-8").strip()
    assert "pane=05-BEAT/0" in line
    assert "exit=127" in line
    assert "ros2-run-that-does-not-exist" in line
    assert "no longer doing its job" in result.stderr


def test_a_pane_whose_command_succeeds_writes_nothing(pane_env: tuple[Path, dict[str, str]]) -> None:
    catmux_dir, env = pane_env

    _run("true", env)

    assert not (catmux_dir.parent / "pane_failures.log").exists()


def test_an_operator_interrupt_is_not_a_failure(pane_env: tuple[Path, dict[str, str]]) -> None:
    """130 is Ctrl-C: somebody stopped a pane on purpose."""
    catmux_dir, env = pane_env

    _run("(exit 130)", env)

    assert not (catmux_dir.parent / "pane_failures.log").exists()


def test_the_hook_is_installed_once_even_if_sourced_twice(pane_env: tuple[Path, dict[str, str]]) -> None:
    _catmux_dir, env = pane_env

    result = _run(f'source {SETUP_SH}\necho "PC=$PROMPT_COMMAND"', env)

    (line,) = [ln for ln in result.stdout.splitlines() if ln.startswith("PC=")]
    assert line.count("__rosotacom_note_exit") == 1


# What the recorder cannot know, and why the session template has to care.
#
# The hook runs from PROMPT_COMMAND, so it sees every line a pane shell runs --
# including the lines that only assemble an argument list before the node
# starts. A line of the form `[[ cond ]] && args+=(...)` exits 1 whenever the
# condition is false, which is the normal case for every option a session leaves
# unset, so each one recorded itself as a dead pane. A peer then reported four
# exited panes at every start while nothing was wrong.

_UNSET_OPTION = '[[ "None" != "None" ]]'
_SET_OPTION = '[[ "22" != "None" ]]'


def _failure_log(catmux_dir: Path) -> Path:
    return catmux_dir.parent / "pane_failures.log"


def test_the_short_circuit_form_records_a_false_failure(pane_env: tuple[Path, dict[str, str]]) -> None:
    """The shape that was in the template: correct behaviour, status 1."""
    catmux_dir, env = pane_env

    _run(f'args=(); {_UNSET_OPTION} && args+=(-p "bit_rate:=None")', env)

    assert _failure_log(catmux_dir).is_file()
    assert "exit=1" in _failure_log(catmux_dir).read_text(encoding="utf-8")


def test_the_if_form_leaves_an_unset_option_silent(pane_env: tuple[Path, dict[str, str]]) -> None:
    """The shape it was replaced with: same behaviour, status 0."""
    catmux_dir, env = pane_env

    _run(f'args=(); if {_UNSET_OPTION}; then args+=(-p "bit_rate:=None"); fi', env)

    assert not _failure_log(catmux_dir).exists()


def test_the_if_form_still_appends_a_set_option(pane_env: tuple[Path, dict[str, str]]) -> None:
    """Silencing the status must not silence the argument."""
    catmux_dir, env = pane_env

    result = _run(
        f'args=(); if {_SET_OPTION}; then args+=(-p "bit_rate:=22"); fi\necho "ARGS=${{args[*]}}"',
        env,
    )

    assert "ARGS=-p bit_rate:=22" in result.stdout
    assert not _failure_log(catmux_dir).exists()


def test_no_session_template_line_uses_the_short_circuit_form() -> None:
    """The template is where this reaches a running session, so it is checked there."""
    template = (
        REPO_ROOT
        / "src"
        / "rosotacom"
        / "resources"
        / "ws"
        / "session"
        / "content"
        / "base"
        / "session_plugin_base.yaml"
    )
    offenders = [
        line.strip()
        for line in template.read_text(encoding="utf-8").splitlines()
        if line.lstrip().startswith("- '[[") and "&&" in line
    ]

    assert offenders == [], (
        "these lines exit non-zero when their option is unset and land in "
        f"pane_failures.log; use `if ...; then ...; fi`: {offenders}"
    )
