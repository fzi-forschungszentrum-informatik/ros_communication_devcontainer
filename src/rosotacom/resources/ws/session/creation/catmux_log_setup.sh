#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

# shellcheck disable=SC2317
# Mirror the current catmux/tmux pane to a per-instance log file.
#
# This file is sourced from catmux common.before_commands. It must be tolerant:
# logging should never prevent a pane command from starting.

if [[ -z "${TMUX_PANE:-}" ]]; then
  echo "[rosotacom_catmux_log] no TMUX_PANE; skipping pane logging" >&2
  return 0 2>/dev/null || exit 0
fi

export RCUTILS_COLORIZED_OUTPUT="${RCUTILS_COLORIZED_OUTPUT:-0}"

ROSOTACOM_CATMUX_LOG_DIR="${ROSOTACOM_CATMUX_LOG_DIR:-${ROSOTACOM_LOGS_DIR:-/tmp/rosotacom_logs}/catmux}"
ROSOTACOM_ROSBAG_DIR="${ROSOTACOM_ROSBAG_DIR:-${ROSOTACOM_INSTANCE_DIR:-/tmp/rosotacom_instance}/rosbags}"
export ROSOTACOM_INSTANCE_DIR ROSOTACOM_CONFIG_DIR ROSOTACOM_CATMUX_LOG_DIR ROSOTACOM_ROSBAG_DIR

if ! command -v tmux >/dev/null 2>&1; then
  echo "[rosotacom_catmux_log] tmux not on PATH; pane logging disabled" >&2
  return 0 2>/dev/null || exit 0
fi

__rosotacom_window="$(tmux display-message -p -t "${TMUX_PANE}" '#{window_name}' 2>/dev/null)"
__rosotacom_window_index="$(tmux display-message -p -t "${TMUX_PANE}" '#{window_index}' 2>/dev/null)"
__rosotacom_pane="$(tmux display-message -p -t "${TMUX_PANE}" '#{pane_index}' 2>/dev/null)"
if [[ -z "${__rosotacom_window}" || -z "${__rosotacom_window_index}" || -z "${__rosotacom_pane}" ]]; then
  echo "[rosotacom_catmux_log] could not query tmux pane id; skipping" >&2
  return 0 2>/dev/null || exit 0
fi

__rosotacom_window_safe="$(printf '%s' "${__rosotacom_window}" | tr -c 'A-Za-z0-9_.-' '_')"
printf -v __rosotacom_window_prefix '%02d' "${__rosotacom_window_index}"
__rosotacom_pane_log_dir="${ROSOTACOM_CATMUX_LOG_DIR}/${__rosotacom_window_prefix}-${__rosotacom_window_safe}"
__rosotacom_pane_log_file="${__rosotacom_pane_log_dir}/${__rosotacom_pane}.log"
export ROSOTACOM_CATMUX_PANE_LOG_DIR="${__rosotacom_pane_log_dir}"
export ROSOTACOM_CATMUX_PANE_LOG_FILE="${__rosotacom_pane_log_file}"

if ! mkdir -p "${__rosotacom_pane_log_dir}" 2>/dev/null; then
  echo "[rosotacom_catmux_log] cannot create ${__rosotacom_pane_log_dir}; skipping" >&2
  return 0 2>/dev/null || exit 0
fi

__rosotacom_strip_ansi="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)/strip_ansi.py"
if [[ -x "${__rosotacom_strip_ansi}" ]] && command -v python3 >/dev/null 2>&1; then
  __rosotacom_pipe_filter="python3 '${__rosotacom_strip_ansi//\'/\'\\\'\'}'"
else
  echo "[rosotacom_catmux_log] strip_ansi.py unavailable; logging raw pane output" >&2
  __rosotacom_pipe_filter="cat"
fi

tmux pipe-pane -o -t "${TMUX_PANE}" \
  "${__rosotacom_pipe_filter} >> '${__rosotacom_pane_log_file//\'/\'\\\'\'}'"

# --- a pane whose command dies must say so -----------------------------------
#
# A pane log is only read by somebody who already suspects the pane. On
# 2026-08-13 the centre's `heartbeat_echo` and one `topic_monitor` exited two
# seconds after start (Cyclone had no free participant index left on domain 46),
# catmux left both panes at a shell prompt, and the session ran for two hours
# with no heartbeat publisher. Nothing anywhere said so: `docker ps` showed a
# healthy container, the other panes worked, and the far side blamed the link.
#
# So every non-zero exit of a pane command is appended to one file per peer,
# which the status overview's startup check reads and quotes. 130 (Ctrl-C) is
# excluded: that is an operator stopping a pane on purpose.
ROSOTACOM_PANE_FAILURE_LOG="$(dirname "${ROSOTACOM_CATMUX_LOG_DIR}")/pane_failures.log"
ROSOTACOM_PANE_LABEL="${__rosotacom_window_prefix}-${__rosotacom_window_safe}/${__rosotacom_pane}"
export ROSOTACOM_PANE_FAILURE_LOG ROSOTACOM_PANE_LABEL

__rosotacom_note_exit() {
  local __status=$?
  if [ "${__status}" -ne 0 ] && [ "${__status}" -ne 130 ]; then
    local __cmd
    __cmd="$(HISTTIMEFORMAT='' builtin history 1 2>/dev/null | sed 's/^[[:space:]]*[0-9]*[[:space:]]*//')"
    printf '%s\tpane=%s\texit=%s\tcommand=%s\n' \
      "$(date -Iseconds)" "${ROSOTACOM_PANE_LABEL}" "${__status}" "${__cmd}" \
      >> "${ROSOTACOM_PANE_FAILURE_LOG}" 2>/dev/null
    printf '\n[rosotacom] pane %s: command exited %s -- this pane is no longer doing its job.\n' \
      "${ROSOTACOM_PANE_LABEL}" "${__status}" >&2
    printf '[rosotacom] recorded in %s\n\n' "${ROSOTACOM_PANE_FAILURE_LOG}" >&2
  fi
  return "${__status}"
}

case "$-" in
  *i*)
    # Deliberately not exported: a child shell would inherit the hook without
    # the function and complain at every prompt.
    if [[ ";${PROMPT_COMMAND:-};" != *";__rosotacom_note_exit;"* ]]; then
      PROMPT_COMMAND="__rosotacom_note_exit;${PROMPT_COMMAND:+${PROMPT_COMMAND}}"
    fi
    ;;
  *)
    echo "[rosotacom_catmux_log] non-interactive shell; pane-exit reporting disabled" >&2
    ;;
esac

{
  printf '\n--- rosotacom catmux pipe-pane started %s ---\n' "$(date -Iseconds)"
  printf '    window=%s (#%s) pane=%s\n' \
    "${__rosotacom_window}" "${__rosotacom_window_index}" "${__rosotacom_pane}"
  printf '    log_file=%s\n' "${__rosotacom_pane_log_file}"
} >&2

unset __rosotacom_window __rosotacom_window_index __rosotacom_pane \
  __rosotacom_window_safe __rosotacom_window_prefix \
  __rosotacom_pane_log_dir __rosotacom_pane_log_file \
  __rosotacom_strip_ansi __rosotacom_pipe_filter
