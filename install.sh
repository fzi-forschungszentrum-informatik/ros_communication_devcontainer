#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2024-2026 the rosotacom contributors.
# Author: Martin Gontscharow <gontscharow@fzi.de>
# Date: 2026-10-01 (publication metadata; not a creation date)
# Licensed under BSD-3-Clause. See LICENSE.txt in the repository root.

set -euo pipefail

# Install rosotacom into a checkout-local virtual environment.
#
# Usage:
#   ./install.sh
#   ./install.sh --global-symlink   # optional legacy compatibility symlinks
#
# Optional env vars:
#   VENV_DIR=.venv
#   ROS2DOCKER_SPEC='ros2docker==0.1.5.dev10'
#   BIN_DIR=~/.local/bin

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"
VENV_DIR="${VENV_DIR:-"$ROOT_DIR/.venv"}"
ROS2DOCKER_SPEC="${ROS2DOCKER_SPEC:-"ros2docker==0.1.5.dev10"}"
BIN_DIR="${BIN_DIR:-"$HOME/.local/bin"}"

INSTALL_GLOBAL_SYMLINKS=false
for arg in "$@"; do
  case "$arg" in
    --global-symlink|--global-symlinks)
      INSTALL_GLOBAL_SYMLINKS=true
      ;;
    -h|--help)
      sed -n '1,18p' "$0"
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

if [[ ! -f "$ROOT_DIR/pyproject.toml" ]]; then
  echo "ERROR: pyproject.toml not found in $ROOT_DIR" >&2
  exit 1
fi

python3 -m venv "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install "$ROS2DOCKER_SPEC"
"$VENV_DIR/bin/python" -m pip install -e "$ROOT_DIR"

cat >> "$VENV_DIR/bin/activate" <<'EOF'

# >>> rosotacom shell completion >>>
# Completion follows the rosotacom executable currently selected by PATH, so
# activating another checkout automatically switches completion to that version.
case $- in
  *i*)
    if [ -n "${ZSH_VERSION:-}" ]; then
      eval "$("$VIRTUAL_ENV/bin/rosotacom" completion zsh)"
    elif [ -n "${BASH_VERSION:-}" ]; then
      eval "$("$VIRTUAL_ENV/bin/rosotacom" completion bash)"
    fi
    ;;
esac
# <<< rosotacom shell completion <<<
EOF

echo "Installed rosotacom into: $VENV_DIR"
echo
echo "Activate it (including shell completion) with:"
echo "  source \"$VENV_DIR/bin/activate\""
echo
echo "Try:"
echo "  rosotacom --version"
echo "  python -m rosotacom --version"
echo "  rosotacom doctor"
echo "  rosotacom examples create ./rosotacom_examples && cd ./rosotacom_examples"
echo "  rosotacom smoke    # ./rosotacom.yaml is auto-discovered from the current dir"
echo
echo "With no project nearby, rosotacom falls back to a built-in example, so"
echo "'rosotacom smoke' works with zero setup. Pin a default with:"
echo "  rosotacom config set project ./rosotacom.yaml --global"
echo
echo "For a released version instead of this checkout, use: pipx install rosotacom"
echo "For contributor checks, install dev tooling with:"
echo "  just setup"

if [[ "$INSTALL_GLOBAL_SYMLINKS" == true ]]; then
  mkdir -p "$BIN_DIR"
  ln -sf "$VENV_DIR/bin/rosotacom" "$BIN_DIR/rosotacom"
  echo
  echo "Installed global symlink into: $BIN_DIR"
  # `start_rosotacom` / `stop_rosotacom` were entry points until 2026-08-14. A
  # machine that ran an older install.sh still has their symlinks, and they now
  # point into a venv whose package no longer provides them. Only links this
  # script could have written are touched — into THIS checkout's venv, never a
  # file or a link belonging to something else.
  for retired in start_rosotacom stop_rosotacom; do
    link="$BIN_DIR/$retired"
    if [[ -L "$link" && "$(readlink -- "$link")" == "$VENV_DIR/bin/$retired" ]]; then
      rm -- "$link"
      echo "Removed retired symlink: $link  (use 'rosotacom start' / 'rosotacom stop')"
    fi
  done
fi
