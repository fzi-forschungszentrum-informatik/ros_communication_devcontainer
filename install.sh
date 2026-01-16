#!/usr/bin/env bash
set -euo pipefail

# Installs convenient CLI names into ~/.local/bin by symlinking:
# - start_rosotacom -> run_session_in_container.py
# - stop_rosotacom  -> stop_session_in_container.py
#
# Usage:
#   ./install.sh
#
# Optional env vars:
#   BIN_DIR=~/.local/bin         # target bin dir
#   START_NAME=start_rosotacom   # command name to create
#   STOP_NAME=stop_rosotacom     # command name to create

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"

BIN_DIR="${BIN_DIR:-"$HOME/.local/bin"}"
START_NAME="${START_NAME:-start_rosotacom}"
STOP_NAME="${STOP_NAME:-stop_rosotacom}"

START_TARGET="$ROOT_DIR/run_session_in_container.py"
STOP_TARGET="$ROOT_DIR/stop_session_in_container.py"
START_DEST="$BIN_DIR/$START_NAME"
STOP_DEST="$BIN_DIR/$STOP_NAME"

if [[ ! -f "$START_TARGET" ]]; then
  echo "ERROR: target script not found: $START_TARGET" >&2
  echo "Run this installer from the fleet_mgmt repo root." >&2
  exit 1
fi

if [[ ! -f "$STOP_TARGET" ]]; then
  echo "ERROR: target script not found: $STOP_TARGET" >&2
  echo "Run this installer from the fleet_mgmt repo root." >&2
  exit 1
fi

mkdir -p "$BIN_DIR"
chmod +x "$START_TARGET" "$STOP_TARGET"
ln -sf "$START_TARGET" "$START_DEST"
ln -sf "$STOP_TARGET" "$STOP_DEST"

echo "Installed: $START_DEST -> $START_TARGET"
echo "Installed: $STOP_DEST -> $STOP_TARGET"

if ! command -v "$START_NAME" >/dev/null 2>&1 || ! command -v "$STOP_NAME" >/dev/null 2>&1; then
  echo
  echo "NOTE: commands are not on PATH in this shell."
  echo "      Add this to your shell rc (~/.bashrc or ~/.zshrc):"
  echo "      export PATH=\"\$HOME/.local/bin:\$PATH\""
fi

echo
echo "Try:"
echo "  $START_NAME --help"
echo "  $STOP_NAME --help"


