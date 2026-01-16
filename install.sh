#!/usr/bin/env bash
set -euo pipefail

# Installs a convenient CLI name (default: rosotacom) into ~/.local/bin by symlinking
# to run_session_in_container.py.
#
# Usage:
#   ./install_rosotacom.sh
#
# Optional env vars:
#   BIN_DIR=~/.local/bin   # target bin dir
#   NAME=rosotacom         # command name to create

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"

BIN_DIR="${BIN_DIR:-"$HOME/.local/bin"}"
NAME="${NAME:-rosotacom}"

TARGET="$ROOT_DIR/run_session_in_container.py"
DEST="$BIN_DIR/$NAME"

if [[ ! -f "$TARGET" ]]; then
  echo "ERROR: target script not found: $TARGET" >&2
  echo "Run this installer from the fleet_mgmt repo root." >&2
  exit 1
fi

mkdir -p "$BIN_DIR"
chmod +x "$TARGET"
ln -sf "$TARGET" "$DEST"

echo "Installed: $DEST -> $TARGET"

if ! command -v "$NAME" >/dev/null 2>&1; then
  echo
  echo "NOTE: '$NAME' is not on PATH in this shell."
  echo "      Add this to your shell rc (~/.bashrc or ~/.zshrc):"
  echo "      export PATH=\"\$HOME/.local/bin:\$PATH\""
fi

echo
echo "Try:"
echo "  $NAME --help"


