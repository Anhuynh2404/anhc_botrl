#!/usr/bin/env bash
# Remove colcon build artifacts from the anhc_botrl workspace.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "[anhc] Cleaning workspace: $WS_DIR"
rm -rf "$WS_DIR/build" "$WS_DIR/install" "$WS_DIR/log"
echo "[anhc] Removed: build/ install/ log/"
