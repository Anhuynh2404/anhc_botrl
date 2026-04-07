#!/usr/bin/env bash
# Launch the full anhc autonomous vehicle simulation stack.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
SETUP_FILE="$WS_DIR/install/setup.bash"

if [[ ! -f "$SETUP_FILE" ]]; then
  echo "[anhc] ERROR: Workspace not built. Run ./scripts/build.sh first."
  exit 1
fi

echo "[anhc] Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "[anhc] Sourcing workspace overlay..."
source "$SETUP_FILE"

echo "[anhc] Launching full simulation stack..."
# TODO: replace with the main launch file once implemented
# ros2 launch anhc_simulation sim_full.launch.py
echo "[anhc] Placeholder: main launch file not yet implemented."
echo "[anhc] Available packages:"
ros2 pkg list | grep anhc
