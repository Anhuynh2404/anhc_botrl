#!/usr/bin/env bash
# Build the anhc_botrl workspace with symlink installs.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "[anhc] Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash

echo "[anhc] Building workspace at: $WS_DIR"
cd "$WS_DIR"

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --event-handlers console_cohesion+ \
  "$@"

echo "[anhc] Build complete. Source with:"
echo "  source $WS_DIR/install/setup.bash"
