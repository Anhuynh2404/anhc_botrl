#!/usr/bin/env bash
# Smoke benchmark (terminal 2): requires nav stack already running.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
fi
exec ros2 launch anhc_benchmark anhc_benchmark_smoke.launch.py "$@"
