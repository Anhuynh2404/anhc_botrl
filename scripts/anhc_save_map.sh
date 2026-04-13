#!/bin/bash
# Save current SLAM map to file
# Usage: bash scripts/anhc_save_map.sh [map_name]
MAP_NAME=${1:-"anhc_map_$(date +%Y%m%d_%H%M%S)"}
MAP_DIR="$(ros2 pkg prefix anhc_simulation 2>/dev/null || echo $HOME)/maps"
mkdir -p $MAP_DIR
echo "Saving map as: $MAP_DIR/$MAP_NAME"
ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/$MAP_NAME" \
  --ros-args -p use_sim_time:=true
echo "Map saved: $MAP_DIR/${MAP_NAME}.pgm and ${MAP_NAME}.yaml"
echo "Use MAP_PATH=$MAP_DIR/$MAP_NAME in navigation launch."
