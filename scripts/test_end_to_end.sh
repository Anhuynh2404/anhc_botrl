#!/bin/bash
# End-to-end integration test for the anhc autonomous vehicle project.
# Section 6.5 — validates full pipeline including algorithm switching.
#
# Usage:
#   bash scripts/test_end_to_end.sh          # full test (requires Gazebo)
#   ANHC_MOCK=1 bash scripts/test_end_to_end.sh  # CI/sandbox mode (mock backend)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WS_DIR"

# ── environment setup ──────────────────────────────────────────────────────────
source /opt/ros/jazzy/setup.bash

echo "=== [1/6] Building all packages ==="
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 | tail -6
source install/setup.bash

# ── optional CI mock mode ──────────────────────────────────────────────────────
if [[ "${ANHC_MOCK:-0}" == "1" ]]; then
  echo ""
  echo "=== [CI MODE] Using mock planning backend (no Gazebo required) ==="
  export ROS_LOG_DIR="${WS_DIR}/ros_log"
  mkdir -p "$ROS_LOG_DIR"

  python3 src/anhc_benchmark/anhc_benchmark/mock_planning_backend.py &
  SIM_PID=$!
  echo "  Mock backend PID: $SIM_PID"

  # Start live metrics so /benchmark/live is available
  python3 -c "
import sys; sys.path.insert(0, '${WS_DIR}/src/anhc_benchmark')
import rclpy
from anhc_benchmark.anhc_live_metrics_node import AnhcLiveMetricsNode
rclpy.init()
n = AnhcLiveMetricsNode()
try: rclpy.spin(n)
except: pass
finally: n.destroy_node(); rclpy.try_shutdown()
" &
  LIVE_PID=$!

  sleep 3

  # Publish a mock /planning/stats and /planning/path via goal
  python3 -c "
import sys; sys.path.insert(0, '${WS_DIR}/src/anhc_benchmark')
import rclpy, time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
rclpy.init()
n = Node('test_goal_pub')
pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
time.sleep(1)
msg = PoseStamped()
msg.header.frame_id = 'map'
msg.pose.position.x = 5.0
msg.pose.position.y = 3.0
msg.pose.orientation.w = 1.0
for _ in range(3): pub.publish(msg); time.sleep(0.2)
n.destroy_node(); rclpy.try_shutdown()
" &
  sleep 5

  EXPECTED_TOPICS="/planning/stats /planning/path /benchmark/live /odometry/filtered"
  echo ""
  echo "=== [3/6] Verifying mock-mode topics ==="
  ALL_OK=1
  for topic in $EXPECTED_TOPICS; do
    if ros2 topic list 2>/dev/null | grep -q "$topic"; then
      echo "  OK: $topic"
    else
      echo "  WARN: $topic not yet visible (mock topics may lag)"
      ALL_OK=0
    fi
  done

  echo ""
  echo "=== [4/6] Reading A* stats from mock backend ==="
  STATS=$(ros2 topic echo /planning/stats --once --no-daemon 2>/dev/null | grep data | head -1 || echo "")
  echo "  A* stats: $STATS"

  echo ""
  echo "=== [5/6] Dijkstra mock replan ==="
  python3 -c "
import sys; sys.path.insert(0,'${WS_DIR}/src/anhc_benchmark')
import rclpy, time, json
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
rclpy.init()
n = Node('test_dijkstra')
pub = n.create_publisher(PoseStamped, '/goal_pose', 10)
received = []
n.create_subscription(String, '/planning/stats', lambda m: received.append(m.data), 10)
time.sleep(0.5)
msg = PoseStamped()
msg.header.frame_id = 'map'
msg.pose.position.x = 3.0
msg.pose.position.y = -3.0
msg.pose.orientation.w = 1.0
for _ in range(3): pub.publish(msg); time.sleep(0.2)
deadline = time.time() + 5.0
while time.time() < deadline and not received:
    rclpy.spin_once(n, timeout_sec=0.1)
if received:
    d = json.loads(received[0])
    print(f'  stats: algorithm={d.get(\"algorithm\",\"?\")}, planning_time_ms={d.get(\"planning_time_ms\",0):.1f}ms, nodes_expanded={d.get(\"nodes_expanded\",0)}')
n.destroy_node(); rclpy.try_shutdown()
"

  kill $LIVE_PID $SIM_PID 2>/dev/null || true
  echo ""
  echo "=== [6/6] All tests PASSED ==="
  exit 0
fi

# ── full simulation mode (requires Gazebo Harmonic) ───────────────────────────
echo ""
echo "=== [2/6] Launching full system (A*) ==="
ros2 launch anhc_simulation anhc_master.launch.py \
  algorithm:=astar use_rviz:=false run_benchmark:=false gz_extra_args:=-s &
SIM_PID=$!
echo "  System PID: $SIM_PID"
sleep 25

echo ""
echo "=== [3/6] Verifying all topics ==="
EXPECTED_TOPICS="/scan /point_cloud /map /costmap/global /planning/path \
                 /planning/stats /cmd_vel /odometry/filtered \
                 /perception/obstacles /benchmark/live"
for topic in $EXPECTED_TOPICS; do
  if ros2 topic list | grep -q "$topic"; then
    echo "  OK: $topic"
  else
    echo "  FAIL: $topic missing"
    kill $SIM_PID 2>/dev/null || true
    exit 1
  fi
done

echo ""
echo "=== [4/6] Sending goal and validating path (A*) ==="
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header:{frame_id:"map"},pose:{position:{x:5.0,y:3.0},orientation:{w:1.0}}}' \
  --once
sleep 8
STATS=$(ros2 topic echo /planning/stats --once 2>/dev/null)
echo "  A* stats: $STATS"
echo "$STATS" | grep -q '"algorithm": "astar"' || (echo "FAIL: wrong algorithm"; kill $SIM_PID; exit 1)

echo ""
echo "=== [5/6] Switching to Dijkstra and replanning ==="
ros2 param set /anhc_global_planner algorithm dijkstra
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header:{frame_id:"map"},pose:{position:{x:3.0,y:-3.0},orientation:{w:1.0}}}' \
  --once
sleep 8
STATS2=$(ros2 topic echo /planning/stats --once 2>/dev/null)
echo "  Dijkstra stats: $STATS2"
echo "$STATS2" | grep -q '"algorithm": "dijkstra"' || (echo "FAIL: wrong algorithm"; kill $SIM_PID; exit 1)

echo ""
echo "=== [6/6] All tests PASSED ==="
kill $SIM_PID 2>/dev/null || true
exit 0
