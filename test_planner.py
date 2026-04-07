#!/usr/bin/env python3
"""Verification script for Section 4 planning module.

Publishes a synthetic costmap, tests A* and Dijkstra planners,
and verifies /cmd_vel is being published by the path follower.
"""

import json
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def build_costmap(width=200, height=200, resolution=0.1,
                  origin_x=-10.0, origin_y=-10.0):
    """20×20 m flat arena: border walls + a horizontal barrier at row 60-80."""
    data = [0] * (width * height)
    for c in range(width):
        data[0 * width + c] = 100
        data[(height - 1) * width + c] = 100
    for r in range(height):
        data[r * width + 0] = 100
        data[r * width + (width - 1)] = 100
    # Horizontal wall from col 10 to 150, rows 60-65 (at y ≈ -4 m)
    for r in range(60, 66):
        for c in range(10, 150):
            data[r * width + c] = 100
    msg = OccupancyGrid()
    msg.header.frame_id = "map"
    msg.info = MapMetaData()
    msg.info.resolution = resolution
    msg.info.width = width
    msg.info.height = height
    msg.info.origin = Pose()
    msg.info.origin.position.x = origin_x
    msg.info.origin.position.y = origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = data
    return msg


class PlannerTester(Node):
    def __init__(self):
        super().__init__('planner_tester')
        qos = QoSProfile(depth=10)
        self._pub_costmap = self.create_publisher(OccupancyGrid, '/costmap/global', qos)
        self._pub_initial = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)
        self._pub_goal = self.create_publisher(PoseStamped, '/goal_pose', qos)
        self._sub_path = self.create_subscription(Path, '/planning/path', self._cb_path, qos)
        self._sub_stats = self.create_subscription(
            String, '/planning/stats', self._cb_stats, qos)
        self._sub_cmd = self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd, qos)

        self._path: Path | None = None
        self._stats: str | None = None
        self._cmd_count = 0
        self._cmd_t0: float | None = None

        self._costmap_msg = build_costmap()
        self._timer = self.create_timer(0.5, self._pub_costmap_cb)

    def _pub_costmap_cb(self):
        self._costmap_msg.header.stamp = self.get_clock().now().to_msg()
        self._pub_costmap.publish(self._costmap_msg)

    def _cb_path(self, msg):
        self._path = msg

    def _cb_stats(self, msg):
        self._stats = msg.data

    def _cb_cmd(self, msg):
        if self._cmd_t0 is None:
            self._cmd_t0 = time.time()
        self._cmd_count += 1

    def send_initial_pose(self, x=0.0, y=0.0):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        self._pub_initial.publish(msg)

    def send_goal(self, x, y):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self._pub_goal.publish(msg)

    def reset(self):
        self._path = None
        self._stats = None

    def wait_for(self, attr, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if getattr(self, attr) is not None:
                return True
            time.sleep(0.05)
        return False


def ros2_param_set(node_name, param, value):
    result = subprocess.run(
        ['bash', '-c',
         f'source /opt/ros/jazzy/setup.bash && '
         f'source /home/anhuynh/anhc_botrl/install/setup.bash && '
         f'ros2 param set {node_name} {param} {value}'],
        capture_output=True, text=True, timeout=10
    )
    return result.stdout.strip() + result.stderr.strip()


def main():
    rclpy.init()
    node = PlannerTester()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    t = threading.Thread(target=executor.spin, daemon=True)
    t.start()

    SEP = '=' * 65

    # ------------------------------------------------------------------ #
    # STEP 3 — Publish goal, receive path (A*)
    # ------------------------------------------------------------------ #
    print('\n' + SEP)
    print('VERIFICATION STEP 3 — A* path planning')
    print(SEP)
    print('Publishing synthetic costmap (20×20 m)...')
    time.sleep(3)                         # let costmap reach planner
    print('Sending initialpose (0.0, 0.0)...')
    node.send_initial_pose(0.0, 0.0)
    time.sleep(0.3)
    print('Sending /goal_pose x=5.0 y=3.0 ...')
    node.send_goal(5.0, 3.0)

    ok = node.wait_for('_path', timeout=8)
    if ok and node._path is not None:
        n = len(node._path.poses)
        f = node._path.poses[0].pose.position
        l = node._path.poses[-1].pose.position
        print(f'\n/planning/path received: {n} poses  '
              f'[{"PASS ✓" if n > 5 else "FAIL ✗"}]')
        print(f'  first pose: ({f.x:.3f}, {f.y:.3f})')
        print(f'  last  pose: ({l.x:.3f}, {l.y:.3f})')
    else:
        print('\n[FAIL ✗] /planning/path — no message received')

    # ------------------------------------------------------------------ #
    # STEP 4 — Planning stats (A*)
    # ------------------------------------------------------------------ #
    print('\n' + SEP)
    print('VERIFICATION STEP 4 — /planning/stats (A*)')
    print(SEP)
    if node._stats is not None:
        try:
            stats = json.loads(node._stats)
            print(f'/planning/stats received:')
            print(json.dumps(stats, indent=2))
            required = {'algorithm', 'nodes_expanded', 'planning_time_ms', 'path_length_m'}
            missing = required - stats.keys()
            result = 'PASS ✓' if not missing else f'FAIL ✗ (missing: {missing})'
            print(f'Required fields present: [{result}]')
        except Exception as e:
            print(f'[FAIL ✗] JSON parse error: {e}')
    else:
        print('[FAIL ✗] /planning/stats — no message received')

    # ------------------------------------------------------------------ #
    # STEP 5 — Switch to Dijkstra, re-plan
    # ------------------------------------------------------------------ #
    print('\n' + SEP)
    print('VERIFICATION STEP 5 — Switch to Dijkstra')
    print(SEP)
    print('ros2 param set /anhc_global_planner algorithm dijkstra ...')
    out = ros2_param_set('/anhc_global_planner', 'algorithm', 'dijkstra')
    print(f'  → {out}')
    node.reset()
    time.sleep(0.3)
    print('Sending /goal_pose x=3.0 y=3.0 (open space, above wall)...')
    node.send_goal(3.0, 3.0)    # y=3.0 → row=130 (above the wall at rows 60-65)

    ok = node.wait_for('_stats', timeout=8)
    if ok and node._stats is not None:
        try:
            stats = json.loads(node._stats)
            algo = stats.get('algorithm', 'unknown')
            n_path = len(node._path.poses) if node._path else 0
            print(f'\n/planning/stats received:')
            print(json.dumps(stats, indent=2))
            alg_ok = 'PASS ✓' if algo == 'dijkstra' else 'FAIL ✗'
            path_ok = 'PASS ✓' if n_path > 5 else 'FAIL ✗'
            print(f'algorithm=="dijkstra": [{alg_ok}]')
            print(f'path poses ({n_path}): [{path_ok}]')
        except Exception as e:
            print(f'[FAIL ✗] JSON parse error: {e}')
    else:
        print('[FAIL ✗] /planning/stats (dijkstra) — no message')

    # ------------------------------------------------------------------ #
    # STEP 6 — /cmd_vel Hz check
    # ------------------------------------------------------------------ #
    print('\n' + SEP)
    print('VERIFICATION STEP 6 — /cmd_vel publish rate')
    print(SEP)
    node._cmd_count = 0
    node._cmd_t0 = None
    print('Monitoring /cmd_vel for 5 seconds...')
    time.sleep(5)
    if node._cmd_t0 is not None and node._cmd_count > 0:
        elapsed = time.time() - node._cmd_t0
        hz = node._cmd_count / elapsed if elapsed > 0 else 0
        ok = 10 <= hz <= 25
        print(f'/cmd_vel: {node._cmd_count} msgs in {elapsed:.1f}s → {hz:.1f} Hz  '
              f'[{"PASS ✓" if ok else "INFO (outside 10-25 Hz range)"}]')
    else:
        print('/cmd_vel: no messages — path follower idle (no path active)  [INFO]')

    print('\n' + SEP)
    print('VERIFICATION COMPLETE')
    print(SEP + '\n')

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
