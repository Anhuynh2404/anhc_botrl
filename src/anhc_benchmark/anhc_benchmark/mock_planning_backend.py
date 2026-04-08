"""Lightweight mock planning backend for sandbox / CI verification.

Mimics the planning stack without Gazebo:
  - Publishes /costmap/global (empty 20×20 grid)
  - On /goal_pose, publishes /planning/path + /planning/stats
  - Drives /odometry/filtered so the robot appears to reach the goal
  - Publishes /perception/depth_obstacles False throughout

NOT installed as a ROS entry point; used only via ``python3 mock_planning_backend.py``.
"""

import json
import math
import random
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String


class MockPlanningBackend(Node):
    def __init__(self) -> None:
        super().__init__("mock_planning_backend")

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._goal_x: float | None = None
        self._goal_y: float | None = None
        self._moving = False

        self._pub_path = self.create_publisher(Path, "/planning/path", 10)
        self._pub_stats = self.create_publisher(String, "/planning/stats", 10)
        self._pub_odom = self.create_publisher(Odometry, "/odometry/filtered", 10)
        self._pub_costmap = self.create_publisher(OccupancyGrid, "/costmap/global", 1)
        self._pub_obstacle = self.create_publisher(Bool, "/perception/depth_obstacles", 10)

        self.create_subscription(PoseStamped, "/goal_pose", self._cb_goal, 10)

        self.create_timer(0.1, self._publish_odom)
        self.create_timer(1.0, self._publish_costmap)
        self.create_timer(0.5, self._publish_obstacle)

        self.get_logger().info("[mock_backend] started")

    def _cb_goal(self, msg: PoseStamped) -> None:
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self._goal_x = gx
        self._goal_y = gy
        algo = getattr(self, "_last_algo", "astar")

        # Build straight-line path
        steps = 20
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        pts: list[tuple[float, float]] = []
        for i in range(steps + 1):
            t = i / steps
            x = self._robot_x + t * (gx - self._robot_x)
            y = self._robot_y + t * (gy - self._robot_y)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
            pts.append((x, y))

        length = math.hypot(gx - self._robot_x, gy - self._robot_y)
        stats = {
            "algorithm": algo,
            "nodes_expanded": random.randint(150, 900),
            "planning_time_ms": round(random.uniform(8.0, 45.0), 2),
            "path_length_m": round(length * random.uniform(1.0, 1.15), 3),
            "status": "success",
        }

        time.sleep(0.05)  # tiny planning delay
        self._pub_path.publish(path_msg)
        self._pub_stats.publish(String(data=json.dumps(stats)))
        self._moving = True
        self.get_logger().info(
            f"[mock_backend] goal ({gx:.1f},{gy:.1f}) → path+stats published"
        )

    def _publish_odom(self) -> None:
        if self._moving and self._goal_x is not None:
            dx = self._goal_x - self._robot_x
            dy = self._goal_y - self._robot_y
            dist = math.hypot(dx, dy)
            step = min(0.08, dist)
            if dist > 0.05:
                self._robot_x += step * dx / dist
                self._robot_y += step * dy / dist
            else:
                self._moving = False

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_footprint"
        msg.pose.pose.position.x = self._robot_x
        msg.pose.pose.position.y = self._robot_y
        msg.pose.pose.orientation.w = 1.0
        msg.twist.twist.linear.x = 0.4 if self._moving else 0.0
        self._pub_odom.publish(msg)

    def _publish_costmap(self) -> None:
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = 0.05
        msg.info.width = 400
        msg.info.height = 400
        msg.info.origin.position.x = -10.0
        msg.info.origin.position.y = -10.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0] * (400 * 400)
        self._pub_costmap.publish(msg)

    def _publish_obstacle(self) -> None:
        self._pub_obstacle.publish(Bool(data=False))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockPlanningBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
