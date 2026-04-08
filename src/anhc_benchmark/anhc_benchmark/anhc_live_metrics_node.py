"""Live metrics publisher node for the anhc autonomous vehicle benchmark.

Subscribes to odometry, planning path, and planning stats; derives real-time
metrics (path deviation, current speed, estimated arrival time) and publishes
them as JSON on /benchmark/live at 2 Hz.
"""

import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


class AnhcLiveMetricsNode(Node):
    """Real-time benchmark metrics publisher.

    Topics
    ------
    Subscriptions:
        /planning/stats    — std_msgs/String (JSON)
        /planning/path     — nav_msgs/Path
        /odometry/filtered — nav_msgs/Odometry

    Publications:
        /benchmark/live    — std_msgs/String (JSON, 2 Hz)

    Published JSON fields
    ---------------------
    path_deviation   : float  — min distance from robot to nearest path point (m)
    current_speed    : float  — robot speed magnitude (m/s)
    eta_s            : float  — estimated seconds to goal; -1 if not computable
    path_length_m    : float  — total planned path length (m); 0 if unknown
    nodes_expanded   : int    — planner nodes_expanded from last stats
    planning_time_ms : float  — planner planning_time_ms from last stats
    algorithm        : str    — active algorithm name
    timestamp        : float  — Unix epoch of this sample
    """

    def __init__(self) -> None:
        super().__init__("anhc_live_metrics")

        self._path: list[tuple[float, float]] = []
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_speed: float = 0.0
        self._last_stats: dict = {}

        self.create_subscription(String, "/planning/stats", self._cb_stats, 10)
        self.create_subscription(Path, "/planning/path", self._cb_path, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._cb_odom, 10)

        self._pub = self.create_publisher(String, "/benchmark/live", 10)
        self.create_timer(0.5, self._publish_metrics)  # 2 Hz

        self.get_logger().info("[anhc_live_metrics] started — publishing at 2 Hz")

    # ──────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_stats(self, msg: String) -> None:
        try:
            self._last_stats = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_path(self, msg: Path) -> None:
        self._path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def _cb_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._robot_speed = math.hypot(vx, vy)

    # ──────────────────────────────────────────────────────────────────────────
    # Timer callback
    # ──────────────────────────────────────────────────────────────────────────

    def _publish_metrics(self) -> None:
        deviation = self._compute_path_deviation()
        eta = self._compute_eta()

        payload = {
            "path_deviation": round(deviation, 4),
            "current_speed": round(self._robot_speed, 4),
            "eta_s": round(eta, 2),
            "path_length_m": float(self._last_stats.get("path_length_m", 0.0)),
            "nodes_expanded": int(self._last_stats.get("nodes_expanded", 0)),
            "planning_time_ms": float(
                self._last_stats.get("planning_time_ms", 0.0)
            ),
            "algorithm": str(self._last_stats.get("algorithm", "unknown")),
            "timestamp": time.time(),
        }
        self._pub.publish(String(data=json.dumps(payload)))

    # ──────────────────────────────────────────────────────────────────────────
    # Metric computations
    # ──────────────────────────────────────────────────────────────────────────

    def _compute_path_deviation(self) -> float:
        """Minimum Euclidean distance from robot to any path waypoint (m)."""
        if not self._path:
            return 0.0
        return min(
            math.hypot(px - self._robot_x, py - self._robot_y)
            for px, py in self._path
        )

    def _compute_eta(self) -> float:
        """Estimated seconds to goal along remaining path; -1 if not computable."""
        if not self._path:
            return -1.0

        # Find index of closest waypoint
        min_idx = 0
        min_dist = float("inf")
        for i, (px, py) in enumerate(self._path):
            d = math.hypot(px - self._robot_x, py - self._robot_y)
            if d < min_dist:
                min_dist = d
                min_idx = i

        # Remaining arc length from closest waypoint to end
        remaining = 0.0
        for i in range(min_idx, len(self._path) - 1):
            px1, py1 = self._path[i]
            px2, py2 = self._path[i + 1]
            remaining += math.hypot(px2 - px1, py2 - py1)

        if self._robot_speed < 0.01:
            return -1.0

        return remaining / self._robot_speed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnhcLiveMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
