"""Pure Pursuit path follower node for the anhc autonomous vehicle."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool


class AnhcPathFollowerNode(Node):
    """Pure-pursuit path tracker.

    Topics
    ------
    Subscriptions:
        /planning/path              — nav_msgs/Path
        /odometry/filtered          — nav_msgs/Odometry
        /perception/depth_obstacles — std_msgs/Bool  (True → stop)

    Publications:
        /cmd_vel                    — geometry_msgs/Twist

    Parameters (YAML)
    -----------------
    max_linear_speed:
        Maximum forward speed (m/s).
    max_angular_speed:
        Maximum angular speed (rad/s).
    lookahead_distance:
        Pure-pursuit lookahead distance (m).
    goal_tolerance_m:
        Distance from goal at which navigation is considered complete (m).
    """

    def __init__(self) -> None:
        super().__init__("anhc_path_follower")

        self.declare_parameter("max_linear_speed", 0.5)
        self.declare_parameter("min_linear_speed", 0.08)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("lookahead_distance", 0.5)
        self.declare_parameter("goal_tolerance_m", 0.15)
        self.declare_parameter("curvature_slowdown_gain", 0.8)
        self.declare_parameter("rotate_in_place_angle_rad", 1.0)
        self.declare_parameter("rotate_in_place_angular_speed", 0.8)
        self.declare_parameter("max_linear_accel", 0.6)
        self.declare_parameter("max_angular_accel", 2.0)

        self._path: list[tuple[float, float]] = []
        self._path_idx: int = 0
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._obstacle_stop: bool = False
        self._last_linear_cmd: float = 0.0
        self._last_angular_cmd: float = 0.0

        self._sub_path = self.create_subscription(
            Path, "/planning/path", self._cb_path, 10
        )
        self._sub_odom = self.create_subscription(
            Odometry, "/odometry/filtered", self._cb_odom, 10
        )
        self._sub_obstacle = self.create_subscription(
            Bool, "/perception/depth_obstacles", self._cb_obstacle, 10
        )

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self._control_dt = 0.05
        self._timer = self.create_timer(self._control_dt, self._control_loop)

        self.get_logger().info("[anhc_path_follower] started")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_path(self, msg: Path) -> None:
        self._path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._path_idx = 0
        self.get_logger().info(
            f"[anhc_path_follower] received path with {len(self._path)} poses"
        )

    def _cb_odom(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self._robot_yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

    def _cb_obstacle(self, msg: Bool) -> None:
        self._obstacle_stop = msg.data
        if msg.data:
            self.get_logger().warn("[anhc_path_follower] obstacle detected — stopping")

    # ------------------------------------------------------------------
    # Control loop (20 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        cmd = Twist()

        if self._obstacle_stop or not self._path:
            self._last_linear_cmd = 0.0
            self._last_angular_cmd = 0.0
            self._pub_cmd.publish(cmd)
            return

        goal = self._path[-1]
        dist_to_goal = math.hypot(
            goal[0] - self._robot_x, goal[1] - self._robot_y
        )

        goal_tol = (
            self.get_parameter("goal_tolerance_m").get_parameter_value().double_value
        )
        if dist_to_goal < goal_tol:
            self.get_logger().info("[anhc_path_follower] goal reached")
            self._path = []
            self._pub_cmd.publish(cmd)
            return

        lookahead = (
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )
        target = self._find_lookahead_point(lookahead)
        if target is None:
            self._pub_cmd.publish(cmd)
            return

        max_v = (
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        min_v = (
            self.get_parameter("min_linear_speed").get_parameter_value().double_value
        )
        max_w = (
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )
        curv_gain = (
            self.get_parameter("curvature_slowdown_gain")
            .get_parameter_value()
            .double_value
        )

        dx = target[0] - self._robot_x
        dy = target[1] - self._robot_y
        angle_to_target = math.atan2(dy, dx)
        alpha = self._normalise_angle(angle_to_target - self._robot_yaw)

        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            self._pub_cmd.publish(cmd)
            return

        rotate_angle = (
            self.get_parameter("rotate_in_place_angle_rad")
            .get_parameter_value()
            .double_value
        )
        rotate_w = (
            self.get_parameter("rotate_in_place_angular_speed")
            .get_parameter_value()
            .double_value
        )

        if abs(alpha) > rotate_angle:
            desired_linear = 0.0
            desired_angular = max(-rotate_w, min(rotate_w, 1.5 * alpha))
        else:
            curvature = 2.0 * math.sin(alpha) / dist
            desired_linear = max_v / (1.0 + curv_gain * abs(curvature))
            desired_linear = max(min_v, min(desired_linear, max_v))
            desired_angular = desired_linear * curvature

        desired_linear = float(max(0.0, min(desired_linear, max_v)))
        desired_angular = float(max(-max_w, min(desired_angular, max_w)))

        max_lin_acc = (
            self.get_parameter("max_linear_accel").get_parameter_value().double_value
        )
        max_ang_acc = (
            self.get_parameter("max_angular_accel").get_parameter_value().double_value
        )
        lin_step = max_lin_acc * self._control_dt
        ang_step = max_ang_acc * self._control_dt

        linear_cmd = self._last_linear_cmd + max(
            -lin_step, min(lin_step, desired_linear - self._last_linear_cmd)
        )
        angular_cmd = self._last_angular_cmd + max(
            -ang_step, min(ang_step, desired_angular - self._last_angular_cmd)
        )

        cmd.linear.x = float(max(0.0, min(linear_cmd, max_v)))
        cmd.angular.z = float(max(-max_w, min(angular_cmd, max_w)))
        self._last_linear_cmd = cmd.linear.x
        self._last_angular_cmd = cmd.angular.z
        self._pub_cmd.publish(cmd)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_lookahead_point(
        self, lookahead: float
    ) -> tuple[float, float] | None:
        """Return a forward lookahead waypoint without backtracking oscillation."""
        if not self._path:
            return None

        self._path_idx = max(0, min(self._path_idx, len(self._path) - 1))

        # Advance path index while the next waypoint is closer.
        # This prevents switching between points behind/ahead of the robot.
        while self._path_idx + 1 < len(self._path):
            curr = self._path[self._path_idx]
            nxt = self._path[self._path_idx + 1]
            d_curr = math.hypot(curr[0] - self._robot_x, curr[1] - self._robot_y)
            d_next = math.hypot(nxt[0] - self._robot_x, nxt[1] - self._robot_y)
            if d_next <= d_curr:
                self._path_idx += 1
            else:
                break

        for i in range(self._path_idx, len(self._path)):
            wx, wy = self._path[i]
            if math.hypot(wx - self._robot_x, wy - self._robot_y) >= lookahead:
                self._path_idx = i
                return (wx, wy)

        self._path_idx = len(self._path) - 1
        return self._path[-1]

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalise_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnhcPathFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
