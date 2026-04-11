"""Pure Pursuit path follower node for the anhc autonomous vehicle.

Diagnosis of prior implementation:
- BUG D present: control used /odometry/filtered (odom frame) directly against
  /planning/path points (map frame), so frame mismatch caused path divergence.
- BUG B present: angular command lacked PID derivative stabilization, resulting
  in oscillatory angular corrections around heading targets.
"""

import math
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from tf2_ros import ExtrapolationException, LookupException, TransformException


class AnhcPathFollowerNode(Node):
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

        self._ang_integral = 0.0
        self._prev_ang_error = 0.0
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0
        self._last_tf_warn_sec = -1.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, "/planning/path", self._cb_path, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._cb_odom, 10)
        self.create_subscription(Bool, "/perception/depth_obstacles", self._cb_obstacle, 10)

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_replan = self.create_publisher(Bool, "/planning/replan_request", 10)
        self._pub_lookahead = self.create_publisher(
            PointStamped, "/planning/debug/lookahead_point", 10
        )
        self._pub_xte = self.create_publisher(
            Float32, "/planning/debug/cross_track_error", 10
        )
        self._pub_pid = self.create_publisher(
            Float32, "/planning/debug/pid_output", 10
        )

        self._control_dt = 0.05
        self._timer = self.create_timer(self._control_dt, self._control_loop)

        self.get_logger().info("[anhc_path_follower] started")

    def _cb_path(self, msg: Path) -> None:
        self._path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._path_idx = 0
        self.get_logger().info(
            f"[anhc_path_follower] received path with {len(self._path_map)} poses"
        )

    def _cb_odom(self, msg: Odometry) -> None:
        self._latest_angular_vel = msg.twist.twist.angular.z

    def _cb_obstacle(self, msg: Bool) -> None:
        self._obstacle_stop = msg.data
        if msg.data:
            self.get_logger().warn("[anhc_path_follower] obstacle detected — stopping")

    def _control_loop(self) -> None:
        if self._obstacle_stop or self._goal_reached:
            self._safe_stop()
            return

        # Snapshot so /planning/path callbacks cannot shorten the list mid-tick
        # (IndexError on lookahead_idx vs multi-threaded executor).
        path_map = list(self._path_map)
        if not path_map:
            self._safe_stop()
            return

        tf = self._lookup_tf("base_footprint", self._path_frame)
        if tf is None:
            self._safe_stop()
            return

        robot_map_x, robot_map_y, robot_map_yaw = self._robot_pose_in_map(tf)
        goal_x, goal_y = path_map[-1]
        goal_dist = math.hypot(goal_x - robot_map_x, goal_y - robot_map_y)
        if goal_dist < float(self.get_parameter("goal_tolerance_m").value):
            self.get_logger().info("[anhc_path_follower] Goal reached")
            self._goal_reached = True
            self._path_map = []
            self._safe_stop()
            return

        path_robot = [
            self._map_to_robot(px, py, robot_map_x, robot_map_y, robot_map_yaw)
            for px, py in path_map
        ]
        closest_idx = self._closest_index(path_robot)
        lookahead = float(self.get_parameter("lookahead_distance").value)
        lookahead_idx = self._lookahead_index(path_robot, closest_idx, lookahead)
        lx, ly = path_robot[lookahead_idx]
        lookahead_map = path_map[lookahead_idx]

        self._pub_lookahead.publish(
            PointStamped(
                header=self._make_header(self._path_frame),
                point=self._to_point(*lookahead_map),
            )
        )

        xte = self._cross_track_error(robot_map_x, robot_map_y, path_map)
        self._pub_xte.publish(Float32(data=float(xte)))
        if xte > float(self.get_parameter("path_lost_threshold").value):
            self.get_logger().warn(
                "[anhc_path_follower] Path lost — cross-track error too large, waiting for replan"
            )
            self._pub_replan.publish(Bool(data=True))
            self._safe_stop()
            return

        max_v = float(self.get_parameter("max_linear_speed").value)
        min_v = float(self.get_parameter("min_linear_speed").value)
        max_w = float(self.get_parameter("max_angular_speed").value)
        alpha = float(self.get_parameter("cmd_smoothing_alpha").value)
        alpha = max(0.0, min(1.0, alpha))

        # Pure Pursuit curvature in robot frame
        denom = max(lookahead * lookahead, 1e-6)
        kappa = 2.0 * ly / denom

        linear_vel = self._clamp(
            max_v * (1.0 - abs(kappa) * 0.5), min_v, max_v
        )
        target_angular = self._clamp(linear_vel * kappa, -max_w, max_w)

        # PID on angular velocity tracking
        kp = float(self.get_parameter("angular_Kp").value)
        ki = float(self.get_parameter("angular_Ki").value)
        kd = float(self.get_parameter("angular_Kd").value)
        angular_error = target_angular - self._latest_angular_vel
        self._ang_integral += angular_error * self._dt
        deriv = (angular_error - self._prev_ang_error) / max(self._dt, 1e-6)
        angular_pid = kp * angular_error + ki * self._ang_integral + kd * deriv
        angular_pid = self._clamp(angular_pid, -max_w, max_w)
        self._prev_ang_error = angular_error
        self._pub_pid.publish(Float32(data=float(angular_pid)))

        sm_linear = alpha * linear_vel + (1.0 - alpha) * self._prev_linear_cmd
        sm_angular = alpha * angular_pid + (1.0 - alpha) * self._prev_angular_cmd
        sm_linear = self._clamp(sm_linear, 0.0, max_v)
        sm_angular = self._clamp(sm_angular, -max_w, max_w)

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
        self._prev_linear_cmd = cmd.linear.x
        self._prev_angular_cmd = cmd.angular.z

    def _lookup_tf(self, target: str, source: str):
        try:
            timeout = Duration(seconds=float(self.get_parameter("transform_timeout").value))
            return self._tf_buffer.lookup_transform(target, source, rclpy.time.Time(), timeout)
        except (TransformException, LookupException, ExtrapolationException):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_tf_warn_sec >= 1.0:
                self.get_logger().warn(
                    f"[anhc_path_follower] TF lookup failed: {source} -> {target}"
                )
                self._last_tf_warn_sec = now
            return None

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

    def _safe_stop(self) -> None:
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0
        self._pub_cmd.publish(Twist())

    def _make_header(self, frame_id: str):
        from std_msgs.msg import Header

        return Header(stamp=self.get_clock().now().to_msg(), frame_id=frame_id)

    @staticmethod
    def _to_point(x: float, y: float):
        from geometry_msgs.msg import Point

        p = Point()
        p.x = x
        p.y = y
        p.z = 0.0
        return p

    @staticmethod
    def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    @staticmethod
    def _clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))


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
