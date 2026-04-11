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
        self.declare_parameter("max_linear_speed", 0.4)
        self.declare_parameter("min_linear_speed", 0.05)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("lookahead_distance", 0.6)
        self.declare_parameter("goal_tolerance_m", 0.20)
        self.declare_parameter("cmd_smoothing_alpha", 0.4)
        self.declare_parameter("angular_Kp", 1.2)
        self.declare_parameter("angular_Ki", 0.01)
        self.declare_parameter("angular_Kd", 0.3)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("transform_timeout", 0.1)
        self.declare_parameter("path_lost_threshold", 1.5)

        self._path_map: list[tuple[float, float]] = []
        self._path_frame: str = "map"
        self._goal_reached = False
        self._latest_angular_vel = 0.0

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

        hz = max(1.0, float(self.get_parameter("control_frequency").value))
        self._dt = 1.0 / hz
        self._obstacle_stop = False
        self._timer = self.create_timer(self._dt, self._control_loop)
        self.get_logger().info("[anhc_path_follower] started")

    def _cb_path(self, msg: Path) -> None:
        self._path_frame = msg.header.frame_id if msg.header.frame_id else "map"
        self._path_map = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._goal_reached = False
        self._ang_integral = 0.0
        self._prev_ang_error = 0.0
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
        if self._obstacle_stop or not self._path_map or self._goal_reached:
            self._safe_stop()
            return

        tf = self._lookup_tf("base_footprint", self._path_frame)
        if tf is None:
            self._safe_stop()
            return

        robot_map_x, robot_map_y, robot_map_yaw = self._robot_pose_in_map(tf)
        goal_x, goal_y = self._path_map[-1]
        goal_dist = math.hypot(goal_x - robot_map_x, goal_y - robot_map_y)
        if goal_dist < float(self.get_parameter("goal_tolerance_m").value):
            self.get_logger().info("[anhc_path_follower] Goal reached")
            self._goal_reached = True
            self._path_map = []
            self._safe_stop()
            return

        path_robot = [
            self._map_to_robot(px, py, robot_map_x, robot_map_y, robot_map_yaw)
            for px, py in self._path_map
        ]
        closest_idx = self._closest_index(path_robot)
        lookahead = float(self.get_parameter("lookahead_distance").value)
        lookahead_idx = self._lookahead_index(path_robot, closest_idx, lookahead)
        lx, ly = path_robot[lookahead_idx]
        lookahead_map = self._path_map[lookahead_idx]

        self._pub_lookahead.publish(
            PointStamped(
                header=self._make_header(self._path_frame),
                point=self._to_point(*lookahead_map),
            )
        )

        xte = self._cross_track_error(robot_map_x, robot_map_y)
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
        cmd.linear.x = float(sm_linear)
        cmd.angular.z = float(sm_angular)
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

    def _robot_pose_in_map(self, tf_base_from_map) -> tuple[float, float, float]:
        # tf: target=base_footprint, source=map. Invert to map<-base.
        t = tf_base_from_map.transform.translation
        q = tf_base_from_map.transform.rotation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        c = math.cos(yaw)
        s = math.sin(yaw)
        bx = -(c * t.x + s * t.y)
        by = -(-s * t.x + c * t.y)
        return bx, by, -yaw

    def _map_to_robot(
        self, px: float, py: float, rx: float, ry: float, ryaw: float
    ) -> tuple[float, float]:
        dx = px - rx
        dy = py - ry
        c = math.cos(-ryaw)
        s = math.sin(-ryaw)
        return (c * dx - s * dy, s * dx + c * dy)

    def _closest_index(self, path_robot: list[tuple[float, float]]) -> int:
        best_i = 0
        best_d = float("inf")
        for i, (x, y) in enumerate(path_robot):
            d = x * x + y * y
            if d < best_d:
                best_d = d
                best_i = i
        return best_i

    def _lookahead_index(
        self, path_robot: list[tuple[float, float]], start_idx: int, lookahead: float
    ) -> int:
        for i in range(start_idx, len(path_robot)):
            x, y = path_robot[i]
            if math.hypot(x, y) >= lookahead:
                return i
        return len(path_robot) - 1

    def _cross_track_error(self, rx: float, ry: float) -> float:
        if len(self._path_map) < 2:
            return math.hypot(self._path_map[0][0] - rx, self._path_map[0][1] - ry)
        best = float("inf")
        for i in range(len(self._path_map) - 1):
            ax, ay = self._path_map[i]
            bx, by = self._path_map[i + 1]
            vx, vy = bx - ax, by - ay
            wx, wy = rx - ax, ry - ay
            v2 = vx * vx + vy * vy
            if v2 < 1e-9:
                d = math.hypot(rx - ax, ry - ay)
            else:
                t = self._clamp((wx * vx + wy * vy) / v2, 0.0, 1.0)
                px = ax + t * vx
                py = ay + t * vy
                d = math.hypot(rx - px, ry - py)
            best = min(best, d)
        return best

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
