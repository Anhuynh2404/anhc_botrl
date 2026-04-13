"""Pure Pursuit path follower for the anhc autonomous vehicle.

Plans are in ``map`` frame; control uses ``map``→``base_footprint`` TF and
publishes ``Twist`` in the robot base frame on ``/cmd_vel``.
"""

from __future__ import annotations

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32
from tf2_ros import ExtrapolationException, LookupException, TransformException

# Match anhc_cmd_vel_idle_gate + ros_gz_bridge (reliable, sufficient depth).
_CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
)


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
        self.declare_parameter("path_lost_threshold", 0.85)
        self.declare_parameter("cmd_smoothing_alpha", 0.35)
        self.declare_parameter("angular_Kp", 1.2)
        self.declare_parameter("angular_Ki", 0.02)
        self.declare_parameter("angular_Kd", 0.08)
        self.declare_parameter("transform_timeout", 0.35)
        self.declare_parameter("control_frequency", 20.0)

        self._path: list[tuple[float, float]] = []
        self._path_frame: str = "map"
        self._path_idx: int = 0
        self._goal_reached: bool = False
        self._obstacle_stop: bool = False
        self._last_linear_cmd: float = 0.0
        self._last_angular_cmd: float = 0.0
        self._latest_angular_vel: float = 0.0

        self._ang_integral: float = 0.0
        self._prev_ang_error: float = 0.0
        self._prev_linear_cmd: float = 0.0
        self._prev_angular_cmd: float = 0.0
        self._last_tf_warn_sec: float = -1.0

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.create_subscription(Path, "/planning/path", self._cb_path, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._cb_odom, 10)
        self.create_subscription(Bool, "/perception/depth_obstacles", self._cb_obstacle, 10)

        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", _CMD_VEL_QOS)
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

        freq = float(
            self.get_parameter("control_frequency").get_parameter_value().double_value
        )
        self._control_dt = 1.0 / max(freq, 1.0)
        self._timer = self.create_timer(self._control_dt, self._control_loop)

        self.get_logger().info(
            f"[anhc_path_follower] started (dt={self._control_dt:.4f}s)"
        )

    def _cb_path(self, msg: Path) -> None:
        self._path = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]
        self._path_frame = msg.header.frame_id if msg.header.frame_id else "map"
        self._path_idx = 0
        self._goal_reached = False
        self._ang_integral = 0.0
        self._prev_ang_error = 0.0
        self.get_logger().info(
            f"[anhc_path_follower] received path with {len(self._path)} poses "
            f"(frame={self._path_frame})"
        )

    def _cb_odom(self, msg: Odometry) -> None:
        self._latest_angular_vel = msg.twist.twist.angular.z

    def _cb_obstacle(self, msg: Bool) -> None:
        self._obstacle_stop = msg.data
        if msg.data:
            self.get_logger().warn("[anhc_path_follower] obstacle detected — stopping")

    def _control_loop(self) -> None:
        if self._obstacle_stop:
            self._safe_stop()
            return

        if self._goal_reached:
            self._safe_stop()
            return

        path_map = list(self._path)
        if not path_map:
            self._safe_stop()
            return

        tf = self._lookup_tf(self._path_frame, "base_footprint")
        if tf is None:
            self._safe_stop()
            return

        robot_map_x, robot_map_y, robot_map_yaw = self._robot_pose_in_map(tf)
        gx, gy = path_map[-1]
        goal_dist = math.hypot(gx - robot_map_x, gy - robot_map_y)
        goal_tol = float(
            self.get_parameter("goal_tolerance_m").get_parameter_value().double_value
        )
        if goal_dist < goal_tol:
            self.get_logger().info("[anhc_path_follower] goal reached")
            self._goal_reached = True
            self._path.clear()
            self._safe_stop()
            return

        path_robot = [
            self._map_to_robot(px, py, robot_map_x, robot_map_y, robot_map_yaw)
            for px, py in path_map
        ]
        closest_idx = self._closest_index(path_robot)
        lookahead_m = float(
            self.get_parameter("lookahead_distance").get_parameter_value().double_value
        )
        lookahead_idx = self._lookahead_index(path_robot, closest_idx, lookahead_m)
        lookahead_idx = self._ensure_lookahead_ahead(path_robot, lookahead_idx)
        lx, ly = path_robot[lookahead_idx]
        lookahead_map_x, lookahead_map_y = path_map[lookahead_idx]

        self._pub_lookahead.publish(
            PointStamped(
                header=self._make_header(self._path_frame),
                point=self._to_point(lookahead_map_x, lookahead_map_y),
            )
        )

        xte = self._cross_track_error(robot_map_x, robot_map_y, path_map)
        self._pub_xte.publish(Float32(data=float(xte)))
        lost_thr = float(
            self.get_parameter("path_lost_threshold").get_parameter_value().double_value
        )
        if xte > lost_thr:
            self.get_logger().warn(
                "[anhc_path_follower] path lost — cross-track error too large, replan"
            )
            self._pub_replan.publish(Bool(data=True))
            self._safe_stop()
            return

        max_v = float(
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        min_v = float(
            self.get_parameter("min_linear_speed").get_parameter_value().double_value
        )
        max_w = float(
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )
        csg = float(
            self.get_parameter("curvature_slowdown_gain").get_parameter_value().double_value
        )
        csg = max(0.0, min(csg, 2.0))
        rip_rad = float(
            self.get_parameter("rotate_in_place_angle_rad").get_parameter_value().double_value
        )
        rip_w = float(
            self.get_parameter("rotate_in_place_angular_speed").get_parameter_value().double_value
        )
        rip_w = min(rip_w, max_w)
        alpha = float(
            self.get_parameter("cmd_smoothing_alpha").get_parameter_value().double_value
        )
        alpha = max(0.0, min(1.0, alpha))

        heading_to_la = math.atan2(ly, lx)
        if abs(heading_to_la) > rip_rad:
            # Face the lookahead before driving; avoids crawling forward while fighting lateral error.
            linear_vel = 0.0
            k_head = 2.2
            target_angular = self._clamp(
                k_head * heading_to_la, -rip_w, rip_w
            )
        else:
            denom = max(lookahead_m * lookahead_m, 1e-6)
            kappa = 2.0 * ly / denom
            linear_vel = max_v * (1.0 - abs(kappa) * csg)
            linear_vel = self._clamp(linear_vel, min_v, max_v)
            target_angular = self._clamp(linear_vel * kappa, -max_w, max_w)

        kp = float(self.get_parameter("angular_Kp").get_parameter_value().double_value)
        ki = float(self.get_parameter("angular_Ki").get_parameter_value().double_value)
        kd = float(self.get_parameter("angular_Kd").get_parameter_value().double_value)
        angular_error = target_angular - self._latest_angular_vel
        self._ang_integral += angular_error * self._control_dt
        deriv = (angular_error - self._prev_ang_error) / max(self._control_dt, 1e-6)
        angular_pid = kp * angular_error + ki * self._ang_integral + kd * deriv
        angular_pid = self._clamp(angular_pid, -max_w, max_w)
        self._prev_ang_error = angular_error
        self._pub_pid.publish(Float32(data=float(angular_pid)))

        sm_linear = alpha * linear_vel + (1.0 - alpha) * self._prev_linear_cmd
        sm_angular = alpha * angular_pid + (1.0 - alpha) * self._prev_angular_cmd
        sm_linear = self._clamp(sm_linear, 0.0, max_v)
        sm_angular = self._clamp(sm_angular, -max_w, max_w)

        max_lin_acc = float(
            self.get_parameter("max_linear_accel").get_parameter_value().double_value
        )
        max_ang_acc = float(
            self.get_parameter("max_angular_accel").get_parameter_value().double_value
        )
        lin_step = max_lin_acc * self._control_dt
        ang_step = max_ang_acc * self._control_dt

        linear_cmd = self._last_linear_cmd + self._clamp(
            sm_linear - self._last_linear_cmd, -lin_step, lin_step
        )
        angular_cmd = self._last_angular_cmd + self._clamp(
            sm_angular - self._last_angular_cmd, -ang_step, ang_step
        )

        cmd = Twist()
        cmd.linear.x = float(max(0.0, min(linear_cmd, max_v)))
        cmd.angular.z = float(max(-max_w, min(angular_cmd, max_w)))
        self._last_linear_cmd = cmd.linear.x
        self._last_angular_cmd = cmd.angular.z
        self._prev_linear_cmd = cmd.linear.x
        self._prev_angular_cmd = cmd.angular.z
        self._pub_cmd.publish(cmd)

    def _lookup_tf(self, target_frame: str, source_frame: str):
        try:
            timeout = Duration(
                seconds=float(
                    self.get_parameter("transform_timeout").get_parameter_value().double_value
                )
            )
            return self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout
            )
        except (TransformException, LookupException, ExtrapolationException):
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_tf_warn_sec >= 1.0:
                self.get_logger().warn(
                    f"[anhc_path_follower] TF lookup failed: {source_frame} → {target_frame}"
                )
                self._last_tf_warn_sec = now
            return None

    @staticmethod
    def _robot_pose_in_map(tf) -> tuple[float, float, float]:
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = AnhcPathFollowerNode._quat_to_yaw(q.x, q.y, q.z, q.w)
        return (t.x, t.y, yaw)

    @staticmethod
    def _map_to_robot(
        px: float,
        py: float,
        rx: float,
        ry: float,
        yaw: float,
    ) -> tuple[float, float]:
        dx, dy = px - rx, py - ry
        c, s = math.cos(yaw), math.sin(yaw)
        xr = c * dx + s * dy
        yr = -s * dx + c * dy
        return (xr, yr)

    @staticmethod
    def _closest_index(path_robot: list[tuple[float, float]]) -> int:
        best_i, best_d = 0, float("inf")
        for i, (x, y) in enumerate(path_robot):
            d = math.hypot(x, y)
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    @staticmethod
    def _lookahead_index(
        path_robot: list[tuple[float, float]],
        start_i: int,
        lookahead: float,
    ) -> int:
        for i in range(start_i, len(path_robot)):
            x, y = path_robot[i]
            if math.hypot(x, y) >= lookahead:
                return i
        return len(path_robot) - 1

    @staticmethod
    def _ensure_lookahead_ahead(
        path_robot: list[tuple[float, float]],
        idx: int,
        ahead_eps: float = 0.12,
    ) -> int:
        """Advance along the path if the chosen lookahead is behind the robot (xr < 0).

        Pure pursuit can otherwise pick a waypoint with negative ``x`` in the base frame
        (still beyond ``lookahead`` distance), yielding heading ≈ ±π and poor motion.
        """
        i = idx
        while i < len(path_robot) - 1 and path_robot[i][0] < ahead_eps:
            i += 1
        return i

    @staticmethod
    def _cross_track_error(
        rx: float, ry: float, path_map: list[tuple[float, float]]
    ) -> float:
        if len(path_map) < 2:
            if not path_map:
                return 0.0
            return math.hypot(rx - path_map[0][0], ry - path_map[0][1])

        best = float("inf")
        for i in range(len(path_map) - 1):
            x1, y1 = path_map[i]
            x2, y2 = path_map[i + 1]
            d = AnhcPathFollowerNode._point_segment_distance(rx, ry, x1, y1, x2, y2)
            if d < best:
                best = d
        return best

    @staticmethod
    def _point_segment_distance(
        px: float, py: float, ax: float, ay: float, bx: float, by: float
    ) -> float:
        abx, aby = bx - ax, by - ay
        apx, apy = px - ax, py - ay
        ab2 = abx * abx + aby * aby
        if ab2 < 1e-12:
            return math.hypot(apx, apy)
        t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
        qx, qy = ax + t * abx, ay + t * aby
        return math.hypot(px - qx, py - qy)

    def _safe_stop(self) -> None:
        self._prev_linear_cmd = 0.0
        self._prev_angular_cmd = 0.0
        self._last_linear_cmd = 0.0
        self._last_angular_cmd = 0.0
        self._ang_integral = 0.0
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
