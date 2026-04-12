"""ROS 2 global planner node for the anhc autonomous vehicle.

Subscribes to a goal pose and the global costmap, runs the selected planning
algorithm, and publishes the resulting path together with JSON-encoded metrics.
"""

import json

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Bool, String
import tf2_ros
from tf2_ros import TransformException

from anhc_planning.planners import (
    AStarPlanner,
    BasePlanner,
    DijkstraPlanner,
    DStarLitePlanner,
    RLPlanner,
    RRTStarPlanner,
)

_PLANNER_REGISTRY = {
    "astar": AStarPlanner,
    "dijkstra": DijkstraPlanner,
    "rrt_star": RRTStarPlanner,
    "dstar_lite": DStarLitePlanner,
    "rl": RLPlanner,
}


class AnhcGlobalPlannerNode(Node):
    """Global path planner node.

    Topics
    ------
    Subscriptions:
        /costmap/global        — nav_msgs/OccupancyGrid (preferred when available)
        /map                   — nav_msgs/OccupancyGrid (fallback / static map)
        /goal_pose             — geometry_msgs/PoseStamped
        /initialpose           — geometry_msgs/PoseWithCovarianceStamped

    Publications:
        /planning/path         — nav_msgs/Path
        /planning/stats        — std_msgs/String  (JSON)

    Parameters
    ----------
    algorithm : str
        Active planner key.  One of: astar, dijkstra, rrt_star, dstar_lite, rl.
    """

    def __init__(self) -> None:
        super().__init__("anhc_global_planner")

        self.declare_parameter(
            "algorithm",
            "astar",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Planning algorithm. Options: astar, dijkstra, rrt_star, dstar_lite, rl",
            ),
        )
        self.declare_parameter(
            "obstacle_cost_threshold",
            253,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Cells with cost >= threshold are treated as lethal.",
            ),
        )
        self.declare_parameter("path_smooth_data_weight", 0.5)
        self.declare_parameter("path_smooth_smooth_weight", 0.3)
        self.declare_parameter("path_smooth_min_waypoints", 5)

        self._costmap: OccupancyGrid | None = None
        self._have_global_costmap: bool = False
        self._manual_start: tuple[float, float] | None = None
        self._last_goal_msg: PoseStamped | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._sub_costmap = self.create_subscription(
            OccupancyGrid, "/costmap/global", self._cb_global_costmap, 1
        )
        self._sub_map = self.create_subscription(
            OccupancyGrid, "/map", self._cb_map, 1
        )
        self._sub_goal = self.create_subscription(
            PoseStamped, "/goal_pose", self._cb_goal, 10
        )
        self._sub_initial = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._cb_initial_pose, 10
        )
        self._sub_replan = self.create_subscription(
            Bool, "/planning/replan_request", self._cb_replan_request, 10
        )

        self._pub_path = self.create_publisher(Path, "/planning/path", 10)
        self._pub_stats = self.create_publisher(String, "/planning/stats", 10)

        self._planner: BasePlanner = self._build_planner()
        self.get_logger().info(
            f"[anhc_global_planner] started — algorithm: {self._planner.get_name()}"
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _cb_global_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg
        self._have_global_costmap = True

    def _cb_map(self, msg: OccupancyGrid) -> None:
        if self._have_global_costmap:
            return
        self._costmap = msg

    def _cb_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        pos = msg.pose.pose.position
        self._manual_start = (pos.x, pos.y)
        self.get_logger().info(
            f"[anhc_global_planner] initial pose set: ({pos.x:.2f}, {pos.y:.2f})"
        )

    def _cb_goal(self, msg: PoseStamped) -> None:
        self._last_goal_msg = msg
        self._plan_to_goal(msg)

    def _cb_replan_request(self, msg: Bool) -> None:
        if not msg.data:
            return
        if self._last_goal_msg is None:
            self.get_logger().warn(
                "[anhc_global_planner] replan requested but no previous goal available"
            )
            return
        self.get_logger().info("[anhc_global_planner] replan requested")
        self._plan_to_goal(self._last_goal_msg)

    def _plan_to_goal(self, msg: PoseStamped) -> None:
        if self._costmap is None:
            self.get_logger().warn(
                "[anhc_global_planner] goal received but no costmap yet — ignoring"
            )
            return

        # --- resolve robot start pose from TF or /initialpose ---
        start = self._get_robot_pose()
        if start is None:
            self.get_logger().warn(
                "[anhc_global_planner] could not determine robot pose — ignoring goal"
            )
            return

        goal = (msg.pose.position.x, msg.pose.position.y)

        # --- hot-swap algorithm if the parameter changed ---
        requested = self.get_parameter("algorithm").get_parameter_value().string_value
        if requested != self._planner.get_name():
            self._planner = self._build_planner(requested)
            self.get_logger().info(
                f"[anhc_global_planner] switched to algorithm: {self._planner.get_name()}"
            )

        self.get_logger().info(
            f"[anhc_global_planner] planning {start} → {goal} "
            f"with {self._planner.get_name()}"
        )

        path_points = self._planner.plan(start, goal, self._costmap)

        if not path_points:
            self.get_logger().warn(
                "[anhc_global_planner] planner returned empty path"
            )
            stats = getattr(self._planner, "stats", {})
            stats.setdefault("algorithm", self._planner.get_name())
            stats.setdefault("nodes_expanded", 0)
            stats.setdefault("planning_time_ms", 0.0)
            stats.setdefault("path_length_m", 0.0)
            stats["status"] = "no_path"
        else:
            stats = getattr(self._planner, "stats", {})
            stats.setdefault("algorithm", self._planner.get_name())
            stats["status"] = "success"

        self.get_logger().info(
            f"[anhc_global_planner] stats: {json.dumps(stats)}"
        )

        self._pub_path.publish(self._build_path_msg(path_points, msg.header.frame_id))
        self._pub_stats.publish(String(data=json.dumps(stats)))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_robot_pose(self) -> tuple[float, float] | None:
        """Resolve start pose for planning.

        Prefer manually provided /initialpose when available (RViz workflow),
        then fall back to TF map→base_footprint / map→base_link.
        """
        if self._manual_start is not None:
            return self._manual_start

        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_footprint", rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            return (x, y)
        except TransformException:
            pass

        try:
            t = self._tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            return (x, y)
        except TransformException:
            pass

        return None

    def _build_planner(self, name: str | None = None) -> BasePlanner:
        if name is None:
            name = self.get_parameter("algorithm").get_parameter_value().string_value
        obstacle_threshold = (
            self.get_parameter("obstacle_cost_threshold")
            .get_parameter_value()
            .integer_value
        )
        smooth_data = (
            self.get_parameter("path_smooth_data_weight")
            .get_parameter_value()
            .double_value
        )
        smooth_weight = (
            self.get_parameter("path_smooth_smooth_weight")
            .get_parameter_value()
            .double_value
        )
        smooth_min_wp = (
            self.get_parameter("path_smooth_min_waypoints")
            .get_parameter_value()
            .integer_value
        )
        cls = _PLANNER_REGISTRY.get(name)
        if cls is None:
            self.get_logger().warn(
                f"[anhc_global_planner] unknown algorithm '{name}', falling back to astar"
            )
            cls = AStarPlanner
        ot = int(obstacle_threshold)
        if cls in (AStarPlanner, DijkstraPlanner):
            return cls(
                obstacle_threshold=ot,
                path_smooth_data_weight=float(smooth_data),
                path_smooth_smooth_weight=float(smooth_weight),
                path_smooth_min_waypoints=int(smooth_min_wp),
            )
        if cls is DStarLitePlanner:
            return cls(obstacle_threshold=ot)
        if cls is RRTStarPlanner:
            return cls(obstacle_threshold=ot)
        if cls is RLPlanner:
            return cls(obstacle_threshold=ot)
        return cls()

    def _build_path_msg(
        self, points: list[tuple[float, float]], frame_id: str
    ) -> Path:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id if frame_id else "map"
        for x, y in points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnhcGlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
