"""Benchmark runner node for the anhc autonomous vehicle.

For each scenario defined in a YAML file, publishes goals to the planning stack,
collects metrics from /planning/stats and /odometry/filtered, and writes
per-trial results to a CSV in ~/anhc_benchmark_results/.
"""

import csv
import json
import math
import os
import threading
import time
from datetime import datetime

import psutil
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

_CSV_FIELDNAMES = [
    "algorithm_name",
    "map_name",
    "start_x",
    "start_y",
    "goal_x",
    "goal_y",
    "planning_time_ms",
    "path_length_m",
    "path_smoothness",
    "nodes_expanded",
    # planning_success: True  ↔ planner returned a valid path (path_length_m > 0 is consistent)
    # success:          True  ↔ robot reached the goal within execution_timeout_s
    # These are independent: planning can succeed while execution times out (success=False).
    "planning_success",
    "success",
    "collision",
    "execution_time_s",
    "clearance_avg_m",
    "cpu_usage_percent",
    "memory_mb",
]


class AnhcBenchmarkRunnerNode(Node):
    """Orchestrates multi-trial, multi-algorithm benchmark runs.

    Topics
    ------
    Subscriptions:
        /planning/stats             — std_msgs/String (JSON)
        /planning/path              — nav_msgs/Path
        /odometry/filtered          — nav_msgs/Odometry
        /perception/depth_obstacles — std_msgs/Bool
        /costmap/global             — nav_msgs/OccupancyGrid

    Publications:
        /goal_pose    — geometry_msgs/PoseStamped
        /initialpose  — geometry_msgs/PoseWithCovarianceStamped

    Parameters
    ----------
    scenario_file : str
        Absolute path to a YAML scenario definition file.
    goal_tolerance : float
        Radius (m) within which the goal is considered reached.  Default 0.30.
    planning_timeout_s : float
        Seconds to wait for /planning/stats before declaring planning failure.
    execution_timeout_s : float
        Seconds to wait for goal-reach/collision before timing out a trial.
    """

    def __init__(self) -> None:
        super().__init__("anhc_benchmark_runner")

        self.declare_parameter("scenario_file", "")
        self.declare_parameter("goal_tolerance", 0.30)
        self.declare_parameter("planning_timeout_s", 15.0)
        self.declare_parameter("execution_timeout_s", 90.0)
        self.declare_parameter(
            "output_dir",
            os.path.expanduser("~/anhc_benchmark_results"),
        )

        # ── shared state (written by ROS callbacks, read by benchmark thread) ──
        self._lock = threading.Lock()
        self._planning_stats: dict | None = None
        self._current_path: list[tuple[float, float]] = []
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_speed: float = 0.0
        self._collision_flag: bool = False
        self._costmap: OccupancyGrid | None = None

        # ── synchronisation events ──
        self._stats_ready = threading.Event()

        # ── results accumulator ──
        self._results: list[dict] = []
        self._csv_path: str = ""

        # ── subscriptions ──
        self.create_subscription(String, "/planning/stats", self._cb_stats, 10)
        self.create_subscription(Path, "/planning/path", self._cb_path, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._cb_odom, 10)
        self.create_subscription(Bool, "/perception/depth_obstacles", self._cb_obstacle, 10)
        self.create_subscription(OccupancyGrid, "/costmap/global", self._cb_costmap, 1)

        # ── publishers ──
        self._pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._pub_init = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        # ── parameter service client (targets the global planner) ──
        self._param_client = self.create_client(
            SetParameters, "/anhc_global_planner/set_parameters"
        )

        # ── launch the benchmark loop in a background daemon thread ──
        self._bench_thread = threading.Thread(
            target=self._benchmark_loop, daemon=True, name="bench_loop"
        )
        self._bench_thread.start()

        self.get_logger().info("[anhc_benchmark_runner] started")

    # ──────────────────────────────────────────────────────────────────────────
    # ROS callbacks
    # ──────────────────────────────────────────────────────────────────────────

    def _cb_stats(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {}
        with self._lock:
            self._planning_stats = data
        self._stats_ready.set()

    def _cb_path(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._current_path = pts

    def _cb_odom(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        with self._lock:
            self._robot_x = msg.pose.pose.position.x
            self._robot_y = msg.pose.pose.position.y
            self._robot_speed = math.hypot(vx, vy)

    def _cb_obstacle(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._collision_flag = True

    def _cb_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = msg

    # ──────────────────────────────────────────────────────────────────────────
    # Benchmark orchestration
    # ──────────────────────────────────────────────────────────────────────────

    def _benchmark_loop(self) -> None:
        # Allow the node and the rest of the system to finish starting up.
        time.sleep(3.0)

        scenario_file = (
            self.get_parameter("scenario_file")
            .get_parameter_value()
            .string_value
        )
        if not scenario_file:
            self.get_logger().error(
                "[anhc_benchmark_runner] 'scenario_file' parameter not set — exiting"
            )
            return

        try:
            with open(scenario_file, "r") as fh:
                cfg = yaml.safe_load(fh)
        except Exception as exc:
            self.get_logger().error(
                f"[anhc_benchmark_runner] cannot load scenario file: {exc}"
            )
            return

        output_dir = os.path.expanduser(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        os.makedirs(output_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(output_dir, f"benchmark_{ts}.csv")
        self._init_csv()

        for scenario_key in sorted(cfg.keys()):
            self._run_scenario(cfg[scenario_key])

        self.get_logger().info(
            f"[anhc_benchmark_runner] all scenarios complete — results at {self._csv_path}"
        )

    def _run_scenario(self, scenario: dict) -> None:
        name = scenario.get("name", "unnamed")
        start = scenario.get("start", [0.0, 0.0])
        goal = scenario.get("goal", [1.0, 0.0])
        algorithms: list[str] = scenario.get("algorithms", ["astar"])
        trials: int = int(scenario.get("trials", 1))

        self.get_logger().info(
            f"[anhc_benchmark_runner] scenario '{name}' — "
            f"{len(algorithms)} algorithm(s), {trials} trial(s) each"
        )

        for algorithm in algorithms:
            for trial_idx in range(trials):
                self.get_logger().info(
                    f"[benchmark] {name}/{algorithm} trial {trial_idx + 1}/{trials}"
                )
                row = self._run_trial(name, algorithm, start, goal)
                self._results.append(row)
                self._append_csv(row)

    def _run_trial(
        self,
        scenario_name: str,
        algorithm: str,
        start: list,
        goal: list,
    ) -> dict:
        """Execute one trial and return a result dict matching _CSV_FIELDNAMES."""
        goal_tol = (
            self.get_parameter("goal_tolerance").get_parameter_value().double_value
        )
        plan_timeout = (
            self.get_parameter("planning_timeout_s")
            .get_parameter_value()
            .double_value
        )
        exec_timeout = (
            self.get_parameter("execution_timeout_s")
            .get_parameter_value()
            .double_value
        )

        row: dict = {
            "algorithm_name": algorithm,
            "map_name": scenario_name,
            "start_x": float(start[0]),
            "start_y": float(start[1]),
            "goal_x": float(goal[0]),
            "goal_y": float(goal[1]),
            "planning_time_ms": 0.0,
            "path_length_m": 0.0,
            "path_smoothness": 0.0,
            "nodes_expanded": 0,
            # Both default to False; set to True only when explicitly confirmed below.
            "planning_success": False,
            "success": False,
            "collision": False,
            "execution_time_s": 0.0,
            "clearance_avg_m": -1.0,
            "cpu_usage_percent": 0.0,
            "memory_mb": 0.0,
        }

        # ── set algorithm parameter on the global planner ──
        self._set_algorithm(algorithm)

        # ── broadcast starting pose so the planner can use it as fallback ──
        self._publish_initialpose(start[0], start[1])
        time.sleep(0.4)

        # ── reset per-trial state ──
        self._stats_ready.clear()
        with self._lock:
            self._planning_stats = None
            self._collision_flag = False

        # ── start CPU / memory monitor thread ──
        cpu_samples: list[float] = []
        mem_samples: list[float] = []
        stop_monitor = threading.Event()

        def _monitor() -> None:
            proc = psutil.Process(os.getpid())
            while not stop_monitor.is_set():
                try:
                    cpu_samples.append(proc.cpu_percent(interval=None))
                    mem_samples.append(proc.memory_info().rss / 1_048_576)
                except psutil.NoSuchProcess:
                    break
                time.sleep(0.1)

        monitor_thread = threading.Thread(target=_monitor, daemon=True)
        monitor_thread.start()

        # ── publish goal → triggers planning ──
        self._publish_goal(goal[0], goal[1])

        # ── wait for planning stats ──
        got_stats = self._stats_ready.wait(timeout=plan_timeout)

        # ── stop CPU monitor ──
        stop_monitor.set()
        monitor_thread.join(timeout=2.0)

        if cpu_samples:
            row["cpu_usage_percent"] = float(sum(cpu_samples) / len(cpu_samples))
        if mem_samples:
            row["memory_mb"] = float(max(mem_samples))

        if not got_stats:
            self.get_logger().warn(
                f"[benchmark] planning timeout for {algorithm} in '{scenario_name}'"
            )
            return row

        with self._lock:
            stats = dict(self._planning_stats) if self._planning_stats else {}
            path_snap = list(self._current_path)
            costmap_snap = self._costmap

        row["planning_time_ms"] = float(stats.get("planning_time_ms", 0.0))
        row["nodes_expanded"] = int(stats.get("nodes_expanded", 0))

        if stats.get("status") == "no_path":
            # Planner explicitly reported no solution — path_length_m stays 0,
            # planning_success stays False, success stays False.
            return row

        # Planner returned a valid path: record path metrics and set planning_success.
        row["path_length_m"] = float(stats.get("path_length_m", 0.0))
        row["planning_success"] = True
        row["path_smoothness"] = self._compute_smoothness(path_snap)
        row["clearance_avg_m"] = self._compute_clearance(path_snap, costmap_snap)

        # ── monitor execution until goal or timeout ──
        t0 = time.time()
        success = False
        collision = False

        while (time.time() - t0) < exec_timeout:
            with self._lock:
                rx, ry = self._robot_x, self._robot_y
                col = self._collision_flag

            if math.hypot(rx - goal[0], ry - goal[1]) < goal_tol:
                success = True
                break
            if col:
                collision = True
                break
            time.sleep(0.1)

        row["success"] = success
        row["collision"] = collision
        row["execution_time_s"] = float(time.time() - t0)

        return row

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _set_algorithm(self, algorithm: str) -> bool:
        if not self._param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "[benchmark] /anhc_global_planner/set_parameters service unavailable"
            )
            return False

        pval = ParameterValue()
        pval.type = ParameterType.PARAMETER_STRING
        pval.string_value = algorithm

        param = Parameter()
        param.name = "algorithm"
        param.value = pval

        req = SetParameters.Request()
        req.parameters = [param]

        future = self._param_client.call_async(req)

        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if future.done() and future.result() is not None:
            results = future.result().results
            ok = bool(results and results[0].successful)
            if not ok:
                self.get_logger().warn(
                    f"[benchmark] failed to set algorithm param to '{algorithm}'"
                )
            return ok

        self.get_logger().warn("[benchmark] set_parameters future timed out")
        return False

    def _publish_goal(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self._pub_goal.publish(msg)

    def _publish_initialpose(self, x: float, y: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.w = 1.0
        self._pub_init.publish(msg)

    # ──────────────────────────────────────────────────────────────────────────
    # Metric computations
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _compute_smoothness(points: list[tuple[float, float]]) -> float:
        """Sum of absolute heading changes between consecutive path segments (rad)."""
        if len(points) < 3:
            return 0.0
        total = 0.0
        for i in range(len(points) - 2):
            dx1 = points[i + 1][0] - points[i][0]
            dy1 = points[i + 1][1] - points[i][1]
            dx2 = points[i + 2][0] - points[i + 1][0]
            dy2 = points[i + 2][1] - points[i + 1][1]
            a1 = math.atan2(dy1, dx1)
            a2 = math.atan2(dy2, dx2)
            diff = abs(a2 - a1)
            if diff > math.pi:
                diff = 2.0 * math.pi - diff
            total += diff
        return total

    @staticmethod
    def _compute_clearance(
        path_points: list[tuple[float, float]],
        costmap: OccupancyGrid | None,
    ) -> float:
        """Mean distance (m) from each path point to the nearest obstacle cell."""
        if not path_points or costmap is None:
            return -1.0

        import numpy as np  # noqa: PLC0415

        info = costmap.info
        data = np.array(costmap.data, dtype=np.float32).reshape(
            info.height, info.width
        )

        occ_y, occ_x = np.where(data > 50)
        if len(occ_x) == 0:
            return -1.0

        occ_wx = info.origin.position.x + (occ_x + 0.5) * info.resolution
        occ_wy = info.origin.position.y + (occ_y + 0.5) * info.resolution

        clearances = []
        for px, py in path_points:
            dists = np.hypot(occ_wx - px, occ_wy - py)
            clearances.append(float(np.min(dists)))

        return float(np.mean(clearances))

    # ──────────────────────────────────────────────────────────────────────────
    # CSV I/O
    # ──────────────────────────────────────────────────────────────────────────

    def _init_csv(self) -> None:
        with open(self._csv_path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDNAMES)
            writer.writeheader()
        self.get_logger().info(f"[benchmark] CSV initialised at {self._csv_path}")

    def _append_csv(self, row: dict) -> None:
        with open(self._csv_path, "a", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=_CSV_FIELDNAMES)
            writer.writerow({k: row.get(k, "") for k in _CSV_FIELDNAMES})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AnhcBenchmarkRunnerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
