"""Benchmark runner — v1 schema (docs/BENCHMARK_PLAN_v1.md).

Writes per-trial CSV under ``~/anhc_benchmark_results/raw/`` with metrics
L_planned, L_executed, Tc, Tg, smoothness, clearance, and success criteria.
"""

from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any, Optional

import numpy as np
import psutil
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

_CSV_V1_FIELDNAMES = [
    "timestamp",
    "scenario_id",
    "scenario_tag",
    "algorithm",
    "trial",
    "success",
    "failure_reason",
    "L_planned_m",
    "L_executed_m",
    "L_ratio",
    "Tc_ms",
    "Tg_s",
    "S_rad_per_m",
    "num_turns",
    "max_curvature",
    "C_min_m",
    "C_avg_m",
    "SR_pct",
    "XTE_max_m",
    "XTE_avg_m",
    "cpu_percent",
    "memory_mb",
    "nodes_expanded",
    "replanning_count",
    "map_name",
    "start_x",
    "start_y",
    "start_theta",
    "goal_x",
    "goal_y",
    "goal_theta",
]


def _yaw_to_quat(theta: float) -> Quaternion:
    half = theta * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def _normalize_pose(
    raw: Any,
    default_theta: float = 0.0,
) -> tuple[float, float, float]:
    if isinstance(raw, dict):
        return (
            float(raw.get("x", 0.0)),
            float(raw.get("y", 0.0)),
            float(raw.get("theta", default_theta)),
        )
    if isinstance(raw, (list, tuple)):
        if len(raw) >= 3:
            return float(raw[0]), float(raw[1]), float(raw[2])
        if len(raw) == 2:
            return float(raw[0]), float(raw[1]), default_theta
    return 0.0, 0.0, default_theta


def _path_length(points: list[tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(len(points) - 1):
        dx = points[i + 1][0] - points[i][0]
        dy = points[i + 1][1] - points[i][1]
        total += math.hypot(dx, dy)
    return total


def _path_heading_changes(points: list[tuple[float, float]]) -> list[float]:
    if len(points) < 3:
        return []
    changes: list[float] = []
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
        changes.append(diff)
    return changes


def _path_metrics(points: list[tuple[float, float]]) -> dict[str, float | int]:
    L = _path_length(points)
    changes = _path_heading_changes(points)
    sum_turn = float(sum(changes))
    s_rad_per_m = sum_turn / L if L > 1e-9 else 0.0
    turn_thresh = math.radians(15.0)
    num_turns = sum(1 for d in changes if d > turn_thresh)
    max_curv = 0.0
    for i, dtheta in enumerate(changes):
        idx = i + 1
        p0, p1, p2 = points[idx - 1], points[idx], points[idx + 1]
        seg_a = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        seg_b = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        arc_len = max(1e-6, min(seg_a, seg_b))
        max_curv = max(max_curv, dtheta / arc_len)
    return {
        "L_geom_m": L,
        "S_rad_per_m": float(s_rad_per_m),
        "num_turns": int(num_turns),
        "max_curvature": float(max_curv),
    }


def _clearance_stats(
    path_points: list[tuple[float, float]],
    costmap: OccupancyGrid | None,
    obstacle_threshold: int = 50,
) -> tuple[float, float]:
    if not path_points or costmap is None:
        return -1.0, -1.0
    info = costmap.info
    data = np.array(costmap.data, dtype=np.float32).reshape(info.height, info.width)
    occ_y, occ_x = np.where(data > obstacle_threshold)
    if len(occ_x) == 0:
        return -1.0, -1.0
    occ_wx = info.origin.position.x + (occ_x + 0.5) * info.resolution
    occ_wy = info.origin.position.y + (occ_y + 0.5) * info.resolution
    mins: list[float] = []
    for px, py in path_points:
        dists = np.hypot(occ_wx - px, occ_wy - py)
        mins.append(float(np.min(dists)))
    return float(np.min(mins)), float(np.mean(mins))


def _path_hits_lethal(
    path_points: list[tuple[float, float]],
    costmap: OccupancyGrid | None,
    lethal: int = 90,
) -> bool:
    if not path_points or costmap is None:
        return False
    info = costmap.info
    w, h = info.width, info.height
    res = info.resolution
    ox = info.origin.position.x
    oy = info.origin.position.y
    data = costmap.data
    for px, py in path_points:
        ix = int((px - ox) / res)
        iy = int((py - oy) / res)
        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            continue
        val = data[iy * w + ix]
        if val >= lethal:
            return True
    return False


class AnhcBenchmarkRunnerNode(Node):
    """Orchestrates benchmark trials; writes v1 CSV."""

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
        self.declare_parameter("startup_delay_s", 3.0)
        self.declare_parameter("gz_world_name", "anhc_office_v2_world")
        self.declare_parameter("enable_gz_dynamic_obstacle", True)

        self._lock = threading.Lock()
        self._planning_stats: dict | None = None
        self._current_path: list[tuple[float, float]] = []
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_speed: float = 0.0
        self._collision_flag: bool = False
        self._costmap: OccupancyGrid | None = None

        self._stats_ready = threading.Event()
        self._csv_path: str = ""

        self.create_subscription(String, "/planning/stats", self._cb_stats, 10)
        self.create_subscription(Path, "/planning/path", self._cb_path, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._cb_odom, 10)
        self.create_subscription(Bool, "/perception/depth_obstacles", self._cb_obstacle, 10)
        self.create_subscription(OccupancyGrid, "/costmap/global", self._cb_costmap, 1)

        self._pub_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._pub_init = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self._pub_path = self.create_publisher(Path, "/planning/path", 10)
        self._pub_gz_obstacle = self.create_publisher(String, "/benchmark/gz_obstacle_cmd", 10)

        self._param_client = self.create_client(
            SetParameters, "/anhc_global_planner/set_parameters"
        )

        self._bench_thread = threading.Thread(
            target=self._benchmark_loop, daemon=True, name="bench_loop"
        )
        self._bench_thread.start()
        self.get_logger().info("[anhc_benchmark_runner] started (v1 CSV schema)")

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

    def _load_scenarios(self, cfg: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
        if "scenarios" in cfg:
            return cfg.get("common", {}), cfg["scenarios"]
        common = cfg.get("common", {})
        skip = {"common", "reproducibility", "scenarios"}
        scenarios = {k: v for k, v in cfg.items() if k not in skip}
        return common, scenarios

    def _validate_scenario(self, sid: str, sc: dict[str, Any]) -> bool:
        for k in ("name", "start", "goal"):
            if k not in sc:
                self.get_logger().error(
                    f"[benchmark] scenario '{sid}' missing '{k}' — skipped"
                )
                return False
        return True

    def _trial_count(
        self,
        algorithm: str,
        scenario: dict[str, Any],
        common: dict[str, Any],
    ) -> int:
        stoch = set(common.get("stochastic_algorithms", ["rrt_star"]))
        td = int(scenario.get("trials_deterministic", common.get("trials_deterministic", 5)))
        ts = int(scenario.get("trials_stochastic", common.get("trials_stochastic", 20)))
        if "trials" in scenario and "trials_deterministic" not in scenario:
            return int(scenario["trials"])
        return ts if algorithm in stoch else td

    def _algorithms_for_scenario(
        self,
        scenario: dict[str, Any],
        common: dict[str, Any],
    ) -> list[str]:
        if "algorithms" in scenario:
            return list(scenario["algorithms"])
        algos = common.get("algorithms")
        if algos:
            return list(algos)
        return ["astar"]

    def _benchmark_loop(self) -> None:
        delay = float(
            self.get_parameter("startup_delay_s").get_parameter_value().double_value
        )
        time.sleep(max(0.0, delay))

        scenario_file = (
            self.get_parameter("scenario_file")
            .get_parameter_value()
            .string_value
        )
        if not scenario_file:
            self.get_logger().error("scenario_file parameter not set — exiting")
            return

        try:
            with open(scenario_file, "r", encoding="utf-8") as fh:
                cfg = yaml.safe_load(fh)
        except Exception as exc:
            self.get_logger().error(f"cannot load scenario file: {exc}")
            return

        if not isinstance(cfg, dict):
            self.get_logger().error("scenario file root must be a mapping")
            return

        common, scenarios = self._load_scenarios(cfg)
        output_base = os.path.expanduser(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        raw_dir = os.path.join(output_base, "raw")
        os.makedirs(raw_dir, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self._csv_path = os.path.join(raw_dir, f"benchmark_{ts}.csv")
        self._init_csv()

        scenario_keys = sorted(scenarios.keys(), key=lambda k: str(k))
        n_sc = len(scenario_keys)
        pause_sc = float(common.get("pause_between_scenarios_s", 5.0))

        for si, scenario_id in enumerate(scenario_keys):
            sc = scenarios[scenario_id]
            if not isinstance(sc, dict):
                continue
            if not self._validate_scenario(str(scenario_id), sc):
                continue
            self._run_scenario(str(scenario_id), sc, common, si + 1, n_sc)
            if si < n_sc - 1:
                time.sleep(pause_sc)

        self.get_logger().info(f"all scenarios complete — CSV: {self._csv_path}")

    def _run_scenario(
        self,
        scenario_id: str,
        scenario: dict[str, Any],
        common: dict[str, Any],
        sc_idx: int,
        n_scenarios: int,
    ) -> None:
        name = str(scenario.get("name", scenario_id))
        tag = str(scenario.get("tag", ""))
        map_name = str(scenario.get("map", name))
        start = _normalize_pose(scenario["start"])
        goal = _normalize_pose(scenario["goal"])
        algorithms = self._algorithms_for_scenario(scenario, common)
        n_algo = len(algorithms)

        self.get_logger().info(
            f"[benchmark] [{sc_idx}/{n_scenarios}] scenario '{name}' — "
            f"{n_algo} algorithm(s)"
        )

        pause_tr = float(common.get("pause_between_trials_s", 3.0))

        for algorithm in algorithms:
            trials = self._trial_count(algorithm, scenario, common)
            for trial_idx in range(trials):
                self.get_logger().info(
                    f"[scenario {sc_idx}/{n_scenarios}][{algorithm}]"
                    f"[trial {trial_idx + 1}/{trials}]"
                )
                dyn = scenario.get("dynamic_obstacle")
                if not isinstance(dyn, dict):
                    dyn = None
                row = self._run_trial(
                    scenario_id=scenario_id,
                    scenario_tag=tag,
                    map_name=map_name,
                    algorithm=algorithm,
                    trial_num=trial_idx + 1,
                    start=start,
                    goal=goal,
                    scenario=scenario,
                    common=common,
                    goal_xy=(goal[0], goal[1]),
                    dynamic_obstacle=dyn,
                )
                self._append_csv(row)
                time.sleep(pause_tr)

    def _publish_empty_path(self) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        self._pub_path.publish(msg)

    def _publish_initialpose(self, x: float, y: float, theta: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation = _yaw_to_quat(theta)
        self._pub_init.publish(msg)

    def _publish_goal(self, x: float, y: float, theta: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation = _yaw_to_quat(theta)
        self._pub_goal.publish(msg)

    def _set_algorithm(self, algorithm: str) -> bool:
        if not self._param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "/anhc_global_planner/set_parameters unavailable"
            )
            return False
        pval = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=algorithm)
        param = Parameter(name="algorithm", value=pval)
        req = SetParameters.Request(parameters=[param])
        future = self._param_client.call_async(req)
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if future.done() and future.result() is not None:
            results = future.result().results
            return bool(results and results[0].successful)
        return False

    def _run_trial(
        self,
        scenario_id: str,
        scenario_tag: str,
        map_name: str,
        algorithm: str,
        trial_num: int,
        start: tuple[float, float, float],
        goal: tuple[float, float, float],
        scenario: dict[str, Any],
        common: dict[str, Any],
        goal_xy: tuple[float, float],
        dynamic_obstacle: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        goal_tol = float(
            scenario.get(
                "goal_tolerance_m",
                common.get(
                    "goal_tolerance_m",
                    self.get_parameter("goal_tolerance").get_parameter_value().double_value,
                ),
            )
        )
        plan_timeout_ms = float(
            scenario.get("planning_timeout_ms", common.get("planning_timeout_ms", 10000.0))
        )
        plan_timeout_s = plan_timeout_ms / 1000.0
        exec_timeout = float(
            scenario.get(
                "execution_timeout_s",
                common.get(
                    "execution_timeout_s",
                    self.get_parameter("execution_timeout_s").get_parameter_value().double_value,
                ),
            )
        )
        stuck_v = float(common.get("robot_stuck_velocity_threshold", 0.01))
        stuck_dur = float(common.get("robot_stuck_duration_s", 10.0))
        lethal = int(common.get("lethal_cost_threshold", 90))
        reset_wait = float(common.get("robot_reset_timeout_s", 0.5))

        row: dict[str, Any] = {k: "" for k in _CSV_V1_FIELDNAMES}
        row["scenario_id"] = scenario_id
        row["scenario_tag"] = scenario_tag
        row["algorithm"] = algorithm
        row["trial"] = trial_num
        row["success"] = 0
        row["failure_reason"] = ""
        row["SR_pct"] = ""
        row["XTE_max_m"] = ""
        row["XTE_avg_m"] = ""
        row["replanning_count"] = 0
        row["map_name"] = map_name
        row["start_x"] = start[0]
        row["start_y"] = start[1]
        row["start_theta"] = start[2]
        row["goal_x"] = goal[0]
        row["goal_y"] = goal[1]
        row["goal_theta"] = goal[2]

        for z in (
            "L_planned_m",
            "L_executed_m",
            "L_ratio",
            "Tc_ms",
            "Tg_s",
            "S_rad_per_m",
            "max_curvature",
            "C_min_m",
            "C_avg_m",
            "cpu_percent",
            "memory_mb",
        ):
            row[z] = 0.0
        row["num_turns"] = 0
        row["nodes_expanded"] = 0

        self._set_algorithm(algorithm)
        self._publish_empty_path()
        self._publish_initialpose(start[0], start[1], start[2])
        time.sleep(max(0.1, reset_wait))

        self._stats_ready.clear()
        with self._lock:
            self._planning_stats = None
            self._collision_flag = False

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

        self._publish_goal(goal[0], goal[1], goal[2])

        got_stats = self._stats_ready.wait(timeout=plan_timeout_s)
        stop_monitor.set()
        monitor_thread.join(timeout=2.0)

        if cpu_samples:
            row["cpu_percent"] = float(sum(cpu_samples) / len(cpu_samples))
        if mem_samples:
            row["memory_mb"] = float(max(mem_samples))

        ts_end = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
        row["timestamp"] = ts_end

        if not got_stats:
            row["failure_reason"] = "planning_timeout"
            return row

        with self._lock:
            stats = dict(self._planning_stats) if self._planning_stats else {}
            path_snap = list(self._current_path)
            costmap_snap = self._costmap

        row["Tc_ms"] = float(stats.get("planning_time_ms", 0.0))
        row["nodes_expanded"] = int(stats.get("nodes_expanded", 0))

        if row["Tc_ms"] > plan_timeout_ms:
            row["failure_reason"] = "planning_timeout"
            return row

        if stats.get("status") == "no_path":
            row["failure_reason"] = "no_path"
            return row

        if len(path_snap) < 2:
            row["failure_reason"] = "no_path"
            return row

        pm = _path_metrics(path_snap)
        L_planned = float(stats.get("path_length_m", 0.0)) or float(pm["L_geom_m"])
        row["L_planned_m"] = L_planned
        row["S_rad_per_m"] = float(pm["S_rad_per_m"])
        row["num_turns"] = int(pm["num_turns"])
        row["max_curvature"] = float(pm["max_curvature"])

        cmin, cavg = _clearance_stats(path_snap, costmap_snap)
        row["C_min_m"] = cmin
        row["C_avg_m"] = cavg

        if _path_hits_lethal(path_snap, costmap_snap, lethal):
            row["failure_reason"] = "lethal_on_path"
            return row

        dyn_state: dict[str, Any] = {
            "spawned": False,
            "timer": None,
            "model": "",
            "gz_world": "",
        }
        use_dyn = bool(
            dynamic_obstacle
            and self.get_parameter("enable_gz_dynamic_obstacle")
            .get_parameter_value()
            .bool_value
        )
        if use_dyn and dynamic_obstacle is not None:
            gz_world = str(
                common.get(
                    "gz_world_name",
                    self.get_parameter("gz_world_name")
                    .get_parameter_value()
                    .string_value,
                )
            )
            safe_sid = "".join(
                c if (c.isalnum() or c == "_") else "_" for c in scenario_id
            )
            model_name = f"anhc_bm_dyn_{safe_sid}_{trial_num}"
            delay = float(dynamic_obstacle.get("activate_after_s", 3.0))

            def _spawn_dyn() -> None:
                spawn = dynamic_obstacle.get("spawn", {})
                vel = dynamic_obstacle.get("velocity", {})
                size = dynamic_obstacle.get("size", {})
                cmd = {
                    "op": "spawn_moving",
                    "gz_world": gz_world,
                    "model_name": model_name,
                    "x": float(spawn.get("x", 0.0)),
                    "y": float(spawn.get("y", 0.0)),
                    "z": float(spawn.get("z", 0.25)),
                    "yaw": float(spawn.get("theta", spawn.get("yaw", 0.0))),
                    "lx": float(size.get("x", 0.35)),
                    "ly": float(size.get("y", 0.35)),
                    "lz": float(size.get("z", 0.5)),
                    "vx": float(vel.get("x", 0.0)),
                    "vy": float(vel.get("y", 0.0)),
                }
                self._pub_gz_obstacle.publish(String(data=json.dumps(cmd)))
                dyn_state["spawned"] = True
                dyn_state["model"] = model_name
                dyn_state["gz_world"] = gz_world

            dyn_timer = threading.Timer(delay, _spawn_dyn)
            dyn_timer.daemon = True
            dyn_timer.start()
            dyn_state["timer"] = dyn_timer

        t_plan = time.time()
        gx, gy = goal_xy

        prev_x: float | None = None
        prev_y: float | None = None
        executed_len = 0.0
        success = False
        failure_reason = ""
        stuck_low_since: float | None = None

        try:
            while (time.time() - t_plan) < exec_timeout:
                with self._lock:
                    rx, ry = self._robot_x, self._robot_y
                    col = self._collision_flag
                    spd = self._robot_speed

                if col:
                    failure_reason = "collision"
                    break

                if prev_x is not None:
                    executed_len += math.hypot(rx - prev_x, ry - prev_y)
                prev_x, prev_y = rx, ry

                if math.hypot(rx - gx, ry - gy) < goal_tol:
                    success = True
                    break

                if spd < stuck_v:
                    if stuck_low_since is None:
                        stuck_low_since = time.time()
                    elif time.time() - stuck_low_since >= stuck_dur:
                        failure_reason = "robot_stuck"
                        break
                else:
                    stuck_low_since = None

                time.sleep(0.05)
        finally:
            tmr = dyn_state.get("timer")
            if isinstance(tmr, threading.Timer) and tmr.is_alive():
                tmr.cancel()
            if dyn_state.get("spawned"):
                gz_w = str(dyn_state.get("gz_world", ""))
                mname = str(dyn_state.get("model", ""))
                if gz_w and mname:
                    self._pub_gz_obstacle.publish(
                        String(
                            data=json.dumps(
                                {
                                    "op": "despawn",
                                    "gz_world": gz_w,
                                    "model_name": mname,
                                }
                            )
                        )
                    )

        tg = time.time() - t_plan
        row["Tg_s"] = float(tg)
        row["L_executed_m"] = float(executed_len)
        if L_planned > 1e-9:
            row["L_ratio"] = float(executed_len / L_planned)
        else:
            row["L_ratio"] = 0.0

        if success:
            row["success"] = 1
            row["failure_reason"] = ""
        else:
            if not failure_reason:
                failure_reason = "execution_timeout"
            row["failure_reason"] = failure_reason

        return row

    def _init_csv(self) -> None:
        with open(self._csv_path, "w", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=_CSV_V1_FIELDNAMES)
            writer.writeheader()
        self.get_logger().info(f"CSV initialised at {self._csv_path}")

    def _append_csv(self, row: dict[str, Any]) -> None:
        with open(self._csv_path, "a", newline="", encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=_CSV_V1_FIELDNAMES)
            writer.writerow({k: row.get(k, "") for k in _CSV_V1_FIELDNAMES})


def main(args: Optional[list[str]] = None) -> None:
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
