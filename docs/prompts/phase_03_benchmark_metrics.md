# Phase 3 — Comprehensive Benchmark Metrics & Algorithm Comparison

## Context

This is a ROS 2 (Jazzy) project at `/home/anhuynh/anhc_botrl`.

**Prerequisite:** Phase 1 (all 8 planners implemented) and Phase 2 (banded inflation layer) must be complete.

The benchmark infrastructure already exists:
- `src/anhc_benchmark/anhc_benchmark/anhc_benchmark_runner_node.py` — orchestrates trials, writes CSV
- `src/anhc_benchmark/anhc_benchmark/anhc_live_metrics_node.py` — live metric streaming
- `src/anhc_benchmark/anhc_benchmark/anhc_results_analyzer_node.py` — post-run analysis
- `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml` — scenario definitions
- `src/anhc_viz/scripts/anhc_plot_results.py` — result visualisation

**Key files to read before starting:**
- `src/anhc_benchmark/anhc_benchmark/anhc_benchmark_runner_node.py` (entire file)
- `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml`
- `src/anhc_planning/anhc_planning/planners/base_planner.py` (`path_length`, `smooth_path`)
- `src/anhc_viz/scripts/anhc_plot_results.py`

---

## Goal

Extend the benchmark system to collect, compute, and visualise **all 6 metric groups** for every algorithm × scenario combination.

---

## Task 1 — Extend Planner Stats

Every planner's `_record_stats` (and the `stats` property) must emit an extended dict.

### Update `BasePlanner` in `src/anhc_planning/anhc_planning/planners/base_planner.py`

Add two new static utility methods:

```python
@staticmethod
def compute_curvature(path: List[Tuple[float, float]]) -> List[float]:
    """Per-waypoint curvature κ = |Δθ| / segment_length (rad/m).
    Returns list of length len(path)-2 (undefined at endpoints).
    """
    curvatures = []
    for i in range(1, len(path) - 1):
        dx1 = path[i][0] - path[i-1][0]
        dy1 = path[i][1] - path[i-1][1]
        dx2 = path[i+1][0] - path[i][0]
        dy2 = path[i+1][1] - path[i][1]
        seg_len = math.hypot(dx1, dy1) + math.hypot(dx2, dy2)
        if seg_len < 1e-9:
            curvatures.append(0.0)
            continue
        a1 = math.atan2(dy1, dx1)
        a2 = math.atan2(dy2, dx2)
        dtheta = abs(a2 - a1)
        if dtheta > math.pi:
            dtheta = 2 * math.pi - dtheta
        curvatures.append(dtheta / seg_len)
    return curvatures

@staticmethod
def compute_jerk(path: List[Tuple[float, float]]) -> List[float]:
    """Approximated jerk = second difference of curvature values (rad/m²).
    Returns list of length len(path)-4 (empty if path too short).
    """
    kappas = BasePlanner.compute_curvature(path)
    if len(kappas) < 3:
        return []
    return [abs(kappas[i+1] - kappas[i]) for i in range(len(kappas) - 1)]

@staticmethod
def count_turns(path: List[Tuple[float, float]], threshold_rad: float = 0.3) -> int:
    """Count heading changes exceeding threshold_rad (default ~17°)."""
    if len(path) < 3:
        return 0
    count = 0
    for i in range(1, len(path) - 1):
        dx1 = path[i][0] - path[i-1][0]; dy1 = path[i][1] - path[i-1][1]
        dx2 = path[i+1][0] - path[i][0];  dy2 = path[i+1][1] - path[i][1]
        a1 = math.atan2(dy1, dx1); a2 = math.atan2(dy2, dx2)
        diff = abs(a2 - a1)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        if diff >= threshold_rad:
            count += 1
    return count
```

### Update `_record_stats` in every planner (A*, Dijkstra, and all Phase-1 planners)

Replace the existing minimal dict with:
```python
def _record_stats(self, nodes_expanded, elapsed_ms, path_length_m, path=None):
    curvatures = BasePlanner.compute_curvature(path) if path and len(path) >= 3 else []
    jerks      = BasePlanner.compute_jerk(path)      if path and len(path) >= 5 else []
    turns      = BasePlanner.count_turns(path)        if path and len(path) >= 3 else 0
    self._stats = {
        "algorithm":        self.get_name(),
        "nodes_expanded":   nodes_expanded,
        "planning_time_ms": round(elapsed_ms, 3),
        # --- path length ---
        "path_length_m":    round(path_length_m, 4),
        "path_steps":       len(path) if path else 0,
        # --- smoothness ---
        "curvature_max":    round(max(curvatures), 4) if curvatures else 0.0,
        "curvature_mean":   round(sum(curvatures)/len(curvatures), 4) if curvatures else 0.0,
        "jerk_max":         round(max(jerks), 4) if jerks else 0.0,
        "jerk_mean":        round(sum(jerks)/len(jerks), 4) if jerks else 0.0,
        "num_turns":        turns,
    }
```

Pass `path=smoothed` in every planner's `plan()` return branch.

---

## Task 2 — Upgrade `anhc_benchmark_runner_node.py`

### New CSV fieldnames

Replace `_CSV_FIELDNAMES` with:
```python
_CSV_FIELDNAMES = [
    # identification
    "scenario_name", "algorithm_name", "trial_idx",
    "map_name", "start_x", "start_y", "goal_x", "goal_y",
    # path length / distance
    "path_length_m", "path_steps",
    # planning time
    "planning_time_ms", "nodes_expanded", "planning_success",
    # execution
    "execution_time_s", "success", "collision",
    # CPU / memory
    "cpu_usage_percent", "memory_mb",
    # smoothness
    "curvature_max", "curvature_mean",
    "jerk_max", "jerk_mean",
    "num_turns", "path_smoothness_total_rad",
    # safety
    "clearance_avg_m", "clearance_min_m", "safety_score",
]
```

### Smoothness metrics (from planner stats)
Map these directly from the JSON stats published on `/planning/stats`:
- `curvature_max`, `curvature_mean`, `jerk_max`, `jerk_mean`, `num_turns`
- `path_smoothness_total_rad` — computed in runner as sum of abs heading changes (keep existing `_compute_smoothness`)

### Safety metrics (computed in runner)
Replace `_compute_clearance` with two methods:

```python
@staticmethod
def _compute_clearance_metrics(
    path_points: list,
    costmap: OccupancyGrid | None,
) -> tuple[float, float, float]:
    """Return (clearance_avg_m, clearance_min_m, safety_score).

    safety_score = clearance_avg_m / inflation_radius (normalised 0–1 approx).
    Returns (-1.0, -1.0, 0.0) if data unavailable.
    """
    if not path_points or costmap is None:
        return -1.0, -1.0, 0.0

    import numpy as np
    info = costmap.info
    data = np.array(costmap.data, dtype=np.float32).reshape(info.height, info.width)

    # obstacle cells: cost > 0 (includes inflation bands)
    occ_y, occ_x = np.where(data > 0)
    if len(occ_x) == 0:
        return -1.0, -1.0, 0.0

    occ_wx = info.origin.position.x + (occ_x + 0.5) * info.resolution
    occ_wy = info.origin.position.y + (occ_y + 0.5) * info.resolution

    clearances = []
    for px, py in path_points:
        dists = np.hypot(occ_wx - px, occ_wy - py)
        clearances.append(float(np.min(dists)))

    avg = float(np.mean(clearances))
    mn  = float(np.min(clearances))
    # Normalise by a typical inflation radius of 1.4 m
    safety = min(1.0, avg / 1.4)
    return avg, mn, safety
```

Update `_run_trial` to call this and populate all three safety fields.

### CPU monitoring — use system-wide measurement

Replace per-process CPU sampling with:
```python
import psutil
cpu_samples.append(psutil.cpu_percent(interval=None))  # system-wide %
```
This is more meaningful when multiple ROS nodes run in parallel.

### Extended scenario file

Update `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml` to include all 8 algorithms:
```yaml
scenario_1:
  name: "short_path_clear"
  start: [0.0, 0.0]
  goal: [4.0, 0.0]
  algorithms: [astar, dijkstra, theta_star, greedy_bfs, jps, rrt_star, dstar_lite, prm]
  trials: 5

scenario_2:
  name: "long_path_with_obstacles"
  start: [0.0, 0.0]
  goal: [8.0, 6.0]
  algorithms: [astar, dijkstra, theta_star, greedy_bfs, jps, rrt_star, dstar_lite, prm]
  trials: 5

scenario_3:
  name: "narrow_corridor"
  start: [1.0, 1.0]
  goal: [9.0, -5.0]
  algorithms: [astar, dijkstra, theta_star, greedy_bfs, jps, rrt_star, dstar_lite, prm]
  trials: 3

scenario_4:
  name: "office_v2_cross_room"
  start: [0.0, 0.0]
  goal: [10.0, 8.0]
  algorithms: [astar, dijkstra, theta_star, greedy_bfs, jps, rrt_star, dstar_lite, prm]
  trials: 3
```

---

## Task 3 — Upgrade `anhc_results_analyzer_node.py`

Read the CSV and compute per-algorithm **summary statistics** (mean ± std for every metric).  
Publish a `std_msgs/String` (JSON) on `/benchmark/summary` with structure:
```json
{
  "scenario_name": {
    "algorithm_name": {
      "path_length_m":    {"mean": 0.0, "std": 0.0},
      "planning_time_ms": {"mean": 0.0, "std": 0.0},
      "execution_time_s": {"mean": 0.0, "std": 0.0},
      "cpu_usage_percent":{"mean": 0.0, "std": 0.0},
      "curvature_mean":   {"mean": 0.0, "std": 0.0},
      "jerk_mean":        {"mean": 0.0, "std": 0.0},
      "num_turns":        {"mean": 0.0, "std": 0.0},
      "clearance_avg_m":  {"mean": 0.0, "std": 0.0},
      "safety_score":     {"mean": 0.0, "std": 0.0}
    }
  }
}
```

Also write a `benchmark_summary_<timestamp>.json` file alongside the CSV.

---

## Task 4 — Upgrade `anhc_plot_results.py`

Rewrite `src/anhc_viz/scripts/anhc_plot_results.py` to produce **6 subplot figure** using `matplotlib`:

```
Figure layout (2 rows × 3 columns):
┌─────────────────┬──────────────────┬──────────────────┐
│ Path Length (m) │ Planning Time    │ Execution Time   │
│   bar chart     │ (ms) bar chart   │ (s) bar chart    │
├─────────────────┼──────────────────┼──────────────────┤
│ CPU Usage (%)   │ Smoothness       │ Safety Score     │
│   bar chart     │ (curvature_mean) │ (clearance_avg)  │
│                 │ bar chart        │ bar chart        │
└─────────────────┴──────────────────┴──────────────────┘
```

Requirements:
- Accept `--csv <path>` and `--scenario <name>` as command-line arguments; if no `--scenario`, show all scenarios side by side.
- Each bar group = one algorithm; bars within a group = scenarios (or vice versa — choose the clearer layout).
- Error bars = ± 1 std dev (from multiple trials).
- Save figure as `benchmark_comparison_<scenario>_<timestamp>.png` in the same directory as the CSV.
- Also show the figure interactively (`plt.show()`).
- Use a colour palette with 8 distinct colours (one per algorithm).
- Add a legend and axis labels with units.

### Usage after implementation:
```bash
python3 ~/anhc_botrl/src/anhc_viz/scripts/anhc_plot_results.py \
  --csv ~/anhc_benchmark_results/benchmark_<timestamp>.csv \
  --scenario short_path_clear
```

---

## Task 5 — Live Metrics Node

Upgrade `src/anhc_benchmark/anhc_benchmark/anhc_live_metrics_node.py` to:
- Subscribe to `/planning/stats` (JSON string) and forward all new fields.
- Compute and publish rolling averages for CPU and memory over a 5-second window.
- Publish on `/benchmark/live_metrics` as `std_msgs/String` (JSON) at 2 Hz.

---

## Verification

```bash
cd ~/anhc_botrl
colcon build --symlink-install --packages-select anhc_planning anhc_benchmark anhc_viz
source install/setup.bash

# Run full benchmark (requires full system to be running separately)
ros2 launch anhc_benchmark anhc_benchmark.launch.py \
  scenario_file:=$(ros2 pkg prefix anhc_benchmark)/share/anhc_benchmark/config/anhc_benchmark_scenarios.yaml

# After completion, analyse and plot
python3 src/anhc_viz/scripts/anhc_plot_results.py \
  --csv ~/anhc_benchmark_results/benchmark_<latest>.csv

# Inspect live metrics during a run
ros2 topic echo /benchmark/live_metrics
ros2 topic echo /planning/stats
```

Expected output: 6-subplot comparison figure saved as PNG with all 8 algorithms visible.
