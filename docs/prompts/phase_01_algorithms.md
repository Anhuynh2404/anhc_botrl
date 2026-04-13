# Phase 1 — Implement Popular Path-Planning Algorithms

## Context

This is a ROS 2 (Jazzy) mobile-robot project at `/home/anhuynh/anhc_botrl`.

The planning package is `src/anhc_planning/`.  
Two algorithms are already **fully working**: `AStarPlanner` (`astar_planner.py`) and `DijkstraPlanner` (`dijkstra_planner.py`).  
Two stubs exist but are **not implemented** yet: `RRTStarPlanner` (`rrt_star_planner.py`) and `DStarLitePlanner` (`dstar_lite_planner.py`).  
One more stub exists for RL: `rl_planner.py` — leave it as is.

**Key files to read before starting:**
- `src/anhc_planning/anhc_planning/planners/base_planner.py` — abstract interface every planner must follow
- `src/anhc_planning/anhc_planning/planners/astar_planner.py` — reference implementation
- `src/anhc_planning/anhc_planning/planners/dijkstra_planner.py` — reference implementation
- `src/anhc_planning/anhc_planning/planners/__init__.py` — registry exports
- `src/anhc_planning/anhc_planning/anhc_global_planner_node.py` — how planners are instantiated (`_PLANNER_REGISTRY`, `_build_planner`)

## Goal

Implement the following **6 algorithms** as planner plugins that:
1. Inherit from `BasePlanner` (same interface as A* and Dijkstra).
2. Operate on `nav_msgs/OccupancyGrid` with the same coordinate helpers (`world_to_grid`, `grid_to_world`, etc.) from `BasePlanner`.
3. Treat cells with cost `>= obstacle_threshold` (default 100) as impassable.
4. Apply inflation-aware traversal cost: `cell_cost = max(0, data[idx]) * (1.0/50.0)` (same as A* and Dijkstra).
5. Use `BasePlanner.smooth_path()` on the result before returning.
6. Record stats in `self._stats` via a `_record_stats(nodes_expanded, elapsed_ms, path_length_m)` method.
7. Expose a `stats` property returning `dict(self._stats)`.

### Algorithms to implement

| File to create/complete | Class | Registry key | Category |
|---|---|---|---|
| `rrt_star_planner.py` (exists as stub) | `RRTStarPlanner` | `rrt_star` | Sampling-based |
| `dstar_lite_planner.py` (exists as stub) | `DStarLitePlanner` | `dstar_lite` | Incremental graph |
| `theta_star_planner.py` (NEW) | `ThetaStarPlanner` | `theta_star` | Any-angle A* |
| `greedy_bfs_planner.py` (NEW) | `GreedyBFSPlanner` | `greedy_bfs` | Heuristic search |
| `jps_planner.py` (NEW) | `JPSPlanner` | `jps` | Jump Point Search |
| `prm_planner.py` (NEW) | `PRMPlanner` | `prm` | Sampling-based |

---

## Algorithm Specifications

### 1. RRT* (`rrt_star_planner.py`)
- Full file replacement of the current stub.
- Parameters: `max_iterations=5000`, `step_size=0.3` (m), `goal_sample_rate=0.1`, `search_radius=1.0` (m), `obstacle_threshold=100`.
- Convert `OccupancyGrid` extent to world-frame bounding box for random sampling.
- Use `BasePlanner.is_valid_cell` for collision checks; also check the straight line between nodes (sample several intermediate points at resolution intervals).
- Rewiring step: within `search_radius`, reconnect neighbours if cost via new node is lower.
- Terminate early if a node lands within `step_size` of goal.
- Return `smooth_path(world_path)`.
- Stats: `nodes_expanded` = tree size at termination.

### 2. D* Lite (`dstar_lite_planner.py`)
- Full file replacement of the current stub.
- Parameters: `obstacle_threshold=100`, `allow_diagonal=True`.
- Standard D* Lite (Koenig & Likhachev 2002): plan backwards from goal to start.
- Use the same 8-connected (or 4-connected) `_NEIGHBOURS` grid as A*/Dijkstra.
- Expose an `update_costmap(new_costmap)` method for incremental replanning (node can call this when costmap changes).
- Return `smooth_path(world_path)`.
- Stats: `nodes_expanded` = number of vertices processed in `ComputeShortestPath`.

### 3. Theta* (`theta_star_planner.py`) — Any-Angle A*
- New file.
- Parameters: `allow_diagonal=True`, `path_smooth_data_weight=0.5`, `path_smooth_smooth_weight=0.3`, `path_smooth_min_waypoints=5`, `obstacle_threshold=100`.
- Extension of A*: in the neighbour relaxation step, if the parent of `current` has line-of-sight to `neighbour`, use the straight-line cost and parent pointer instead of going through `current`.
- Line-of-sight check: Bresenham's line algorithm on the grid; reject if any cell along the line has `cost >= obstacle_threshold`.
- Return `smooth_path(world_path)`.
- Key difference from A*: waypoints are NOT constrained to grid centres — they follow any-angle straight paths.

### 4. Greedy Best-First Search (`greedy_bfs_planner.py`)
- New file.
- Parameters: same as Dijkstra but no smoothing weight tuning needed (keep defaults).
- Pure greedy search: priority queue ordered only by heuristic `h = euclidean(current, goal)`. No `g` cost in the priority.
- Same 8-connected grid, same obstacle/cost model.
- NOT optimal — but fast. Good benchmark contrast point.
- Return `smooth_path(world_path)`.

### 5. Jump Point Search (`jps_planner.py`)
- New file.
- Parameters: `obstacle_threshold=100`, `path_smooth_data_weight=0.5`, `path_smooth_smooth_weight=0.3`, `path_smooth_min_waypoints=5`.
- Standard JPS on uniform-cost 8-connected grid (Harabor & Grastien 2011).
- Pruning rules for cardinal and diagonal moves.
- Jump function: recursively scan in a direction until a jump point is found (forced neighbour or goal).
- Return `smooth_path(world_path)`.
- Stats: `nodes_expanded` = number of jump points processed.

### 6. PRM — Probabilistic Roadmap (`prm_planner.py`)
- New file.
- Parameters: `n_samples=500`, `connection_radius=1.5` (m), `obstacle_threshold=100`.
- Build phase: scatter `n_samples` random collision-free samples across the map; connect pairs within `connection_radius` if straight-line is collision-free.
- Query phase: connect start and goal to the roadmap (find nearest `k=5` nodes); run Dijkstra on the roadmap graph.
- Cache the roadmap (re-build only when costmap changes — compare header stamp).
- Return `smooth_path(world_path)`.
- Stats: `nodes_expanded` = roadmap nodes traversed in Dijkstra query phase.

---

## Registration

After creating all planner files, update these files:

### `src/anhc_planning/anhc_planning/planners/__init__.py`
Add imports and `__all__` entries for `ThetaStarPlanner`, `GreedyBFSPlanner`, `JPSPlanner`, `PRMPlanner`.

### `src/anhc_planning/anhc_planning/anhc_global_planner_node.py`
1. Add imports for all new planners.
2. Add entries to `_PLANNER_REGISTRY`:
   ```python
   "rrt_star":   RRTStarPlanner,
   "dstar_lite": DStarLitePlanner,
   "theta_star": ThetaStarPlanner,
   "greedy_bfs": GreedyBFSPlanner,
   "jps":        JPSPlanner,
   "prm":        PRMPlanner,
   ```
3. In `_build_planner`, handle each new class — pass `obstacle_threshold` and smoothing params where applicable; use sensible defaults for sampling-based planners.
4. Update the `algorithm` parameter description string to list all 8 options.

### `src/anhc_planning/config/planner_params.yaml`
Add parameter blocks for each new algorithm (e.g. `rrt_star_max_iterations`, `rrt_star_step_size`, `prm_n_samples`, etc.).

---

## Quality Requirements

- All files must pass `flake8` and `pep257` with the existing project configuration.
- No external dependencies beyond what is already in the workspace (`numpy`, `scipy`).  
  Sampling-based planners may use `random` from the standard library.
- Every planner must handle edge cases: start == goal, start/goal in obstacle, no path found.
- Include docstrings on the class and `plan()` method matching the style of `astar_planner.py`.

---

## Verification

After implementation, verify with:
```bash
cd ~/anhc_botrl
colcon build --symlink-install --packages-select anhc_planning
source install/setup.bash
# Quick smoke test — import all planners
python3 -c "
from anhc_planning.planners import (
    AStarPlanner, DijkstraPlanner, RRTStarPlanner, DStarLitePlanner,
    ThetaStarPlanner, GreedyBFSPlanner, JPSPlanner, PRMPlanner
)
print('All planners imported successfully')
"
```

Switch algorithms at runtime (no restart needed):
```bash
ros2 param set /anhc_global_planner algorithm theta_star
ros2 param set /anhc_global_planner algorithm rrt_star
ros2 param set /anhc_global_planner algorithm jps
```
