"""Dijkstra path planner (uniform-cost search, no heuristic)."""

import heapq
import time
from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner

# 8-connected neighbour offsets (row_delta, col_delta, move_cost)
_NEIGHBOURS = [
    (-1,  0, 1.0),
    ( 1,  0, 1.0),
    ( 0, -1, 1.0),
    ( 0,  1, 1.0),
    (-1, -1, 1.4142),
    (-1,  1, 1.4142),
    ( 1, -1, 1.4142),
    ( 1,  1, 1.4142),
]

_OBSTACLE_THRESHOLD = 253
_COST_SCALE = 1.0 / 50.0


class DijkstraPlanner(BasePlanner):
    """Pure Dijkstra (uniform-cost search) grid planner.

    Identical cost model and connectivity to :class:`AStarPlanner` — the sole
    difference is the absence of a heuristic, making it an admissible but
    exhaustive baseline for benchmarking.

    Parameters
    ----------
    allow_diagonal:
        Enable 8-connectivity when True; 4-connectivity otherwise.
    path_smooth_data_weight:
        Data retention weight for gradient-descent smoothing.
    path_smooth_smooth_weight:
        Smoothness weight for gradient-descent smoothing.
    path_smooth_min_waypoints:
        Skip smoothing when path is shorter than this.
    obstacle_threshold:
        Cells with cost ≥ this value are treated as impassable.
    """

    def __init__(
        self,
        allow_diagonal: bool = True,
        path_smooth_data_weight: float = 0.5,
        path_smooth_smooth_weight: float = 0.3,
        path_smooth_min_waypoints: int = 5,
        obstacle_threshold: int = _OBSTACLE_THRESHOLD,
    ) -> None:
        self._allow_diagonal = allow_diagonal
        self._path_smooth_data_weight = path_smooth_data_weight
        self._path_smooth_smooth_weight = path_smooth_smooth_weight
        self._path_smooth_min_waypoints = path_smooth_min_waypoints
        self._obstacle_threshold = obstacle_threshold
        self._stats: Dict = {}

    # ------------------------------------------------------------------
    # BasePlanner interface
    # ------------------------------------------------------------------

    def get_name(self) -> str:
        return "dijkstra"

    def get_params(self) -> Dict:
        return {
            "allow_diagonal": self._allow_diagonal,
            "path_smooth_data_weight": self._path_smooth_data_weight,
            "path_smooth_smooth_weight": self._path_smooth_smooth_weight,
            "path_smooth_min_waypoints": self._path_smooth_min_waypoints,
            "obstacle_threshold": self._obstacle_threshold,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Run Dijkstra and return a smoothed world-frame path.

        Returns an empty list when no path can be found.
        """
        t0 = time.monotonic()

        info = costmap.info
        width: int = info.width
        height: int = info.height
        resolution: float = info.resolution
        ox: float = info.origin.position.x
        oy: float = info.origin.position.y
        data: List[int] = [
            -1 if v == -1 else (v + 256 if v < 0 else v)
            for v in costmap.data
        ]

        start_cell = self.world_to_grid(start[0], start[1], ox, oy, resolution)
        goal_cell = self.world_to_grid(goal[0], goal[1], ox, oy, resolution)

        if not self.is_valid_cell(
            start_cell[0], start_cell[1], height, width, data, self._obstacle_threshold
        ):
            self._record_stats(0, time.monotonic() - t0, 0.0)
            return []

        if not self.is_valid_cell(
            goal_cell[0], goal_cell[1], height, width, data, self._obstacle_threshold
        ):
            self._record_stats(0, time.monotonic() - t0, 0.0)
            return []

        # open heap: (g, (row, col))  — no heuristic term
        open_heap: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(open_heap, (0.0, start_cell))

        g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {start_cell: None}
        nodes_expanded = 0

        neighbours = _NEIGHBOURS if self._allow_diagonal else _NEIGHBOURS[:4]

        while open_heap:
            g_curr, current = heapq.heappop(open_heap)

            if current == goal_cell:
                grid_path = self.reconstruct_path(came_from, current)
                world_path = [
                    self.grid_to_world(r, c, ox, oy, resolution)
                    for r, c in grid_path
                ]
                smoothed = self.smooth_path(
                    world_path,
                    data_weight=self._path_smooth_data_weight,
                    smooth_weight=self._path_smooth_smooth_weight,
                    min_waypoints=self._path_smooth_min_waypoints,
                )
                length = self.path_length(smoothed)
                elapsed_ms = (time.monotonic() - t0) * 1000.0
                self._record_stats(nodes_expanded, elapsed_ms, length)
                return smoothed

            if g_curr > g_score.get(current, float("inf")):
                continue

            nodes_expanded += 1
            r, c = current
            for dr, dc, move_cost in neighbours:
                nr, nc = r + dr, c + dc
                if not self.is_valid_cell(nr, nc, height, width, data, self._obstacle_threshold):
                    continue
                cell_cost = max(0, data[nr * width + nc]) * _COST_SCALE
                tentative_g = g_score[current] + move_cost + cell_cost
                neighbour = (nr, nc)
                if tentative_g < g_score.get(neighbour, float("inf")):
                    g_score[neighbour] = tentative_g
                    came_from[neighbour] = current
                    heapq.heappush(open_heap, (tentative_g, neighbour))

        elapsed_ms = (time.monotonic() - t0) * 1000.0
        self._record_stats(nodes_expanded, elapsed_ms, 0.0)
        return []

    # ------------------------------------------------------------------
    # Stats
    # ------------------------------------------------------------------

    @property
    def stats(self) -> Dict:
        """Most recent planning statistics."""
        return dict(self._stats)

    def _record_stats(
        self, nodes_expanded: int, planning_time_ms: float, path_length_m: float
    ) -> None:
        self._stats = {
            "algorithm": self.get_name(),
            "nodes_expanded": nodes_expanded,
            "planning_time_ms": round(planning_time_ms, 3),
            "path_length_m": round(path_length_m, 4),
        }
