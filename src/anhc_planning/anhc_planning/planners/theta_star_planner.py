"""Theta* any-angle path planner — extension of A* with line-of-sight."""

import heapq
import math
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

_OBSTACLE_THRESHOLD = 100
_COST_SCALE = 1.0 / 50.0


class ThetaStarPlanner(BasePlanner):
    """Any-angle A* planner using Theta* line-of-sight shortcutting.

    In the neighbour-relaxation step, if the *grandparent* of the
    current node has line-of-sight to the neighbour (verified with
    Bresenham's algorithm), the grandparent is used as the direct
    parent instead of the current node.  This allows waypoints to
    follow straight any-angle paths rather than grid-constrained
    zigzags.

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
        Cells with cost >= this value are treated as impassable.
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
        """Return the canonical algorithm identifier."""
        return "theta_star"

    def get_params(self) -> Dict:
        """Return a dictionary of the planner's current configuration."""
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
        """Run Theta* and return a smoothed any-angle world-frame path.

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

        start_cell = self.world_to_grid(
            start[0], start[1], ox, oy, resolution
        )
        goal_cell = self.world_to_grid(
            goal[0], goal[1], ox, oy, resolution
        )

        if not self.is_valid_cell(
            start_cell[0], start_cell[1],
            height, width, data, self._obstacle_threshold,
        ):
            snapped = self.nearest_valid_cell(
                start_cell, height, width, data, self._obstacle_threshold
            )
            if snapped is None:
                self._record_stats(0, (time.monotonic() - t0) * 1e3, 0.0)
                return []
            start_cell = snapped

        if not self.is_valid_cell(
            goal_cell[0], goal_cell[1],
            height, width, data, self._obstacle_threshold,
        ):
            snapped = self.nearest_valid_cell(
                goal_cell, height, width, data, self._obstacle_threshold
            )
            if snapped is None:
                self._record_stats(0, (time.monotonic() - t0) * 1e3, 0.0)
                return []
            goal_cell = snapped

        if start_cell == goal_cell:
            world = self.grid_to_world(
                start_cell[0], start_cell[1], ox, oy, resolution
            )
            self._record_stats(0, (time.monotonic() - t0) * 1e3, 0.0)
            return [world]

        # open heap: (f, g, (row, col))
        g_start = 0.0
        h_start = self.heuristic(start_cell, goal_cell)
        open_heap: List = []
        heapq.heappush(open_heap, (g_start + h_start, g_start, start_cell))

        g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        came_from: Dict[
            Tuple[int, int], Optional[Tuple[int, int]]
        ] = {start_cell: None}
        nodes_expanded = 0

        nbrs = _NEIGHBOURS if self._allow_diagonal else _NEIGHBOURS[:4]

        while open_heap:
            _f, g_curr, current = heapq.heappop(open_heap)

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
                elapsed_ms = (time.monotonic() - t0) * 1e3
                self._record_stats(nodes_expanded, elapsed_ms, length)
                return smoothed

            if g_curr > g_score.get(current, float("inf")):
                continue

            nodes_expanded += 1
            r, c = current
            parent = came_from.get(current)

            for dr, dc, move_cost in nbrs:
                nr, nc = r + dr, c + dc
                if not self.is_valid_cell(
                    nr, nc, height, width, data, self._obstacle_threshold
                ):
                    continue
                neighbour = (nr, nc)
                cell_cost = (
                    max(0, data[nr * width + nc]) * _COST_SCALE
                )

                # Theta*: try grandparent shortcut via line-of-sight
                if parent is not None and self._line_of_sight(
                    parent[0], parent[1], nr, nc,
                    height, width, data,
                ):
                    pr, pc = parent
                    straight = math.sqrt(
                        (nr - pr) ** 2 + (nc - pc) ** 2
                    )
                    tentative_g = g_score[parent] + straight + cell_cost
                    if tentative_g < g_score.get(neighbour, float("inf")):
                        g_score[neighbour] = tentative_g
                        came_from[neighbour] = parent
                        h = self.heuristic(neighbour, goal_cell)
                        heapq.heappush(
                            open_heap,
                            (tentative_g + h, tentative_g, neighbour),
                        )
                    continue

                # Standard A* relaxation
                tentative_g = (
                    g_score[current] + move_cost + cell_cost
                )
                if tentative_g < g_score.get(neighbour, float("inf")):
                    g_score[neighbour] = tentative_g
                    came_from[neighbour] = current
                    h = self.heuristic(neighbour, goal_cell)
                    heapq.heappush(
                        open_heap,
                        (tentative_g + h, tentative_g, neighbour),
                    )

        elapsed_ms = (time.monotonic() - t0) * 1e3
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
        self,
        nodes_expanded: int,
        planning_time_ms: float,
        path_length_m: float,
    ) -> None:
        self._stats = {
            "algorithm": self.get_name(),
            "nodes_expanded": nodes_expanded,
            "planning_time_ms": round(planning_time_ms, 3),
            "path_length_m": round(path_length_m, 4),
        }

    # ------------------------------------------------------------------
    # Line-of-sight check (Bresenham's algorithm)
    # ------------------------------------------------------------------

    def _line_of_sight(
        self,
        r0: int,
        c0: int,
        r1: int,
        c1: int,
        height: int,
        width: int,
        data: List[int],
    ) -> bool:
        """Return True if the grid segment (r0,c0)→(r1,c1) is obstacle-free.

        Uses Bresenham's line algorithm; any cell along the line that
        meets or exceeds *obstacle_threshold* causes a False return.
        """
        dr = abs(r1 - r0)
        dc = abs(c1 - c0)
        r_step = 1 if r1 >= r0 else -1
        c_step = 1 if c1 >= c0 else -1
        err = dr - dc
        r, c = r0, c0
        while True:
            if not self.is_valid_cell(
                r, c, height, width, data, self._obstacle_threshold
            ):
                return False
            if r == r1 and c == c1:
                return True
            e2 = 2 * err
            move_r = e2 > -dc
            move_c = e2 < dr
            # Diagonal step: also check the two corner-adjacent cells to
            # prevent the path from cutting through diagonal obstacle gaps.
            if move_r and move_c:
                if not self.is_valid_cell(
                    r + r_step, c,
                    height, width, data, self._obstacle_threshold,
                ) or not self.is_valid_cell(
                    r, c + c_step,
                    height, width, data, self._obstacle_threshold,
                ):
                    return False
            if move_r:
                err -= dc
                r += r_step
            if move_c:
                err += dr
                c += c_step
