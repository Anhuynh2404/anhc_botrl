"""Jump Point Search (JPS) planner for uniform-cost 8-connected grids."""

import heapq
import math
import time
from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner

_OBSTACLE_THRESHOLD = 100

# Move costs used for computing g-values between jump points
_CARDINAL_COST = 1.0
_DIAGONAL_COST = math.sqrt(2.0)


class JPSPlanner(BasePlanner):
    """Jump Point Search (Harabor & Grastien 2011) on an 8-connected grid.

    JPS prunes the search space by exploiting grid symmetry: instead of
    expanding all 8 neighbours, it scans in the movement direction and
    returns only "jump points" — cells that have forced neighbours and
    therefore cannot be ignored.  This dramatically reduces the number
    of nodes added to the open set compared to plain A*.

    .. note::
        JPS is designed for uniform-cost grids.  Inflation costs between
        jump points are not accumulated; only obstacle cells (cost >=
        *obstacle_threshold*) are treated as impassable.

    Parameters
    ----------
    obstacle_threshold:
        Cells with cost >= this value are treated as impassable.
    path_smooth_data_weight:
        Data retention weight for gradient-descent smoothing.
    path_smooth_smooth_weight:
        Smoothness weight for gradient-descent smoothing.
    path_smooth_min_waypoints:
        Skip smoothing when path is shorter than this.
    """

    def __init__(
        self,
        obstacle_threshold: int = _OBSTACLE_THRESHOLD,
        path_smooth_data_weight: float = 0.5,
        path_smooth_smooth_weight: float = 0.3,
        path_smooth_min_waypoints: int = 5,
    ) -> None:
        self._obstacle_threshold = obstacle_threshold
        self._path_smooth_data_weight = path_smooth_data_weight
        self._path_smooth_smooth_weight = path_smooth_smooth_weight
        self._path_smooth_min_waypoints = path_smooth_min_waypoints
        self._stats: Dict = {}

        # Cached map state set during plan()
        self._height: int = 0
        self._width: int = 0
        self._data: List[int] = []

    # ------------------------------------------------------------------
    # BasePlanner interface
    # ------------------------------------------------------------------

    def get_name(self) -> str:
        """Return the canonical algorithm identifier."""
        return "jps"

    def get_params(self) -> Dict:
        """Return a dictionary of the planner's current configuration."""
        return {
            "obstacle_threshold": self._obstacle_threshold,
            "path_smooth_data_weight": self._path_smooth_data_weight,
            "path_smooth_smooth_weight": self._path_smooth_smooth_weight,
            "path_smooth_min_waypoints": self._path_smooth_min_waypoints,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Run Jump Point Search and return a smoothed world-frame path.

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

        # Cache for use in helper methods
        self._height = height
        self._width = width
        self._data = data

        start_cell = self.world_to_grid(
            start[0], start[1], ox, oy, resolution
        )
        goal_cell = self.world_to_grid(
            goal[0], goal[1], ox, oy, resolution
        )

        if not self._walkable(start_cell[0], start_cell[1]):
            snapped = self.nearest_valid_cell(
                start_cell, height, width, data, self._obstacle_threshold
            )
            if snapped is None:
                self._record_stats(0, (time.monotonic() - t0) * 1e3, 0.0)
                return []
            start_cell = snapped

        if not self._walkable(goal_cell[0], goal_cell[1]):
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

        gr, gc = goal_cell

        # open heap: (f, g, counter, (row, col), (parent_dr, parent_dc))
        counter = 0
        h0 = self.heuristic(start_cell, goal_cell)
        open_heap: List = []
        heapq.heappush(
            open_heap, (h0, 0.0, counter, start_cell, (0, 0))
        )

        g_score: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        came_from: Dict[
            Tuple[int, int], Optional[Tuple[int, int]]
        ] = {start_cell: None}
        nodes_expanded = 0

        while open_heap:
            _f, g_curr, _cnt, current, pdir = heapq.heappop(open_heap)

            if current == goal_cell:
                jump_path = self.reconstruct_path(came_from, current)
                grid_path = self._expand_jump_path(jump_path)
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

            for dr, dc in self._successor_dirs(r, c, pdir):
                jp = self._jump(r, c, dr, dc, gr, gc)
                if jp is None:
                    continue
                jr, jc = jp
                move_cost = self._jump_cost(r, c, jr, jc, dr, dc)
                new_g = g_curr + move_cost
                if new_g < g_score.get(jp, float("inf")):
                    g_score[jp] = new_g
                    came_from[jp] = current
                    h = self.heuristic(jp, goal_cell)
                    counter += 1
                    jump_dir = (
                        0 if jr == r else (1 if jr > r else -1),
                        0 if jc == c else (1 if jc > c else -1),
                    )
                    heapq.heappush(
                        open_heap,
                        (new_g + h, new_g, counter, jp, jump_dir),
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
    # JPS core helpers
    # ------------------------------------------------------------------

    def _walkable(self, r: int, c: int) -> bool:
        """Return True if cell (r, c) is in-bounds and non-lethal."""
        if r < 0 or r >= self._height or c < 0 or c >= self._width:
            return False
        val = self._data[r * self._width + c]
        if val < 0:
            return True  # unknown — allow traversal
        return val < self._obstacle_threshold

    def _successor_dirs(
        self,
        r: int,
        c: int,
        parent_dir: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Return the set of directions to probe from (r, c).

        For the start node (parent_dir == (0,0)) all 8 directions are
        returned.  Otherwise only the natural and forced directions for
        the incoming *parent_dir* are returned.
        """
        dr, dc = parent_dir
        if dr == 0 and dc == 0:
            # Start node — expand all
            return [
                (-1, 0), (1, 0), (0, -1), (0, 1),
                (-1, -1), (-1, 1), (1, -1), (1, 1),
            ]

        dirs: List[Tuple[int, int]] = []

        if dr != 0 and dc != 0:
            # Diagonal movement — natural: (dr,0), (0,dc), (dr,dc)
            if self._walkable(r + dr, c):
                dirs.append((dr, 0))
            if self._walkable(r, c + dc):
                dirs.append((0, dc))
            if self._walkable(r + dr, c + dc):
                dirs.append((dr, dc))
            # Forced
            if (
                not self._walkable(r - dr, c)
                and self._walkable(r - dr, c + dc)
            ):
                dirs.append((-dr, dc))
            if (
                not self._walkable(r, c - dc)
                and self._walkable(r + dr, c - dc)
            ):
                dirs.append((dr, -dc))
        elif dr == 0:
            # Horizontal movement — natural: (0, dc)
            dirs.append((0, dc))
            # Forced: perpendicular cells blocked on the opposite side
            if (
                not self._walkable(r - 1, c)
                and self._walkable(r - 1, c + dc)
            ):
                dirs.append((-1, dc))
            if (
                not self._walkable(r + 1, c)
                and self._walkable(r + 1, c + dc)
            ):
                dirs.append((1, dc))
        else:
            # Vertical movement — natural: (dr, 0)
            dirs.append((dr, 0))
            # Forced
            if (
                not self._walkable(r, c - 1)
                and self._walkable(r + dr, c - 1)
            ):
                dirs.append((dr, -1))
            if (
                not self._walkable(r, c + 1)
                and self._walkable(r + dr, c + 1)
            ):
                dirs.append((dr, 1))

        return dirs

    def _jump(
        self,
        r: int,
        c: int,
        dr: int,
        dc: int,
        gr: int,
        gc: int,
    ) -> Optional[Tuple[int, int]]:
        """Scan from (r, c) in direction (dr, dc) until a jump point.

        Returns the jump point coordinates, or None if an obstacle is
        hit before any jump point is found.
        """
        # Iterative scan for cardinal directions; diagonal uses
        # recursion only for the two cardinal sub-scans.
        while True:
            nr, nc = r + dr, c + dc
            if not self._walkable(nr, nc):
                return None
            if nr == gr and nc == gc:
                return (nr, nc)

            if dr != 0 and dc != 0:
                # Diagonal: check for forced neighbours
                if (
                    not self._walkable(nr - dr, nc)
                    and self._walkable(nr - dr, nc + dc)
                ) or (
                    not self._walkable(nr, nc - dc)
                    and self._walkable(nr + dr, nc - dc)
                ):
                    return (nr, nc)
                # Cardinal sub-scans
                if (
                    self._jump(nr, nc, dr, 0, gr, gc) is not None
                    or self._jump(nr, nc, 0, dc, gr, gc) is not None
                ):
                    return (nr, nc)
            elif dr == 0:
                # Horizontal: forced if perpendicular wall
                if (
                    not self._walkable(nr - 1, nc)
                    and self._walkable(nr - 1, nc + dc)
                ) or (
                    not self._walkable(nr + 1, nc)
                    and self._walkable(nr + 1, nc + dc)
                ):
                    return (nr, nc)
            else:
                # Vertical: forced if perpendicular wall
                if (
                    not self._walkable(nr, nc - 1)
                    and self._walkable(nr + dr, nc - 1)
                ) or (
                    not self._walkable(nr, nc + 1)
                    and self._walkable(nr + dr, nc + 1)
                ):
                    return (nr, nc)

            r, c = nr, nc

    @staticmethod
    def _jump_cost(
        r1: int, c1: int,
        r2: int, c2: int,
        dr: int, dc: int,
    ) -> float:
        """Cost of moving from (r1,c1) to jump point (r2,c2)."""
        if dr != 0 and dc != 0:
            steps = abs(r2 - r1)
            return steps * _DIAGONAL_COST
        return (abs(r2 - r1) + abs(c2 - c1)) * _CARDINAL_COST

    @staticmethod
    def _expand_jump_path(
        jump_pts: List[Tuple[int, int]],
    ) -> List[Tuple[int, int]]:
        """Interpolate all grid cells between consecutive jump points."""
        if not jump_pts:
            return []
        cells: List[Tuple[int, int]] = []
        for i in range(len(jump_pts) - 1):
            r1, c1 = jump_pts[i]
            r2, c2 = jump_pts[i + 1]
            dr = 0 if r2 == r1 else (1 if r2 > r1 else -1)
            dc = 0 if c2 == c1 else (1 if c2 > c1 else -1)
            r, c = r1, c1
            while (r, c) != (r2, c2):
                cells.append((r, c))
                r += dr
                c += dc
        cells.append(jump_pts[-1])
        return cells
