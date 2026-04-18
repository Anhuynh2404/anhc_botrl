"""D* Lite incremental re-planning algorithm (Koenig & Likhachev 2002)."""

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
_INF = float("inf")

# Sentinel for lazy-deletion priority queue
_REMOVED = object()


class DStarLitePlanner(BasePlanner):
    """Incremental re-planning algorithm for dynamic environments (D* Lite).

    Plans backward from *goal* to *start* so that incremental updates
    when the costmap changes are cheap.  Expose :meth:`update_costmap`
    to trigger an incremental replan without rebuilding the search tree
    from scratch.

    Parameters
    ----------
    obstacle_threshold:
        Cells with cost >= this value are treated as impassable.
    allow_diagonal:
        Enable 8-connectivity when True; 4-connectivity otherwise.
    """

    def __init__(
        self,
        obstacle_threshold: int = _OBSTACLE_THRESHOLD,
        allow_diagonal: bool = True,
        path_smooth_data_weight: float = 0.5,
        path_smooth_smooth_weight: float = 0.3,
        path_smooth_min_waypoints: int = 5,
    ) -> None:
        self._obstacle_threshold = obstacle_threshold
        self._allow_diagonal = allow_diagonal
        self._path_smooth_data_weight = path_smooth_data_weight
        self._path_smooth_smooth_weight = path_smooth_smooth_weight
        self._path_smooth_min_waypoints = path_smooth_min_waypoints
        self._stats: Dict = {}

        # D* Lite search state (initialised on first plan call)
        self._g: Dict[Tuple[int, int], float] = {}
        self._rhs: Dict[Tuple[int, int], float] = {}
        self._km: float = 0.0
        # Open set: heap of [key, counter, node] with lazy deletion.
        # The unique counter prevents heapq from ever comparing `node`.
        self._heap: List = []
        self._open_set: Dict[Tuple[int, int], list] = {}
        self._heap_counter: int = 0

        # Map state
        self._data: Optional[List[int]] = None
        self._width: int = 0
        self._height: int = 0
        self._resolution: float = 1.0
        self._ox: float = 0.0
        self._oy: float = 0.0

        self._goal_cell: Optional[Tuple[int, int]] = None
        self._last_start: Optional[Tuple[int, int]] = None
        self._initialized: bool = False

    # ------------------------------------------------------------------
    # BasePlanner interface
    # ------------------------------------------------------------------

    def get_name(self) -> str:
        """Return the canonical algorithm identifier."""
        return "dstar_lite"

    def get_params(self) -> Dict:
        """Return a dictionary of the planner's current configuration."""
        return {
            "obstacle_threshold": self._obstacle_threshold,
            "allow_diagonal": self._allow_diagonal,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Run D* Lite and return a smoothed world-frame path.

        On the first call (or when the goal changes) the full backward
        search is executed.  Subsequent calls with the same goal reuse
        the search tree, only updating it for costmap changes.

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

        need_reinit = (
            not self._initialized
            or goal_cell != self._goal_cell
            or self._data is None
        )

        self._width = width
        self._height = height
        self._resolution = resolution
        self._ox = ox
        self._oy = oy
        self._data = data

        if need_reinit:
            self._ds_initialize(start_cell, goal_cell)
        else:
            # Incremental: account for heuristic change due to start move
            self._km += self._h(self._last_start, start_cell)
            self._last_start = start_cell

        nodes_expanded = self._compute_shortest_path(start_cell)

        if self._g_val(start_cell) == _INF:
            elapsed_ms = (time.monotonic() - t0) * 1e3
            self._record_stats(nodes_expanded, elapsed_ms, 0.0)
            return []

        # Extract path by greedy descent on g values
        path_cells = self._extract_path(start_cell, goal_cell)
        if not path_cells:
            elapsed_ms = (time.monotonic() - t0) * 1e3
            self._record_stats(nodes_expanded, elapsed_ms, 0.0)
            return []

        world_path = [
            self.grid_to_world(r, c, ox, oy, resolution)
            for r, c in path_cells
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

    def update_costmap(self, new_costmap: OccupancyGrid) -> None:
        """Update the internal costmap and mark affected vertices.

        Call this when the environment changes.  The next :meth:`plan`
        call will run an incremental replan rather than a full restart.
        """
        if not self._initialized or self._data is None:
            return

        info = new_costmap.info
        new_data: List[int] = [
            -1 if v == -1 else (v + 256 if v < 0 else v)
            for v in new_costmap.data
        ]

        changed: List[Tuple[int, int]] = []
        for r in range(self._height):
            for c in range(self._width):
                idx = r * self._width + c
                if (
                    idx < len(self._data)
                    and idx < len(new_data)
                    and self._data[idx] != new_data[idx]
                ):
                    changed.append((r, c))

        if not changed:
            return

        self._width = info.width
        self._height = info.height
        self._resolution = info.resolution
        self._ox = info.origin.position.x
        self._oy = info.origin.position.y
        self._data = new_data

        if self._last_start is None:
            return

        nbrs = _NEIGHBOURS if self._allow_diagonal else _NEIGHBOURS[:4]
        for cell in changed:
            r, c = cell
            for dr, dc, _ in nbrs:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self._height and 0 <= nc < self._width:
                    self._update_vertex((nr, nc), self._last_start)
            self._update_vertex(cell, self._last_start)

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
    # D* Lite core
    # ------------------------------------------------------------------

    def _ds_initialize(
        self,
        s_start: Tuple[int, int],
        s_goal: Tuple[int, int],
    ) -> None:
        """Reset all D* Lite state for a new goal."""
        self._g = {}
        self._rhs = {}
        self._km = 0.0
        self._heap = []
        self._open_set = {}
        self._heap_counter = 0
        self._goal_cell = s_goal
        self._rhs[s_goal] = 0.0
        self._ds_push(s_goal, self._key(s_goal, s_start))
        self._last_start = s_start
        self._initialized = True

    def _key(
        self,
        s: Tuple[int, int],
        s_start: Tuple[int, int],
    ) -> Tuple[float, float]:
        """Compute the D* Lite priority key for node s."""
        min_val = min(self._g_val(s), self._rhs_val(s))
        return (min_val + self._h(s, s_start) + self._km, min_val)

    @staticmethod
    def _h(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Octile / Euclidean heuristic between grid cells."""
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return math.sqrt(dr * dr + dc * dc)

    def _g_val(self, s: Tuple[int, int]) -> float:
        return self._g.get(s, _INF)

    def _rhs_val(self, s: Tuple[int, int]) -> float:
        return self._rhs.get(s, _INF)

    def _edge_cost(
        self,
        s1: Tuple[int, int],
        s2: Tuple[int, int],
    ) -> float:
        """Edge traversal cost from s1 to s2 on the current costmap."""
        r2, c2 = s2
        if self._data is None:
            return _INF
        idx = r2 * self._width + c2
        if idx < 0 or idx >= len(self._data):
            return _INF
        val = self._data[idx]
        if val < 0:
            return _INF
        if val >= self._obstacle_threshold:
            return _INF
        dr = abs(s2[0] - s1[0])
        dc = abs(s2[1] - s1[1])
        move = 1.4142 if dr == 1 and dc == 1 else 1.0
        return move + max(0, val) * _COST_SCALE

    def _nbrs(self, s: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Return valid grid neighbours of s."""
        r, c = s
        pattern = _NEIGHBOURS if self._allow_diagonal else _NEIGHBOURS[:4]
        result = []
        for dr, dc, _ in pattern:
            nr, nc = r + dr, c + dc
            if 0 <= nr < self._height and 0 <= nc < self._width:
                result.append((nr, nc))
        return result

    # -- Priority queue (lazy deletion) --

    def _ds_push(
        self,
        s: Tuple[int, int],
        k: Tuple[float, float],
    ) -> None:
        if s in self._open_set:
            old = self._open_set[s]
            old[2] = _REMOVED  # mark old entry as removed
        self._heap_counter += 1
        # [key, counter, node] — counter ensures node is never compared
        entry = [k, self._heap_counter, s]
        self._open_set[s] = entry
        heapq.heappush(self._heap, entry)

    def _ds_remove(self, s: Tuple[int, int]) -> None:
        if s in self._open_set:
            entry = self._open_set.pop(s)
            entry[2] = _REMOVED

    def _top_key(self) -> Tuple[float, float]:
        while self._heap:
            k, _cnt, node = self._heap[0]
            if node is _REMOVED:
                heapq.heappop(self._heap)
                continue
            if node in self._open_set:
                return k
            heapq.heappop(self._heap)
        return (_INF, _INF)

    def _ds_pop(
        self,
    ) -> Tuple[Optional[Tuple[float, float]], Optional[Tuple[int, int]]]:
        while self._heap:
            k, _cnt, node = heapq.heappop(self._heap)
            if node is _REMOVED:
                continue
            if node in self._open_set:
                del self._open_set[node]
                return k, node
        return None, None

    # -- Core algorithm --

    def _update_vertex(
        self,
        u: Tuple[int, int],
        s_start: Tuple[int, int],
    ) -> None:
        if u != self._goal_cell:
            best = _INF
            for v in self._nbrs(u):
                c = self._edge_cost(u, v) + self._g_val(v)
                if c < best:
                    best = c
            self._rhs[u] = best
        self._ds_remove(u)
        if self._g_val(u) != self._rhs_val(u):
            self._ds_push(u, self._key(u, s_start))

    def _compute_shortest_path(
        self, s_start: Tuple[int, int]
    ) -> int:
        """Run D* Lite ComputeShortestPath.  Returns nodes_expanded."""
        nodes_expanded = 0
        while True:
            top_k = self._top_key()
            k_start = self._key(s_start, s_start)
            rhs_s = self._rhs_val(s_start)
            g_s = self._g_val(s_start)
            if top_k >= k_start and rhs_s == g_s:
                break
            k_old, u = self._ds_pop()
            if u is None:
                break
            nodes_expanded += 1
            k_new = self._key(u, s_start)
            if k_old < k_new:
                self._ds_push(u, k_new)
            elif self._g_val(u) > self._rhs_val(u):
                self._g[u] = self._rhs_val(u)
                for s in self._nbrs(u):
                    self._update_vertex(s, s_start)
            else:
                self._g[u] = _INF
                for s in self._nbrs(u) + [u]:
                    self._update_vertex(s, s_start)
        return nodes_expanded

    def _extract_path(
        self,
        s_start: Tuple[int, int],
        s_goal: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Greedily follow minimum-cost successors from start to goal."""
        path = [s_start]
        visited = {s_start}
        current = s_start
        max_steps = self._width * self._height
        for _ in range(max_steps):
            if current == s_goal:
                break
            best_next: Optional[Tuple[int, int]] = None
            best_cost = _INF
            for v in self._nbrs(current):
                c = self._edge_cost(current, v) + self._g_val(v)
                if c < best_cost:
                    best_cost = c
                    best_next = v
            if best_next is None or best_next in visited:
                return []
            visited.add(best_next)
            path.append(best_next)
            current = best_next
        return path if current == s_goal else []
