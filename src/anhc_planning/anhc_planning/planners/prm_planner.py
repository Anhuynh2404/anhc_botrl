"""Probabilistic Roadmap (PRM) planner with Dijkstra query phase."""

import heapq
import math
import random
import time
from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner

_OBSTACLE_THRESHOLD = 100
_COST_SCALE = 1.0 / 50.0


class PRMPlanner(BasePlanner):
    """Sampling-based Probabilistic Roadmap planner.

    **Build phase** (once per costmap): scatter *n_samples* random
    collision-free samples across the map and connect pairs within
    *connection_radius* metres if the straight line between them is
    obstacle-free.

    **Query phase**: attach *start* and *goal* to the nearest *k*
    roadmap nodes; run Dijkstra on the roadmap graph to find the
    shortest path.

    The roadmap is cached and only rebuilt when the costmap header
    stamp changes.

    Parameters
    ----------
    n_samples:
        Number of random collision-free samples to scatter.
    connection_radius:
        Maximum distance (metres) for connecting two roadmap nodes.
    obstacle_threshold:
        Cells with cost >= this value are treated as impassable.
    """

    def __init__(
        self,
        n_samples: int = 500,
        connection_radius: float = 1.5,
        obstacle_threshold: int = _OBSTACLE_THRESHOLD,
    ) -> None:
        self._n_samples = n_samples
        self._connection_radius = connection_radius
        self._obstacle_threshold = obstacle_threshold
        self._stats: Dict = {}

        # Cached roadmap
        self._roadmap_nodes: List[Tuple[float, float]] = []
        # adj[i] = list of (j, cost)
        self._roadmap_adj: Dict[int, List[Tuple[int, float]]] = {}
        self._cached_stamp: Optional[object] = None

        # Cached map params (refreshed on each plan call)
        self._width: int = 0
        self._height: int = 0
        self._resolution: float = 1.0
        self._ox: float = 0.0
        self._oy: float = 0.0
        self._data: Optional[List[int]] = None

    # ------------------------------------------------------------------
    # BasePlanner interface
    # ------------------------------------------------------------------

    def get_name(self) -> str:
        """Return the canonical algorithm identifier."""
        return "prm"

    def get_params(self) -> Dict:
        """Return a dictionary of the planner's current configuration."""
        return {
            "n_samples": self._n_samples,
            "connection_radius": self._connection_radius,
            "obstacle_threshold": self._obstacle_threshold,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Build / reuse roadmap and run Dijkstra query phase.

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

        self._width = width
        self._height = height
        self._resolution = resolution
        self._ox = ox
        self._oy = oy
        self._data = data

        # Snap start / goal out of obstacles
        start_cell = self.world_to_grid(
            start[0], start[1], ox, oy, resolution
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
            start = self.grid_to_world(
                snapped[0], snapped[1], ox, oy, resolution
            )

        goal_cell = self.world_to_grid(
            goal[0], goal[1], ox, oy, resolution
        )
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
            goal = self.grid_to_world(
                snapped[0], snapped[1], ox, oy, resolution
            )

        if self._dist(start, goal) < 1e-6:
            self._record_stats(0, (time.monotonic() - t0) * 1e3, 0.0)
            return [start, goal]

        # Rebuild roadmap only if costmap changed
        stamp = (
            costmap.header.stamp.sec,
            costmap.header.stamp.nanosec,
        )
        if stamp != self._cached_stamp or not self._roadmap_nodes:
            self._build_roadmap(ox, oy, width, height, resolution, data)
            self._cached_stamp = stamp

        # Query: attach start and goal as temporary nodes
        nodes_expanded = self._dijkstra_query(start, goal)

        if nodes_expanded < 0:
            elapsed_ms = (time.monotonic() - t0) * 1e3
            self._record_stats(
                abs(nodes_expanded), elapsed_ms, 0.0
            )
            return []

        # Path was stored in self._last_path during query
        world_path = self._last_path
        smoothed = self.smooth_path(world_path)
        length = self.path_length(smoothed)
        elapsed_ms = (time.monotonic() - t0) * 1e3
        self._record_stats(nodes_expanded, elapsed_ms, length)
        return smoothed

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
    # Roadmap build
    # ------------------------------------------------------------------

    def _build_roadmap(
        self,
        ox: float, oy: float,
        width: int, height: int,
        resolution: float,
        data: List[int],
    ) -> None:
        """Scatter samples and connect nearby collision-free pairs."""
        x_min = ox
        y_min = oy
        x_max = ox + width * resolution
        y_max = oy + height * resolution

        samples: List[Tuple[float, float]] = []
        attempts = 0
        max_attempts = self._n_samples * 20
        while len(samples) < self._n_samples and attempts < max_attempts:
            attempts += 1
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            r, c = self.world_to_grid(x, y, ox, oy, resolution)
            if self.is_valid_cell(
                r, c, height, width, data, self._obstacle_threshold
            ):
                samples.append(
                    self.grid_to_world(r, c, ox, oy, resolution)
                )

        self._roadmap_nodes = samples
        n = len(samples)
        adj: Dict[int, List[Tuple[int, float]]] = {
            i: [] for i in range(n)
        }

        for i in range(n):
            for j in range(i + 1, n):
                d = self._dist(samples[i], samples[j])
                if d > self._connection_radius:
                    continue
                if self._line_free(
                    samples[i], samples[j], ox, oy, resolution,
                    height, width, data,
                ):
                    cost = d
                    adj[i].append((j, cost))
                    adj[j].append((i, cost))

        self._roadmap_adj = adj

    # ------------------------------------------------------------------
    # Query phase (Dijkstra on roadmap + virtual start/goal nodes)
    # ------------------------------------------------------------------

    def _dijkstra_query(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        k: int = 5,
    ) -> int:
        """Connect start/goal to roadmap and run Dijkstra.

        Returns nodes_expanded (positive) on success, or
        -(nodes_expanded) if no path found.
        The found path is stored in self._last_path.
        """
        self._last_path: List[Tuple[float, float]] = []
        nodes = self._roadmap_nodes
        n = len(nodes)

        # Virtual indices: n = start, n+1 = goal
        s_idx = n
        g_idx = n + 1

        # Build temporary adjacency for start and goal
        start_edges: List[Tuple[int, float]] = []
        goal_edges: List[Tuple[int, float]] = []

        # Find k nearest roadmap nodes to start/goal
        dists_start = sorted(
            ((self._dist(start, nodes[i]), i) for i in range(n))
        )
        for _, i in dists_start[:k]:
            d = self._dist(start, nodes[i])
            if d <= self._connection_radius * 2 and self._line_free(
                start, nodes[i],
                self._ox, self._oy, self._resolution,
                self._height, self._width, self._data or [],
            ):
                start_edges.append((i, d))

        dists_goal = sorted(
            ((self._dist(goal, nodes[i]), i) for i in range(n))
        )
        for _, i in dists_goal[:k]:
            d = self._dist(goal, nodes[i])
            if d <= self._connection_radius * 2 and self._line_free(
                goal, nodes[i],
                self._ox, self._oy, self._resolution,
                self._height, self._width, self._data or [],
            ):
                goal_edges.append((i, d))

        if not start_edges or not goal_edges:
            return -0  # 0 nodes expanded, no path

        # Also try direct start→goal if close enough
        d_sg = self._dist(start, goal)
        if d_sg <= self._connection_radius * 2 and self._line_free(
            start, goal,
            self._ox, self._oy, self._resolution,
            self._height, self._width, self._data or [],
        ):
            self._last_path = [start, goal]
            return 0

        # Dijkstra on extended graph
        g_score: Dict[int, float] = {s_idx: 0.0}
        came_from: Dict[int, Optional[int]] = {s_idx: None}
        heap: List = [(0.0, s_idx)]
        nodes_expanded = 0

        while heap:
            g_curr, u = heapq.heappop(heap)
            if g_curr > g_score.get(u, float("inf")):
                continue
            nodes_expanded += 1

            if u == g_idx:
                # Reconstruct world path
                path_idx: List[int] = []
                cur: Optional[int] = g_idx
                while cur is not None:
                    path_idx.append(cur)
                    cur = came_from.get(cur)
                path_idx.reverse()
                world: List[Tuple[float, float]] = []
                for idx in path_idx:
                    if idx == s_idx:
                        world.append(start)
                    elif idx == g_idx:
                        world.append(goal)
                    else:
                        world.append(nodes[idx])
                self._last_path = world
                return nodes_expanded

            # Neighbours
            if u == s_idx:
                edges = start_edges
            elif u < n:
                edges = self._roadmap_adj.get(u, [])
                # Add edge to goal if applicable
                edges = list(edges)
                for gi_cost in goal_edges:
                    if gi_cost[0] == u:
                        edges.append((g_idx, gi_cost[1]))
            else:
                edges = []

            for v, cost in edges:
                new_g = g_curr + cost
                if new_g < g_score.get(v, float("inf")):
                    g_score[v] = new_g
                    came_from[v] = u
                    heapq.heappush(heap, (new_g, v))

        return -nodes_expanded  # no path found

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Euclidean distance between two world-frame points."""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx * dx + dy * dy)

    def _line_free(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        ox: float, oy: float, resolution: float,
        height: int, width: int, data: List[int],
    ) -> bool:
        """Return True if the segment p1→p2 is obstacle-free.

        Uses Bresenham's line on the grid cells along the segment.
        """
        r0, c0 = self.world_to_grid(p1[0], p1[1], ox, oy, resolution)
        r1, c1 = self.world_to_grid(p2[0], p2[1], ox, oy, resolution)

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
            # Safe diagonal: reject if either adjacent corner cell is blocked.
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
