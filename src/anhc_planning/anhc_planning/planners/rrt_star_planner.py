"""RRT* (Rapidly-exploring Random Tree Star) sampling-based planner."""

import math
import random
import time
from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner

_OBSTACLE_THRESHOLD = 100
_COST_SCALE = 1.0 / 50.0


class RRTStarPlanner(BasePlanner):
    """Asymptotically-optimal sampling-based motion planner (RRT*).

    Builds a tree by random sampling, selecting the best parent
    within *search_radius*, then rewiring neighbours if a lower-cost
    path through the new node is found.

    Parameters
    ----------
    max_iterations:
        Maximum number of tree-expansion iterations.
    step_size:
        Maximum edge length (metres) per tree extension.
    goal_sample_rate:
        Probability [0, 1] of sampling the goal node directly.
    search_radius:
        Rewiring radius (metres) for the RRT* neighbour search.
    obstacle_threshold:
        Cells with cost >= this value are treated as impassable.
    """

    def __init__(
        self,
        max_iterations: int = 5000,
        step_size: float = 0.3,
        goal_sample_rate: float = 0.1,
        search_radius: float = 1.0,
        obstacle_threshold: int = _OBSTACLE_THRESHOLD,
        path_smooth_data_weight: float = 0.5,
        path_smooth_smooth_weight: float = 0.3,
        path_smooth_min_waypoints: int = 5,
    ) -> None:
        self._max_iterations = max_iterations
        self._step_size = step_size
        self._goal_sample_rate = goal_sample_rate
        self._search_radius = search_radius
        self._obstacle_threshold = obstacle_threshold
        self._path_smooth_data_weight = path_smooth_data_weight
        self._path_smooth_smooth_weight = path_smooth_smooth_weight
        self._path_smooth_min_waypoints = path_smooth_min_waypoints
        self._stats: Dict = {}

    # ------------------------------------------------------------------
    # BasePlanner interface
    # ------------------------------------------------------------------

    def get_name(self) -> str:
        """Return the canonical algorithm identifier."""
        return "rrt_star"

    def get_params(self) -> Dict:
        """Return a dictionary of the planner's current configuration."""
        return {
            "max_iterations": self._max_iterations,
            "step_size": self._step_size,
            "goal_sample_rate": self._goal_sample_rate,
            "search_radius": self._search_radius,
            "obstacle_threshold": self._obstacle_threshold,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Run RRT* and return a smoothed world-frame path.

        Returns an empty list when no path can be found within
        *max_iterations*.
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
            self._record_stats(1, (time.monotonic() - t0) * 1e3, 0.0)
            return [start, goal]

        # World-frame bounding box for sampling
        x_min = ox
        y_min = oy
        x_max = ox + width * resolution
        y_max = oy + height * resolution

        # Tree storage (parallel lists — avoids object overhead)
        nodes: List[Tuple[float, float]] = [start]
        parents: List[int] = [-1]
        costs: List[float] = [0.0]

        goal_idx: Optional[int] = None

        for _ in range(self._max_iterations):
            # Biased sampling
            if random.random() < self._goal_sample_rate:
                x_rand = goal
            else:
                x_rand = (
                    random.uniform(x_min, x_max),
                    random.uniform(y_min, y_max),
                )

            nearest_idx = self._nearest(nodes, x_rand)
            x_nearest = nodes[nearest_idx]
            x_new = self._steer(x_nearest, x_rand)

            if not self._collision_free(
                x_nearest, x_new, ox, oy, resolution, height, width, data
            ):
                continue

            # Choose best parent within search radius
            near_indices = self._near(nodes, x_new)
            best_parent = nearest_idx
            min_cost = costs[nearest_idx] + self._dist(x_nearest, x_new)

            for idx in near_indices:
                if not self._collision_free(
                    nodes[idx], x_new,
                    ox, oy, resolution, height, width, data,
                ):
                    continue
                c = costs[idx] + self._dist(nodes[idx], x_new)
                if c < min_cost:
                    min_cost = c
                    best_parent = idx

            new_idx = len(nodes)
            nodes.append(x_new)
            parents.append(best_parent)
            costs.append(min_cost)

            # Rewire neighbours
            for idx in near_indices:
                if idx == best_parent:
                    continue
                candidate = min_cost + self._dist(x_new, nodes[idx])
                if candidate < costs[idx] and self._collision_free(
                    x_new, nodes[idx],
                    ox, oy, resolution, height, width, data,
                ):
                    parents[idx] = new_idx
                    costs[idx] = candidate

            # Check goal proximity
            if self._dist(x_new, goal) <= self._step_size:
                if self._collision_free(
                    x_new, goal,
                    ox, oy, resolution, height, width, data,
                ):
                    goal_idx = len(nodes)
                    nodes.append(goal)
                    parents.append(new_idx)
                    costs.append(min_cost + self._dist(x_new, goal))
                    break

        if goal_idx is None:
            elapsed_ms = (time.monotonic() - t0) * 1e3
            self._record_stats(len(nodes), elapsed_ms, 0.0)
            return []

        # Reconstruct path
        path: List[Tuple[float, float]] = []
        idx = goal_idx
        while idx != -1:
            path.append(nodes[idx])
            idx = parents[idx]
        path.reverse()

        smoothed = self.smooth_path(
            path,
            data_weight=self._path_smooth_data_weight,
            smooth_weight=self._path_smooth_smooth_weight,
            min_waypoints=self._path_smooth_min_waypoints,
        )
        length = self.path_length(smoothed)
        elapsed_ms = (time.monotonic() - t0) * 1e3
        self._record_stats(len(nodes), elapsed_ms, length)
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
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Euclidean distance between two world-frame points."""
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx * dx + dy * dy)

    def _steer(
        self,
        x_from: Tuple[float, float],
        x_to: Tuple[float, float],
    ) -> Tuple[float, float]:
        """Move from x_from toward x_to by at most step_size."""
        d = self._dist(x_from, x_to)
        if d <= self._step_size:
            return x_to
        angle = math.atan2(x_to[1] - x_from[1], x_to[0] - x_from[0])
        return (
            x_from[0] + self._step_size * math.cos(angle),
            x_from[1] + self._step_size * math.sin(angle),
        )

    def _nearest(
        self,
        nodes: List[Tuple[float, float]],
        x: Tuple[float, float],
    ) -> int:
        """Return index of the nearest node to x."""
        best = 0
        best_d = float("inf")
        for i, n in enumerate(nodes):
            d = self._dist(n, x)
            if d < best_d:
                best_d = d
                best = i
        return best

    def _near(
        self,
        nodes: List[Tuple[float, float]],
        x: Tuple[float, float],
    ) -> List[int]:
        """Return indices of all nodes within search_radius of x."""
        return [
            i for i, n in enumerate(nodes)
            if self._dist(n, x) <= self._search_radius
        ]

    def _collision_free(
        self,
        x1: Tuple[float, float],
        x2: Tuple[float, float],
        ox: float,
        oy: float,
        resolution: float,
        height: int,
        width: int,
        data: List[int],
    ) -> bool:
        """Check that the straight line from x1 to x2 is obstacle-free.

        Samples intermediate points at *resolution* intervals.
        """
        d = self._dist(x1, x2)
        if d < 1e-9:
            r, c = self.world_to_grid(x1[0], x1[1], ox, oy, resolution)
            return self.is_valid_cell(
                r, c, height, width, data, self._obstacle_threshold
            )
        steps = max(int(d / resolution) + 1, 2)
        for i in range(steps + 1):
            t = i / steps
            x = x1[0] + t * (x2[0] - x1[0])
            y = x1[1] + t * (x2[1] - x1[1])
            r, c = self.world_to_grid(x, y, ox, oy, resolution)
            if not self.is_valid_cell(
                r, c, height, width, data, self._obstacle_threshold
            ):
                return False
        return True
