"""Abstract base class for all path planners in the anhc_planning framework."""

import math
from collections import deque
from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid


class BasePlanner(ABC):
    """Common interface that every planner plugin must implement.

    Concrete planners (A*, Dijkstra, RRT*, D* Lite, RL) inherit from this
    class so the global-planner node can hot-swap algorithms without any
    structural changes.
    """

    # ---------------------------------------------------------------------------
    # Abstract interface — every planner must implement these three methods
    # ---------------------------------------------------------------------------

    @abstractmethod
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Compute a collision-free path from *start* to *goal*.

        Parameters
        ----------
        start:
            (x, y) in map-frame metres.
        goal:
            (x, y) in map-frame metres.
        costmap:
            nav_msgs/OccupancyGrid describing the environment.

        Returns
        -------
        List of (x, y) waypoints in map-frame metres, including start and goal.
        Returns an empty list if no path can be found.
        """

    @abstractmethod
    def get_name(self) -> str:
        """Return the canonical algorithm identifier (e.g. "astar")."""

    @abstractmethod
    def get_params(self) -> Dict:
        """Return a dictionary of the planner's current configuration."""

    # ---------------------------------------------------------------------------
    # Shared utility methods available to all subclasses
    # ---------------------------------------------------------------------------

    @staticmethod
    def heuristic(
        a: Tuple[int, int],
        b: Tuple[int, int],
        weight: float = 1.0,
    ) -> float:
        """Weighted Euclidean heuristic between two grid cells.

        Parameters
        ----------
        a, b:
            (row, col) grid indices.
        weight:
            Scaling factor (>1 makes search greedy; =1 is standard A*).
        """
        dr = a[0] - b[0]
        dc = a[1] - b[1]
        return weight * math.sqrt(dr * dr + dc * dc)

    @staticmethod
    def reconstruct_path(
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Walk the *came_from* map backwards to build the ordered path.

        Parameters
        ----------
        came_from:
            Mapping from each visited cell to its predecessor.
        current:
            The goal cell (start of back-tracking).

        Returns
        -------
        Ordered list of grid cells from start to goal (inclusive).
        """
        path: List[Tuple[int, int]] = []
        while current is not None:
            path.append(current)
            current = came_from.get(current)
        path.reverse()
        return path

    @staticmethod
    def is_valid_cell(
        row: int,
        col: int,
        height: int,
        width: int,
        data: List[int],
        obstacle_threshold: int = 65,
        allow_unknown: bool = True,
    ) -> bool:
        """Return True if the cell is within bounds and below the obstacle threshold.

        Parameters
        ----------
        row, col:
            Grid indices to check.
        height, width:
            Costmap dimensions in cells.
        data:
            Flat row-major costmap data array (−1 = unknown).
        obstacle_threshold:
            Cells with cost ≥ this value are treated as impassable (default 65).
        """
        if row < 0 or row >= height or col < 0 or col >= width:
            return False
        cost = data[row * width + col]
        if cost < 0:
            return allow_unknown
        return cost < obstacle_threshold

    @staticmethod
    def world_to_grid(
        x: float,
        y: float,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> Tuple[int, int]:
        """Convert map-frame (x, y) metres to (row, col) grid indices.

        Row-major OccupancyGrid indexing uses ``index = row * width + col`` with
        ``col`` along map +x and ``row`` along map +y. Use ``floor`` so negative
        coordinates and cell boundaries match map_server / RViz.
        """
        col = int(math.floor((x - origin_x) / resolution))
        row = int(math.floor((y - origin_y) / resolution))
        return (row, col)

    @staticmethod
    def grid_to_world(
        row: int,
        col: int,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> Tuple[float, float]:
        """Convert (row, col) grid indices to map-frame (x, y) metres (cell centre)."""
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        return (x, y)

    @staticmethod
    def smooth_path(
        path: List[Tuple[float, float]],
        data_weight: float = 0.5,
        smooth_weight: float = 0.3,
        tolerance: float = 1e-6,
        min_waypoints: int = 5,
    ) -> List[Tuple[float, float]]:
        """Gradient-descent path smoothing while preserving endpoints."""
        if len(path) < max(3, min_waypoints):
            return path
        smoothed = [list(p) for p in path]
        change = tolerance + 1.0
        while change > tolerance:
            change = 0.0
            for i in range(1, len(smoothed) - 1):
                for j in range(2):
                    old = smoothed[i][j]
                    smoothed[i][j] += data_weight * (path[i][j] - smoothed[i][j])
                    smoothed[i][j] += smooth_weight * (
                        smoothed[i - 1][j]
                        + smoothed[i + 1][j]
                        - 2.0 * smoothed[i][j]
                    )
                    change += abs(old - smoothed[i][j])
        return [tuple(p) for p in smoothed]

    @staticmethod
    def path_length(path: List[Tuple[float, float]]) -> float:
        """Return the cumulative Euclidean length of a waypoint list (metres)."""
        total = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            total += math.sqrt(dx * dx + dy * dy)
        return total

    @staticmethod
    def nearest_valid_cell(
        start: Tuple[int, int],
        height: int,
        width: int,
        data: List[int],
        obstacle_threshold: int = 65,
        max_radius_cells: int = 50,
    ) -> Optional[Tuple[int, int]]:
        """Find nearest valid cell around start using bounded BFS.

        Returns None if no valid cell is found within max_radius_cells.
        """
        sr, sc = start
        if BasePlanner.is_valid_cell(
            sr, sc, height, width, data, obstacle_threshold
        ):
            return start

        q = deque([(sr, sc, 0)])
        visited = {(sr, sc)}
        neigh = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while q:
            r, c, d = q.popleft()
            if d >= max_radius_cells:
                continue

            for dr, dc in neigh:
                nr, nc = r + dr, c + dc
                if (nr, nc) in visited:
                    continue
                visited.add((nr, nc))

                if nr < 0 or nr >= height or nc < 0 or nc >= width:
                    continue

                if BasePlanner.is_valid_cell(
                    nr, nc, height, width, data, obstacle_threshold
                ):
                    return (nr, nc)
                q.append((nr, nc, d + 1))

        return None
