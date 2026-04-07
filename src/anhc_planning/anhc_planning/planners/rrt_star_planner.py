"""RRT* (Rapidly-exploring Random Tree Star) planner stub.

This file is a scaffold for a future sampling-based planner.
Implement the :meth:`plan` method to provide a working planner.
"""

from typing import Dict, List, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner


class RRTStarPlanner(BasePlanner):
    """Asymptotically-optimal sampling-based motion planner (RRT*).

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
    """

    def __init__(
        self,
        max_iterations: int = 5000,
        step_size: float = 0.3,
        goal_sample_rate: float = 0.1,
        search_radius: float = 1.0,
    ) -> None:
        self._max_iterations = max_iterations
        self._step_size = step_size
        self._goal_sample_rate = goal_sample_rate
        self._search_radius = search_radius

    def get_name(self) -> str:
        return "rrt_star"

    def get_params(self) -> Dict:
        return {
            "max_iterations": self._max_iterations,
            "step_size": self._step_size,
            "goal_sample_rate": self._goal_sample_rate,
            "search_radius": self._search_radius,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Run RRT* to find a path from *start* to *goal*.

        .. note::
            Not yet implemented.  Returns an empty list.
        """
        raise NotImplementedError(
            "RRTStarPlanner.plan() is not yet implemented. "
            "See rrt_star_planner.py for the scaffold."
        )
