"""D* Lite (Dynamic A*) planner stub.

This file is a scaffold for a future incremental re-planning algorithm.
Implement the :meth:`plan` method to provide a working planner.
"""

from typing import Dict, List, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner


class DStarLitePlanner(BasePlanner):
    """Incremental re-planning algorithm for dynamic environments (D* Lite).

    D* Lite reuses previous search results when the costmap changes,
    making it efficient for robots operating in partially-known environments.

    Parameters
    ----------
    obstacle_threshold:
        Cells with cost ≥ this value are treated as impassable.
    allow_diagonal:
        Enable 8-connectivity when True; 4-connectivity otherwise.
    """

    def __init__(
        self,
        obstacle_threshold: int = 65,
        allow_diagonal: bool = True,
    ) -> None:
        self._obstacle_threshold = obstacle_threshold
        self._allow_diagonal = allow_diagonal

    def get_name(self) -> str:
        return "dstar_lite"

    def get_params(self) -> Dict:
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
        """Run D* Lite to find a path from *start* to *goal*.

        .. note::
            Not yet implemented.  Returns an empty list.
        """
        raise NotImplementedError(
            "DStarLitePlanner.plan() is not yet implemented. "
            "See dstar_lite_planner.py for the scaffold."
        )
