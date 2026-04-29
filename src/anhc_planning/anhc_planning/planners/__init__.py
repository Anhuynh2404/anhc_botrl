"""Planner plugin registry for anhc_planning."""

from .astar_planner import AStarPlanner
from .base_planner import BasePlanner
from .dijkstra_planner import DijkstraPlanner
from .dstar_lite_planner import DStarLitePlanner
from .greedy_bfs_planner import GreedyBFSPlanner
from .jps_planner import JPSPlanner
from .prm_planner import PRMPlanner
from .rrt_star_planner import RRTStarPlanner
from .theta_star_planner import ThetaStarPlanner

try:
    from .rl_planner import RLPlanner
except ModuleNotFoundError as exc:
    _RL_IMPORT_ERROR = exc

    class RLPlanner:  # type: ignore[no-redef]
        """Fallback when optional RL dependencies are not installed."""

        def __init__(self, *args, **kwargs):
            raise ModuleNotFoundError(
                "RL planner requires 'stable_baselines3'. "
                "Install it to use algorithm:=rl."
            ) from _RL_IMPORT_ERROR

__all__ = [
    'BasePlanner',
    'AStarPlanner',
    'DijkstraPlanner',
    'RRTStarPlanner',
    'DStarLitePlanner',
    'ThetaStarPlanner',
    'GreedyBFSPlanner',
    'JPSPlanner',
    'PRMPlanner',
    'RLPlanner',
]
