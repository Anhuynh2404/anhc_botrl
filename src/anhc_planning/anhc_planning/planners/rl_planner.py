"""Reinforcement-Learning-based planner stub.

This file is a scaffold for a future learned policy planner.
Implement the :meth:`plan` method to provide a working planner.
"""

from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid

from .base_planner import BasePlanner


class RLPlanner(BasePlanner):
    """Navigation policy driven by a trained reinforcement-learning agent.

    The agent receives a local costmap patch and current pose as observations
    and produces waypoint actions.  This scaffold is designed to support both
    online inference (live model) and offline waypoint generation.

    Parameters
    ----------
    model_path:
        Filesystem path to the serialised policy checkpoint (e.g. ONNX, TorchScript).
    observation_radius_m:
        Radius (metres) of the local costmap crop fed to the policy.
    action_horizon:
        Number of future waypoints the policy outputs per step.
    device:
        Inference device identifier (e.g. "cpu", "cuda:0").
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        observation_radius_m: float = 5.0,
        action_horizon: int = 10,
        device: str = "cpu",
    ) -> None:
        self._model_path = model_path
        self._observation_radius_m = observation_radius_m
        self._action_horizon = action_horizon
        self._device = device

    def get_name(self) -> str:
        return "rl"

    def get_params(self) -> Dict:
        return {
            "model_path": self._model_path,
            "observation_radius_m": self._observation_radius_m,
            "action_horizon": self._action_horizon,
            "device": self._device,
        }

    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        costmap: OccupancyGrid,
    ) -> List[Tuple[float, float]]:
        """Query the RL policy to produce a path from *start* to *goal*.

        .. note::
            Not yet implemented.  Returns an empty list.
        """
        raise NotImplementedError(
            "RLPlanner.plan() is not yet implemented. "
            "See rl_planner.py for the scaffold."
        )
