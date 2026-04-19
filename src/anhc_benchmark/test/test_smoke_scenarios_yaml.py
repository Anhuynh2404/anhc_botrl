"""Validate smoke / full benchmark YAML structure for CI."""

import os

import yaml


def _pkg_share_config(filename: str) -> str:
    root = os.path.dirname(os.path.dirname(__file__))
    return os.path.join(root, "config", filename)


def test_smoke_yaml_loads():
    path = _pkg_share_config("anhc_benchmark_scenarios_smoke.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert "common" in cfg
    assert "scenarios" in cfg
    assert "astar" in cfg["common"]["algorithms"]
    assert len(cfg["scenarios"]) >= 1
    key = next(iter(cfg["scenarios"]))
    sc = cfg["scenarios"][key]
    assert "start" in sc and "goal" in sc


def test_full_yaml_loads():
    path = _pkg_share_config("anhc_benchmark_scenarios.yaml")
    with open(path, "r", encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh)
    assert "scenarios" in cfg
    assert len(cfg["scenarios"]) >= 7
    s7 = cfg["scenarios"].get("scenario_7", {})
    assert "dynamic_obstacle" in s7
    assert "gz_world_name" in cfg["common"]
