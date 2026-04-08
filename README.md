# anhc_botrl — Autonomous Vehicle Simulation

A professional autonomous vehicle simulation project built on **ROS2 Jazzy Jalisco** and **Gazebo Harmonic (gz-sim8)**, featuring multiple path-planning algorithms, a live benchmarking framework, and full RViz2 visualisation.

## Architecture Overview

```
anhc_botrl/
├── src/
│   ├── anhc_description/       # Robot URDF/SDF, meshes
│   ├── anhc_simulation/        # Gazebo worlds, models, master launch
│   ├── anhc_sensor_drivers/    # LiDAR + Camera bridge nodes (C++)
│   ├── anhc_localization/      # SLAM, IMU, odometry fusion (C++)
│   ├── anhc_perception/        # Point cloud processing, object detection (C++)
│   ├── anhc_mapping/           # Costmap, occupancy grid (C++)
│   ├── anhc_control/           # Motion controller, PID, cmd_vel (C++)
│   ├── anhc_planning/          # Path planning: A*, Dijkstra, RRT*, D*Lite, RL (Python)
│   ├── anhc_behavior/          # State machine / behavior manager (Python)
│   ├── anhc_benchmark/         # Algorithm benchmarking & metrics (Python)
│   └── anhc_viz/               # RViz2 configs, dashboard, result plots (Python)
├── config/                     # Global YAML configurations
├── scripts/                    # Shell helpers (build, clean, test)
└── docs/                       # Architecture notes
```

## Prerequisites

| Component | Version |
|-----------|---------|
| Ubuntu    | 24.04 LTS (Noble) |
| ROS2      | Jazzy Jalisco (desktop-full) |
| Gazebo    | Harmonic (gz-sim8) |
| Python    | 3.12+ |
| CMake     | 3.8+ |

### ROS2 packages
```
ros-jazzy-slam-toolbox
ros-jazzy-robot-localization
ros-jazzy-nav2-msgs
ros-jazzy-ros-gz-bridge
ros-jazzy-ros-gz-sim
```

### Python packages
```
numpy scipy scikit-learn opencv-python open3d
pandas psutil matplotlib rich
```

### Install all dependencies
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop-full \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-nav2-msgs \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  gz-sim8

pip3 install numpy scipy scikit-learn opencv-python open3d \
             pandas psutil matplotlib rich
```

## Quick Start

```bash
cd ~/anhc_botrl
colcon build --symlink-install
source install/setup.bash

# Run with A* (default, opens Gazebo GUI + RViz2):
ros2 launch anhc_simulation anhc_master.launch.py algorithm:=astar

# Headless A* (no GUI, saves CPU):
ros2 launch anhc_simulation anhc_master.launch.py \
  algorithm:=astar use_rviz:=false gz_extra_args:=-s

# Run benchmark (all scenarios, A* vs Dijkstra):
ros2 launch anhc_simulation anhc_master.launch.py \
  algorithm:=astar use_rviz:=false run_benchmark:=true

# Plot benchmark results:
python3 src/anhc_viz/scripts/anhc_plot_results.py
```

## Master Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `algorithm` | `astar` | Planning algorithm: `astar`, `dijkstra`, `rrt_star`, `dstar_lite`, `rl` |
| `use_rviz` | `true` | Open RViz2 with `anhc_full_system.rviz` |
| `run_benchmark` | `false` | Include the benchmark framework |
| `world` | `anhc_indoor` | Gazebo world name (without `.sdf`) |
| `gz_extra_args` | `""` | Extra args for gz sim (use `-s` for headless) |

## Switching Algorithms at Runtime

```bash
# Hot-swap the planning algorithm while the system is running:
ros2 param set /anhc_global_planner algorithm dijkstra
ros2 param set /anhc_global_planner algorithm rrt_star
ros2 param set /anhc_global_planner algorithm astar
```

The planner switches on the next goal message — no restart required.

## Benchmarking

```bash
# Run full benchmark (default scenario file):
ros2 launch anhc_simulation anhc_master.launch.py run_benchmark:=true

# Or run the benchmark standalone:
ros2 launch anhc_benchmark anhc_benchmark.launch.py \
  scenario_file:=$(ros2 pkg prefix anhc_benchmark)/share/anhc_benchmark/config/anhc_benchmark_scenarios.yaml

# Generate comparison plots:
python3 src/anhc_viz/scripts/anhc_plot_results.py
# → 4 PNG files in ~/anhc_benchmark_results/plots/
# → Markdown report at ~/anhc_benchmark_results/report.md
```

## End-to-End Integration Test

```bash
# Full test (requires Gazebo Harmonic):
bash scripts/test_end_to_end.sh

# CI / sandbox mode (mock planning backend, no Gazebo):
ANHC_MOCK=1 bash scripts/test_end_to_end.sh
```

## Adding a New Planning Algorithm

1. Create `src/anhc_planning/anhc_planning/planners/my_algo_planner.py`:
   ```python
   from .base_planner import BasePlanner
   class MyAlgoPlanner(BasePlanner):
       def plan(self, start, goal, costmap): ...
       def get_name(self) -> str: return "my_algo"
   ```
2. Add to `src/anhc_planning/anhc_planning/planners/__init__.py`
3. Register in `_PLANNER_REGISTRY` in `anhc_global_planner_node.py`
4. Add `my_algo` to the `algorithms` list in `config/anhc_benchmark_scenarios.yaml`

All benchmarking, dashboard, and visualisation components work automatically.

## Extending to RL

Implement `RLPlanner(BasePlanner)` — the `plan()` method calls your RL policy.
The full benchmarking and visualisation pipeline operates automatically,
enabling rigorous comparison of the RL agent against classical planners.

## Package Build Order

```
anhc_description
  └── anhc_simulation
        └── anhc_sensor_drivers
              └── anhc_localization
                    └── anhc_perception
                          └── anhc_mapping
                                └── anhc_planning
                                      └── anhc_control
                                            └── anhc_behavior
                                                  └── anhc_benchmark
                                                        └── anhc_viz
```

## System Requirements

| Component | Version |
|-----------|---------|
| Ubuntu    | 24.04 LTS (Noble) |
| ROS2      | Jazzy Jalisco |
| Gazebo    | Harmonic (gz-sim8) |
| Python    | 3.12+ |
| CMake     | 3.8+ |

## License

Apache-2.0 — see [LICENSE](LICENSE)
