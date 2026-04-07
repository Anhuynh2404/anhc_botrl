# anhc_botrl — Autonomous Vehicle Simulation

A professional autonomous vehicle simulation project built on **ROS2 Jazzy Jalisco** and **Gazebo Harmonic (gz-sim8)**.

## Architecture Overview

```
anhc_botrl/
├── src/
│   ├── anhc_description/       # Robot URDF/SDF, meshes
│   ├── anhc_simulation/        # Gazebo worlds, models, launch
│   ├── anhc_sensor_drivers/    # LiDAR + Camera bridge nodes (C++)
│   ├── anhc_localization/      # SLAM, IMU, odometry fusion (C++)
│   ├── anhc_perception/        # Point cloud processing, object detection (C++)
│   ├── anhc_mapping/           # Costmap, occupancy grid (C++)
│   ├── anhc_control/           # Motion controller, PID, cmd_vel (C++)
│   ├── anhc_planning/          # Path planning: A*, Dijkstra, RRT (Python)
│   ├── anhc_behavior/          # State machine / behavior manager (Python)
│   ├── anhc_benchmark/         # Algorithm metrics & logging (Python)
│   └── anhc_viz/               # RViz2 configs, custom dashboards (Python)
├── config/                     # Global YAML configurations
├── scripts/                    # Shell helpers
└── docs/                       # Architecture notes
```

## System Requirements

| Component | Version |
|-----------|---------|
| Ubuntu    | 24.04 LTS (Noble) |
| ROS2      | Jazzy Jalisco |
| Gazebo    | Harmonic (gz-sim8) |
| Python    | 3.12+ |
| CMake     | 3.8+ |

## Dependencies

### ROS2 Packages
```
ros-jazzy-rclcpp
ros-jazzy-rclpy
ros-jazzy-std-msgs
ros-jazzy-geometry-msgs
ros-jazzy-sensor-msgs
ros-jazzy-nav-msgs
ros-jazzy-visualization-msgs
ros-jazzy-tf2
ros-jazzy-tf2-ros
ros-jazzy-tf2-geometry-msgs
ros-jazzy-ros-gz-bridge
ros-jazzy-ros-gz-sim
ros-jazzy-nav2-msgs
```

### Install Dependencies
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rclcpp ros-jazzy-rclpy \
  ros-jazzy-std-msgs ros-jazzy-geometry-msgs \
  ros-jazzy-sensor-msgs ros-jazzy-nav-msgs \
  ros-jazzy-visualization-msgs \
  ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
  ros-jazzy-nav2-msgs
```

## Build

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build all packages
cd ~/anhc_botrl
./scripts/build.sh

# Source the workspace
source install/setup.bash
```

## Run

```bash
# Launch the full simulation stack
./scripts/launch_all.sh
```

## Clean

```bash
./scripts/clean.sh
```

## Package Dependency Order

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

## License

Apache-2.0 — see [LICENSE](LICENSE)
