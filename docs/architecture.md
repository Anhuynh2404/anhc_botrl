# anhc_botrl — Architecture Notes

## System Diagram

```
Gazebo Harmonic (gz-sim8)
        │
        │  ros_gz_bridge
        ▼
anhc_sensor_drivers  ──────────────────────────────────────────────┐
  • /scan (LaserScan)                                               │
  • /camera/image_raw (Image)                                       │
  • /imu/data (Imu)                                                 │
        │                                                           │
        ▼                                                           │
anhc_localization                                                   │
  • SLAM / AMCL                                                     │
  • IMU + Odometry EKF fusion                                       │
  • Publishes: /tf, /odom                                           │
        │                                                           │
        ├──────────────────────────┐                                │
        ▼                          ▼                                │
anhc_perception           anhc_mapping                             │
  • Object detection        • Costmap 2D                            │
  • Cluster extraction       • Occupancy grid                       │
        │                          │                                │
        └────────────┬─────────────┘                                │
                     ▼                                              │
              anhc_planning                                         │
                • A* / Dijkstra / RRT                               │
                • Publishes: /plan                                   │
                     │                                              │
                     ▼                                              │
              anhc_control                                          │
                • PID controller                                     │
                • Publishes: /cmd_vel                               │
                     │                                              │
                     └──────────────────────────────────────────────┘
                                                                    │
                                              Gazebo receives /cmd_vel
```

## Key Topics

| Topic              | Type                   | Publisher              | Subscriber              |
|--------------------|------------------------|------------------------|-------------------------|
| `/scan`            | `sensor_msgs/LaserScan`| anhc_sensor_drivers    | anhc_localization, anhc_perception |
| `/camera/image_raw`| `sensor_msgs/Image`    | anhc_sensor_drivers    | anhc_perception         |
| `/imu/data`        | `sensor_msgs/Imu`      | anhc_sensor_drivers    | anhc_localization       |
| `/odom`            | `nav_msgs/Odometry`    | anhc_localization      | anhc_planning, anhc_control |
| `/map`             | `nav_msgs/OccupancyGrid`| anhc_mapping          | anhc_planning           |
| `/plan`            | `nav_msgs/Path`        | anhc_planning          | anhc_control, anhc_behavior |
| `/cmd_vel`         | `geometry_msgs/Twist`  | anhc_control           | Gazebo bridge           |
| `/tf`              | `tf2_msgs/TFMessage`   | anhc_localization      | All packages            |

## Planning Algorithms (anhc_planning)

- **A\*** — heuristic grid-based search, optimal with admissible heuristic
- **Dijkstra** — uniform-cost search, optimal, no heuristic
- **RRT** — sampling-based, good for high-dimensional / kinodynamic problems
- **RRT\*** — asymptotically optimal variant of RRT

## Benchmark Metrics (anhc_benchmark)

- Path length (meters)
- Planning time (milliseconds)
- Smoothness (curvature integral)
- Success rate (% goals reached)
- CPU / memory usage per planner
