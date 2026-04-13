# Phase 5 — Dashboard & Visualization Upgrade

## Context

This is a ROS 2 (Jazzy) project at `/home/anhuynh/anhc_botrl`.

**Prerequisite:** Phases 1–4 must be complete (all planners, banded inflation layer, benchmark metrics, and office_v2 map).

Existing visualization assets:
- `src/anhc_viz/rviz/anhc_nav.rviz` — navigation RViz config
- `src/anhc_viz/rviz/anhc_full_system.rviz` — full-system RViz config
- `src/anhc_viz/rviz/anhc_mapping.rviz` — mapping phase RViz config
- `src/anhc_viz/anhc_viz/anhc_dashboard_panel.py` — existing panel (read its content)
- `src/anhc_viz/scripts/anhc_plot_results.py` — offline comparison plotter (upgraded in Phase 3)

**Key files to read before starting:**
- `src/anhc_viz/anhc_viz/anhc_dashboard_panel.py` (full file)
- `src/anhc_viz/rviz/anhc_nav.rviz` (full file)
- `src/anhc_viz/rviz/anhc_full_system.rviz` (full file)
- `src/anhc_viz/setup.py` (entry points)
- `src/anhc_benchmark/anhc_benchmark/anhc_live_metrics_node.py` (full file)

---

## Task 1 — Upgrade RViz Configs for Banded Costmap Display

### Files to update:
- `src/anhc_viz/rviz/anhc_nav.rviz`
- `src/anhc_viz/rviz/anhc_full_system.rviz`

### Changes:

**1a. Costmap Display with gradient color scheme**

Find or add the `Map` panel displaying `/costmap/global` and set:
```yaml
- Class: rviz_default_plugins/Map
  Name: Global Costmap (Banded)
  Topic:
    Value: /costmap/global
  Color Scheme: costmap     # built-in: 0=white → 100=red via blue/green gradient
  Alpha: 0.65
  Draw Behind: false
  Enabled: true
```

**1b. Raw map display (keep separate)**
```yaml
- Class: rviz_default_plugins/Map
  Name: SLAM Map
  Topic:
    Value: /map
  Color Scheme: map         # black/white/grey
  Alpha: 0.85
  Draw Behind: true
  Enabled: true
```

**1c. Path display for each algorithm (using /planning/path)**
```yaml
- Class: rviz_default_plugins/Path
  Name: Planned Path
  Topic:
    Value: /planning/path
  Color:
    R: 0
    G: 0.8
    B: 0.2
  Line Width: 0.04
  Enabled: true
```

**1d. Add a `MarkerArray` display for inflation band legend**
Subscribe to `/costmap/inflation_legend` (this topic will be published by the upgraded costmap node in Task 2):
```yaml
- Class: rviz_default_plugins/MarkerArray
  Name: Inflation Band Legend
  Topic:
    Value: /costmap/inflation_legend
  Enabled: true
```

---

## Task 2 — Inflation Band Legend Publisher

In `src/anhc_mapping/scripts/anhc_global_costmap_node.py`, add a publisher for visual legend markers in RViz.

After publishing `/costmap/global`, also publish a `visualization_msgs/MarkerArray` on `/costmap/inflation_legend` showing color-coded rectangles indicating each cost band.

Add marker strips at the left edge of the map bounding box:
```python
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

# In __init__:
self._pub_legend = self.create_publisher(MarkerArray, '/costmap/inflation_legend', _MAP_QOS)

# In _map_cb, after publishing costmap:
self._publish_legend(msg)

def _publish_legend(self, map_msg):
    """Publish 5 colored flat boxes in the map frame as a cost band legend."""
    bands = [
        (100, (1.0, 0.0, 0.0, 0.8), "Lethal (100)"),
        ( 80, (1.0, 0.4, 0.0, 0.7), "Inscribed (80)"),
        ( 60, (1.0, 0.8, 0.0, 0.7), "High cost (60)"),
        ( 40, (0.6, 1.0, 0.0, 0.6), "Moderate (40)"),
        ( 20, (0.2, 0.8, 0.2, 0.5), "Low cost (20)"),
    ]
    ma = MarkerArray()
    ox = map_msg.info.origin.position.x - 1.5  # place legend outside map
    oy = map_msg.info.origin.position.y
    for i, (cost, (r, g, b, a), label) in enumerate(bands):
        m = Marker()
        m.header = map_msg.header
        m.ns = "inflation_legend"
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = ox
        m.pose.position.y = oy + i * 0.5
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0
        m.scale.x = 0.4; m.scale.y = 0.4; m.scale.z = 0.1
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a
        m.text = label
        ma.markers.append(m)
    self._pub_legend.publish(ma)
```

---

## Task 3 — Upgrade `anhc_dashboard_panel.py`

File: `src/anhc_viz/anhc_viz/anhc_dashboard_panel.py`

Read the existing file first, then upgrade it to show real-time metrics from `/planning/stats` and `/benchmark/live_metrics`.

### The upgraded panel must display:

**Section 1 — Current Algorithm**
- Algorithm name (from `/planning/stats` JSON field `algorithm`)
- Planning time (ms)
- Path length (m) and number of steps
- Nodes expanded

**Section 2 — Smoothness**
- Curvature mean (rad/m)
- Jerk mean (rad/m²)
- Number of turns

**Section 3 — Safety**
- Clearance avg (m)
- Clearance min (m)
- Safety score (0–1)

**Section 4 — System**
- CPU usage (%)
- Memory (MB)
- Algorithm switcher: dropdown or buttons for all 8 algorithms

### Implementation requirements:
- Subscribe to `/planning/stats` (std_msgs/String, JSON) to update Sections 1–3.
- Subscribe to `/benchmark/live_metrics` (std_msgs/String, JSON) for Section 4.
- Add an **Algorithm Switcher** widget: 8 buttons or a dropdown. Clicking one calls:
  ```python
  # Use rclpy parameter service or topic to hot-swap the planner
  ros2 param set /anhc_global_planner algorithm <name>
  ```
  Implement this via a `rclpy` service call to `/anhc_global_planner/set_parameters` inside the panel's ROS node.
- If the panel is Qt-based (rqt plugin), use `QLabel` and `QPushButton` widgets.
- If it is a standalone node, write to stdout in a readable table format.
- Update display at 5 Hz (use a `QTimer` or `rclpy.create_timer`).

---

## Task 4 — Algorithm Comparison Overlay in RViz

Create a new script `src/anhc_viz/scripts/anhc_path_overlay_node.py` that:

1. Subscribes to `/planning/path` and stores the last path per algorithm.
2. On a `std_srvs/Trigger` service call `/viz/compare_paths`, publishes all stored paths as a `visualization_msgs/MarkerArray` on `/viz/path_comparison` using a distinct colour per algorithm:

| Algorithm | Colour |
|---|---|
| astar | Blue (0, 0.4, 1) |
| dijkstra | Cyan (0, 1, 1) |
| theta_star | Green (0, 0.8, 0.2) |
| greedy_bfs | Yellow (1, 0.9, 0) |
| jps | Magenta (0.9, 0, 0.9) |
| rrt_star | Orange (1, 0.5, 0) |
| dstar_lite | Red (1, 0, 0) |
| prm | Purple (0.5, 0, 0.8) |

3. Each path is a `LINE_STRIP` marker with `scale.x = 0.025` (2.5 cm width).
4. Markers persist for 30 seconds (set `lifetime.sec = 30`).

Add a new RViz display in `anhc_full_system.rviz` for `/viz/path_comparison`.

### Usage:
```bash
# Run the overlay node
ros2 run anhc_viz anhc_path_overlay_node.py

# Trigger comparison (after running several algorithms to the same goal)
ros2 service call /viz/compare_paths std_srvs/srv/Trigger {}
```

---

## Task 5 — Update `setup.py` Entry Points

File: `src/anhc_viz/setup.py`

Add the new script to `console_scripts` or `scripts` install:
```python
entry_points={
    'console_scripts': [
        'anhc_path_overlay_node = anhc_viz.scripts.anhc_path_overlay_node:main',
        # ...existing entries...
    ],
},
```
Or ensure it is installed as a script:
```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/anhc_viz']),
    ('share/' + package_name, ['package.xml']),
    ...
    (os.path.join('lib', package_name), glob('scripts/*.py')),
],
```

---

## Task 6 — Launch Integration

Update `src/anhc_simulation/launch/anhc_nav.launch.py` to optionally start the path overlay node:
```python
DeclareLaunchArgument('use_path_overlay', default_value='false',
                      description='Start path comparison overlay node.'),

Node(
    package='anhc_viz',
    executable='anhc_path_overlay_node.py',
    name='anhc_path_overlay',
    parameters=[{'use_sim_time': True}],
    condition=IfCondition(LaunchConfiguration('use_path_overlay')),
    output='screen',
),
```

---

## Verification

```bash
cd ~/anhc_botrl
colcon build --symlink-install --packages-select anhc_viz anhc_mapping anhc_simulation
source install/setup.bash

# Launch full navigation stack with path overlay
ros2 launch anhc_simulation anhc_nav.launch.py \
  algorithm:=astar \
  use_path_overlay:=true

# In RViz, you should see:
# 1. Raw /map in black/white
# 2. /costmap/global with red-orange-yellow-green banded colors
# 3. Colored band legend markers to the left of the map
# 4. Planned path in green

# Run with theta_star, then trigger comparison
ros2 param set /anhc_global_planner algorithm theta_star
# Send a new 2D Goal Pose in RViz
ros2 service call /viz/compare_paths std_srvs/srv/Trigger {}
# Both astar and theta_star paths should appear in different colours
```

---

## Summary of New/Modified Files

| File | Action |
|---|---|
| `src/anhc_viz/rviz/anhc_nav.rviz` | Update costmap display, add path overlay display |
| `src/anhc_viz/rviz/anhc_full_system.rviz` | Same + add path comparison display |
| `src/anhc_viz/anhc_viz/anhc_dashboard_panel.py` | Upgrade with full metrics + algorithm switcher |
| `src/anhc_viz/scripts/anhc_path_overlay_node.py` | NEW — path comparison overlay |
| `src/anhc_viz/setup.py` | Add new script entry point |
| `src/anhc_mapping/scripts/anhc_global_costmap_node.py` | Add legend marker publisher |
| `src/anhc_simulation/launch/anhc_nav.launch.py` | Add `use_path_overlay` argument |
