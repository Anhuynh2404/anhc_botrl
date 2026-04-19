# Phase 2 — Upgrade Inflation Layer (Gradient Costmap)

## Context

This is a ROS 2 (Jazzy) project at `/home/anhuynh/anhc_botrl`.

The current costmap node is:
- `src/anhc_mapping/scripts/anhc_global_costmap_node.py`

It already uses `scipy.ndimage.distance_transform_edt` but the output is only:
- `100` = obstacle / lethal zone (within `robot_radius`)
- `1–99` = continuous exponential decay via `cost_scaling_factor`
- `0` = free space

**The upgrade goal** is to produce a **discrete banded** inflation layer with **soft pastel cool-tone colors** (mint green → sky blue → soft purple) in RViz, matching this convention:

| Distance from obstacle | Cost value | Color in RViz     |
|---|---|---|
| ≤ robot_radius          | **100** (lethal)             | Soft Red / Coral     |
| robot_radius … band_1   | **80**  (inscribed)          | Mint Green           |
| band_1 … band_2         | **60**  (possibly inscribed) | Sky Blue             |
| band_2 … band_3         | **40**  (high cost)          | Cornflower Blue      |
| band_3 … band_4         | **20**  (moderate cost)      | Soft Lavender/Purple |
| > band_4                | **0**   (free)               | White                |

The band distances are **configurable parameters** (in metres), not hardcoded.

**Key files to read before starting:**
- `src/anhc_mapping/scripts/anhc_global_costmap_node.py`
- `src/anhc_mapping/config/anhc_costmap_params.yaml`
- `src/anhc_viz/rviz/anhc_nav.rviz`
- `src/anhc_viz/rviz/anhc_full_system.rviz`

---

## Task 1 — Upgrade `anhc_global_costmap_node.py`

Rewrite `_map_cb` to produce the banded gradient:

### New ROS parameters to declare:
```yaml
robot_radius:          0.30   # lethal zone radius (m)
inflation_band_1:      0.45   # edge of cost-80 band (m)
inflation_band_2:      0.70   # edge of cost-60 band (m)
inflation_band_3:      1.00   # edge of cost-40 band (m)
inflation_band_4:      1.20   # edge of cost-20 band (m)  ← changed from 1.40
use_discrete_bands:    true   # if false, keep the legacy exponential mode
cost_scaling_factor:   3.0    # used only when use_discrete_bands=false
```

### Logic (when `use_discrete_bands=true`):
```python
import numpy as np
from scipy.ndimage import distance_transform_edt

# 1. Build binary obstacle mask: True where OccupancyGrid value >= 65
obstacles = (grid >= 65)

# 2. Compute per-cell Euclidean distance to nearest obstacle (in metres)
dist_m = distance_transform_edt(~obstacles) * resolution

# 3. Assign banded costs (use numpy vectorized assignments, no Python loops)
cost = np.zeros((h, w), dtype=np.int8)
cost[dist_m <= robot_radius]                                      = 100
cost[(dist_m > robot_radius)   & (dist_m <= inflation_band_1)]   = 80
cost[(dist_m > inflation_band_1) & (dist_m <= inflation_band_2)] = 60
cost[(dist_m > inflation_band_2) & (dist_m <= inflation_band_3)] = 40
cost[(dist_m > inflation_band_3) & (dist_m <= inflation_band_4)] = 20
# cost already 0 beyond band_4 (1.20 m)

# 4. Preserve unknown cells (-1 from SLAM)
cost[grid < 0] = -1
```

### Fallback (when `use_discrete_bands=false` or scipy not available):
Keep the existing exponential formula but clip to int8 range.

### Additional requirements:
- Re-read parameters every time `_map_cb` fires (use `get_parameter().value` inside the
  callback so runtime `ros2 param set` changes take effect immediately without restarting).
- Add a log message at `INFO` level showing current band distances when the node starts
  and whenever parameters change.
- Publish the same topic `/costmap/global` with the same QoS profile
  (reliable + transient_local, depth 1).
- Also publish a new topic `/costmap/global/metadata` as `std_msgs/String` (JSON):
  ```json
  {"robot_radius": 0.30, "bands": [0.45, 0.70, 1.00, 1.20], "costs": [100, 80, 60, 40, 20, 0]}
  ```

---

## Task 2 — Update `anhc_costmap_params.yaml`

File: `src/anhc_mapping/config/anhc_costmap_params.yaml`

```yaml
anhc_global_costmap_node:
  ros__parameters:
    robot_radius: 0.30
    inflation_band_1: 0.45
    inflation_band_2: 0.70
    inflation_band_3: 1.00
    inflation_band_4: 1.20        # ← changed from 1.40
    use_discrete_bands: true
    cost_scaling_factor: 3.0
```

---

## Task 3 — Update planner obstacle threshold

Update `src/anhc_planning/config/planner_params.yaml`:
```yaml
# obstacle_cost_threshold options:
#   100 → robot passes through all inflation zones (hugs walls)
#   80  → robot avoids cost-80 zone (respects inscribed radius)
#   60  → robot avoids cost-60 zone (recommended default — keeps ≥0.70m from walls)
#   40  → robot keeps large clearance
#
# With inflation_band_4=1.20m, the robot can now navigate in corridors ≥1.20m wide.
# Previously required ≥1.40m clearance.
anhc_global_planner:
  ros__parameters:
    obstacle_cost_threshold: 60
```

Update the `obstacle_cost_threshold` default in `anhc_global_planner_node.py`
`declare_parameter` call from `100` to `60`.

---

## Task 4 — RViz Costmap Color Scheme (Pastel Cool Tones)

Update **both** RViz config files to display the banded costmap with soft pastel cool-tone colors.

Files:
- `src/anhc_viz/rviz/anhc_nav.rviz`
- `src/anhc_viz/rviz/anhc_full_system.rviz`

### Color mapping for the `/costmap/global` Map display:

Use `Color Scheme: raw` and define a custom color table with these RGBA values:

| Cost | Color Name        | R   | G   | B   | A   |
|------|-------------------|-----|-----|-----|-----|
| 100  | Soft Coral Red    | 220 | 100 | 100 | 200 |
| 80   | Mint Green        | 150 | 220 | 180 | 180 |
| 60   | Sky Blue          | 100 | 180 | 230 | 160 |
| 40   | Cornflower Blue   | 100 | 130 | 210 | 140 |
| 20   | Soft Lavender     | 170 | 150 | 220 | 120 |
| 0    | White (free)      | 255 | 255 | 255 |   0 |

### RViz Map display settings:
```yaml
- Class: rviz_default_plugins/Map
  Name: Costmap Global
  Topic:
    Value: /costmap/global
  Color Scheme: costmap
  Alpha: 0.70
  Draw Behind: false
```

> **Note:** RViz's built-in `costmap` scheme goes red→yellow→green→white.
> For full pastel cool-tone control, implement a custom `rviz_plugin` or use
> `Color Scheme: raw` with a color interpreter node that republishes
> `/costmap/global` as a colored `sensor_msgs/Image` overlay on `/costmap/visual`.

### Optional — Color relay node
If you want exact pastel colors without a custom plugin, create
`src/anhc_viz/scripts/costmap_color_relay.py` that:
- Subscribes to `/costmap/global` (OccupancyGrid)
- Converts to `nav_msgs/OccupancyGrid` with values remapped to a
  `sensor_msgs/Image` (RGB) using the color table above
- Publishes to `/costmap/visual` for display in RViz as an Image overlay

---

## Task 5 — Verify

```bash
cd ~/anhc_botrl
colcon build --symlink-install --packages-select anhc_mapping
source install/setup.bash

# Check parameters load correctly
ros2 run anhc_mapping anhc_global_costmap_node.py --ros-args \
  -p robot_radius:=0.30 \
  -p inflation_band_1:=0.45 \
  -p inflation_band_4:=1.20 \
  -p use_discrete_bands:=true
# Should print band distances at INFO level

# Live update without restart:
ros2 param set /anhc_global_costmap_node inflation_band_4 1.5
# Costmap should immediately re-publish with updated bands on next /map message
```

### Expected costmap values after upgrade

| Distance from wall | Expected cost |
|--------------------|---------------|
| 0.20 m             | 100           |
| 0.40 m             | 80            |
| 0.55 m             | 60            |
| 0.85 m             | 40            |
| 1.10 m             | 20            |
| 1.25 m             | **0** (free)  |

Robot can now navigate in corridors as narrow as **1.20 m** wide (previously required 1.40 m).

---

## Notes

- Do NOT change the costmap topic name or QoS — downstream planners depend on
  `/costmap/global` with transient_local QoS.
- `inflation_band_4` reduced from `1.40 m` → `1.20 m`: robot has less clearance but
  gains access to narrower corridors. Tune back up if collision risk increases.
- The pastel cool-tone scheme (mint → sky blue → lavender) is easier on the eyes during
  long RViz monitoring sessions compared to the harsh red/orange/yellow defaults.
- Planners using `obstacle_cost_threshold=60` will avoid the sky-blue zone entirely,
  giving the robot at least `inflation_band_2=0.70 m` clearance from walls.
