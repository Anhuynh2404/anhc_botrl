# Phase 2 — Upgrade Inflation Layer (Gradient Costmap)

## Context

This is a ROS 2 (Jazzy) project at `/home/anhuynh/anhc_botrl`.

The current costmap node is:
- `src/anhc_mapping/scripts/anhc_global_costmap_node.py`

It already uses `scipy.ndimage.distance_transform_edt` but the output is only:
- `100` = obstacle / lethal zone (within `robot_radius`)
- `1–99` = continuous exponential decay via `cost_scaling_factor`
- `0` = free space

**The upgrade goal** is to produce a **discrete banded** inflation layer that matches the convention widely used in Nav2 / ROS 2:

| Distance from obstacle | Cost value | Color in RViz |
|---|---|---|
| ≤ robot_radius | **100** (lethal) | Red |
| robot_radius … band_1 | **80** (inscribed) | Orange-Red |
| band_1 … band_2 | **60** (possibly inscribed) | Orange |
| band_2 … band_3 | **40** (high cost) | Yellow |
| band_3 … band_4 | **20** (moderate cost) | Green |
| > band_4 | **0** (free) | White |

The band distances are **configurable parameters** (in metres), not hardcoded.

**Key files to read before starting:**
- `src/anhc_mapping/scripts/anhc_global_costmap_node.py`
- `src/anhc_mapping/config/anhc_costmap_params.yaml`
- `src/anhc_viz/rviz/anhc_nav.rviz` (for costmap display settings)
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
inflation_band_4:      1.40   # edge of cost-20 band (m)
# any cell beyond inflation_band_4 gets cost 0
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
# cost already 0 beyond band_4

# 4. Preserve unknown cells (-1 from SLAM)
cost[grid < 0] = -1
```

### Fallback (when `use_discrete_bands=false` or scipy not available):
Keep the existing exponential formula but clip to int8 range.

### Additional requirements:
- Re-read parameters every time `_map_cb` fires (use `get_parameter().value` inside the callback so runtime `ros2 param set` changes take effect immediately without restarting the node).
- Add a log message at `INFO` level showing current band distances when the node starts and whenever parameters change.
- Publish the same topic `/costmap/global` with the same QoS profile (reliable + transient_local, depth 1).
- Also publish a new topic `/costmap/global/metadata` as `std_msgs/String` (JSON) with the current band config:
  ```json
  {"robot_radius": 0.30, "bands": [0.45, 0.70, 1.00, 1.40], "costs": [100, 80, 60, 40, 20, 0]}
  ```

---

## Task 2 — Update `anhc_costmap_params.yaml`

File: `src/anhc_mapping/config/anhc_costmap_params.yaml`

Add (or update) all new parameters:
```yaml
anhc_global_costmap_node:
  ros__parameters:
    robot_radius: 0.30
    inflation_band_1: 0.45
    inflation_band_2: 0.70
    inflation_band_3: 1.00
    inflation_band_4: 1.40
    use_discrete_bands: true
    cost_scaling_factor: 3.0
```

---

## Task 3 — Update planner obstacle threshold

The planners (A*, Dijkstra, and all new planners from Phase 1) use `obstacle_threshold=100` by default, meaning only lethal cells (cost=100) block navigation.

Update `src/anhc_planning/config/planner_params.yaml` to document the recommended threshold values:
```yaml
# obstacle_cost_threshold options:
#   100 → robot passes through cost-80/60/40/20 zones (hugs walls)
#   80  → robot avoids cost-80 zone (respects inscribed radius strictly)
#   60  → robot avoids cost-60 zone (recommended default for safety)
#   40  → robot keeps large clearance
anhc_global_planner:
  ros__parameters:
    obstacle_cost_threshold: 60
```

Update the `obstacle_cost_threshold` default in `anhc_global_planner_node.py` `declare_parameter` call from `100` to `60`.

---

## Task 4 — RViz Costmap Color Scheme

Update **both** RViz config files to show the banded costmap in meaningful colours:

Files:
- `src/anhc_viz/rviz/anhc_nav.rviz`
- `src/anhc_viz/rviz/anhc_full_system.rviz`

For the `Map` display showing `/costmap/global`, set:
```yaml
Color Scheme: costmap   # built-in costmap scheme: 0=white, 100=red gradient
Alpha: 0.7
Draw Behind: false
```

Also ensure there is a separate `Map` display for the raw `/map` topic with `Color Scheme: map` (black/white/grey).

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
  -p inflation_band_4:=1.40 \
  -p use_discrete_bands:=true
# Should print band distances at INFO level

# At runtime, update a band without restarting:
ros2 param set /anhc_global_costmap_node inflation_band_4 2.0
# Costmap should immediately re-publish with updated bands on next /map message
```

### Expected costmap values after upgrade
With the default parameters, a cell 0.5 m from the nearest obstacle should have cost **60**, not an arbitrary exponential value.

---

## Notes

- Do NOT change the costmap topic name or QoS — downstream planners and benchmark nodes depend on `/costmap/global` with transient_local QoS.
- The banded scheme makes the inflation layer **interpretable** at a glance in RViz: red=danger, orange=caution, yellow=prefer avoid, green=slight penalty, white=free.
- Planners using `obstacle_cost_threshold=60` will avoid the orange zone entirely, giving the robot at least `inflation_band_2=0.7 m` clearance from walls.
