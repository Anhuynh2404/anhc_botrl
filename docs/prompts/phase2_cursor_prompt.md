# Cursor AI Prompt — Phase 2: Upgrade Inflation Layer (Gradient Costmap)

## Project Context

ROS 2 Jazzy project at `/home/anhuynh/anhc_botrl`.

**Before starting, scan these files:**
- `src/anhc_mapping/scripts/anhc_global_costmap_node.py`
- `src/anhc_mapping/config/anhc_costmap_params.yaml`
- `src/anhc_planning/config/planner_params.yaml`
- `src/anhc_planning/scripts/anhc_global_planner_node.py`
- `src/anhc_viz/rviz/anhc_nav.rviz`
- `src/anhc_viz/rviz/anhc_full_system.rviz`

---

## Changes Required

### 1. `anhc_global_costmap_node.py` — Rewrite `_map_cb`

Replace the current exponential decay with **discrete banded costs** using numpy vectorized operations:

```python
import numpy as np
from scipy.ndimage import distance_transform_edt

obstacles = (grid >= 65)
dist_m = distance_transform_edt(~obstacles) * resolution

cost = np.zeros((h, w), dtype=np.int8)
cost[dist_m <= robot_radius]                                       = 100
cost[(dist_m > robot_radius)    & (dist_m <= inflation_band_1)]   = 80
cost[(dist_m > inflation_band_1) & (dist_m <= inflation_band_2)]  = 60
cost[(dist_m > inflation_band_2) & (dist_m <= inflation_band_3)]  = 40
cost[(dist_m > inflation_band_3) & (dist_m <= inflation_band_4)]  = 20
cost[grid < 0] = -1   # ← must be LAST line — preserve SLAM unknown cells
```

**Declare these new ROS parameters** (re-read inside callback on every `/map` message):
```python
robot_radius          = 0.30   # m
inflation_band_1      = 0.45   # m
inflation_band_2      = 0.70   # m
inflation_band_3      = 1.00   # m
inflation_band_4      = 1.20   # m  ← allows navigation in corridors ≥1.20m
use_discrete_bands    = True
cost_scaling_factor   = 3.0    # legacy exponential, used only if use_discrete_bands=False
```

**Add publisher** for `/costmap/global/metadata` (`std_msgs/String`, JSON):
```json
{"robot_radius": 0.30, "bands": [0.45, 0.70, 1.00, 1.20], "costs": [100, 80, 60, 40, 20, 0]}
```

**Log at INFO level** on startup and on each param change:
```
[INFO] Costmap bands: lethal=0.30m | 80=0.45m | 60=0.70m | 40=1.00m | 20=1.20m
```

**Do NOT change** topic name `/costmap/global` or QoS (reliable + transient_local, depth=1).

---

### 2. `anhc_costmap_params.yaml` — Add new parameters

```yaml
anhc_global_costmap_node:
  ros__parameters:
    robot_radius: 0.30
    inflation_band_1: 0.45
    inflation_band_2: 0.70
    inflation_band_3: 1.00
    inflation_band_4: 1.20
    use_discrete_bands: true
    cost_scaling_factor: 3.0
```

---

### 3. `planner_params.yaml` + `anhc_global_planner_node.py` — Update threshold

In `planner_params.yaml`:
```yaml
anhc_global_planner:
  ros__parameters:
    obstacle_cost_threshold: 60   # robot keeps ≥0.70m clearance from walls
```

In `anhc_global_planner_node.py`: change `declare_parameter('obstacle_cost_threshold', 100)`
to `declare_parameter('obstacle_cost_threshold', 60)`.

---

### 4. RViz files — Pastel Cool-Tone Color Scheme

Update **both** `anhc_nav.rviz` and `anhc_full_system.rviz`.

For the `Map` display of `/costmap/global`, apply these settings:
```yaml
Color Scheme: costmap
Alpha: 0.70
Draw Behind: false
```

Additionally, create a **color relay node** at
`src/anhc_viz/scripts/costmap_color_relay.py` that:
- Subscribes to `/costmap/global` (OccupancyGrid)
- Remaps cost values → RGB using this pastel cool-tone table:

| Cost | Color             | R   | G   | B   |
|------|-------------------|-----|-----|-----|
| 100  | Soft Coral Red    | 220 | 100 | 100 |
| 80   | Mint Green        | 150 | 220 | 180 |
| 60   | Sky Blue          | 100 | 180 | 230 |
| 40   | Cornflower Blue   | 100 | 130 | 210 |
| 20   | Soft Lavender     | 170 | 150 | 220 |
| 0    | White             | 255 | 255 | 255 |

- Publishes colored result to `/costmap/visual` as `sensor_msgs/Image`
- Add an `Image` display in RViz for `/costmap/visual` with `Alpha: 0.7`

---

## Constraints

- Do NOT modify costmap topic name or QoS settings
- Do NOT delete any existing planner or SLAM configuration
- `cost[grid < 0] = -1` must always be the **last** assignment in the banding logic
- Use numpy vectorized operations only — no Python cell-by-cell loops
- `colcon build --symlink-install` must succeed with no errors after changes

---

## Verification

After implementation, confirm:
```bash
colcon build --symlink-install --packages-select anhc_mapping anhc_planning anhc_viz
source install/setup.bash

# Test live param update (no restart needed)
ros2 param set /anhc_global_costmap_node inflation_band_4 1.5
```

Expected cost values:
| Distance | Cost |
|----------|------|
| 0.20 m   | 100  |
| 0.40 m   | 80   |
| 0.55 m   | 60   |
| 0.85 m   | 40   |
| 1.10 m   | 20   |
| 1.25 m   | 0    |

Robot navigates freely in corridors **≥ 1.20 m** wide.
