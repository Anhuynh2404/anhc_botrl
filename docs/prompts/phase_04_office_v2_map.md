# Phase 4 — New Map: office_v2.stl Integration

## Context

This is a ROS 2 (Jazzy) project at `/home/anhuynh/anhc_botrl`.

The STL file for the new office environment already exists at:
```
src/anhc_simulation/models/office_v2.stl
```

The **existing** v1 setup to use as reference:
- World file: `src/anhc_simulation/worlds/anhc_indoor.sdf` (uses `office_v1.stl`)
- Map files: `src/anhc_simulation/maps/anhc_indoor_map.pgm` + `anhc_indoor_map.yaml`
- Launch file: `src/anhc_simulation/launch/anhc_nav.launch.py` (parameter `world:=anhc_indoor`)

**Key files to read before starting:**
- `src/anhc_simulation/worlds/anhc_indoor.sdf` (full file — reference for world structure)
- `src/anhc_simulation/maps/anhc_indoor_map.yaml`
- `src/anhc_simulation/launch/anhc_sim.launch.py`
- `src/anhc_simulation/launch/anhc_nav.launch.py`
- `src/anhc_simulation/CMakeLists.txt` (to know what to add to install targets)

---

## Task 1 — Create Gazebo World: `anhc_office_v2.sdf`

Create `src/anhc_simulation/worlds/anhc_office_v2.sdf`.

Copy the structure of `anhc_indoor.sdf` exactly, with these changes:
1. `<world name="anhc_office_v2_world">` (update world name)
2. In the `office_env` model, replace both `<uri>` values from `../models/office_v1.stl` → `../models/office_v2.stl`
3. Adjust `start_waypoint` and `goal_waypoint` poses to sensible positions inside the v2 office (use `(0, 0)` and `(5, 5)` as safe defaults — these can be refined after first visual inspection in Gazebo).
4. Keep all plugins, physics settings, lighting, and floor model identical.

---

## Task 2 — Generate 2D Navigation Map for office_v2

The map must be generated from the Gazebo simulation using `slam_toolbox`. Follow these steps:

### Step 2a — Launch simulation with office_v2 world
```bash
cd ~/anhc_botrl
source install/setup.bash
ros2 launch anhc_simulation anhc_sim_mapping.launch.py world:=anhc_office_v2
```

### Step 2b — Teleoperate the robot to build the map
```bash
# In a second terminal
ros2 launch anhc_simulation anhc_teleop.launch.py
```
Drive the robot through all corridors and rooms in the office_v2 environment. The SLAM map will update in real time in RViz.

### Step 2c — Save the map
```bash
# When map coverage is complete:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'src/anhc_simulation/maps/anhc_office_v2_map'}}"
```
This saves:
- `src/anhc_simulation/maps/anhc_office_v2_map.pgm`
- `src/anhc_simulation/maps/anhc_office_v2_map.yaml`

### Step 2c alternative — If slam_toolbox save is unavailable:
```bash
ros2 run nav2_map_server map_saver_cli \
  -f src/anhc_simulation/maps/anhc_office_v2_map \
  --ros-args -p save_map_timeout:=5.0
```

### Step 2d — Verify map YAML
Open `src/anhc_simulation/maps/anhc_office_v2_map.yaml` and confirm it contains:
```yaml
image: anhc_office_v2_map.pgm
mode: trinary
resolution: 0.050   # 5 cm per cell — same as v1
origin: [<x>, <y>, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```
The `origin` values are populated automatically by the map saver.

> **Note:** If the map cannot be generated interactively right now, create **placeholder** map files that the AI should fill in later, and document in comments where the actual PGM/YAML values will come from.

---

## Task 3 — Update Launch Files

### 3a — `anhc_nav.launch.py`

File: `src/anhc_simulation/launch/anhc_nav.launch.py`

The `map_file` and `world` parameters should **already support arbitrary values** via launch arguments. Confirm (and fix if needed) that:
```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  map_file:=$(ros2 pkg prefix anhc_simulation)/share/anhc_simulation/maps/anhc_office_v2_map.yaml
```
works correctly. The launch file must:
1. Accept `world` argument (currently present: default `anhc_indoor`).
2. Accept `map_file` argument with absolute or package-relative path.
3. Pass the world name to `anhc_sim.launch.py` via its `world` argument.
4. Pass `map_file` to `map_server` / `nav2_map_server` node.

If the launch file already supports this, add a convenience **shorthand** argument `use_office_v2:=false` that, when set to `true`, automatically sets `world:=anhc_office_v2` and `map_file` to the package-relative path of `anhc_office_v2_map.yaml`:
```python
DeclareLaunchArgument('use_office_v2', default_value='false',
                      description='Use office_v2 world and map.'),
```
Use `IfCondition` / `PythonExpression` substitution to select the correct world and map file.

### 3b — `anhc_sim.launch.py`

File: `src/anhc_simulation/launch/anhc_sim.launch.py`

Confirm the `world` argument is forwarded correctly. The existing default is `anhc_indoor` — no changes needed unless the path resolution is broken.

### 3c — `anhc_master.launch.py` (if it exists)

Add a `use_office_v2` pass-through argument to `anhc_master.launch.py` if it includes `anhc_nav.launch.py`.

---

## Task 4 — Update CMakeLists.txt

File: `src/anhc_simulation/CMakeLists.txt`

Ensure the new files are installed:
```cmake
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY maps    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models  DESTINATION share/${PROJECT_NAME})
```
If these `install` lines already exist (they likely do), no change is needed.

---

## Task 5 — Update Benchmark Scenarios

File: `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml`

Add/verify `scenario_4` uses the new map name:
```yaml
scenario_4:
  name: "office_v2_cross_room"
  map_file: "anhc_office_v2_map"    # used as label; actual file set via launch arg
  start: [0.0, 0.0]
  goal: [5.0, 5.0]                  # update after inspecting actual map
  algorithms: [astar, dijkstra, theta_star, greedy_bfs, jps, rrt_star, dstar_lite, prm]
  trials: 3
```

---

## Task 6 — RViz Configuration for office_v2

The existing RViz configs (`anhc_nav.rviz`, `anhc_full_system.rviz`) work for any map because they subscribe to topic names, not file names. No changes needed unless the TF frames differ.

However, update the RViz `Fixed Frame` to `map` (already the case) and confirm the `Map` display subscribes to `/map` and `/costmap/global` — these topics do not change.

---

## Verification

```bash
cd ~/anhc_botrl
colcon build --symlink-install --packages-select anhc_simulation
source install/setup.bash

# Option A: with pre-saved map
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  map_file:=$(ros2 pkg prefix anhc_simulation)/share/anhc_simulation/maps/anhc_office_v2_map.yaml \
  algorithm:=astar

# Option B: shorthand
ros2 launch anhc_simulation anhc_nav.launch.py use_office_v2:=true algorithm:=astar
```

Expected result:
- Gazebo opens with `office_v2.stl` environment.
- RViz shows the pre-built `anhc_office_v2_map` (black/white map).
- Robot spawns at origin; AMCL localises.
- Setting a **2D Goal Pose** in RViz produces a planned path using the chosen algorithm.

---

## Notes on STL Mesh Alignment

If the `office_v2.stl` mesh is geometrically offset from the origin (which is common for exported CAD files), the map will be misaligned. Adjust the `<pose>` tag in `anhc_office_v2.sdf` accordingly:
```xml
<pose>x_offset y_offset 0 0 0 rotation_z</pose>
```
The correct offset can be found by:
1. Loading the STL in Gazebo and noting where the floor-level walls start.
2. Comparing with the map's `origin` field in the YAML.

Document the final pose in a comment in the SDF file.
