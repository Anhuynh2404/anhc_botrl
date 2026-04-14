# Phase 6 — Robot Visual Upgrade: OBJ Meshes + 4-Wheel Conversion

## Context

This is a ROS 2 (Jazzy / Gazebo Harmonic) mobile-robot project at `/home/anhuynh/anhc_botrl`.

The robot description package is `src/anhc_description/`.  
The simulation stack uses **Gazebo Harmonic** (`gz-sim`) with the `gz-sim-diff-drive-system` plugin.

### Current state of the robot (before this phase)

| Aspect | Current value |
|---|---|
| Chassis visual | Primitive `<box>` (no mesh) |
| Drive wheels | 2 × `continuous` joints: `left_wheel_joint`, `right_wheel_joint` |
| Rear support | 2 × `fixed` passive casters: `rear_left_caster_joint`, `rear_right_caster_joint` (sphere visual) |
| Wheel visual | Primitive `<cylinder>` (no mesh) |
| LiDAR visual | Primitive `<cylinder>` (no mesh) |
| Camera visual | Primitive `<box>` (no mesh) |
| DiffDrive joints | `left_wheel_joint`, `right_wheel_joint` only |

### New mesh assets (already on disk — do NOT create or move them)

All mesh files are located at:
```
src/anhc_description/meshes/anhc_botrl/visual/obj/
  baselink.obj          + baselink.mtl
  camera.obj            + camera.mtl
  lidar.obj             + lidar.mtl
  front_left_wheel.obj  + front_left_wheel.mtl
  front_right_wheel.obj + front_right_wheel.mtl
  back_left_wheel.obj   + back_left_wheel.mtl
  back_right_wheel.obj  + back_right_wheel.mtl
```

**Format:** Wavefront OBJ + MTL (exported from Blender 5.x).  
**Units:** Blender exports in **metres** — scale `"1 1 1"` in URDF.  
**Materials:** The `.mtl` file must reside in the **same directory** as its `.obj` file. RViz2 and Gazebo Harmonic load MTL automatically by matching base name.

**URDF mesh URI prefix:**  
`package://anhc_description/meshes/anhc_botrl/visual/obj/`

### Confirmed mesh bounding boxes (metres)

| Mesh file | Size (m) | Maps to URDF property |
|---|---|---|
| `baselink.obj` | 0.8 × 0.8 × 0.6 | chassis_length=0.8, chassis_width=0.8, chassis_height=0.6 |
| `front_left_wheel.obj` | 0.16 × 0.16 × 0.08 | wheel_radius=0.08, wheel_width=0.08 |
| `front_right_wheel.obj` | 0.16 × 0.16 × 0.08 | wheel_radius=0.08, wheel_width=0.08 |
| `back_left_wheel.obj` | 0.16 × 0.16 × 0.08 | wheel_radius=0.08, wheel_width=0.08 |
| `back_right_wheel.obj` | 0.16 × 0.16 × 0.08 | wheel_radius=0.08, wheel_width=0.08 |
| `camera.obj` | ~0.08 × 0.18 × 0.06 | camera collision box |
| `lidar.obj` | ~0.10 × 0.08 × 0.06 | lidar (no collision) |

---

## Goal

1. **Replace all primitive visuals** in `anhc_bot.urdf.xacro` with the corresponding OBJ meshes above.
2. **Update xacro properties** to match the new robot dimensions (chassis and wheel sizes above).
3. **Convert from 2-wheel + 2-caster to 4-wheel**: remove passive casters, add real front and rear wheels.
4. **Update `anhc_bot.gazebo.xacro`** so the DiffDrive plugin drives all 4 wheels (2 per side) and friction properties apply to all 4 wheel links.
5. Keep collision geometry as **primitives** (box/cylinder) — OBJ collision meshes are intentionally avoided for simulation stability.
6. Keep all sensor plugins (`gpu_lidar`, `camera`, `depth_camera`, `imu`) **unchanged**.
7. **Set `GZ_SIM_RESOURCE_PATH`** in `anhc_sim.launch.py` so Gazebo Harmonic can resolve `model://` URIs that it generates internally when parsing the URDF.

---

## Key Files to Read Before Starting

- `src/anhc_description/urdf/anhc_bot.urdf.xacro` — current URDF (full read required)
- `src/anhc_description/urdf/anhc_bot.gazebo.xacro` — Gazebo plugins (full read required)
- `src/anhc_simulation/launch/anhc_sim.launch.py` — launch file (check GZ_SIM_RESOURCE_PATH)

---

## Task 1 — Update `anhc_bot.urdf.xacro`

### 1.1  Update xacro properties to match new mesh dimensions

At the top of the file, update:

```xml
<xacro:property name="chassis_length" value="0.8"/>
<xacro:property name="chassis_width"  value="0.8"/>
<xacro:property name="chassis_height" value="0.6"/>
<xacro:property name="wheel_radius"   value="0.08"/>
<xacro:property name="wheel_width"    value="0.08"/>
```

> Keep `wheel_x`, `wheel_y`, `chassis_mass`, `wheel_mass` values as-is unless physically unreasonable
> for the new dimensions. Recalculate inertia tensors if mass values change significantly.

### 1.2  Base link (chassis)

Replace the primitive `<box>` visual of `base_link` with:

```xml
<!-- OBJ from Blender in metres; scale = 1 1 1. MTL auto-loaded by matching base name. -->
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://anhc_description/meshes/anhc_botrl/visual/obj/baselink.obj"
          scale="1 1 1"/>
  </geometry>
</visual>
```

Update the collision `<box>` to use the new chassis dimensions:

```xml
<collision>
  <origin xyz="0 0 ${chassis_height/2.0}" rpy="0 0 0"/>
  <geometry>
    <box size="${chassis_length} ${chassis_width} ${chassis_height}"/>
  </geometry>
</collision>
```

### 1.3  Wheel macro — replace cylinder with OBJ mesh

Update the `wheel_link` xacro macro to accept a new parameter `mesh_file`:

```xml
<!-- mesh_file: OBJ filename under meshes/anhc_botrl/visual/obj/ -->
<xacro:macro name="wheel_link" params="name x y mesh_file">
  <link name="${name}">
    <inertial> ... (unchanged) ... </inertial>
    <!-- OBJ from Blender in metres; scale = 1 1 1. MTL auto-loaded by matching base name. -->
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <mesh filename="package://anhc_description/meshes/anhc_botrl/visual/obj/${mesh_file}"
              scale="1 1 1"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_width}"/>  <!-- primitive kept -->
      </geometry>
    </collision>
  </link>
  <joint name="${name}_joint" type="continuous">
    ... (unchanged except parent/child names follow ${name}) ...
  </joint>
</xacro:macro>
```

### 1.4  Remove old wheel instances + casters, add 4-wheel instances

**Delete** the following lines:
```xml
<xacro:wheel_link name="left_wheel"  x="${wheel_x}"  y="${wheel_y}"/>
<xacro:wheel_link name="right_wheel" x="${wheel_x}"  y="-${wheel_y}"/>
<xacro:caster_wheel name="rear_left_caster"  x="-0.18" y="0.16"/>
<xacro:caster_wheel name="rear_right_caster" x="-0.18" y="-0.16"/>
```

**Delete** the entire `caster_wheel` xacro macro definition block.

**Add** four wheel instances:

```xml
<!-- Front wheels (powered, drive wheels) -->
<xacro:wheel_link name="front_left_wheel"  x="${wheel_x}"  y="${wheel_y}"  mesh_file="front_left_wheel.obj"/>
<xacro:wheel_link name="front_right_wheel" x="${wheel_x}"  y="-${wheel_y}" mesh_file="front_right_wheel.obj"/>

<!-- Rear wheels — same joint type (continuous), driven passively via physics -->
<xacro:wheel_link name="back_left_wheel"   x="-0.18"       y="${wheel_y}"  mesh_file="back_left_wheel.obj"/>
<xacro:wheel_link name="back_right_wheel"  x="-0.18"       y="-${wheel_y}" mesh_file="back_right_wheel.obj"/>
```

> **Note on rear wheel x position:** `-0.18` may need adjustment if the new chassis (0.8m long)
> requires wider wheel base. Centre the rear wheels proportionally to the chassis length.
> A safe value: `x="-${chassis_length/2.0 - 0.05}"`.

### 1.5  LiDAR link

Replace primitive `<cylinder>` visual of `lidar_link` with:

```xml
<!-- OBJ from Blender in metres; scale = 1 1 1. MTL auto-loaded by matching base name. -->
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://anhc_description/meshes/anhc_botrl/visual/obj/lidar.obj"
          scale="1 1 1"/>
  </geometry>
</visual>
```

Keep `lidar_link` with **no `<collision>`** (original intent: avoid spurious contacts).

### 1.6  Camera link

Replace primitive `<box>` visual of `camera_link` with:

```xml
<!-- OBJ from Blender in metres; scale = 1 1 1. MTL auto-loaded by matching base name. -->
<visual>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <geometry>
    <mesh filename="package://anhc_description/meshes/anhc_botrl/visual/obj/camera.obj"
          scale="1 1 1"/>
  </geometry>
</visual>
```

Keep the collision `<box>` **unchanged**.

---

## Task 2 — Update `anhc_bot.gazebo.xacro`

### 2.1  Friction properties — cover all 4 wheels

Replace the two existing `<gazebo reference="left_wheel">` and `<gazebo reference="right_wheel">`
blocks with four identical blocks, one per wheel:

```xml
<gazebo reference="front_left_wheel">  <mu1>1.2</mu1><mu2>1.2</mu2><kp>100000.0</kp><kd>10.0</kd> </gazebo>
<gazebo reference="front_right_wheel"> <mu1>1.2</mu1><mu2>1.2</mu2><kp>100000.0</kp><kd>10.0</kd> </gazebo>
<gazebo reference="back_left_wheel">   <mu1>1.2</mu1><mu2>1.2</mu2><kp>100000.0</kp><kd>10.0</kd> </gazebo>
<gazebo reference="back_right_wheel">  <mu1>1.2</mu1><mu2>1.2</mu2><kp>100000.0</kp><kd>10.0</kd> </gazebo>
```

### 2.2  DiffDrive plugin — drive all 4 wheels

The `gz-sim-diff-drive-system` plugin supports **multiple `<left_joint>` / `<right_joint>` tags**.
Replace the single-joint version:

```xml
<!-- OLD (delete) -->
<left_joint>left_wheel_joint</left_joint>
<right_joint>right_wheel_joint</right_joint>
```

with:

```xml
<!-- NEW: 4WD — both left wheels share the same velocity command, same for right -->
<left_joint>front_left_wheel_joint</left_joint>
<left_joint>back_left_wheel_joint</left_joint>
<right_joint>front_right_wheel_joint</right_joint>
<right_joint>back_right_wheel_joint</right_joint>
```

Update `wheel_separation` and `wheel_radius` to match new properties:

```xml
<wheel_separation>0.42</wheel_separation>  <!-- recalculate: 2 × wheel_y -->
<wheel_radius>0.08</wheel_radius>
```

Keep **all other plugin parameters unchanged** (topics, odom settings, etc.).

### 2.3  Remove caster Gazebo references (if any exist)

Search for any `<gazebo reference="rear_left_caster">` or `<gazebo reference="rear_right_caster">`
blocks and delete them.

---

## Task 3 — Update `anhc_sim.launch.py`

### 3.1  Set GZ_SIM_RESOURCE_PATH

Gazebo Harmonic converts `package://` URIs to `model://` internally when parsing URDF → SDF.
The `GZ_SIM_RESOURCE_PATH` must point to the **parent directory** of the package share so Gazebo
can resolve `model://anhc_description/meshes/...`.

Add to `generate_launch_description()` (before the `LaunchDescription` return):

```python
from ament_index_python.packages import get_package_share_directory
import os

_pkg_share = get_package_share_directory("anhc_description")
_gz_extra_path = os.path.dirname(_pkg_share)   # e.g. install/anhc_description/share
_existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
_gz_resource_path = _gz_extra_path + (":" + _existing_gz_path if _existing_gz_path else "")
```

Add inside `LaunchDescription([...])`:

```python
SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", _gz_resource_path),
```

---

## Constraints & Rules

| Rule | Detail |
|---|---|
| **Do not modify sensor plugins** | `gpu_lidar`, `camera`, `depth_camera`, `imu` sections are untouched |
| **Do not modify inertia values** | Keep all `<inertial>` blocks as they are (unless chassis_mass changes) |
| **Do not modify joint dynamics** | Keep `<limit>`, `<dynamics>` on wheel joints |
| **Collision geometry stays primitive** | Never add OBJ `<mesh>` inside `<collision>` |
| **OBJ scale is always 1 1 1** | Blender exports in metres — no unit conversion needed |
| **MTL lives next to OBJ** | Never move or rename `.mtl` files; they are auto-resolved by base name |
| **One file per task** | Modify only `anhc_bot.urdf.xacro` (Task 1), `anhc_bot.gazebo.xacro` (Task 2), `anhc_sim.launch.py` (Task 3) |
| **No new files** | Do not create new files; edit existing ones only |
| **Preserve XML formatting** | Match 2-space indentation style of the existing file |

---

## Verification

### Build check

```bash
cd ~/anhc_botrl
colcon build --symlink-install --packages-select anhc_description anhc_simulation
source install/setup.bash
```

No errors expected.

### URDF parse check

```bash
check_urdf <(xacro src/anhc_description/urdf/anhc_bot.urdf.xacro)
```

Expected output: `"robot name is: anhc_bot"` with no errors.  
All 4 wheel links (`front_left_wheel`, `front_right_wheel`, `back_left_wheel`, `back_right_wheel`)
must appear in the link tree; **no caster links** should be present.

### Visual check in RViz

```bash
ros2 launch anhc_description display.launch.py
```

In RViz2:
- Robot model must show 4 wheels with OBJ mesh shapes (with material colours from MTL).
- Chassis must show OBJ mesh (not plain box).
- LiDAR and camera must show OBJ mesh shapes.
- No floating/detached links.
- TF tree: `base_footprint → base_link → {front_left_wheel, front_right_wheel, back_left_wheel, back_right_wheel, lidar_link, camera_link, imu_link}`

### Simulation check

```bash
ros2 launch anhc_simulation anhc_sim.launch.py
```

In Gazebo Harmonic:
- No `Unable to find file with URI [model://...]` errors in log.
- Robot spawns without crashing or sinking through the floor.
- All 4 wheels touch the ground.
- Sending a `/cmd_vel` command causes all 4 wheels to rotate:
  ```bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.0}}"
  ```
- Robot moves forward smoothly.

---

## Troubleshooting Guide

| Symptom | Likely cause | Fix |
|---|---|---|
| Mesh not visible in RViz | Wrong scale (OBJ in mm not m) | Confirm Blender export unit; change `scale="1 1 1"` to `scale="0.001 0.001 0.001"` |
| Mesh visible but no colour | MTL file not found | Ensure `.mtl` is in same directory as `.obj` and has the same base name |
| `model://` URI errors in Gazebo log | `GZ_SIM_RESOURCE_PATH` not set | Add `SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ...)` in launch file (Task 3) |
| Robot sinks into ground | Wheel collision radius doesn't match new mesh | Adjust `wheel_radius` property or collision cylinder radius |
| Robot spins in place | DiffDrive `wheel_separation` mismatch | Recalculate: `wheel_separation = 2 × wheel_y` |
| Wheels not spinning | Joint names in DiffDrive don't match URDF | Confirm joint names end with `_joint` suffix |
| `check_urdf` fails | Caster link/joint still referenced | Delete all `caster_wheel` macro calls and definition |
| OBJ mesh not found at runtime | Wrong package name in URI | URI must be `package://anhc_description/...` (not `file://`) |
| Mesh origin offset from TF frame | CAD origin ≠ URDF joint origin | Adjust `<visual><origin xyz="..."/>` per link to compensate offset |
