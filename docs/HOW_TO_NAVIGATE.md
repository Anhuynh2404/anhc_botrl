# HOW TO NAVIGATE

## 1) Quet map bang teleop (office_v2)

### Buoc 1 — Build va source

```bash
cd ~/anhc_botrl
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Buoc 2 — Chay mapping (Terminal A)

```bash
cd ~/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch anhc_simulation anhc_sim_mapping.launch.py world:=anhc_office_v2 use_rviz:=true
```

Neu RViz nhap nhay TF hoac may yeu GPU, dung cach on dinh hon:

```bash
ros2 launch anhc_simulation anhc_sim_mapping.launch.py world:=anhc_office_v2 use_rviz:=false
```

### Buoc 3 — Chay teleop (Terminal B)

```bash
cd ~/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch anhc_simulation anhc_teleop.launch.py
```

Trong cua so teleop: `i` tien, `,` lui, `j/l` quay, `k` dung.

### Buoc 4 — Luu map sau khi quet xong (Terminal C)

```bash
cd ~/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/anhc_save_map.sh office_v2_map_01
```

Map duoc luu tai:

```bash
MAP_DIR=$(ros2 pkg prefix anhc_simulation)/maps
echo "$MAP_DIR/office_v2_map_01.yaml"
```

---

## 2) Chay navigation voi map da luu

### A* (astar)

```bash
cd ~/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
MAP_DIR=$(ros2 pkg prefix anhc_simulation)/maps

ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  use_office_v2:=true \
  use_slam:=false \
  map_file:=$MAP_DIR/office_v2_map_01.yaml \
  algorithm:=astar \
  use_rviz:=true
```

### Dijkstra

```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  use_office_v2:=true \
  use_slam:=false \
  map_file:=$MAP_DIR/office_v2_map_01.yaml \
  algorithm:=dijkstra \
  use_rviz:=true
```

### Cac thuat toan global planner co san

| Gia tri `algorithm` | Ghi chu ngan |
|---------------------|--------------|
| `astar` | Mac dinh, can bang toc do / chat luong |
| `dijkstra` | Duong ngan nhat theo do dai canh (8-huong) |
| `greedy_bfs` | Greedy BFS theo heuristic (nhanh, khong toi uu) |
| `theta_star` | Any-angle tren luoi, duong it gap khuc hon |
| `jps` | Jump Point Search (nhanh tren luoi deu) |
| `rrt_star` | Sampling, tot cho khong gian rong, thoi gian bien doi |
| `dstar_lite` | Luoi, ho tro cap nhat khi costmap doi (sim: it dung incremental) |
| `prm` | Probabilistic Roadmap |
| `rl` | Planner RL (neu da cau hinh) |

Vi du:

```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  use_office_v2:=true \
  use_slam:=false \
  map_file:=$MAP_DIR/office_v2_map_01.yaml \
  algorithm:=theta_star \
  use_rviz:=true
```

### Dat goal trong RViz

1. Chon Fixed Frame la `map`.
2. Chon tool **2D Goal Pose**.
3. Click + keo de dat vi tri va huong dich.

---

## 3) Doi thuat toan khong restart

```bash
source ~/anhc_botrl/install/setup.bash
ros2 param set /anhc_global_planner algorithm astar
# greedy_bfs | theta_star | jps | rrt_star | dstar_lite | prm | ...
ros2 param get /anhc_global_planner algorithm
```

---

## 4) Troubleshooting nhanh

Neu RViz/Gazebo bi loạn do process cu con chay:

```bash
pkill -f "ros2 launch" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f rviz2 2>/dev/null
pkill -f slam_toolbox 2>/dev/null
```

Kiem tra scan va TF:

```bash
ros2 topic hz /scan
ros2 topic echo /tf --once
```
