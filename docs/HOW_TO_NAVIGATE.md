# Quét map
## Chạy teleop để quét map

```bash
ros2 launch anhc_simulation anhc_teleop.launch.py
```

## Lưu map

```bash
ros2 run nav2_map_server map_saver_cli -f src/anhc_simulation/maps/name_map
```

# Hướng dẫn chạy Navigation (flow mới office_v2 + SLAM)

## Bước 1 — Build

```bash
cd ~/anhc_botrl
colcon build --symlink-install
source install/setup.bash
```

## Bước 2 — Launch office_v2 với A* (mặc định)

```bash
cd ~/anhc_botrl
source install/setup.bash

ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  use_office_v2:=true \
  use_slam:=true \
  algorithm:=astar \
  use_rviz:=true
```

## Bước 3 — Launch office_v2 với Dijkstra

```bash
cd ~/anhc_botrl
source install/setup.bash

ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  use_office_v2:=true \
  use_slam:=true \
  algorithm:=dijkstra \
  use_rviz:=true
```

## Bước 4 — Đặt goal trong RViz2

1. Chờ RViz2 mở và map SLAM bắt đầu được vẽ theo dữ liệu lidar.
2. Chờ robot model xuất hiện đúng vị trí. Với flow SLAM online (`use_slam:=true`) không dùng map tĩnh/AMCL ban đầu.
3. Trên toolbar RViz2, click **2D Goal Pose**.
4. Click và kéo trên map để đặt goal (click = vị trí, kéo = hướng robot).
5. Xem đường xanh (planned path) xuất hiện.
6. Robot tự động di chuyển theo path follower.

## Bước 5 — Switch thuật toán không cần restart

```bash
source ~/anhc_botrl/install/setup.bash

# A* (đã hoạt động trước)
ros2 param set /anhc_global_planner algorithm astar

# Dijkstra
ros2 param set /anhc_global_planner algorithm dijkstra

# Theta* — any-angle, đường thẳng hơn A*
ros2 param set /anhc_global_planner algorithm theta_star

# Greedy BFS — nhanh nhất, không tối ưu
ros2 param set /anhc_global_planner algorithm greedy_bfs

# Jump Point Search — nhanh, lưới đồng nhất
ros2 param set /anhc_global_planner algorithm jps

# RRT* — sampling-based, phù hợp không gian rộng
ros2 param set /anhc_global_planner algorithm rrt_star

# D* Lite — incremental replanning
ros2 param set /anhc_global_planner algorithm dstar_lite

# PRM — roadmap xác suất
ros2 param set /anhc_global_planner algorithm prm
```
ros2 param get /anhc_global_planner algorithm

Gửi goal mới sau khi switch (ví dụ dùng lại **2D Goal Pose** trong RViz2).

## Bước 6 — Xem thống kê thuật toán

```bash
ros2 topic echo /planning/stats
```

## Bước 7 — Chạy với map khác (map tĩnh đã lưu)

```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_indoor \
  use_slam:=false \
  map_file:=/path/to/your/map.yaml \
  algorithm:=astar
```

## Ghi chú về localization

- Flow khuyến nghị hiện tại cho môi trường `office_v2`: `use_office_v2:=true` + `use_slam:=true` để tạo map mới trực tiếp trong phiên chạy.
- Sau khi quét xong, lưu map bằng `map_saver_cli` rồi mới chuyển sang chế độ map tĩnh (`use_slam:=false` + `map_file:=...`).

- Map dạng **`.yaml` + `.pgm`** (như `anhc_indoor_map`) thường dùng với **`nav2_amcl`**. `anhc_nav.launch.py` tự chọn AMCL nếu **không** có cặp file serialize của slam_toolbox cùng tên base với map (`<tên_map>.posegraph` và `<tên_map>.data` nằm cùng thư mục với file YAML).
- Nếu bạn có map serialize từ **slam_toolbox**, đặt hai file đó cạnh file YAML và đặt `map_file` trỏ tới YAML đó; launch sẽ dùng **slam_toolbox** ở chế độ **localization** và tham số `map_file_name` trỏ tới base path (không đuôi).

## Tùy chọn launch

- `use_rviz:=false` — không mở RViz2.
- `gz_extra_args:=-s` — Gazebo headless (chỉ server).
- `world:=<tên_world>` — đổi world (không đuôi `.sdf`).
- `use_office_v2:=true` — shorthand cho world `anhc_office_v2` và bật flow phù hợp office_v2.
- `use_slam:=true|false` — `true` để quét map mới; `false` để chạy localization với map đã lưu.
