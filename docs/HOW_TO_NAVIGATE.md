# Hướng dẫn chạy Navigation với map đã lưu

## Bước 1 — Build

```bash
cd ~/anhc_botrl
colcon build --symlink-install
source install/setup.bash
```

## Bước 2 — Launch với A* (mặc định)

```bash
ros2 launch anhc_simulation anhc_nav.launch.py algorithm:=astar
```

## Bước 3 — Launch với Dijkstra

```bash
ros2 launch anhc_simulation anhc_nav.launch.py algorithm:=dijkstra
```

## Bước 4 — Đặt goal trong RViz2

1. Chờ RViz2 mở và map hiện ra (màu trắng/đen).
2. Chờ robot model xuất hiện đúng vị trí. Mặc định AMCL dùng initial pose `(0, 0, yaw=0)` trong `anhc_amcl_nav.yaml` (`set_initial_pose: true`) để ngay lập tức có TF `map→odom`. Nếu vị trí trên map lệch so với thực tế trong Gazebo, dùng **2D Pose Estimate** một lần để chỉnh lại.
3. Trên toolbar RViz2, click **2D Goal Pose**.
4. Click và kéo trên map để đặt goal (click = vị trí, kéo = hướng robot).
5. Xem đường xanh (planned path) xuất hiện.
6. Robot tự động di chuyển theo path follower.

## Bước 5 — Switch thuật toán không cần restart

```bash
# Switch sang Dijkstra
ros2 param set /anhc_global_planner algorithm dijkstra

# Switch về A*
ros2 param set /anhc_global_planner algorithm astar
```

Gửi goal mới sau khi switch (ví dụ dùng lại **2D Goal Pose** trong RViz2).

## Bước 6 — Xem thống kê thuật toán

```bash
ros2 topic echo /planning/stats
```

## Bước 7 — Chạy với map khác

```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  map_file:=/path/to/your/map.yaml \
  algorithm:=astar
```

## Ghi chú về localization

- Map dạng **`.yaml` + `.pgm`** (như `anhc_indoor_map`) thường dùng với **`nav2_amcl`**. `anhc_nav.launch.py` tự chọn AMCL nếu **không** có cặp file serialize của slam_toolbox cùng tên base với map (`<tên_map>.posegraph` và `<tên_map>.data` nằm cùng thư mục với file YAML).
- Nếu bạn có map serialize từ **slam_toolbox**, đặt hai file đó cạnh file YAML và đặt `map_file` trỏ tới YAML đó; launch sẽ dùng **slam_toolbox** ở chế độ **localization** và tham số `map_file_name` trỏ tới base path (không đuôi).

## Tùy chọn launch

- `use_rviz:=false` — không mở RViz2.
- `gz_extra_args:=-s` — Gazebo headless (chỉ server).
- `world:=<tên_world>` — đổi world (không đuôi `.sdf`).
