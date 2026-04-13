## Chạy full từ đầu đến cuối (copy-paste được)

Mình viết theo đúng flow ổn định nhất cho repo hiện tại của bạn.

---

### 0) Mở **3 terminal** và vào workspace

```bash
cd /home/anhuynh/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Làm như trên cho cả 3 terminal.

---

## PHASE A — Tạo map (nếu chưa có map chuẩn)

### Terminal 1: build các package cần
```bash
cd /home/anhuynh/anhc_botrl
source /opt/ros/jazzy/setup.bash
colcon build --packages-select anhc_simulation anhc_localization anhc_mapping anhc_planning anhc_viz --symlink-install
source install/setup.bash
```

### Terminal 1: chạy sim + mapping
```bash
ros2 launch anhc_simulation anhc_sim_mapping.launch.py
```

### Terminal 2: teleop để lái robot quét map
```bash
cd /home/anhuynh/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel -p use_sim_time:=true
```

```bash
ros2 launch anhc_simulation anhc_teleop.launch.py
```

Lái robot đi hết môi trường.

### Terminal 3: lưu map
```bash
cd /home/anhuynh/anhc_botrl
source /opt/ros/jazzy/setup.bash
source install/setup.bash
mkdir -p "$HOME/maps"
bash scripts/anhc_save_map.sh anhc_indoor_map
```

### Verify map tồn tại
```bash
ls -la "$HOME/maps/anhc_indoor_map.yaml" "$HOME/maps/anhc_indoor_map.pgm"
```

> Nếu script vẫn lưu sai chỗ, dùng lệnh fallback:
```bash
ros2 run nav2_map_server map_saver_cli -f "$HOME/maps/anhc_indoor_map" --ros-args -p use_sim_time:=true
```

---

## PHASE B/C — Navigation + planner + follower mới

### 1) Dừng toàn bộ launch cũ (quan trọng)
```bash
pkill -f "ros2 launch" || true
pkill -f gz || true
```

### 2) Build lại planning stack
```bash
cd /home/anhuynh/anhc_botrl
source /opt/ros/jazzy/setup.bash
colcon build --packages-select anhc_localization anhc_mapping anhc_planning anhc_perception anhc_simulation --symlink-install
source install/setup.bash
```

### 3) Launch full system với map đã lưu
```bash
MAP_FILE="$HOME/maps/anhc_indoor_map.yaml"
ros2 launch anhc_simulation anhc_master.launch.py algorithm:=astar map_file:=$MAP_FILE use_rviz:=false
```

---

## Checklist verify nhanh (terminal khác)

### A. Node follower có chạy chưa
```bash
source /opt/ros/jazzy/setup.bash
source /home/anhuynh/anhc_botrl/install/setup.bash
ros2 node list | rg "anhc_path_follower|anhc_global_planner|anhc_costmap_node"
```

### B. Publish goal
```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

### C. Theo dõi cmd_vel angular (ổn định dao động)
```bash
timeout 15s ros2 topic echo /cmd_vel --field angular.z
```

### D. Cross-track error
```bash
timeout 15s ros2 topic echo /planning/debug/cross_track_error | python3 -c "
import sys
vals=[float(l.split('data: ')[-1]) for l in sys.stdin if 'data:' in l]
if vals:
    print(f'Mean cross-track error: {sum(vals)/len(vals):.3f}m')
    print(f'Max cross-track error:  {max(vals):.3f}m')
    print('PASS' if max(vals) < 0.5 else 'FAIL')
else:
    print('No samples')
"
```

### E. Lookahead publish rate
```bash
timeout 8s ros2 topic hz /planning/debug/lookahead_point
```

### F. Switch Dijkstra regression test
```bash
ros2 param set /anhc_global_planner algorithm dijkstra
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 2.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}" --once
```

### G. Replan trigger test
```bash
ros2 topic pub /planning/replan_request std_msgs/msg/Bool "{data: true}" --once
timeout 8s ros2 topic echo /planning/stats --once
```

---

## Nếu gặp lỗi giống trước (map_server fail)

Chạy ngay:
```bash
ls -la "$HOME/maps/anhc_indoor_map.yaml"
```

- Nếu **không có file**: quay lại Phase A để tạo map.
- Nếu có file mà vẫn fail: mở YAML kiểm tra `image:` có trỏ đúng `.pgm` không:
```bash
cat "$HOME/maps/anhc_indoor_map.yaml"
```
`image:` nên là path đúng (thường cùng thư mục).

---

Nếu bạn muốn, mình có thể gửi thêm một phiên bản “1 script chạy toàn bộ verify tự động” để bạn chỉ cần chạy 1 lệnh.