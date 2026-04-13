# Lỗi khi set goal pose trên Rviz
```bash
[rviz2-18] [INFO] [1776043837.705979726] [rviz2]: Setting goal pose: Frame:map, Position(8.80927, 4.28996, 0), Orientation(0, 0, 0.0967649, 0.995307) = Angle: 0.193833
[anhc_global_planner_node-16] [WARN] [1776043837.710587141] [anhc_global_planner]: [anhc_global_planner] goal received but no costmap yet — ignoring
```

1. goal received but no costmap yet
* map_server publish /map với transient_local (giữ bản tin cuối).
* anhc_global_costmap_node trước đây publish /costmap/global với QoS volatile → bản tin có thể đi trước lúc anhc_global_planner subscribe, nên planner không nhận được gì.
* Subscriber /map của planner cũng nên dùng transient_local để luôn nhận được map đã latched.
Đã sửa: Cùng profile QoS (reliable + transient_local, depth 1) cho sub /map và pub /costmap/global trong anhc_global_costmap_node.py, và cho sub /map + /costmap/global trong anhc_global_planner_node.py.

Thêm: Nếu goal đến khi chưa có map/costmap, goal được giữ lại và tự chạy plan khi map/costmap tới (_pending_goal + _flush_pending_goal).

2. AMCL: Failed to transform initial pose in time (extrapolation into the future)
3. RViz: LaserScan timestamp ... earlier than all the data in the transform cache

TF odom → base_footprint sai thời điểm
Node anhc_odom_tf_node trước đây mặc định gắn TF với get_clock().now() trong khi vị trí trong /odom thuộc header.stamp cũ hơn. Với SLAM, tf2 và bộ lọc thông điệp cần cùng mốc thời gian với laser; gán pose cũ vào “bây giờ” làm scan bị đặt sai → map phình, lặp vòng không khớp, nhìn như nhiều bản copy lệch.