# Hướng dẫn sử dụng Prompt Phases với Cursor AI

## Tổng quan các Phase

| File | Nội dung | Prerequisite |
|---|---|---|
| `phase_01_algorithms.md` | Triển khai 6 thuật toán mới (RRT*, D* Lite, Theta*, Greedy BFS, JPS, PRM) | Không có |
| `phase_02_inflation_layer.md` | Nâng cấp Inflation Layer (gradient bands 100→80→60→40→20→0) | Không có |
| `phase_03_benchmark_metrics.md` | Benchmark đầy đủ 6 nhóm metric + biểu đồ so sánh | Phase 1 + Phase 2 |
| `phase_04_office_v2_map.md` | Tích hợp map mới `office_v2.stl` | Không có |
| `phase_05_dashboard_viz.md` | Dashboard & visualization nâng cao | Phase 1 + 2 + 3 + 4 |
| `phase_06_robot_visual_4wheel.md` | Thay vỏ robot bằng mesh STL + chuyển từ 2 bánh + 2 caster sang 4 bánh thực | Không có |

---

## Cách dùng trong Cursor AI

### Cách 1 — Dùng `@` reference (khuyến nghị)

Trong chat box của Cursor AI, gõ `@` rồi chọn file:

```
@docs/prompts/phase_01_algorithms.md

Đọc và triển khai toàn bộ nội dung trong file prompt này.
```

Hoặc ngắn gọn hơn:

```
Triển khai @docs/prompts/phase_01_algorithms.md
```

Cursor sẽ tự động đọc nội dung file và thực thi tất cả các task được mô tả.

---

### Cách 2 — Dùng lệnh tường minh

```
Please read the file at docs/prompts/phase_01_algorithms.md and implement everything described in it.
```

```
Đọc file docs/prompts/phase_02_inflation_layer.md và thực hiện tất cả các task trong đó.
```

---

### Cách 3 — Chạy từng Task trong một Phase

Nếu bạn muốn kiểm soát từng phần, chỉ định task cụ thể:

```
@docs/prompts/phase_01_algorithms.md

Chỉ thực hiện Task cho thuật toán Theta* (theta_star_planner.py). 
Bỏ qua các thuật toán khác.
```

```
@docs/prompts/phase_03_benchmark_metrics.md

Chỉ thực hiện Task 4 (upgrade anhc_plot_results.py) — bỏ qua các task còn lại.
```

---

### Cách 4 — Chạy nhiều Phase song song (Advanced)

Phases 1, 2, và 4 **không phụ thuộc nhau** — có thể yêu cầu AI thực hiện đồng thời:

```
Hãy đọc và thực hiện cả ba file prompt sau đây song song:

1. @docs/prompts/phase_01_algorithms.md
2. @docs/prompts/phase_02_inflation_layer.md  
3. @docs/prompts/phase_04_office_v2_map.md

Thực hiện tất cả các task trong ba file đó.
```

> **Lưu ý:** Phase 3 và Phase 5 cần Phase 1 + 2 hoàn thành trước.

---

## Thứ tự khuyến nghị

```
Phase 1 ──┐
           ├──► Phase 3 ──┐
Phase 2 ──┘               ├──► Phase 5
                           │
Phase 4 ──────────────────┘
```

### Luồng thực hiện tối ưu:

**Bước 1:** Chạy Phase 1, 2, 4 (có thể song song hoặc tuần tự):
```
@docs/prompts/phase_01_algorithms.md   Implement this.
@docs/prompts/phase_02_inflation_layer.md   Implement this.
@docs/prompts/phase_04_office_v2_map.md   Implement this.
```

**Bước 2:** Sau khi Phase 1 và 2 xong → chạy Phase 3:
```
@docs/prompts/phase_03_benchmark_metrics.md   Implement this.
```

**Bước 3:** Sau khi tất cả Phase 1–4 xong → chạy Phase 5:
```
@docs/prompts/phase_05_dashboard_viz.md   Implement this.
```

---

## Sau khi AI hoàn thành mỗi Phase

### Build và kiểm tra:

```bash
cd ~/anhc_botrl
colcon build --symlink-install
source install/setup.bash
```

### Phase 1 — Kiểm tra planners:
```bash
python3 -c "
from anhc_planning.planners import (
    AStarPlanner, DijkstraPlanner, ThetaStarPlanner,
    GreedyBFSPlanner, JPSPlanner, RRTStarPlanner,
    DStarLitePlanner, PRMPlanner
)
print('OK — all 8 planners imported')
"
```

### Phase 2 — Kiểm tra costmap bands:
```bash
# Launch và quan sát trong RViz — costmap phải hiển thị màu gradient
ros2 launch anhc_simulation anhc_nav.launch.py algorithm:=astar
# Trong RViz: Global Costmap phải có màu đỏ gần tường, vàng/xanh xa tường
```

### Phase 3 — Chạy benchmark:
```bash
ros2 launch anhc_benchmark anhc_benchmark.launch.py \
  scenario_file:=$(ros2 pkg prefix anhc_benchmark)/share/anhc_benchmark/config/anhc_benchmark_scenarios.yaml
# Sau khi xong:
python3 src/anhc_viz/scripts/anhc_plot_results.py --csv ~/anhc_benchmark_results/benchmark_<latest>.csv
```

### Phase 4 — Kiểm tra map mới:
```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  map_file:=$(ros2 pkg prefix anhc_simulation)/share/anhc_simulation/maps/anhc_office_v2_map.yaml
```

### Phase 5 — Kiểm tra overlay:
```bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  algorithm:=astar use_path_overlay:=true
# Sau khi đặt goal, switch sang theta_star, đặt lại goal:
ros2 param set /anhc_global_planner algorithm theta_star
# Trigger so sánh:
ros2 service call /viz/compare_paths std_srvs/srv/Trigger {}
```

---

## Mẹo khi làm việc với Cursor AI

1. **Cung cấp context**: Trước mỗi prompt, có thể nói thêm trạng thái hiện tại, ví dụ:
   ```
   Phase 1 đã xong. Bây giờ hãy thực hiện @docs/prompts/phase_02_inflation_layer.md
   ```

2. **Yêu cầu đọc file trước**: Nếu AI có vẻ không hiểu codebase:
   ```
   Trước tiên đọc các file: src/anhc_planning/planners/astar_planner.py và 
   src/anhc_planning/planners/base_planner.py, sau đó thực hiện @docs/prompts/phase_01_algorithms.md
   ```

3. **Kiểm tra lỗi linter**: Sau mỗi phase, yêu cầu AI kiểm tra:
   ```
   Kiểm tra lỗi flake8 trong src/anhc_planning/anhc_planning/planners/ và sửa nếu có.
   ```

4. **Xử lý lỗi build**: Nếu `colcon build` báo lỗi, paste lỗi vào chat:
   ```
   colcon build báo lỗi sau, hãy sửa:
   [paste error here]
   ```

5. **Commit từng phase**: Sau khi mỗi phase build thành công:
   ```
   Tạo git commit cho Phase 1 với message phù hợp.
   ```

---

## Các câu lệnh mẫu hoàn chỉnh

### Tiếng Việt:
```
Đọc và triển khai toàn bộ nội dung trong @docs/prompts/phase_01_algorithms.md
Sau khi xong, build và chạy smoke test để xác nhận tất cả planner import được.
```

```
Triển khai @docs/prompts/phase_02_inflation_layer.md
Đảm bảo sau khi build, costmap hiển thị 5 dải màu gradient trong RViz.
```

### English:
```
Read @docs/prompts/phase_01_algorithms.md and implement everything in it.
```

```
Implement all tasks in @docs/prompts/phase_03_benchmark_metrics.md. 
Phases 1 and 2 are already complete.
```
