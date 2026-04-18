# Kế hoạch Benchmark — từ kịch bản đến biểu đồ

Tài liệu này mô tả **tuần tự công việc** để xây dựng pipeline benchmark cho `anhc_botrl`, bám theo bộ metric trong [`METRIX.md`](METRIX.md). Các bước có thể giao cho người hoặc agent thực hiện theo thứ tự; phần “Trạng thái hiện tại” giúp đồng bộ với code đã có (`anhc_benchmark`, `anhc_viz`).

---

## Mục tiêu

- So sánh **cùng một tập kịch bản** giữa nhiều thuật toán global planner.
- Thu thập đủ **L, Tc, S, SR, C** (định nghĩa trong `METRIX.md`) cho mỗi cặp (scenario × algorithm × trial).
- Xuất **CSV** → phân tích bằng Python → **Radar chart** (tất cả thuật toán trên một hình) để thấy trade-off ngay.

---

## Chuẩn metric (tham chiếu)

| Ký hiệu | Tên | Đơn vị | Ghi chú ngắn |
|--------|-----|--------|----------------|
| **L** | Path Length | m | Tổng quãng đường từ xuất phát đến đích (theo path đã lập hoặc odometry thực tế — cần thống nhất trong implement) |
| **Tc** | Computation Time | ms | Thời gian planner tìm đường (lấy từ `/planning/stats` → `planning_time_ms` hoặc tương đương) |
| **S** | Path Smoothness | rad/m | Độ “gấp khúc” của polyline (ví dụ tổng \|Δθ\| / L trên các đoạn) |
| **SR** | Success Rate | % | Tỷ lệ trial thành công (có path, robot đến đích trong ngưỡng, không timeout) |
| **C** | Safety Clearance | m | Khoảng cách tối thiểu (hoặc trung bình) từ đường đi / robot tới vật cản (costmap hoặc map tĩnh) |

**Bổ sung gợi ý (không bắt buộc, hữu ích cho radar):**

- **Tg** — Time to goal (s): thời gian từ lúc có path đến lúc đạt goal (path follower); phân biệt chất lượng điều khiển với Tc.
- **XTE_max** — Cross-track error cực đại (m): nếu đã log từ `/planning/debug/cross_track_error`.

Nếu thêm metric, cập nhật `METRIX.md` và schema CSV cùng một lần để tránh lệch tài liệu và code.

---

## Trạng thái hiện tại (repo)

| Thành phần | Vị trí | Ghi chú |
|------------|--------|---------|
| Runner | `src/anhc_benchmark/anhc_benchmark/anhc_benchmark_runner_node.py` | Đổi algorithm, chạy scenario, ghi CSV |
| Scenarios | `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml` | Cần mở rộng / chuẩn hóa theo Việc 1 |
| Live metrics | `anhc_live_metrics_node.py` | JSON trên `/benchmark/live` |
| Analyzer | `anhc_results_analyzer_node.py` | Đọc CSV, báo cáo `report.md` |
| Plot (bar/line…) | `src/anhc_viz/scripts/anhc_plot_results.py` | **Cần bổ sung Radar chart** theo Việc 4 |
| Launch tích hợp | `anhc_master.launch.py` với `run_benchmark:=true` | |

---

# Việc 1 — Thiết kế Scenarios (kịch bản thí nghiệm)

**Mục đích:** Định nghĩa tập tình huống **tái lập được**, phản ánh đa dạng môi trường (hành lang thẳng, góc cua, chật, xa).

### 1.1 Nguyên tắc thiết kế

- Mỗi scenario: **tên**, **world/map** (nếu khác nhau), **start (x,y,θ)** và **goal (x,y,θ)** trong frame `map`, **số trial** `trials ≥ 1`.
- Ưu tiên **cùng map** (ví dụ `office_v2`) trước; sau mới thêm map khác để generalize.
- Phân loại scenario (nhãn nội bộ): `straight`, `single_turn`, `multi_turn`, `narrow`, `long_range` — giúp giải thích radar theo từng nhóm sau này.

### 1.2 Deliverable

- File `anhc_benchmark_scenarios.yaml` (hoặc tách `scenarios/office_v2.yaml` nếu file quá dài) với:
  - Danh sách đủ thuật toán cần so (`astar`, `dijkstra`, `greedy_bfs`, `theta_star`, `jps`, `rrt_star`, `dstar_lite`, … — khớp `algorithm` trong `anhc_global_planner`).
  - Tham số chung: timeout planning, timeout execution (nếu có), pause giữa các trial.

### 1.3 Tiêu chí “đủ tốt”

- Ít nhất **3–5 scenario** khác nhau trên cùng map đã dùng trong `HOW_TO_NAVIGATE.md`.
- Mỗi scenario có **mô tả một dòng** trong comment YAML (tiếng Việt hoặc Anh).

---

# Việc 2 — Implement Benchmark Runner tự động

**Mục đích:** Một lần chạy (launch) thực hiện toàn bộ ma trận scenario × algorithm × trial **không can thiệp tay**.

### 2.1 Hành vi cần có (checklist)

- [ ] Đọc file scenario (YAML), validate schema (thiếu key → log lỗi và bỏ qua scenario).
- [ ] Với mỗi trial: set `ros2 param` **algorithm** trên `anhc_global_planner`, đợi planner sẵn sàng.
- [ ] Gửi goal (PoseStamped `/goal_pose` hoặc topic mà stack đang dùng) đúng frame `map`.
- [ ] Chờ kết quả: path + stats JSON (`/planning/stats`) và/hoặc tín hiệu “goal reached” từ path follower / odometry.
- [ ] Ghi **một dòng CSV** mỗi trial với timestamp, scenario, algorithm, trial index, raw metrics.
- [ ] Xử lý lỗi: planning timeout, không có path, TF lỗi → đánh dấu trial **failed** (phục vụ SR).

### 2.2 Deliverable

- `anhc_benchmark_runner_node.py` cập nhật (hoặc module phụ) đảm bảo vòng đời trial **deterministic** (reset trạng thái giữa các trial nếu cần: ví dụ dừng robot, clear old path).

### 2.3 Cách chạy thử nhanh

```bash
ros2 launch anhc_simulation anhc_master.launch.py \
  algorithm:=astar use_rviz:=false run_benchmark:=true gz_extra_args:=-s
```

(hoặc `ros2 launch anhc_benchmark anhc_benchmark.launch.py` khi stack planning đã chạy sẵn — tùy cách bạn tổ chức pipeline.)

---

# Việc 3 — Thu thập đủ metrics theo METRIX.md

**Mục đích:** Mỗi trial có đủ trường để suy ra **L, Tc, S, SR, C** (và cột phụ nếu dùng).

### 3.1 Nguồn dữ liệu đề xuất

| Metric | Nguồn chính | Ghi chú implement |
|--------|-------------|-------------------|
| **L** | Tổng độ dài polyline `/planning/path` **hoặc** quãng đường lọc từ `/odometry/filtered` trong phase “đi theo path” | Thống nhất một định nghĩa trong báo cáo (path geometric vs executed). |
| **Tc** | JSON `/planning/stats` → `planning_time_ms` | Đã có từ `anhc_global_planner`. |
| **S** | Tính offline từ chuỗi waypoints: \(S \approx \sum_i |\Delta\theta_i| / L\) hoặc công thức tương đương | Implement hàm thuần Python trong `anhc_benchmark` hoặc `anhc_viz`. |
| **SR** | Aggregated: `100 * success_trials / total_trials` theo nhóm (algorithm × scenario) | Một trial success nếu có path và đạt goal trong tolerance / không abort. |
| **C** | Khoảng cách tối thiểu từ mẫu điểm trên path tới **obstacle** (map trắng/đen hoặc cost ≥ threshold) | Có thể dùng `/map` hoặc `/costmap/global` (lưu ý inflation). |

### 3.2 Schema CSV (đề xuất cột tối thiểu)

- `timestamp`, `scenario_id`, `algorithm`, `trial`, `success` (0/1)
- `L_m`, `Tc_ms`, `S_rad_per_m`, `C_m` (nếu tính được mỗi trial)
- `error_message` (rỗng nếu OK)

Sau khi chạy xong, script phân tích có thể pivot theo `algorithm` và scenario.

### 3.3 Deliverable

- Runner + (nếu cần) node nhỏ “metric_postprocess” hoặc logic trong analyzer để **không để trống** các cột bắt buộc.

---

# Việc 4 — Phân tích và vẽ biểu đồ (CSV → Python)

**Mục đích:** Từ CSV thô → thống kê → **Radar chart** so tất cả thuật toán trên một hình.

### 4.1 Chuẩn hóa cho radar

Radar cần các trục **cùng hướng “tốt hơn”** (ví dụ: L, Tc, S nhỏ hơn là tốt; SR, C lớn hơn là tốt). Với mỗi metric:

- Chuẩn hóa min-max theo batch (hoặc theo scenario):  
  `score = (value - min) / (max - min)` rồi **đảo dấu** nếu “nhỏ hơn là tốt” để mọi trục đều **càng ra ngoài càng tốt**.

### 4.2 Biểu đồ bắt buộc

- **Một figure Radar:** mỗi **thuật toán** một đường đa giác (cùng trục: L, Tc, S, SR, C hoặc bản đã chuẩn hóa).
- Tùy chọn: radar **theo từng scenario** (subplot) nếu không muốn trộn điều kiện khác nhau.

### 4.3 Deliverable

- Cập nhật hoặc thêm script (ví dụ mở rộng `anhc_plot_results.py`):
  - `--csv PATH`
  - `--radar-out PATH.png`
  - `--group-by scenario` (optional)
- Phụ thuộc Python: `matplotlib`, `numpy` (và `pandas` nếu tiện đọc CSV).

### 4.4 Báo cáo

- Giữ `~/anhc_benchmark_results/report.md` (hoặc thư mục repo `bench_results/`) với bảng tóm tắt + đường dẫn ảnh radar.

---

## Thứ tự thực hiện đề xuất

1. **Việc 1** — cố định scenario + YAML (nền cho mọi thứ).
2. **Việc 2** — đảm bảo runner chạy end-to-end một scenario × hai algorithm (smoke test).
3. **Việc 3** — lấp đủ cột metric và kiểm tra một file CSV mẫu tay.
4. **Việc 4** — script radar + một lần chạy demo lưu PNG.

---

## Rủi ro và lưu ý

- **use_sim_time:** toàn bộ node benchmark và planning phải đồng bộ `use_sim_time` khi chạy Gazebo.
- **RRT\*** có tính ngẫu nhiên: SR và L có thể dao động — nên **tăng trials** hoặc seed cố định nếu planner hỗ trợ.
- **C** phụ thuộc costmap inflation: ghi rõ trong báo cáo bạn dùng map tĩnh hay costmap global.

---

## Liên kết

- Định nghĩa metric: [`METRIX.md`](METRIX.md)
- Điều hướng sim/nav: [`HOW_TO_NAVIGATE.md`](HOW_TO_NAVIGATE.md)
- Prompt kỹ thuật chi tiết (nếu có): `docs/prompts/phase_03_benchmark_metrics.md`
