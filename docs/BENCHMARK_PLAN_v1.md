# Kế hoạch Benchmark — anhc_botrl
## Từ kịch bản đến biểu đồ so sánh Classical vs RL

> **Phiên bản:** 2.0  
> **Cập nhật:** 2026-04-19  
> **Tham chiếu:** `METRIX.md`, `HOW_TO_NAVIGATE.md`

---

## Mục tiêu

- So sánh **cùng một tập kịch bản** giữa nhiều thuật toán global planner trên cùng map và điều kiện kiểm soát.
- Thu thập đủ **L_planned, L_executed, Tc, Tg, S, SR, C** cho mỗi cặp `(scenario × algorithm × trial)`.
- Xuất **CSV** → phân tích bằng Python → **Radar chart** đặt tất cả thuật toán trên một hình để thấy trade-off ngay.
- Tạo **baseline document** làm chuẩn so sánh khi RL agent được train xong.

---

## Danh sách thuật toán

```yaml
algorithms:
  - astar
  - dijkstra
  - greedy_bfs
  - theta_star
  - bidirectional_astar
  - rrt_star       # stochastic — cần N trials lớn hơn
  - dstar_lite
  # Dành chỗ cho RL agent (thêm sau):
  # - rl_ppo
  # - rl_sac
```

---

## Phần 1 — Bộ Metric chuẩn

### 1.1 Metric bắt buộc

| Ký hiệu | Tên đầy đủ | Đơn vị | Định nghĩa chính xác |
|---|---|---|---|
| **L_planned** | Planned Path Length | m | Tổng độ dài Euclidean của polyline `/planning/path` — phản ánh chất lượng planner thuần túy |
| **L_executed** | Executed Path Length | m | Tổng quãng đường từ `/odometry/filtered` trong suốt phase đi theo path — phản ánh cả planner + follower + control |
| **L_ratio** | Path Execution Ratio | — | `L_executed / L_planned`; gần 1.0 = follower bám tốt; >> 1.0 = drift nhiều |
| **Tc** | Computation Time | ms | Thời gian planner tìm đường, lấy từ `/planning/stats → planning_time_ms` |
| **Tg** | Time to Goal | s | Thời gian từ khi nhận path đến khi robot đạt goal trong tolerance — bắt buộc để so sánh RL |
| **S** | Path Smoothness | rad/m | `Σ\|Δθᵢ\| / L_planned` trên các đoạn liên tiếp của path — đo độ gấp khúc trung bình |
| **num_turns** | Number of Turns | count | Số lần `\|Δθ\| > 15°` (0.26 rad) trên path — bổ sung cho S |
| **max_curvature** | Max Curvature | rad/m | Góc cua lớn nhất tại một điểm trên path |
| **SR** | Success Rate | % | `100 × success_trials / total_trials` theo nhóm `(algorithm × scenario)` |
| **C_min** | Min Safety Clearance | m | Khoảng cách tối thiểu từ bất kỳ điểm nào trên path đến obstacle (từ `/costmap/global` hoặc `/map`) |
| **C_avg** | Avg Safety Clearance | m | Khoảng cách trung bình từ các điểm mẫu trên path đến obstacle gần nhất |

### 1.2 Metric bổ sung (ghi nếu có, không bắt buộc)

| Ký hiệu | Tên | Đơn vị | Ghi chú |
|---|---|---|---|
| **XTE_max** | Max Cross-Track Error | m | Từ `/planning/debug/cross_track_error` — nếu topic đã được implement |
| **XTE_avg** | Avg Cross-Track Error | m | Trung bình cross-track error trong suốt hành trình |
| **cpu_percent** | CPU Usage | % | `psutil.cpu_percent()` khi đang plan |
| **memory_mb** | Memory Usage | MB | `psutil.Process().memory_info().rss / 1024²` |
| **nodes_expanded** | Nodes Expanded | count | Số node đã expand (chỉ có với grid-based planners) |
| **replanning_count** | Replan Count | count | Số lần replanning xảy ra trong một trial |

### 1.3 Định nghĩa SUCCESS / FAILURE (cho SR)

Một trial được tính **SUCCESS** khi thoả mãn **tất cả** điều kiện:

```
✓ Planner trả về path có ít nhất 2 waypoints
✓ path không có điểm nào có costmap cell >= 90 (lethal zone)
✓ Robot đến trong vòng goal_tolerance_m = 0.30 m so với goal
✓ Không vượt quá planning_timeout_ms = 10_000 ms
✓ Không vượt quá execution_timeout_s = 120 s
✓ Robot không bị stuck (velocity < 0.01 m/s trong > 10 s khi chưa đến goal)
```

Một trial được tính **FAILED** và lý do được ghi vào `failure_reason`:

| failure_reason | Mô tả |
|---|---|
| `no_path` | Planner trả về path rỗng |
| `planning_timeout` | Tc > planning_timeout_ms |
| `execution_timeout` | Tg > execution_timeout_s |
| `robot_stuck` | Không di chuyển trong 10 giây liên tiếp |
| `tf_error` | TF lookup failed trong quá trình chạy |
| `system_error` | Lỗi khác (node crash, bridge down...) |

---

## Phần 2 — Thiết kế Scenarios

### 2.1 Nguyên tắc thiết kế

- Mỗi scenario có: **tên**, **loại** (tag), **start (x, y, θ)** và **goal (x, y, θ)** trong frame `map`, **số trial**.
- Ưu tiên cùng map `office_v2` trước; sau mới thêm map khác để generalize.
- Loại scenario (tag): `straight`, `single_turn`, `multi_turn`, `narrow`, `long_range`, `dynamic` — giúp giải thích radar theo từng nhóm.
- Đơn vị góc θ: **radian**. `0.0` = hướng East (trục X dương).

### 2.2 Số trial khuyến nghị theo loại thuật toán

| Loại thuật toán | Ví dụ | N trials tối thiểu | Lý do |
|---|---|---|---|
| Deterministic | A*, Dijkstra, Theta* | 5 | Kết quả giống nhau → ít trial vẫn đủ |
| Stochastic | RRT*, RRT | 20 | Kết quả ngẫu nhiên → cần nhiều trial để mean ổn định |
| RL agent | PPO, SAC | 20 | Có noise trong inference |

### 2.3 Danh sách scenarios

```yaml
# anhc_benchmark_scenarios.yaml
# Map: office_v2 | Frame: map | Đơn vị: mét, radian

scenarios:

  scenario_1:
    name: "straight_short"
    tag: "straight"
    description: "Đường thẳng ngắn, ít vật cản — baseline tốc độ thuần túy"
    map: "office_v2"
    start: {x: 0.0, y: 0.0, theta: 0.0}
    goal:  {x: 5.0, y: 0.0, theta: 0.0}
    trials_deterministic: 5
    trials_stochastic: 20

  scenario_2:
    name: "single_turn_90deg"
    tag: "single_turn"
    description: "Một góc cua 90° — đo khả năng xử lý turning đơn"
    map: "office_v2"
    start: {x: 0.0, y: 0.0, theta: 0.0}
    goal:  {x: 6.42551326751709, y: 7.196953773498535, theta: 0.0}
    trials_deterministic: 5
    trials_stochastic: 20

  scenario_3:
    name: "long_multi_obstacle"
    tag: "long_range"
    description: "Đường dài qua nhiều vật cản — đo quality of path phức tạp"
    map: "office_v2"
    start: {x: 5.033462047576904, y: -0.6205399036407471, theta: -1.5707963267948966}
    goal: {x: 3.8101, y: -8.8955, theta: 0.0}
    trials_deterministic: 5
    trials_stochastic: 20

  scenario_4:
    name: "narrow_corridor"
    tag: "narrow"
    description: "Hành lang hẹp (~1.5m) — đo clearance và safety score"
    map: "office_v2"
    start: {x: -4.6743, y: 7.0688, theta: 0.0}
    goal: {x: -5.8754, y: 9.3762, theta: 0.0}
    trials_deterministic: 5
    trials_stochastic: 20

  scenario_5:
    name: "u_turn_required"
    tag: "multi_turn"
    description: "Goal nằm sau lưng, phải vòng qua — đo khả năng quay đầu"
    map: "office_v2"
    start: {x: 5.4675, y: -0.4117, theta: 0.0}
    goal: {x: -0.7258, y: -0.1630, theta: 0.0}
    trials_deterministic: 5
    trials_stochastic: 20

  scenario_6:
    name: "multi_room_doors"
    tag: "multi_turn"
    description: "Đi qua 2–3 cửa phòng liên tiếp — đo khả năng điều hướng phức tạp"
    map: "office_v2"
    start: {x: 4.7131, y: 3.1482, theta: 0.0}
    goal: {x: 10.3400, y: -3.1807, theta: 0.0}
    trials_deterministic: 5
    trials_stochastic: 20

  scenario_7:
    name: "dynamic_obstacle"
    tag: "dynamic"
    description: "Một box di chuyển ngang qua đường giữa path — test lợi thế RL vs Classical"
    map: "office_v2"
    start: {x: 0.0, y: 0.0, theta: 0.0}
    goal:  {x: 8.0, y: 0.0, theta: 0.0}
    dynamic_obstacle:
      spawn: {x: 4.0, y: 2.0}
      velocity: {x: 0.0, y: -0.5}   # di chuyển về phía path
      activate_after_s: 3.0          # xuất hiện sau 3 giây
    trials_deterministic: 5
    trials_stochastic: 20
    note: "Classical planners cần D* Lite hoặc local planner để handle; RL agent react real-time"

# Tham số chung
common:
  planning_timeout_ms: 10000
  execution_timeout_s: 120
  goal_tolerance_m: 0.30
  robot_stuck_velocity_threshold: 0.01   # m/s
  robot_stuck_duration_s: 10.0
  pause_between_trials_s: 3.0
  pause_between_scenarios_s: 5.0
```

### 2.4 Reproducibility config

```yaml
reproducibility:
  random_seed: 42                # cho RRT* và các stochastic algo
  gz_pause_before_start_s: 2.0   # đảm bảo Gazebo ổn định trước khi bắt đầu
  robot_reset_method: teleport   # hoặc respawn
  robot_reset_timeout_s: 5.0     # chờ sau reset trước khi gửi goal
  costmap_clear_on_reset: true   # clear costmap giữa các trial
  use_sim_time: true             # bắt buộc khi chạy Gazebo
```

---

## Phần 3 — Schema CSV

### 3.1 Cột bắt buộc (mỗi row = 1 trial)

```
timestamp          : ISO 8601 string, thời điểm kết thúc trial
scenario_id        : string, ví dụ "scenario_1"
scenario_tag       : string, ví dụ "straight"
algorithm          : string, ví dụ "astar"
trial              : int, bắt đầu từ 1
success            : int, 1 = success / 0 = failed
failure_reason     : string, rỗng nếu success

L_planned_m        : float, độ dài path từ /planning/path
L_executed_m       : float, quãng đường thực từ odometry
L_ratio            : float, L_executed / L_planned

Tc_ms              : float, thời gian planning
Tg_s               : float, thời gian từ có path đến goal_reached

S_rad_per_m        : float, smoothness tổng quát
num_turns          : int,   số lần |Δθ| > 15°
max_curvature      : float, curvature lớn nhất

C_min_m            : float, clearance tối thiểu
C_avg_m            : float, clearance trung bình

SR_pct             : float, tính aggregate sau — để trống trong raw CSV
```

### 3.2 Cột bổ sung (ghi nếu có)

```
XTE_max_m          : float
XTE_avg_m          : float
cpu_percent        : float
memory_mb          : float
nodes_expanded     : int
replanning_count   : int
```

### 3.3 Tên file và vị trí

```
~/anhc_benchmark_results/
├── raw/
│   ├── benchmark_YYYYMMDD_HHMMSS.csv     ← raw mỗi lần chạy
│   └── ...
├── plots/
│   ├── radar_all_algorithms.png
│   ├── bar_path_length.png
│   ├── bar_planning_time.png
│   ├── bar_tg.png
│   ├── bar_smoothness.png
│   ├── bar_clearance.png
│   └── bar_success_rate.png
└── report.md                             ← tổng hợp tự động
```

---

## Phần 4 — Implement Benchmark Runner

### 4.1 Vòng đời một trial

```
Bước 1: Reset robot về start position
         → teleport hoặc respawn tại (start.x, start.y, start.theta)
         → chờ robot_reset_timeout_s

Bước 2: Clear costmap và old path
         → gọi service clear_costmap nếu có
         → publish empty path lên /planning/path

Bước 3: Set algorithm
         → ros2 param set /anhc_global_planner algorithm <algo>
         → chờ param được set thành công

Bước 4: Ghi t_start = now()

Bước 5: Gửi goal
         → publish PoseStamped lên /goal_pose
         → frame_id = "map"

Bước 6: Chờ và thu thập
         → subscribe /planning/stats → nhận Tc, L_planned, S, C
         → subscribe /odometry/filtered → tính L_executed
         → theo dõi distance to goal → phát hiện goal_reached
         → phát hiện robot_stuck
         → timeout nếu quá execution_timeout_s

Bước 7: Ghi kết quả vào CSV
         → 1 dòng với tất cả metrics
         → failure_reason nếu failed

Bước 8: Pause pause_between_trials_s
```

### 4.2 Hành vi bắt buộc (checklist)

- [ ] Đọc và validate YAML — thiếu key bắt buộc → log lỗi, bỏ qua scenario
- [ ] Set `ros2 param` algorithm và xác nhận đã set thành công trước khi gửi goal
- [ ] Gửi goal đúng frame `map`, đúng tọa độ từ YAML
- [ ] Phát hiện goal_reached từ odometry (distance < goal_tolerance_m)
- [ ] Phát hiện robot_stuck (velocity < threshold trong duration)
- [ ] Xử lý TF error không crash runner
- [ ] Ghi đúng 1 dòng CSV mỗi trial, kể cả failed trial
- [ ] Log tiến độ: `[scenario 2/7][algo dijkstra][trial 3/5]`
- [ ] Sau khi chạy xong toàn bộ → tự động gọi analyzer

### 4.3 Cách chạy

```bash
# Bước 1: Launch navigation stack
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=office_v2 use_rviz:=false

# Bước 2: Chờ stack ready (~30 giây), rồi launch benchmark
ros2 launch anhc_benchmark anhc_benchmark.launch.py \
  scenario_file:=$(ros2 pkg prefix anhc_benchmark)/share/\
anhc_benchmark/config/anhc_benchmark_scenarios.yaml

# Hoặc dùng master launch (tích hợp):
ros2 launch anhc_simulation anhc_master.launch.py \
  algorithm:=astar use_rviz:=false run_benchmark:=true \
  gz_extra_args:=-s
```

---

## Phần 5 — Phân tích và vẽ biểu đồ

### 5.1 Chuẩn hóa cho Radar chart

Tất cả trục radar phải theo hướng **càng ra ngoài = càng tốt**:

| Metric | Hướng tốt | Xử lý trước normalize |
|---|---|---|
| L_planned_m | Nhỏ hơn | Không cần (min-max, đảo dấu) |
| Tc_ms | Nhỏ hơn | **Log10 trước** để compress outlier (RRT* ~2000ms vs A* ~5ms) |
| Tg_s | Nhỏ hơn | Không cần |
| S_rad_per_m | Nhỏ hơn | Không cần |
| SR_pct | Lớn hơn | Không đảo dấu |
| C_min_m | Lớn hơn | Không đảo dấu |

```python
import numpy as np

def normalize_for_radar(values, higher_is_better=False,
                        log_scale=False):
    """
    Chuẩn hóa về [0, 1] với hướng "lớn hơn = tốt hơn".
    """
    v = np.array(values, dtype=float)
    if log_scale:
        v = np.log10(v + 1e-9)   # tránh log(0)
    vmin, vmax = v.min(), v.max()
    if vmax == vmin:
        return np.ones_like(v) * 0.5
    normalized = (v - vmin) / (vmax - vmin)
    if not higher_is_better:
        normalized = 1.0 - normalized
    return normalized
```

### 5.2 Danh sách biểu đồ bắt buộc

| File | Loại | Nội dung |
|---|---|---|
| `radar_all_algorithms.png` | Radar/Spider | Tất cả thuật toán, 6 trục: L, Tc, Tg, S, SR, C |
| `bar_path_length.png` | Bar + errorbar | L_planned mean ± std per algorithm, grouped by scenario |
| `bar_planning_time.png` | Bar + errorbar | Tc_ms, log scale Y-axis |
| `bar_time_to_goal.png` | Bar + errorbar | Tg_s per algorithm |
| `bar_smoothness.png` | Grouped bar | S và num_turns per algorithm |
| `bar_clearance.png` | Bar + errorbar | C_min và C_avg per algorithm |
| `bar_success_rate.png` | Bar | SR% per algorithm per scenario |
| `scatter_tc_vs_tg.png` | Scatter | Tc_ms (X) vs Tg_s (Y), mỗi algo một màu |

### 5.3 Cách chạy script

```bash
python3 src/anhc_viz/scripts/anhc_plot_comparison.py \
  --csv ~/anhc_benchmark_results/raw/benchmark_latest.csv \
  --out-dir ~/anhc_benchmark_results/plots/ \
  --radar-out radar_all_algorithms.png \
  --group-by scenario
```

### 5.4 Cấu trúc script

```python
# anhc_plot_comparison.py
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

def load_and_validate(csv_path):
    df = pd.read_csv(csv_path)
    required = ['algorithm', 'scenario_id', 'trial',
                'success', 'L_planned_m', 'Tc_ms',
                'Tg_s', 'S_rad_per_m', 'C_min_m']
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Thiếu cột: {missing}")
    return df

def compute_sr(df):
    """Tính SR% aggregate theo (algorithm, scenario_id)."""
    sr = df.groupby(['algorithm', 'scenario_id'])\
           ['success'].mean() * 100
    return sr.reset_index().rename(
        columns={'success': 'SR_pct'})

def plot_radar(summary_df, out_path):
    """Radar chart tất cả thuật toán."""
    metrics = ['L_planned_m', 'Tc_ms', 'Tg_s',
               'S_rad_per_m', 'SR_pct', 'C_min_m']
    labels  = ['Path Length', 'Plan Time (log)',
               'Time to Goal', 'Smoothness',
               'Success Rate', 'Clearance']
    higher_is_better = [False, False, False, False, True, True]
    log_scale        = [False, True,  False, False, False, False]
    # ... (implement radar drawing)
    plt.savefig(out_path, dpi=150, bbox_inches='tight')

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv', required=True)
    parser.add_argument('--out-dir', default='plots/')
    parser.add_argument('--radar-out',
                        default='radar_all_algorithms.png')
    parser.add_argument('--group-by',
                        default='all',
                        choices=['all', 'scenario'])
    args = parser.parse_args()
    df = load_and_validate(args.csv)
    # ... gọi các hàm plot
```

---

## Phần 6 — Báo cáo kết quả

### 6.1 Template report.md

```markdown
# Benchmark Report — anhc_botrl
**Map:** office_v2  
**Date:** YYYY-MM-DD  
**Algorithms tested:** astar, dijkstra, theta_star, rrt_star, greedy_bfs, bidir_astar, dstar_lite  
**Scenarios:** 7 (straight, single_turn, long_range, narrow, u_turn, multi_room, dynamic)  
**Trials per algo:** 5 (deterministic) / 20 (stochastic)

## Tổng hợp (mean ± 95% CI)

| Algorithm | L_m | Tc_ms | Tg_s | S | SR% | C_min_m |
|---|---|---|---|---|---|---|
| A* | 12.3±0.4 | 45±8 | 28±3 | 0.23±0.02 | 100% | 0.38±0.05 |
| Dijkstra | 12.3±0.4 | 89±12 | 29±4 | 0.23±0.02 | 100% | 0.38±0.05 |
| Theta* | 11.1±0.3 | 67±9 | 25±3 | 0.12±0.01 | 98% | 0.41±0.04 |
| RRT* | 13.2±1.8 | 1240±320 | 32±7 | 0.18±0.05 | 85% | 0.35±0.08 |

## Target cho RL agent

| Metric | Target | Nguồn gốc |
|---|---|---|
| SR | ≥ 95% | Best classical: 100% (A*) |
| Tg_s | ≤ 25s | Best classical: Theta* |
| C_min_m | ≥ 0.38m | Avg classical |
| L_planned_m | ≤ 1.2 × A* | Cho phép dài hơn 20% |
| Tc_ms | ≤ 10ms | Real-time inference |

## Radar Chart

![Radar](plots/radar_all_algorithms.png)

## Nhận xét

- **A* và Dijkstra:** Optimal path, 100% SR, nhưng Tc cao hơn Greedy BFS.
- **Theta*:** Path mượt nhất (S thấp nhất), Tg tốt nhất.
- **RRT*:** SR thấp nhất, Tc cao nhất — không phù hợp real-time.
- **Scenario dynamic:** Classical planners bị fail khi obstacle
  xuất hiện bất ngờ → cần replan. RL agent dự kiến handle tốt hơn.
```

### 6.2 Công thức 95% Confidence Interval

```python
import scipy.stats as stats

def confidence_interval_95(data):
    n = len(data)
    mean = np.mean(data)
    se = stats.sem(data)           # standard error
    ci = se * stats.t.ppf(0.975, df=n-1)
    return mean, ci

# Dùng:
# mean, ci = confidence_interval_95(df[df['algorithm']=='astar']['Tc_ms'])
# → print(f"{mean:.1f} ± {ci:.1f}")
```

> **Lưu ý:** Dùng 95% CI thay vì ± std vì CI phụ thuộc N — với N=5 thì CI rộng hơn std, phản ánh đúng mức độ không chắc chắn của ít sample.

---

## Phần 7 — Thứ tự thực hiện

```
Bước 1 — Cố định scenarios và YAML              [Việc 2.3]
          Nền cho mọi thứ phía sau.

Bước 2 — Smoke test runner                      [Việc 4]
          1 scenario × 2 algo × 2 trials
          → verify CSV được ghi đúng.

Bước 3 — Lấp đủ cột metric                     [Việc 3]
          Kiểm tra CSV mẫu bằng tay.

Bước 4 — Chạy full benchmark                   [Việc 4.3]
          Tất cả scenario × tất cả algo.

Bước 5 — Phân tích và vẽ biểu đồ              [Việc 5]
          CSV → plots/ → radar.

Bước 6 — Tạo report.md                         [Việc 6]
          Baseline document cho RL comparison.

Bước 7 — Train RL agent                         [Tương lai]
          Dùng cùng framework, cùng scenarios.

Bước 8 — So sánh RL vs Classical               [Tương lai]
          Đặt RL lên cùng radar chart.
```

---

## Phần 8 — Rủi ro và lưu ý

| Rủi ro | Mức độ | Cách xử lý |
|---|---|---|
| `use_sim_time` desync | Cao | Bắt buộc tất cả nodes dùng `use_sim_time: true` |
| RRT* kết quả ngẫu nhiên | Cao | N ≥ 20 trials, set `random_seed: 42` |
| Robot bị stuck giữa trial | Trung bình | Timeout + phát hiện stuck → ghi `robot_stuck` |
| Costmap chưa sẵn sàng khi gửi goal | Trung bình | Chờ costmap warm-up 5s sau reset |
| office_v2 mesh collision gây robot embed | Trung bình | Dùng simplified collision, spawn z = 0.15m |
| Metric C phụ thuộc inflation radius | Thấp | Ghi rõ trong report: dùng `/costmap/global` với inflation 0.55m |
| Radar bị squish do outlier | Thấp | Log10 cho Tc trước normalize |

---

## Liên kết

- Định nghĩa metric chi tiết: [`METRIX.md`](METRIX.md)
- Hướng dẫn điều hướng: [`HOW_TO_NAVIGATE.md`](HOW_TO_NAVIGATE.md)
- Scenarios YAML: `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml`
- Runner node: `src/anhc_benchmark/anhc_benchmark/anhc_benchmark_runner_node.py`
- Plot script: `src/anhc_viz/scripts/anhc_plot_comparison.py`
- Kết quả: `~/anhc_benchmark_results/`
