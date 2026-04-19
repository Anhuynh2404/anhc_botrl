# Phase 7 — Chuyển môi trường sang `office_v2.obj` + `office_v2.mtl`

## Bối cảnh

Bạn đang làm việc trong project ROS 2 tại:
`/home/anhuynh/anhc_botrl`

Môi trường mới đã được thiết kế và lưu tại:
- `src/anhc_simulation/models/office_v2.obj`
- `src/anhc_simulation/models/office_v2.mtl`

Mục tiêu: thay thế môi trường cũ đang dùng (v1 hoặc STL cũ) bằng môi trường mới `office_v2.obj/.mtl`, đảm bảo launch navigation chạy ổn định trong Gazebo + RViz.

---

## Yêu cầu thực hiện

### 1) Khảo sát và xác định điểm đang dùng môi trường cũ

Trước khi sửa, đọc các file sau:
- `src/anhc_simulation/worlds/anhc_indoor.sdf`
- `src/anhc_simulation/worlds/anhc_office_v2.sdf` (nếu đã tồn tại)
- `src/anhc_simulation/launch/anhc_sim.launch.py`
- `src/anhc_simulation/launch/anhc_nav.launch.py`
- `src/anhc_simulation/CMakeLists.txt`
- `src/anhc_simulation/maps/anhc_office_v2_map.yaml` (nếu có)

Tìm toàn bộ tham chiếu đang trỏ tới mesh cũ (ví dụ `office_v1.stl`, `office_v2.stl`, hoặc world cũ).

### 2) Cập nhật world SDF để dùng OBJ + MTL mới

Nếu đã có file `src/anhc_simulation/worlds/anhc_office_v2.sdf`, cập nhật trực tiếp.  
Nếu chưa có, tạo mới bằng cách copy cấu trúc từ `anhc_indoor.sdf`.

Trong model môi trường (thường là `office_env`):
- Đổi tất cả `<uri>` của visual/collision về:
  - `../models/office_v2.obj`
- Giữ nguyên physics/plugins/lights nếu đang ổn định.
- Đảm bảo world name hợp lệ, ví dụ: `anhc_office_v2_world`.

Lưu ý:
- `office_v2.mtl` phải nằm cùng thư mục với `office_v2.obj` như hiện tại.
- Không đổi tên file OBJ/MTL trừ khi bắt buộc.

### 3) Đảm bảo launch có thể chạy world mới

Kiểm tra và chỉnh:
- `src/anhc_simulation/launch/anhc_sim.launch.py`
- `src/anhc_simulation/launch/anhc_nav.launch.py`

Yêu cầu:
- Có thể truyền `world:=anhc_office_v2`.
- `anhc_nav.launch.py` forward world xuống `anhc_sim.launch.py`.
- `map_file` vẫn hoạt động đúng cho map v2:
  - `.../maps/anhc_office_v2_map.yaml`

Nếu hệ thống đã hỗ trợ rồi thì chỉ cần xác nhận, không refactor thừa.

### 4) Kiểm tra packaging/install

Trong `src/anhc_simulation/CMakeLists.txt`, đảm bảo các thư mục này được install:
- `models`
- `worlds`
- `maps`

Nếu đã có sẵn `install(DIRECTORY ...)` đúng thì không sửa.

### 5) Đồng bộ scenario benchmark (nếu có)

Trong `src/anhc_benchmark/config/anhc_benchmark_scenarios.yaml`:
- Xác nhận scenario dùng `anhc_office_v2_map`.
- Nếu start/goal cũ không còn hợp lý với map mới, cập nhật toạ độ an toàn.

### 6) Kiểm tra chất lượng sau khi sửa

Sau khi sửa, chạy:

```bash
cd ~/anhc_botrl
colcon build --symlink-install
source install/setup.bash
ros2 launch anhc_simulation anhc_nav.launch.py \
  world:=anhc_office_v2 \
  map_file:=$(ros2 pkg prefix anhc_simulation)/share/anhc_simulation/maps/anhc_office_v2_map.yaml \
  algorithm:=astar
```

Kỳ vọng:
- Gazebo hiển thị đúng mô hình văn phòng mới từ `office_v2.obj`.
- Material từ `office_v2.mtl` được nạp đúng (không bị mất texture/màu cơ bản).
- RViz hiển thị map và robot định vị được.
- Có thể đặt goal và planner tạo path bình thường.

### 7) Ràng buộc khi chỉnh sửa

- Chỉ thay những gì cần để migration sang môi trường mới.
- Không thay đổi logic planner hoặc benchmark ngoài phạm vi cần thiết.
- Giữ coding style hiện có.
- Sau khi hoàn thành, liệt kê rõ:
  - file nào đã sửa
  - sửa gì
  - cách verify nhanh

---

## Câu lệnh yêu cầu AI thực thi (copy dùng ngay)

Đọc và thực hiện toàn bộ nội dung trong file `docs/prompts/phase_07_office_v2_obj_mtl_migration.md`.  
Mục tiêu là thay môi trường cũ sang môi trường mới dùng `src/anhc_simulation/models/office_v2.obj` và `src/anhc_simulation/models/office_v2.mtl`, đảm bảo launch nav chạy ổn định và verify được bằng lệnh ROS 2.
