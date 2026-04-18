# Bộ Metrics dùng đánh giá 

* Path Length (L) đơn vị meters : Tổng quãng đường robot đi từ điểm xuất phát đến đích
* Computation Time (Tc) đơn vị ms:  Thời gian thuật toán chạy để tìm ra đường đi.
* Path Smoothness (S) đơn vị rad/m : Độ êm ái / ít gấp khúc của đường đi
* Success Rate (SR) đơn vị % : Tỷ lệ lần chạy thuật toán thành công.
* Safety Clearance (C) đơn vị meters : Khoảng cách an toàn từ robot đến chướng ngại vật

**Gợi ý mở rộng (tùy benchmark):** thời gian đến đích sau khi có path (execution), cross-track error cực đại — chỉ thêm khi thống nhất định nghĩa trong CSV/báo cáo.

Kế hoạch triển khai đo — scenario — runner — CSV — radar chart: xem [`BENCHMARK_PLAN.md`](BENCHMARK_PLAN.md).