---
name: dip-physics-simulation
description: >-
  Mô phỏng Động lực học Phi tuyến & Trực quan hóa 3D cho Con lắc ngược đôi (DIP).
  Chạy giải ODE phi tuyến với bão hòa lực (Saturation Clamp) và nhiễu xung kích (Impulse Disturbance),
  xuất file cấu hình vật lý MuJoCo MJCF (model.xml) và vẽ đồ thị đáp ứng thời gian (visualize.py).
---

# Skill: `dip-physics-simulation`

## Tổng quan
Skill này phụ trách việc kiểm chứng động học và thuật toán điều khiển trong môi trường mô phỏng số phi tuyến thực tế và mô phỏng vật lý 3D chuẩn công nghiệp (MuJoCo).

## Khi nào sử dụng
- Khi cần kiểm tra khả năng ổn định vòng kín của bộ điều khiển LQR với phương trình vi phân phi tuyến đầy đủ (Nonlinear ODE), không chỉ là mô hình tuyến tính xấp xỉ.
- Khi cần đánh giá sức bền của hệ thống dưới tác động của bão hòa lực chấp hành (Actuator Force Saturation Clamp, ví dụ $\pm 25\text{ N}$) và tải dòng tối đa của động cơ.
- Khi kiểm thử tính bền vững (Robustness) trước nhiễu xung kích bên ngoài (Impulse Disturbance, ví dụ một cú đẩy $15\text{ N}$ trong $50\text{ ms}$).
- Khi chạy mô phỏng vật lý 3D tương tác với MuJoCo thông qua file `simulation/model.xml`.
- Khi cần xuất đồ thị đáp ứng quá độ thời gian của các trạng thái $(x, \theta_1, \theta_2, u)$ hoặc animation.

## Các lệnh & Công cụ thực thi

### 1. Chạy mô phỏng số phi tuyến RK4 / Solve IVP
```bash
python simulation/nonlinear_sim.py
```
- Tích phân phương trình chuyển động phi tuyến thực nghiệm với bước tích phân $\Delta t = 1\text{ ms}$.
- Tích hợp bộ điều khiển LQR và bộ giới hạn lực bão hòa.
- Áp đặt nhiễu xung kích tại $t = 1.5\text{ s}$.
- Xuất file kết quả quỹ đạo: `simulation/output_trajectory.csv`.

### 2. Trực quan hóa Đồ thị & Đánh giá Chỉ số Hiệu năng
```bash
python simulation/visualize.py
```
- Đọc file `simulation/output_trajectory.csv` và tính toán các chỉ số kỹ thuật:
  - Độ lệch vị trí cực đại của xe trượt ($x_{\max}$).
  - Độ lệch góc cực đại của thanh lắc 1 và 2 ($\theta_{1,\max}, \theta_{2,\max}$).
  - Lực chấp hành cực đại ($u_{\max}$) và thời gian quá độ xác lập ($t_s$).
- Vẽ và lưu đồ thị chất lượng cao 4 kênh ra file `simulation/dip_response.png`.

### 3. Mô phỏng 3D tương tác với MuJoCo
```bash
python -m mujoco.viewer --mjcf=simulation/model.xml
```
- Khởi chạy cửa sổ tương tác 3D MuJoCo với mô hình vật lý chính xác:
  - Ray trượt dẫn hướng (slider joint) giới hạn hành trình $[-1.2\text{ m}, +1.2\text{ m}]$.
  - Khớp bản lề 1 & 2 với ma sát nhớt và ma sát khô thực.
  - Cảm biến góc, vận tốc góc, vị trí xe và động cơ truyền động tuyến tính.
