---
name: dip-dynamics-modeling
description: >-
  Mô hình hóa Động lực học & Tính toán LQR cho hệ Con lắc ngược đôi trên xe trượt (DIP).
  Sử dụng SymPy suy dẫn phương trình Euler-Lagrange, tuyến tính hóa quanh điểm thẳng đứng,
  kiểm tra tính điều khiển được (Controllability Rank), giải Riccati (Bryson's Rule) và xuất ma trận K.
---

# Skill: `dip-dynamics-modeling`

## Tổng quan
Skill này chịu trách nhiệm tự động hóa toàn bộ chu trình thiết lập mô hình toán học và tổng hợp bộ điều khiển tối ưu LQR cho hệ Double Inverted Pendulum on a Cart (DIP).

## Khi nào sử dụng
- Khi cần kiểm tra lại hoặc suy dẫn lại phương trình động lực học Euler-Lagrange phi tuyến bằng SymPy.
- Khi người dùng thay đổi kích thước, khối lượng cơ khí ($M, m_1, m_2, L_1, L_2$) hoặc ma sát ($b, d_1, d_2$).
- Khi cần tuyến tính hóa hệ quanh điểm cân bằng thẳng đứng và xuất ma trận trạng thái $A$ (6x6), $B$ (6x1).
- Khi kiểm tra tính điều khiển được $\text{rank}(\mathcal{C}) == 6$ của hệ thống.
- Khi tinh chỉnh trọng số Bryson $Q, R$ để tìm bộ độ lợi phản hồi trạng thái tối ưu $K = [k_x, k_{\dot{x}}, k_{\theta_1}, k_{\dot{\theta}_1}, k_{\theta_2}, k_{\dot{\theta}_2}]$.
- Khi tự động sinh mã định nghĩa C-header (`dip_config.h`) để nạp trực tiếp vào firmware nhúng.

## Các lệnh & Công cụ thực thi

### 1. Chạy suy dẫn ký hiệu SymPy
```bash
python simulation/sympy_derivation.py
```
- Tự động thiết lập hàm Lagrange $\mathcal{L} = T - V$.
- Đạo hàm Euler-Lagrange với lực suy rộng không bảo toàn $Q$.
- Tính ma trận quán tính $\mathbf{M}(q)$, ma trận Coriolis $\mathbf{C}(q, \dot{q})$, vector trọng lực $\mathbf{G}(q)$.
- Xuất ma trận tuyến tính $A$ và $B$ dưới dạng symbolic và numerical.

### 2. Thiết kế & Tinh chỉnh bộ điều khiển LQR
```bash
python simulation/lqr_design.py
```
- Kiểm tra tính điều khiển được (Kalman Controllability Matrix Rank).
- Thiết lập trọng số theo quy tắc Bryson:
  $$Q_{ii} = \frac{1}{(\Delta z_{i,\max})^2}, \quad R = \frac{1}{(u_{\max})^2}$$
- Giải phương trình đại số Riccati liên tục (CARE):
  $$\mathbf{A}^T \mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T \mathbf{P} + \mathbf{Q} = 0$$
- Xuất ma trận độ lợi $K = \mathbf{R}^{-1}\mathbf{B}^T \mathbf{P}$ và các cực vòng kín $\lambda(\mathbf{A} - \mathbf{B}\mathbf{K})$.
- Xuất đoạn mã C header snippet tương thích với `firmware/include/dip_config.h`.

## Tinh chỉnh Tham số Nhanh
Để thay đổi thông số vật lý của hệ thống, chỉnh sửa trực tiếp hàm `get_default_parameters()` trong `simulation/sympy_derivation.py` hoặc class `DIPDynamics` trong `simulation/lqr_design.py`.
