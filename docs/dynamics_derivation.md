# Double Inverted Pendulum on a Cart (DIP) - Mathematical Modeling & Derivation

## 1. Hệ tọa độ và Thông số vật lý

Hệ thống bao gồm một xe trượt khối lượng $M$ di chuyển trên ray ngang theo phương $x(t)$, và hai thanh lắc liên kết bản lề nối tiếp nhau:
- **Thanh lắc 1 (Link 1):** Khối lượng $m_1$, chiều dài toàn phần $L_1$, khoảng cách từ khớp 1 đến khối tâm $l_1 = L_1/2$, mô-men quán tính quanh khối tâm $J_1 = \frac{1}{12}m_1 L_1^2$, góc lệch $\theta_1(t)$ so với phương thẳng đứng hướng lên (chiều dương ngược chiều kim đồng hồ).
- **Thanh lắc 2 (Link 2):** Khối lượng $m_2$, chiều dài toàn phần $L_2$, khoảng cách từ khớp 2 đến khối tâm $l_2 = L_2/2$, mô-men quán tính quanh khối tâm $J_2 = \frac{1}{12}m_2 L_2^2$, góc lệch $\theta_2(t)$ so với phương thẳng đứng hướng lên.

```
       (O) Khối tâm Link 2 (m2, J2)
        |
        \  theta2
         \
          * Khớp 2 (Joint 2, d2)
          |
          \  theta1
           \
       [======] Xe trượt (Cart M)
       --[oo]--  --> Lực điều khiển u(t), ma sát b
=================================================== Ray dẫn hướng (x)
```

### Bảng thông số vật lý danh định (Benchmark Parameters)

| Ký hiệu | Thông số | Giá trị | Đơn vị |
| :--- | :--- | :--- | :--- |
| $M$ | Khối lượng xe trượt | 1.20 | kg |
| $m_1$ | Khối lượng thanh lắc 1 | 0.15 | kg |
| $m_2$ | Khối lượng thanh lắc 2 | 0.10 | kg |
| $L_1$ | Chiều dài thanh lắc 1 | 0.30 | m |
| $L_2$ | Chiều dài thanh lắc 2 | 0.25 | m |
| $l_1$ | Khoảng cách đến khối tâm 1 | 0.15 | m |
| $l_2$ | Khoảng cách đến khối tâm 2 | 0.125 | m |
| $J_1$ | Mô-men quán tính Link 1 | $1.125 \times 10^{-3}$ | $\text{kg}\cdot\text{m}^2$ |
| $J_2$ | Mô-men quán tính Link 2 | $5.208 \times 10^{-4}$ | $\text{kg}\cdot\text{m}^2$ |
| $g$ | Gia tốc trọng trường | 9.81 | $\text{m/s}^2$ |
| $b$ | Hệ số ma sát nhớt ray trượt | 0.50 | $\text{N}\cdot\text{s/m}$ |
| $d_1$ | Hệ số cản khớp quay 1 | 0.005 | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ |
| $d_2$ | Hệ số cản khớp quay 2 | 0.003 | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ |

---

## 2. Thiết lập Phương trình Động học & Động lực học Lagrange

Tọa độ vị trí của các khối tâm:
- Xe trượt:
  $$x_c = x, \quad y_c = 0$$
- Khối tâm Link 1:
  $$x_1 = x - l_1 \sin\theta_1, \quad y_1 = l_1 \cos\theta_1$$
- Vị trí khớp 2:
  $$x_{j2} = x - L_1 \sin\theta_1, \quad y_{j2} = L_1 \cos\theta_1$$
- Khối tâm Link 2:
  $$x_2 = x - L_1 \sin\theta_1 - l_2 \sin\theta_2, \quad y_2 = L_1 \cos\theta_1 + l_2 \cos\theta_2$$

Động năng toàn phần của hệ thống ($T = T_{\text{cart}} + T_1 + T_2$):
$$T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m_1 (\dot{x}_1^2 + \dot{y}_1^2) + \frac{1}{2} J_1 \dot{\theta}_1^2 + \frac{1}{2} m_2 (\dot{x}_2^2 + \dot{y}_2^2) + \frac{1}{2} J_2 \dot{\theta}_2^2$$

Thế năng toàn phần ($V$ lấy mốc chuẩn tại ray $y = 0$):
$$V = m_1 g y_1 + m_2 g y_2 = (m_1 l_1 + m_2 L_1) g \cos\theta_1 + m_2 g l_2 \cos\theta_2$$

Hàm Lagrangian:
$$\mathcal{L} = T - V$$

Vector tọa độ suy rộng: $q = [x, \theta_1, \theta_2]^T$.

Lực suy rộng phi thế $Q = [Q_x, Q_{\theta_1}, Q_{\theta_2}]^T$:
$$Q_x = u - b \dot{x}$$
$$Q_{\theta_1} = - d_1 \dot{\theta}_1 + d_2 (\dot{\theta}_2 - \dot{\theta}_1)$$
$$Q_{\theta_2} = - d_2 (\dot{\theta}_2 - \dot{\theta}_1)$$

Phương trình Euler-Lagrange:
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i, \quad i \in \{1, 2, 3\}$$

Phương trình chuyển động phi tuyến có dạng ma trận chuẩn:
$$\mathbf{M}(q)\ddot{q} + \mathbf{C}(q, \dot{q})\dot{q} + \mathbf{G}(q) + \mathbf{D}\dot{q} = \mathbf{B}_u u$$

---

## 3. Tuyến tính hóa quanh Điểm Cân bằng Thẳng đứng

Điểm cân bằng thẳng đứng: $x_0 = 0, \theta_{1,0} = 0, \theta_{2,0} = 0, \dot{x}_0 = 0, \dot{\theta}_{1,0} = 0, \dot{\theta}_{2,0} = 0, u_0 = 0$.

Với góc lệch nhỏ ($\sin\theta \approx \theta$, $\cos\theta \approx 1$, bỏ qua các tích bậc cao $\dot{\theta}^2 \approx 0$):
$$\mathbf{M}_0 \ddot{q} + \mathbf{D}_0 \dot{q} + \mathbf{K}_0 q = \mathbf{B}_0 u$$

Trong đó:
$$\mathbf{M}_0 = \begin{bmatrix}
M + m_1 + m_2 & -(m_1 l_1 + m_2 L_1) & -m_2 l_2 \\
-(m_1 l_1 + m_2 L_1) & J_1 + m_1 l_1^2 + m_2 L_1^2 & m_2 L_1 l_2 \\
-m_2 l_2 & m_2 L_1 l_2 & J_2 + m_2 l_2^2
\end{bmatrix}$$

$$\mathbf{D}_0 = \begin{bmatrix}
b & 0 & 0 \\
0 & d_1 + d_2 & -d_2 \\
0 & -d_2 & d_2
\end{bmatrix}, \quad
\mathbf{K}_0 = \begin{bmatrix}
0 & 0 & 0 \\
0 & -(m_1 l_1 + m_2 L_1) g & 0 \\
0 & 0 & -m_2 l_2 g
\end{bmatrix}, \quad
\mathbf{B}_0 = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}$$

Gia tốc suy rộng:
$$\ddot{q} = -\mathbf{M}_0^{-1} \mathbf{K}_0 q - \mathbf{M}_0^{-1} \mathbf{D}_0 \dot{q} + \mathbf{M}_0^{-1} \mathbf{B}_0 u$$

Vector trạng thái không gian:
$$z = [x, \dot{x}, \theta_1, \dot{\theta}_1, \theta_2, \dot{\theta}_2]^T$$

Phương trình không gian trạng thái tuyến tính liên tục:
$$\dot{z}(t) = \mathbf{A} z(t) + \mathbf{B} u(t)$$

---

## 4. Thiết kế Bộ điều khiển Tối ưu LQR & Quy tắc Bryson

### Tính điều khiển được (Controllability)
Ma trận điều khiển được Kalman:
$$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{A}\mathbf{B} & \mathbf{A}^2\mathbf{B} & \mathbf{A}^3\mathbf{B} & \mathbf{A}^4\mathbf{B} & \mathbf{A}^5\mathbf{B} \end{bmatrix}$$
Hệ thống điều khiển được khi và chỉ khi:
$$\text{rank}(\mathcal{C}) = 6$$

### Hàm mục tiêu tối ưu (Performance Index)
$$J = \int_{0}^{\infty} \left( z(t)^T \mathbf{Q} z(t) + u(t)^T \mathbf{R} u(t) \right) dt$$

Áp dụng quy tắc Bryson (Bryson's Rule) để xác định trọng số:
$$Q_{ii} = \frac{1}{(\Delta z_{i,\max})^2}, \quad R = \frac{1}{(u_{\max})^2}$$

Với:
- $\Delta x_{\max} = 0.05\text{ m}$
- $\Delta \dot{x}_{\max} = 0.40\text{ m/s}$
- $\Delta \theta_{1,\max} = 0.035\text{ rad } (\approx 2^\circ)$
- $\Delta \dot{\theta}_{1,\max} = 0.40\text{ rad/s}$
- $\Delta \theta_{2,\max} = 0.035\text{ rad } (\approx 2^\circ)$
- $\Delta \dot{\theta}_{2,\max} = 0.40\text{ rad/s}$
- $u_{\max} = 20.0\text{ N}$

Giải phương trình đại số Riccati liên tục (CARE):
$$\mathbf{A}^T \mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T \mathbf{P} + \mathbf{Q} = 0$$

Ma trận độ lợi hồi tiếp tối ưu:
$$\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}^T \mathbf{P} = \begin{bmatrix} k_x & k_{\dot{x}} & k_{\theta_1} & k_{\dot{\theta}_1} & k_{\theta_2} & k_{\dot{\theta}_2} \end{bmatrix}$$

Luật điều khiển:
$$u(t) = -\mathbf{K} z(t)$$
