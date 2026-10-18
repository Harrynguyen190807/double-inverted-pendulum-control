# Double Inverted Pendulum on a Cart (DIP) - Mathematical Modeling & Derivation

This document provides a clear, step-by-step engineering derivation of the equations of motion (EOM) and the optimal Linear Quadratic Regulator (LQR) for the Double Inverted Pendulum on a Cart (DIP).

---

## 1. System Coordinates & Physical Parameters

The system consists of a motorized cart moving horizontally along an axis `x(t)`, with two pendulums (links) attached in series via low-friction rotational joints.

```text
       (O) Link 2 Center of Mass (m2, J2)
        |
        \   theta2 (Angle from vertical)
         \
          * Joint 2 (Pivot between Link 1 and Link 2)
          |
          \   theta1 (Angle from vertical)
           \
       [======] Cart (Mass M)
       --[oo]--  ---> Actuator Force u(t), Friction b
=================================================== Horizontal Guide Rail (x)
```

### Coordinate Definitions
- **Cart position `x`**: Linear displacement along the horizontal rail (meters, positive to the right).
- **Cart velocity `dx`**: Linear velocity along the rail (m/s).
- **Link 1 angle `theta1`**: Angle of the lower pole measured from the upright vertical axis (radians, positive counter-clockwise).
- **Link 2 angle `theta2`**: Angle of the upper pole measured from the upright vertical axis (radians, positive counter-clockwise).
- **Control input `u`**: Net horizontal force exerted on the cart by the linear motor (Newtons).

### Physical Benchmark Parameters Table

| Parameter | Symbol | Nominal Value | SI Unit | Description |
| :--- | :--- | :--- | :--- | :--- |
| Cart Mass | M | 1.20 | kg | Mass of the motorized cart |
| Link 1 Mass | m1 | 0.15 | kg | Mass of the lower pendulum rod |
| Link 2 Mass | m2 | 0.10 | kg | Mass of the upper pendulum rod |
| Link 1 Length | L1 | 0.30 | m | Total physical length of lower rod |
| Link 2 Length | L2 | 0.25 | m | Total physical length of upper rod |
| Link 1 COM Distance | l1 | 0.15 | m | Pivot to center-of-mass distance (L1 / 2) |
| Link 2 COM Distance | l2 | 0.125 | m | Pivot to center-of-mass distance (L2 / 2) |
| Link 1 Inertia | J1 | 0.001125 | kg*m^2 | Moment of inertia about center of mass |
| Link 2 Inertia | J2 | 0.000521 | kg*m^2 | Moment of inertia about center of mass |
| Gravity | g | 9.81 | m/s^2 | Acceleration due to gravity |
| Cart Friction | b | 0.50 | N*s/m | Viscous damping along guide rail |
| Joint 1 Damping | d1 | 0.005 | N*m*s/rad | Viscous rotational friction at Joint 1 |
| Joint 2 Damping | d2 | 0.003 | N*m*s/rad | Viscous rotational friction at Joint 2 |

---

## 2. Kinematics (Positions and Velocities)

Let the coordinate origin `(0, 0)` be located on the horizontal guide rail.

### 2.1 Center of Mass Positions
- **Cart Center of Mass:**
  - `x_c = x`
  - `y_c = 0`

- **Link 1 Center of Mass:**
  - `x1 = x - l1 * sin(theta1)`
  - `y1 = l1 * cos(theta1)`

- **Joint 2 Pivot Location:**
  - `x_j2 = x - L1 * sin(theta1)`
  - `y_j2 = L1 * cos(theta1)`

- **Link 2 Center of Mass:**
  - `x2 = x - L1 * sin(theta1) - l2 * sin(theta2)`
  - `y2 = L1 * cos(theta1) + l2 * cos(theta2)`

### 2.2 Velocities (Time Derivatives)
Taking the time derivative of each position:

```text
dx_c = dx
dy_c = 0

dx1  = dx - l1 * cos(theta1) * dtheta1
dy1  = -l1 * sin(theta1) * dtheta1

dx2  = dx - L1 * cos(theta1) * dtheta1 - l2 * cos(theta2) * dtheta2
dy2  = -L1 * sin(theta1) * dtheta1 - l2 * sin(theta2) * dtheta2
```

---

## 3. Lagrangian Energy Formulation

### 3.1 Total Kinetic Energy (T)
The total kinetic energy is the sum of cart translational energy plus each pendulum's translational and rotational energy:

$$
T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m_1 (\dot{x}_1^2 + \dot{y}_1^2) + \frac{1}{2} J_1 \dot{\theta}_1^2 + \frac{1}{2} m_2 (\dot{x}_2^2 + \dot{y}_2^2) + \frac{1}{2} J_2 \dot{\theta}_2^2
$$

### 3.2 Total Potential Energy (V)
With datum `y = 0` at the rail:

$$
V = m_1 g y_1 + m_2 g y_2 = (m_1 l_1 + m_2 L_1) g \cos(\theta_1) + m_2 g l_2 \cos(\theta_2)
$$

### 3.3 The Lagrangian (L)
The Lagrangian function is defined as kinetic energy minus potential energy:

$$
\mathcal{L} = T - V
$$

---

## 4. Equations of Motion (Euler-Lagrange)

The generalized coordinate vector is:

$$
q = \begin{bmatrix} x \\ \theta_1 \\ \theta_2 \end{bmatrix}
$$

The non-conservative generalized forces acting on each coordinate are:
- Cart horizontal force: `Q_x = u - b * dx`
- Joint 1 torque: `Q_th1 = -d1 * dtheta1 + d2 * (dtheta2 - dtheta1)`
- Joint 2 torque: `Q_th2 = -d2 * (dtheta2 - dtheta1)`

Applying the Euler-Lagrange formula for each coordinate `i`:

$$
\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i
$$

This produces the standard matrix form:

$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + G(q) + D \dot{q} = B_u u
$$

where:
- `M(q)` is the 3x3 symmetric mass/inertia matrix.
- `C(q, dq)` contains centrifugal and Coriolis interaction forces.
- `G(q)` is the gravity vector.
- `D` is the damping matrix representing track and pivot friction.
- `B_u = [1, 0, 0]^T` maps the horizontal actuator force to cart acceleration.

---

## 5. Linearization Around the Upright Equilibrium

At the upright equilibrium:
- `x = 0, theta1 = 0, theta2 = 0`
- `dx = 0, dtheta1 = 0, dtheta2 = 0`
- `u = 0`

For small angular deviations:
- `sin(theta) ≈ theta`
- `cos(theta) ≈ 1`
- Second-order velocity terms are negligible: `dtheta^2 ≈ 0`

The linearized second-order differential equation becomes:

$$
M_0 \ddot{q} + D_0 \dot{q} + K_0 q = B_0 u
$$

### Linearized Mass Matrix (M0)

$$
M_0 = \begin{bmatrix}
M + m_1 + m_2 & -(m_1 l_1 + m_2 L_1) & -m_2 l_2 \\
-(m_1 l_1 + m_2 L_1) & J_1 + m_1 l_1^2 + m_2 L_1^2 & m_2 L_1 l_2 \\
-m_2 l_2 & m_2 L_1 l_2 & J_2 + m_2 l_2^2
\end{bmatrix}
$$

### Linearized Damping Matrix (D0)

$$
D_0 = \begin{bmatrix}
b & 0 & 0 \\
0 & d_1 + d_2 & -d_2 \\
0 & -d_2 & d_2
\end{bmatrix}
$$

### Linearized Gravity Stiffness Matrix (K0)

$$
K_0 = \begin{bmatrix}
0 & 0 & 0 \\
0 & -(m_1 l_1 + m_2 L_1) g & 0 \\
0 & 0 & -m_2 l_2 g
\end{bmatrix}
$$

### Linearized Input Vector (B0)

$$
B_0 = \begin{bmatrix} 1 \\ 0 \\ 0 \end{bmatrix}
$$

Solving for accelerations:

$$
\ddot{q} = -M_0^{-1} K_0 q - M_0^{-1} D_0 \dot{q} + M_0^{-1} B_0 u
$$

---

## 6. Continuous State-Space Representation

We define the 6-dimensional state vector:

$$
z = \begin{bmatrix} x \\ \dot{x} \\ \theta_1 \\ \dot{\theta}_1 \\ \theta_2 \\ \dot{\theta}_2 \end{bmatrix}
$$

The first-order continuous state-space equation is:

$$
\dot{z}(t) = A z(t) + B u(t)
$$

where `A` is a 6x6 system matrix and `B` is a 6x1 input matrix:

```text
[ dx/dt     ]   [ 0  1  0  0  0  0 ] [ x        ]   [ 0   ]
[ ddx/dt    ]   [                  ] [ dx       ]   [ B1  ]
[ dth1/dt   ] = [ 0  0  0  1  0  0 ] [ th1      ] + [ 0   ] * u
[ ddth1/dt  ]   [    A_ddq_matrix  ] [ dth1     ]   [ B3  ]
[ dth2/dt   ]   [ 0  0  0  0  0  1 ] [ th2      ]   [ 0   ]
[ ddth2/dt  ]   [                  ] [ dth2     ]   [ B5  ]
```

---

## 7. Optimal LQR Control Design

### 7.1 Controllability Verification
The Kalman controllability matrix is formed by:

$$
\mathcal{C} = \begin{bmatrix} B & AB & A^2 B & A^3 B & A^4 B & A^5 B \end{bmatrix}
$$

The system is fully controllable if:
- `rank(C) == 6` (All 6 state modes can be steered by the single actuator force `u`).

### 7.2 Performance Cost Function (Bryson's Rule)
We seek an optimal control force `u(t)` that minimizes the quadratic performance index:

$$
J = \int_{0}^{\infty} \left( z(t)^T Q z(t) + R u(t)^2 \right) dt
$$

Bryson's Rule sets the diagonal weights inversely proportional to the square of maximum acceptable errors:

- `Q_ii = 1 / (max_state_error_i)^2`
- `R    = 1 / (max_force_u)^2`

Tuning constraints for this system:
- Maximum cart error: `0.05 m` -> `Q[0,0] = 400.0`
- Maximum cart rate: `0.40 m/s` -> `Q[1,1] = 6.25`
- Maximum Link 1 angle error: `0.035 rad (~2.0°)` -> `Q[2,2] = 816.3`
- Maximum Link 1 rate: `0.40 rad/s` -> `Q[3,3] = 6.25`
- Maximum Link 2 angle error: `0.035 rad (~2.0°)` -> `Q[4,4] = 816.3`
- Maximum Link 2 rate: `0.40 rad/s` -> `Q[5,5] = 6.25`
- Maximum control force: `25.0 N` -> `R = 0.0016`

### 7.3 Continuous Algebraic Riccati Equation (CARE)
The unique positive-definite matrix `P = P^T > 0` satisfies:

$$
A^T P + P A - P B R^{-1} B^T P + Q = 0
$$

The optimal state feedback gain vector `K` is:

$$
K = R^{-1} B^T P = \begin{bmatrix} k_x & k_{\dot{x}} & k_{\theta_1} & k_{\dot{\theta}_1} & k_{\theta_2} & k_{\dot{\theta}_2} \end{bmatrix}
$$

### 7.4 Resulting Closed-Loop Control Law
The real-time control effort applied to the motor driver is:

$$
u(t) = -K z(t) = -(k_x x + k_{\dot{x}} \dot{x} + k_{\theta_1} \theta_1 + k_{\dot{\theta}_1} \dot{\theta}_1 + k_{\theta_2} \theta_2 + k_{\dot{\theta}_2} \dot{\theta}_2)
$$

All closed-loop poles `eig(A - B*K)` have negative real parts, proving asymptotic stability.
