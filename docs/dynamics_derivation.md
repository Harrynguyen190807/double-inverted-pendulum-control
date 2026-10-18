# Double Inverted Pendulum on a Cart (DIP) - Mathematical Modeling & Derivation

## 1. Coordinate System & Physical Parameters

The system comprises a cart of mass $M$ moving horizontally along track coordinate $x(t)$, and two hinged pendulums connected in series:
- **Link 1 (Lower Pendulum):** Mass $m_1$, total length $L_1$, distance to center of mass $l_1 = L_1/2$, moment of inertia about COM $J_1 = \frac{1}{12}m_1 L_1^2$, angular deflection $\theta_1(t)$ from the upright vertical axis (positive counter-clockwise).
- **Link 2 (Upper Pendulum):** Mass $m_2$, total length $L_2$, distance to center of mass $l_2 = L_2/2$, moment of inertia about COM $J_2 = \frac{1}{12}m_2 L_2^2$, angular deflection $\theta_2(t)$ from the upright vertical axis.

```
       (O) Center of Mass Link 2 (m2, J2)
        |
        \  theta2
         \
          * Joint 2 (d2)
          |
          \  theta1
           \
       [======] Cart (M)
       --[oo]--  --> Control Force u(t), Friction b
=================================================== Guide Rail (x)
```

### Nominal Physical Parameters (Benchmark System)

| Symbol | Description | Value | SI Unit |
| :--- | :--- | :--- | :--- |
| $M$ | Cart Mass | 1.20 | $\text{kg}$ |
| $m_1$ | Pendulum 1 Mass | 0.15 | $\text{kg}$ |
| $m_2$ | Pendulum 2 Mass | 0.10 | $\text{kg}$ |
| $L_1$ | Pendulum 1 Total Length | 0.30 | $\text{m}$ |
| $L_2$ | Pendulum 2 Total Length | 0.25 | $\text{m}$ |
| $l_1$ | Center of Mass Distance 1 | 0.15 | $\text{m}$ |
| $l_2$ | Center of Mass Distance 2 | 0.125 | $\text{m}$ |
| $J_1$ | Moment of Inertia 1 | $1.125 \times 10^{-3}$ | $\text{kg}\cdot\text{m}^2$ |
| $J_2$ | Moment of Inertia 2 | $5.208 \times 10^{-4}$ | $\text{kg}\cdot\text{m}^2$ |
| $g$ | Gravitational Acceleration | 9.81 | $\text{m/s}^2$ |
| $b$ | Cart Track Viscous Damping | 0.50 | $\text{N}\cdot\text{s/m}$ |
| $d_1$ | Joint 1 Damping | 0.005 | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ |
| $d_2$ | Joint 2 Damping | 0.003 | $\text{N}\cdot\text{m}\cdot\text{s/rad}$ |

---

## 2. Kinematics & Euler-Lagrange Dynamics

Position vectors of the centers of mass:
- Cart:
  $$x_c = x, \quad y_c = 0$$
- Center of Mass Link 1:
  $$x_1 = x - l_1 \sin\theta_1, \quad y_1 = l_1 \cos\theta_1$$
- Joint 2 Position:
  $$x_{j2} = x - L_1 \sin\theta_1, \quad y_{j2} = L_1 \cos\theta_1$$
- Center of Mass Link 2:
  $$x_2 = x - L_1 \sin\theta_1 - l_2 \sin\theta_2, \quad y_2 = L_1 \cos\theta_1 + l_2 \cos\theta_2$$

Total kinetic energy ($T = T_{\text{cart}} + T_1 + T_2$):
$$T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m_1 (\dot{x}_1^2 + \dot{y}_1^2) + \frac{1}{2} J_1 \dot{\theta}_1^2 + \frac{1}{2} m_2 (\dot{x}_2^2 + \dot{y}_2^2) + \frac{1}{2} J_2 \dot{\theta}_2^2$$

Total potential energy ($V$ referenced to datum $y = 0$):
$$V = m_1 g y_1 + m_2 g y_2 = (m_1 l_1 + m_2 L_1) g \cos\theta_1 + m_2 g l_2 \cos\theta_2$$

Lagrangian function:
$$\mathcal{L} = T - V$$

Generalized coordinate vector: $\mathbf{q} = [x, \theta_1, \theta_2]^T$.

Non-conservative generalized forces $\mathbf{Q} = [Q_x, Q_{\theta_1}, Q_{\theta_2}]^T$:
$$Q_x = u - b \dot{x}$$
$$Q_{\theta_1} = - d_1 \dot{\theta}_1 + d_2 (\dot{\theta}_2 - \dot{\theta}_1)$$
$$Q_{\theta_2} = - d_2 (\dot{\theta}_2 - \dot{\theta}_1)$$

Euler-Lagrange equations:
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = Q_i, \quad i \in \{1, 2, 3\}$$

Yields the matrix equation of motion:
$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{G}(\mathbf{q}) + \mathbf{D}\dot{\mathbf{q}} = \mathbf{B}_u u$$

---

## 3. Linearization Around the Upright Equilibrium

The vertical equilibrium point is $[\mathbf{q}_0, \dot{\mathbf{q}}_0] = [\mathbf{0}, \mathbf{0}], u_0 = 0$.

Applying small-angle approximations ($\sin\theta \approx \theta$, $\cos\theta \approx 1$, neglecting second-order rate products $\dot{\theta}^2 \approx 0$):
$$\mathbf{M}_0 \ddot{\mathbf{q}} + \mathbf{D}_0 \dot{\mathbf{q}} + \mathbf{K}_0 \mathbf{q} = \mathbf{B}_0 u$$

where:
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

Solving for generalized accelerations:
$$\ddot{\mathbf{q}} = -\mathbf{M}_0^{-1} \mathbf{K}_0 \mathbf{q} - \mathbf{M}_0^{-1} \mathbf{D}_0 \dot{\mathbf{q}} + \mathbf{M}_0^{-1} \mathbf{B}_0 u$$

Defining the state vector:
$$\mathbf{z} = [x, \dot{x}, \theta_1, \dot{\theta}_1, \theta_2, \dot{\theta}_2]^T$$

Continuous state-space representation:
$$\dot{\mathbf{z}}(t) = \mathbf{A} \mathbf{z}(t) + \mathbf{B} u(t)$$

---

## 4. LQR Controller Synthesis & Bryson's Rule

### Controllability
The Kalman controllability matrix:
$$\mathcal{C} = \begin{bmatrix} \mathbf{B} & \mathbf{A}\mathbf{B} & \mathbf{A}^2\mathbf{B} & \mathbf{A}^3\mathbf{B} & \mathbf{A}^4\mathbf{B} & \mathbf{A}^5\mathbf{B} \end{bmatrix}$$
The system is fully state-controllable if and only if:
$$\text{rank}(\mathcal{C}) = 6$$

### Performance Index & Weight Selection
$$J = \int_{0}^{\infty} \left( \mathbf{z}(t)^T \mathbf{Q} \mathbf{z}(t) + u(t)^T \mathbf{R} u(t) \right) dt$$

Using Bryson's rule to balance maximum permissible deviations:
$$Q_{ii} = \frac{1}{(\Delta z_{i,\max})^2}, \quad R = \frac{1}{(u_{\max})^2}$$

Nominal constraints:
- $\Delta x_{\max} = 0.05\text{ m}$
- $\Delta \dot{x}_{\max} = 0.40\text{ m/s}$
- $\Delta \theta_{1,\max} = 0.035\text{ rad } (\approx 2^\circ)$
- $\Delta \dot{\theta}_{1,\max} = 0.40\text{ rad/s}$
- $\Delta \theta_{2,\max} = 0.035\text{ rad } (\approx 2^\circ)$
- $\Delta \dot{\theta}_{2,\max} = 0.40\text{ rad/s}$
- $u_{\max} = 20.0\text{ N}$

Continuous Algebraic Riccati Equation (CARE):
$$\mathbf{A}^T \mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T \mathbf{P} + \mathbf{Q} = 0$$

Optimal feedback gain:
$$\mathbf{K} = \mathbf{R}^{-1}\mathbf{B}^T \mathbf{P} = \begin{bmatrix} k_x & k_{\dot{x}} & k_{\theta_1} & k_{\dot{\theta}_1} & k_{\theta_2} & k_{\dot{\theta}_2} \end{bmatrix}$$

Control law:
$$u(t) = -\mathbf{K} \mathbf{z}(t)$$
