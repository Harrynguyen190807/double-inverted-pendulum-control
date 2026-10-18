---
name: dip-dynamics-modeling
description: >-
  Dynamics Modeling & LQR Synthesis for the Double Inverted Pendulum on a Cart (DIP).
  Derives symbolic Euler-Lagrange equations via SymPy, linearizes around upright equilibrium,
  verifies controllability matrix rank, solves continuous ARE (Bryson's Rule), and exports gain matrix K.
---

# Skill: `dip-dynamics-modeling`

## Overview
This skill automates the complete mathematical modeling and optimal controller synthesis pipeline for the Double Inverted Pendulum on a Cart (DIP) system.

## When to Use
- When deriving or re-verifying nonlinear Euler-Lagrange equations of motion using SymPy.
- When updating mechanical or physical parameters ($M, m_1, m_2, L_1, L_2$) or damping coefficients ($b, d_1, d_2$).
- When linearizing the dynamics around the upright vertical equilibrium to extract state-space matrices $\mathbf{A} \in \mathbb{R}^{6 \times 6}$ and $\mathbf{B} \in \mathbb{R}^{6 \times 1}$.
- When verifying the Kalman Controllability Matrix rank ($\text{rank}(\mathcal{C}) == 6$).
- When tuning Bryson's rule weights $\mathbf{Q}, \mathbf{R}$ to synthesize the optimal state-feedback gain vector $\mathbf{K} = [k_x, k_{\dot{x}}, k_{\theta_1}, k_{\dot{\theta}_1}, k_{\theta_2}, k_{\dot{\theta}_2}]$.
- When generating C header code snippets (`dip_config.h`) for embedded firmware.

## Commands & Workflows

### 1. Symbolic Derivation via SymPy
```bash
python simulation/sympy_derivation.py
```
- Formulates the Lagrangian $\mathcal{L} = T - V$.
- Evaluates Euler-Lagrange equations with non-conservative generalized forces $\mathbf{Q}$.
- Computes inertia matrix $\mathbf{M}(\mathbf{q})$, Coriolis matrix $\mathbf{C}(\mathbf{q}, \dot{\mathbf{q}})$, and gravity vector $\mathbf{G}(\mathbf{q})$.
- Extracts symbolic and numerical linearized $\mathbf{A}$ and $\mathbf{B}$ matrices.

### 2. Numerical LQR Design & Bryson Tuning
```bash
python simulation/lqr_design.py
```
- Computes Kalman controllability matrix:
  $$\mathcal{C} = [\mathbf{B}, \mathbf{A}\mathbf{B}, \mathbf{A}^2\mathbf{B}, \mathbf{A}^3\mathbf{B}, \mathbf{A}^4\mathbf{B}, \mathbf{A}^5\mathbf{B}]$$
- Applies Bryson's rule:
  $$Q_{ii} = \frac{1}{(\Delta z_{i,\max})^2}, \quad R = \frac{1}{(u_{\max})^2}$$
- Solves Continuous Algebraic Riccati Equation (CARE):
  $$\mathbf{A}^T \mathbf{P} + \mathbf{P}\mathbf{A} - \mathbf{P}\mathbf{B}\mathbf{R}^{-1}\mathbf{B}^T \mathbf{P} + \mathbf{Q} = 0$$
- Computes closed-loop poles $\lambda(\mathbf{A} - \mathbf{B}\mathbf{K})$ and confirms asymptotic stability.
- Exports C-compatible macro definitions for `firmware/include/dip_config.h`.

## Parameter Adjustment
To modify mechanical parameters, edit `get_default_parameters()` in `simulation/sympy_derivation.py` or the `DIPDynamics` class in `simulation/lqr_design.py`.
