---
name: dip-physics-simulation
description: >-
  Nonlinear ODE Physics Simulation & 3D Visualization for the Double Inverted Pendulum (DIP).
  Integrates full nonlinear equations with force saturation clamping and impulse disturbances,
  provides MuJoCo MJCF 3D physics definition (model.xml), and plots response trajectories (visualize.py).
---

# Skill: `dip-physics-simulation`

## Overview
This skill executes high-fidelity numerical simulation and physics-based validation for the Double Inverted Pendulum on a Cart (DIP), benchmarking controller robustness before hardware deployment.

## When to Use
- When validating LQR closed-loop stability against the full nonlinear differential equations of motion rather than the linearized model.
- When evaluating actuator force saturation limits (e.g. $\pm 25\text{ N}$) and testing for integrator windup or pole tip-over.
- When performing robustness testing against external impulse disturbances (e.g. an abrupt $15\text{ N}$ lateral push lasting $50\text{ ms}$).
- When running 3D interactive physics simulation using MuJoCo via `simulation/model.xml`.
- When generating 4-channel response plots ($x, \theta_1, \theta_2, u$) or performance summary metrics.

## Commands & Workflows

### 1. Run Nonlinear RK4 Integration
```bash
python simulation/nonlinear_sim.py
```
- Performs Runge-Kutta 4th-order numerical integration with a step size of $\Delta t = 1.0\text{ ms}$.
- Applies LQR state feedback with force saturation limits.
- Injects external disturbance force at $t = 1.5\text{ s}$.
- Exports trajectory data to `simulation/output_trajectory.csv`.

### 2. Plot Response Trajectories & Performance Metrics
```bash
python simulation/visualize.py
```
- Reads `simulation/output_trajectory.csv` and outputs engineering KPIs:
  - Peak cart excursion ($x_{\max}$)
  - Maximum link deflection angles ($\theta_{1,\max}, \theta_{2,\max}$)
  - Maximum control effort ($u_{\max}$) and settling time ($t_s$)
- Generates high-resolution 4-channel response figure saved to `simulation/dip_response.png`.

### 3. Launch Interactive 3D MuJoCo Simulation
```bash
python -m mujoco.viewer --mjcf=simulation/model.xml
```
- Renders an interactive 3D environment including:
  - Slider joint rail with physical limits $[-1.2\text{ m}, +1.2\text{ m}]$.
  - Low-friction revolute joints with viscous damping.
  - Linear cart actuator with torque/force saturation constraints.
