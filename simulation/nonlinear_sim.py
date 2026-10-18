"""
Double Inverted Pendulum on a Cart (DIP) - Nonlinear ODE Simulation & LQR Stabilization
Lead Mechatronics & Embedded Control Engineer
"""

import math
import os
import csv

# Default physical benchmark parameters
PARAMS = {
    'M': 1.2,       # Cart mass (kg)
    'm1': 0.15,     # Pendulum 1 mass (kg)
    'm2': 0.10,     # Pendulum 2 mass (kg)
    'L1': 0.30,     # Pendulum 1 total length (m)
    'L2': 0.25,     # Pendulum 2 total length (m)
    'l1': 0.15,     # COM 1 distance (m)
    'l2': 0.125,    # COM 2 distance (m)
    'J1': (1.0 / 12.0) * 0.15 * (0.30 ** 2),  # 0.001125 kg*m^2
    'J2': (1.0 / 12.0) * 0.10 * (0.25 ** 2),  # 0.000521 kg*m^2
    'g': 9.81,      # Gravity (m/s^2)
    'b': 0.5,       # Cart friction (N*s/m)
    'd1': 0.005,    # Joint 1 friction (N*m*s/rad)
    'd2': 0.003,    # Joint 2 friction (N*m*s/rad)
    'u_max': 25.0   # Actuator force saturation limit (N)
}

# Pre-computed optimal LQR gains for default system: u = - K * z
# State z = [x, dx, th1, dth1, th2, dth2]
DEFAULT_K = [400.0, 397.857298, 1819.630163, -179.157067, -3720.664533, -341.436391]



def solve_3x3_linear_system(A, b):
    """Solves A * x = b for 3x3 matrix using Gaussian elimination with partial pivoting."""
    # Create augmented matrix [A | b]
    M = [[A[i][j] for j in range(3)] + [b[i]] for i in range(3)]
    for i in range(3):
        # Pivot
        max_row = i
        for k in range(i + 1, 3):
            if abs(M[k][i]) > abs(M[max_row][i]):
                max_row = k
        M[i], M[max_row] = M[max_row], M[i]

        pivot = M[i][i]
        if abs(pivot) < 1e-12:
            raise ValueError("Singular mass matrix in DIP dynamics")

        for k in range(i + 1, 4):
            M[i][k] /= pivot
        M[i][i] = 1.0

        for row in range(3):
            if row != i:
                factor = M[row][i]
                for col in range(i, 4):
                    M[row][col] -= factor * M[i][col]

    return [M[0][3], M[1][3], M[2][3]]


def dip_nonlinear_derivatives(t, state, u_force, p=PARAMS):
    """
    Computes [dx, ddx, dth1, ddth1, dth2, ddth2] from full nonlinear Lagrangian EOM.
    State vector: [x, dx, th1, dth1, th2, dth2]
    """
    x, dx, th1, dth1, th2, dth2 = state

    M, m1, m2 = p['M'], p['m1'], p['m2']
    L1, l1, l2 = p['L1'], p['l1'], p['l2']
    J1, J2 = p['J1'], p['J2']
    g, b, d1, d2 = p['g'], p['b'], p['d1'], p['d2']

    s1, c1 = math.sin(th1), math.cos(th1)
    s2, c2 = math.sin(th2), math.cos(th2)
    s12 = math.sin(th1 - th2)
    c12 = math.cos(th1 - th2)

    # Mass matrix elements M(q)
    # Row 1 (cart equation):
    m11 = M + m1 + m2
    m12 = -(m1 * l1 + m2 * L1) * c1
    m13 = -m2 * l2 * c2

    # Row 2 (pendulum 1 equation):
    m21 = m12
    m22 = J1 + m1 * (l1 ** 2) + m2 * (L1 ** 2)
    m23 = m2 * L1 * l2 * c12

    # Row 3 (pendulum 2 equation):
    m31 = m13
    m32 = m23
    m33 = J2 + m2 * (l2 ** 2)

    M_mat = [
        [m11, m12, m13],
        [m21, m22, m23],
        [m31, m32, m33]
    ]

    # Generalized forces & Coriolis/centrifugal & gravity terms
    # Cart RHS:
    # u - b*dx - (m1*l1 + m2*L1)*s1*dth1^2 - m2*l2*s2*dth2^2
    rhs1 = u_force - b * dx - (m1 * l1 + m2 * L1) * s1 * (dth1 ** 2) - m2 * l2 * s2 * (dth2 ** 2)

    # Pendulum 1 RHS:
    # -d1*dth1 + d2*(dth2 - dth1) + (m1*l1 + m2*L1)*g*s1 - m2*L1*l2*s12*(dth2^2)
    rhs2 = (-d1 * dth1 + d2 * (dth2 - dth1)
            + (m1 * l1 + m2 * L1) * g * s1
            - m2 * L1 * l2 * s12 * (dth2 ** 2))

    # Pendulum 2 RHS:
    # -d2*(dth2 - dth1) + m2*g*l2*s2 + m2*L1*l2*s12*(dth1^2)
    rhs3 = (-d2 * (dth2 - dth1)
            + m2 * g * l2 * s2
            + m2 * L1 * l2 * s12 * (dth1 ** 2))

    rhs = [rhs1, rhs2, rhs3]

    # Accelerations [ddx, ddth1, ddth2]
    ddq = solve_3x3_linear_system(M_mat, rhs)

    return [dx, ddq[0], dth1, ddq[1], dth2, ddq[2]]


def simulate_dip(t_final=5.0, dt=0.001,
                 init_state=None,
                 K=DEFAULT_K,
                 disturb_time=1.5, disturb_force=15.0, disturb_dur=0.05,
                 output_csv="simulation/output_trajectory.csv"):
    """
    Performs high-fidelity Runge-Kutta 4th-order (RK4) integration with LQR feedback,
    actuator force saturation clamp, and an external impulse disturbance.
    """
    if init_state is None:
        # Initial condition: Cart at 0, small perturbation on angles (+2 deg on th1, -1.5 deg on th2)
        th1_0 = math.radians(2.0)
        th2_0 = math.radians(-1.5)
        init_state = [0.0, 0.0, th1_0, 0.0, th2_0, 0.0]

    u_max = PARAMS['u_max']
    n_steps = int(t_final / dt)

    t = 0.0
    state = list(init_state)

    history = []

    print(f"Starting DIP Simulation: t_span=[0, {t_final}s], dt={dt}s, Steps={n_steps}")
    print(f"Initial State: x={state[0]:.3f}m, th1={math.degrees(state[2]):.2f}deg, th2={math.degrees(state[4]):.2f}deg")

    for step in range(n_steps):
        # 1. External disturbance force
        f_disturb = 0.0
        if disturb_time <= t < (disturb_time + disturb_dur):
            f_disturb = disturb_force

        # 2. LQR Control Law: u = - K * z
        u_lqr = - sum(k * z for k, z in zip(K, state))

        # 3. Force Saturation Clamp: [-u_max, +u_max]
        u_control = max(-u_max, min(u_max, u_lqr))
        u_total = u_control + f_disturb

        # Record telemetry
        history.append({
            'time': t,
            'x': state[0],
            'dx': state[1],
            'th1': state[2],
            'dth1': state[3],
            'th2': state[4],
            'dth2': state[5],
            'u': u_control,
            'f_disturb': f_disturb
        })

        # 4. RK4 Integration Step
        k1 = dip_nonlinear_derivatives(t, state, u_total)
        s_k2 = [s + 0.5 * dt * d for s, d in zip(state, k1)]

        k2 = dip_nonlinear_derivatives(t + 0.5 * dt, s_k2, u_total)
        s_k3 = [s + 0.5 * dt * d for s, d in zip(state, k2)]

        k3 = dip_nonlinear_derivatives(t + 0.5 * dt, s_k3, u_total)
        s_k4 = [s + dt * d for s, d in zip(state, k3)]

        k4 = dip_nonlinear_derivatives(t + dt, s_k4, u_total)

        state = [
            s + (dt / 6.0) * (dk1 + 2 * dk2 + 2 * dk3 + dk4)
            for s, dk1, dk2, dk3, dk4 in zip(state, k1, k2, k3, k4)
        ]

        t += dt

    # Save to CSV
    os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
    with open(output_csv, 'w', newline='') as f:
        fieldnames = ['time', 'x', 'dx', 'th1', 'dth1', 'th2', 'dth2', 'u', 'f_disturb']
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(history)

    print(f"Simulation completed successfully. Output saved to: {output_csv}")
    final_x = state[0]
    final_th1 = math.degrees(state[2])
    final_th2 = math.degrees(state[4])
    print(f"Final State at t={t_final:.2f}s: x={final_x:.4f}m, th1={final_th1:.4f}deg, th2={final_th2:.4f}deg")
    return history


if __name__ == '__main__':
    simulate_dip()
