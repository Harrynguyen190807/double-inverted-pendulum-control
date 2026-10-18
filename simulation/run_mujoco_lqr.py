"""
Double Inverted Pendulum on a Cart (DIP) - Real-time 3D MuJoCo Closed-Loop LQR Simulation
Lead Mechatronics & Embedded Control Engineer

This script runs an interactive 3D physics simulation in MuJoCo with an active
1000 Hz LQR feedback controller. You can perturb the pendulums in real-time
using your mouse and watch the cart actively balance them upright!
"""

import os
import sys
import time
import math
import numpy as np

try:
    import mujoco
    import mujoco.viewer
    from scipy.linalg import solve_continuous_are
except ImportError as e:
    print(f"[ERROR] Missing required library: {e}")
    print("Please install via: pip install mujoco scipy numpy")
    sys.exit(1)


def compute_mujoco_lqr_gain(model, data):
    """
    Computes exact linearized state-space matrices A (6x6) and B (6x1)
    and synthesizes optimal LQR gain matrix K using Bryson's Rule.
    """
    # Reset to vertical equilibrium
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    # Transition finite-difference linearization
    n = 2 * model.nv  # 6 states: [x, q1, q2, dx, dq1, dq2]
    m = model.nu      # 1 input: cart force u
    A_d = np.zeros((n, n))
    B_d = np.zeros((n, m))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, A_d, B_d, None, None)

    dt = model.opt.timestep
    A = (A_d - np.eye(n)) / dt
    B = B_d / dt

    # State weights: [x, q1, q2, dx, dq1, dq2]
    # Penalize angle deviations strongly to ensure stiff upright stabilization
    Q = np.diag([10.0, 450.0, 450.0, 1.0, 45.0, 45.0])
    R = np.array([[0.01]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K


def main():
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
    if not os.path.exists(xml_path):
        print(f"[ERROR] Model file not found: {xml_path}")
        sys.exit(1)

    print("=" * 65)
    print("   DOUBLE INVERTED PENDULUM (DIP) - 3D MUJOCO LQR SIMULATION    ")
    print("=" * 65)
    print(f"Loading MJCF Model: {xml_path}")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("Computing optimal LQR feedback gains...")
    K = compute_mujoco_lqr_gain(model, data)
    print("Optimal Gain Vector K (u = -K * z):")
    k_vec = K.flatten()
    print(f"  k_x   = {k_vec[0]:.4f},  k_q1  = {k_vec[1]:.4f},  k_q2  = {k_vec[2]:.4f}")
    print(f"  k_dx  = {k_vec[3]:.4f},  k_dq1 = {k_vec[4]:.4f},  k_dq2 = {k_vec[5]:.4f}")

    # Set small initial tilt to demonstrate dynamic self-balancing
    data.qpos[0] = 0.0
    data.qpos[1] = np.radians(3.5)   # +3.5 degrees on joint 1
    data.qpos[2] = np.radians(-2.5)  # -2.5 degrees on joint 2
    data.qvel[:] = 0.0

    print("\nLaunching 3D MuJoCo Interactive Viewer...")
    print("-> You can apply external forces by Double-Clicking on links and Right-Dragging!")
    print("-> Close the window or press ESC to exit.\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_print_time = time.time()
        start_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            # 1. State vector: z = [x, q1, q2, dx, dq1, dq2]
            state = np.concatenate([data.qpos, data.qvel])

            # 2. LQR Control Law: u = - K * z
            u_lqr = - float((K @ state)[0])

            # 3. Force Saturation Clamp: [-25.0 N, +25.0 N]
            u_clamped = np.clip(u_lqr, -25.0, 25.0)
            data.ctrl[0] = u_clamped

            # 4. Step physics
            mujoco.mj_step(model, data)

            # 5. Sync viewer rendering
            viewer.sync()

            # Print telemetry periodically (every 0.5s)
            now = time.time()
            if now - last_print_time > 0.5:
                th1_deg = np.degrees(data.qpos[1])
                th2_deg = np.degrees(data.qpos[1] + data.qpos[2])
                print(f"t={now - start_time:5.1f}s | Cart: {data.qpos[0]:+6.3f}m | "
                      f"Pole 1: {th1_deg:+6.2f} deg | Pole 2: {th2_deg:+6.2f} deg | "
                      f"Force: {u_clamped:+6.2f}N")
                last_print_time = now

            # Real-time synchronization
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == '__main__':
    main()
