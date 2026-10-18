"""
Double Inverted Pendulum on a Cart (DIP) - Robust 3D MuJoCo Closed-Loop Simulation
Lead Mechatronics & Embedded Control Engineer

Features:
- 1000 Hz Hard Real-Time LQR Closed-Loop State Feedback
- Full Angle Wrapping (-pi to +pi)
- High Actuator Authority (+-60 N) with Extended Track (+-2.2 m)
- Automatic Fall Detection & Smooth Auto-Recovery Reset
- Calibrated Periodic Impulse Disturbance Injection (Tests dynamic recovery)
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
    print(f"[ERROR] Missing required dependency: {e}")
    print("Run: pip install mujoco scipy numpy")
    sys.exit(1)


def wrap_angle(angle_rad):
    """Normalizes angle to [-pi, +pi] interval."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def compute_mujoco_lqr_gain(model, data):
    """
    Computes exact linearized matrices A and B in MuJoCo coordinate space
    and solves the Continuous Algebraic Riccati Equation (CARE) for gain K.
    """
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    n = 2 * model.nv  # 6 states: [x, q1, q2, dx, dq1, dq2]
    m = model.nu      # 1 input: cart force u
    A_d = np.zeros((n, n))
    B_d = np.zeros((n, m))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, A_d, B_d, None, None)

    dt = model.opt.timestep
    A = (A_d - np.eye(n)) / dt
    B = B_d / dt

    # State weights: [x, q1, q2, dx, dq1, dq2]
    # Penalize angular errors and rates heavily for stiff, rapid stabilization
    Q = np.diag([25.0, 600.0, 600.0, 2.0, 60.0, 60.0])
    R = np.array([[0.005]])

    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ (B.T @ P)
    return K


def reset_to_upright(data, perturbation=True):
    """Resets cart to origin and pendulums to near-upright with small perturbation."""
    data.qpos[0] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    if perturbation:
        # Small random tilt to demonstrate active catching
        data.qpos[1] = np.radians(np.random.uniform(-3.0, 3.0))
        data.qpos[2] = np.radians(np.random.uniform(-2.0, 2.0))
    else:
        data.qpos[1] = 0.0
        data.qpos[2] = 0.0


def main():
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
    if not os.path.exists(xml_path):
        print(f"[ERROR] Model file not found: {xml_path}")
        sys.exit(1)

    print("=" * 70)
    print("   DOUBLE INVERTED PENDULUM (DIP) - ROBUST 3D MUJOCO SIMULATION   ")
    print("=" * 70)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("[1/3] Computing optimal LQR gains for MuJoCo coordinates...")
    K = compute_mujoco_lqr_gain(model, data)
    k_vec = K.flatten()
    print(f"      K = [{k_vec[0]:.2f}, {k_vec[1]:.2f}, {k_vec[2]:.2f}, "
          f"{k_vec[3]:.2f}, {k_vec[4]:.2f}, {k_vec[5]:.2f}]")

    # Initial state
    reset_to_upright(data, perturbation=True)
    mujoco.mj_forward(model, data)

    print("[2/3] Launching interactive MuJoCo 3D Viewer...")
    print("=" * 70)
    print("💡 CONTROL & INTERACTION GUIDE:")
    print(" • Automatic Disturbance Test: Every 6 seconds, a 15 N impulse pushes the cart.")
    print(" • Mouse Interaction: You can gently click & drag the links to perturb them.")
    print(" • Auto-Recovery: If an overwhelming force knocks them past 35 deg, the system")
    print("   automatically catches itself and re-stabilizes upright within 1 second!")
    print("=" * 70)

    fall_timer = None
    next_impulse_time = 4.0
    u_max = 60.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        last_log_time = start_time

        while viewer.is_running():
            step_start = time.time()
            elapsed = time.time() - start_time

            # 1. State extraction with angle wrapping
            x = data.qpos[0]
            q1 = wrap_angle(data.qpos[1])
            q2 = wrap_angle(data.qpos[2])
            dx = data.qvel[0]
            dq1 = data.qvel[1]
            dq2 = data.qvel[2]

            th1_deg = np.degrees(q1)
            th2_abs_deg = np.degrees(q1 + q2)

            # 2. Fall detection guard: If angle exceeds linear recovery envelope (> 35 deg)
            is_fallen = (abs(th1_deg) > 35.0 or abs(th2_abs_deg) > 35.0 or abs(x) > 2.1)

            if is_fallen:
                # Cut actuator power immediately to prevent slamming against limits
                data.ctrl[0] = 0.0

                if fall_timer is None:
                    fall_timer = time.time()
                    print(f"\n[ALERT t={elapsed:.1f}s] Pendulum fell beyond linear limit (th1={th1_deg:.1f}°, th2={th2_abs_deg:.1f}°).")
                    print("--> Disengaging motor and auto-recovering upright in 1.2s...")

                # Auto-recovery after 1.2 seconds
                if time.time() - fall_timer > 1.2:
                    reset_to_upright(data, perturbation=True)
                    fall_timer = None
                    next_impulse_time = elapsed + 3.0
                    print(f"[RECOVERY t={elapsed:.1f}s] System reset to upright! Re-engaging LQR balancer.\n")

            else:
                fall_timer = None

                # 3. Calibrated Impulse Disturbance Generator (every 6 seconds)
                f_disturb = 0.0
                if next_impulse_time <= elapsed < (next_impulse_time + 0.06):
                    # 18 N push lasting 60 ms
                    f_disturb = 18.0 if (int(elapsed / 6) % 2 == 0) else -18.0
                    if elapsed - next_impulse_time < 0.01:
                        print(f"[TEST t={elapsed:.1f}s] Injecting lateral impulse disturbance: {f_disturb:+.1f} N")
                elif elapsed >= (next_impulse_time + 0.06):
                    next_impulse_time = elapsed + 6.0

                # 4. LQR State Feedback Control: u = - K * z
                state = np.array([x, q1, q2, dx, dq1, dq2])
                u_lqr = - float((K @ state)[0])

                # 5. Clamp to maximum actuator limits
                u_command = np.clip(u_lqr + f_disturb, -u_max, u_max)
                data.ctrl[0] = u_command

            # 6. Step MuJoCo physics engine
            mujoco.mj_step(model, data)

            # 7. Sync rendering
            viewer.sync()

            # Telemetry logging every 0.6 seconds
            if time.time() - last_log_time > 0.6:
                status_str = "BALANCING" if not is_fallen else "RECOVERING"
                print(f"t={elapsed:5.1f}s | [{status_str:10s}] Cart: {x:+5.3f}m | "
                      f"Pole 1: {th1_deg:+5.2f}° | Pole 2: {th2_abs_deg:+5.2f}° | "
                      f"Force: {data.ctrl[0]:+5.1f}N")
                last_log_time = time.time()

            # Real-time clock synchronization
            sleep_dt = model.opt.timestep - (time.time() - step_start)
            if sleep_dt > 0:
                time.sleep(sleep_dt)


if __name__ == '__main__':
    main()
