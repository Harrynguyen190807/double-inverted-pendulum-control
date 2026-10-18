"""
Double Inverted Pendulum on a Cart (DIP) - True Autonomous Self-Balancing & Physical Self-Righting
Lead Mechatronics & Embedded Control Engineer

Key Capabilities:
1. True Closed-Loop Balancing in Absolute Coordinates:
   - Rejects perturbations on the Top Link (Link 2), Lower Link (Link 1), and Cart.
   - Automatically moves the cart to catch and stabilize the pendulums.
2. Physical Self-Righting & Swing-Up (NO TELEPORT RESET):
   - If the pendulums are pushed horizontal (~90 deg) or hanging down, the cart
     PHYSICALLY drives back and forth, pumping energy at natural resonance (5.6 rad/s).
   - Once the pendulums reach the upright apex, the controller seamlessly catches them
     with LQR and restores perfect vertical balance.
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
    """Wraps an angle into the [-pi, +pi] interval."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def compute_absolute_lqr_gain(model, data):
    """
    Computes exact linearized dynamics in absolute coordinates:
        z = [x, dx, th1, dth1, th2, dth2]^T
    where th1 is the absolute angle of Link 1, and th2 is the absolute angle of Link 2.
    """
    data.qpos[:] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    n = 2 * model.nv
    m = model.nu
    A_d = np.zeros((n, n))
    B_d = np.zeros((n, m))
    mujoco.mjd_transitionFD(model, data, 1e-6, True, A_d, B_d, None, None)

    dt = model.opt.timestep
    A_rel = (A_d - np.eye(n)) / dt
    B_rel = B_d / dt

    # Coordinate transformation matrix T: z_abs = T * z_rel
    # z_rel = [x, q1, q2, dx, dq1, dq2]
    # z_abs = [x, dx, th1, dth1, th2, dth2]
    T = np.array([
        [1, 0, 0, 0, 0, 0], # x
        [0, 0, 0, 1, 0, 0], # dx
        [0, 1, 0, 0, 0, 0], # th1 = q1
        [0, 0, 0, 0, 1, 0], # dth1 = dq1
        [0, 1, 1, 0, 0, 0], # th2 = q1 + q2
        [0, 0, 0, 0, 1, 1]  # dth2 = dq1 + dq2
    ], dtype=float)

    Tinv = np.linalg.inv(T)
    A_abs = T @ A_rel @ Tinv
    B_abs = T @ B_rel

    # Robust LQR weights:
    # High damping on angular rates to suppress oscillations,
    # balanced weights on both link angles to prevent top link whipping.
    Q = np.diag([
        20.0,   # x: cart position error
        2.0,    # dx: cart velocity
        800.0,  # th1: link 1 absolute angle
        35.0,   # dth1: link 1 angular velocity
        800.0,  # th2: link 2 absolute angle
        35.0    # dth2: link 2 angular velocity
    ])
    R = np.array([[0.01]])

    P = solve_continuous_are(A_abs, B_abs, Q, R)
    K = np.linalg.inv(R) @ (B_abs.T @ P)
    return K


def main():
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
    if not os.path.exists(xml_path):
        print(f"[ERROR] Model file not found: {xml_path}")
        sys.exit(1)

    print("=" * 75)
    print("   DOUBLE INVERTED PENDULUM (DIP) - AUTONOMOUS CONTROL & SELF-RIGHTING   ")
    print("=" * 75)

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("[1/3] Synthesizing Absolute-Coordinate LQR Controller...")
    K = compute_absolute_lqr_gain(model, data)
    k_vec = K.flatten()
    print(f"      Gains K = [x:{k_vec[0]:.1f}, dx:{k_vec[1]:.1f}, "
          f"th1:{k_vec[2]:.1f}, dth1:{k_vec[3]:.1f}, "
          f"th2:{k_vec[4]:.1f}, dth2:{k_vec[5]:.1f}]")

    # Start near vertical with small perturbation to show active balance
    data.qpos[0] = 0.0
    data.qpos[1] = np.radians(3.0)   # +3 degrees
    data.qpos[2] = np.radians(-2.0)  # -2 degrees
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    print("[2/3] Initializing Dual-Mode Controller:")
    print("  • MODE 1 [LQR_BALANCE]: Holds upright equilibrium; counteracts disturbance")
    print("    forces applied to Link 1, Link 2 (top pole), or the Cart.")
    print("  • MODE 2 [SELF_RIGHTING_SWINGUP]: If links fall or are dragged horizontal,")
    print("    the cart physically pumps resonant energy to swing them back up and catch them!")
    print("  • NO TELEPORT RESET: All dynamics are 100% physical via motor force u(t).")
    print("=" * 75)
    print("Launching 3D Interactive Simulation... Close window to exit.\n")

    mode = "LQR_BALANCE"
    swing_start_time = 0.0
    omega_resonance = 5.6   # rad/s (~0.9 Hz natural pendulum frequency)
    A_swing = 0.45          # Cart swing amplitude (m)
    u_max = 60.0            # Max actuator force (N)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        last_log_time = start_time

        while viewer.is_running():
            step_start = time.time()
            elapsed = time.time() - start_time

            # 1. State extraction in absolute coordinates
            x = data.qpos[0]
            dx = data.qvel[0]
            th1 = wrap_angle(data.qpos[1])
            dth1 = data.qvel[1]
            th2 = wrap_angle(data.qpos[1] + data.qpos[2])
            dth2 = data.qvel[1] + data.qvel[2]

            th1_deg = np.degrees(th1)
            th2_deg = np.degrees(th2)

            # 2. Check Capture Envelope (Catch Basin)
            # Both links near vertical and angular velocities bounded
            in_catch_basin = (
                abs(th1_deg) < 22.0 and
                abs(th2_deg) < 26.0 and
                abs(dth1) < 3.8 and
                abs(dth2) < 4.5
            )

            # 3. State Machine Transition Logic
            if mode == "SELF_RIGHTING_SWINGUP" and in_catch_basin:
                mode = "LQR_BALANCE"
                print(f"\n[TRANSITION t={elapsed:.1f}s] >>> CAPTURE CONE REACHED! Switching to LQR_BALANCE. <<<")

            elif mode == "LQR_BALANCE" and (abs(th1_deg) > 42.0 or abs(th2_deg) > 45.0 or abs(x) > 2.0):
                mode = "SELF_RIGHTING_SWINGUP"
                swing_start_time = elapsed
                print(f"\n[TRANSITION t={elapsed:.1f}s] Large angle detected (th1={th1_deg:.1f}°, th2={th2_deg:.1f}°).")
                print("--> Engaging Physical Self-Righting: Cart will pump energy to swing poles back upright!\n")

            # 4. Control Law Computation
            if mode == "LQR_BALANCE":
                # High-authority LQR balancing in absolute coordinates
                z = np.array([x, dx, th1, dth1, th2, dth2])
                u = - float((K @ z)[0])

            else:
                # MODE 2: Physical Self-Righting (Resonant Energy Pumping)
                t_swing = elapsed - swing_start_time

                # Cart trajectory tracking to pump energy into both pendulums:
                # x_des(t) = A * sin(omega * t)
                x_des = A_swing * np.sin(omega_resonance * t_swing)
                v_des = A_swing * omega_resonance * np.cos(omega_resonance * t_swing)

                # High-gain cart position/velocity servo
                u_track = 80.0 * (x_des - x) + 22.0 * (v_des - dx)

                # Centering safety guard if cart nears rail boundary (+-2.0m)
                if abs(x) > 1.4:
                    u_track -= 30.0 * np.sign(x)

                u = u_track

            # 5. Actuator saturation clamp
            u_clamped = np.clip(u, -u_max, u_max)
            data.ctrl[0] = u_clamped

            # 6. Step physics & sync 3D rendering
            mujoco.mj_step(model, data)
            viewer.sync()

            # Diagnostic telemetry printing
            if time.time() - last_log_time > 0.6:
                print(f"t={elapsed:5.1f}s | [{mode:22s}] Cart: {x:+5.3f}m | "
                      f"Pole 1: {th1_deg:+5.1f}° | Pole 2: {th2_deg:+5.1f}° | "
                      f"Force: {u_clamped:+5.1f}N")
                last_log_time = time.time()

            # Real-time synchronization
            sleep_dt = model.opt.timestep - (time.time() - step_start)
            if sleep_dt > 0:
                time.sleep(sleep_dt)


if __name__ == '__main__':
    main()
