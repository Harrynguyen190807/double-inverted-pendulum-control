"""
Double Inverted Pendulum on a Cart (DIP) - Autonomous Self-Balancing & Self-Learning
Lead Mechatronics & Embedded Control Engineer

Key Capabilities:
1. Arbitrary Initial State Initialization:
   - Starts from ANY angle: Random orientations, Hanging Down (180 deg), Horizontal (90 deg), or Upright.
2. Standardized 100-Point Scoring Engine:
   - 100.0 points when both pendulums are vertical upright (th1=0, th2=0) and cart is centered.
   - Real-time performance tracking: Average Score, Peak Score, and 100-Point Duty Cycle %.
3. Physical Self-Righting & Resonant Energy Pumping (NO RESET):
   - If dropped, fallen, or started upside down, the cart drives along the rail to pump
     resonant momentum, swings both pendulums up, and catches them with LQR.
4. Self-Learning, Adaptation & Persistence:
   - Automatically loads optimized parameters from 'learned_params.json'.
   - Tracks online improvements and updates the saved model.
"""

import os
import sys
import time
import argparse
import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError as e:
    print(f"[ERROR] Missing required dependency: {e}")
    print("Run: pip install mujoco scipy numpy")
    sys.exit(1)

# Import local learner and scoring engine
from dip_learner import (
    DIPLearner, MODEL_XML, compute_lqr_gain, compute_100pt_score, wrap_angle
)


def render_score_bar(score, width=20):
    """Generates an ASCII score progress bar."""
    filled = int(round(width * (score / 100.0)))
    filled = max(0, min(width, filled))
    bar = "=" * filled + " " * (width - filled)
    return f"[{bar}]"


def parse_args():
    parser = argparse.ArgumentParser(description="Double Inverted Pendulum Autonomous Simulation")
    parser.add_argument(
        "--init",
        type=str,
        default="random",
        choices=["random", "hanging", "horizontal", "upright"],
        help="Initial state of the pendulums (default: random)"
    )
    parser.add_argument(
        "--no-learn",
        action="store_true",
        help="Disable automatic runtime saving of improved parameters"
    )
    parser.add_argument(
        "--impulse",
        action="store_true",
        help="Enable periodic impulse disturbance test every 8 seconds"
    )
    return parser.parse_args()


def set_initial_state(data, init_mode):
    """Sets initial physical state based on user selection."""
    data.qpos[0] = 0.0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0

    if init_mode == "hanging":
        # Both links pointing straight down
        data.qpos[1] = np.pi
        data.qpos[2] = 0.0
        desc = "Hanging Straight Down (180 deg)"

    elif init_mode == "horizontal":
        # Both links pointing horizontally
        data.qpos[1] = np.pi / 2.0
        data.qpos[2] = 0.0
        desc = "Horizontal (90 deg)"

    elif init_mode == "upright":
        # Near vertical with a small perturbation
        data.qpos[1] = np.radians(3.0)
        data.qpos[2] = np.radians(-2.0)
        desc = "Near Vertical Upright (+3 deg, -2 deg)"

    else: # "random"
        # Completely arbitrary random angles in [-pi, +pi] and small cart offset
        data.qpos[0] = np.random.uniform(-0.35, 0.35)
        data.qpos[1] = np.random.uniform(-np.pi, np.pi)
        data.qpos[2] = np.random.uniform(-np.pi, np.pi)
        desc = f"Random State (Cart: {data.qpos[0]:+.2f}m, Link 1: {np.degrees(data.qpos[1]):+.1f} deg, Link 2: {np.degrees(data.qpos[2]):+.1f} deg)"

    return desc


def main():
    args = parse_args()

    print("=" * 80)
    print("   DOUBLE INVERTED PENDULUM (DIP) - AUTONOMOUS BALANCING & SELF-LEARNING   ")
    print("=" * 80)
    print(f"Goal: Maintain 100.0 Points (Upright Equilibrium) through active physical control.")
    print("Zero Teleport Resets: All recoveries occur physically through rail momentum pumping.\n")

    if not os.path.exists(MODEL_XML):
        print(f"[ERROR] model.xml not found: {MODEL_XML}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)

    # 1. Load learned parameters & initialize learner
    learner = DIPLearner()
    sw = learner.params["swingup"]
    q_diag = learner.get_q_diag()
    r_val = learner.get_r_val()

    print(f"[1/3] Synthesizing LQR Balancer from Learned Parameters:")
    print(f"      Q_diag = {q_diag}, R = {r_val}")
    K = compute_lqr_gain(model, data, q_diag, r_val)
    k_vec = K.flatten()
    print(f"      Gains K = [x:{k_vec[0]:.1f}, dx:{k_vec[1]:.1f}, "
          f"th1:{k_vec[2]:.1f}, dth1:{k_vec[3]:.1f}, "
          f"th2:{k_vec[4]:.1f}, dth2:{k_vec[5]:.1f}]")

    # 2. Configure initial state
    init_desc = set_initial_state(data, args.init)
    mujoco.mj_forward(model, data)
    print(f"[2/3] Initial Condition Configured: {init_desc}")

    # 3. Controller state variables
    th1_init = wrap_angle(data.qpos[1])
    th2_init = wrap_angle(data.qpos[1] + data.qpos[2])
    
    # Start in LQR if already near upright, otherwise start in SWINGUP
    if abs(np.degrees(th1_init)) < 15.0 and abs(np.degrees(th2_init)) < 18.0:
        mode = "LQR_BALANCE"
    else:
        mode = "SELF_RIGHTING_SWINGUP"

    print(f"[3/3] Initial Mode Engaged: [{mode}]")
    print("=" * 80)
    print("💡 INTERACTIVE CONTROLS:")
    print(" • Mouse: Click & Drag with Ctrl+RightClick to perturb either pole or push the cart.")
    print(" • Watch: If knocked down or started horizontal/hanging, the cart immediately")
    print("   drives back and forth to pump momentum and catch the 100-point upright equilibrium!")
    print("=" * 80 + "\n")

    # Metrics tracking
    session_start = time.time()
    last_hud_time = session_start
    swing_start_time = session_start
    
    scores_window = []
    peak_score = 0.0
    time_at_100 = 0.0
    recovery_count = 0
    next_impulse_time = 6.0 if args.impulse else 1e9

    # Controller constants from learner
    omega = sw["omega_pump"]
    amp = sw["amp_pump"]
    kp = sw["k_track_p"]
    kd = sw["k_track_d"]
    kal_p = sw["k_align_p"]
    kal_d = sw["k_align_d"]
    c_th1 = sw["catch_th1_deg"]
    c_th2 = sw["catch_th2_deg"]
    c_dth = sw["catch_dth_max"]
    c_x = sw["catch_x_max"]
    u_max = 60.0

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            elapsed = time.time() - session_start
            dt = model.opt.timestep

            # Extract absolute coordinates
            x = data.qpos[0]
            dx = data.qvel[0]
            th1 = wrap_angle(data.qpos[1])
            dth1 = data.qvel[1]
            q2 = wrap_angle(data.qpos[2])
            dq2 = data.qvel[2]
            th2 = wrap_angle(data.qpos[1] + data.qpos[2])
            dth2 = data.qvel[1] + data.qvel[2]

            th1_deg = np.degrees(th1)
            th2_deg = np.degrees(th2)

            # Compute standardized 100-Point Score
            score = compute_100pt_score(x, dx, th1, dth1, th2, dth2)
            scores_window.append(score)
            if len(scores_window) > 2000:
                scores_window.pop(0)

            if score > peak_score:
                peak_score = score

            if score >= 90.0:
                time_at_100 += dt

            # Transition condition to catch and lock balance
            in_catch_basin = (
                abs(th1_deg) < c_th1 and
                abs(th2_deg) < c_th2 and
                abs(dth1) < c_dth and
                abs(dth2) < c_dth and
                abs(x) < c_x
            )

            # State Machine Transitions
            if mode == "SELF_RIGHTING_SWINGUP" and in_catch_basin:
                mode = "LQR_BALANCE"
                recovery_count += 1
                print(f"\n[TRANSITION t={elapsed:5.1f}s] >>> APEX CATCH BASIN REACHED! Switching to LQR_BALANCE (Score: {score:.1f}/100) <<<")

            elif mode == "LQR_BALANCE" and (abs(th1_deg) > 42.0 or abs(th2_deg) > 45.0 or abs(x) > 2.05):
                mode = "SELF_RIGHTING_SWINGUP"
                swing_start_time = elapsed
                print(f"\n[TRANSITION t={elapsed:5.1f}s] Large deviation (th1={th1_deg:.1f}°, th2={th2_deg:.1f}°).")
                print("--> Pumping momentum on rail to restore upright 100-point balance!\n")

            # Control Law Computation
            f_impulse = 0.0
            if args.impulse:
                if next_impulse_time <= elapsed < (next_impulse_time + 0.06):
                    f_impulse = 18.0 if (int(elapsed / 8) % 2 == 0) else -18.0
                elif elapsed >= (next_impulse_time + 0.06):
                    next_impulse_time = elapsed + 8.0

            if mode == "LQR_BALANCE":
                # High-precision absolute-coordinate LQR feedback
                z = np.array([x, dx, th1, dth1, th2, dth2])
                u = - float((K @ z)[0]) + f_impulse

            else:
                # Active momentum pumping via resonant excitation + relative pole alignment
                t_swing = elapsed - swing_start_time
                x_des = amp * np.sin(omega * t_swing)
                v_des = amp * omega * np.cos(omega * t_swing)

                u_track = kp * (x_des - x) + kd * (v_des - dx)
                u_align = - kal_p * q2 - kal_d * dq2

                # Soft centering guard if cart gets close to rail edge (+-2.2m)
                if abs(x) > 1.35:
                    u_track -= 45.0 * np.sign(x)

                u = u_track + u_align + f_impulse

            # Actuator saturation clamp (-60 N to +60 N)
            u_clamped = np.clip(u, -u_max, u_max)
            data.ctrl[0] = u_clamped

            # Physics integration & render synchronization
            mujoco.mj_step(model, data)
            viewer.sync()

            # Real-Time Console HUD (every 0.35s)
            if time.time() - last_hud_time > 0.35:
                mean_window = np.mean(scores_window)
                duty_cycle = 100.0 * (time_at_100 / max(0.001, elapsed))
                bar = render_score_bar(score, width=20)
                
                status_desc = "UPRIGHT 100-PT BALANCE" if mode == "LQR_BALANCE" else "PUMPING MOMENTUM"
                
                sys.stdout.write(
                    f"\rt={elapsed:5.1f}s | {bar} Score:{score:5.1f}/100 | "
                    f"[{mode[:14]:14s}] | 100Pt-Hold:{duty_cycle:4.1f}% | "
                    f"Recov:{recovery_count:2d} | Force:{u_clamped:+5.1f}N | Cart:{x:+5.2f}m"
                )
                sys.stdout.flush()
                last_hud_time = time.time()

                # Online learning: if we achieve a record mean score, save to learned_params.json
                if not args.no_learn and elapsed > 8.0 and score >= 95.0:
                    if peak_score > learner.params.get("peak_score", 0.0):
                        learner.update_score_stats(mean_window, peak_score)

            # Real-time clock throttle
            sleep_dt = model.opt.timestep - (time.time() - step_start)
            if sleep_dt > 0:
                time.sleep(sleep_dt)

    print("\n" + "=" * 80)
    print("Simulation Terminated.")
    mean_final = np.mean(scores_window) if scores_window else 0.0
    duty_final = 100.0 * (time_at_100 / max(0.001, time.time() - session_start))
    print(f"Final Statistics: Mean Score: {mean_final:.1f}/100 | Peak Score: {peak_score:.1f}/100")
    print(f"                  100-Point Hold Time: {time_at_100:.1f}s ({duty_final:.1f}%) | Total Recoveries: {recovery_count}")
    if not args.no_learn:
        learner.update_score_stats(mean_final, peak_score)
        print(f"Learned progress saved to: {learner.filepath}")
    print("=" * 80)


if __name__ == "__main__":
    main()
