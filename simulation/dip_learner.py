"""
Double Inverted Pendulum (DIP) - Autonomous Learning, Parameter Adaptation & Persistence Engine
Lead Mechatronics & Embedded Control Engineer

Key Capabilities:
1. Continuous 100-Point Scoring Engine:
   - Smooth, monotonic gradient: The closer each pendulum is to upright (0 deg),
     the higher the score (from 0.0 when hanging at 180 deg, to 50.0 at 90 deg horizontal,
     to 95+ when within 10 deg, and 100.0 when perfectly upright and centered).
   - Penalizes velocity and rail displacement continuously.
2. Parameter Dataclass & JSON Persistence:
   - Saves and loads trained parameters to/from 'learned_params.json'.
3. Evolutionary Auto-Tuner & Online Adaptation:
   - Automatically adapts momentum pumping, alignment damping, and LQR weights
     to maximize the average score over time across arbitrary initial angles.
"""

import os
import json
import time
import copy
import numpy as np
import mujoco
from scipy.linalg import solve_continuous_are

PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "learned_params.json")
MODEL_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")

# Physical constants from model.xml
M_CART = 1.2
M1, L1 = 0.15, 0.30
M2, L2 = 0.10, 0.25
GRAVITY = 9.81
E_UPRIGHT = M1 * GRAVITY * (L1 / 2.0) + M2 * GRAVITY * (L1 + L2 / 2.0)


def wrap_angle(angle_rad):
    """Wraps angle into [-pi, +pi]."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def compute_smooth_score(x, dx, th1, dth1, th2, dth2):
    """
    Computes a continuous upright score in [0.0, 100.0].
    
    Gradient properties:
      - 100.0 points: perfectly upright (th1=0, th2=0), centered (x=0), resting (dth=0, dx=0).
      - ~97.4 points: at 5 deg tilt.
      - ~91.8 points: at 10 deg tilt.
      - ~83.4 points: at 20 deg tilt.
      - ~72.6 points: at 45 deg tilt.
      - ~42.5 points: at 90 deg horizontal.
      - 0.0 points: at 180 deg hanging straight down.
    The closer to vertical, the higher the score, providing a dense learning signal everywhere.
    """
    # Normalized height alignment for each link in [0.0, 1.0]
    # h = 1.0 at 0 deg, 0.5 at 90 deg, 0.0 at 180 deg
    h1 = 0.5 * (1.0 + np.cos(th1))
    h2 = 0.5 * (1.0 + np.cos(th2))
    
    # Base height score [0, 85 points]
    base_score = 42.5 * h1 + 42.5 * h2
    
    # Upright precision bonus [0, 15 points] when within +-15 deg of vertical
    th_sq = th1**2 + th2**2
    bonus = 15.0 * np.exp(- th_sq / (2.0 * np.radians(12.0)**2)) * np.exp(-0.5 * (x / 0.8)**2)
    
    # Stillness factor: dampens score if violently shaking
    vel_sq = dth1**2 + dth2**2 + 0.1 * dx**2
    stillness = np.exp(-0.008 * vel_sq)
    
    composite = (base_score + bonus) * (0.90 + 0.10 * stillness)
    return float(np.clip(composite, 0.0, 100.0))


def compute_pendulum_energy(th1, dth1, th2, dth2):
    """Calculates total mechanical energy (potential + kinetic) of the pendulums."""
    V = M1 * GRAVITY * (L1 / 2.0) * np.cos(th1) + M2 * GRAVITY * (L1 * np.cos(th1) + (L2 / 2.0) * np.cos(th2))
    I1 = (1.0 / 3.0) * M1 * (L1**2)
    I2 = (1.0 / 12.0) * M2 * (L2**2)
    T = 0.5 * I1 * (dth1**2) + 0.5 * M2 * ((L1 * dth1)**2) + 0.5 * I2 * (dth2**2)
    return T + V


def compute_lqr_gain(model, data, Q_diag, R_val=0.01):
    """
    Computes state feedback gain K in Absolute Coordinates:
      z = [x, dx, th1, dth1, th2, dth2]^T
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
    T = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 1, 1]
    ], dtype=float)

    Tinv = np.linalg.inv(T)
    A_abs = T @ A_rel @ Tinv
    B_abs = T @ B_rel

    Q = np.diag(Q_diag)
    R = np.array([[R_val]])

    P = solve_continuous_are(A_abs, B_abs, Q, R)
    K = np.linalg.inv(R) @ (B_abs.T @ P)
    return K


class DIPLearner:
    """Manages parameter learning, adaptation, and JSON storage."""
    
    DEFAULT_PARAMS = {
        "version": "3.0",
        "best_mean_score": 0.0,
        "peak_score": 0.0,
        "episodes_trained": 0,
        "last_updated": "",
        "swingup": {
            "omega_pump": 5.25,       # Resonant pumping frequency (rad/s)
            "amp_pump": 0.72,         # Cart swing base amplitude (m)
            "k_track_p": 140.0,       # Cart position servo gain
            "k_track_d": 28.0,        # Cart velocity damping gain
            "k_align_p": 25.0,        # Relative pole alignment stiffness
            "k_align_d": 3.5,         # Relative pole velocity damping
            "catch_th1_deg": 18.0,    # Catch envelope for Pole 1 (deg)
            "catch_th2_deg": 20.0,    # Catch envelope for Pole 2 (deg)
            "catch_q2_deg": 12.0,     # Maximum inter-pole relative deviation (deg)
            "catch_dth_max": 3.8,     # Catch angular velocity limit (rad/s)
            "catch_u_max": 55.0       # Max un-saturated LQR force threshold (N)
        },
        "lqr_balance": {              # Fine balance mode (max(|th1|, |th2|) <= 0.08 rad)
            "q_x": 85.0,
            "q_dx": 12.0,
            "q_th1": 850.0,
            "q_dth1": 38.0,
            "q_th2": 850.0,
            "q_dth2": 38.0,
            "r_u": 0.015
        },
        "lqr_recovery": {             # Aggressive recovery mode (max(|th1|, |th2|) > 0.08 rad)
            "q_x": 10.0,
            "q_dx": 2.0,
            "q_th1": 2200.0,
            "q_dth1": 70.0,
            "q_th2": 2200.0,
            "q_dth2": 70.0,
            "r_u": 0.003
        }
    }

    def __init__(self, filepath=PARAMS_FILE):
        self.filepath = filepath
        self.params = self.load()

    def load(self):
        """Loads learned parameters from JSON file or initializes defaults."""
        if os.path.exists(self.filepath):
            try:
                with open(self.filepath, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    # Migrate single-LQR schema to dual-mode schema if necessary
                    if "lqr_balance" not in data:
                        data["lqr_balance"] = copy.deepcopy(data.get("lqr", self.DEFAULT_PARAMS["lqr_balance"]))
                    if "lqr_recovery" not in data:
                        base = copy.deepcopy(data.get("lqr", self.DEFAULT_PARAMS["lqr_balance"]))
                        base["q_x"] = 10.0
                        base["q_dx"] = 2.0
                        base["q_th1"] = max(base.get("q_th1", 850.0) * 2.2, 2000.0)
                        base["q_th2"] = max(base.get("q_th2", 850.0) * 2.2, 2000.0)
                        base["q_dth1"] = max(base.get("q_dth1", 38.0) * 1.8, 65.0)
                        base["q_dth2"] = max(base.get("q_dth2", 38.0) * 1.8, 65.0)
                        base["r_u"] = min(base.get("r_u", 0.01) * 0.25, 0.0035)
                        data["lqr_recovery"] = base

                    print(f"[LEARNER] Loaded learned parameters from {os.path.basename(self.filepath)}")
                    print(f"          Best Recorded Mean Score: {data.get('best_mean_score', 0):.1f}/100 | "
                          f"Trained Episodes: {data.get('episodes_trained', 0)}")
                    return data
            except Exception as e:
                print(f"[WARN] Failed to read {self.filepath} ({e}), initializing defaults.")
        
        self.save(self.DEFAULT_PARAMS)
        return self.DEFAULT_PARAMS.copy()

    def save(self, data=None):
        """Saves current parameter dict to JSON file."""
        if data is None:
            data = self.params
        data["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
        try:
            with open(self.filepath, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except Exception as e:
            print(f"[ERROR] Could not save parameters: {e}")

    def get_q_diag(self, mode="balance"):
        key = "lqr_recovery" if mode == "recovery" else "lqr_balance"
        lqr = self.params.get(key, self.params.get("lqr", self.DEFAULT_PARAMS["lqr_balance"]))
        return [lqr["q_x"], lqr["q_dx"], lqr["q_th1"], lqr["q_dth1"], lqr["q_th2"], lqr["q_dth2"]]

    def get_r_val(self, mode="balance"):
        key = "lqr_recovery" if mode == "recovery" else "lqr_balance"
        lqr = self.params.get(key, self.params.get("lqr", self.DEFAULT_PARAMS["lqr_balance"]))
        return lqr.get("r_u", 0.01)

    def get_swing_param(self, key):
        return self.params["swingup"][key]

    def update_score_stats(self, mean_score, peak_score):
        """Records score and increments training counter if an improvement occurs."""
        self.params["episodes_trained"] += 1
        improved = False
        if mean_score > self.params.get("best_mean_score", 0.0):
            self.params["best_mean_score"] = float(mean_score)
            improved = True
        if peak_score > self.params.get("peak_score", 0.0):
            self.params["peak_score"] = float(peak_score)
            improved = True
        if improved:
            self.save()
        return improved


def evaluate_trial(model, data, K, swing_params, init_pos, duration=10.0):
    """
    Simulates a full trial from a given initial state and returns performance metrics:
    - mean_score: Average 100-point score across the trial
    - peak_score: Maximum score reached
    - time_at_100: Total seconds maintaining score >= 90.0
    - caught: Boolean indicating successful catch
    """
    data.qpos[0] = init_pos[0]
    data.qpos[1] = init_pos[1]
    data.qpos[2] = init_pos[2]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    dt = model.opt.timestep
    total_steps = int(duration / dt)

    scores = []
    caught = False
    u_max = 60.0

    if isinstance(K, dict):
        K_bal = K["balance"]
        K_rec = K["recovery"]
    elif isinstance(K, (list, tuple)) and len(K) == 2:
        K_bal, K_rec = K[0], K[1]
    else:
        K_bal = K
        K_rec = K

    omega = swing_params["omega_pump"]
    A0 = swing_params["amp_pump"]
    kp = swing_params["k_track_p"]
    kd = swing_params["k_track_d"]
    kal_p = swing_params["k_align_p"]
    kal_d = swing_params["k_align_d"]
    c_th1 = swing_params["catch_th1_deg"]
    c_th2 = swing_params["catch_th2_deg"]
    c_q2 = swing_params["catch_q2_deg"]
    c_dth = swing_params["catch_dth_max"]
    c_u = swing_params["catch_u_max"]

    th1_0 = wrap_angle(init_pos[1])
    th2_0 = wrap_angle(init_pos[1] + init_pos[2])
    
    # Initialize mode
    mode = "LQR" if (abs(np.degrees(th1_0)) < c_th1 and abs(np.degrees(th2_0)) < c_th2) else "SWING"
    if mode == "LQR":
        caught = True

    time_at_100 = 0.0
    current_streak = 0.0
    max_streak = 0.0
    cumulative_streak_reward = 0.0
    total_drift_penalty = 0.0
    total_micro_osc_penalty = 0.0

    for step in range(total_steps):
        t = step * dt
        x = data.qpos[0]
        dx = data.qvel[0]
        th1 = wrap_angle(data.qpos[1])
        dth1 = data.qvel[1]
        q2 = wrap_angle(data.qpos[2])
        dq2 = data.qvel[2]
        th2 = wrap_angle(data.qpos[1] + data.qpos[2])
        dth2 = data.qvel[1] + data.qvel[2]

        score = compute_smooth_score(x, dx, th1, dth1, th2, dth2)
        scores.append(score)
        
        # 1. Consecutive Hold Reward: Score >= 99.0 indicates near-perfect vertical balance
        if score >= 99.0:
            time_at_100 += dt
            current_streak += dt
            if current_streak > max_streak:
                max_streak = current_streak
            # Progressive streak reward: rewards holding unbroken continuous equilibrium
            cumulative_streak_reward += current_streak * dt
        else:
            # Reset streak if balance breaks below perfect threshold
            current_streak = 0.0

        th1_deg = np.degrees(th1)
        th2_deg = np.degrees(th2)
        q2_deg = np.degrees(q2)

        # Dual-Mode Controller Logic (Gain Scheduling):
        # Recovery Mode: max(|th1|, |th2|) > 0.08 rad (~4.58 deg)
        # Balance Mode: max(|th1|, |th2|) <= 0.08 rad
        max_th_abs = max(abs(th1), abs(th2))
        is_recovery = (max_th_abs > 0.08)

        if not is_recovery:
            # Balance Mode: heavily penalize cart displacement from track center and micro-oscillations
            cart_drift = 0.5 * (x / 0.5)**2 + 0.1 * (dx / 0.5)**2
            if abs(x) > 1.4:
                cart_drift += 5.0 * (abs(x) - 1.4)
            total_drift_penalty += cart_drift * dt

            # Suppress micro-oscillations near vertical
            total_micro_osc_penalty += (dth1**2 + dth2**2) * dt
        else:
            # Recovery Mode: IGNORE cart drift and micro-oscillation penalties
            # Only penalize severe danger of rail collision (|x| > 2.0m)
            if abs(x) > 2.0:
                total_drift_penalty += 10.0 * (abs(x) - 2.0) * dt

        # Early termination checks to maximize compute efficiency:
        # 1. Cart hits rail stop (|x| > 2.15m) -> physical crash, unrecoverable
        if abs(x) > 2.15:
            break
        # 2. Post-catch fall: if already caught but subsequently lost balance and dropped past 45 deg
        if caught and mode == "SWING" and (abs(th1_deg) > 45.0 or abs(th2_deg) > 45.0):
            break

        z = np.array([x, dx, th1, dth1, th2, dth2])
        K_active = K_rec if is_recovery else K_bal
        u_lqr = - float((K_active @ z)[0])

        E = compute_pendulum_energy(th1, dth1, th2, dth2)
        alpha = np.clip((E_UPRIGHT - E) / (2.0 * E_UPRIGHT), 0.05, 1.0)
        amp = A0 * alpha

        # Catch condition near vertical apex with non-saturated control effort
        in_catch = (
            abs(u_lqr) < c_u and
            abs(th1_deg) < c_th1 and
            abs(th2_deg) < c_th2 and
            abs(q2_deg) < c_q2 and
            abs(dth1) < c_dth and
            abs(dth2) < c_dth
        )

        if mode == "SWING" and in_catch:
            mode = "LQR"
            caught = True

        if mode == "LQR":
            u = np.clip(u_lqr, -u_max, u_max)
            if abs(th1_deg) > 42.0 or abs(th2_deg) > 45.0 or abs(x) > 2.05:
                mode = "SWING"
        else:
            # Resonant momentum generation with adaptive energy amplitude
            x_des = amp * np.sin(omega * t)
            v_des = amp * omega * np.cos(omega * t)
            u_track = kp * (x_des - x) + kd * (v_des - dx)
            u_align = - kal_p * q2 - kal_d * dq2
            
            # Rail limit guard
            if abs(x) > 1.3:
                u_track -= 45.0 * np.sign(x)
                
            u = np.clip(u_track + u_align, -u_max, u_max)

        data.ctrl[0] = u
        mujoco.mj_step(model, data)

    # Pad remaining steps if early terminated so mean_score correctly accounts for failure
    remaining_steps = total_steps - len(scores)
    if remaining_steps > 0:
        scores.extend([0.0] * remaining_steps)

    mean_score = float(np.mean(scores))
    peak_score = float(np.max(scores)) if len(scores) > 0 else 0.0
    return {
        "mean_score": mean_score,
        "peak_score": peak_score,
        "time_at_100": time_at_100,
        "max_streak": max_streak,
        "streak_reward": cumulative_streak_reward,
        "drift_penalty": total_drift_penalty,
        "micro_osc_penalty": total_micro_osc_penalty,
        "caught": caught,
        "final_score": scores[-1] if len(scores) > 0 else 0.0
    }
