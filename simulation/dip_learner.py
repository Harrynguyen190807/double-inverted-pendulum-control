"""
Double Inverted Pendulum (DIP) - Autonomous Learning, Parameter Adaptation & Persistence Engine
Lead Mechatronics & Embedded Control Engineer

This module provides:
1. Standardized 100-Point Scoring Engine:
   - 100.0 points when both pendulums are upright (th1=0, th2=0) and cart is centered (x=0).
   - Decreases smoothly with angular tilt. Drops to 0.0 when horizontal (90 deg) or hanging.
2. Parameter Dataclass & JSON Persistence:
   - Saves and loads trained parameters to/from 'learned_params.json'.
3. Online & Offline Evolutionary Auto-Tuner:
   - Optimizes momentum pumping frequency, amplitude, alignment gains, and LQR weights
     across arbitrary initial states (hanging, horizontal, random angles).
"""

import os
import json
import time
import numpy as np
import mujoco
from scipy.linalg import solve_continuous_are

PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "learned_params.json")
MODEL_XML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")


def wrap_angle(angle_rad):
    """Wraps angle into [-pi, +pi]."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


def compute_100pt_score(x, dx, th1, dth1, th2, dth2):
    """
    Computes upright score in [0.0, 100.0].
    
    Formula:
      - 100.0 points: perfectly upright (th1=0, th2=0), centered (x=0), resting (dth=0, dx=0).
      - Drops smoothly as pendulums lean.
      - 0.0 points if either pendulum is horizontal (>= 90 deg) or hanging downward.
    """
    # Angle alignment factors (upper hemisphere only)
    c1 = max(0.0, np.cos(th1))
    c2 = max(0.0, np.cos(th2))
    
    # Pure 0 if past horizontal
    if c1 <= 0.0 or c2 <= 0.0:
        return 0.0
    
    # Exponential angular precision: (cos th)^4 gives 100 at 0 deg, 90 at 10 deg, 50 at 25 deg, 0 at 90 deg
    angle_factor = (c1 ** 3.5) * (c2 ** 3.5)
    
    # Cart centering factor: Gaussian decay around x=0 (half-width 0.8m)
    cart_factor = np.exp(-0.5 * (x / 0.7) ** 2)
    
    # Dynamic stillness factor: rewards quiet steady-state, penalizes jitter
    speed_sq = dth1**2 + dth2**2 + 0.2 * dx**2
    vel_factor = np.exp(-0.015 * speed_sq)
    
    # Weighted composite: 85% upright geometry, 15% centering & stillness
    composite = angle_factor * (0.85 + 0.15 * cart_factor * vel_factor)
    return float(np.clip(100.0 * composite, 0.0, 100.0))


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
    # z_rel = [x, q1, q2, dx, dq1, dq2]
    # z_abs = [x, dx, th1, dth1, th2, dth2]
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
        "version": "2.0",
        "best_mean_score": 0.0,
        "peak_score": 0.0,
        "episodes_trained": 0,
        "last_updated": "",
        "swingup": {
            "omega_pump": 5.4,        # Resonant pumping frequency (rad/s)
            "amp_pump": 0.72,         # Cart swing amplitude (m)
            "k_track_p": 120.0,       # Cart position servo gain
            "k_track_d": 24.0,        # Cart velocity damping gain
            "k_align_p": 15.0,        # Relative pole alignment stiffness
            "k_align_d": 2.2,         # Relative pole velocity damping
            "catch_th1_deg": 18.0,    # Catch envelope for Pole 1 (deg)
            "catch_th2_deg": 22.0,    # Catch envelope for Pole 2 (deg)
            "catch_dth_max": 3.2,     # Catch angular velocity limit (rad/s)
            "catch_x_max": 0.65       # Max cart distance from center for catch (m)
        },
        "lqr": {
            "q_x": 25.0,
            "q_dx": 3.0,
            "q_th1": 850.0,
            "q_dth1": 38.0,
            "q_th2": 850.0,
            "q_dth2": 38.0,
            "r_u": 0.01
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
                    print(f"[LEARNER] Loaded learned parameters from {os.path.basename(self.filepath)}")
                    print(f"          Best Recorded Mean Score: {data.get('best_mean_score', 0):.1f}/100 | "
                          f"Trained Episodes: {data.get('episodes_trained', 0)}")
                    return data
            except Exception as e:
                print(f"[WARN] Failed to read {self.filepath} ({e}), initializing defaults.")
        
        # Save defaults if file not present
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
            # print(f"[LEARNER] Saved updated parameters to {os.path.basename(self.filepath)}")
        except Exception as e:
            print(f"[ERROR] Could not save parameters: {e}")

    def get_q_diag(self):
        lqr = self.params["lqr"]
        return [lqr["q_x"], lqr["q_dx"], lqr["q_th1"], lqr["q_dth1"], lqr["q_th2"], lqr["q_dth2"]]

    def get_r_val(self):
        return self.params["lqr"].get("r_u", 0.01)

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
    Simulates a full trial from a given initial state and returns evaluation metrics:
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
    mode = "SWING"
    u_max = 60.0

    omega = swing_params["omega_pump"]
    amp = swing_params["amp_pump"]
    kp = swing_params["k_track_p"]
    kd = swing_params["k_track_d"]
    kal_p = swing_params["k_align_p"]
    kal_d = swing_params["k_align_d"]
    c_th1 = swing_params["catch_th1_deg"]
    c_th2 = swing_params["catch_th2_deg"]
    c_dth = swing_params["catch_dth_max"]
    c_x = swing_params["catch_x_max"]

    time_at_100 = 0.0

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

        score = compute_100pt_score(x, dx, th1, dth1, th2, dth2)
        scores.append(score)
        if score >= 90.0:
            time_at_100 += dt

        th1_deg = np.degrees(th1)
        th2_deg = np.degrees(th2)

        # Catch condition near vertical apex
        in_catch = (
            abs(th1_deg) < c_th1 and
            abs(th2_deg) < c_th2 and
            abs(dth1) < c_dth and
            abs(dth2) < c_dth and
            abs(x) < c_x
        )

        if mode == "SWING" and in_catch:
            mode = "LQR"
            caught = True

        if mode == "LQR":
            z = np.array([x, dx, th1, dth1, th2, dth2])
            u = - float((K @ z)[0])
            if abs(th1_deg) > 42.0 or abs(th2_deg) > 45.0 or abs(x) > 2.0:
                mode = "SWING"
        else:
            # Resonant momentum generation with relative alignment damping
            x_des = amp * np.sin(omega * t)
            v_des = amp * omega * np.cos(omega * t)
            u_track = kp * (x_des - x) + kd * (v_des - dx)
            u_align = - kal_p * q2 - kal_d * dq2
            
            # Boundary protection
            if abs(x) > 1.4:
                u_track -= 40.0 * np.sign(x)
                
            u = u_track + u_align

        u_clamped = np.clip(u, -u_max, u_max)
        data.ctrl[0] = u_clamped
        mujoco.mj_step(model, data)

    mean_score = float(np.mean(scores))
    peak_score = float(np.max(scores))
    return {
        "mean_score": mean_score,
        "peak_score": peak_score,
        "time_at_100": time_at_100,
        "caught": caught,
        "final_score": scores[-1]
    }
