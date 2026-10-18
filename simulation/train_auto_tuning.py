"""
Fast Evolutionary Auto-Tuner & Self-Learning Optimizer for DIP
Lead Mechatronics & Embedded Control Engineer

This script runs high-speed headless physics simulations to automatically learn
and tune momentum-pumping, swing-up, and LQR parameters across arbitrary initial states
(hanging down, horizontal, random angles), maximizing the average 100-point score
and saving the best parameters to 'learned_params.json'.
"""

import os
import sys
import copy
import numpy as np
import mujoco

from dip_learner import (
    DIPLearner, MODEL_XML, compute_lqr_gain, evaluate_trial, compute_smooth_score
)

def main():
    print("=" * 75)
    print("   DOUBLE INVERTED PENDULUM (DIP) - HIGH-EFFICIENCY AUTO-TUNER   ")
    print("=" * 75)
    print("Objective: Maximize Average 100-Point Score across ALL initial angles.")
    print("Parameters saved automatically to: learned_params.json\n")

    if not os.path.exists(MODEL_XML):
        print(f"[ERROR] model.xml not found at {MODEL_XML}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)

    learner = DIPLearner()
    best_params = copy.deepcopy(learner.params)

    # Suite of initial states:
    test_scenarios = [
        ("Near Upright (+6 deg)",    [0.0, np.radians(6.0), np.radians(-4.0)]),
        ("Slight Tilt (-4 deg)",     [0.0, np.radians(-4.0), np.radians(2.0)]),
        ("Hanging Down (180 deg)",   [0.0, np.pi, 0.0]),
        ("Horizontal (+90 deg)",     [0.0, np.pi / 2.0, 0.0]),
        ("Diagonal (-120 deg)",      [0.0, np.radians(-120.0), np.radians(20.0)]),
    ]

    def evaluate_candidate(params_to_test):
        total_mean = 0.0
        total_peak = 0.0
        total_time100 = 0.0
        catches = 0

        lqr_cfg = params_to_test["lqr"]
        q_vec = [lqr_cfg["q_x"], lqr_cfg["q_dx"], lqr_cfg["q_th1"], lqr_cfg["q_dth1"], lqr_cfg["q_th2"], lqr_cfg["q_dth2"]]
        k_cand = compute_lqr_gain(model, data, q_vec, lqr_cfg["r_u"])
        
        sw_cfg = params_to_test["swingup"]

        for name, init_pos in test_scenarios:
            res = evaluate_trial(model, data, k_cand, sw_cfg, init_pos, duration=8.0)
            total_mean += res["mean_score"]
            total_peak += res["peak_score"]
            total_time100 += res["time_at_100"]
            if res["caught"]:
                catches += 1

        n = len(test_scenarios)
        avg_mean = total_mean / n
        avg_peak = total_peak / n
        avg_time100 = total_time100 / n
        
        # Primary objective: Maximize average score across all angles
        fitness = avg_mean + 0.5 * avg_peak + 2.0 * avg_time100 + 10.0 * catches
        return fitness, avg_mean, avg_peak, avg_time100, catches

    print("[1/3] Evaluating current baseline parameters...")
    base_fit, base_mean, base_peak, base_t100, base_catches = evaluate_candidate(best_params)
    print(f"      Baseline Fitness: {base_fit:.1f} | Mean Score: {base_mean:.1f}/100 | "
          f"Peak: {base_peak:.1f} | 100-Pt Hold Time: {base_t100:.2f}s | Catches: {base_catches}/{len(test_scenarios)}\n")

    best_fitness = base_fit

    print("[2/3] Starting Evolutionary Optimization (20 Generations)...")
    generations = 20
    population_size = 6

    mutation_scales = {
        "omega_pump": 0.08,
        "amp_pump": 0.03,
        "k_track_p": 6.0,
        "k_track_d": 1.5,
        "k_align_p": 2.0,
        "k_align_d": 0.3,
        "catch_th1_deg": 1.0,
        "catch_th2_deg": 1.0,
        "catch_q2_deg": 1.0,
        "catch_dth_max": 0.2,
        "catch_u_max": 2.0
    }

    for gen in range(1, generations + 1):
        candidates = []

        for _ in range(population_size):
            cand = copy.deepcopy(best_params)
            sw = cand["swingup"]
            
            # Mutate swingup parameters within safe physical limits
            sw["omega_pump"] = float(np.clip(sw["omega_pump"] + np.random.normal(0, mutation_scales["omega_pump"]), 4.8, 5.8))
            sw["amp_pump"] = float(np.clip(sw["amp_pump"] + np.random.normal(0, mutation_scales["amp_pump"]), 0.55, 0.85))
            sw["k_track_p"] = float(np.clip(sw["k_track_p"] + np.random.normal(0, mutation_scales["k_track_p"]), 100.0, 160.0))
            sw["k_track_d"] = float(np.clip(sw["k_track_d"] + np.random.normal(0, mutation_scales["k_track_d"]), 20.0, 35.0))
            sw["k_align_p"] = float(np.clip(sw["k_align_p"] + np.random.normal(0, mutation_scales["k_align_p"]), 15.0, 35.0))
            sw["k_align_d"] = float(np.clip(sw["k_align_d"] + np.random.normal(0, mutation_scales["k_align_d"]), 2.0, 5.0))
            sw["catch_th1_deg"] = float(np.clip(sw["catch_th1_deg"] + np.random.normal(0, mutation_scales["catch_th1_deg"]), 14.0, 22.0))
            sw["catch_th2_deg"] = float(np.clip(sw["catch_th2_deg"] + np.random.normal(0, mutation_scales["catch_th2_deg"]), 16.0, 24.0))
            sw["catch_q2_deg"] = float(np.clip(sw["catch_q2_deg"] + np.random.normal(0, mutation_scales["catch_q2_deg"]), 8.0, 16.0))
            sw["catch_dth_max"] = float(np.clip(sw["catch_dth_max"] + np.random.normal(0, mutation_scales["catch_dth_max"]), 2.5, 4.5))
            sw["catch_u_max"] = float(np.clip(sw["catch_u_max"] + np.random.normal(0, mutation_scales["catch_u_max"]), 45.0, 58.0))

            fit, m_sc, p_sc, t100, cat = evaluate_candidate(cand)
            candidates.append((fit, m_sc, p_sc, t100, cat, cand))

        candidates.sort(key=lambda x: x[0], reverse=True)
        top_fit, top_m_sc, top_p_sc, top_t100, top_cat, top_cand = candidates[0]

        if top_fit > best_fitness:
            best_fitness = top_fit
            best_params = top_cand
            best_params["best_mean_score"] = top_m_sc
            best_params["peak_score"] = top_p_sc
            best_params["episodes_trained"] = best_params.get("episodes_trained", 0) + population_size
            learner.save(best_params)

            print(f"  [Gen {gen:02d}] [+] IMPROVED FITNESS: {top_fit:6.1f} | Mean Score: {top_m_sc:5.1f}/100 | "
                  f"Peak: {top_p_sc:5.1f} | 100-Pt Time: {top_t100:4.2f}s | Catches: {top_cat}/{len(test_scenarios)} | SAVED!")
        else:
            best_params["episodes_trained"] = best_params.get("episodes_trained", 0) + population_size
            if gen % 4 == 0:
                print(f"  [Gen {gen:02d}] Best Fitness: {best_fitness:6.1f} | Top Gen Mean: {top_m_sc:5.1f}/100 | Episodes: {best_params['episodes_trained']}")

    print("\n[3/3] Auto-Tuning Optimization Complete!")
    print(f"      Final Best Mean Score: {best_params.get('best_mean_score', 0):.1f}/100")
    print(f"      Best Peak Score:       {best_params.get('peak_score', 0):.1f}/100")
    print(f"      Parameters permanently saved to: {learner.filepath}")
    print("=" * 75)

if __name__ == "__main__":
    main()
