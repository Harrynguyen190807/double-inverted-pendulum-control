"""
Autonomous Optimization & Evolutionary Auto-Tuner for Double Inverted Pendulum (DIP)
Lead Mechatronics & Embedded Control Engineer

Features:
1. Flexible CLI Configuration:
   - Configurable generation count (--generations / --gens, default: 100).
   - Configurable population size (--pop-size, default: 40).
   - Configurable episode evaluation duration (--max-hold-target / --duration, default: 20.0s).
2. Multi-Objective Fitness Function:
   - Consecutive Hold Reward: Progressive quadratic bonus for unbroken streaks in the 100-point zone (>= 99.0).
   - Cart Drift Penalty: Quadratic penalty on displacement from track center (|x|) and cart velocity (|dx|).
   - Micro-Oscillation Suppression: Penalizes angular velocities (dth1^2 + dth2^2) near upright.
   - Comprehensive multi-scenario evaluation across diverse initial angles.
3. Stagnation Detection & Adaptive Mutation:
   - Tracks stagnation when Best Fitness does not improve for >= 5 consecutive generations.
   - Automatically doubles mutation noise scale and injects 20% randomized exploratory individuals
     to escape local minima.
4. Persistence & Reporting:
   - Automatically writes best parameters to 'learned_params.json'.
   - Detailed progress telemetry logging every generation.
"""

import os
import sys
import copy
import argparse
import numpy as np
import mujoco

from dip_learner import (
    DIPLearner, MODEL_XML, compute_lqr_gain, evaluate_trial
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Autonomous Evolutionary Auto-Tuner for Double Inverted Pendulum"
    )
    parser.add_argument(
        "--generations", "--gens",
        type=int,
        default=100,
        dest="generations",
        help="Total number of evolutionary generations to optimize (default: 100)"
    )
    parser.add_argument(
        "--pop-size",
        type=int,
        default=40,
        dest="pop_size",
        help="Population size per generation (default: 40)"
    )
    parser.add_argument(
        "--max-hold-target", "--duration",
        type=float,
        default=20.0,
        dest="max_hold_target",
        help="Maximum simulation duration (s) per test scenario (default: 20.0s)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility"
    )
    return parser.parse_args()


# Parameter search space bounds for mutation and random injection
PARAM_BOUNDS = {
    "omega_pump":    (4.5, 5.8),
    "amp_pump":      (0.50, 0.85),
    "k_track_p":     (90.0, 170.0),
    "k_track_d":     (18.0, 36.0),
    "k_align_p":     (12.0, 35.0),
    "k_align_d":     (1.8, 5.0),
    "catch_th1_deg": (13.0, 22.0),
    "catch_th2_deg": (15.0, 24.0),
    "catch_q2_deg":  (8.0, 16.0),
    "catch_dth_max": (2.2, 4.5),
    "catch_u_max":   (42.0, 58.0),
    # LQR weights
    "q_x":           (40.0, 110.0),
    "q_dx":          (5.0, 18.0),
    "q_th1":         (600.0, 1200.0),
    "q_dth1":        (25.0, 55.0),
    "q_th2":         (600.0, 1200.0),
    "q_dth2":        (25.0, 55.0),
    "r_u":           (0.005, 0.03)
}

# Baseline mutation standard deviations
BASE_MUTATION_SCALES = {
    "omega_pump":    0.06,
    "amp_pump":      0.025,
    "k_track_p":     5.0,
    "k_track_d":     1.2,
    "k_align_p":     1.8,
    "k_align_d":     0.25,
    "catch_th1_deg": 0.8,
    "catch_th2_deg": 0.8,
    "catch_q2_deg":  0.8,
    "catch_dth_max": 0.15,
    "catch_u_max":   1.5,
    "q_x":           4.0,
    "q_dx":          0.8,
    "q_th1":         35.0,
    "q_dth1":        2.0,
    "q_th2":         35.0,
    "q_dth2":        2.0,
    "r_u":           0.001
}


def sample_random_individual(template_params):
    """Generates a completely new exploratory individual within valid physical bounds."""
    ind = copy.deepcopy(template_params)
    sw = ind["swingup"]
    lqr = ind["lqr"]

    for k in sw:
        if k in PARAM_BOUNDS:
            low, high = PARAM_BOUNDS[k]
            sw[k] = float(np.random.uniform(low, high))

    for k in lqr:
        if k in PARAM_BOUNDS:
            low, high = PARAM_BOUNDS[k]
            lqr[k] = float(np.random.uniform(low, high))

    return ind


def mutate_individual(base_params, mutation_multiplier=1.0):
    """Applies Gaussian perturbation scaled by mutation_multiplier with physical boundary clamping."""
    ind = copy.deepcopy(base_params)
    sw = ind["swingup"]
    lqr = ind["lqr"]

    for k in sw:
        if k in BASE_MUTATION_SCALES and k in PARAM_BOUNDS:
            scale = BASE_MUTATION_SCALES[k] * mutation_multiplier
            low, high = PARAM_BOUNDS[k]
            sw[k] = float(np.clip(sw[k] + np.random.normal(0.0, scale), low, high))

    for k in lqr:
        if k in BASE_MUTATION_SCALES and k in PARAM_BOUNDS:
            scale = BASE_MUTATION_SCALES[k] * mutation_multiplier
            low, high = PARAM_BOUNDS[k]
            lqr[k] = float(np.clip(lqr[k] + np.random.normal(0.0, scale), low, high))

    return ind


def main():
    args = parse_args()

    if args.seed is not None:
        np.random.seed(args.seed)

    print("=" * 85)
    print("   DOUBLE INVERTED PENDULUM (DIP) - ADVANCED OPTIMIZER & AUTO-TUNER   ")
    print("=" * 85)
    print(f"Configurations:")
    print(f" • Total Generations:        {args.generations}")
    print(f" • Population Size:          {args.pop_size} individuals / gen")
    print(f" • Max Hold Target Duration: {args.max_hold_target:.1f} s / trial")
    print(f" • Multi-Objective Reward:   Consecutive Hold Streak + Cart Center Guard + Low Jitter")
    print(f" • Adaptive Mutation:        5-Gen Stagnation Detection -> 2.0x Noise + 20% Random Injection")
    print(f" • Parameters Output File:   learned_params.json\n")

    if not os.path.exists(MODEL_XML):
        print(f"[ERROR] model.xml not found at {MODEL_XML}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)

    learner = DIPLearner()
    best_params = copy.deepcopy(learner.params)

    # Suite of challenging test scenarios covering all angle regimes:
    test_scenarios = [
        ("Near Upright (+6 deg)",    [0.0, np.radians(6.0), np.radians(-4.0)]),
        ("Slight Tilt (-4 deg)",     [0.0, np.radians(-4.0), np.radians(2.0)]),
        ("Hanging Down (180 deg)",   [0.0, np.pi, 0.0]),
        ("Horizontal (+90 deg)",     [0.0, np.pi / 2.0, 0.0]),
        ("Diagonal (-120 deg)",      [0.0, np.radians(-120.0), np.radians(20.0)]),
    ]

    def evaluate_candidate(candidate_params):
        """
        Evaluates a candidate across all scenarios and computes the multi-objective fitness.
        """
        total_mean = 0.0
        total_peak = 0.0
        total_max_streak = 0.0
        total_streak_reward = 0.0
        total_drift = 0.0
        total_micro_osc = 0.0
        catches = 0

        lqr_cfg = candidate_params["lqr"]
        q_vec = [
            lqr_cfg["q_x"], lqr_cfg["q_dx"],
            lqr_cfg["q_th1"], lqr_cfg["q_dth1"],
            lqr_cfg["q_th2"], lqr_cfg["q_dth2"]
        ]
        k_cand = compute_lqr_gain(model, data, q_vec, lqr_cfg.get("r_u", 0.01))
        sw_cfg = candidate_params["swingup"]

        for name, init_pos in test_scenarios:
            res = evaluate_trial(model, data, k_cand, sw_cfg, init_pos, duration=args.max_hold_target)
            total_mean += res["mean_score"]
            total_peak += res["peak_score"]
            total_max_streak += res["max_streak"]
            total_streak_reward += res["streak_reward"]
            total_drift += res["drift_penalty"]
            total_micro_osc += res["micro_osc_penalty"]
            if res["caught"]:
                catches += 1

        n = len(test_scenarios)
        avg_mean = total_mean / n
        avg_peak = total_peak / n
        avg_streak = total_max_streak / n
        avg_streak_reward = total_streak_reward / n
        avg_drift = total_drift / n
        avg_micro_osc = total_micro_osc / n

        # Multi-Objective Fitness Synthesis:
        # 1. Base Mean Score: ensures high average score throughout episode
        # 2. Peak Score: rewards reaching vertical
        # 3. Consecutive Hold Streak: strong positive slope for maintaining unbroken 99+ pts
        # 4. Progressive Streak Integral: rewards sustaining equilibrium without any drop
        # 5. Catch Bonus: rewards successfully executing the apex transition
        # 6. Cart Drift Penalty: heavily penalizes moving away from x=0
        # 7. Micro-Oscillation Penalty: eliminates limit cycles and chattering
        fitness = (
            1.0 * avg_mean +
            0.3 * avg_peak +
            14.0 * avg_streak +
            1.5 * avg_streak_reward +
            25.0 * catches -
            2.0 * avg_drift -
            0.8 * avg_micro_osc
        )

        metrics = {
            "fitness": fitness,
            "avg_mean": avg_mean,
            "avg_peak": avg_peak,
            "avg_streak": avg_streak,
            "catches": catches,
            "avg_drift": avg_drift,
            "avg_micro_osc": avg_micro_osc
        }
        return fitness, metrics

    print("[1/3] Evaluating current baseline parameters...")
    base_fit, base_metrics = evaluate_candidate(best_params)
    print(f"      Baseline Fitness:     {base_fit:6.1f}")
    print(f"      Mean Score:           {base_metrics['avg_mean']:5.1f} / 100")
    print(f"      Peak Score:           {base_metrics['avg_peak']:5.1f} / 100")
    print(f"      Max Hold Streak:      {base_metrics['avg_streak']:5.2f} s")
    print(f"      Apex Catches:         {base_metrics['catches']} / {len(test_scenarios)}")
    print(f"      Cart Drift Penalty:   {base_metrics['avg_drift']:5.2f}")
    print(f"      Micro-Osc Penalty:    {base_metrics['avg_micro_osc']:5.2f}\n")

    best_fitness = base_fit
    best_metrics_recorded = base_metrics
    stagnation_counter = 0
    mutation_multiplier = 1.0

    print(f"[2/3] Starting Evolutionary Optimization ({args.generations} Generations, Pop Size {args.pop_size})...\n")

    for gen in range(1, args.generations + 1):
        candidates = []

        # Determine population composition:
        # If stagnant >= 5 generations: inject 20% random individuals to escape local minimum
        num_random = int(round(0.20 * args.pop_size)) if stagnation_counter >= 5 else 0
        num_mutated = args.pop_size - num_random

        # 1. Mutated candidates around current best
        for _ in range(num_mutated):
            cand = mutate_individual(best_params, mutation_multiplier=mutation_multiplier)
            fit, met = evaluate_candidate(cand)
            candidates.append((fit, met, cand))

        # 2. Random exploratory injection
        for _ in range(num_random):
            cand = sample_random_individual(best_params)
            fit, met = evaluate_candidate(cand)
            candidates.append((fit, met, cand))

        # Sort candidates descending by fitness
        candidates.sort(key=lambda x: x[0], reverse=True)
        top_fit, top_met, top_cand = candidates[0]

        # Check for improvement
        if top_fit > best_fitness:
            improvement = top_fit - best_fitness
            best_fitness = top_fit
            best_params = top_cand
            best_metrics_recorded = top_met

            # Reset stagnation & mutation scale
            stagnation_counter = 0
            mutation_multiplier = 1.0

            # Update stats and persist to JSON file
            best_params["best_mean_score"] = float(top_met["avg_mean"])
            best_params["peak_score"] = float(top_met["avg_peak"])
            best_params["best_max_streak"] = float(top_met["avg_streak"])
            best_params["episodes_trained"] = best_params.get("episodes_trained", 0) + args.pop_size
            learner.save(best_params)

            print(f"  [Gen {gen:03d}/{args.generations:03d}] [+] NEW BEST FIT: {top_fit:6.1f} (+{improvement:4.1f}) | "
                  f"Mean: {top_met['avg_mean']:5.1f}/100 | Peak: {top_met['avg_peak']:5.1f} | "
                  f"Streak: {top_met['avg_streak']:4.2f}s | Catches: {top_met['catches']}/{len(test_scenarios)} | "
                  f"MutScale: {mutation_multiplier:.1f}x | SAVED!")
        else:
            stagnation_counter += 1
            best_params["episodes_trained"] = best_params.get("episodes_trained", 0) + args.pop_size

            # Stagnation detection check
            if stagnation_counter >= 5:
                mutation_multiplier = 2.0
                stag_msg = f"-> STAGNATION DETECTED ({stagnation_counter} gens): Boosted MutScale=2.0x, 20% Random Injection"
            else:
                mutation_multiplier = 1.0
                stag_msg = f"(Stagnant: {stagnation_counter}/5)"

            # Print periodic progress log every 2 generations (or when stagnant)
            if gen % 2 == 0 or stagnation_counter == 5:
                print(f"  [Gen {gen:03d}/{args.generations:03d}] BestFit: {best_fitness:6.1f} | TopGenMean: {top_met['avg_mean']:5.1f}/100 | "
                      f"TopStreak: {top_met['avg_streak']:4.2f}s | Catches: {top_met['catches']}/{len(test_scenarios)} | {stag_msg}")

    print("\n" + "=" * 85)
    print("[3/3] Evolutionary Optimization Complete!")
    print(f"      Final Best Fitness:         {best_fitness:6.1f}")
    print(f"      Final Best Mean Score:      {best_metrics_recorded['avg_mean']:5.1f} / 100")
    print(f"      Final Best Peak Score:      {best_metrics_recorded['avg_peak']:5.1f} / 100")
    print(f"      Final Max Hold Streak:      {best_metrics_recorded['avg_streak']:5.2f} s")
    print(f"      Apex Catch Success:         {best_metrics_recorded['catches']} / {len(test_scenarios)}")
    print(f"      Parameters permanently saved to: {learner.filepath}")
    print("=" * 85)


if __name__ == "__main__":
    main()
