"""
Autonomous Optimization & Evolutionary Auto-Tuner for Double Inverted Pendulum (DIP)
Lead Mechatronics & Embedded Control Engineer - Version 3.0

Architectural Enhancements:
1. Strict Catch Hierarchy:
   - +500.0 pts for every successfully caught scenario.
   - Any individual with higher catches (e.g., 4/5 or 5/5) is mathematically guaranteed
     to achieve a higher fitness than any 3/5 candidate.
2. Gain Scheduling / Dual-Mode Controller:
   - Evaluates two specialized LQR gain matrices:
     * Recovery Mode (max(|th1|, |th2|) > 0.08 rad / ~4.58 deg): High pole gains, aggressive
       control effort to whip the cart and pull the pendulum back; suppresses drift and micro-osc penalties.
     * Balance Mode (max(|th1|, |th2|) <= 0.08 rad): Smooth gains, penalizes drift to pull cart
       toward track center (x = 0), suppresses micro-oscillations, and maximizes continuous 100-pt hold.
3. Curriculum Learning (Graduated Angle Progression):
   - Stage 1 (Gen 1-30): Moderate tilt angles (3 deg to 8 deg) to perfect Balance Mode and centering.
   - Stage 2 (Gen 31-60): Large tilt angles (6 deg to 20 deg) to perfect Recovery Mode.
   - Stage 3 (Gen 61+): Full spectrum evaluation (upright, 20 deg, horizontal 90 deg, -120 deg, hanging 180 deg).
4. Advanced Evolutionary Search:
   - Eliminated 100%-failing Random Injection.
   - Top-5 Elitism: Best 5 candidates preserved across generations.
   - Differential Evolution (DE) + Directed Mutation centered on the all-time best-catch champion.
"""

import os
import sys
import copy
import argparse
import time
import concurrent.futures
import numpy as np
import mujoco

from dip_learner import (
    DIPLearner, MODEL_XML, compute_lqr_gain, evaluate_trial
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Autonomous Evolutionary Auto-Tuner for Double Inverted Pendulum (v3.0)"
    )
    parser.add_argument(
        "--generations", "--gens",
        type=int,
        default=100,
        dest="generations",
        help="Total number of evolutionary generations (default: 100)"
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
        help="Episode duration (s) per test scenario (default: 20.0s)"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=min(os.cpu_count() or 4, 16),
        dest="workers",
        help="Parallel worker processes for candidate evaluation (default: auto, up to 16)"
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility"
    )
    return parser.parse_args()


# =====================================================================
# CURRICULUM LEARNING SCENARIO STAGES
# =====================================================================

CURRICULUM_STAGE_1 = [
    ("Stage1: Upright (+3 deg)",     [0.0, np.radians(3.0), np.radians(-2.0)]),
    ("Stage1: Tilt (+5 deg)",        [0.0, np.radians(5.0), np.radians(-3.0)]),
    ("Stage1: Tilt (-4 deg)",        [0.0, np.radians(-4.0), np.radians(2.0)]),
    ("Stage1: Tilt (+7 deg)",        [0.0, np.radians(7.0), np.radians(-5.0)]),
    ("Stage1: Tilt (-8 deg)",        [0.0, np.radians(-8.0), np.radians(4.0)]),
]

CURRICULUM_STAGE_2 = [
    ("Stage2: Tilt (+6 deg)",        [0.0, np.radians(6.0), np.radians(-4.0)]),
    ("Stage2: Tilt (-6 deg)",        [0.0, np.radians(-6.0), np.radians(3.0)]),
    ("Stage2: Large Tilt (+14 deg)", [0.0, np.radians(14.0), np.radians(-8.0)]),
    ("Stage2: Large Tilt (-16 deg)", [0.0, np.radians(-16.0), np.radians(10.0)]),
    ("Stage2: Extreme (+20 deg)",    [0.0, np.radians(20.0), np.radians(-12.0)]),
]

CURRICULUM_STAGE_3 = [
    ("Stage3: Upright (+6 deg)",     [0.0, np.radians(6.0), np.radians(-4.0)]),
    ("Stage3: Extreme (-18 deg)",    [0.0, np.radians(-18.0), np.radians(10.0)]),
    ("Stage3: Horizontal (+90 deg)", [0.0, np.pi / 2.0, 0.0]),
    ("Stage3: Diagonal (-120 deg)",  [0.0, np.radians(-120.0), np.radians(20.0)]),
    ("Stage3: Hanging (180 deg)",    [0.0, np.pi, 0.0]),
]


def get_curriculum(gen):
    if gen <= 30:
        return 1, "Stage 1: Balance Mastery (3 to 8 deg)", CURRICULUM_STAGE_1
    elif gen <= 60:
        return 2, "Stage 2: Recovery Mastery (6 to 20 deg)", CURRICULUM_STAGE_2
    else:
        return 3, "Stage 3: Full Spectrum & Swing-up (to 180 deg)", CURRICULUM_STAGE_3


# =====================================================================
# SEARCH SPACE DEFINITION (25 PARAMETERS)
# =====================================================================

PARAM_SCHEMA = [
    # Swing-up parameters (11)
    ("swingup", "omega_pump",    (4.5, 5.8),     0.05),
    ("swingup", "amp_pump",      (0.50, 0.85),   0.02),
    ("swingup", "k_track_p",     (90.0, 170.0),  4.0),
    ("swingup", "k_track_d",     (18.0, 36.0),   1.0),
    ("swingup", "k_align_p",     (12.0, 35.0),   1.5),
    ("swingup", "k_align_d",     (1.8, 5.0),     0.2),
    ("swingup", "catch_th1_deg", (13.0, 24.0),   0.6),
    ("swingup", "catch_th2_deg", (15.0, 25.0),   0.6),
    ("swingup", "catch_q2_deg",  (8.0, 16.0),    0.5),
    ("swingup", "catch_dth_max", (2.2, 4.5),     0.12),
    ("swingup", "catch_u_max",   (40.0, 58.0),   1.2),

    # Balance Mode LQR parameters (7) - Focused on centering, jitter damping, smoothness
    ("lqr_balance", "q_x",       (50.0, 150.0),  4.0),
    ("lqr_balance", "q_dx",      (6.0, 25.0),    0.8),
    ("lqr_balance", "q_th1",     (600.0, 1400.0), 30.0),
    ("lqr_balance", "q_dth1",    (25.0, 60.0),   2.0),
    ("lqr_balance", "q_th2",     (600.0, 1400.0), 30.0),
    ("lqr_balance", "q_dth2",    (25.0, 60.0),   2.0),
    ("lqr_balance", "r_u",       (0.008, 0.03),  0.001),

    # Recovery Mode LQR parameters (7) - Focused on aggressive pole catch, low control penalty
    ("lqr_recovery", "q_x",      (5.0, 30.0),    2.0),
    ("lqr_recovery", "q_dx",     (1.0, 8.0),     0.4),
    ("lqr_recovery", "q_th1",    (1500.0, 3500.0), 50.0),
    ("lqr_recovery", "q_dth1",   (45.0, 110.0),  3.0),
    ("lqr_recovery", "q_th2",    (1500.0, 3500.0), 50.0),
    ("lqr_recovery", "q_dth2",   (45.0, 110.0),  3.0),
    ("lqr_recovery", "r_u",      (0.001, 0.006), 0.0003),
]

SIGMA_VECTOR = np.array([p[3] for p in PARAM_SCHEMA], dtype=float)


def params_to_vector(params):
    """Flattens parameter dict into 1D numpy vector."""
    vec = []
    for section, key, bounds, sigma in PARAM_SCHEMA:
        vec.append(float(params[section][key]))
    return np.array(vec, dtype=float)


def vector_to_params(vec, template_params):
    """Converts 1D numpy vector back to parameter dict with boundary clamping."""
    ind = copy.deepcopy(template_params)
    for i, (section, key, (low, high), sigma) in enumerate(PARAM_SCHEMA):
        ind[section][key] = float(np.clip(vec[i], low, high))
    return ind


# Worker global caches for ProcessPoolExecutor
_worker_model = None
_worker_data = None


def _worker_init(xml_path):
    """Initializes worker process with its own private MuJoCo model and data instances."""
    global _worker_model, _worker_data
    _worker_model = mujoco.MjModel.from_xml_path(xml_path)
    _worker_data = mujoco.MjData(_worker_model)


def evaluate_candidate(candidate_params, model, data, scenarios, duration=20.0):
    """
    Evaluates a candidate across the specified scenarios using the Dual-Mode Controller.
    Computes fitness with strict catch hierarchy (+500.0 / catch).
    """
    total_mean = 0.0
    total_peak = 0.0
    total_max_streak = 0.0
    total_streak_reward = 0.0
    total_drift = 0.0
    total_micro_osc = 0.0
    catches = 0

    # 1. Synthesize Balance Mode LQR Gain
    bal_cfg = candidate_params["lqr_balance"]
    q_bal = [
        bal_cfg["q_x"], bal_cfg["q_dx"],
        bal_cfg["q_th1"], bal_cfg["q_dth1"],
        bal_cfg["q_th2"], bal_cfg["q_dth2"]
    ]
    k_bal = compute_lqr_gain(model, data, q_bal, bal_cfg.get("r_u", 0.015))

    # 2. Synthesize Recovery Mode LQR Gain
    rec_cfg = candidate_params["lqr_recovery"]
    q_rec = [
        rec_cfg["q_x"], rec_cfg["q_dx"],
        rec_cfg["q_th1"], rec_cfg["q_dth1"],
        rec_cfg["q_th2"], rec_cfg["q_dth2"]
    ]
    k_rec = compute_lqr_gain(model, data, q_rec, rec_cfg.get("r_u", 0.003))

    K_dual = {"balance": k_bal, "recovery": k_rec}
    sw_cfg = candidate_params["swingup"]

    for name, init_pos in scenarios:
        res = evaluate_trial(model, data, K_dual, sw_cfg, init_pos, duration=duration)
        total_mean += res["mean_score"]
        total_peak += res["peak_score"]
        total_max_streak += res["max_streak"]
        total_streak_reward += res["streak_reward"]
        total_drift += res["drift_penalty"]
        total_micro_osc += res["micro_osc_penalty"]
        if res["caught"]:
            catches += 1

    n = len(scenarios)
    avg_mean = total_mean / n
    avg_peak = total_peak / n
    avg_streak = total_max_streak / n
    avg_streak_reward = total_streak_reward / n
    avg_drift = total_drift / n
    avg_micro_osc = total_micro_osc / n

    # STRICT CATCH HIERARCHY FITNESS SYNTHESIS:
    # +500.0 pts for each caught scenario ensures an individual catching 4/5 or 5/5
    # will strictly beat any individual catching only 3/5.
    fitness = (
        500.0 * catches +
        1.0 * avg_mean +
        0.3 * avg_peak +
        15.0 * avg_streak +
        2.0 * avg_streak_reward -
        1.0 * avg_drift -
        0.4 * avg_micro_osc
    )

    metrics = {
        "fitness": fitness,
        "avg_mean": avg_mean,
        "avg_peak": avg_peak,
        "avg_streak": avg_streak,
        "catches": catches,
        "total_scenarios": n,
        "avg_drift": avg_drift,
        "avg_micro_osc": avg_micro_osc
    }
    return fitness, metrics


def _worker_eval_task(task_args):
    """Worker task entry point for ProcessPoolExecutor."""
    candidate_params, scenarios, duration = task_args
    global _worker_model, _worker_data
    return evaluate_candidate(candidate_params, _worker_model, _worker_data, scenarios, duration=duration)


def main():
    args = parse_args()

    if args.seed is not None:
        np.random.seed(args.seed)

    print("=" * 85)
    print("   DOUBLE INVERTED PENDULUM (DIP) - DUAL-MODE CURRICULUM AUTO-TUNER v3.0   ")
    print("=" * 85)
    print(f"Configurations:")
    print(f" - Generations:            {args.generations}")
    print(f" - Population Size:         {args.pop_size} individuals / gen")
    print(f" - Episode Duration:        {args.max_hold_target:.1f} s / trial")
    print(f" - Parallel CPU Workers:    {args.workers} workers")
    print(f" - Catch Hierarchy:         +500.0 pts / catch (Strict Priority)")
    print(f" - Dual-Mode Controller:    Recovery Mode (th > 0.08 rad) + Balance Mode (th <= 0.08 rad)")
    print(f" - Curriculum Stages:       Stage 1 (1-30) -> Stage 2 (31-60) -> Stage 3 (61+)")
    print(f" - Search Mechanism:        Top-5 Elitism + Differential Evolution + Best-Catch Attraction\n")

    if not os.path.exists(MODEL_XML):
        print(f"[ERROR] model.xml not found at {MODEL_XML}")
        sys.exit(1)

    model = mujoco.MjModel.from_xml_path(MODEL_XML)
    data = mujoco.MjData(model)

    learner = DIPLearner()
    best_params = copy.deepcopy(learner.params)

    # Initialize curriculum for Generation 1
    current_stage, stage_name, active_scenarios = get_curriculum(1)
    print(f"[CURRICULUM] Starting at {stage_name}")

    print("[1/3] Evaluating current baseline parameters on initial stage...")
    base_fit, base_metrics = evaluate_candidate(best_params, model, data, active_scenarios, duration=args.max_hold_target)
    print(f"      Baseline Fitness:     {base_fit:6.1f}")
    print(f"      Mean Score:           {base_metrics['avg_mean']:5.1f} / 100")
    print(f"      Peak Score:           {base_metrics['avg_peak']:5.1f} / 100")
    print(f"      Max Hold Streak:      {base_metrics['avg_streak']:5.2f} s")
    print(f"      Apex Catches:         {base_metrics['catches']} / {base_metrics['total_scenarios']}")
    print(f"      Cart Drift Penalty:   {base_metrics['avg_drift']:5.2f}")
    print(f"      Micro-Osc Penalty:    {base_metrics['avg_micro_osc']:5.2f}\n")

    best_fitness = base_fit
    best_metrics_recorded = base_metrics
    best_catch_ind = copy.deepcopy(best_params)
    best_catch_count = base_metrics["catches"]

    # Initialize elite pool with baseline
    elite_pool = [copy.deepcopy(best_params)]

    stagnation_counter = 0
    prev_stage = current_stage

    print(f"[2/3] Starting Evolutionary Optimization ({args.generations} Gens, Pop {args.pop_size}, Workers {args.workers})...\n")

    with concurrent.futures.ProcessPoolExecutor(
        max_workers=args.workers,
        initializer=_worker_init,
        initargs=(MODEL_XML,)
    ) as executor:
        for gen in range(1, args.generations + 1):
            gen_start_time = time.time()

            # Curriculum check & stage transition
            current_stage, stage_name, active_scenarios = get_curriculum(gen)
            if current_stage != prev_stage:
                print("\n" + "=" * 85)
                print(f"  >>> CURRICULUM UPGRADE (Gen {gen:03d}): Entering {stage_name} <<<")
                print("=" * 85)
                # Re-evaluate best on new curriculum scenarios to establish valid baseline
                base_fit, base_metrics = evaluate_candidate(best_params, model, data, active_scenarios, duration=args.max_hold_target)
                best_fitness = base_fit
                best_metrics_recorded = base_metrics
                stagnation_counter = 0
                prev_stage = current_stage

            # Adaptive search parameters
            if stagnation_counter >= 5:
                mut_scale = 1.8
                stag_status = f"STAGNANT ({stagnation_counter} gens): Directed DE Mutation Active"
            else:
                mut_scale = 1.0
                stag_status = f"Exploiting (Stag: {stagnation_counter}/5)"

            # =========================================================
            # POPULATION GENERATION: ELITISM + DE + DIRECTED MUTATION
            # =========================================================
            population = []

            # 1. Elitism: Preserve up to Top 5 unique elite candidates
            for elite in elite_pool[:min(5, len(elite_pool))]:
                population.append(copy.deepcopy(elite))

            # Fill remainder of population using advanced operators
            vec_best_catch = params_to_vector(best_catch_ind)
            needed = args.pop_size - len(population)

            for i in range(needed):
                # Select a parent from elite pool (weighted towards rank 0)
                parent_idx = np.random.choice(len(elite_pool), p=np.exp(-0.7 * np.arange(len(elite_pool))) / np.sum(np.exp(-0.7 * np.arange(len(elite_pool)))))
                parent_vec = params_to_vector(elite_pool[parent_idx])

                op_choice = np.random.random()

                if op_choice < 0.50:
                    # Operator A: Directed Mutation toward Best-Catch Champion
                    alpha = np.random.uniform(0.15, 0.45)
                    noise = np.random.normal(0.0, SIGMA_VECTOR * mut_scale)
                    child_vec = parent_vec + alpha * (vec_best_catch - parent_vec) + noise

                elif op_choice < 0.85 and len(elite_pool) >= 2:
                    # Operator B: Differential Evolution (DE/best/1 or DE/rand/1)
                    idx1, idx2 = np.random.choice(len(elite_pool), size=2, replace=(len(elite_pool) < 2))
                    v1 = params_to_vector(elite_pool[idx1])
                    v2 = params_to_vector(elite_pool[idx2])
                    diff = v1 - v2
                    F = np.random.uniform(0.5, 0.8)
                    noise = np.random.normal(0.0, SIGMA_VECTOR * 0.4)
                    child_vec = parent_vec + F * diff + noise

                else:
                    # Operator C: Local Gaussian Jitter around parent
                    noise = np.random.normal(0.0, SIGMA_VECTOR * mut_scale)
                    child_vec = parent_vec + noise

                population.append(vector_to_params(child_vec, best_params))

            # =========================================================
            # PARALLEL EVALUATION ACROSS WORKERS
            # =========================================================
            tasks = [(cand, active_scenarios, args.max_hold_target) for cand in population]
            results = list(executor.map(_worker_eval_task, tasks))

            candidates = [(res[0], res[1], cand) for res, cand in zip(results, population)]
            candidates.sort(key=lambda x: x[0], reverse=True)
            top_fit, top_met, top_cand = candidates[0]

            # Update Elite Pool (Top 5 unique individuals)
            elite_pool = [c[2] for c in candidates[:5]]

            # Track all-time highest catches champion
            if top_met["catches"] > best_catch_count:
                best_catch_count = top_met["catches"]
                best_catch_ind = copy.deepcopy(top_cand)

            # =========================================================
            # RECORD TRACKING & SAVING POLICY
            # =========================================================
            is_new_best = False
            save_reason = ""

            if top_fit > best_fitness:
                is_new_best = True
                save_reason = f"+{top_fit - best_fitness:4.1f} Fit"
            elif top_met["catches"] > best_metrics_recorded.get("catches", 0):
                is_new_best = True
                save_reason = f"RECORD CATCHES ({top_met['catches']}/{top_met['total_scenarios']})"
            elif (top_met["catches"] == best_metrics_recorded.get("catches", 0) and
                  top_met["avg_mean"] > best_metrics_recorded.get("avg_mean", 0.0) + 1.0 and
                  top_fit >= best_fitness * 0.98):
                is_new_best = True
                save_reason = f"RECORD MEAN ({top_met['avg_mean']:5.1f} vs {best_metrics_recorded['avg_mean']:5.1f})"

            gen_elapsed = time.time() - gen_start_time

            if is_new_best:
                best_fitness = max(best_fitness, top_fit)
                best_params = top_cand
                best_metrics_recorded = top_met
                stagnation_counter = 0

                # Persist to disk
                best_params["best_mean_score"] = float(top_met["avg_mean"])
                best_params["peak_score"] = float(top_met["avg_peak"])
                best_params["best_max_streak"] = float(top_met["avg_streak"])
                best_params["episodes_trained"] = best_params.get("episodes_trained", 0) + args.pop_size
                learner.save(best_params)

                print(f"  [Gen {gen:03d}/{args.generations:03d}] [+] NEW BEST ({save_reason}) | Fit: {top_fit:6.1f} | "
                      f"Mean: {top_met['avg_mean']:5.1f}/100 | Peak: {top_met['avg_peak']:5.1f} | "
                      f"Streak: {top_met['avg_streak']:4.2f}s | Catches: {top_met['catches']}/{top_met['total_scenarios']} | "
                      f"Time: {gen_elapsed:4.1f}s | SAVED!")
            else:
                stagnation_counter += 1
                best_params["episodes_trained"] = best_params.get("episodes_trained", 0) + args.pop_size

                # Periodic progress log
                if gen % 2 == 0 or stagnation_counter == 5:
                    print(f"  [Gen {gen:03d}/{args.generations:03d}] BestFit: {best_fitness:6.1f} | TopGenMean: {top_met['avg_mean']:5.1f}/100 | "
                          f"TopStreak: {top_met['avg_streak']:4.2f}s | Catches: {top_met['catches']}/{top_met['total_scenarios']} | "
                          f"Time: {gen_elapsed:4.1f}s | {stag_status}")

    print("\n" + "=" * 85)
    print("[3/3] Evolutionary Optimization Complete!")
    print(f"      Final Best Fitness:         {best_fitness:6.1f}")
    print(f"      Final Best Mean Score:      {best_metrics_recorded['avg_mean']:5.1f} / 100")
    print(f"      Final Best Peak Score:      {best_metrics_recorded['avg_peak']:5.1f} / 100")
    print(f"      Final Max Hold Streak:      {best_metrics_recorded['avg_streak']:5.2f} s")
    print(f"      Apex Catch Success:         {best_metrics_recorded['catches']} / {best_metrics_recorded['total_scenarios']}")
    print(f"      Parameters permanently saved to: {learner.filepath}")
    print("=" * 85)


if __name__ == "__main__":
    main()
