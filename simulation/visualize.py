"""
Double Inverted Pendulum on a Cart (DIP) - Time Response Plotting & Animation
Lead Mechatronics & Embedded Control Engineer
"""

import os
import csv
import math


def load_simulation_csv(csv_path="simulation/output_trajectory.csv"):
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Trajectory file not found: {csv_path}")

    data = {
        'time': [], 'x': [], 'dx': [],
        'th1_deg': [], 'dth1_deg': [],
        'th2_deg': [], 'dth2_deg': [],
        'u': [], 'f_disturb': []
    }

    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            data['time'].append(float(row['time']))
            data['x'].append(float(row['x']))
            data['dx'].append(float(row['dx']))
            data['th1_deg'].append(math.degrees(float(row['th1'])))
            data['dth1_deg'].append(math.degrees(float(row['dth1'])))
            data['th2_deg'].append(math.degrees(float(row['th2'])))
            data['dth2_deg'].append(math.degrees(float(row['dth2'])))
            data['u'].append(float(row['u']))
            data['f_disturb'].append(float(row['f_disturb']))

    return data


def plot_responses(csv_path="simulation/output_trajectory.csv", save_path="simulation/dip_response.png"):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib is not installed. To generate plots, run: pip install matplotlib")
        return

    data = load_simulation_csv(csv_path)
    t = data['time']

    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle("Double Inverted Pendulum (DIP) - Closed-Loop LQR Stabilization", fontsize=14, fontweight='bold')

    # 1. Cart Position
    axs[0].plot(t, data['x'], 'b-', linewidth=1.5, label='Cart Position x(t)')
    axs[0].axhline(0, color='gray', linestyle='--', alpha=0.6)
    axs[0].set_ylabel("x [m]")
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper right')

    # 2. Pendulum Angles
    axs[1].plot(t, data['th1_deg'], 'r-', linewidth=1.5, label=r'$\theta_1$ (Link 1)')
    axs[1].plot(t, data['th2_deg'], 'g--', linewidth=1.5, label=r'$\theta_2$ (Link 2)')
    axs[1].axhline(0, color='gray', linestyle='--', alpha=0.6)
    axs[1].set_ylabel(r"Angle [$^\circ$]")
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='upper right')

    # 3. Angular Velocities
    axs[2].plot(t, data['dth1_deg'], 'r-', linewidth=1.2, label=r'$\dot{\theta}_1$ [deg/s]')
    axs[2].plot(t, data['dth2_deg'], 'g--', linewidth=1.2, label=r'$\dot{\theta}_2$ [deg/s]')
    axs[2].axhline(0, color='gray', linestyle='--', alpha=0.6)
    axs[2].set_ylabel(r"Rate [$^\circ$/s]")
    axs[2].grid(True, linestyle=':', alpha=0.6)
    axs[2].legend(loc='upper right')

    # 4. Control Force u(t) & Disturbance
    axs[3].plot(t, data['u'], 'm-', linewidth=1.5, label='Control Force u(t) [N]')
    axs[3].plot(t, data['f_disturb'], 'k:', linewidth=1.5, label='Impulse Disturbance [N]')
    axs[3].axhline(25.0, color='red', linestyle=':', alpha=0.7, label=r'Saturation Clamp $\pm 25$N')
    axs[3].axhline(-25.0, color='red', linestyle=':', alpha=0.7)
    axs[3].set_xlabel("Time [s]")
    axs[3].set_ylabel("Force [N]")
    axs[3].grid(True, linestyle=':', alpha=0.6)
    axs[3].legend(loc='upper right')

    plt.tight_layout()
    os.makedirs(os.path.dirname(os.path.abspath(save_path)), exist_ok=True)
    plt.savefig(save_path, dpi=300)
    print(f"Response plot successfully saved to: {save_path}")


def print_summary_metrics(csv_path="simulation/output_trajectory.csv"):
    data = load_simulation_csv(csv_path)
    n = len(data['time'])
    final_idx = n - 1

    max_x = max(abs(x) for x in data['x'])
    max_th1 = max(abs(th) for th in data['th1_deg'])
    max_th2 = max(abs(th) for th in data['th2_deg'])
    max_u = max(abs(u) for u in data['u'])

    print("\n" + "=" * 60)
    print("               DIP PERFORMANCE SUMMARY REPORT               ")
    print("=" * 60)
    print(f"Total Simulation Time  : {data['time'][final_idx]:.2f} s")
    print(f"Max Cart Displacement  : {max_x:.4f} m")
    print(f"Max Pole 1 Deviation   : {max_th1:.2f} deg")
    print(f"Max Pole 2 Deviation   : {max_th2:.2f} deg")
    print(f"Max Actuator Force     : {max_u:.2f} N (Limit: 25.0 N)")
    print(f"Final Steady-State x   : {data['x'][final_idx]:.4f} m")
    print(f"Final Pole 1 Angle     : {data['th1_deg'][final_idx]:.4f} deg")
    print(f"Final Pole 2 Angle     : {data['th2_deg'][final_idx]:.4f} deg")
    print("=" * 60 + "\n")


if __name__ == '__main__':
    csv_file = "simulation/output_trajectory.csv"
    if os.path.exists(csv_file):
        print_summary_metrics(csv_file)
        plot_responses(csv_file)
    else:
        print(f"Run 'python simulation/nonlinear_sim.py' first to generate {csv_file}")
