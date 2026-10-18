"""
Double Inverted Pendulum on a Cart (DIP) - Symbolic Dynamics Derivation & Linearization
Lead Mechatronics & Embedded Control Engineer
"""

import sympy as sp
import numpy as np


def derive_dip_dynamics():
    print("=" * 70)
    print("Deriving Nonlinear Equations of Motion for Double Inverted Pendulum (DIP)")
    print("=" * 70)

    # Time variable
    t = sp.symbols('t')

    # Generalized coordinates as functions of time
    x = sp.Function('x')(t)
    th1 = sp.Function('th1')(t)
    th2 = sp.Function('th2')(t)

    # First and second derivatives
    dx = x.diff(t)
    dth1 = th1.diff(t)
    dth2 = th2.diff(t)
    ddx = dx.diff(t)
    ddth1 = dth1.diff(t)
    ddth2 = dth2.diff(t)

    # Physical parameters (symbols)
    M = sp.symbols('M', positive=True)      # Cart mass (kg)
    m1 = sp.symbols('m1', positive=True)    # Pendulum 1 mass (kg)
    m2 = sp.symbols('m2', positive=True)    # Pendulum 2 mass (kg)
    L1 = sp.symbols('L1', positive=True)    # Pendulum 1 total length (m)
    L2 = sp.symbols('L2', positive=True)    # Pendulum 2 total length (m)
    l1 = sp.symbols('l1', positive=True)    # COM distance for Pendulum 1 (m)
    l2 = sp.symbols('l2', positive=True)    # COM distance for Pendulum 2 (m)
    J1 = sp.symbols('J1', positive=True)    # Inertia of Pendulum 1 about COM (kg*m^2)
    J2 = sp.symbols('J2', positive=True)    # Inertia of Pendulum 2 about COM (kg*m^2)
    g = sp.symbols('g', positive=True)      # Gravitational acceleration (m/s^2)
    b = sp.symbols('b', nonnegative=True)   # Cart track viscous friction (N*s/m)
    d1 = sp.symbols('d1', nonnegative=True) # Joint 1 rotational friction (N*m*s/rad)
    d2 = sp.symbols('d2', nonnegative=True) # Joint 2 rotational friction (N*m*s/rad)
    u = sp.symbols('u')                     # Control force applied to cart (N)

    # Kinematics: Positions of Cart, COM 1, Joint 2, COM 2
    # Coordinate system: x horizontal right (+), y vertical upwards (+)
    # Angle theta measured from upright vertical (+ counter-clockwise)
    x_c = x
    y_c = sp.S(0)

    # COM 1
    x1 = x - l1 * sp.sin(th1)
    y1 = l1 * sp.cos(th1)

    # Joint 2
    x_j2 = x - L1 * sp.sin(th1)
    y_j2 = L1 * sp.cos(th1)

    # COM 2
    x2 = x - L1 * sp.sin(th1) - l2 * sp.sin(th2)
    y2 = L1 * sp.cos(th1) + l2 * sp.cos(th2)

    # Velocities
    dx_c = dx
    dy_c = sp.S(0)

    dx1 = x1.diff(t)
    dy1 = y1.diff(t)

    dx2 = x2.diff(t)
    dy2 = y2.diff(t)

    # Kinetic Energy (T)
    T_cart = sp.Rational(1, 2) * M * (dx_c**2 + dy_c**2)
    T_p1 = sp.Rational(1, 2) * m1 * (dx1**2 + dy1**2) + sp.Rational(1, 2) * J1 * dth1**2
    T_p2 = sp.Rational(1, 2) * m2 * (dx2**2 + dy2**2) + sp.Rational(1, 2) * J2 * dth2**2
    T = sp.simplify(T_cart + T_p1 + T_p2)

    # Potential Energy (V) (reference datum at y = 0)
    V_p1 = m1 * g * y1
    V_p2 = m2 * g * y2
    V = sp.simplify(V_p1 + V_p2)

    # Lagrangian (L = T - V)
    L = T - V

    # Generalized coordinates and velocities
    q = sp.Matrix([x, th1, th2])
    dq = sp.Matrix([dx, dth1, dth2])
    ddq = sp.Matrix([ddx, ddth1, ddth2])

    # Non-conservative Generalized Forces Q
    # Relative damping at joint 2 is d2 * (dth2 - dth1)
    Q = sp.Matrix([
        u - b * dx,
        - d1 * dth1 + d2 * (dth2 - dth1),
        - d2 * (dth2 - dth1)
    ])

    # Euler-Lagrange equations: d/dt(dL/d(dq_i)) - dL/dq_i = Q_i
    print("Evaluating Euler-Lagrange equations...")
    EOM = []
    for i in range(3):
        dL_ddq_i = L.diff(dq[i])
        d_dt_dL_ddq_i = dL_ddq_i.diff(t)
        dL_dq_i = L.diff(q[i])
        eq = d_dt_dL_ddq_i - dL_dq_i - Q[i]
        EOM.append(sp.simplify(eq))

    EOM = sp.Matrix(EOM)

    # Extract Inertia/Mass Matrix M_mat(q) such that M_mat(q)*ddq + F_rem(q, dq) = B*u
    M_mat = EOM.jacobian(ddq)
    # Remaining terms including Coriolis, gravity, damping
    F_rem = sp.simplify(EOM - M_mat * ddq)

    print("Mass Matrix M(q) size:", M_mat.shape)

    # --- Linearization about upright equilibrium ---
    # Equilibrium point: x=0, th1=0, th2=0, dx=0, dth1=0, dth2=0, u=0
    # Small angle approximations: sin(th) -> th, cos(th) -> 1, dth^2 -> 0, etc.
    print("\nLinearizing around vertical equilibrium [x=0, th1=0, th2=0]...")

    # Dummy static symbols for replacement
    x_s, th1_s, th2_s = sp.symbols('x_s th1_s th2_s')
    dx_s, dth1_s, dth2_s = sp.symbols('dx_s dth1_s dth2_s')
    ddx_s, ddth1_s, ddth2_s = sp.symbols('ddx_s ddth1_s ddth2_s')

    sub_dict = {
        x: x_s, th1: th1_s, th2: th2_s,
        dx: dx_s, dth1: dth1_s, dth2: dth2_s,
        ddx: ddx_s, ddth1: ddth1_s, ddth2: ddth2_s
    }

    EOM_static = EOM.subs(sub_dict)

    # Linearization using 1st-order Taylor series around zero
    vars_all = sp.Matrix([x_s, dx_s, th1_s, dth1_s, th2_s, dth2_s, ddx_s, ddth1_s, ddth2_s, u])
    eq_pt = {v: 0 for v in vars_all}

    # Jacobian w.r.t ddq, dq, q, u
    ddq_s = sp.Matrix([ddx_s, ddth1_s, ddth2_s])
    dq_s = sp.Matrix([dx_s, dth1_s, dth2_s])
    q_s = sp.Matrix([x_s, th1_s, th2_s])

    M0 = EOM_static.jacobian(ddq_s).subs(eq_pt)
    D0 = EOM_static.jacobian(dq_s).subs(eq_pt)
    K0 = EOM_static.jacobian(q_s).subs(eq_pt)
    Bu0 = -EOM_static.jacobian(sp.Matrix([u])).subs(eq_pt)

    # Linearized second-order ODE: M0 * ddq + D0 * dq + K0 * q = Bu0 * u
    # => ddq = - M0^(-1) * K0 * q - M0^(-1) * D0 * dq + M0^(-1) * Bu0 * u
    M0_inv = M0.inv()

    # State vector: z = [x, dx, th1, dth1, th2, dth2]^T
    # [dx]     [ 0   1   0   0   0   0 ] [x]     [ 0 ]
    # [ddx]    [                       ] [dx]    [   ]
    # [dth1] = [ 0   0   0   1   0   0 ] [th1] + [ 0 ] * u
    # [ddth1]  [                       ] [dth1]  [   ]
    # [dth2]   [ 0   0   0   0   0   1 ] [th2]   [ 0 ]
    # [ddth2]  [                       ] [dth2]  [   ]

    A_ddq_q = - M0_inv * K0
    A_ddq_dq = - M0_inv * D0
    B_ddq = M0_inv * Bu0

    # Build 6x6 A matrix and 6x1 B matrix
    A_sym = sp.zeros(6, 6)
    B_sym = sp.zeros(6, 1)

    # Row 0: x_dot = dx
    A_sym[0, 1] = 1

    # Row 1: x_ddot
    A_sym[1, 0] = A_ddq_q[0, 0]
    A_sym[1, 1] = A_ddq_dq[0, 0]
    A_sym[1, 2] = A_ddq_q[0, 1]
    A_sym[1, 3] = A_ddq_dq[0, 1]
    A_sym[1, 4] = A_ddq_q[0, 2]
    A_sym[1, 5] = A_ddq_dq[0, 2]
    B_sym[1, 0] = B_ddq[0, 0]

    # Row 2: th1_dot = dth1
    A_sym[2, 3] = 1

    # Row 3: th1_ddot
    A_sym[3, 0] = A_ddq_q[1, 0]
    A_sym[3, 1] = A_ddq_dq[1, 0]
    A_sym[3, 2] = A_ddq_q[1, 1]
    A_sym[3, 3] = A_ddq_dq[1, 1]
    A_sym[3, 4] = A_ddq_q[1, 2]
    A_sym[3, 5] = A_ddq_dq[1, 2]
    B_sym[3, 0] = B_ddq[1, 0]

    # Row 4: th2_dot = dth2
    A_sym[4, 5] = 1

    # Row 5: th2_ddot
    A_sym[5, 0] = A_ddq_q[2, 0]
    A_sym[5, 1] = A_ddq_dq[2, 0]
    A_sym[5, 2] = A_ddq_q[2, 1]
    A_sym[5, 3] = A_ddq_dq[2, 1]
    A_sym[5, 4] = A_ddq_q[2, 2]
    A_sym[5, 5] = A_ddq_dq[2, 2]
    B_sym[5, 0] = B_ddq[2, 0]

    print("\nSymbolic Linearization Complete.")
    return {
        'M_mat': M_mat,
        'F_rem': F_rem,
        'M0': M0,
        'D0': D0,
        'K0': K0,
        'A_sym': A_sym,
        'B_sym': B_sym,
        'symbols': {
            'M': M, 'm1': m1, 'm2': m2,
            'L1': L1, 'L2': L2, 'l1': l1, 'l2': l2,
            'J1': J1, 'J2': J2, 'g': g,
            'b': b, 'd1': d1, 'd2': d2
        }
    }


def get_default_parameters():
    """Default benchmark physical parameters of experimental DIP testbench."""
    L1 = 0.30       # Pendulum 1 total length (m)
    L2 = 0.25       # Pendulum 2 total length (m)
    m1 = 0.15       # Pendulum 1 mass (kg)
    m2 = 0.10       # Pendulum 2 mass (kg)

    return {
        'M': 1.2,                                # Cart mass (kg)
        'm1': m1,                                # Pendulum 1 mass (kg)
        'm2': m2,                                # Pendulum 2 mass (kg)
        'L1': L1,                                # Pendulum 1 length (m)
        'L2': L2,                                # Pendulum 2 length (m)
        'l1': L1 / 2.0,                          # COM distance 1 (m)
        'l2': L2 / 2.0,                          # COM distance 2 (m)
        'J1': (1.0 / 12.0) * m1 * (L1 ** 2),     # Inertia 1 (kg*m^2)
        'J2': (1.0 / 12.0) * m2 * (L2 ** 2),     # Inertia 2 (kg*m^2)
        'g': 9.81,                               # Gravity (m/s^2)
        'b': 0.5,                                # Cart track viscous damping (N*s/m)
        'd1': 0.005,                             # Joint 1 damping (N*m*s/rad)
        'd2': 0.003                              # Joint 2 damping (N*m*s/rad)
    }


def compute_numerical_matrices(params=None):
    """Derives and evaluates numerical A (6x6) and B (6x1) matrices."""
    if params is None:
        params = get_default_parameters()

    res = derive_dip_dynamics()
    sym_dict = res['symbols']

    val_map = {
        sym_dict['M']: params['M'],
        sym_dict['m1']: params['m1'],
        sym_dict['m2']: params['m2'],
        sym_dict['L1']: params['L1'],
        sym_dict['L2']: params['L2'],
        sym_dict['l1']: params['l1'],
        sym_dict['l2']: params['l2'],
        sym_dict['J1']: params['J1'],
        sym_dict['J2']: params['J2'],
        sym_dict['g']: params['g'],
        sym_dict['b']: params['b'],
        sym_dict['d1']: params['d1'],
        sym_dict['d2']: params['d2']
    }

    A_num = np.array(res['A_sym'].subs(val_map)).astype(np.float64)
    B_num = np.array(res['B_sym'].subs(val_map)).astype(np.float64)

    return A_num, B_num


if __name__ == '__main__':
    A, B = compute_numerical_matrices()
    np.set_printoptions(precision=4, suppress=True)
    print("\n--- Numerical State-Space Matrix A (6x6) ---")
    print(A)
    print("\n--- Numerical Input Matrix B (6x1) ---")
    print(B)
