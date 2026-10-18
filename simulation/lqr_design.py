"""
Double Inverted Pendulum on a Cart (DIP) - LQR Controller Design & Controllability Analysis
Lead Mechatronics & Embedded Control Engineer
"""

import numpy as np
from scipy.linalg import solve_continuous_are


class DIPDynamics:
    """Analytical linearized model for fast evaluation and LQR design."""
    def __init__(self, M=1.2, m1=0.15, m2=0.10, L1=0.30, L2=0.25,
                 b=0.5, d1=0.005, d2=0.003, g=9.81):
        self.M = M
        self.m1 = m1
        self.m2 = m2
        self.L1 = L1
        self.L2 = L2
        self.l1 = L1 / 2.0
        self.l2 = L2 / 2.0
        self.J1 = (1.0 / 12.0) * m1 * (L1 ** 2)
        self.J2 = (1.0 / 12.0) * m2 * (L2 ** 2)
        self.b = b
        self.d1 = d1
        self.d2 = d2
        self.g = g

    def get_linearized_matrices(self):
        """
        Computes linearized mass matrix M0, damping matrix D0, stiffness matrix K0,
        and input vector Bu0 around vertical equilibrium [x=0, th1=0, th2=0].
        
        Lagrangian formulation leads to:
            M0 * ddq + D0 * dq + K0 * q = Bu0 * u
            where q = [x, th1, th2]^T
        """
        # Mass matrix M0 at upright equilibrium:
        # T = 0.5*M*dx^2 + 0.5*m1*(dx - l1*dth1)^2 + 0.5*J1*dth1^2
        #     + 0.5*m2*(dx - L1*dth1 - l2*dth2)^2 + 0.5*J2*dth2^2
        m11 = self.M + self.m1 + self.m2
        m12 = -(self.m1 * self.l1 + self.m2 * self.L1)
        m13 = -self.m2 * self.l2

        m21 = m12
        m22 = self.J1 + self.m1 * (self.l1 ** 2) + self.m2 * (self.L1 ** 2)
        m23 = self.m2 * self.L1 * self.l2

        m31 = m13
        m32 = m23
        m33 = self.J2 + self.m2 * (self.l2 ** 2)

        M0 = np.array([
            [m11, m12, m13],
            [m21, m22, m23],
            [m31, m32, m33]
        ], dtype=np.float64)

        # Damping matrix D0:
        # Q = [u - b*dx, -d1*dth1 + d2*(dth2 - dth1), -d2*(dth2 - dth1)]
        D0 = np.array([
            [self.b, 0.0, 0.0],
            [0.0, self.d1 + self.d2, -self.d2],
            [0.0, -self.d2, self.d2]
        ], dtype=np.float64)

        # Stiffness matrix K0 (from potential energy V = m1*g*l1*cos(th1) + m2*g*(L1*cos(th1) + l2*cos(th2)))
        # -dV/dq evaluated at th=0 gives restoring/destabilizing gravity terms:
        # dV/dth1 = -(m1*l1 + m2*L1)*g*sin(th1) -> approx -(m1*l1 + m2*L1)*g * th1
        # In Euler-Lagrange, d/dt(dL/ddq) - dL/dq = Q => M0*ddq + ... - dV/dq = Q
        # So K0 matrix satisfies:
        k22 = -(self.m1 * self.l1 + self.m2 * self.L1) * self.g
        k33 = -self.m2 * self.l2 * self.g

        K0 = np.array([
            [0.0, 0.0, 0.0],
            [0.0, k22, 0.0],
            [0.0, 0.0, k33]
        ], dtype=np.float64)

        Bu0 = np.array([
            [1.0],
            [0.0],
            [0.0]
        ], dtype=np.float64)

        # State space: z = [x, dx, th1, dth1, th2, dth2]^T
        M0_inv = np.linalg.inv(M0)
        A_ddq_q = -M0_inv @ K0
        A_ddq_dq = -M0_inv @ D0
        B_ddq = M0_inv @ Bu0

        A = np.zeros((6, 6), dtype=np.float64)
        B = np.zeros((6, 1), dtype=np.float64)

        # Row 0: dx/dt = dx
        A[0, 1] = 1.0

        # Row 1: ddx/dt
        A[1, 0] = A_ddq_q[0, 0]
        A[1, 1] = A_ddq_dq[0, 0]
        A[1, 2] = A_ddq_q[0, 1]
        A[1, 3] = A_ddq_dq[0, 1]
        A[1, 4] = A_ddq_q[0, 2]
        A[1, 5] = A_ddq_dq[0, 2]
        B[1, 0] = B_ddq[0, 0]

        # Row 2: dth1/dt = dth1
        A[2, 3] = 1.0

        # Row 3: ddth1/dt
        A[3, 0] = A_ddq_q[1, 0]
        A[3, 1] = A_ddq_dq[1, 0]
        A[3, 2] = A_ddq_q[1, 1]
        A[3, 3] = A_ddq_dq[1, 1]
        A[3, 4] = A_ddq_q[1, 2]
        A[3, 5] = A_ddq_dq[1, 2]
        B[3, 0] = B_ddq[1, 0]

        # Row 4: dth2/dt = dth2
        A[4, 5] = 1.0

        # Row 5: ddth2/dt
        A[5, 0] = A_ddq_q[2, 0]
        A[5, 1] = A_ddq_dq[2, 0]
        A[5, 2] = A_ddq_q[2, 1]
        A[5, 3] = A_ddq_dq[2, 1]
        A[5, 4] = A_ddq_q[2, 2]
        A[5, 5] = A_ddq_dq[2, 2]
        B[5, 0] = B_ddq[2, 0]

        return A, B


def check_controllability(A, B):
    """
    Computes Controllability Matrix:
    C = [B, A*B, A^2*B, A^3*B, A^4*B, A^5*B]
    Returns rank and boolean is_controllable.
    """
    n = A.shape[0]
    C_list = [B]
    M_temp = B.copy()
    for _ in range(1, n):
        M_temp = A @ M_temp
        C_list.append(M_temp)
    C = np.hstack(C_list)
    rank = np.linalg.matrix_rank(C)
    is_controllable = (rank == n)
    return C, rank, is_controllable


def design_lqr_bryson(A, B, max_deviations=None, max_u=20.0):
    """
    Synthesizes LQR Gain Matrix K using Bryson's Rule.
    Cost Function J = integral( z^T Q z + u^T R u ) dt
    Bryson's rule:
        Q_ii = 1 / (max_state_error_i)^2
        R = 1 / (max_u)^2
    """
    if max_deviations is None:
        # Default tuning targets:
        # x: 0.05 m, dx: 0.5 m/s
        # th1: 0.035 rad (~2 deg), dth1: 0.5 rad/s
        # th2: 0.035 rad (~2 deg), dth2: 0.5 rad/s
        max_deviations = [0.05, 0.40, 0.035, 0.40, 0.035, 0.40]

    q_diag = [1.0 / (d ** 2) for d in max_deviations]
    Q = np.diag(q_diag)
    R = np.array([[1.0 / (max_u ** 2)]])

    # Solve Continuous Algebraic Riccati Equation (CARE):
    # A^T P + P A - P B R^(-1) B^T P + Q = 0
    P = solve_continuous_are(A, B, Q, R)

    # Optimal feedback gain K = R^(-1) B^T P
    K = np.linalg.inv(R) @ (B.T @ P)

    # Closed-loop matrix and poles
    A_cl = A - B @ K
    eigvals = np.linalg.eigvals(A_cl)
    is_stable = np.all(np.real(eigvals) < 0)

    return K, P, Q, R, eigvals, is_stable


def export_c_header_gain(K):
    """Formats LQR gain as C #define / const array for firmware."""
    k_vec = K.flatten()
    c_code = (
        "/* Auto-generated LQR Gain Constants for DIP */\n"
        f"#define LQR_K_X       ({k_vec[0]:.6f}f)\n"
        f"#define LQR_K_X_DOT   ({k_vec[1]:.6f}f)\n"
        f"#define LQR_K_TH1     ({k_vec[2]:.6f}f)\n"
        f"#define LQR_K_TH1_DOT ({k_vec[3]:.6f}f)\n"
        f"#define LQR_K_TH2     ({k_vec[4]:.6f}f)\n"
        f"#define LQR_K_TH2_DOT ({k_vec[5]:.6f}f)\n\n"
        f"static const float LQR_GAINS[6] = {{\n"
        f"    {k_vec[0]:.6f}f, {k_vec[1]:.6f}f, {k_vec[2]:.6f}f,\n"
        f"    {k_vec[3]:.6f}f, {k_vec[4]:.6f}f, {k_vec[5]:.6f}f\n"
        f"}};\n"
    )
    return c_code


if __name__ == '__main__':
    dip = DIPDynamics()
    A, B = dip.get_linearized_matrices()
    _, rank, is_ctrl = check_controllability(A, B)
    print(f"Controllability Matrix Rank: {rank} / 6 (Controllable: {is_ctrl})")

    K, P, Q, R, poles, is_stable = design_lqr_bryson(A, B)
    print("\nOptimal LQR Gain Matrix K (u = -K * z):")
    print(f"K = {K}")
    print(f"\nClosed-loop Poles:")
    for p in poles:
        print(f"  {p.real:+.4f} + {p.imag:+.4f}j")
    print(f"System Asymptotically Stable: {is_stable}")

    print("\n" + export_c_header_gain(K))
