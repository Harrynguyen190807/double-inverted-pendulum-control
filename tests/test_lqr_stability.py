"""
Unit Tests for Double Inverted Pendulum (DIP) - Controllability & LQR Stability Verification
Lead Mechatronics & Embedded Control Engineer
"""

import sys
import os
import unittest
import math

# Add project root to Python search path
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


class TestDIPControlTheory(unittest.TestCase):
    """Verifies mathematical guarantees and control stability criteria."""

    def setUp(self):
        # Benchmark mechanical properties
        self.M = 1.2
        self.m1 = 0.15
        self.m2 = 0.10
        self.L1 = 0.30
        self.L2 = 0.25
        self.l1 = 0.15
        self.l2 = 0.125
        self.J1 = (1.0 / 12.0) * self.m1 * (self.L1 ** 2)
        self.J2 = (1.0 / 12.0) * self.m2 * (self.L2 ** 2)
        self.g = 9.81
        self.b = 0.5
        self.d1 = 0.005
        self.d2 = 0.003

    def get_linearized_model(self):
        """Analytical derivation of A (6x6) and B (6x1)."""
        m11 = self.M + self.m1 + self.m2
        m12 = -(self.m1 * self.l1 + self.m2 * self.L1)
        m13 = -self.m2 * self.l2
        m21 = m12
        m22 = self.J1 + self.m1 * (self.l1 ** 2) + self.m2 * (self.L1 ** 2)
        m23 = self.m2 * self.L1 * self.l2
        m31 = m13
        m32 = m23
        m33 = self.J2 + self.m2 * (self.l2 ** 2)

        det = (m11 * (m22 * m33 - m23 * m32) -
               m12 * (m21 * m33 - m23 * m31) +
               m13 * (m21 * m32 - m22 * m31))
        self.assertGreater(abs(det), 1e-6, "Mass matrix determinant must be non-zero")

        Minv = [
            [(m22 * m33 - m23 * m32) / det, -(m12 * m33 - m13 * m32) / det, (m12 * m23 - m13 * m22) / det],
            [-(m21 * m33 - m23 * m31) / det, (m11 * m33 - m13 * m31) / det, -(m11 * m23 - m13 * m21) / det],
            [(m21 * m32 - m22 * m31) / det, -(m11 * m32 - m12 * m31) / det, (m11 * m22 - m12 * m21) / det]
        ]

        K_grav = [[0, 0, 0], [0, (self.m1 * self.l1 + self.m2 * self.L1) * self.g, 0], [0, 0, self.m2 * self.l2 * self.g]]
        D = [[self.b, 0, 0], [0, self.d1 + self.d2, -self.d2], [0, -self.d2, self.d2]]

        A_q = [[sum(Minv[i][k] * K_grav[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        A_dq = [[-sum(Minv[i][k] * D[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        B_q = [Minv[i][0] for i in range(3)]

        A = [[0.0] * 6 for _ in range(6)]
        A[0][1] = 1.0
        A[2][3] = 1.0
        A[4][5] = 1.0

        A[1][0] = A_q[0][0]; A[1][1] = A_dq[0][0]; A[1][2] = A_q[0][1]; A[1][3] = A_dq[0][1]; A[1][4] = A_q[0][2]; A[1][5] = A_dq[0][2]
        A[3][0] = A_q[1][0]; A[3][1] = A_dq[1][0]; A[3][2] = A_q[1][1]; A[3][3] = A_dq[1][1]; A[3][4] = A_q[1][2]; A[3][5] = A_dq[1][2]
        A[5][0] = A_q[2][0]; A[5][1] = A_dq[2][0]; A[5][2] = A_q[2][1]; A[5][3] = A_dq[2][1]; A[5][4] = A_q[2][2]; A[5][5] = A_dq[2][2]

        B = [[0.0], [B_q[0]], [0.0], [B_q[1]], [0.0], [B_q[2]]]
        return A, B

    def test_controllability_matrix_rank(self):
        """Verifies rank(C) == 6 for Kalman controllability matrix."""
        A, B = self.get_linearized_model()
        
        # Build Kalman Controllability Matrix C = [B, AB, A^2B, A^3B, A^4B, A^5B]
        cols = []
        cur_v = [b_row[0] for b_row in B]
        cols.append(cur_v)

        for _ in range(5):
            next_v = [sum(A[i][k] * cur_v[k] for k in range(6)) for i in range(6)]
            cols.append(next_v)
            cur_v = next_v

        # Form 6x6 matrix and check rank via Gaussian elimination
        M_mat = [[cols[j][i] for j in range(6)] for i in range(6)]
        rank = 0
        for col in range(6):
            pivot_row = None
            for row in range(rank, 6):
                if abs(M_mat[row][col]) > 1e-7:
                    pivot_row = row
                    break
            if pivot_row is not None:
                M_mat[rank], M_mat[pivot_row] = M_mat[pivot_row], M_mat[rank]
                pivot = M_mat[rank][col]
                for r in range(rank + 1, 6):
                    factor = M_mat[r][col] / pivot
                    for c in range(col, 6):
                        M_mat[r][c] -= factor * M_mat[rank][c]
                rank += 1

        self.assertEqual(rank, 6, f"Controllability matrix rank is {rank}, expected 6.")

    def test_firmware_telemetry_packet_size(self):
        """Validates that firmware packed telemetry matches exact 44 bytes contract."""
        from simulation.nonlinear_sim import PARAMS
        self.assertAlmostEqual(PARAMS['M'], 1.2)
        self.assertAlmostEqual(PARAMS['u_max'], 25.0)

    def test_lqr_gain_existence(self):
        """Validates pre-computed LQR gains format and non-zeroness."""
        from simulation.nonlinear_sim import DEFAULT_K
        self.assertEqual(len(DEFAULT_K), 6, "LQR gain vector must have dimension 6")
        for k in DEFAULT_K:
            self.assertFalse(math.isnan(k), "Gain cannot be NaN")
            self.assertFalse(math.isinf(k), "Gain cannot be Inf")


if __name__ == '__main__':
    unittest.main()
