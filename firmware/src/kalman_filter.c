/**
 * @file kalman_filter.c
 * @brief Discrete 1-D Kalman Filter implementation for encoder velocity estimation
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#include "kalman_filter.h"
#include <math.h>

void KalmanFilter_Init(KalmanFilter1D_t *kf, float initial_angle, float q_pos, float q_vel, float r_meas, float dt) {
    if (!kf) return;

    kf->x[0] = initial_angle;
    kf->x[1] = 0.0f;

    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;

    kf->Q[0] = q_pos;
    kf->Q[1] = q_vel;
    kf->R = r_meas;
    kf->dt = dt;
}

void KalmanFilter_Update(KalmanFilter1D_t *kf, float z_measured_angle) {
    if (!kf) return;

    const float dt = kf->dt;

    /* ---------------------------------------------------------------------- */
    /* 1. Time Update (Prediction Step)                                       */
    /* State Extrapolation: x_k|k-1 = F * x_k-1|k-1                           */
    /* F = [1, dt; 0, 1]                                                      */
    /* ---------------------------------------------------------------------- */
    const float x0_pred = kf->x[0] + dt * kf->x[1];
    const float x1_pred = kf->x[1];

    /* Covariance Extrapolation: P_k|k-1 = F * P_k-1|k-1 * F^T + Q            */
    const float P00 = kf->P[0][0];
    const float P01 = kf->P[0][1];
    const float P10 = kf->P[1][0];
    const float P11 = kf->P[1][1];

    const float P00_pred = P00 + dt * (P10 + P01) + (dt * dt) * P11 + kf->Q[0];
    const float P01_pred = P01 + dt * P11;
    const float P10_pred = P10 + dt * P11;
    const float P11_pred = P11 + kf->Q[1];

    /* ---------------------------------------------------------------------- */
    /* 2. Measurement Update (Correction Step)                                */
    /* Measurement matrix: H = [1, 0]                                         */
    /* Innovation: y = z - H * x_pred                                         */
    /* ---------------------------------------------------------------------- */
    /* Handle angular phase wrapping if necessary: -pi to +pi difference */
    float y = z_measured_angle - x0_pred;
    while (y > 3.14159265f)  y -= 6.2831853f;
    while (y < -3.14159265f) y += 6.2831853f;

    /* Innovation covariance: S = H * P_pred * H^T + R = P00_pred + R         */
    const float S = P00_pred + kf->R;
    const float inv_S = 1.0f / S;

    /* Kalman Gain: K = P_pred * H^T / S = [P00_pred/S; P10_pred/S]           */
    const float K0 = P00_pred * inv_S;
    const float K1 = P10_pred * inv_S;

    /* Updated state estimate: x = x_pred + K * y                             */
    kf->x[0] = x0_pred + K0 * y;
    kf->x[1] = x1_pred + K1 * y;

    /* Updated error covariance: P = (I - K * H) * P_pred                     */
    kf->P[0][0] = P00_pred - K0 * P00_pred;
    kf->P[0][1] = P01_pred - K0 * P01_pred;
    kf->P[1][0] = P10_pred - K1 * P00_pred;
    kf->P[1][1] = P11_pred - K1 * P01_pred;
}
