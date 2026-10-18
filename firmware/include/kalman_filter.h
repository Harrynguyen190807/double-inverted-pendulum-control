/**
 * @file kalman_filter.h
 * @brief Discrete 1-D Kalman Filter for Smooth Angle & Angular Velocity Estimation
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x[2];      /**< State: [0] = angle (rad), [1] = angular velocity (rad/s) */
    float P[2][2];   /**< Error covariance matrix (2x2) */
    float Q[2];      /**< Process noise variance [Q_pos, Q_vel] */
    float R;         /**< Measurement noise variance */
    float dt;        /**< Sampling time in seconds */
} KalmanFilter1D_t;

/**
 * @brief Initializes 1-D Kalman Filter structure.
 */
void KalmanFilter_Init(KalmanFilter1D_t *kf, float initial_angle, float q_pos, float q_vel, float r_meas, float dt);

/**
 * @brief Executes one predict-update cycle with a new raw angle measurement.
 * @return Filtered state: angle in [0] and clean angular velocity in [1].
 */
void KalmanFilter_Update(KalmanFilter1D_t *kf, float z_measured_angle);

/**
 * @brief Retrieves estimated angle.
 */
static inline float KalmanFilter_GetAngle(const KalmanFilter1D_t *kf) {
    return kf->x[0];
}

/**
 * @brief Retrieves estimated angular velocity.
 */
static inline float KalmanFilter_GetVelocity(const KalmanFilter1D_t *kf) {
    return kf->x[1];
}

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H */
