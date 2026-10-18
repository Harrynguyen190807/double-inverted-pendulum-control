/**
 * @file dip_controller.c
 * @brief Industrial 1 kHz LQR Controller Implementation for Double Inverted Pendulum
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#include "dip_controller.h"
#include <math.h>

/* Zero-reference offsets for AS5048A calibrated upright vertical position */
static const uint16_t AS5048A_ZERO_OFFSET_J1 = 8192U;  /**< Mid-scale default */
static const uint16_t AS5048A_ZERO_OFFSET_J2 = 8192U;

void DIP_Controller_Init(DIP_Controller_t *ctrl) {
    if (!ctrl) return;

    ctrl->x = 0.0f;
    ctrl->x_dot = 0.0f;
    ctrl->th1 = 0.0f;
    ctrl->th1_dot = 0.0f;
    ctrl->th2 = 0.0f;
    ctrl->th2_dot = 0.0f;

    ctrl->u_lqr = 0.0f;
    ctrl->u_friction = 0.0f;
    ctrl->u_command = 0.0f;
    ctrl->pwm_duty = 0.0f;

    ctrl->system_state = DIP_STATE_STANDBY;
    ctrl->status_flags = 0;
    ctrl->loop_counter = 0;

    /* Initialize Kalman Filters */
    /* Cart KF: Q_pos = 1e-5, Q_vel = 1e-2, R_meas = 1e-4 */
    KalmanFilter_Init(&ctrl->kf_cart, 0.0f, 1e-5f, 1e-2f, 1e-4f, CONTROL_DT_SEC);
    /* Pole 1 KF */
    KalmanFilter_Init(&ctrl->kf_th1, 0.0f, KF_PROCESS_NOISE_POS, KF_PROCESS_NOISE_VEL, KF_MEASUREMENT_NOISE, CONTROL_DT_SEC);
    /* Pole 2 KF */
    KalmanFilter_Init(&ctrl->kf_th2, 0.0f, KF_PROCESS_NOISE_POS, KF_PROCESS_NOISE_VEL, KF_MEASUREMENT_NOISE, CONTROL_DT_SEC);
}

void DIP_Controller_TriggerEmergencyStop(DIP_Controller_t *ctrl, uint8_t reason_flag) {
    if (!ctrl) return;

    ctrl->system_state = DIP_STATE_FAILSAFE_TRIP;
    ctrl->status_flags |= reason_flag;
    ctrl->status_flags &= ~STATUS_FLAG_BALANCING;

    ctrl->u_lqr = 0.0f;
    ctrl->u_friction = 0.0f;
    ctrl->u_command = 0.0f;
    ctrl->pwm_duty = 0.0f;
}

float DIP_Controller_Step(DIP_Controller_t *ctrl,
                          int32_t raw_cart_encoder_ticks,
                          uint16_t raw_as5048a_joint1_counts,
                          uint16_t raw_as5048a_joint2_counts) {
    if (!ctrl) return 0.0f;

    ctrl->loop_counter++;

    /* If locked in emergency trip, guarantee 0 PWM output immediately */
    if (ctrl->system_state == DIP_STATE_FAILSAFE_TRIP) {
        ctrl->pwm_duty = 0.0f;
        return 0.0f;
    }

    /* ---------------------------------------------------------------------- */
    /* 1. Sensor Measurement Conversion & Offset Compensation                 */
    /* ---------------------------------------------------------------------- */
    /* Cart position */
    const float raw_cart_pos_m = (float)raw_cart_encoder_ticks * CART_METERS_PER_TICK;

    /* Joint 1 relative to upright zero (wrap to [-pi, +pi]) */
    int32_t diff1 = (int32_t)raw_as5048a_joint1_counts - (int32_t)AS5048A_ZERO_OFFSET_J1;
    if (diff1 > 8192)  diff1 -= 16384;
    if (diff1 < -8192) diff1 += 16384;
    const float raw_th1_rad = (float)diff1 * AS5048A_RAD_PER_COUNT;

    /* Joint 2 relative to upright zero (wrap to [-pi, +pi]) */
    int32_t diff2 = (int32_t)raw_as5048a_joint2_counts - (int32_t)AS5048A_ZERO_OFFSET_J2;
    if (diff2 > 8192)  diff2 -= 16384;
    if (diff2 < -8192) diff2 += 16384;
    const float raw_th2_rad = (float)diff2 * AS5048A_RAD_PER_COUNT;

    /* ---------------------------------------------------------------------- */
    /* 2. Discrete Kalman Filter Estimation (Noise-Free Rates)                 */
    /* ---------------------------------------------------------------------- */
    KalmanFilter_Update(&ctrl->kf_cart, raw_cart_pos_m);
    KalmanFilter_Update(&ctrl->kf_th1, raw_th1_rad);
    KalmanFilter_Update(&ctrl->kf_th2, raw_th2_rad);

    ctrl->x = KalmanFilter_GetAngle(&ctrl->kf_cart);
    ctrl->x_dot = KalmanFilter_GetVelocity(&ctrl->kf_cart);

    ctrl->th1 = KalmanFilter_GetAngle(&ctrl->kf_th1);
    ctrl->th1_dot = KalmanFilter_GetVelocity(&ctrl->kf_th1);

    ctrl->th2 = KalmanFilter_GetAngle(&ctrl->kf_th2);
    ctrl->th2_dot = KalmanFilter_GetVelocity(&ctrl->kf_th2);

    /* ---------------------------------------------------------------------- */
    /* 3. Safety Guard & Emergency Fail-Safe Cutoff Check                     */
    /* ---------------------------------------------------------------------- */
    if (fabsf(ctrl->th1) > FAILSAFE_MAX_ANGLE_RAD || fabsf(ctrl->th2) > FAILSAFE_MAX_ANGLE_RAD) {
        DIP_Controller_TriggerEmergencyStop(ctrl, STATUS_FLAG_FAILSAFE_TH);
        return 0.0f;
    }

    if (fabsf(ctrl->x) > FAILSAFE_MAX_CART_POS_M) {
        DIP_Controller_TriggerEmergencyStop(ctrl, STATUS_FLAG_FAILSAFE_CART);
        return 0.0f;
    }

    ctrl->system_state = DIP_STATE_BALANCING;
    ctrl->status_flags |= STATUS_FLAG_BALANCING;

    /* ---------------------------------------------------------------------- */
    /* 4. Optimal LQR Control Calculation: u = - K * z                        */
    /* ---------------------------------------------------------------------- */
    ctrl->u_lqr = - (LQR_K_X       * ctrl->x +
                     LQR_K_X_DOT   * ctrl->x_dot +
                     LQR_K_TH1     * ctrl->th1 +
                     LQR_K_TH1_DOT * ctrl->th1_dot +
                     LQR_K_TH2     * ctrl->th2 +
                     LQR_K_TH2_DOT * ctrl->th2_dot);

    /* ---------------------------------------------------------------------- */
    /* 5. Coulomb Stiction Feedforward Friction Compensation                  */
    /* ---------------------------------------------------------------------- */
    if (fabsf(ctrl->x_dot) > VELOCITY_DEADBAND_M_S) {
        ctrl->u_friction = (ctrl->x_dot > 0.0f) ? COULOMB_STICTION_FORCE_N : -COULOMB_STICTION_FORCE_N;
    } else {
        /* Static breakaway feedforward aligned with required motion direction */
        if (fabsf(ctrl->u_lqr) > 0.1f) {
            ctrl->u_friction = (ctrl->u_lqr > 0.0f) ? (0.6f * COULOMB_STICTION_FORCE_N) : (-0.6f * COULOMB_STICTION_FORCE_N);
        } else {
            ctrl->u_friction = 0.0f;
        }
    }

    const float u_raw = ctrl->u_lqr + ctrl->u_friction;

    /* ---------------------------------------------------------------------- */
    /* 6. Actuator Saturation Clamp & PWM Output Scaling                      */
    /* ---------------------------------------------------------------------- */
    if (u_raw > ACTUATOR_MAX_FORCE_N) {
        ctrl->u_command = ACTUATOR_MAX_FORCE_N;
        ctrl->status_flags |= STATUS_FLAG_FORCE_SATURATED;
    } else if (u_raw < -ACTUATOR_MAX_FORCE_N) {
        ctrl->u_command = -ACTUATOR_MAX_FORCE_N;
        ctrl->status_flags |= STATUS_FLAG_FORCE_SATURATED;
    } else {
        ctrl->u_command = u_raw;
        ctrl->status_flags &= ~STATUS_FLAG_FORCE_SATURATED;
    }

    /* Map commanded force to PWM percentage [-100.0%, +100.0%] */
    ctrl->pwm_duty = ctrl->u_command * FORCE_TO_PWM_RATIO;
    if (ctrl->pwm_duty > ACTUATOR_PWM_MAX_DUTY)  ctrl->pwm_duty = ACTUATOR_PWM_MAX_DUTY;
    if (ctrl->pwm_duty < -ACTUATOR_PWM_MAX_DUTY) ctrl->pwm_duty = -ACTUATOR_PWM_MAX_DUTY;

    return ctrl->pwm_duty;
}
