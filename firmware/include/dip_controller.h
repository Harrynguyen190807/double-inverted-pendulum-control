/**
 * @file dip_controller.h
 * @brief High-performance 1 kHz LQR Controller with Stiction Compensation & Failsafe
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#ifndef DIP_CONTROLLER_H
#define DIP_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dip_config.h"
#include "kalman_filter.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Controller operational state
 */
typedef enum {
    DIP_STATE_UNINITIALIZED = 0,
    DIP_STATE_STANDBY       = 1,
    DIP_STATE_BALANCING     = 2,
    DIP_STATE_FAILSAFE_TRIP = 3
} DIP_State_t;

/**
 * @brief Status bit flags for telemetry and diagnostics
 */
#define STATUS_FLAG_BALANCING       (1U << 0)
#define STATUS_FLAG_FAILSAFE_TH     (1U << 1)
#define STATUS_FLAG_FAILSAFE_CART   (1U << 2)
#define STATUS_FLAG_FORCE_SATURATED (1U << 3)
#define STATUS_FLAG_ENCODER_FAULT   (1U << 4)

/**
 * @brief Main DIP Controller Context Structure
 */
typedef struct {
    /* State Vector: z = [x, x_dot, th1, th1_dot, th2, th2_dot]^T */
    float x;             /**< Cart position (m) */
    float x_dot;         /**< Cart velocity (m/s) */
    float th1;           /**< Pendulum 1 angle (rad) */
    float th1_dot;       /**< Pendulum 1 angular velocity (rad/s) */
    float th2;           /**< Pendulum 2 angle (rad) */
    float th2_dot;       /**< Pendulum 2 angular velocity (rad/s) */

    /* Controller outputs */
    float u_lqr;         /**< Pure LQR control force (N) */
    float u_friction;    /**< Coulomb stiction feedforward force (N) */
    float u_command;     /**< Total commanded force after clamp (N) */
    float pwm_duty;      /**< Commanded motor PWM duty (-100% to +100%) */

    /* Sensor estimators */
    KalmanFilter1D_t kf_cart;
    KalmanFilter1D_t kf_th1;
    KalmanFilter1D_t kf_th2;

    /* Status & Diagnostics */
    DIP_State_t system_state;
    uint8_t status_flags;
    uint32_t loop_counter;
} DIP_Controller_t;

/**
 * @brief Initializes controller structures, Kalman filters, and state machines.
 */
void DIP_Controller_Init(DIP_Controller_t *ctrl);

/**
 * @brief High-frequency 1 kHz Control Step executed inside Hardware Timer ISR.
 * 
 * @param ctrl Pointer to controller context
 * @param raw_cart_encoder_ticks Raw 32-bit hardware encoder count for cart
 * @param raw_as5048a_joint1_counts 14-bit magnetic encoder reading for Joint 1 (0..16383)
 * @param raw_as5048a_joint2_counts 14-bit magnetic encoder reading for Joint 2 (0..16383)
 * @return Commanded PWM duty cycle clamped to [-100.0%, +100.0%] (0.0% if failsafe triggered)
 */
float DIP_Controller_Step(DIP_Controller_t *ctrl,
                          int32_t raw_cart_encoder_ticks,
                          uint16_t raw_as5048a_joint1_counts,
                          uint16_t raw_as5048a_joint2_counts);

/**
 * @brief Emergency shutdown: immediately cuts power to the motor and locks controller in fail-safe.
 */
void DIP_Controller_TriggerEmergencyStop(DIP_Controller_t *ctrl, uint8_t reason_flag);

#ifdef __cplusplus
}
#endif

#endif /* DIP_CONTROLLER_H */
