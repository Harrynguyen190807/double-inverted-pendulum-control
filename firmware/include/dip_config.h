/**
 * @file dip_config.h
 * @brief Configuration parameters for Double Inverted Pendulum (DIP) Embedded Control
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#ifndef DIP_CONFIG_H
#define DIP_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ========================================================================== */
/* TIMING & EXECUTION SETTINGS                                                */
/* ========================================================================== */
#define CONTROL_LOOP_FREQ_HZ         (1000U)                      /**< Hardware Timer ISR frequency */
#define CONTROL_DT_SEC               (1.0f / (float)CONTROL_LOOP_FREQ_HZ) /**< Sampling time Ts = 1ms */

/* ========================================================================== */
/* ENCODER & SENSOR PARAMETERS                                                */
/* ========================================================================== */
/* Cart Linear Position Encoder (e.g., optical quadrature x4 on timing belt) */
#define CART_ENCODER_CPR             (4000.0f)                    /**< Counts per motor revolution */
#define CART_PULLEY_CIRCUMFERENCE_M  (0.120f)                     /**< Belt drive pitch circumference */
#define CART_METERS_PER_TICK         (CART_PULLEY_CIRCUMFERENCE_M / CART_ENCODER_CPR)

/* AS5048A 14-Bit Magnetic Rotary Encoders (SPI) for Joint 1 & Joint 2 */
#define AS5048A_RESOLUTION           (16384.0f)                   /**< 14-bit single turn (2^14) */
#define AS5048A_RAD_PER_COUNT        (6.28318530718f / AS5048A_RESOLUTION)
#define AS5048A_SPI_CMD_READ_ANGLE   (0xFFFFU)                    /**< Read angle register command with parity */

/* ========================================================================== */
/* LQR OPTIMAL GAIN VECTOR: u = - (Kx*x + Kx_dot*x_dot + ... )               */
/* State Vector z = [x, x_dot, th1, th1_dot, th2, th2_dot]^T                 */
/* ========================================================================== */
#define LQR_K_X                      (400.000000f)
#define LQR_K_X_DOT                  (397.857298f)
#define LQR_K_TH1                    (1819.630163f)
#define LQR_K_TH1_DOT                (-179.157067f)
#define LQR_K_TH2                    (-3720.664533f)
#define LQR_K_TH2_DOT                (-341.436391f)


/* ========================================================================== */
/* ACTUATOR & MOTOR DRIVER LIMITS                                             */
/* ========================================================================== */
#define ACTUATOR_MAX_FORCE_N         (25.0f)                      /**< Maximum linear motor force */
#define ACTUATOR_PWM_MAX_DUTY        (100.0f)                     /**< Max PWM duty cycle % */
#define FORCE_TO_PWM_RATIO           (ACTUATOR_PWM_MAX_DUTY / ACTUATOR_MAX_FORCE_N)

/* Friction & Stiction Compensation */
#define COULOMB_STICTION_FORCE_N     (1.20f)                      /**< Static breakaway friction feedforward */
#define VELOCITY_DEADBAND_M_S        (0.015f)                     /**< Velocity threshold for Coulomb sign */

/* ========================================================================== */
/* SAFETY & EMERGENCY FAIL-SAFE THRESHOLDS                                    */
/* ========================================================================== */
#define FAILSAFE_MAX_ANGLE_RAD       (0.349066f)                  /**< 20.0 degrees emergency cutoff */
#define FAILSAFE_MAX_CART_POS_M      (0.95f)                      /**< Physical rail limit clamp */

/* ========================================================================== */
/* KALMAN FILTER TUNING PARAMETERS (Discrete 1-D for each rotary joint)       */
/* State: [angle, angular_velocity]^T                                         */
/* ========================================================================== */
#define KF_PROCESS_NOISE_POS         (1e-4f)                      /**< Q_pos */
#define KF_PROCESS_NOISE_VEL         (1e-1f)                      /**< Q_vel */
#define KF_MEASUREMENT_NOISE         (1e-3f)                      /**< R_meas (AS5048A quantization) */

/* Telemetry Baudrate */
#define TELEMETRY_UART_BAUDRATE      (921600U)

#ifdef __cplusplus
}
#endif

#endif /* DIP_CONFIG_H */
