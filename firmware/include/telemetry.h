/**
 * @file telemetry.h
 * @brief High-speed binary telemetry framing for DIP over UART (Baudrate 921600)
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dip_controller.h"
#include <stdint.h>
#include <stddef.h>

#define TELEMETRY_SYNC_WORD          (0x55AAU)

#pragma pack(push, 1)
/**
 * @brief Binary Telemetry Packet Structure (Total: 44 bytes, ultra-compact)
 */
typedef struct {
    uint16_t sync_header;        /**< 0x55AA */
    uint32_t sequence_id;        /**< Monotonically increasing packet counter */
    uint32_t timestamp_us;       /**< Microseconds hardware timestamp */
    float x;                     /**< Cart position (m) */
    float x_dot;                 /**< Cart velocity (m/s) */
    float th1;                   /**< Pole 1 angle (rad) */
    float th1_dot;               /**< Pole 1 rate (rad/s) */
    float th2;                   /**< Pole 2 angle (rad) */
    float th2_dot;               /**< Pole 2 rate (rad/s) */
    float u_command;             /**< Actuator force (N) */
    float pwm_duty;              /**< PWM Duty Cycle (%) */
    uint8_t status_flags;        /**< System status flags */
    uint16_t checksum_crc16;     /**< CRC-16 CCITT */
} TelemetryPacket_t;
#pragma pack(pop)

/**
 * @brief Computes standard CCITT CRC-16 over data buffer.
 */
uint16_t Telemetry_ComputeCRC16(const uint8_t *data, size_t length);

/**
 * @brief Packages controller state into binary telemetry frame ready for DMA UART TX.
 * @return Number of serialized bytes (sizeof(TelemetryPacket_t))
 */
size_t Telemetry_Pack(const DIP_Controller_t *ctrl,
                      uint32_t timestamp_us,
                      uint8_t *tx_buffer,
                      size_t max_buffer_len);

#ifdef __cplusplus
}
#endif

#endif /* TELEMETRY_H */
