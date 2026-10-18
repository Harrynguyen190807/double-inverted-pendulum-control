/**
 * @file telemetry.c
 * @brief High-speed binary telemetry framing and CRC16 implementation
 * @author Lead Mechatronics & Embedded Control Engineer
 */

#include "telemetry.h"
#include <string.h>

static uint32_t s_packet_sequence = 0;

uint16_t Telemetry_ComputeCRC16(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000U) {
                crc = (crc << 1) ^ 0x1021U; /* CCITT polynomial */
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

size_t Telemetry_Pack(const DIP_Controller_t *ctrl,
                      uint32_t timestamp_us,
                      uint8_t *tx_buffer,
                      size_t max_buffer_len) {
    if (!ctrl || !tx_buffer || max_buffer_len < sizeof(TelemetryPacket_t)) {
        return 0;
    }

    TelemetryPacket_t packet;
    packet.sync_header = TELEMETRY_SYNC_WORD;
    packet.sequence_id = ++s_packet_sequence;
    packet.timestamp_us = timestamp_us;

    packet.x = ctrl->x;
    packet.x_dot = ctrl->x_dot;
    packet.th1 = ctrl->th1;
    packet.th1_dot = ctrl->th1_dot;
    packet.th2 = ctrl->th2;
    packet.th2_dot = ctrl->th2_dot;
    packet.u_command = ctrl->u_command;
    packet.pwm_duty = ctrl->pwm_duty;
    packet.status_flags = ctrl->status_flags;

    /* Compute CRC-16 over all bytes preceding the checksum field */
    const size_t payload_len = offsetof(TelemetryPacket_t, checksum_crc16);
    packet.checksum_crc16 = Telemetry_ComputeCRC16((const uint8_t *)&packet, payload_len);

    memcpy(tx_buffer, &packet, sizeof(TelemetryPacket_t));
    return sizeof(TelemetryPacket_t);
}
