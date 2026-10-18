---
name: dip-firmware-embedded
description: >-
  Embedded C/C++ Real-Time 1 kHz Firmware Development for the Double Inverted Pendulum (DIP).
  Covers hardware encoder timer decoding x4, AS5048A 14-bit SPI reading, Discrete 1-D Kalman Filters,
  LQR execution, Coulomb stiction feedforward, emergency fail-safe cutoff, and 921600-baud UART telemetry.
---

# Skill: `dip-firmware-embedded`

## Overview
This skill provides guidance, architectural specifications, and implementation code for building production-grade embedded C/C++ firmware targeting hard real-time 1 kHz Timer ISRs on microcontrollers such as STM32 Cortex-M4/M7, ESP32-S3, or TI C2000 DSPs.

## When to Use
- When deploying the LQR balancing algorithm to physical embedded target hardware.
- When configuring quadrature hardware timers for cart position decoding.
- When reading 14-bit magnetic rotary encoders (AS5048A) over high-speed SPI.
- When implementing Discrete 1-D Kalman Filters to reconstruct angular velocities without amplifying high-frequency quantization noise.
- When tuning Coulomb stiction feedforward compensation to eliminate stick-slip friction around zero cart velocity.
- When setting up hardware fail-safe limits to immediately de-energize the motor driver when $|\theta_1| > 20^\circ$, $|\theta_2| > 20^\circ$, or $|x| > 0.95\text{ m}$.
- When streaming real-time diagnostic telemetry via DMA UART at 921600 baud with CRC-16 CCITT verification.

## Firmware Module Structure

```text
firmware/
├── include/
│   ├── dip_config.h       # Hardware parameters, LQR gains, safety thresholds, timer configs
│   ├── dip_controller.h   # Controller context, state machine, 1 kHz ISR step prototype
│   ├── kalman_filter.h    # Discrete 1-D Kalman Filter definition for state & rate estimation
│   └── telemetry.h        # 44-byte binary telemetry frame and CCITT CRC-16 prototypes
└── src/
    ├── main.c             # Main loop, 1 kHz hardware timer ISR, peripheral HAL stubs
    ├── dip_controller.c   # LQR calculation, Coulomb friction feedforward, fail-safe trip
    ├── kalman_filter.c    # Time-update and measurement-update Kalman filter implementation
    └── telemetry.c        # Serialization and CRC-16 calculation for DMA UART transmission
```

## Compilation & Verification

### Build & Test on Host Workstation (GCC / Clang)
```bash
gcc -Wall -Wextra -O2 -I firmware/include firmware/src/*.c -lm -o firmware/dip_firmware_sim
./firmware/dip_firmware_sim
```

### Deployment to Physical MCU (STM32CubeIDE / Keil / CMake ARM GCC)
1. Add `firmware/include` to compiler include search paths.
2. Add `firmware/src/*.c` to compilation source list.
3. Call the controller inside the 1.0 ms timer update interrupt:
   ```c
   void TIM1_UP_IRQHandler(void) {
       if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)) {
           __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
           TIM1_UP_IRQHandler_1kHz();
       }
   }
   ```
4. Set UART peripheral to `921600 baud, 8 data bits, no parity, 1 stop bit` with DMA transmission enabled.
