---
name: dip-firmware-embedded
description: >-
  Phát triển & Tối ưu Firmware Nhúng C/C++ điều khiển thời gian thực 1 kHz cho Con lắc ngược đôi (DIP).
  Bao gồm đọc Encoder phần cứng x4, giao tiếp SPI AS5048A 14-bit, Bộ lọc Kalman rời rạc 1-D ước lượng vận tốc,
  thuật toán LQR, bù ma sát tĩnh Coulomb, ngắt bảo vệ khẩn cấp Fail-safe và giao thức Telemetry UART 921600 baud.
---

# Skill: `dip-firmware-embedded`

## Tổng quan
Skill này hướng dẫn và hỗ trợ việc xây dựng, tối ưu hóa và kiểm thử mã nguồn C/C++ thuần nhúng chạy trực tiếp trên vi điều khiển (STM32 Cortex-M4/M7, ESP32-S3, TI C2000) với ngắt định thời cứng 1 kHz (Hardware Timer ISR).

## Khi nào sử dụng
- Khi triển khai thuật toán điều khiển LQR lên vi điều khiển thực tế.
- Khi cần cấu hình và đọc dữ liệu encoder quang quadrature x4 qua bộ đếm phần cứng Hardware Timer Encoder Mode.
- Khi cần đọc góc chính xác từ 2 cảm biến từ tính tuyệt đối 14-bit AS5048A qua giao diện SPI tốc độ cao.
- Khi cần khử nhiễu lượng tử hóa và vi sai vận tốc góc bằng Bộ lọc Kalman rời rạc 1-D (Discrete Kalman Filter) nhằm tránh khuếch đại nhiễu cao tần vào tín hiệu điều khiển.
- Khi tinh chỉnh bộ bù ma sát tĩnh Coulomb (Coulomb Stiction Compensation) để khắc phục hiện tượng trễ tĩnh tại điểm không vận tốc.
- Khi cấu hình cơ chế ngắt an toàn khẩn cấp phần cứng (Emergency Fail-Safe Cutoff) để ngắt nguồn động cơ tức thì khi $|\theta_1| > 20^\circ$, $|\theta_2| > 20^\circ$ hoặc $|x| > 0.95\text{ m}$.
- Khi tích hợp giao thức truyền thông telemetry nhị phân UART tốc độ cao (Baudrate 921600, CRC16-CCITT) để đẩy dữ liệu thời gian thực lên máy tính với độ trễ cực thấp.

## Cấu trúc Module Firmware

```text
firmware/
├── include/
│   ├── dip_config.h       # Tham số phần cứng, độ lợi LQR, ngưỡng an toàn, tần số ngắt
│   ├── dip_controller.h   # Khung điều khiển, trạng thái máy, nguyên mẫu hàm 1 kHz Step
│   ├── kalman_filter.h    # Định nghĩa cấu trúc và thuật toán Kalman 1-D cho vị trí & góc
│   └── telemetry.h        # Định nghĩa gói tin nhị phân packed 44 bytes và thuật toán CRC16
└── src/
    ├── main.c             # Điểm vào chính, ngắt định thời cứng 1 kHz, mô phỏng HAL
    ├── dip_controller.c   # Triển khai tính toán LQR, bù ma sát Coulomb, cắt PWM bão hòa
    ├── kalman_filter.c    # Triển khai bước dự đoán và cập nhật Kalman 1-D
    └── telemetry.c        # Đóng gói dữ liệu DMA và tính toán kiểm tra CRC16 CCITT
```

## Các lệnh & Hướng dẫn Biên dịch

### Biên dịch & Chạy thử nghiệm trên Host (GCC / Clang)
```bash
gcc -Wall -Wextra -O2 -I firmware/include firmware/src/*.c -lm -o firmware/dip_firmware_sim
./firmware/dip_firmware_sim
```

### Triển khai lên Vi điều khiển Thực tế (STM32CubeIDE / Keil / CMake ARM GCC)
1. Thêm các file trong `firmware/include` vào Include Paths.
2. Thêm các file trong `firmware/src` vào Source Groups.
3. Trong hàm ngắt `TIMx_UP_IRQHandler` (cấu hình chu kỳ đúng $1.0\text{ ms}$):
   ```c
   void TIM1_UP_IRQHandler(void) {
       if (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE)) {
           __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
           TIM1_UP_IRQHandler_1kHz(); // Gọi hàm điều khiển cốt lõi
       }
   }
   ```
4. Cấu hình UART DMA ở chế độ Circular hoặc Normal với Baudrate `921600, 8-N-1`.
