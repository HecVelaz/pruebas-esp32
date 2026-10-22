# Pines del ESP32-S3 (Freenove N16R8)

<!-- Movido desde CLAUDE.md (2026-10-22), sin cambios de texto. -->

- **Pines:** ver la tabla de abajo. I2C del IMU a 400 kHz, AD0→GND (0x68). NCS y FSYNC del IMU sin conectar.

| GPIO | Uso | Estado |
|------|-----|--------|
| 8 | I2C SDA del IMU (MPU-6500) | ocupado |
| 9 | I2C SCL del IMU | ocupado |
| 41 | RX1 ← TXD del GPS (pin 2 del UP501) | ocupado |
| 42 | TX1 → RXD del GPS (pin 1 del UP501) | ocupado |
| 47 | PPS del GPS | reservado (sin conectar) |
| 14 | RX2 ← TX del lidar | reservado |
| 21 | TX2 → RX del lidar | reservado |
| 19, 20 | USB nativo (`/dev/ttyACM0`, micro-ROS serial) | prohibido |
| 26–37 | flash y PSRAM octal | prohibido |
| 0, 3, 45, 46 | strapping | evitar |
| 43, 44 | UART0, conectado al CH343 del segundo USB de la placa | evitar |
| 10–13, 17, 18 | cámara (también 4–7, 15, 16 y 8/9, ya usados) | libres si no se usa la cámara |
| 38–40 | ranura SD | libres si no se usa la SD |
| 2, 48 | LED y LED RGB de la placa | evitar |
| 4 | RPWM del IBT-2 (motor) | ocupado |
| 5 | LPWM del IBT-2 | ocupado |
| 6 | R_EN del IBT-2 | ocupado |
| 7 | L_EN del IBT-2 | ocupado |
| 15 | Fase A del encoder del motor | ocupado |
| 16 | Fase B del encoder del motor | ocupado |
| 1 | — | libre |

Alimentación: IMU y GPS en 3V3 (el UP501 en VDD y VDD_B). UARTs: UART0 libre (CH343), UART1 = GPS, UART2 = lidar.
