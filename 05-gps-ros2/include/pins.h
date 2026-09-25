#pragma once

#include <stdint.h>

// Placa: Freenove ESP32-S3-WROOM (N16R8) sobre breakout Freenove v1.2.
// Evitar GPIO19/20 (USB nativo) y GPIO26-37 (flash y PSRAM octal del N16R8).
// Ocupados por el IMU: GPIO8/9 (I2C). Reservados para el lidar: GPIO14 (RX2) y GPIO21 (TX2).

// GPS Fastrax UP501 en UART1 (mismo cableado que 04-gps-up501) (Serial1). TX y RX van cruzados.
constexpr int PIN_GPS_RX = 41;  // RX1 del ESP32 <- TXD del GPS (pin 2)
constexpr int PIN_GPS_TX = 42;  // TX1 del ESP32 -> RXD del GPS (pin 1)
constexpr uint32_t GPS_BAUD = 9600;  // valor de fábrica del UP501, 8N1
// PPS (pin 6 del GPS) sin conectar. Si se usa más adelante: GPIO47.
