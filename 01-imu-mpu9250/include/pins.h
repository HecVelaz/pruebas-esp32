#pragma once

#include <stdint.h>

// Placa: Freenove ESP32-S3-WROOM (N16R8) sobre breakout Freenove v1.2.
// Evitar GPIO19/20 (USB nativo) y GPIO26-37 (flash y PSRAM octal del N16R8).

// Bus I2C del IMU MPU-9250/6500
constexpr int PIN_I2C_SDA = 8;
constexpr int PIN_I2C_SCL = 9;
constexpr uint32_t I2C_FREQ_HZ = 400000;  // 400 kHz (modo rápido)

// AD0 conectado a GND -> dirección 0x68 (con AD0 a 3V3 sería 0x69)
constexpr uint8_t IMU_I2C_ADDR = 0x68;
