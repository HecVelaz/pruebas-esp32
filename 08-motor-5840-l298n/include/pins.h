#pragma once

// Placa: ESP32-WROOM-32D (ESP32 clásico, DevKit con conversor USB-serie CH340 -> /dev/ttyUSB0).
// Evitar: GPIO6-11 (flash), 0/2/5/12/15 (strapping), 1/3 (UART0 = USB), 34-39 (solo entrada y sin pull-up).

// Driver L298N, canal A (OUT1/OUT2)
constexpr int PIN_ENA = 25;  // PWM: velocidad. Sacar el jumper de ENA y conectar el pin de señal (no el de 5 V)
constexpr int PIN_IN1 = 26;  // sentido
constexpr int PIN_IN2 = 27;  // sentido

// Encoder incremental externo 38S6G5-B-G24N (NPN colector abierto): necesita pull-up a 3V3
constexpr int PIN_ENC_A = 32;  // cable blanco
constexpr int PIN_ENC_B = 33;  // cable verde
