#pragma once

// Placa: ESP32-C3-MINI-1. GPIO de 3,3 V (no toleran 5 V).
// Evitar: 2, 8 y 9 (strapping; el 8 suele tener el LED), 11-17 (flash del módulo), 18/19 (USB nativo),
// 20/21 (UART0). Libres y usados acá: 4, 5, 6, 7 y 10.

// IBT-2 (BTS7960) de J3, solo. R_EN y L_EN juntos al GPIO 6, SIN pull-down (decisión del usuario, 2026-10-22):
// mientras el C3 arranca, se flashea o se resetea, el EN flota y el puente puede habilitarse, así que los 12 V se
// prenden solo con el firmware corriendo y se apagan antes de flashear/resetear. VCC del IBT-2 a 3V3 (74HC244),
// R_IS y L_IS sin conectar.
constexpr int PIN_RPWM = 4;
constexpr int PIN_LPWM = 5;
constexpr int PIN_EN = 6;

// Encoder SCX-555 de J3 a 3V3 (verde = 3V3, azul = GND): trae pull-up en su placa, y además el interno
constexpr int PIN_ENC_A = 7;   // negro
constexpr int PIN_ENC_B = 10;  // rojo
