#pragma once

// Placa: ESP32-C3-MINI-1. GPIO de 3,3 V (no toleran 5 V).
// Evitar: 2, 8 y 9 (strapping; el 8 suele tener el LED), 11-17 (flash del módulo), 18/19 (USB nativo),
// 20/21 (UART0). Libres y usados acá: 4, 5, 6, 7 y 10 (los mismos que en 15).

// L298N, canal A (OUT1/OUT2). ENA al pin de señal, SACANDO el jumper de ENA (el otro pin es 5 V). Sin pull-down:
// mientras el C3 arranca, se flashea o se resetea, ENA, IN1 e IN2 flotan y el puente puede habilitarse, así que los
// 12 V se prenden solo con el firmware corriendo y se apagan antes de flashear/resetear (como en 15).
// El L298N toma como alto desde ~2,3 V: anda directo con 3,3 V.
constexpr int PIN_IN1 = 4;
constexpr int PIN_IN2 = 5;
constexpr int PIN_ENA = 6;

// Encoder SCX-555 de J3 a 3V3 (verde = 3V3, azul = GND): trae pull-up en su placa, y además el interno
constexpr int PIN_ENC_A = 7;   // negro
constexpr int PIN_ENC_B = 10;  // rojo
