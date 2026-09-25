#pragma once

#include <stdint.h>

// Motor 36GP-555 de 12 V, 160 rpm en la salida, eje de 8 mm: reductora planetaria y
// encoder Hall A/B en el eje del motor (antes de la reductora).

// Relación de la reductora. Variante de 160 rpm: el 555 gira a ~8000 rpm a 12 V -> 8000 / 160 = 50.
// Las otras variantes son 5.2 / 14 / 27 / 82 / 139 / 218 / 369 / 516 / 721. Verificar con el comando
// 'n' (contar vueltas de la salida, ver README). Solo afecta a las rpm de salida que se imprimen.
constexpr float REDUCCION = 50.0f;

constexpr float ENCODER_PPR = 16.0f;  // pulsos por vuelta del eje del motor, por fase (dato del vendedor)
constexpr float ENCODER_X = 4.0f;     // se cuentan los flancos de subida y bajada de A y de B
// Si con duty positivo las rpm salen negativas, poner true (o cruzar A y B).
constexpr bool ENCODER_INVERTIDO = false;

// PWM del BTS7960: 20 kHz queda fuera del rango audible (el chip admite hasta ~25 kHz).
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;  // 0..1023

// Límites de la prueba. La rampa reduce el pico de corriente al arrancar con el motor libre
// (para la fuente de 12 V / 3 A), pero NO limita la corriente si el motor se traba: para eso
// está el corte por atasco. No hay medición de corriente (R_IS/L_IS sin conectar).
constexpr float DUTY_MAX_PCT = 100.0f;  // duty máximo permitido (valor absoluto)
constexpr float RAMPA_PCT_S = 50.0f;    // cambio máximo del duty: 0 -> 100 % en 2 s

// Parada antes de invertir el sentido o de un escalón: duty 0 durante PARADA_MIN_MS y el motor
// por debajo de RPM_QUIETO (rpm del eje del motor; 60 rpm son ~1 rpm en la salida con 50:1).
constexpr uint32_t PARADA_MIN_MS = 300;
constexpr float RPM_QUIETO = 60.0f;

// Corte por atasco: con |duty| >= ATASCO_DUTY_PCT, si el motor gira a menos de ATASCO_RPM
// (rpm del motor, en el sentido pedido) durante ATASCO_MS, se frena. También salta si el encoder
// está desconectado o invertido. ATASCO_DUTY_PCT debe quedar por encima de la zona muerta.
constexpr float ATASCO_DUTY_PCT = 40.0f;
constexpr float ATASCO_RPM = 300.0f;
constexpr uint32_t ATASCO_MS = 1000;
