#pragma once

#include <stdint.h>

// Motor 36GP-555 de 12 V, 160 rpm, reductora 50:1, encoder Hall 16 PPR (los mismos valores que 06-motor-36gp555).
constexpr float REDUCCION = 50.0f;
constexpr float ENCODER_PPR = 16.0f;
constexpr float ENCODER_X = 4.0f;
constexpr bool ENCODER_INVERTIDO = false;

constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;

// Rampa por defecto (comando r): 0 -> 100 % en RAMPA_S_DEF segundos, termina a VUELTAS_DEF vueltas de salida.
// Las vueltas al llegar al 100 % son proporcionales a la duración de la rampa: medido sin carga (2026-09-25),
// 5 s -> 5 vueltas con 98,8 %, así que 3 s -> 3 vueltas con 98,8 % (el giro completo 0 -> 100 %).
constexpr float RAMPA_S_DEF = 3.0f;
constexpr float VUELTAS_DEF = 3.0f;
// Si llega al 100 % sin completar las vueltas (carga alta), se mantiene en 100 % como máximo este tiempo
constexpr uint32_t ESPERA_EN_100_MS = 15000;

// El motor debe estar quieto para empezar (rpm del eje del motor)
constexpr float RPM_QUIETO = 60.0f;

// Corte por atasco, como en 06: con duty >= ATASCO_DUTY_PCT, menos de ATASCO_RPM (rpm del motor en el
// sentido pedido) durante ATASCO_MS. Con carga, subir ATASCO_DUTY_PCT si corta antes de tiempo.
constexpr float ATASCO_DUTY_PCT = 40.0f;
constexpr float ATASCO_RPM = 300.0f;
constexpr uint32_t ATASCO_MS = 1000;
