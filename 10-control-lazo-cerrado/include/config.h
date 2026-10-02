#pragma once

// Parámetros de J1 (base giratoria): motor 36GP-555 + IBT-2. Los pines están en pins.h.

// Encoder Hall 16 PPR x4 en el eje del motor, reductora 50:1 -> 3200 cuentas por vuelta de la salida
// de la reductora. La transmisión de esa salida a la articulación todavía no se midió (paso 1).
constexpr float J1_CUENTAS_POR_VUELTA = 16.0f * 4.0f * 50.0f;

// Paso 0: signos. Se ajustan con los comandos "ie" e "is" y después se copian acá.
//  - J1_ENCODER_INVERTIDO: true si con duty positivo las cuentas bajan.
//  - J1_SENTIDO_INVERTIDO: true si, con el encoder ya de acuerdo, el duty positivo mueve la base al
//    revés del sentido positivo de q1 (antihorario visto desde arriba). Invierte motor y encoder juntos,
//    así siguen de acuerdo.
constexpr bool J1_ENCODER_INVERTIDO = false;
constexpr bool J1_SENTIDO_INVERTIDO = false;

// PWM del IBT-2 (igual que en 06)
constexpr uint32_t J1_PWM_FREQ_HZ = 20000;
constexpr uint8_t J1_PWM_BITS = 10;

// Seguridad mientras no se conozca la transmisión ni haya finales de carrera
constexpr float J1_DUTY_MAX_PCT = 40.0f;      // los pulsos del paso 0 no pasan de este duty
constexpr uint32_t J1_PULSO_MAX_MS = 1000;    // ni de este tiempo
constexpr int32_t J1_LIMITE_CUENTAS = 400;    // corte si se aleja más que esto del cero (1/8 de vuelta de salida)
constexpr int32_t J1_CUENTAS_SIN_GIRO = 5;    // menos que esto en un pulso = no giró
constexpr uint32_t J1_SIN_CUENTAS_MS = 150;  // corte si con duty aplicado no llega ninguna cuenta en este tiempo
