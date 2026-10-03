#pragma once

#include <stdint.h>

// Parámetros de J1 (base giratoria): motor 36GP-555 + IBT-2. Los pines están en pins.h.

// Encoder Hall 16 PPR x4 en el eje del motor, reductora 50:1 -> 3200 cuentas por vuelta de la salida
// de la reductora. De ahí a la base: correa GT2, polea de 18 dientes en el motor y 90 en la base (5:1).
constexpr float J1_CUENTAS_POR_VUELTA_SALIDA = 16.0f * 4.0f * 50.0f;
constexpr float J1_RELACION_CORREA = 90.0f / 18.0f;
// 16 000 cuentas por vuelta de la base = 44,4 por grado. Verificado en el paso 1 (m 45 paralelo a un
// doblez de 45°, error de pocos grados como mucho, a ojo).
constexpr float J1_CUENTAS_POR_GRADO = J1_CUENTAS_POR_VUELTA_SALIDA * J1_RELACION_CORREA / 360.0f;

// Paso 0: signos. Se ajustan con los comandos "ie" e "is" y después se copian acá.
//  - J1_ENCODER_INVERTIDO: true si con duty positivo las cuentas bajan.
//  - J1_SENTIDO_INVERTIDO: true si, con el encoder ya de acuerdo, el duty positivo mueve la base al
//    revés del sentido positivo de q1 (antihorario visto desde arriba). Invierte motor y encoder juntos,
//    así siguen de acuerdo.
constexpr bool J1_ENCODER_INVERTIDO = false;
constexpr bool J1_SENTIDO_INVERTIDO = true;   // prueba 0 (2026-10-02): duty + giraba horario

// PWM del IBT-2 (igual que en 06)
constexpr uint32_t J1_PWM_FREQ_HZ = 20000;
constexpr uint8_t J1_PWM_BITS = 10;

// Seguridad mientras no haya finales de carrera (los topes de la base están en ±75°)
constexpr float J1_LIMITE_GRADOS = 60.0f;     // corte si la base se aleja más que esto del cero.
                                              // Paso 1 (2026-10-02): transmisión confirmada (m 45 = 45° reales)
constexpr float J1_DUTY_MAX_PCT = 40.0f;      // ningún movimiento de estas pruebas pasa de este duty
constexpr uint32_t J1_PULSO_MAX_MS = 1000;    // duración máxima de un pulso
constexpr int32_t J1_CUENTAS_SIN_GIRO = 5;    // menos que esto en un pulso = no giró
constexpr uint32_t J1_SIN_CUENTAS_MS = 150;   // ventana de avance: en este tiempo, con duty aplicado,
constexpr int32_t J1_AVANCE_MIN = 3;          // la base tiene que avanzar al menos esto en el sentido pedido

// Paso 1: comando "m <grados>" (ir a un ángulo con duty fijo, sin control: frena al llegar)
constexpr float J1_DUTY_MOVER_PCT = 30.0f;    // con 25 % casi no arranca con la carga del brazo
constexpr uint32_t J1_MOVER_MAX_MS = 15000;   // a 30 % la base va a ~10-15 °/s
