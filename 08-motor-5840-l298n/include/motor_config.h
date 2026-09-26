#pragma once

#include <stdint.h>

#include "L298N.h"

// Motor 5840-31ZY (ZENG WHCD): reductor de tornillo sin fin, 12 V, 160 rpm en la salida.
// Encoder EXTERNO acoplado al eje de SALIDA: 38S6G5-B-G24N, 1000 PPR, fases A/B, NPN colector abierto, 5-24 V.
// Como el encoder mide directamente la salida, las rpm "del motor" que imprime el programa son las de salida:
// REDUCCION = 1 deja los dos valores iguales (se mantiene el formato de 06 para reutilizar el análisis).
constexpr float REDUCCION = 1.0f;

constexpr float ENCODER_PPR = 1000.0f;  // pulsos por vuelta, por fase
constexpr float ENCODER_X = 4.0f;       // flancos de A y B: 4000 cuentas por vuelta de salida
// Si con duty positivo las rpm salen negativas, poner true (o cruzar blanco y verde).
// Medido 2026-09-25 con el acople actual: +30 % daba -32 rpm -> invertido.
constexpr bool ENCODER_INVERTIDO = true;

// Forma de aplicar el PWM (ver lib/L298N/L298N.h):
//   Freno: PWM en IN1/IN2 con ENA fijo; en la parte apagada el motor queda frenado. Velocidad casi
//          proporcional al duty (por defecto).
//   RuedaLibre: PWM en ENA; en la parte apagada, rueda libre. Medido 2026-09-25: curva saturada
//          (40 % -> 86 rpm, 100 % -> 141 rpm) y zona muerta mal definida.
constexpr L298N::Modo PWM_MODO = L298N::Modo::Freno;

// El L298N es bipolar y conmuta lento: a frecuencias altas pierde linealidad en los extremos.
// 1 kHz es lineal (se escucha un zumbido); probar 5-10 kHz si molesta el ruido.
constexpr uint32_t PWM_FREQ_HZ = 1000;
constexpr uint8_t PWM_BITS = 10;  // 0..1023

// Límites de la prueba. La rampa reduce el pico de corriente al arrancar con el motor libre, pero NO
// limita la corriente si se traba. El L298N soporta 2 A por canal: límite de la fuente en 2 A.
constexpr float DUTY_MAX_PCT = 100.0f;
constexpr float RAMPA_PCT_S = 50.0f;  // 0 -> 100 % en 2 s

// Parada antes de invertir el sentido o de un escalón: duty 0 durante PARADA_MIN_MS y la salida por
// debajo de RPM_QUIETO (rpm de salida; con 4000 cuentas por vuelta, 1 rpm son ~7 cuentas en 100 ms).
constexpr uint32_t PARADA_MIN_MS = 300;
constexpr float RPM_QUIETO = 1.0f;

// Corte por atasco: con |duty| >= ATASCO_DUTY_PCT, si la salida gira a menos de ATASCO_RPM (rpm de salida,
// en el sentido pedido) durante ATASCO_MS, se frena. También salta si el encoder está desconectado o
// invertido. ATASCO_DUTY_PCT debe quedar por encima de la zona muerta (el sin fin tiene mucha fricción).
constexpr float ATASCO_DUTY_PCT = 50.0f;
constexpr float ATASCO_RPM = 5.0f;
constexpr uint32_t ATASCO_MS = 1000;
