#pragma once

#include <stdint.h>

// Parámetros de las tres articulaciones para mover el brazo con el joystick (lazo abierto: el joystick da el duty).
// Los pines están en pins.h. El cero es la posición en la que arranca el programa (o la del último "z").

struct ConfigJoystick {
  const char *nombre;
  float cuentasPorGrado;
  bool encoderInvertido;  // true si con duty positivo las cuentas bajan
  bool sentidoInvertido;  // true si el duty positivo mueve la articulación al revés de su sentido positivo
  uint32_t pwmHz;
  float limiteNeg;  // ° desde el cero; fuera de esto solo deja volver hacia adentro
  float limitePos;
  float dutyMin;    // % con el joystick apenas movido (lo que hace falta para arrancar)
  float dutyMax;    // % con el joystick a fondo
};

constexpr int N_ART = 3;

constexpr ConfigJoystick CFG[N_ART] = {
    // J1 (base): valores de 12-j1-base. Signos medidos (paso 0, 2026-10-26). Arranca con ~20 % a 20 kHz.
    // Escala: polea de 84 dientes en la base (41,48 cuentas por grado, medido en 12 el 2026-10-30).
    // Cero hacia el frente (2026-10-30): -30° (mástil, choca a -38°) / +90°; + se aleja del mástil (signos de 12).
    {"J1", 16.0f * 4.0f * 50.0f * (84.0f / 18.0f) / 360.0f, false, false, 20000, -30.0f, 90.0f, 18.0f, 40.0f},
    // J2 (hombro): valores de 13-j2-hombro, todo PROVISORIO (signos sin medir; a 20 kHz arrancaba con 40 %).
    {"J2", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, false, false, 1000, -15.0f, 15.0f, 15.0f, 50.0f},
    // J3 (codo): valores de 14-j3-codo, todo PROVISORIO.
    {"J3", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, false, false, 1000, -15.0f, 15.0f, 15.0f, 50.0f},
};

constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t TS_MS = 10;            // lazo a 100 Hz
constexpr uint32_t HOMBRE_MUERTO_MS = 300;  // sin comandos del joystick durante esto: frena las tres
constexpr float RAMPA_PCT_POR_S = 200.0f;   // el duty sube como mucho así (baja de golpe)
constexpr float ZONA_MUERTA_JOY = 0.05f;    // |eje| menor que esto = joystick en el centro
constexpr uint32_t TELEMETRIA_MS = 50;      // "Q ..." a 20 Hz

// Atasco: con este duty o más y menos de ATASCO_CUENTAS en ATASCO_MS, corta esa articulación hasta que se suelte
// el joystick (fuente con límite de 3 A: trabado, el motor pide hasta 10 A).
constexpr float ATASCO_DUTY_PCT = 30.0f;
constexpr uint32_t ATASCO_MS = 500;
constexpr int32_t ATASCO_CUENTAS = 5;
