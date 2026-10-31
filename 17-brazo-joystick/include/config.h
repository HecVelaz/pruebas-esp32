#pragma once

#include <stdint.h>

// Parámetros de las tres articulaciones para mover el brazo con el joystick (lazo abierto: el joystick da el duty).
// Los pines están en pins.h.
// Cero (2026-10-30): la pose en la que los tres finales de carrera están apretados (fotos en LIMITES/ceros_bumpers).
// Cada articulación se aleja de su final de carrera en el sentido POSITIVO, así que el rango es de ~0° hacia arriba.
// El cero se fija al arrancar, con "Z" o solo, cada vez que la articulación toca su final de carrera.

struct ConfigJoystick {
  const char *nombre;
  float cuentasPorGrado;
  bool encoderInvertido;  // true si con duty positivo las cuentas bajan
  bool sentidoInvertido;  // true si el duty positivo mueve la articulación al revés de su sentido positivo
  uint32_t pwmHz;
  float limiteNeg;  // ° desde el cero; fuera de esto solo deja volver hacia adentro (se cambian en vivo con "L")
  float limitePos;
  float dutyMin;    // % con el joystick apenas movido (lo que hace falta para arrancar)
  float dutyMax;    // % con el joystick a fondo
};

constexpr int N_ART = 3;

constexpr ConfigJoystick CFG[N_ART] = {
    // J1 (base): valores de 12-j1-base (polea de 84: 41,48 cuentas/°; arranca con ~20 % a 20 kHz).
    // Cero en el final de carrera (~ +90° de 12, brazo del lado contrario al mástil); + = hacia el frente y el
    // mástil (12 igual desde el 2026-10-30). Límite 118°: con el brazo recogido (J2 y J3 en 0) llegó a 121,8°
    // cerca del soporte de la visión (resultados/limites_20261030_221357.csv). Con el brazo estirado choca antes:
    // sin medir.
    {"J1", 16.0f * 4.0f * 50.0f * (84.0f / 18.0f) / 360.0f, true, true, 20000, -5.0f, 118.0f, 18.0f, 40.0f},
    // J2 (hombro): + = se aleja del final de carrera (antihorario visto desde atrás). Signos SIN MEDIR (botón
    // "Invertir" de la página). Límite de arriba: el último punto medido de la franja del cuatro barras
    // (resultados/limites_20261030_210733.csv: J2 97,3° con J3 70-113°), donde el usuario decidió cortar.
    {"J2", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, false, false, 1000, -5.0f, 97.0f, 15.0f, 50.0f},
    // J3 (codo): + = el codo baja, alejándose del final de carrera. Límite de arriba 113°: último punto medido. 2026-10-30: con false/false iba al revés
    // (con el joystick) -> los dos en true (motor y encoder juntos).
    {"J3", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, true, true, 1000, -5.0f, 113.0f, 15.0f, 50.0f},
};

constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t TS_MS = 10;            // lazo a 100 Hz
constexpr uint32_t HOMBRE_MUERTO_MS = 300;  // sin comandos del joystick durante esto: frena las tres
constexpr float RAMPA_PCT_POR_S = 200.0f;   // el duty sube como mucho así
// Al soltar el joystick, el duty baja como mucho así (0 = de golpe). J1 con rampa (2026-10-30): de golpe frenaba
// seco y el brazo rebotaba; de 40 % a 0 tarda ~0,33 s. J2 y J3 de golpe: no se sostienen solas. El hombre muerto,
// FRENAR, los límites, el final de carrera y el atasco cortan siempre de golpe.
constexpr float RAMPA_BAJADA_PCT_POR_S[N_ART] = {120.0f, 0.0f, 0.0f};
constexpr float ZONA_MUERTA_JOY = 0.05f;    // |eje| menor que esto = joystick en el centro
constexpr uint32_t TELEMETRIA_MS = 50;      // "Q ..." a 20 Hz

// Atasco: con este duty o más y menos de ATASCO_CUENTAS en ATASCO_MS, corta esa articulación hasta que se suelte
// el joystick (fuente con límite de 3 A: trabado, el motor pide hasta 10 A).
constexpr float ATASCO_DUTY_PCT = 30.0f;
constexpr uint32_t ATASCO_MS = 500;
constexpr int32_t ATASCO_CUENTAS = 5;

// Cuatro barras (J2 y J3 juntas), medido con el joystick el 2026-10-30 (resultados/limites_20261030_210733.csv):
// el codo baja como mucho q2 + 9,7° (peor caso) y el hombro sube como mucho q3 + 27,3° (peor caso; arrastra a J3).
// Con 2° de margen:  q2 - Q2_MENOS_Q3_MAX <= q3 <= q2 + Q3_MENOS_Q2_MAX
constexpr float Q3_MENOS_Q2_MAX = 7.5f;
constexpr float Q2_MENOS_Q3_MAX = 25.0f;

// Finales de carrera: un cambio vale con FC_LECTURAS lecturas seguidas iguales, una por ciclo (TS_MS)
constexpr int FC_LECTURAS = 3;
