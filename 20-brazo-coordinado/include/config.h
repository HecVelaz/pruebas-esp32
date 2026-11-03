#pragma once

#include "Articulacion.h"
#include "pins.h"

// 20 · J1, J2 y J3 con el MISMO controlador (lib/Articulacion), un lazo no bloqueante a 100 Hz.
// Valores de 12-j1-base (J1) y 19-j2-j3-conjunto (J2, J3), todos a 20 kHz. Lo marcado PROVISORIO no está medido en
// esta configuración. Cero: el flanco de cada final de carrera ("h"); + = alejarse del final de carrera.

// ---------- Cuatro barras (17-brazo-joystick, 2026-10-30; con el cero del 30/10 = la pose de la foto ceros_bumpers)
//   q2 - Q2_MENOS_Q3_MAX <= q3 <= q2 + Q3_MENOS_Q2_MAX
constexpr float Q3_MENOS_Q2_MAX = 7.5f;
constexpr float Q2_MENOS_Q3_MAX = 25.0f;
constexpr float FRANJA_HOLGURA = 0.5f;  // el control frena todo si la medición sale de la franja más que esto

// ---------- Movimientos de una articulación ("j") y homing ("h")
constexpr float MARGEN_LIMITE = 3.0f;      // ° de margen a los límites (propios y de la franja) para los destinos
constexpr float HOMING_RETROCESO = 3.0f;   // ° donde queda cada una después de su cero
constexpr float HOMING_Q2_MAX_ARRANQUE = 20.0f;  // con el cero conocido, J2 más alta: el codo no llega a su final
// Sin cero el homing supone el brazo abajo y recogido: si una avanza más que esto sin su final de carrera, corta
// (2026-11-03: arrancó con el brazo estirado, J3 subió 28° y se trabó contra la franja). Desde "mv 3 3 3", ~3°
constexpr float HOMING_RECORRIDO_SIN_CERO = 15.0f;
constexpr float HOMING_Q2_JUNTAS = 5.0f;  // las tres hacen el homing juntas si J2 está abajo (o sin cero: el brazo
                                          // tiene que estar recogido); si no, J3 -> J2 -> J1
constexpr uint32_t ASENTAMIENTO_MS = 2000;
constexpr bool FRENO_EN_BANDA = true;      // 19, 2026-10-31: frenadas, quietas y sin ruido

// ---------- Fase 5b: movimientos coordinados ("mv") ----------
// Las tres siguen el MISMO perfil normalizado (quíntico: velocidad y aceleración 0 en los extremos) y terminan juntas;
// J2 y J3 avanzan al mismo ritmo, así que el camino en (q2, q3) es una recta y la franja (convexa) se cumple.
constexpr float MV_FACTOR_VEL = 0.5f;      // fracción de vMax/aMax de cada una para "mv" (PROVISORIO: empezar despacio)
constexpr float MV_T_MIN = 1.0f;           // s, duración mínima de un movimiento
// Llegada siempre con el ángulo creciendo (el juego de los engranajes queda del mismo lado): todas las que se mueven
// (también las que suben, desde 2026-10-31) terminan el recorrido APROX_GRADOS por debajo y suben el último tramo a
// VEL_APROX.
constexpr float APROX_GRADOS = 2.0f;
constexpr float VEL_APROX = 2.0f;          // °/s (pico del perfil del último tramo)
constexpr float MV_ERROR_MAX = 5.0f;       // ° entre la referencia y la medición: corta (no sigue la trayectoria)
constexpr uint32_t MV_LLEGADA_MAX_MS = 20000;  // después de la trayectoria, tiempo para quedar las tres firmes

// ---------- Joystick ("jv a1 a2 a3", 2026-11-03): cada eje (-1 a 1) es la velocidad de su articulación, en lazo
// cerrado (la referencia avanza a esa velocidad y la cascada la sigue); al soltar, se sostiene donde quedó. Límites y
// franja con MARGEN_LIMITE: frena antes del borde. Lo usa tools/joystick.py (de 17) con --placa 20.
constexpr float JOY_FACTOR_VEL = 0.4f;      // fracción de vMax con el eje a fondo (J1 12 °/s, J2 y J3 8 °/s)
constexpr float JOY_FACTOR_ACEL = 0.5f;     // fracción de aMax para acelerar y frenar la referencia
constexpr float JOY_ZONA_MUERTA = 0.05f;    // |eje| menor que esto = 0
constexpr uint32_t JOY_HOMBRE_MUERTO_MS = 300;  // sin "jv" en este tiempo: los ejes vuelven a 0 (frena y sostiene)

// ---------- J1 (base): 36GP-555 160 rpm (50:1), correa 18 -> 84 (12-j1-base)
constexpr ConfigArticulacion CFG_J1 = {
    "J1",
    PIN_J1_RPWM, PIN_J1_LPWM, PIN_BRAZO_EN, PIN_FC_J1, 0, 1, PIN_J1_ENC_A, PIN_J1_ENC_B, PCNT_UNIT_0, 20000, 10,
    16.0f * 4.0f * 50.0f * (84.0f / 18.0f) / 360.0f,  // 41,48 cuentas/°
    false, true,                                       // signos de 12 (2026-10-30, + hacia el mástil)
    // Límites: 0-118° medidos con el brazo RECOGIDO; estirado choca antes con el soporte de la visión (sin medir):
    // 90° PROVISORIO (decisión del usuario, 2026-10-31)
    0.0f, 90.0f, 70.0f,
    0.549f, 4.0f,                         // PI de velocidad de 12 (terminada)
    17.0f, 17.0f, 17.0f, 17.0f, 0.0f,     // feedforward de 12 (sin gravedad)
    2.0f, 30.0f, 60.0f, 3.0f, 0.3f, 0.6f, 200,
    3.0f, 8.0f, 0.0f, 0.2f,               // sostener (J1 se autobloquea por la correa: frena enseguida)
    1.5f, 2.0f, 2.0f, 50.0f, 130.0f, 1500, 3.0f, 1.5f,  // homing a 1,5 °/s (2026-11-03, antes 1; rápido a 3 °/s)
    2.0f, 35.0f, 300, 25.0f, 150, 5.0f, 100, 30.0f,
    true};  // se autobloquea: al llegar, freno (como 12; con el PI de posición iba y venía por el juego de la correa)

// ---------- J2 (hombro): 36-555 58 rpm (139:1), engranajes 15 -> 75 (19, a 20 kHz)
constexpr ConfigArticulacion CFG_J2 = {
    "J2",
    PIN_J2_RPWM, PIN_J2_LPWM, PIN_BRAZO_EN, PIN_FC_J2, 2, 3, PIN_J2_ENC_A, PIN_J2_ENC_B, PCNT_UNIT_1, 20000, 10,
    16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f,  // 123,6 cuentas/°
    false, false,
    0.0f, 97.0f, 70.0f,
    0.549f, 4.0f,
    35.0f, 25.0f, 22.0f, 8.0f, 8.0f,      // despegue 35 % (medido a 20 kHz), resto PROVISORIO (19)
    2.0f, 20.0f, 60.0f, 3.0f, 0.3f, 0.6f, 200,  // 2026-10-31: con tolPos 0,15 y bandaInt 0,1 J2 repetía peor (0,29°; una vez "no quedan firmes")
    3.0f, 8.0f, 0.0f, 0.2f,               // sin compensación de fricción (19: llega sin ella)
    // homing a 1,5 °/s (2026-11-03: 0,5 tardaba 6,3 s en 3°, 1 tardaba 3,4 s; 19: flanco ~0,07° a 0,5). Duty máximo 65 % (PROVISORIO): con 50 % no la levantaba desde el final
    // de carrera (2026-10-31: subió 1,5° y se trabó; sostenerla abajo pide ~25 % más el despegue)
    1.5f, 2.0f, 2.0f, 65.0f, 120.0f, 1500, 2.0f, 1.5f,  // con cero: 2 °/s hasta 1,5° del cero anterior
    2.0f, 60.0f, 300, 45.0f, 150, 5.0f, 100, 30.0f,
    false};

// ---------- J3 (codo): 36-555 58 rpm (139:1), engranajes 15 -> 75 (19, a 20 kHz)
// El feedforward de J3 en cascada no se midió a 20 kHz (en 19 se sostenía con su lazo simple). + = el codo baja, a
// favor del peso; - lo sube contra el peso (sostener pide -17 a -27 %). PROVISORIO: ajustar con "ff 3 ...".
constexpr ConfigArticulacion CFG_J3 = {
    "J3",
    PIN_J3_RPWM, PIN_J3_LPWM, PIN_BRAZO_EN, PIN_FC_J3, 4, 5, PIN_J3_ENC_A, PIN_J3_ENC_B, PCNT_UNIT_2, 20000, 10,
    16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f,
    false, true,                          // 19: motor y encoder invertidos juntos
    0.0f, 113.0f, 70.0f,
    0.549f, 4.0f,
    20.0f, 10.0f, 40.0f, 30.0f, 0.0f,     // PROVISORIO (sin medir)
    2.0f, 20.0f, 60.0f, 3.0f, 0.15f, 0.6f, 200,  // tolPos 0,3 -> 0,15 (2026-10-31: repite en 0,03° en vez de 0,30°)
    3.0f, 8.0f, 15.0f, 0.1f,              // compensación de fricción 15 % (19: no se pasa, 0,09°); bandaInt 0,2 -> 0,1
    1.5f, 2.0f, 2.0f, 50.0f, 120.0f, 1500, 2.0f, 1.5f,  // homing a 1,5 °/s (2026-11-03; 19: flanco ~0,04° a 1)
    2.0f, 60.0f, 300, 45.0f, 150, 5.0f, 100, 30.0f,
    false};
