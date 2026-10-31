#pragma once

#include <stdint.h>

// 19 · J2 y J3 juntas. Una es la ACTIVA (la que se ajusta con los comandos de siempre: p, m, bc, ev, ep, a...) y la
// otra se SOSTIENE sola en segundo plano (otra.cpp), en el ángulo que se le pida con "o <grados>".
// Se elige al compilar: -e j2 (activa J2, se sostiene J3) o -e j3 (activa J3, se sostiene J2).
// Cero: la pose con los finales de carrera de J2 y J3 apretados ("z"). + = alejarse del final de carrera.
// Lo marcado PROVISORIO viene de 13 y 14 y se reemplaza con lo que se mida (README.md).

// ---------- Cuatro barras ----------
// Medido con el joystick (17-brazo-joystick, 2026-10-30, resultados/limites_20261030_210733.csv): el codo baja como
// mucho q2 + 9,7° y el hombro sube como mucho q3 + 27,3° (peores casos). Con 2° de margen:
//   q2 - Q2_MENOS_Q3_MAX <= q3 <= q2 + Q3_MENOS_Q2_MAX
constexpr float Q3_MENOS_Q2_MAX = 7.5f;
constexpr float Q2_MENOS_Q3_MAX = 25.0f;

// ---------- J2 (hombro) ----------
namespace cfg_j2 {
constexpr const char *NOMBRE = "J2";
constexpr const char *PIEZA = "hombro";
constexpr const char *SENTIDO_POS = "alejándose del final de carrera (antihorario visto desde atrás)";
constexpr const char *SENTIDO_NEG = "hacia el final de carrera";
constexpr const char *MARCA = "J2 y J3 apretando sus finales de carrera (17-brazo-joystick/LIMITES/ceros_bumpers)";
constexpr bool SIGNOS_EN_VIVO = false;  // signos medidos con 17 (2026-10-30): fijos acá
// El 36-555 es planetario y NO se autobloquea: sin corriente el brazo cae. Frenado (EN alto, PWM 0) cae más despacio.
constexpr bool FRENO_EN_REPOSO = true;
// "sa" al arrancar: después de "z" sostiene el ángulo entre comando y comando. Recién sirve con los signos bien
// (paso 0): si el encoder está al revés, el sostener empuja hacia el lado equivocado (lo corta el límite).
constexpr bool SOSTENER_AUTO = true;

// Encoder SCX-555 (16 PPR x4) en el eje del motor -> 64 cuentas por vuelta del motor x reductora 139:1.
// Del motor al hombro: engranajes helicoidales de 15 dientes (motor) y 75 (hombro) -> 5:1.
constexpr const char *MOTOR = "36-555 58 rpm";
constexpr float REDUCTORA = 139.0f;
constexpr float CUENTAS_POR_VUELTA_SALIDA = 16.0f * 4.0f * REDUCTORA;
constexpr float RELACION_TRANSMISION = 75.0f / 15.0f;  // engranajes
constexpr float CUENTAS_POR_GRADO = CUENTAS_POR_VUELTA_SALIDA * RELACION_TRANSMISION / 360.0f;  // 123,6 (VERIFICAR, paso 1)

// Signos (positivo = alejarse del final de carrera): medidos con 17-brazo-joystick el 2026-10-30 (J2 bien con
// false/false). Verificar con "p" en el paso 1.
//  - ENCODER_INVERTIDO: true si con duty positivo las cuentas bajan.
//  - SENTIDO_INVERTIDO: true si, con el encoder ya de acuerdo, el duty positivo baja el brazo.
constexpr bool ENCODER_INVERTIDO = false;
constexpr bool SENTIDO_INVERTIDO = false;

// PWM del IBT-2: 20 kHz como J1 y el firmware del robot. Si a duty bajo rinde poco (con el 5840 en 10, a 20 kHz el
// IBT-2 entregaba ~1/3 de la tensión esperada), probar 1000.
// 2026-10-26, paso 0 a 20 kHz: arrancaba recién con 40 % subiendo Y bajando (35 % no movía) → se prueba 1 kHz.
constexpr uint32_t PWM_FREQ_HZ = 1000;
constexpr uint8_t PWM_BITS = 10;

// Límites propios desde el final de carrera (medidos con 17 el 2026-10-30). Además, la franja del cuatro barras
// (abajo, BANDA_*) los achica según dónde esté J3.
constexpr float LIMITE_POS_GRADOS = 97.0f;
constexpr float LIMITE_NEG_GRADOS = 0.0f;
constexpr float DUTY_MAX_PCT = 60.0f;      // PROVISORIO. A 12 V el motor trabado pide hasta 10 A: fuente con límite de 3 A
constexpr uint32_t PULSO_MAX_MS = 500;     // a 100 % el hombro va a ~70 °/s (58 rpm / 5 x 6)
constexpr uint32_t PULSO_DEF_MS = 100;
constexpr int32_t CUENTAS_SIN_GIRO = 5;
constexpr uint32_t SIN_CUENTAS_MS = 150;
constexpr int32_t AVANCE_MIN = 3;

// Paso 1: "m <grados>"
constexpr float DUTY_MOVER_PCT = 35.0f;    // PROVISORIO: ajustar con lo que dé el paso 0 subiendo
constexpr uint32_t MOVER_MAX_MS = 10000;

// Paso 2: PI de velocidad. PROVISORIO: las ganancias de J1 (con este motor K es ~1/3 de la de J1, así que el lazo
// queda ~3 veces más lento: seguro para empezar). Con el modelo medido (barrido "bc" + escalones), rediseñar.
// Feedforward PROVISORIO: subiendo la zona muerta de J1; bajando la gravedad ayuda (en 10, con el 36GP-555 en el
// hombro, 8 % era el intermedio entre pasarse y no frenar). Medir con "bc" y ajustar con "ff".
constexpr float KP_VEL = 0.549f;
constexpr float KI_VEL = 4.0f;
constexpr float FF_PENDIENTE_POS = 0.0f;   // compensación de gravedad (%/°): medir con "bc"
constexpr float ZONA_MUERTA_PCT = 17.0f;
constexpr float ZONA_MUERTA_NEG_PCT = 8.0f;
constexpr float DESPEGUE_PCT = ZONA_MUERTA_PCT;
constexpr float DESPEGUE_NEG_PCT = ZONA_MUERTA_NEG_PCT;
constexpr float ARRANQUE_PCT = 35.0f;      // PROVISORIO: debajo de esto no actúa el corte "sin cuentas" (sostener el
                                           // peso ya pide duty sin moverse). Ajustar con el paso 0
constexpr uint32_t TS_VEL_US = 10000;
constexpr int VENTANA_VEL = 4;
constexpr float VEL_MAX = 30.0f;
constexpr float VEL_MIN = 5.0f;
constexpr float CONTRARIO_VEL = 5.0f;
constexpr uint32_t CONTRARIO_MS = 100;
constexpr uint32_t ESCALON_MAX_MS = 3000;
constexpr uint32_t PAUSA_MS = 500;
constexpr uint32_t CICLOS_MAX = 60;
constexpr float ATASCO_VEL = 2.0f;
constexpr float ATASCO_DUTY_PCT = 50.0f;   // debajo de DUTY_MAX_PCT
constexpr uint32_t ATASCO_MS = 300;

// Paso 3: P de posición (como J1)
constexpr float KPP = 2.0f;
constexpr uint32_t TS_POS_US = 20000;
constexpr float VMAX_POS = 20.0f;
constexpr float AMAX_POS = 60.0f;
constexpr float VMIN_POS = 3.0f;
constexpr float VMIN_BAJA_POS = VMIN_POS;
constexpr uint32_t TRABADA_MS = 100;
constexpr float DITHER_PCT = 0.0f;
constexpr float TOL_POS = 0.3f;
constexpr float TOL_SALIDA = 0.6f;
constexpr uint32_t LLEGADA_MS = 200;
constexpr uint32_t IR_MAX_MS = 15000;
constexpr uint32_t EP_SEG_MAX_MS = 5000;
constexpr uint32_t EP_CICLOS_MAX = 20;
// Empujón de arranque de "m" y "p": desactivado (0)
constexpr float KICK_PCT = 0.0f;
constexpr uint32_t KICK_MS = 0;
constexpr int32_t KICK_CUENTAS = 0;
constexpr bool KICK_SOLO_POSITIVO = true;
// Sostener quieto ("s", "sa"): duty = uHold + KHP·error, y uHold aprende el duty que sostiene el peso.
// En 10 (36GP-555 de 160 rpm en el hombro) funcionó con 6 y 15. Este motor da ~3 veces más par por % de duty:
// PROVISORIO a la mitad. Si oscila quieto, bajar khp; si cae y tarda en volver, subir khi (en vivo: "khp", "khi").
constexpr float KHP_SOSTENER = 3.0f;   // % por grado
constexpr float KHI_SOSTENER = 8.0f;   // % por grado y por segundo
}  // namespace cfg_j2

// ---------- J3 (codo) ----------
namespace cfg_j3 {
constexpr const char *NOMBRE = "J3";
constexpr const char *PIEZA = "codo";
constexpr const char *SENTIDO_POS = "el codo baja (alejándose del final de carrera)";
constexpr const char *SENTIDO_NEG = "el codo sube (hacia el final de carrera)";
constexpr const char *MARCA = "J2 y J3 apretando sus finales de carrera (17-brazo-joystick/LIMITES/ceros_bumpers)";
constexpr bool SIGNOS_EN_VIVO = false;  // signos medidos con 17 (2026-10-30): fijos acá
// El 36-555 es planetario y NO se autobloquea (el sin fin del 5840 sí): sin corriente el antebrazo cae. Frenado (EN alto, PWM 0) cae más despacio.
constexpr bool FRENO_EN_REPOSO = true;
// "sa" al arrancar: después de "z" sostiene el ángulo entre comando y comando. Recién sirve con los signos bien
// (paso 0): si el encoder está al revés, el sostener empuja hacia el lado equivocado (lo corta el límite).
constexpr bool SOSTENER_AUTO = true;

// Encoder SCX-555 (16 PPR x4) en el eje del motor -> 64 cuentas por vuelta del motor x reductora 139:1.
// Del motor al codo: engranajes helicoidales de 15 dientes (motor) y 75 (codo) -> 5:1.
constexpr const char *MOTOR = "36-555 58 rpm";
constexpr float REDUCTORA = 139.0f;
constexpr float CUENTAS_POR_VUELTA_SALIDA = 16.0f * 4.0f * REDUCTORA;
constexpr float RELACION_TRANSMISION = 75.0f / 15.0f;  // engranajes
constexpr float CUENTAS_POR_GRADO = CUENTAS_POR_VUELTA_SALIDA * RELACION_TRANSMISION / 360.0f;  // 123,6 (VERIFICAR, paso 1)

// Signos (positivo = alejarse del final de carrera = el codo baja): medidos con 17 el 2026-10-30. En 17 iba al revés
// con false/false y quedó con los dos invertidos = acá SENTIDO_INVERTIDO (invierte motor y encoder juntos).
// Verificar con "p" en el paso 1.
//  - ENCODER_INVERTIDO: true si con duty positivo las cuentas bajan.
//  - SENTIDO_INVERTIDO: true si, con el encoder ya de acuerdo, el duty positivo baja la pinza.
constexpr bool ENCODER_INVERTIDO = false;
constexpr bool SENTIDO_INVERTIDO = true;

// PWM del IBT-2: 20 kHz como J1 y el firmware del robot. Con el 5840 (10, 2026-10-03), a 20 kHz el IBT-2 entregaba
// ~1/3 de la tensión esperada a duty bajo y a 1 kHz rendía 2-3 veces más: si pasa lo mismo, probar 1000.
// 2026-10-26: en 13 (J2, mismo motor y driver) a 20 kHz no arrancaba con menos de 40 % y a 1 kHz sí → 1 kHz.
constexpr uint32_t PWM_FREQ_HZ = 1000;
constexpr uint8_t PWM_BITS = 10;

// Límites propios desde el final de carrera (medidos con 17 el 2026-10-30). Además, la franja del cuatro barras
// (abajo, BANDA_*) los achica según dónde esté J2.
constexpr float LIMITE_POS_GRADOS = 113.0f;
constexpr float LIMITE_NEG_GRADOS = 0.0f;
constexpr float DUTY_MAX_PCT = 60.0f;      // PROVISORIO. A 12 V el motor trabado pide hasta 10 A: fuente con límite de 3 A
constexpr uint32_t PULSO_MAX_MS = 500;     // a 100 % el codo va a ~70 °/s (58 rpm / 5 x 6)
constexpr uint32_t PULSO_DEF_MS = 100;
constexpr int32_t CUENTAS_SIN_GIRO = 5;
constexpr uint32_t SIN_CUENTAS_MS = 150;
constexpr int32_t AVANCE_MIN = 3;

// Paso 1: "m <grados>"
constexpr float DUTY_MOVER_PCT = 35.0f;    // PROVISORIO: ajustar con lo que dé el paso 0 subiendo
constexpr uint32_t MOVER_MAX_MS = 10000;

// Paso 2: PI de velocidad. PROVISORIO: las ganancias de J1 (con este motor K es ~1/3 de la de J1, así que el lazo
// queda ~3 veces más lento: seguro para empezar). Con el modelo medido (barrido "bc" + escalones), rediseñar.
// Feedforward PROVISORIO: subiendo la zona muerta de J1; bajando la gravedad ayuda (en 10, con el 36GP-555 en el
// hombro, 8 % era el intermedio entre pasarse y no frenar). Medir con "bc" y ajustar con "ff". Con el 5840 la carga
// del codo cambiaba mucho con el ángulo (~1,4 %/° subiendo): esperar una pendiente parecida.
constexpr float KP_VEL = 0.549f;
constexpr float KI_VEL = 4.0f;
constexpr float FF_PENDIENTE_POS = 0.0f;   // compensación de gravedad (%/°): medir con "bc"
constexpr float ZONA_MUERTA_PCT = 17.0f;
constexpr float ZONA_MUERTA_NEG_PCT = 8.0f;
constexpr float DESPEGUE_PCT = ZONA_MUERTA_PCT;
constexpr float DESPEGUE_NEG_PCT = ZONA_MUERTA_NEG_PCT;
constexpr float ARRANQUE_PCT = 35.0f;      // PROVISORIO: debajo de esto no actúa el corte "sin cuentas" (sostener el
                                           // peso ya pide duty sin moverse). Ajustar con el paso 0
constexpr uint32_t TS_VEL_US = 10000;
constexpr int VENTANA_VEL = 4;
constexpr float VEL_MAX = 30.0f;
constexpr float VEL_MIN = 5.0f;
constexpr float CONTRARIO_VEL = 5.0f;
constexpr uint32_t CONTRARIO_MS = 100;
constexpr uint32_t ESCALON_MAX_MS = 3000;
constexpr uint32_t PAUSA_MS = 500;
constexpr uint32_t CICLOS_MAX = 60;
constexpr float ATASCO_VEL = 2.0f;
constexpr float ATASCO_DUTY_PCT = 50.0f;   // debajo de DUTY_MAX_PCT
constexpr uint32_t ATASCO_MS = 300;

// Paso 3: P de posición (como J1)
constexpr float KPP = 2.0f;
constexpr uint32_t TS_POS_US = 20000;
constexpr float VMAX_POS = 20.0f;
constexpr float AMAX_POS = 60.0f;
constexpr float VMIN_POS = 3.0f;
constexpr float VMIN_BAJA_POS = VMIN_POS;
constexpr uint32_t TRABADA_MS = 100;
constexpr float DITHER_PCT = 0.0f;
constexpr float TOL_POS = 0.3f;
constexpr float TOL_SALIDA = 0.6f;
constexpr uint32_t LLEGADA_MS = 200;
constexpr uint32_t IR_MAX_MS = 15000;
constexpr uint32_t EP_SEG_MAX_MS = 5000;
constexpr uint32_t EP_CICLOS_MAX = 20;
// Empujón de arranque de "m" y "p": desactivado (0)
constexpr float KICK_PCT = 0.0f;
constexpr uint32_t KICK_MS = 0;
constexpr int32_t KICK_CUENTAS = 0;
constexpr bool KICK_SOLO_POSITIVO = true;
// Sostener quieto ("s", "sa"): duty = uHold + KHP·error, y uHold aprende el duty que sostiene el peso.
// Como J2 (PROVISORIO): en 10, el hombro con el 36GP-555 de 160 rpm funcionó con 6 y 15, y este motor da ~3 veces
// más par por % de duty. Si oscila quieto, bajar khp; si cae y tarda en volver, subir khi (en vivo: "khp", "khi").
constexpr float KHP_SOSTENER = 3.0f;   // % por grado
constexpr float KHI_SOSTENER = 8.0f;   // % por grado y por segundo
}  // namespace cfg_j3

// ---------- La otra articulación (la que se sostiene en segundo plano, otra.cpp) ----------
// Lazo de posición simple a 100 Hz: la referencia va hacia el destino a VEL °/s y
// duty = uHold + KHP·error, con uHold que aprende el duty que sostiene el peso (KHI). PROVISORIO: KHP/KHI de 13/14.
// Signos con el criterio de 17: INV_CUENTAS invierte las cuentas, INV_DUTY el duty (J3: los dos, como en 17).
struct ConfigOtra {
  const char *nombre;
  float cuentasPorGrado;
  bool invCuentas, invDuty;
  uint32_t pwmHz;
  float limNeg, limPos;  // límites propios (°)
  float khp, khi;        // % por grado, % por grado y por segundo
  float dutyMax;         // %
  float vel;             // °/s de la referencia
};
constexpr ConfigOtra OTRA_J2 = {"J2", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, false, false, 1000,
                                0.0f, 97.0f, 3.0f, 8.0f, 50.0f, 5.0f};
constexpr ConfigOtra OTRA_J3 = {"J3", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, true, true, 1000,
                                0.0f, 113.0f, 3.0f, 8.0f, 50.0f, 5.0f};
constexpr float OTRA_ERROR_AVISO = 5.0f;   // ° de error durante OTRA_ERROR_MS: avisa (sigue sosteniendo)
constexpr uint32_t OTRA_ERROR_MS = 1000;
constexpr float OTRA_ATASCO_DUTY = 45.0f;  // con este duty o más y sin moverse OTRA_ATASCO_MS: deja de empujar (freno)
constexpr uint32_t OTRA_ATASCO_MS = 1500;

#if defined(ARTICULACION_J3)
namespace art = cfg_j3;
constexpr ConfigOtra OTRA = OTRA_J2;
// activa - otra = q3 - q2, entre -Q2_MENOS_Q3_MAX y +Q3_MENOS_Q2_MAX
constexpr float BANDA_MIN = -Q2_MENOS_Q3_MAX;
constexpr float BANDA_MAX = Q3_MENOS_Q2_MAX;
#else
namespace art = cfg_j2;
constexpr ConfigOtra OTRA = OTRA_J3;
// activa - otra = q2 - q3, entre -Q3_MENOS_Q2_MAX y +Q2_MENOS_Q3_MAX
constexpr float BANDA_MIN = -Q3_MENOS_Q2_MAX;
constexpr float BANDA_MAX = Q2_MENOS_Q3_MAX;
#endif
