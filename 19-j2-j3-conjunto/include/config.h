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
// 2026-10-31 (19, comando "pwm", fuente a la bornera): a 20 kHz J2 va y sostiene bien (a 12 / 4 / 20: error ≤ 0,20°,
// duty de sostener estable ~16-17 %; a 1 kHz, justo antes, dos SIN ASENTAR con el duty de -11 a +18 %) y no pita.
// Decisión del usuario: 20 kHz. El 40 % del 26/10 fue con la fuente por el adaptador XT60 de contacto malo.
// La zona muerta, el despegue y ARRANQUE_PCT de abajo son de 1 kHz: PROVISORIOS hasta medirlos a 20 kHz.
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;

// Límites propios desde el final de carrera (medidos con 17 el 2026-10-30). Además, la franja del cuatro barras
// (abajo, BANDA_*) los achica según dónde esté J3.
constexpr float LIMITE_POS_GRADOS = 97.0f;
constexpr float LIMITE_NEG_GRADOS = 0.0f;
// 2026-10-31: 60 -> 70 % para 20 kHz (más duty por la misma fuerza). PROVISORIO. A 12 V el motor trabado pide hasta
// 10 A: fuente con límite de 3 A
constexpr float DUTY_MAX_PCT = 70.0f;
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
constexpr float ZONA_MUERTA_PCT = 17.0f;      // PROVISORIO: de 1 kHz (medir a 20 kHz)
constexpr float ZONA_MUERTA_NEG_PCT = 8.0f;   // PROVISORIO: de 1 kHz
constexpr float DESPEGUE_PCT = ZONA_MUERTA_PCT;
constexpr float DESPEGUE_NEG_PCT = ZONA_MUERTA_NEG_PCT;
constexpr float ARRANQUE_PCT = 45.0f;      // PROVISORIO (35 -> 45 para 20 kHz: J2 arranca con 35 %): debajo de esto no actúa el corte "sin cuentas" (sostener el
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
constexpr float ATASCO_DUTY_PCT = 60.0f;   // debajo de DUTY_MAX_PCT (50 -> 60 para 20 kHz, PROVISORIO)
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
// Compensación de fricción al sostener (ver FR_RAMPA_GRADOS abajo). 2026-10-31 a 20 kHz (-e j2): J2 llega y se
// asienta sin ella (a 25 / a 20: error +0,19° / -0,05°, duty ~0 %). PROVISORIO.
constexpr float FR_SOSTENER = 0.0f;   // %
// Fase 3b: modelo del duty que la sostiene, u = SOST_C + SOST_K2·q2 + SOST_K3·q3 (%, q en °), de
// resultados/mapa_gravedad_20261031_135400.csv (20 kHz, 13 poses desde abajo y desde arriba). J2: solo q2 (validación
// cruzada 8,6 %; plano 8,9 %, cos q2 + cos q3 10,1 %: sobreajusta). +26 % en 10°, ~0 en 54°, -15 % en 80°.
// La fricción pesa como la gravedad (±7 %): el integrador corrige el resto. PROVISORIO.
constexpr float SOST_C = 31.4f;
constexpr float SOST_K2 = -0.58f;
constexpr float SOST_K3 = 0.0f;
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
// 2026-10-31 (19, comando "pwm", fuente a la bornera): a 20 kHz J2 va y sostiene bien (a 12 / 4 / 20: error ≤ 0,20°,
// duty de sostener estable ~16-17 %; a 1 kHz, justo antes, dos SIN ASENTAR con el duty de -11 a +18 %) y no pita.
// Decisión del usuario: 20 kHz. El 40 % del 26/10 fue con la fuente por el adaptador XT60 de contacto malo.
// La zona muerta, el despegue y ARRANQUE_PCT de abajo son de 1 kHz: PROVISORIOS hasta medirlos a 20 kHz.
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;

// Límites propios desde el final de carrera (medidos con 17 el 2026-10-30). Además, la franja del cuatro barras
// (abajo, BANDA_*) los achica según dónde esté J2.
constexpr float LIMITE_POS_GRADOS = 113.0f;
constexpr float LIMITE_NEG_GRADOS = 0.0f;
// 2026-10-31: 60 -> 70 % para 20 kHz (más duty por la misma fuerza). PROVISORIO. A 12 V el motor trabado pide hasta
// 10 A: fuente con límite de 3 A
constexpr float DUTY_MAX_PCT = 70.0f;
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
constexpr float ZONA_MUERTA_PCT = 17.0f;      // PROVISORIO: de 1 kHz (medir a 20 kHz)
constexpr float ZONA_MUERTA_NEG_PCT = 8.0f;   // PROVISORIO: de 1 kHz
constexpr float DESPEGUE_PCT = ZONA_MUERTA_PCT;
constexpr float DESPEGUE_NEG_PCT = ZONA_MUERTA_NEG_PCT;
constexpr float ARRANQUE_PCT = 45.0f;      // PROVISORIO (35 -> 45 para 20 kHz: J2 arranca con 35 %): debajo de esto no actúa el corte "sin cuentas" (sostener el
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
constexpr float ATASCO_DUTY_PCT = 60.0f;   // debajo de DUTY_MAX_PCT (50 -> 60 para 20 kHz, PROVISORIO)
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
// Compensación de fricción al sostener. 2026-10-31 a 20 kHz (-e j2, J3 como la otra, J2 en 20°), de 14° a 10°: sin
// ella se pasaba hasta 8,53° y quedaba con -0,20°; con 10 % +0,22°; con 15 % no se pasa (10,16 -> 9,91°) y queda con
// +0,09°, LLEGÓ enseguida; con 20 % en 14°, -0,24°. Queda 15 %. PROVISORIO (medido en una sola pose).
constexpr float FR_SOSTENER = 15.0f;  // %
// Fase 3b: modelo del duty que la sostiene (ver cfg_j2). J3: constante -16,8 % (validación cruzada 7,2 %; plano 6,2 %,
// pero con pendientes chicas y ruido de ±7 %: se deja constante). PROVISORIO.
constexpr float SOST_C = -16.8f;
constexpr float SOST_K2 = 0.0f;
constexpr float SOST_K3 = 0.0f;
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
constexpr ConfigOtra OTRA_J2 = {"J2", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, false, false, cfg_j2::PWM_FREQ_HZ,
                                0.0f, 97.0f, 3.0f, 8.0f, 60.0f, 5.0f};
constexpr ConfigOtra OTRA_J3 = {"J3", 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f, true, true, cfg_j3::PWM_FREQ_HZ,
                                0.0f, 113.0f, 3.0f, 8.0f, 60.0f, 5.0f};
constexpr float OTRA_ERROR_AVISO = 5.0f;   // ° de error durante OTRA_ERROR_MS: avisa (sigue sosteniendo)
constexpr uint32_t OTRA_ERROR_MS = 1000;
// Con este duty o más y sin moverse OTRA_ATASCO_MS: deja de empujar (freno). 45 -> 55 % y el duty máximo de la otra
// 50 -> 60 % para 20 kHz (2026-10-31: J3 sostenía con -34 % en (10°, 3°)). PROVISORIO.
constexpr float OTRA_ATASCO_DUTY = 55.0f;
constexpr uint32_t OTRA_ATASCO_MS = 1500;
// Apoyada en su final de carrera, la otra sostiene su peso con el duty aprendido, pero hacia él nunca más que esto
// (el aprendido puede haber crecido viajando). J3 sostiene con -17 a -27 % a 20 kHz (mapa del 2026-10-31). PROVISORIO.
constexpr float OTRA_FC_SOSTEN_MAX = 30.0f;
// Mientras la otra va a su destino, los movimientos de la activa esperan como mucho esto (a 5 °/s, 113° son ~23 s).
// PROVISORIO.
constexpr uint32_t OTRA_ESPERA_MAX_MS = 30000;

// ---------- Sostener quieto (las dos: la activa en main.cpp y la otra en otra.cpp) ----------
// Banda del integrador: con |error| menor que esto, uHold no integra (con la fricción, la zona muerta de 8-17 % y
// KHP 3 %/°, integrar errores de décimas hacía que el duty fuera y viniera entre dos puntos). PROVISORIO. En vivo: "bi".
// 2026-10-31 (prueba F2, -e j2): con 0,1° J3 iba y venía ±1° (stick-slip: 8,9° ↔ 10,6° pidiendo 10°, duty de −22 a
// −4 %); con 0,2° quedó quieta a 0,04° con el duty fijo (−24,5 %). Queda 0,2° (revisar con las fases 3 y 6).
constexpr float BANDA_INT_HOLD = 0.2f;  // °
// Freno dentro de la banda: llegada (|error| < TOL_POS y quieta LLEGADA_MS) -> duty 0 (freno, sin pitido ni
// corriente); si se aleja más de TOL_SALIDA vuelve a corregir con el uHold aprendido. Con la reductora (~695:1) el
// brazo se quedó quieto frenado (2026-10-31, J2 en 48°): falta ver si en todas las poses. PROVISORIO. En vivo: "fb".
// 2026-10-31: prendido por defecto (a 20 kHz las dos quedan frenadas, quietas y sin ruido; error al llegar 0,07-0,19°).
constexpr bool FRENO_EN_BANDA = true;
// "p" negativo con el final de carrera de la activa SUELTO puede pasar su límite propio (0°) para llegar a él (lo
// corta al apretarse), como mucho esto más abajo (por si el final de carrera no anda). La franja se respeta siempre.
constexpr float HASTA_FC_MAX_GRADOS = 10.0f;
// Compensación de fricción al sostener (las dos): suma FR·clamp(error / FR_RAMPA_GRADOS, -1, 1) al duty. A 20 kHz la
// franja de duty en la que la fricción no deja moverse es ancha (J3 en ~10°: de +13 a -33 %) y el integrador la
// cruzaba de a saltos (J3 iba y venía ±1°). Con 0,3° de error o más suma FR entero; cerca de 0 casi nada (no tiembla).
// El valor de cada una está en su cfg_jX::FR_SOSTENER; en vivo, "fr" (activa) y "ofr" (otra).
constexpr float FR_RAMPA_GRADOS = 0.3f;
// Fase 3b: el sostener de las dos suma el modelo de cfg_jX (SOST_*) según la pose, y uHold aprende solo la diferencia
// (antes se arrastraba el duty de la pose anterior: en 1 kHz J2 subió sola de 25° a 27,1°). En vivo: "gm".
// 2026-10-31 (prueba F3b): apagado por defecto. A 20 kHz la fricción sostiene J2 en todo un rango de duty y el modelo
// la empeoraba (en 50° cayó 1,2°: el residuo para compensar el modelo se arrastraba a la pose siguiente).
constexpr bool MODELO_SOSTEN = false;
// ---------- Fase 4: homing ("h") ----------
// Los finales de carrera tienen un solo punto de apoyo y el golpe al llegar rápido los giraba (2026-10-31: el cero se
// corrió ~9° en J3 y ~30° en J2). El homing llega siempre desde el mismo lado (+, alejado) y despacio, pone el cero
// en el flanco (el instante en que se aprieta), retrocede y queda sosteniendo. Primero J3 (la otra o la activa, según
// el entorno, siempre el codo primero: con el codo abajo, bajar el hombro no viola la franja), después J2. PROVISORIO.
constexpr float HOMING_VEL_BUSCAR = 1.0f;     // °/s hacia el final de carrera
constexpr float HOMING_VEL_ALEJAR = 2.0f;     // °/s para soltarlo si arranca apretado
constexpr float HOMING_SALIDA_GRADOS = 2.0f;  // se aleja esto más allá de donde se suelta, antes de buscarlo
constexpr float HOMING_RETROCESO = 3.0f;      // ° donde queda sosteniendo después del cero
constexpr float HOMING_RECORRIDO_MAX = 120.0f;  // ° como mucho buscando
constexpr float HOMING_DUTY_MAX = 50.0f;      // % (la activa; la otra, con su duty máximo)
constexpr uint32_t HOMING_TRABADA_MS = 1500;  // sin avanzar (con el duty al tope) esto: corta
constexpr float HOMING_SIGUE_MAX = 3.0f;      // la otra: si la articulación queda más atrás que esto de la referencia
// Sin acercamiento rápido (si el flanco se corrió, a velocidad normal podía golpearlo): busca a HOMING_VEL_BUSCAR
// desde donde esté. Con el cero conocido, J2 más alto que esto: bajarla antes (si no, el codo no llega a su final de
// carrera sin salir de la franja); y si durante la búsqueda la franja se viola, corta.
constexpr float HOMING_Q2_MAX_ARRANQUE = 20.0f;
// La otra buscando su final de carrera con "o" negativo: a esta velocidad (no a OTRA.vel)
constexpr float OTRA_VEL_BUSCAR_FC = 2.0f;
// Asentamiento: después de llegar, cuánto sigue sosteniendo antes de informar la posición final ("Asentado").
constexpr uint32_t ASENTAMIENTO_MS = 2000;  // PROVISORIO

#if defined(ARTICULACION_J3)
namespace art = cfg_j3;
namespace otra_cfg = cfg_j2;  // tolerancias de llegada de la otra (las mismas de su config)
constexpr ConfigOtra OTRA = OTRA_J2;
// activa - otra = q3 - q2, entre -Q2_MENOS_Q3_MAX y +Q3_MENOS_Q2_MAX
constexpr float BANDA_MIN = -Q2_MENOS_Q3_MAX;
constexpr float BANDA_MAX = Q3_MENOS_Q2_MAX;
#else
namespace art = cfg_j2;
namespace otra_cfg = cfg_j3;
constexpr ConfigOtra OTRA = OTRA_J3;
// activa - otra = q2 - q3, entre -Q3_MENOS_Q2_MAX y +Q2_MENOS_Q3_MAX
constexpr float BANDA_MIN = -Q3_MENOS_Q2_MAX;
constexpr float BANDA_MAX = Q2_MENOS_Q3_MAX;
#endif
