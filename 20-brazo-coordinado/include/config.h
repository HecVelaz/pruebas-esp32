#pragma once

#include "Articulacion.h"
#include "pins.h"

// 20 · J1, J2 y J3 con el MISMO controlador (lib/Articulacion), un lazo no bloqueante a 100 Hz.
// Valores de 12-j1-base (J1) y 19-j2-j3-conjunto (J2, J3), todos a 20 kHz. Lo marcado PROVISORIO no está medido en
// esta configuración. Cero: el flanco de cada final de carrera ("h"); + = alejarse del final de carrera.

// ---------- Cuatro barras con la manivela de J3 de 95 mm (2026-11-04; antes 80 mm y una franja de ancho fijo)
// Medido con 17 (entorno explorar) desde el cero de los finales de carrera: 17-brazo-joystick/resultados/
// limites_20261104_213919.csv. La franja es
//   q2 - Q2_MENOS_Q3_MAX <= q3 <= q3MaxFranja(q2)
// Borde de abajo (el hombro sube con el codo atrás): q2 - q3 llegó a 51,0° (J2 no subía más con J3 en 5°; 53,8 y 54,1
// en 70 y 113°) -> 49 con 2° de margen.
constexpr float Q2_MENOS_Q3_MAX = 49.0f;
// Borde de abajo para los DESTINOS (donde J2/J3 quedan sostenidas: "mv", "j" y el joystick), con MARGEN_FRANJA: cerca
// del punto muerto del cuatro barras J3 no queda firme (2026-11-05: q3 - q2 ~ -35 no queda; >= -30 tarda 10-16 s;
// >= -25 firme en <= 1,1 s). El camino de un mv y el freno de seguridad siguen con Q2_MENOS_Q3_MAX (el límite
// mecánico): pasar por ahí moviéndose no es problema y así no aparecen cortes nuevos a mitad de camino
constexpr float Q2_MENOS_Q3_DESTINO = 25.0f;
// 2026-11-10 noche (doble helicoidales): borde de abajo medido con el joystick y "fr 0" (resultados/
// limites_20261110_214813.csv). Bajando J3 con J2 quieta, J2 se corre sola (A) mucho antes que q2 - 49: q2 - q3 de
// 20 a 27° desde J2 = 60. Una tabla (q2, q3 mínimo), sin margen, que se suma a las rectas de arriba (manda la más
// alta). J2 = 50 sin borde propio (esta tarde J3 llegó a 7,4; esta noche a 20,2 sin nada): (50, 1,0) es la recta
// q2 - 49 ahí, para que no haya salto. 55 -> 22,5 (A; otra vez llegó a 19,8 sin A). Fuera de la tabla: abajo de
// 50, solo las rectas; arriba de 80, el último valor
struct PuntoFranjaAbajo {
  float q2, q3Min;
};
constexpr PuntoFranjaAbajo FRANJA_ABAJO[] = {
    {50.0f, 1.0f},  {55.1f, 22.5f}, {60.9f, 40.3f}, {65.2f, 44.2f},
    {70.0f, 45.2f}, {74.7f, 47.8f}, {80.0f, 57.5f},
};
// Borde de arriba (el codo estira): no es una recta, una tabla (q2, q3 máximo) interpolada; fuera de la tabla, el
// valor del extremo. Con J2 en 0-30° J3 empezaba a ARRASTRAR a J2 (A: no es una traba, J2 se sostiene con su lazo):
// medidos, sin margen (si no, la pose de reposo (3, 3) quedaba afuera). Desde 45° J3 se TRABA (T): medido - 2°.
// Medidos: 0 -> 3,2 (A) · 15 -> 10,8 (A) · 30 -> 27,9 (A) · 45 -> 56,2 · 60 -> 74,2 · 75 -> 84,7 · 90 -> 93,9 ·
// 105 -> 103,6 (T). Entre 30 y 45° no hay punto medido (el borde pasa de arrastre a traba).
struct PuntoFranja {
  float q2, q3Max;
};
// 2026-11-10: J2 y J3 con engranajes DOBLE helicoidales (misma relación, mucho menos juego): el borde de arriba bajó
// hasta 23° (tabla anterior: 0 -> 3,2 · 15 -> 10,8 · 30 -> 27,9 · 45 -> 54,2 · 60 -> 72,2 · 75 -> 82,7 · 90 -> 91,9 ·
// 105 -> 101,6). Medido de nuevo con el joystick y "fr 0" desde el cero del homing (resultados/
// limites_20261110_145545.csv): en todos J3 empezaba a ARRASTRAR a J2 (sin margen, como antes). El punto de 0° no
// se midió: estimado (en el homing, con J2 en 3,4° J3 se trabó en 1,05°). El borde de abajo no se volvió a medir
// 2026-11-10 noche: medido de nuevo hasta J2 = 45 (resultados/limites_20261110_214813.csv y _222525.csv). J3 sube
// mucho más que lo de la tarde (9,9 -> 3,0 · 15,1 -> 4,0 · 30,1 -> 16,1 · 45,1 -> 31,3, descartados): 5,8 -> 8,1 ·
// 10,6 -> 10,3 · 15,1 -> 15,4 · 21,0 -> 20,3 llegaron sin nada (L: el borde está más arriba, se usa lo alcanzado);
// 30,4 -> 32,4 y 45,2 -> 51,9 con arrastre (A). Quedan (0, 0,5) estimado y (3,4, 1,05) (J3 trabado en el homing de la
// tarde, trayectoria_20261110_144239_cortado). Desde 60, J3 se TRABA (T, limites_20261110_224610.csv): 60,1 -> 72,8 ·
// 74,6 -> 85,3 · 79,8 -> 90,1 (como con los engranajes viejos: es la traba del cuatro barras), cargados con medido - 2°
// como antes. Los de la tarde (60,2 -> 52,0 · 74,6 -> 75,1 · 80,1 -> 83,4) quedan descartados
constexpr PuntoFranja FRANJA_ARRIBA[] = {
    {0.0f, 0.5f},   {3.4f, 1.05f},  {5.8f, 8.1f},   {10.6f, 10.3f}, {15.1f, 15.4f}, {21.0f, 20.3f},
    {30.4f, 32.4f}, {45.2f, 51.9f}, {60.1f, 70.8f}, {74.6f, 83.3f}, {79.8f, 88.1f},
};
constexpr float FRANJA_HOLGURA = 0.5f;  // el control frena todo si la medición sale de la franja más que esto
// Margen a la franja para los destinos de "j", "mv" y el joystick (el de los límites propios es MARGEN_LIMITE). Con
// 3° como los límites, "mv 3 3 3" y el reposo del homing quedaban afuera (en q2 = 3 el codo llega a 4,7°)
constexpr float MARGEN_FRANJA = 1.0f;
// Margen a la franja de TODO el camino de un "mv" (antes solo el destino: el camino podía pasar a 0,1° del borde). En el
// brazo J2 se atrasa al bajar ~0,6° y con FRANJA_HOLGURA 0,5 se cortaba (2026-11-05, vuelta de (74, 78) a (10, 5):
// trayectoria_20261105_133335 y _150143, 0,52° afuera en J2 ~30°). Si arranca más cerca del borde, no se acerca más
constexpr float MARGEN_CAMINO = 1.0f;

// ---------- Movimientos de una articulación ("j") y homing ("h")
constexpr float MARGEN_LIMITE = 3.0f;      // ° de margen a los límites (propios y de la franja) para los destinos
constexpr float HOMING_RETROCESO = 3.0f;   // ° donde queda cada una después de su cero
// 2026-11-10 (doble helicoidales): con J2 en 3,4° J3 no pasa de 1,05° ni con 66 % (trayectoria_20261110_144239_cortado):
// el borde de arriba de la franja bajó. J2 queda más alta que J3 después del homing (como el reposo, mv 3 10 5)
// 10 -> 20° con la franja medida el mismo día: con J2 en 10° el borde de J3 es 3,0° (el retroceso de J3 quedaba justo
// en el borde); en 20° es ~8°. Reposo nuevo: mv 3 20 5
// 2026-11-10 (noche, pedido del usuario: J2 más recogida): 20 -> 17°. El borde de J3 ahí es ~5,5° (J3 queda en ~3,2°:
// ~2,3° de aire); con 15° quedaba a 0,8°, menos que MARGEN_FRANJA. Reposo: mv 3 17 4
// Más tarde, mismo día (pedido del usuario: la pose que ve en el joystick, J1 2,8 · J2 6,0 · J3 3,2 leídos): J2 6°,
// J1 y J3 3° (llegan a ~2,8 y ~3,2 como en los homing anteriores). Para que la franja no la corte se agregó el punto
// (6,0, 3,2) a FRANJA_ARRIBA. El reposo de los scripts sigue en mv 3 17 4
constexpr float HOMING_RETROCESO_J2 = 6.0f;
constexpr float HOMING_RETROCESO_J3 = 3.0f;
constexpr float HOMING_PRE_SUBIDA_J2 = 8.0f;  // ° que sube J2 antes del homing si J3 arranca con su final de carrera apretado (2026-11-10)
constexpr float HOMING_Q2_MAX_ARRANQUE = 25.0f;  // con el cero conocido, J2 más alta: el codo no llega a su final
// Sin cero, recorrido máximo buscando el final de carrera. 2026-11-03: 15° (arrancó estirado, J3 subió 28° y se trabó
// contra la franja vieja). 2026-11-04, pedido del usuario: SIN límite propio (0 = el homingRecorridoMax de cada una,
// 120-130°, todo el rango), para bajar el brazo con "h" desde donde quedó. Siguen el corte por trabada (duty al tope
// sin avanzar) y el recorrido máximo. Sin cero las tres bajan juntas a la misma velocidad: q3 - q2 no cambia, así que
// arrancar con el codo no más estirado que el hombro (q3 <= q2, como en las poses de trabajo)
constexpr float HOMING_RECORRIDO_SIN_CERO = 0.0f;
constexpr float HOMING_Q2_JUNTAS = 5.0f;  // las tres hacen el homing juntas si J2 está abajo (o sin cero: el brazo
                                          // tiene que estar recogido); si no, J3 -> J2 -> J1
constexpr uint32_t ASENTAMIENTO_MS = 2000;
constexpr bool FRENO_EN_BANDA = true;      // 19, 2026-10-31: frenadas, quietas y sin ruido

// ---------- Fase 5b: movimientos coordinados ("mv") ----------
// Las tres siguen el MISMO perfil normalizado (quíntico: velocidad y aceleración 0 en los extremos) y terminan juntas;
// J2 y J3 avanzan al mismo ritmo, así que el camino en (q2, q3) es una recta. La franja ya no es convexa (tabla del
// borde de arriba, 2026-11-04): "mv" revisa el camino entero, con la aproximación, antes de arrancar.
constexpr float MV_FACTOR_VEL = 0.5f;      // fracción de vMax/aMax de cada una para "mv" (PROVISORIO: empezar despacio)
constexpr float MV_T_MIN = 1.0f;           // s, duración mínima de un movimiento
// Llegada siempre con el ángulo creciendo (el juego de los engranajes queda del mismo lado): todas las que se mueven
// (también las que suben, desde 2026-10-31) terminan el recorrido APROX_GRADOS por debajo y suben el último tramo a
// VEL_APROX.
// Por articulación (J1, J2, J3). J1 (2026-11-03): a 2 °/s la correa y la caja 50:1 avanzan a saltos (se traba con
// 15-18 % y salta a 5-7 °/s, ~2 s por llegada; con el brazo estirado se ve en la punta): un tramo corto y rápido. En
// la aproximación cada una tiene su duración (J1 termina antes y frena; J2 y J3 siguen juntas)
constexpr float APROX_GRADOS[3] = {0.5f, 2.0f, 2.0f};
constexpr float VEL_APROX[3] = {5.0f, 2.0f, 2.0f};  // °/s (pico del perfil del último tramo)
// Sentido de la aproximación: false = siempre desde abajo (J2 y J3: el juego queda del mismo lado contra el peso);
// true = en el sentido en que viene (J1, 2026-11-03: llegando desde arriba bajaba y SUBÍA el último tramo; la correa
// cambiaba de lado y la punta, con el brazo estirado, oscilaba ±1,4 cm a ~1,5 Hz con los encoders quietos. Llegando en
// el mismo sentido, no). Para la repetibilidad alcanza: cada pose se repite siempre por el mismo camino
constexpr bool APROX_MISMO_SENTIDO[3] = {true, false, false};
// Antes del último tramo, J2/J3 tienen que estar al menos esto por debajo del objetivo (antes, la mitad del tramo: 1°).
// En los mv cortos se pasan del punto previo y bajar en seco no despega: J3 de 20 a 30 quedó en 29,10 y cortó
// (2026-11-05, trayectoria_20261105_155923_cortado; J2 de 74 a 80 quedó en 79,38, _130006_cortado). Con el doble de
// tolLlegada sigue siendo una llegada desde abajo
constexpr float APROX_ABAJO_MIN = 0.3f;  // °
// Duración propia en el recorrido: true = no espera a las otras (su perfil a su vMax/aMax con MV_FACTOR_VEL; termina
// antes y sostiene en su punto previo hasta la aproximación). J1 (2026-11-03): con la duración de J2/J3 (12,9 s para
// 17°) el final de su perfil iba tan lento que se trabó 1,6 s en 18,7°, se despegó con 28 % a 16 °/s, se pasó a 21,0°
// y volvió: la punta se hamacó ±35 px a ~1,6 Hz. J1 no tiene franja: no necesita terminar con las otras. J2 y J3 sí
// (la recta en q2-q3)
constexpr bool MV_DURACION_PROPIA[3] = {true, false, false};
// Tope de aceleración en el recorrido del "mv" (°/s², 0 = MV_FACTOR_VEL·aMax). J1 (2026-11-03): con su duración propia
// los 17° del primer mv duraban 2,1 s y frenaba a ~22 °/s²: iba 0,6° adelante de la referencia, se pasaba a 20,15° y el
// sostener la volvía (una inversión). Los mv de 40° frenan a ~10 °/s² y no se pasan. No toca el aMax de J1 (lo usan
// el sostener, "j" y el joystick) ni la aproximación
constexpr float MV_ACEL_MAX[3] = {10.0f, 0.0f, 0.0f};
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
    // Límites: 0-118° medidos con el brazo RECOGIDO; estirado choca antes con el soporte de la visión (sin medir).
    // 2026-11-03: 90 -> 118° PROVISORIO, para medir con el joystick hasta dónde llega (el usuario espera 120-130°);
    // ojo con el brazo estirado: ni "mv" ni el joystick esquivan el soporte. Con MARGEN_LIMITE, hasta 115°
    0.0f, 118.0f, 70.0f,
    0.549f, 4.0f,                         // PI de velocidad de 12 (terminada)
    17.0f, 17.0f, 17.0f, 17.0f, 0.0f,     // feedforward de 12 (sin gravedad)
    2.0f, 30.0f, 60.0f, 3.0f, 0.3f, 0.6f, 200,
    3.0f, 8.0f, 0.0f, 0.2f,               // sostener (J1 se autobloquea por la correa: frena enseguida)
    1.5f, 2.0f, 2.0f, 50.0f, 130.0f, 1500, 3.0f, 1.5f,  // homing a 1,5 °/s (2026-11-03, antes 1; rápido a 3 °/s)
    2.0f, 35.0f, 300, 25.0f, 150, 5.0f, 100, 30.0f,
    true,   // se autobloquea: al llegar, freno (como 12; con el PI de posición iba y venía por el juego de la correa)
    // tolLlegada 0,15 (2026-11-05): con 0,3 bajando a 20° frenaba siempre en 20,30 (el primer instante dentro de 0,3°).
    // El 03/11 tolPos 0,15 hacía un ciclo límite en el homing; ahora "cruzó" también es llegar y frena ahí
    0.15f,
    0.03f};  // tAnticipo: frena antes de cruzar (a 5,4 °/s desliza 0,15-0,17°)

// ---------- J2 (hombro): 36-555 58 rpm (139:1), engranajes 15 -> 75 (19, a 20 kHz)
constexpr ConfigArticulacion CFG_J2 = {
    "J2",
    PIN_J2_RPWM, PIN_J2_LPWM, PIN_BRAZO_EN, PIN_FC_J2, 2, 3, PIN_J2_ENC_A, PIN_J2_ENC_B, PCNT_UNIT_1, 20000, 10,
    16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f,  // 123,6 cuentas/°
    false, false,
    // 2026-11-04 (manivela de J3 de 95 mm): 97 -> 105° (último punto medido de la franja) -> 83°. Cerca de 90° el peso
    // casi no carga a J2: en la aproximación se pega, despega con +27 % y se pasa ~0,7° (95,65 en un mv a 95 y 90,75 en
    // uno a 90) y para volver hacia abajo no despega ni con -46 % (corte "sin cuentas", trayectoria_20261104_224409 y
    // _225335_cortado). En 74° el peso frena el salto (se pasa <= 0,15°). Con MARGEN_LIMITE, destinos hasta 80°
    0.0f, 83.0f, 70.0f,
    0.549f, 4.0f,
    35.0f, 25.0f, 22.0f, 8.0f, 8.0f,      // despegue 35 % (medido a 20 kHz), resto PROVISORIO (19)
    2.0f, 20.0f, 60.0f, 3.0f, 0.3f, 0.6f, 200,  // 2026-10-31: con tolPos 0,15 y bandaInt 0,1 J2 repetía peor (0,29°; una vez "no quedan firmes")
    3.0f, 8.0f, 0.0f, 0.2f,               // sin compensación de fricción (19: llega sin ella)
    // homing a 1,5 °/s (2026-11-03: 0,5 tardaba 6,3 s en 3°, 1 tardaba 3,4 s; 19: flanco ~0,07° a 0,5). Duty máximo 65 % (PROVISORIO): con 50 % no la levantaba desde el final
    // de carrera (2026-10-31: subió 1,5° y se trabó; sostenerla abajo pide ~25 % más el despegue)
    // 2026-11-10 (noche, pedido del usuario: J2 busca más rápido): con cero 2 -> 5 °/s hasta 4° del cero anterior
    // (antes 1,5°: más lugar para frenar); el flanco se sigue tomando a 1,5 °/s. Sin cero no cambia (no sabe dónde está)
    1.5f, 2.0f, 2.0f, 65.0f, 120.0f, 1500, 5.0f, 4.0f,
    // arranquePct ("sin cuentas") 45 -> 60 % (2026-11-04, trayectoria_20261104_234152: pegada, J2 despega con 44-48 % y
    // J3 con hasta 37 %; con 45 % se cortaba a veces, en la repetibilidad _233423_cortado). Igual que el atasco (60 %)
    2.0f, 60.0f, 300, 60.0f, 150, 5.0f, 100, 30.0f,
    false,
    // tolLlegada 0,3 -> 0,15 y freno anticipado (2026-11-05, trayectoria_20261105_113041: a 74° llegaba en 73,72 (trabada,
    // aceptada por 0,3) o 74,20-74,22 (se despegaba a 3-5 °/s y se pasaba); repetibilidad_20261105_111907 RP 0,43°)
    0.15f,
    // tAnticipo 0,03 -> 0,055 s: frenada (duty 0) J2 sigue subiendo velocidad × 0,052-0,058 s (trayectoria_20261105_114200:
    // de 73,92 a 5,1 °/s hasta 74,21; de 73,93 a 3,2 °/s hasta 74,10; de 73,95 a 3,8 °/s hasta 74,15)
    // 2026-11-10 (doble helicoidales): probado 0,01 s porque en 20 y 30° frenada casi no desliza y queda 0,13-0,19° corta
    // (trayectoria_20261110_152310; con 0,01 llega a 0,05°, _152706). Pero en la repetibilidad no rinde: peor RP de J2
    // 0,13 -> 0,16° (en 74° el rango pasa de 0,06 a 0,20°: ahí sí desliza) y J3 0,07 -> 0,09° (en (20, 6) llega a 6,07 o
    // 6,18 en vez de siempre 6,03). repetibilidad_20261110_151950 (0,055) contra _153901 (0,01). Queda 0,055
    0.055f};

// ---------- J3 (codo): 36-555 58 rpm (139:1), engranajes 15 -> 75 (19, a 20 kHz)
// El feedforward de J3 en cascada no se midió a 20 kHz (en 19 se sostenía con su lazo simple). + = el codo baja, a
// favor del peso; - lo sube contra el peso (sostener pide -17 a -27 %). PROVISORIO: ajustar con "ff 3 ...".
constexpr ConfigArticulacion CFG_J3 = {
    "J3",
    PIN_J3_RPWM, PIN_J3_LPWM, PIN_BRAZO_EN, PIN_FC_J3, 4, 5, PIN_J3_ENC_A, PIN_J3_ENC_B, PCNT_UNIT_2, 20000, 10,
    16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f,
    false, true,                          // 19: motor y encoder invertidos juntos
    // 2026-11-04 (manivela de 95 mm): 113 -> 101,5°, el borde de la franja en q2 = 105 (traba en 103,6° - 2°)
    0.0f, 101.5f, 70.0f,
    0.549f, 4.0f,
    20.0f, 10.0f, 40.0f, 30.0f, 0.0f,     // PROVISORIO (sin medir)
    // tolPos 0,3 -> 0,15 (2026-10-31: repite en 0,03° en vez de 0,30°) -> 0,20 (2026-11-05): "firme" con el mismo umbral
    // que la llegada (tolLlegada 0,15) no tenía margen: llegó a 0,149, se corrió a 0,156 y tardó 12 s en quedar firme
    // (trayectoria_20261105_162907, (65, 60)). La llegada sigue en 0,15
    2.0f, 20.0f, 60.0f, 3.0f, 0.20f, 0.6f, 200,
    3.0f, 8.0f, 15.0f, 0.1f,              // compensación de fricción 15 % (19: no se pasa, 0,09°); bandaInt 0,2 -> 0,1
    1.5f, 2.0f, 2.0f, 50.0f, 120.0f, 1500, 2.0f, 1.5f,  // homing a 1,5 °/s (2026-11-03; 19: flanco ~0,04° a 1)
    // arranquePct ("sin cuentas") 45 -> 60 % (2026-11-04, trayectoria_20261104_234152: pegada, J2 despega con 44-48 % y
    // J3 con hasta 37 %; con 45 % se cortaba a veces, en la repetibilidad _233423_cortado). Igual que el atasco (60 %)
    2.0f, 60.0f, 300, 60.0f, 150, 5.0f, 100, 30.0f,
    false,
    0.15f,   // tolLlegada = tolPos
    // tAnticipo 0 -> 0,045 s (2026-11-05, trayectoria_20261105_193535): subiendo a (30, 20) J3 cruzaba el objetivo a
    // 3,0-3,2 °/s y seguía 0,137° (velocidad × 0,042-0,058 s): las 4 llegadas en 20,13; la primera a (20, 14) en 14,13
    0.045f};
