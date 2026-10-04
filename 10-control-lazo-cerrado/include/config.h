#pragma once

#include <stdint.h>

// Parámetros de cada articulación. El firmware se compila para una sola: -e j1 (base), -e j2 (hombro) o -e j3 (codo);
// "art" apunta al espacio de nombres de la elegida. Los pines están en pins.h.

// ===================== J1 (base giratoria): motor 36GP-555 + IBT-2 =====================
namespace cfg_j1 {
constexpr const char *NOMBRE = "J1";
constexpr const char *PIEZA = "base";
constexpr const char *SENTIDO_POS = "ANTIHORARIO visto desde arriba";
constexpr const char *SENTIDO_NEG = "HORARIO visto desde arriba";
constexpr const char *MARCA = "brazo al costado del robot";
constexpr bool SIGNOS_EN_VIVO = false;  // "ie"/"is" deshabilitados: los límites son asimétricos

// Encoder Hall 16 PPR x4 en el eje del motor, reductora 50:1 -> 3200 cuentas por vuelta de la salida
// de la reductora. De ahí a la base: correa GT2, polea de 18 dientes en el motor y 90 en la base (5:1).
constexpr float CUENTAS_POR_VUELTA_SALIDA = 16.0f * 4.0f * 50.0f;
constexpr float RELACION_CORREA = 90.0f / 18.0f;
// 16 000 cuentas por vuelta de la base = 44,4 por grado. Verificado en el paso 1 (m 45 paralelo a un
// doblez de 45°, error de pocos grados como mucho, a ojo).
constexpr float CUENTAS_POR_GRADO = CUENTAS_POR_VUELTA_SALIDA * RELACION_CORREA / 360.0f;

// Paso 0: signos. Se ajustan con los comandos "ie" e "is" y después se copian acá.
//  - ENCODER_INVERTIDO: true si con duty positivo las cuentas bajan.
//  - SENTIDO_INVERTIDO: true si, con el encoder ya de acuerdo, el duty positivo mueve la base al
//    revés del sentido positivo de q1 (antihorario visto desde arriba). Invierte motor y encoder juntos,
//    así siguen de acuerdo.
constexpr bool ENCODER_INVERTIDO = false;
constexpr bool SENTIDO_INVERTIDO = true;   // prueba 0 (2026-10-02): duty + giraba horario

// PWM del IBT-2 (igual que en 06)
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;

// Seguridad mientras no haya finales de carrera. Límites de software desde el cero (brazo al costado),
// pedidos por el usuario (2026-10-02): +135° antihorario, -45° horario visto desde arriba.
constexpr float LIMITE_POS_GRADOS = 135.0f;
constexpr float LIMITE_NEG_GRADOS = -45.0f;
constexpr float DUTY_MAX_PCT = 40.0f;      // ningún movimiento de estas pruebas pasa de este duty
constexpr uint32_t PULSO_MAX_MS = 1000;    // duración máxima de un pulso
constexpr uint32_t PULSO_DEF_MS = 200;     // duración de "p" si no se da
constexpr int32_t CUENTAS_SIN_GIRO = 5;    // menos que esto en un pulso = no giró
constexpr uint32_t SIN_CUENTAS_MS = 150;   // ventana de avance: en este tiempo, con duty aplicado,
constexpr int32_t AVANCE_MIN = 3;          // la base tiene que avanzar al menos esto en el sentido pedido

// Paso 1: comando "m <grados>" (ir a un ángulo con duty fijo, sin control: frena al llegar)
constexpr float DUTY_MOVER_PCT = 30.0f;    // con 25 % casi no arranca con la carga del brazo
constexpr uint32_t MOVER_MAX_MS = 15000;   // a 30 % la base va a ~10-15 °/s

// Paso 2: PI de velocidad de la base. Diseño (tools/diseno_velocidad.py, cancelación de polo con Ti = tau,
// modelo K = 1,90 (°/s)/%, tau = 65 ms) para ts = 0,25 s: Kp = 0,549, Ki = 8,44. En placa el juego de la
// correa (no está en el modelo) daba 18 % de sobrepico: con Ki = 4 (80 escalones, 2026-10-02) quedó en
// ts 0,33-0,38 s y sobrepico 4-7 %. Especificación ajustada: ts <= 0,4 s, sobrepico <= 10 %.
constexpr float KP_VEL = 0.549f;           // % de duty por °/s de error
constexpr float KI_VEL = 4.0f;             // % de duty por ° de error acumulado
constexpr float ZONA_MUERTA_PCT = 17.0f;   // feedforward: duty con el que la base apenas se mueve
constexpr float ZONA_MUERTA_NEG_PCT = ZONA_MUERTA_PCT;  // feedforward con velocidad negativa (simétrico)
constexpr float DESPEGUE_PCT = ZONA_MUERTA_PCT;          // feedforward con la articulación quieta (igual)
constexpr float DESPEGUE_NEG_PCT = ZONA_MUERTA_NEG_PCT;
constexpr float FF_PENDIENTE_POS = 0.0f;  // sin compensación de gravedad
constexpr float ARRANQUE_PCT = 25.0f;      // duty para arrancar desde quieta (fricción de arranque, pasos 0-3).
                                              // Debajo de esto el motor no la mueve: el corte "sin cuentas" no actúa
constexpr uint32_t TS_VEL_US = 10000;      // lazo de velocidad a 100 Hz
constexpr int VENTANA_VEL = 4;             // la velocidad se mide en 4 muestras (40 ms): 0,56 °/s por cuenta
constexpr float VEL_MAX = 40.0f;           // °/s de la base, máximo que acepta el escalón
constexpr float VEL_MIN = 5.0f;            // y mínimo (más lento, el corte sin cuentas tarda demasiado)
constexpr float CONTRARIO_VEL = 5.0f;      // corte si va al revés de lo pedido a más de esto (°/s) ...
constexpr uint32_t CONTRARIO_MS = 100;     // ... durante este tiempo
constexpr uint32_t ESCALON_MAX_MS = 3000;  // duración máxima de cada mitad del escalón
constexpr uint32_t PAUSA_MS = 500;         // quieto entre escalones: no invierte de golpe (juego de la correa)
constexpr uint32_t CICLOS_MAX = 60;        // ciclos +v, 0, -v, 0 por prueba (40 ciclos de 3 s = 2 min)
constexpr float ATASCO_VEL = 2.0f;         // atasco: menos de esto (°/s) ...
constexpr float ATASCO_DUTY_PCT = 35.0f;   // ... con este duty o más ...
constexpr uint32_t ATASCO_MS = 300;        // ... durante este tiempo

// Paso 3: P de posición encima del PI de velocidad (tools/diseno_posicion.py). Lazo de velocidad cerrado
// ~ 1/(0,09 s + 1) (medido en el paso 2): sin sobrepico hace falta Kpp <= 1/(4*0,09) = 2,78. Elegido 2
// (margen por el juego de la correa; igual que brazo_config.h). Simulado: 0 -> 20° en 1,7 s, sin sobrepico.
constexpr float KPP = 2.0f;                // (°/s) de velocidad pedida por ° de error
constexpr uint32_t TS_POS_US = 20000;      // lazo de posición a 50 Hz
constexpr float VMAX_POS = 30.0f;          // °/s, velocidad máxima de los movimientos
constexpr float AMAX_POS = 60.0f;          // °/s², cambio máximo de la velocidad pedida (sin golpes)
constexpr float VMIN_POS = 3.0f;           // °/s, mínimo mientras no llegó: más lento se traba antes de llegar
constexpr float VMIN_BAJA_POS = VMIN_POS;  // °/s, mínimo bajando (velocidad negativa); se cambia con "bj"
constexpr uint32_t TRABADA_MS = 100;       // en marcha pero quieta este tiempo: vuelve a la rampa de despegue ("bj")
constexpr float DITHER_PCT = 0.0f;         // ± % de vibración en el duty mientras se mueve ("bj")
constexpr float TOL_POS = 0.3f;            // °, a menos de esto del objetivo: llegó, velocidad 0 (freno)
constexpr float TOL_SALIDA = 0.6f;         // °, una vez que llegó, solo vuelve a moverse si se aleja más que
                                              // esto (histéresis: sin ella zumbaba en el borde de la tolerancia)
constexpr uint32_t LLEGADA_MS = 200;       // llegó: dentro de la tolerancia y quieta durante esto
constexpr uint32_t IR_MAX_MS = 15000;      // "a <grados>": tiempo máximo para llegar
constexpr uint32_t EP_SEG_MAX_MS = 5000;   // "ep": duración máxima de cada tramo
constexpr uint32_t EP_CICLOS_MAX = 20;
// Empujón de arranque de "m" y "p": desactivado (0)
constexpr float KICK_PCT = 0.0f;
constexpr uint32_t KICK_MS = 0;
constexpr int32_t KICK_CUENTAS = 0;
constexpr bool KICK_SOLO_POSITIVO = true;
}  // namespace cfg_j1

// ===================== J2 (hombro): motor 5840-31ZY (sin fin) + L298N canal A =====================
// Mueve la barra verde con un piñón y un engranaje grande en el eje del hombro. J3 (barra roja) queda quieto:
// cada grado de J2 cambia en un grado el ángulo entre la roja y la verde (gamma4). Con la roja de 60 mm el
// cuatro barras funciona bien con gamma4 entre -112° y -30° (tools/cuatro_barras.py): fuera de eso, punto muerto.
namespace cfg_j2 {
constexpr const char *NOMBRE = "J2";
constexpr const char *PIEZA = "brazo";
constexpr const char *SENTIDO_POS = "HACIA ARRIBA";
constexpr const char *SENTIDO_NEG = "HACIA ABAJO";
constexpr const char *MARCA = "barra verde en su marca";
constexpr bool SIGNOS_EN_VIVO = true;  // "ie"/"is" habilitados: los límites provisorios son simétricos

// Encoder 38S6G5 (1000 PPR x4) acoplado al eje de SALIDA del sin fin: 4000 cuentas por vuelta de esa salida.
// De ahí al hombro: piñón -> engranaje grande. MEDIR en el paso 1 (dientes del grande / dientes del piñón).
// Mientras valga 1, los "grados" del firmware son de la salida del sin fin: el brazo se mueve MENOS que eso
// (si el engranaje reduce), así que los límites provisorios quedan del lado seguro.
constexpr float CUENTAS_POR_VUELTA_SALIDA = 1000.0f * 4.0f;
constexpr float RELACION_ENGRANAJE = 1.0f;  // MEDIR
constexpr float CUENTAS_POR_GRADO = CUENTAS_POR_VUELTA_SALIDA * RELACION_ENGRANAJE / 360.0f;

// Paso 0: signos (positivo = el brazo sube). En 08, con este acople, las cuentas iban al revés.
constexpr bool ENCODER_INVERTIDO = true;   // paso 0 (2026-10-03): con duty + las cuentas subían (OK)
constexpr bool SENTIDO_INVERTIDO = true;   // paso 0 (2026-10-03): con duty + el brazo bajaba

// PWM del L298N en modo freno (igual que en 08: lineal a 1 kHz)
constexpr uint32_t PWM_FREQ_HZ = 1000;
constexpr uint8_t PWM_BITS = 10;

// Límites PROVISORIOS desde la marca, hasta medir la postura (ángulos de la verde y de la roja) y calcular la
// ventana del cuatro barras con tools/cuatro_barras.py.
constexpr float LIMITE_POS_GRADOS = 15.0f;
constexpr float LIMITE_NEG_GRADOS = -15.0f;
constexpr float DUTY_MAX_PCT = 55.0f;      // bajando arrancó con 35 %; subiendo (contra la gravedad) hace falta más
constexpr uint32_t PULSO_MAX_MS = 200;     // el sin fin va rápido: a 40 % la salida gira a ~350 °/s
constexpr uint32_t PULSO_DEF_MS = 100;
constexpr int32_t CUENTAS_SIN_GIRO = 5;
constexpr uint32_t SIN_CUENTAS_MS = 150;
constexpr int32_t AVANCE_MIN = 3;

// Paso 1: "m <grados>"
constexpr float DUTY_MOVER_PCT = 30.0f;    // VERIFICAR con el arranque del paso 0
constexpr uint32_t MOVER_MAX_MS = 10000;

// Paso 2: PI de velocidad. Modelo de 08 (modo freno): K = 1,33 rpm/% de la salida del sin fin = 7,98 (°/s)/%,
// tau = 59 ms. Ganancias PROVISORIAS por cancelación de polo para ts = 0,25 s (como J1): Kp = tau/(K·ts/4),
// Ki = Kp/tau. Se rehacen con tools/diseno_velocidad.py cuando se conozcan la transmisión y la zona muerta.
constexpr float K_VEL = 1.33f * 6.0f / RELACION_ENGRANAJE;  // (°/s del brazo) por %
constexpr float TAU_VEL = 0.059f;
constexpr float KP_VEL = TAU_VEL / (K_VEL * 0.25f / 4.0f);
constexpr float KI_VEL = KP_VEL / TAU_VEL;
constexpr float ZONA_MUERTA_PCT = 30.0f;   // MEDIR en el paso 0 (subiendo y bajando: la gravedad cambia)
constexpr float ZONA_MUERTA_NEG_PCT = ZONA_MUERTA_PCT;  // feedforward con velocidad negativa (simétrico)
constexpr float DESPEGUE_PCT = ZONA_MUERTA_PCT;          // feedforward con la articulación quieta (igual)
constexpr float DESPEGUE_NEG_PCT = ZONA_MUERTA_NEG_PCT;
constexpr float FF_PENDIENTE_POS = 0.0f;  // sin compensación de gravedad
constexpr float ARRANQUE_PCT = 40.0f;      // MEDIR en el paso 0
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
constexpr float ATASCO_DUTY_PCT = 50.0f;   // por encima de la zona muerta y debajo de DUTY_MAX_PCT
constexpr uint32_t ATASCO_MS = 300;

// Paso 3: P de posición (provisorio, como J1)
constexpr float KPP = 2.0f;
constexpr uint32_t TS_POS_US = 20000;
constexpr float VMAX_POS = 15.0f;
constexpr float AMAX_POS = 60.0f;
constexpr float VMIN_POS = 3.0f;
constexpr float VMIN_BAJA_POS = VMIN_POS;  // °/s, mínimo bajando (velocidad negativa); se cambia con "bj"
constexpr uint32_t TRABADA_MS = 100;       // en marcha pero quieta este tiempo: vuelve a la rampa de despegue ("bj")
constexpr float DITHER_PCT = 0.0f;         // ± % de vibración en el duty mientras se mueve ("bj")
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
}  // namespace cfg_j2

// ===================== J3 (codo): motor 5840-31ZY (sin fin) + L298N =====================
// Mueve la barra roja (balancín de 60 mm) con un piñón y un engranaje grande en el eje del hombro; la roja empuja
// el acoplador y este el antebrazo. J2 (barra verde) queda quieto: cada grado de J3 cambia en un grado gamma4.
// EN EL BANCO (2026-10-03) va cableado en el canal A, con los pines de J2 (25/26/27 y encoder 32/33); en el robot
// va en el canal B (13/16/17 y 18/19). Con la roja de 60 mm el
// cuatro barras funciona bien con gamma4 entre -112° y -30° (tools/cuatro_barras.py): fuera de eso, punto muerto.
namespace cfg_j3 {
constexpr const char *NOMBRE = "J3";
constexpr const char *PIEZA = "antebrazo";
constexpr const char *SENTIDO_POS = "PINZA HACIA ARRIBA";
constexpr const char *SENTIDO_NEG = "PINZA HACIA ABAJO";
constexpr const char *MARCA = "barra roja en su marca";
constexpr bool SIGNOS_EN_VIVO = true;

// Encoder 38S6G5 (1000 PPR x4) acoplado al eje de SALIDA del sin fin: 4000 cuentas por vuelta de esa salida.
// De ahí al hombro: piñón -> engranaje grande. MEDIR en el paso 1 (dientes del grande / dientes del piñón).
// Mientras valga 1, los "grados" del firmware son de la salida del sin fin: el brazo se mueve MENOS que eso
// (si el engranaje reduce), así que los límites provisorios quedan del lado seguro.
constexpr float CUENTAS_POR_VUELTA_SALIDA = 1000.0f * 4.0f;
constexpr float RELACION_ENGRANAJE = 56.0f / 18.0f;  // paso 1 (2026-10-03): piñón 18, engranaje 56 dientes
constexpr float CUENTAS_POR_GRADO = CUENTAS_POR_VUELTA_SALIDA * RELACION_ENGRANAJE / 360.0f;

// DRIVER: IBT-2 desde el 2026-10-03 (antes L298N, que con 12 V daba 9,6 V al motor). Con el IBT-2 el motor recibe
// ~12 V: a igual duty va ~25 % más rápido. Los duty de abajo son los del L298N x 0,8, PROVISORIOS: se vuelven a
// medir en el paso 0 (plan_j3.html, conexiones_j3.html).

// Paso 0: signos (positivo = la pinza sube). Con el L298N (2026-10-03): encoder invertido, sentido normal.
// Con el IBT-2 el encoder sigue igual (mismos A y B); el sentido depende de cómo vayan M+ y M-: VERIFICAR.
constexpr bool ENCODER_INVERTIDO = true;
constexpr bool SENTIDO_INVERTIDO = false;  // VERIFICAR con el IBT-2 ("is" lo cambia en vivo)

// PWM del IBT-2: 1 kHz (2026-10-03). A 20 kHz (como J1) el BTS7960 parecía comerse parte de cada pulso (0,6 V en
// vez de 1,8 V a 15 %) y subiendo no seguía ni con 40 %; con el L298N a 1 kHz subía con 40 %.
constexpr uint32_t PWM_FREQ_HZ = 1000;
constexpr uint8_t PWM_BITS = 10;

// Límites PROVISORIOS desde la marca, hasta medir la postura (ángulos de la verde y de la roja) y calcular la
// ventana del cuatro barras con tools/cuatro_barras.py.
// Postura medida (2026-10-03): ~50° entre la verde y la roja (gamma4 ~ -50°). Subir la pinza abre ese ángulo
// (verificado a ojo): hacia arriba hay ~52° hasta el borde con 10° de margen, hacia abajo ~10°
// (tools/cuatro_barras.py --q2 60 --theta 110). El cero (z) se fija en esa postura.
// 2026-10-03: barra roja a 112° en la marca (inclinómetro: 68° medido del otro lado), verde ~60° ->
// tools/cuatro_barras.py --q2 60 --theta 112: puntos muertos a ~-22° y ~+60° desde la marca. Límites a 5-7° de ellos
// (llegó a -17,5° sin trabarse). Si J2 se mueve, recalcular. Rango de trabajo elegido por el usuario (2026-10-03):
// -12° a +45° (los +70° anteriores pasaban el punto muerto calculado de arriba, ~+60°).
// Rango -12° / +45° pedido; en el barrido de carga a +34° cortó por atasco con ~80 % y 3,3 A, y a +25° llega a
// ~62-68 % y 2 A (límite de la fuente). Rango útil: -12° / +25°.
constexpr float LIMITE_POS_GRADOS = 28.0f;  // "m", "a" y "bc" llegan hasta 3° antes
constexpr float LIMITE_NEG_GRADOS = -15.0f;
// TEMPORAL (2026-10-03): 80 % a pedido del usuario. Con el IBT-2, desde quieta arranca con 75-80 % (con 40 % no).
// Quedan activos los límites de posición y el corte "sin cuentas" (150 ms). Volver a 40-55 % después del paso 0.
constexpr float DUTY_MAX_PCT = 80.0f;      // L298N: 50 %. Subiendo hacía falta 40 % del L298N (~32 % acá)
constexpr uint32_t PULSO_MAX_MS = 300;     // el sin fin va rápido: a 40 % la salida gira a ~350 °/s
constexpr uint32_t PULSO_DEF_MS = 100;
constexpr int32_t CUENTAS_SIN_GIRO = 5;
constexpr uint32_t SIN_CUENTAS_MS = 150;
constexpr int32_t AVANCE_MIN = 3;

// Paso 1: "m <grados>"
constexpr float DUTY_MOVER_PCT = 60.0f;    // a 1 kHz sube a ~19 °/s. Bajando conviene "m <°> 25" (con 60 % va rápido)
constexpr uint32_t MOVER_MAX_MS = 10000;

// Paso 2: PI de velocidad. Paso 0 con el IBT-2 a 1 kHz (2026-10-03, pulsos de 300 ms, velocidades medias):
//   subiendo: 40 % se clava, 45 % ~2 °/s, 50 % ~7 °/s, 60 % ~19 °/s, 70 % ~26 °/s  -> ~1,1 (°/s)/% sobre ~44 %
//   bajando: -10 y -15 % no arrancan, -20 % ~22 °/s, -30 % ~37 °/s                  -> ~1,5 (°/s)/% sobre ~18 %
// La diferencia es el peso del antebrazo: ayuda al bajar y frena al subir. Por eso el feedforward es asimétrico.
// Ganancias PROVISORIAS por cancelación de polo para ts = 0,25 s (como J1): Kp = tau/(K·ts/4), Ki = Kp/tau, con
// K = 1,3 (promedio de los dos sentidos) y tau = 59 ms (de 08). Se rehacen con los escalones de velocidad.
constexpr float K_VEL = 1.3f;              // (°/s de la barra roja) por %
constexpr float TAU_VEL = 0.059f;
// En placa (2026-10-03, ev 10, 3 ciclos): con Kp 0,73 / Ki 12,3 oscilaba todo el escalón (~8 Hz); Ki 3 quedaba
// lento (subiendo, 5-6 °/s). Elegido Kp 0,5 / Ki 5: oscila menos y llega a ~10 °/s. Subiendo, la carga cambia con la
// posición (de 0 a +10°, el duty para 10 °/s pasa de ~25 a ~37 %) y el integrador la sigue con atraso.
constexpr float KP_VEL = 0.35f;  // diseño: TAU_VEL / (K_VEL * 0.25f / 4.0f) = 0,73. Con 0,35 (y ff 14/9 bajando) se
                                 // traba menos al bajar de +10° a 0° (1 traba por bajada en vez de 2-3)
constexpr float KI_VEL = 5.0f;   // diseño: KP_VEL / TAU_VEL = 12,3
// Feedforward en dos partes (primer "ev 10 1000 3", 2026-10-03): con 44 % / 18 % fijos, al arrancar se disparaba a
// ~50 °/s subiendo y ~30 °/s bajando (sobrepico de 200-400 %) y después oscilaba trabándose y soltándose. Despegar
// pide mucho más que mantenerlo en marcha: en régimen, a 10 °/s, el duty quedó en ~40-46 % subiendo y ~-9 % bajando.
//  - DESPEGUE: con la barra quieta (|w medida| < VEL_QUIETO), lo que hace falta para que arranque.
//  - ZONA_MUERTA (marcha): ya en movimiento. Marcha = duty en régimen - K·10 °/s.
// Compensación de gravedad (barridos de carga a 4 °/s, 2026-10-03, resultados/barrido_*): subiendo, el duty crece
// casi lineal con el ángulo, ~1,4 %/° (dos barridos: 22 y 32 % en +1,5°, 37 y 47 % en +10,5°, 62 % en +22,5°;
// la diferencia entre barridos la corrige el integrador). Bajando es casi constante: ~-5 a -7 % en todo el rango.
// Subiendo: marcha(θ) = ZONA_MUERTA_PCT + FF_PENDIENTE_POS·θ y despegue(θ) = DESPEGUE_PCT + FF_PENDIENTE_POS·θ.
constexpr float FF_PENDIENTE_POS = 1.4f;      // %/° subiendo
constexpr float ZONA_MUERTA_PCT = 24.0f;      // en marcha, subiendo, en 0° (antes 33 fijo: sobraba abajo y faltaba arriba)
constexpr float ZONA_MUERTA_NEG_PCT = 9.0f;   // en marcha, bajando (3 y 6 %: el duty caía debajo de la fricción)
constexpr float DESPEGUE_PCT = 31.0f;         // quieta, para subir, en 0° (marcha + 7, como el 40/33 anterior)
constexpr float DESPEGUE_NEG_PCT = 14.0f;     // quieta, para bajar (18: se soltaba de golpe)
constexpr float ARRANQUE_PCT = 20.0f;      // con menos que esto no se mueve en ningún sentido: el corte "sin cuentas"
                                           // no actúa (no hay peligro aunque el encoder esté suelto)
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
constexpr float ATASCO_DUTY_PCT = 70.0f;   // por encima de la zona muerta subiendo y debajo de DUTY_MAX_PCT
constexpr uint32_t ATASCO_MS = 300;

// Paso 3: P de posición (provisorio, como J1)
constexpr float KPP = 2.0f;
constexpr uint32_t TS_POS_US = 20000;
constexpr float VMAX_POS = 25.0f;          // 15 hasta el 2026-10-03; 25 a pedido del usuario
constexpr float AMAX_POS = 60.0f;
constexpr float VMIN_POS = 3.0f;
constexpr float VMIN_BAJA_POS = 6.0f;      // °/s, mínimo bajando; con 3 se clavaba al final de la bajada ("bj")
constexpr uint32_t TRABADA_MS = 40;        // trabada este tiempo: vuelve a la rampa de despegue (100 ms: paradas de
                                           // 110-130 ms; 40 ms: de 50-60 ms) ("bj")
constexpr float DITHER_PCT = 0.0f;         // ± % de vibración en el duty mientras se mueve ("bj")
constexpr float TOL_POS = 0.3f;
constexpr float TOL_SALIDA = 0.6f;
constexpr uint32_t LLEGADA_MS = 200;
constexpr uint32_t IR_MAX_MS = 15000;
constexpr uint32_t EP_SEG_MAX_MS = 5000;
constexpr uint32_t EP_CICLOS_MAX = 20;
// Empujón de arranque de "m" y "p" (2026-10-03, con el L298N): subiendo, con 40-50 % no despegaba desde quieta
// (fricción del sin fin + peso del antebrazo), pero en movimiento alcanzaba con 40 %. Con el L298N el motor recibía
// 9,6 V; directo a 12 V (2026-10-03) sube "con mucha fuerza". Con el IBT-2 (2026-10-03): desde quieta no arranca
// con 40-70 % y sí con 75-80 % (fricción estática del sin fin + peso), igual que el empujón de 75 % del L298N.
// Empujón de 80 % para medir con qué duty SIGUE moviéndose una vez que arrancó.
// Arranca con KICK_PCT hasta que el encoder cuenta KICK_CUENTAS o pasan KICK_MS, y después sigue con el duty pedido.
constexpr float KICK_PCT = 80.0f;
constexpr uint32_t KICK_MS = 150;
constexpr int32_t KICK_CUENTAS = 5;
constexpr bool KICK_SOLO_POSITIVO = true;  // bajando, la gravedad ayuda: con 30 % alcanza
}  // namespace cfg_j3

#if defined(ARTICULACION_J3)
namespace art = cfg_j3;
#elif defined(ARTICULACION_J2)
namespace art = cfg_j2;
#else
namespace art = cfg_j1;
#endif
