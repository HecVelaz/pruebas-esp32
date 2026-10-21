#pragma once

#include <stdint.h>

// Parámetros de J1 (base giratoria). Los pines están en pins.h.
// Copiados de 10-control-lazo-cerrado (2026-10-20): ahí J1 ya pasó los pasos 0-5 con el mismo motor, la misma correa
// y los mismos pines (21/22, encoder 34/35). Lo que cambia ahora es que J2 y J3 están conectadas al mismo EN.
// Las pruebas de esta carpeta confirman esos valores con el brazo completo armado.

namespace cfg_j1 {
constexpr const char *NOMBRE = "J1";
constexpr const char *PIEZA = "base";
constexpr const char *SENTIDO_POS = "ANTIHORARIO visto desde arriba";
constexpr const char *SENTIDO_NEG = "HORARIO visto desde arriba";
constexpr const char *MARCA = "brazo al costado del robot";
constexpr bool SIGNOS_EN_VIVO = false;  // "ie"/"is" deshabilitados: los límites son asimétricos
// En 10 era false (rueda libre al terminar: la base no carga peso). Con el EN común, rueda libre suelta también J2 y
// J3, que no se autobloquean: frenado, el brazo cae mucho más despacio. Igual, apoyar el brazo en su tope.
constexpr bool FRENO_EN_REPOSO = true;
constexpr bool SOSTENER_AUTO = false;  // "sa" al arrancar: se autobloquea por la correa y el 50:1 alcanza

// Encoder Hall 16 PPR x4 en el eje del motor, reductora 50:1 -> 3200 cuentas por vuelta de la salida
// de la reductora. De ahí a la base: correa GT2, polea de 18 dientes en el motor y 90 en la base (5:1).
constexpr const char *MOTOR = "36GP-555 160 rpm";
constexpr float REDUCTORA = 50.0f;
constexpr float CUENTAS_POR_VUELTA_SALIDA = 16.0f * 4.0f * REDUCTORA;
constexpr float RELACION_TRANSMISION = 90.0f / 18.0f;  // correa
// 16 000 cuentas por vuelta de la base = 44,4 por grado. Verificado en el paso 1 (m 45 paralelo a un
// doblez de 45°, error de pocos grados como mucho, a ojo).
constexpr float CUENTAS_POR_GRADO = CUENTAS_POR_VUELTA_SALIDA * RELACION_TRANSMISION / 360.0f;

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
// Sostener quieto ("s", "sa"): J1 no lo necesita (se autobloquea), valores de 10
constexpr float KHP_SOSTENER = 6.0f;   // % por grado
constexpr float KHI_SOSTENER = 15.0f;  // % por grado y por segundo
}  // namespace cfg_j1

namespace art = cfg_j1;
