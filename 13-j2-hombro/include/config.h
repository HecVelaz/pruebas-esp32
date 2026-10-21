#pragma once

#include <stdint.h>

// Parámetros de J2 (hombro, barra verde). Los pines están en pins.h.
// Motor DEFINITIVO: 36-555 de 58 rpm (139:1) con encoder SCX-555 y engranajes helicoidales 15 -> 75 (5:1, confirmado
// el 17/10 en recolector_de_frutas). SIN CARACTERIZAR: lo marcado PROVISORIO sale de J1 o de la prueba del 36GP-555 de
// 160 rpm en el hombro (10, 2026-10-13) y se reemplaza con lo que se mida en cada paso (README.md).
// J3 (barra roja) queda quieta y frenada durante estas pruebas: el cuatro barras limita cuánto puede girar J2
// (tools/cuatro_barras.py, con los ángulos de la verde y la roja medidos en la marca).
namespace cfg_j2 {
constexpr const char *NOMBRE = "J2";
constexpr const char *PIEZA = "hombro";
constexpr const char *SENTIDO_POS = "HACIA ARRIBA";
constexpr const char *SENTIDO_NEG = "HACIA ABAJO";
constexpr const char *MARCA = "barra verde en su marca";
constexpr bool SIGNOS_EN_VIVO = true;  // "ie"/"is" habilitados: los límites del paso 0 son simétricos
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

// Paso 0: signos (positivo = el brazo sube). SIN MEDIR: se ajustan en vivo con "ie" e "is" y se copian acá.
//  - ENCODER_INVERTIDO: true si con duty positivo las cuentas bajan.
//  - SENTIDO_INVERTIDO: true si, con el encoder ya de acuerdo, el duty positivo baja el brazo.
constexpr bool ENCODER_INVERTIDO = false;
constexpr bool SENTIDO_INVERTIDO = false;

// PWM del IBT-2: 20 kHz como J1 y el firmware del robot. Si a duty bajo rinde poco (con el 5840 en 10, a 20 kHz el
// IBT-2 entregaba ~1/3 de la tensión esperada), probar 1000.
constexpr uint32_t PWM_FREQ_HZ = 20000;
constexpr uint8_t PWM_BITS = 10;

// Límites PROVISORIOS desde la marca, simétricos para poder usar "ie"/"is" en el paso 0. Después del paso 0, medir
// la postura en la marca y calcular los definitivos con tools/cuatro_barras.py (con 10° de margen al punto muerto).
constexpr float LIMITE_POS_GRADOS = 15.0f;
constexpr float LIMITE_NEG_GRADOS = -15.0f;
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

namespace art = cfg_j2;
