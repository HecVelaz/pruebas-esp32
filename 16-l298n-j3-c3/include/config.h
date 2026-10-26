#pragma once

#include <stdint.h>

// Parámetros de la prueba. Los de J3 (encoder, transmisión) son los de 14-j3-codo/include/config.h.
namespace cfg {
// Encoder SCX-555 (16 PPR x4) en el eje del motor, reductora 139:1, engranajes 15 -> 75 (5:1)
constexpr float CUENTAS_POR_GRADO = 16.0f * 4.0f * 139.0f * (75.0f / 15.0f) / 360.0f;  // 123,6

// PWM del L298N: 1 kHz como en 08 (es bipolar y lento: a 20 kHz pierde mucho a duty bajo). Modo freno de lib/L298N:
// ENA fijo en alto y el PWM en IN1 (positivo) o IN2 (negativo).
constexpr uint32_t PWM_FREQ_HZ = 1000;
constexpr uint8_t PWM_BITS = 10;

// J3 es planetario y no se autobloquea: frenado (ENA alto, IN1 = IN2 = 0) el antebrazo cae más despacio que suelto
constexpr bool FRENO_EN_REPOSO = true;

// Seguridad: el motor solo se mueve en pulsos cortos, con duty limitado, y no sale de ±LIMITE_GRADOS desde el cero.
// A 100 % el codo va a ~70 °/s (58 rpm / 5 x 6): a 60 % y 500 ms son ~20° como mucho por pulso.
constexpr float DUTY_MAX_PCT = 60.0f;
constexpr float DUTY_DEF_PCT = 25.0f;     // con IBT-2 (14) la zona muerta es ~17 % subiendo; el L298N pierde ~2 V
constexpr uint32_t PULSO_MAX_MS = 500;
constexpr uint32_t PULSO_DEF_MS = 150;
constexpr float LIMITE_GRADOS = 15.0f;    // como el provisorio de 14 (cuatro barras): ±15° desde donde se hizo "z"

// Si en SIN_CUENTAS_MS el encoder no cuenta al menos CUENTAS_SIN_GIRO, se corta (motor trabado o encoder mal).
// Con "se" se apaga este control para probar el driver sin encoder: entonces manda el tiempo neto de pulsos.
constexpr uint32_t SIN_CUENTAS_MS = 150;
constexpr int32_t CUENTAS_SIN_GIRO = 5;

// Respaldo por tiempo (por si el encoder falla): suma de pulsos positivos menos negativos desde el último "z"
constexpr uint32_t NETO_MAX_MS = 3000;          // con el encoder controlando
constexpr uint32_t NETO_MAX_SIN_ENC_MS = 600;   // sin encoder ("se"): ~25° a 60 %, menos a duty bajo

// Diagnóstico ("m"), SOLO con el motor desconectado de OUT1 / OUT2: duty fijo hasta una tecla, para medir con el
// multímetro. Corta a los DIAG_MAX_MS, o si el encoder cuenta CUENTAS_SIN_GIRO (el motor seguía conectado).
constexpr uint32_t DIAG_MAX_MS = 30000;
}  // namespace cfg
