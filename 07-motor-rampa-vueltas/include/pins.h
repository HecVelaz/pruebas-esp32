#pragma once

// Placa: Freenove ESP32-S3-WROOM (N16R8) sobre breakout Freenove v1.2.
// Evitar GPIO19/20 (USB nativo) y GPIO26-37 (flash y PSRAM octal del N16R8).
// Pines elegidos para no chocar con el IMU (8/9), el GPS (41/42/47) ni el lidar (14/21),
// así se pueden conectar todos juntos más adelante. GPIO4-7 y 15/16 son de la cámara, que no se usa.

// Driver IBT-2 (doble BTS7960). Su VCC va a 3V3 del ESP32 (ver README).
constexpr int PIN_RPWM = 4;  // PWM del medio puente derecho: motor hacia adelante
constexpr int PIN_LPWM = 5;  // PWM del medio puente izquierdo: motor hacia atrás
constexpr int PIN_R_EN = 6;  // habilitación del medio puente derecho
constexpr int PIN_L_EN = 7;  // habilitación del medio puente izquierdo
// R_IS y L_IS (sensado de corriente) sin conectar.

// Encoder Hall del motor (fases A y B), alimentado con 3V3.
constexpr int PIN_ENC_A = 15;
constexpr int PIN_ENC_B = 16;
