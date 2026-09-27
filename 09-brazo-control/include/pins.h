#pragma once

// Placa: ESP32-WROOM-32D (ESP32 clásico, DevKit con CH340 -> /dev/ttyUSB0). Controla el brazo completo.
// Evitar: GPIO6-11 (flash), 0/2/5/12/15 (strapping), 1/3 (UART0 = USB).
// 34-39: solo entrada y SIN pull-up interno: lo que se conecte ahí necesita pull-up externo.
// Usa los 15 GPIO de uso general; no queda ninguno libre.

// J1 (base giratoria): motor 36GP-555 160 rpm + IBT-2 (BTS7960)
constexpr int PIN_J1_RPWM = 21;
constexpr int PIN_J1_LPWM = 22;
constexpr int PIN_J1_EN = 23;  // R_EN y L_EN juntos al mismo GPIO (+ pull-down 10 kΩ a GND)
// Encoder Hall del 36GP-555 (placa SCX-555): alimentar a 3V3 y pull-up EXTERNO de 10 kΩ a 3V3 en A y B
constexpr int PIN_J1_ENC_A = 34;  // negro = Signal A
constexpr int PIN_J1_ENC_B = 35;  // rojo = Signal B

// J2 (hombro): motor 5840-31ZY + L298N canal A (mismos pines que 08-motor-5840-l298n)
constexpr int PIN_J2_ENA = 25;  // jumper de ENA afuera (+ pull-down 10 kΩ)
constexpr int PIN_J2_IN1 = 26;
constexpr int PIN_J2_IN2 = 27;
constexpr int PIN_J2_ENC_A = 32;  // blanco (encoder 38S6G5, NPN colector abierto: pull-up interno)
constexpr int PIN_J2_ENC_B = 33;  // verde

// J3 (codo, balancín del cuatro barras): motor 5840-31ZY + L298N canal B
constexpr int PIN_J3_ENB = 13;  // jumper de ENB afuera (+ pull-down 10 kΩ)
constexpr int PIN_J3_IN3 = 16;
constexpr int PIN_J3_IN4 = 17;
constexpr int PIN_J3_ENC_A = 18;  // blanco
constexpr int PIN_J3_ENC_B = 19;  // verde

// Pinza: servo de apertura y servo de giro (J4). Señal a 3,3 V; alimentación de 5-6 V APARTE (no del ESP32).
constexpr int PIN_SERVO_PINZA = 4;
constexpr int PIN_SERVO_GIRO = 14;  // GPIO14 saca pulsos durante el arranque: el servo puede moverse un instante

// Finales de carrera para el homing (normalmente abiertos a GND, pull-up EXTERNO de 10 kΩ). -1 = no hay.
// Sin final de carrera, el cero se fija a mano en la pose de referencia (comando z).
constexpr int PIN_FC_J1 = 36;  // VP
constexpr int PIN_FC_J2 = 39;  // VN
constexpr int PIN_FC_J3 = -1;  // no queda GPIO libre
