#pragma once

// Placa: ESP32-WROOM-32D (ESP32 clásico, DevKit con CH340 -> /dev/ttyUSB0).
// Mismos pines que el brazo del robot (recolector_de_frutas: firmware/brazo/include/pins.h y docs/conexiones.html),
// así las ganancias y el cableado de estas pruebas pasan tal cual al firmware del brazo.
// Evitar: GPIO6-11 (flash), 0/2/5/12/15 (strapping), 1/3 (UART0 = USB).
// 34-39: solo entrada y SIN pull-up interno: lo que se conecte ahí necesita pull-up externo de 10 kΩ a 3V3.

// J1 (base giratoria): motor 36GP-555 160 rpm + IBT-2 (BTS7960), VCC del IBT-2 a 3V3
constexpr int PIN_J1_RPWM = 21;
constexpr int PIN_J1_LPWM = 22;
constexpr int PIN_J1_EN = 23;  // R_EN y L_EN juntos (+ pull-down 10 kΩ a GND)
// Encoder Hall del 36GP-555 (placa SCX-555) alimentado a 3V3, pull-up EXTERNO de 10 kΩ en A y B
constexpr int PIN_J1_ENC_A = 34;  // negro = Signal A
constexpr int PIN_J1_ENC_B = 35;  // rojo = Signal B

// J2 (hombro): motor 5840-31ZY + L298N canal A, modo freno (igual que 08-motor-5840-l298n)
constexpr int PIN_J2_ENA = 25;  // jumper de ENA afuera (+ pull-down 10 kΩ)
constexpr int PIN_J2_IN1 = 26;
constexpr int PIN_J2_IN2 = 27;
constexpr int PIN_J2_ENC_A = 32;  // blanco (encoder 38S6G5, NPN colector abierto: pull-up interno)
constexpr int PIN_J2_ENC_B = 33;  // verde

// J3 (codo, barra roja): motor 5840-31ZY + IBT-2 (BTS7960), VCC del IBT-2 a 3V3. Antes iba en el canal B del
// L298N de J2 (ENB/IN3/IN4 en estos mismos pines); se pasó al IBT-2 el 2026-10-03 (conexiones_j3.html).
constexpr int PIN_J3_RPWM = 16;
constexpr int PIN_J3_LPWM = 17;
constexpr int PIN_J3_EN = 13;     // R_EN y L_EN juntos (+ pull-down 10 kΩ a GND)
constexpr int PIN_J3_ENC_A = 18;  // blanco (encoder 38S6G5, NPN colector abierto: pull-up interno)
constexpr int PIN_J3_ENC_B = 19;  // verde

// Finales de carrera (NA a GND, pull-up EXTERNO de 10 kΩ). Todavía no instalados.
constexpr int PIN_FC_J1 = 36;  // VP
constexpr int PIN_FC_J2 = 39;  // VN
