#pragma once

// Placa: ESP32-WROOM-32D (ESP32 clásico, DevKit con CH340 -> /dev/ttyUSB0).
// Mismos pines que el brazo del robot, conexionado v0.4 (recolector_de_frutas: firmware/brazo/include/pins.h y
// docs/CONEXIONES.md), así las ganancias y el cableado de estas pruebas pasan tal cual al firmware del brazo.
// Evitar: GPIO6-11 (flash), 0/2/5/12/15 (strapping), 1/3 (UART0 = USB).
// 34-39: solo entrada y SIN pull-up interno: lo que se conecte ahí necesita pull-up externo de 10 kΩ a 3V3.
// Queda libre el GPIO 13 (reserva).

// Los tres IBT-2 (BTS7960) comparten el EN: R_EN y L_EN de los tres al GPIO 23 (+ pull-down 10 kΩ a GND).
// El firmware de una articulación deja RPWM y LPWM de las otras dos en bajo: si están conectadas, quedan frenadas
// mientras el EN está alto (y en rueda libre cuando lo baja).
constexpr int PIN_BRAZO_EN = 23;

// J1 (base giratoria): motor 36GP-555 160 rpm + IBT-2, VCC del IBT-2 a 3V3
constexpr int PIN_J1_RPWM = 21;
constexpr int PIN_J1_LPWM = 22;
constexpr int PIN_J1_EN = PIN_BRAZO_EN;
// Encoder Hall del 36GP-555 (placa SCX-555, pull-ups de 5,1 kΩ en su placa) alimentado a 3V3; 10 kΩ externos opcionales
constexpr int PIN_J1_ENC_A = 34;  // negro = Signal A
constexpr int PIN_J1_ENC_B = 35;  // rojo = Signal B

// J2 (hombro): 36-555 58 rpm + IBT-2 + correa 20 -> 100, VCC del IBT-2 a 3V3. Prueba 2026-10-10: el 36GP-555 de 160 rpm de J1.
// Encoder SCX-555 a 3V3 (como J1); pull-up interno además del de su placa
constexpr int PIN_J2_RPWM = 25;
constexpr int PIN_J2_LPWM = 26;
constexpr int PIN_J2_EN = PIN_BRAZO_EN;
constexpr int PIN_J2_ENC_A = 32;  // negro = Signal A
constexpr int PIN_J2_ENC_B = 33;  // rojo = Signal B

// J3 (codo, barra roja): 36-555 58 rpm + IBT-2 + correa 20 -> 100, VCC del IBT-2 a 3V3. Encoder SCX-555 a 3V3
constexpr int PIN_J3_RPWM = 16;
constexpr int PIN_J3_LPWM = 17;
constexpr int PIN_J3_EN = PIN_BRAZO_EN;
constexpr int PIN_J3_ENC_A = 18;  // negro = Signal A
constexpr int PIN_J3_ENC_B = 19;  // rojo = Signal B

// Pinza (no se usan en estas pruebas): señal a 3,3 V, alimentación de 6 V aparte
constexpr int PIN_SERVO_PINZA = 4;
constexpr int PIN_SERVO_GIRO = 14;

// Finales de carrera (NA a GND). Todavía no instalados.
constexpr int PIN_FC_J1 = 36;  // VP: pull-up EXTERNO de 10 kΩ a 3V3
constexpr int PIN_FC_J2 = 39;  // VN: pull-up EXTERNO de 10 kΩ a 3V3
constexpr int PIN_FC_J3 = 27;  // pull-up interno
