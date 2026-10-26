#pragma once

// Placa: ESP32-WROOM-32D (DevKit con CH340 -> /dev/ttyUSB0), la del brazo del robot.
// Conexionado DEFINITIVO v0.5 (recolector_de_frutas: firmware/brazo/include/pins.h y docs/CONEXIONES.md, 12/10):
// las tres articulaciones quedan conectadas a la vez; el firmware de esta carpeta mueve las tres (joystick).
// Evitar: GPIO6-11 (flash), 0/2/5/12/15 (strapping), 1/3 (UART0 = USB).
// 34-39: solo entrada y SIN pull-up interno: ahí van solo los encoders, que traen pull-up de 5,1 kΩ en su placa.
// Queda libre el GPIO 13 (reserva).

// Los tres IBT-2 (BTS7960) comparten el EN: R_EN y L_EN de los tres al GPIO 23 (+ pull-down 10 kΩ a GND).
// VCC de los tres IBT-2 al riel de 5 V (VIN del shield); R_IS y L_IS sin conectar.
// Con el EN alto y duty 0 cada articulación queda FRENADA; con el EN bajo (rueda libre) quedan las tres SUELTAS.
// J2 y J3 no se autobloquean: sueltas, el brazo cae. Este firmware no suelta nunca: frena.
constexpr int PIN_BRAZO_EN = 23;

// J1 (base giratoria): 36GP-555 160 rpm (50:1) + IBT-2 + correa 18 -> 90
constexpr int PIN_J1_RPWM = 21;
constexpr int PIN_J1_LPWM = 22;
constexpr int PIN_J1_ENC_A = 34;  // negro = Signal A (SCX-555 a 3V3)
constexpr int PIN_J1_ENC_B = 35;  // rojo = Signal B

// J2 (hombro, barra verde): 36-555 58 rpm (139:1) + IBT-2 + engranajes helicoidales 15 -> 75
constexpr int PIN_J2_RPWM = 25;
constexpr int PIN_J2_LPWM = 26;
constexpr int PIN_J2_ENC_A = 36;  // VP · negro (SCX-555 a 3V3; solo el pull-up de su placa)
constexpr int PIN_J2_ENC_B = 39;  // VN · rojo

// J3 (codo, balancín rojo del cuatro barras): 36-555 58 rpm (139:1) + IBT-2 + engranajes helicoidales 15 -> 75
constexpr int PIN_J3_RPWM = 16;
constexpr int PIN_J3_LPWM = 17;
constexpr int PIN_J3_ENC_A = 18;  // negro (pull-up de su placa + interno)
constexpr int PIN_J3_ENC_B = 19;  // rojo

// Pinza (no se usa en estas pruebas): señal a 3,3 V, alimentación de 6 V aparte (buck 3)
constexpr int PIN_SERVO_PINZA = 4;
constexpr int PIN_SERVO_GIRO = 14;

// Finales de carrera (NA a GND, pull-up interno). Estas pruebas no los usan: el cero se fija a mano con "z".
constexpr int PIN_FC_J1 = 32;
constexpr int PIN_FC_J2 = 33;
constexpr int PIN_FC_J3 = 27;
