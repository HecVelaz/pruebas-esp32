#pragma once

// Conexionado v0.5 del robot (recolector_de_frutas/docs/CONEXIONES.md), igual que 12-14 y 17.

// Finales de carrera: microinterruptor V-152 (SPDT, rodillo), común a GND y NA al GPIO, pull-up interno.
// Suelto = 1 (3,3 V), apretado = 0 (GND). Sin capacitor.
constexpr int PIN_FC_J1 = 32;
constexpr int PIN_FC_J2 = 27;  // 2026-10-30: J2 y J3 cruzados respecto de v0.5 (era 33)
constexpr int PIN_FC_J3 = 33;  // 2026-10-30: era 27

// EN común de los tres IBT-2 y sus PWM: se dejan en bajo para que ningún motor se mueva.
constexpr int PIN_BRAZO_EN = 23;
constexpr int PINES_PWM[] = {21, 22, 25, 26, 16, 17};
