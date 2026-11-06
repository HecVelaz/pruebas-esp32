#pragma once

// ESP32 DevKit "ESP32S" (CP2102). Único lugar con GPIO de esta prueba.
// Evitar: 0/2/5/12/15 (strapping), 6–11 (flash), 1/3 (UART0 = USB), 34–39 (solo entrada).

// Señal del servo MG996R (cable naranja). Marrón = GND común, rojo = +5–6 V de fuente externa.
constexpr int PIN_SERVO = 18;

// Canal LEDC que genera el PWM de 50 Hz.
constexpr int CANAL_SERVO = 0;
