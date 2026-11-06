#pragma once

// ESP32 DevKit "ESP32S" (CP2102), la misma placa que 21-servo-mg996r-pinza. Único lugar con GPIO de esta prueba.
// Evitar: 0/2/5/12/15 (strapping), 6–11 (flash), 1/3 (UART0 = USB), 34–39 (solo entrada).

// Señal del servo de la muñeca (cable naranja o amarillo). GPIO 19 para que pueda convivir con el servo
// de la pinza en el GPIO 18. Marrón/negro = GND común, rojo = +5–6 V de fuente externa.
constexpr int PIN_SERVO = 19;

// Canal LEDC que genera el PWM de 50 Hz (el 0 queda para la pinza).
constexpr int CANAL_SERVO = 1;
