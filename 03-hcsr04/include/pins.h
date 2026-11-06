#pragma once

// Pines de la base en la S3 (docs/html/placa_s3.html, recolector_de_frutas/firmware/base/include/pins.h).

// HC-SR04 (VCC a 5 V): un TRIG y un ECHO por sensor. El ECHO sale a 5 V y pasa por un divisor 1 kΩ / 2 kΩ
// (~3,3 V) en la perforada antes de llegar al GPIO. Los tres miran al frente: se disparan de a uno.
constexpr int PIN_US_TRIG_FRENTE = 10;
constexpr int PIN_US_TRIG_IZQ = 41;
constexpr int PIN_US_TRIG_DER = 42;
constexpr int PIN_US_ECHO_FRENTE = 11;
constexpr int PIN_US_ECHO_IZQ = 12;
constexpr int PIN_US_ECHO_DER = 13;

// EN común de los dos IBT-2 de tracción y sus PWM: se dejan en bajo para que ninguna rueda se mueva.
constexpr int PIN_TRACCION_EN = 38;
constexpr int PINES_PWM[] = {4, 5, 6, 7};
