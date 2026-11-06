#pragma once

// Pines de la base en la S3 (docs/html/placa_s3.html, firmware/base/include/pins.h del robot).

// Bumpers: microinterruptor V-152-1C25 (SPDT), común (C) a GND y NA al GPIO, pull-up interno.
// Suelto = 1 (3,3 V), apretado = 0 (GND). Sin capacitor.
constexpr int PIN_BUMPER_FRENTE = 14;
constexpr int PIN_BUMPER_IZQ = 21;
constexpr int PIN_BUMPER_DER = 39;

// EN común de los dos IBT-2 de tracción y sus PWM: se dejan en bajo para que ninguna rueda se mueva.
constexpr int PIN_TRACCION_EN = 38;
constexpr int PINES_PWM[] = {4, 5, 6, 7};
