#pragma once

#include <stdint.h>

#include "esp_attr.h"

// Encoder en cuadratura x4 leído con interrupciones en los flancos de A y de B. Para chips sin PCNT (el ESP32-C3
// no lo tiene; en el ESP32 y el S3 usar EncoderPCNT de 06). Una transición imposible (saltar dos estados por ruido)
// no suma ni resta. Alcanza para el SCX-555 del 36-555: a 58 rpm de salida son ~8600 cuentas/s.
// Sin Serial ni pines fijos.
class EncoderCuadratura {
 public:
  // pullup: activa el pull-up interno (los SCX-555 traen uno en su placa; no molesta tener los dos).
  bool begin(int pinA, int pinB, bool pullup = true);

  int32_t count() const { return count_; }
  void reset();

  bool ready() const { return ready_; }
  int levelA() const;
  int levelB() const;

 private:
  static void IRAM_ATTR onEdge(void *arg);

  int pinA_ = -1, pinB_ = -1;
  volatile int32_t count_ = 0;
  volatile uint8_t estado_ = 0;  // (A << 1) | B del último flanco
  bool ready_ = false;
};
