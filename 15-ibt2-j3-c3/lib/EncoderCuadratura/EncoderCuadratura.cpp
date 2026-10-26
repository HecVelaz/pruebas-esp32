#include "EncoderCuadratura.h"

#include <Arduino.h>

// Índice: (estado anterior << 2) | estado nuevo, con estado = (A << 1) | B. 0 = sin cambio o salto imposible.
static const int8_t PASO[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

bool EncoderCuadratura::begin(int pinA, int pinB, bool pullup) {
  pinA_ = pinA;
  pinB_ = pinB;
  pinMode(pinA_, pullup ? INPUT_PULLUP : INPUT);
  pinMode(pinB_, pullup ? INPUT_PULLUP : INPUT);
  estado_ = (digitalRead(pinA_) << 1) | digitalRead(pinB_);
  count_ = 0;
  attachInterruptArg(pinA_, onEdge, this, CHANGE);
  attachInterruptArg(pinB_, onEdge, this, CHANGE);
  ready_ = true;
  return true;
}

void IRAM_ATTR EncoderCuadratura::onEdge(void *arg) {
  EncoderCuadratura *e = static_cast<EncoderCuadratura *>(arg);
  const uint8_t nuevo = (digitalRead(e->pinA_) << 1) | digitalRead(e->pinB_);
  portENTER_CRITICAL_ISR(&mux);
  e->count_ += PASO[(e->estado_ << 2) | nuevo];
  e->estado_ = nuevo;
  portEXIT_CRITICAL_ISR(&mux);
}

void EncoderCuadratura::reset() {
  portENTER_CRITICAL(&mux);
  count_ = 0;
  portEXIT_CRITICAL(&mux);
}

int EncoderCuadratura::levelA() const { return pinA_ < 0 ? -1 : digitalRead(pinA_); }
int EncoderCuadratura::levelB() const { return pinB_ < 0 ? -1 : digitalRead(pinB_); }
