#include "L298N.h"

#include <Arduino.h>

bool L298N::begin(int pinEna, int pinIn1, int pinIn2, uint8_t ch, uint32_t freqHz, uint8_t bits) {
  ready_ = false;
  pinEna_ = pinEna;
  pinIn1_ = pinIn1;
  pinIn2_ = pinIn2;
  ch_ = ch;
  maxCount_ = (1u << bits) - 1;

  // Primero todo apagado: ENA en bajo (rueda libre) y los dos IN en bajo
  pinMode(pinEna_, OUTPUT);
  digitalWrite(pinEna_, LOW);
  pinMode(pinIn1_, OUTPUT);
  pinMode(pinIn2_, OUTPUT);
  digitalWrite(pinIn1_, LOW);
  digitalWrite(pinIn2_, LOW);

  if (ledcSetup(ch_, freqHz, bits) == 0) return false;
  ledcAttachPin(pinEna_, ch_);
  ledcWrite(ch_, 0);
  duty_ = 0.0f;
  enabled_ = false;
  ready_ = true;
  return true;
}

bool L298N::setDuty(float duty) {
  if (!ready_ || !isfinite(duty)) return false;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;

  if (duty == 0.0f) {
    // Freno: los dos IN iguales con el puente habilitado
    digitalWrite(pinIn1_, LOW);
    digitalWrite(pinIn2_, LOW);
    ledcWrite(ch_, maxCount_);
  } else {
    // Si cambia el sentido, apagar el PWM antes de mover los IN (nunca los dos transistores de un lado a la vez)
    const bool adelante = duty > 0.0f;
    if ((duty_ > 0.0f) != adelante || duty_ == 0.0f) ledcWrite(ch_, 0);
    digitalWrite(pinIn1_, adelante ? HIGH : LOW);
    digitalWrite(pinIn2_, adelante ? LOW : HIGH);
    ledcWrite(ch_, (uint32_t)(fabsf(duty) * maxCount_ + 0.5f));
  }
  duty_ = duty;
  enabled_ = true;
  return true;
}

void L298N::coast() {
  duty_ = 0.0f;
  enabled_ = false;
  if (!ready_) {
    if (pinEna_ >= 0) digitalWrite(pinEna_, LOW);
    return;
  }
  ledcWrite(ch_, 0);
  digitalWrite(pinIn1_, LOW);
  digitalWrite(pinIn2_, LOW);
}
