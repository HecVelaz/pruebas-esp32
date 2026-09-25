#include "BTS7960.h"

#include <Arduino.h>

bool BTS7960::begin(int pinRpwm, int pinLpwm, int pinRen, int pinLen, uint8_t chR, uint8_t chL,
                    uint32_t freqHz, uint8_t bits) {
  ready_ = false;
  pinRen_ = pinRen;
  pinLen_ = pinLen;
  chR_ = chR;
  chL_ = chL;
  maxCount_ = (1u << bits) - 1;

  // Primero todo apagado, para que el motor no arranque mientras se configura
  pinMode(pinRen_, OUTPUT);
  pinMode(pinLen_, OUTPUT);
  setEnable(false);

  if (ledcSetup(chR_, freqHz, bits) == 0) return false;
  if (ledcSetup(chL_, freqHz, bits) == 0) return false;
  ledcAttachPin(pinRpwm, chR_);
  ledcAttachPin(pinLpwm, chL_);
  ledcWrite(chR_, 0);
  ledcWrite(chL_, 0);
  duty_ = 0.0f;
  ready_ = true;
  return true;
}

bool BTS7960::setDuty(float duty) {
  if (!ready_ || !isfinite(duty)) return false;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  duty_ = duty;

  const uint32_t count = (uint32_t)(fabsf(duty) * maxCount_ + 0.5f);
  // Primero se apaga el lado que no corresponde, para no tener los dos PWM activos a la vez
  if (duty >= 0.0f) {
    ledcWrite(chL_, 0);
    ledcWrite(chR_, count);
  } else {
    ledcWrite(chR_, 0);
    ledcWrite(chL_, count);
  }
  if (!enabled_) setEnable(true);
  return true;
}

void BTS7960::coast() {
  if (pinRen_ < 0) return;
  setEnable(false);
  duty_ = 0.0f;
  if (!ready_) return;
  ledcWrite(chR_, 0);
  ledcWrite(chL_, 0);
}

void BTS7960::setEnable(bool on) {
  digitalWrite(pinRen_, on ? HIGH : LOW);
  digitalWrite(pinLen_, on ? HIGH : LOW);
  enabled_ = on;
}
