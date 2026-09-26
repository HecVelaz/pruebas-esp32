#include "L298N.h"

#include <Arduino.h>

bool L298N::begin(int pinEna, int pinIn1, int pinIn2, uint8_t chA, uint8_t chB, Modo modo, uint32_t freqHz,
                  uint8_t bits) {
  ready_ = false;
  pinEna_ = pinEna;
  pinIn1_ = pinIn1;
  pinIn2_ = pinIn2;
  chA_ = chA;
  chB_ = chB;
  modo_ = modo;
  maxCount_ = (1u << bits) - 1;
  periodoUs_ = freqHz ? 1000000u / freqHz : 1000;

  // Primero todo apagado: ENA en bajo (rueda libre) y los dos IN en bajo
  pinMode(pinEna_, OUTPUT);
  digitalWrite(pinEna_, LOW);
  pinMode(pinIn1_, OUTPUT);
  digitalWrite(pinIn1_, LOW);
  pinMode(pinIn2_, OUTPUT);
  digitalWrite(pinIn2_, LOW);
  pwmEna_ = pwmIn1_ = pwmIn2_ = false;

  if (ledcSetup(chA_, freqHz, bits) == 0) return false;
  if (modo_ == Modo::Freno && ledcSetup(chB_, freqHz, bits) == 0) return false;
  ledcWrite(chA_, 0);
  if (modo_ == Modo::Freno) ledcWrite(chB_, 0);
  duty_ = 0.0f;
  enabled_ = false;
  ready_ = true;
  return true;
}

void L298N::fijar(int pin, uint8_t ch, bool &conPwm, int nivel) {
  if (conPwm) {
    ledcWrite(ch, 0);  // para que al volver a conectarlo arranque desde 0
    ledcDetachPin(pin);
    pinMode(pin, OUTPUT);
    conPwm = false;
  }
  digitalWrite(pin, nivel);
}

void L298N::pwm(int pin, uint8_t ch, bool &conPwm, uint32_t cuenta) {
  ledcWrite(ch, cuenta);
  if (!conPwm) {
    ledcAttachPin(pin, ch);
    conPwm = true;
  }
}

bool L298N::setDuty(float duty) {
  if (!ready_ || !isfinite(duty)) return false;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  const uint32_t cuenta = (uint32_t)(fabsf(duty) * maxCount_ + 0.5f);
  const bool adelante = duty > 0.0f;
  const bool cambiaSentido = duty != 0.0f && duty_ != 0.0f && (duty_ > 0.0f) != adelante;

  if (modo_ == Modo::Freno) {
    if (duty == 0.0f) {  // freno: los dos IN en bajo con ENA en alto
      fijar(pinIn1_, chA_, pwmIn1_, LOW);
      fijar(pinIn2_, chB_, pwmIn2_, LOW);
    } else if (adelante) {  // primero se suelta el IN del otro sentido, en el acto
      fijar(pinIn2_, chB_, pwmIn2_, LOW);
      pwm(pinIn1_, chA_, pwmIn1_, cuenta);
    } else {
      fijar(pinIn1_, chA_, pwmIn1_, LOW);
      pwm(pinIn2_, chB_, pwmIn2_, cuenta);
    }
    digitalWrite(pinEna_, HIGH);
  } else {  // Modo::RuedaLibre: PWM en ENA
    if (duty == 0.0f) {  // freno: ENA fijo en alto con los dos IN en bajo
      digitalWrite(pinIn1_, LOW);
      digitalWrite(pinIn2_, LOW);
      fijar(pinEna_, chA_, pwmEna_, HIGH);
    } else {
      if (!pwmEna_ || cambiaSentido) {
        // Saliendo del freno, de la rueda libre o cambiando de sentido: ENA en bajo en el acto, y
        // recién después se mueven los IN y se conecta el PWM
        const bool veniaConPwm = pwmEna_;
        fijar(pinEna_, chA_, pwmEna_, LOW);
        // El canal pudo quedar con el duty anterior hasta fin de ciclo: esperar un período
        if (veniaConPwm) delayMicroseconds(periodoUs_ + 50);
        digitalWrite(pinIn1_, adelante ? HIGH : LOW);
        digitalWrite(pinIn2_, adelante ? LOW : HIGH);
      }
      pwm(pinEna_, chA_, pwmEna_, cuenta);
    }
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
  if (modo_ == Modo::Freno) {
    digitalWrite(pinEna_, LOW);
    fijar(pinIn1_, chA_, pwmIn1_, LOW);
    fijar(pinIn2_, chB_, pwmIn2_, LOW);
  } else {
    fijar(pinEna_, chA_, pwmEna_, LOW);
    digitalWrite(pinIn1_, LOW);
    digitalWrite(pinIn2_, LOW);
  }
}
