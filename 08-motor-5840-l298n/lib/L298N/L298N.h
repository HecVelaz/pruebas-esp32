#pragma once

#include <stdint.h>

// Driver mínimo de un canal del módulo L298N (puente H bipolar): ENA con PWM, IN1/IN2 dan el sentido.
// Misma interfaz que lib/BTS7960 de 06, para reutilizar el programa de prueba. Sin Serial ni pines fijos.
//
//   IN1 = 1, IN2 = 0, ENA = PWM -> adelante (en la parte apagada del PWM el puente se deshabilita
//                                  y la corriente del motor circula por los diodos: rueda libre)
//   IN1 = 0, IN2 = 1, ENA = PWM -> atrás
//   IN1 = IN2 = 0, ENA = 1      -> freno (bornes del motor unidos por los transistores de abajo)
//   ENA = 0                     -> rueda libre (puente deshabilitado)
class L298N {
 public:
  // Devuelve false si el LEDC no puede generar la frecuencia con esa resolución.
  bool begin(int pinEna, int pinIn1, int pinIn2, uint8_t ch, uint32_t freqHz = 1000, uint8_t bits = 10);

  // duty entre -1 y 1. Positivo: IN1; negativo: IN2; 0: freno.
  // Devuelve false (sin tocar nada) si begin() no terminó bien o el duty no es un número finito.
  bool setDuty(float duty);
  bool brake() { return setDuty(0.0f); }
  // Deshabilita el puente: rueda libre.
  void coast();

  float duty() const { return duty_; }
  bool enabled() const { return enabled_; }
  bool ready() const { return ready_; }

 private:
  int pinIn1_ = -1, pinIn2_ = -1, pinEna_ = -1;
  uint8_t ch_ = 0;
  uint32_t maxCount_ = 1023;
  float duty_ = 0.0f;
  bool enabled_ = false;
  bool ready_ = false;
};
