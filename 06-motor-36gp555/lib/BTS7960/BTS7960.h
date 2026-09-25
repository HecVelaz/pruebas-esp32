#pragma once

#include <stdint.h>

// Driver mínimo del módulo IBT-2 (dos medios puentes BTS7960 formando un puente H).
// Sin Serial ni pines fijos: los pines y los canales LEDC se pasan en begin().
//
// Modos (con EN en alto):
//   RPWM = PWM, LPWM = 0  -> adelante (en la parte baja del PWM los dos lados van a GND: frenado)
//   RPWM = 0,   LPWM = PWM -> atrás
//   RPWM = 0,   LPWM = 0   -> freno (bornes del motor en cortocircuito)
// Con EN en bajo las salidas quedan en alta impedancia: el motor gira libre.
class BTS7960 {
 public:
  // Devuelve false si el LEDC no puede generar la frecuencia con esa resolución.
  bool begin(int pinRpwm, int pinLpwm, int pinRen, int pinLen, uint8_t chR, uint8_t chL,
             uint32_t freqHz = 20000, uint8_t bits = 10);

  // duty entre -1 y 1. Positivo: RPWM; negativo: LPWM; 0: freno. Habilita el puente.
  // Devuelve false (sin tocar nada) si begin() no terminó bien o el duty no es un número finito.
  bool setDuty(float duty);
  // Freno activo (duty 0 con el puente habilitado).
  bool brake() { return setDuty(0.0f); }
  // Deshabilita el puente: rueda libre.
  void coast();

  float duty() const { return duty_; }
  bool enabled() const { return enabled_; }
  bool ready() const { return ready_; }

 private:
  void setEnable(bool on);

  int pinRen_ = -1, pinLen_ = -1;
  uint8_t chR_ = 0, chL_ = 1;
  uint32_t maxCount_ = 1023;
  float duty_ = 0.0f;
  bool enabled_ = false;
  bool ready_ = false;
};
