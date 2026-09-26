#pragma once

#include <stdint.h>

// Driver mínimo de un canal del módulo L298N (puente H bipolar). Misma interfaz que lib/BTS7960 de 06
// (setDuty, brake, coast), para reutilizar el programa de prueba. Sin Serial ni pines fijos.
//
// Dos formas de aplicar el PWM (se elige en begin()):
//   Modo::Freno (por defecto): ENA fijo en alto y el PWM en IN1 (adelante) o IN2 (atrás). En la parte
//     apagada del ciclo los dos IN quedan en bajo: el motor queda frenado (la corriente recircula por los
//     transistores de abajo). La velocidad es casi proporcional al duty, como con el BTS7960.
//   Modo::RuedaLibre: el PWM en ENA e IN1/IN2 fijos según el sentido. En la parte apagada el puente se
//     deshabilita y la corriente cae por los diodos: con poca carga el motor recibe más tensión media que
//     el duty y la curva velocidad-duty se satura (medido: 40 % ya da el 60 % de la velocidad máxima).
//
//   duty 0 -> freno: IN1 = IN2 = 0 con ENA = 1
//   coast() -> rueda libre: ENA = 0
//
// Un pin que deja de tener PWM se desconecta del LEDC y se fija en su nivel en el acto: un ledcWrite(0)
// recién se aplica al terminar el ciclo de PWM y dejaría hasta un período con el valor anterior.
class L298N {
 public:
  enum class Modo { Freno, RuedaLibre };

  // chA y chB: dos canales LEDC libres. Devuelve false si el LEDC no puede generar la frecuencia.
  bool begin(int pinEna, int pinIn1, int pinIn2, uint8_t chA, uint8_t chB, Modo modo = Modo::Freno,
             uint32_t freqHz = 1000, uint8_t bits = 10);

  // duty entre -1 y 1. Positivo: adelante (IN1); negativo: atrás (IN2); 0: freno.
  // Devuelve false (sin tocar nada) si begin() no terminó bien o el duty no es un número finito.
  bool setDuty(float duty);
  bool brake() { return setDuty(0.0f); }
  // Deshabilita el puente: rueda libre.
  void coast();

  float duty() const { return duty_; }
  bool enabled() const { return enabled_; }
  bool ready() const { return ready_; }
  Modo modo() const { return modo_; }

 private:
  // Saca el pin del LEDC (si lo tenía) y lo deja fijo en 'nivel' de inmediato.
  void fijar(int pin, uint8_t ch, bool &conPwm, int nivel);
  // Conecta el pin al LEDC con ese duty.
  void pwm(int pin, uint8_t ch, bool &conPwm, uint32_t cuenta);

  int pinEna_ = -1, pinIn1_ = -1, pinIn2_ = -1;
  uint8_t chA_ = 0, chB_ = 1;
  Modo modo_ = Modo::Freno;
  uint32_t periodoUs_ = 1000;
  uint32_t maxCount_ = 1023;
  bool pwmEna_ = false, pwmIn1_ = false, pwmIn2_ = false;
  float duty_ = 0.0f;
  bool enabled_ = false;
  bool ready_ = false;
};
