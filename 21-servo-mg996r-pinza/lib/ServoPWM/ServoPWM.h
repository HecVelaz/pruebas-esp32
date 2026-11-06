#pragma once
#include <Arduino.h>

// Servo de hobby con el LEDC del ESP32 (Arduino core 2.x): 50 Hz, ancho de pulso en µs.
// Sin Serial ni pines fijos. El pulso se limita a [minUs, maxUs] y se mueve en rampa
// (velocidad en µs por segundo) para no golpear la pinza contra un tope.
class ServoPWM {
public:
  // minUs / maxUs: límites duros del pulso (la orden se recorta a esa franja).
  bool begin(int pin, int canal, uint16_t minUs, uint16_t maxUs);

  // Destino del pulso. Arranca a moverse en el próximo update(). Si estaba suelto,
  // el primer pulso es directo (no hay forma de saber dónde está el servo).
  void irA(uint16_t us);
  // Cambia la franja; si el destino queda afuera, se recorta.
  bool limites(uint16_t minUs, uint16_t maxUs);
  void velocidad(float usPorSegundo) { vel_ = usPorSegundo; }  // 0 = sin rampa

  // Llamar seguido desde loop(): avanza la rampa.
  void update();

  // Deja de mandar pulsos: el servo queda sin fuerza (no sostiene).
  void soltar();

  bool suelto() const { return suelto_; }
  bool enDestino() const { return !suelto_ && actual_ == destino_; }
  uint16_t actualUs() const { return (uint16_t)(actual_ + 0.5f); }
  uint16_t destinoUs() const { return (uint16_t)destino_; }
  uint16_t minUs() const { return min_; }
  uint16_t maxUs() const { return max_; }

private:
  void escribir(float us);

  int pin_ = -1, canal_ = 0;
  uint16_t min_ = 1000, max_ = 2000;
  float actual_ = 1500, destino_ = 1500;
  float vel_ = 0;
  bool suelto_ = true;
  uint32_t tAnterior_ = 0;
};
