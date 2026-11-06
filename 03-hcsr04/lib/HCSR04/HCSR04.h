#pragma once

// Copia de recolector_de_frutas/firmware/lib/HCSR04 (commit b2b5b2d), para probar en 03-hcsr04 el mismo driver de la base.

#include <Arduino.h>

// Varios HC-SR04, cada uno con su TRIG y su ECHO medido por interrupción: no bloquea (sin pulseIn).
// Sin Serial ni pines fijos. El ECHO sale a 5 V: cada pin necesita un divisor (1 kΩ arriba, 2 kΩ abajo).
//
// Se disparan de a uno, por turno, para que un sensor no tome el eco de otro (los de la base miran todos al frente).
// Uso: disparar() cada >= 40 ms (el eco de "nada al alcance" dura hasta ~38 ms); cada llamada cierra la medición
// del sensor anterior y dispara el siguiente. Con n sensores, cada uno se actualiza cada n disparos.
class HCSR04 {
 public:
  static constexpr int MAX_SENSORES = 4;
  static constexpr uint32_t ECO_MAX_US = 25000;  // ~4,3 m; más largo = nada al alcance (algunos dan 38 ms)

  bool begin(const int *pinesTrig, const int *pinesEcho, int n);
  // Cierra la medición del sensor disparado antes (si no respondió, NAN) y da un pulso de 10 µs al siguiente.
  void disparar();
  // Metros de la última medición de ese sensor. INFINITY = eco largo (nada al alcance);
  // NAN = no hubo ningún pulso de eco (sensor desconectado, sin alimentación o todavía sin medir).
  float distancia(int i) const;

 private:
  static void IRAM_ATTR alCambiar(void *arg);

  struct Canal {
    HCSR04 *padre;
    int pinTrig;
    int pin;
    volatile uint32_t subidaUs;
    volatile uint32_t anchoUs;  // 0 = sin eco en este ciclo
    float metros;
  };

  // La interrupción puede correr en otro núcleo que disparar(): noInterrupts() no alcanza
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
  int n_ = 0;
  int actual_ = -1;  // sensor disparado por última vez (-1 = ninguno todavía)
  Canal canal_[MAX_SENSORES] = {};
};
