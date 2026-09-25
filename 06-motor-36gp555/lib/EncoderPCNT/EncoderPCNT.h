#pragma once

#include <stdint.h>

#include "driver/pcnt.h"
#include "esp_attr.h"

// Encoder en cuadratura leído por hardware con el PCNT del ESP32 (x4: flancos de A y de B).
// No usa interrupciones por flanco, así que no carga la CPU aunque el motor gire rápido.
// El contador de hardware (16 bits) nunca se borra durante la marcha: cuando llega a su límite
// vuelve a 0 y una interrupción suma ese límite a un acumulador, así no se pierden cuentas
// aunque pase mucho tiempo entre lecturas. Sin Serial ni pines fijos.
class EncoderPCNT {
 public:
  // filterNs: descarta pulsos más cortos (ruido del PWM). Máximo ~12700 ns.
  bool begin(int pinA, int pinB, pcnt_unit_t unit = PCNT_UNIT_0, uint32_t filterNs = 10000);

  // Cuentas acumuladas desde begin() (o desde reset()).
  int64_t count() const;
  // Cuentas desde la llamada anterior a delta().
  int32_t delta();
  void reset();

  bool ready() const { return ready_; }

  // Niveles actuales de A y B (para verificar el cableado con el motor girado a mano).
  int levelA() const;
  int levelB() const;

 private:
  static void IRAM_ATTR onLimit(void *arg);

  pcnt_unit_t unit_ = PCNT_UNIT_0;
  int pinA_ = -1, pinB_ = -1;
  volatile int64_t overflow_ = 0;  // suma de los límites alcanzados (la escribe la interrupción)
  int64_t lastDelta_ = 0;
  bool ready_ = false;
};
