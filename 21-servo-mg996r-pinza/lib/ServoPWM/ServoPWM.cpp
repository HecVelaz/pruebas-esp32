#include "ServoPWM.h"

namespace {
constexpr uint32_t FREC_HZ = 50;
constexpr uint8_t BITS = 16;                    // 20 ms / 65536 ≈ 0,3 µs por paso
constexpr float PERIODO_US = 1e6f / FREC_HZ;    // 20000 µs
constexpr uint32_t MAX_CUENTA = (1u << BITS) - 1;
}  // namespace

bool ServoPWM::begin(int pin, int canal, uint16_t minUs, uint16_t maxUs) {
  if (minUs >= maxUs) return false;
  pin_ = pin;
  canal_ = canal;
  min_ = minUs;
  max_ = maxUs;
  if (ledcSetup(canal_, FREC_HZ, BITS) == 0) return false;
  ledcAttachPin(pin_, canal_);
  soltar();
  return true;
}

bool ServoPWM::limites(uint16_t minUs, uint16_t maxUs) {
  if (minUs >= maxUs) return false;
  min_ = minUs;
  max_ = maxUs;
  if (!suelto_) irA((uint16_t)destino_);
  return true;
}

void ServoPWM::irA(uint16_t us) {
  if (us < min_) us = min_;
  if (us > max_) us = max_;
  destino_ = us;
  if (suelto_ || vel_ <= 0) {
    suelto_ = false;
    actual_ = destino_;
    escribir(actual_);
  }
  tAnterior_ = micros();
}

void ServoPWM::update() {
  if (suelto_ || actual_ == destino_) {
    tAnterior_ = micros();
    return;
  }
  uint32_t ahora = micros();
  float dt = (ahora - tAnterior_) * 1e-6f;
  tAnterior_ = ahora;
  float paso = vel_ * dt;
  float falta = destino_ - actual_;
  if (fabsf(falta) <= paso) actual_ = destino_;
  else actual_ += (falta > 0 ? paso : -paso);
  escribir(actual_);
}

void ServoPWM::soltar() {
  suelto_ = true;
  ledcWrite(canal_, 0);
}

void ServoPWM::escribir(float us) {
  ledcWrite(canal_, (uint32_t)(us / PERIODO_US * MAX_CUENTA + 0.5f));
}
