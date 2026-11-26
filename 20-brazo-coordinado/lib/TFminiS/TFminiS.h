#pragma once
#include <stdint.h>
#include <string.h>

// Tramas UART de fábrica (cm). Sin Arduino, bus ni pines fijos.
class TFminiS {
 public:
  static constexpr uint32_t VIGENCIA_MS = 300;

  bool recibir(uint8_t byte, uint32_t ahora) {
    if (usados_ && uint32_t(ahora - ultimoByte_) > 50) usados_ = 0;
    ultimoByte_ = ahora;
    if (usados_ == 0 && byte != 0x59) return false;
    if (usados_ == 1 && byte != 0x59) { usados_ = 0; return false; }
    trama_[usados_++] = byte;
    if (usados_ < sizeof(trama_)) return false;
    uint8_t suma = 0;
    for (unsigned i = 0; i < 8; ++i) suma += trama_[i];
    if (suma != trama_[8]) {
      // Ventana deslizante: recupera cabeceras dentro de una trama corrupta.
      memmove(trama_, trama_ + 1, 8);
      usados_ = 8;
      while (usados_ && (trama_[0] != 0x59 || (usados_ > 1 && trama_[1] != 0x59))) {
        --usados_;
        memmove(trama_, trama_ + 1, usados_);
      }
      return false;
    }
    usados_ = 0;
    distanciaCm_ = uint16_t(trama_[2]) | (uint16_t(trama_[3]) << 8);
    intensidad_ = uint16_t(trama_[4]) | (uint16_t(trama_[5]) << 8);
    recibida_ = true;
    ultimaTrama_ = ahora;
    valida_ = distanciaCm_ >= 10 && distanciaCm_ <= 1200 && intensidad_ >= 100 && intensidad_ != 65535;
    return true;
  }

  bool valida(uint32_t ahora) const {
    return recibida_ && valida_ && uint32_t(ahora - ultimaTrama_) <= VIGENCIA_MS;
  }
  float metros() const { return distanciaCm_ * .01f; }
  uint16_t intensidad() const { return intensidad_; }
  void descartar() { usados_ = 0; recibida_ = false; }

 private:
  uint8_t trama_[9] = {};
  unsigned usados_ = 0;
  uint16_t distanciaCm_ = 0, intensidad_ = 0;
  uint32_t ultimoByte_ = 0, ultimaTrama_ = 0;
  bool recibida_ = false, valida_ = false;
};
