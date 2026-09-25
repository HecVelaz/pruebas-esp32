#include "EncoderPCNT.h"

#include <Arduino.h>

#include "driver/gpio.h"

// Límites del contador de hardware. Al alcanzar uno, el PCNT vuelve a 0 y genera un evento.
static constexpr int16_t LIM_ALTO = 32000;
static constexpr int16_t LIM_BAJO = -32000;

static bool isrServiceInstalado = false;

void IRAM_ATTR EncoderPCNT::onLimit(void *arg) {
  EncoderPCNT *self = static_cast<EncoderPCNT *>(arg);
  uint32_t status = 0;
  pcnt_get_event_status(self->unit_, &status);
  if (status & PCNT_EVT_H_LIM) self->overflow_ += LIM_ALTO;
  if (status & PCNT_EVT_L_LIM) self->overflow_ += LIM_BAJO;
}

bool EncoderPCNT::begin(int pinA, int pinB, pcnt_unit_t unit, uint32_t filterNs) {
  ready_ = false;
  unit_ = unit;
  pinA_ = pinA;
  pinB_ = pinB;

  // Cuadratura x4, como el ejemplo rotary_encoder de ESP-IDF 4.4.
  // Canal 0: cuenta los flancos de A; el nivel de B decide el sentido.
  pcnt_config_t c = {};
  c.pulse_gpio_num = pinA;
  c.ctrl_gpio_num = pinB;
  c.channel = PCNT_CHANNEL_0;
  c.unit = unit;
  c.pos_mode = PCNT_COUNT_DEC;
  c.neg_mode = PCNT_COUNT_INC;
  c.lctrl_mode = PCNT_MODE_REVERSE;
  c.hctrl_mode = PCNT_MODE_KEEP;
  c.counter_h_lim = LIM_ALTO;
  c.counter_l_lim = LIM_BAJO;
  if (pcnt_unit_config(&c) != ESP_OK) return false;

  // Canal 1: cuenta los flancos de B; el nivel de A decide el sentido.
  c.pulse_gpio_num = pinB;
  c.ctrl_gpio_num = pinA;
  c.channel = PCNT_CHANNEL_1;
  c.pos_mode = PCNT_COUNT_INC;
  c.neg_mode = PCNT_COUNT_DEC;
  if (pcnt_unit_config(&c) != ESP_OK) return false;

  // Pull-ups internos por si las salidas del encoder son de colector abierto
  gpio_pullup_en((gpio_num_t)pinA);
  gpio_pullup_en((gpio_num_t)pinB);

  // El filtro se mide en ciclos del APB (80 MHz), 10 bits
  uint32_t cycles = filterNs * 80 / 1000;
  if (cycles > 1023) cycles = 1023;
  if (cycles > 0) {
    pcnt_set_filter_value(unit, (uint16_t)cycles);
    pcnt_filter_enable(unit);
  } else {
    pcnt_filter_disable(unit);
  }

  // Eventos de límite: acumulan el desborde del contador de 16 bits
  pcnt_event_enable(unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(unit, PCNT_EVT_L_LIM);
  if (!isrServiceInstalado) {
    if (pcnt_isr_service_install(0) != ESP_OK) return false;
    isrServiceInstalado = true;
  }
  if (pcnt_isr_handler_add(unit, onLimit, this) != ESP_OK) return false;

  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  overflow_ = 0;
  lastDelta_ = 0;
  pcnt_counter_resume(unit);
  ready_ = true;
  return true;
}

int64_t EncoderPCNT::count() const {
  if (!ready_) return 0;
  // Si la interrupción de límite actualiza el acumulador entre las dos lecturas, repetir:
  // así el acumulador y el contador de hardware son del mismo instante. (Queda la latencia de la
  // interrupción, de menos de un microsegundo, una vez cada 32000 cuentas.)
  int64_t antes, despues;
  int16_t v;
  do {
    antes = overflow_;
    pcnt_get_counter_value(unit_, &v);
    despues = overflow_;
  } while (antes != despues);
  return antes + v;
}

int32_t EncoderPCNT::delta() {
  const int64_t c = count();
  const int64_t d = c - lastDelta_;
  lastDelta_ = c;
  return (int32_t)d;
}

void EncoderPCNT::reset() {
  if (!ready_) return;
  pcnt_counter_pause(unit_);
  pcnt_counter_clear(unit_);
  overflow_ = 0;
  lastDelta_ = 0;
  pcnt_counter_resume(unit_);
}

int EncoderPCNT::levelA() const { return gpio_get_level((gpio_num_t)pinA_); }
int EncoderPCNT::levelB() const { return gpio_get_level((gpio_num_t)pinB_); }
