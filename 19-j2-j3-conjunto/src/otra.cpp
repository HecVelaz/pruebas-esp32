#include "otra.h"

#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"
#include "config.h"
#include "pins.h"

namespace otra {
namespace {

#if defined(ARTICULACION_J3)
constexpr int PIN_R = PIN_J2_RPWM, PIN_L = PIN_J2_LPWM, PIN_A = PIN_J2_ENC_A, PIN_B = PIN_J2_ENC_B, PIN_F = PIN_FC_J2;
#else
constexpr int PIN_R = PIN_J3_RPWM, PIN_L = PIN_J3_LPWM, PIN_A = PIN_J3_ENC_A, PIN_B = PIN_J3_ENC_B, PIN_F = PIN_FC_J3;
#endif
constexpr uint8_t CH_R = 2, CH_L = 3;  // la activa usa 0 y 1
constexpr uint32_t TS_MS = 10;

BTS7960 motor;
EncoderPCNT enc;

// Compartido entre núcleos: escrito por un lado y leído por el otro, de 32 bits (atómico en el ESP32)
volatile bool activo = false;
volatile bool pedidoCero = false;
volatile float dest = 0.0f;
volatile float q = 0.0f;
volatile float u = 0.0f;
volatile bool fc = false;
volatile float kp = OTRA.khp, ki = OTRA.khi;
volatile const char *avisoPendiente = nullptr;
volatile bool reiniciar = false;  // sostener() desde parado: la tarea arranca la referencia donde está

int64_t ceroCuentas = 0;
float ref = 0.0f, uHold = 0.0f;

float leerGrados() {
  int64_t c = enc.count() - ceroCuentas;
  if (OTRA.invCuentas) c = -c;
  return (float)c / OTRA.cuentasPorGrado;
}

void aplicar(float pct) {
  u = pct;
  motor.setDuty((OTRA.invDuty ? -pct : pct) / 100.0f);
}

void tarea(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  uint32_t msError = 0, msAtasco = 0;
  float qAtasco = 0.0f;
  bool atascada = false;
  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(TS_MS));
    if (pedidoCero) {
      ceroCuentas = enc.count();
      ref = 0.0f;
      pedidoCero = false;
    }
    const float g = leerGrados();
    q = g;
    fc = digitalRead(PIN_F) == LOW;
    if (!activo) {
      uHold = 0.0f;
      if (u != 0.0f) aplicar(0.0f);  // por si parar() llegó en medio de un periodo
      continue;
    }
    const float dt = TS_MS / 1000.0f;
    if (reiniciar) {  // arranca donde está, sin salto
      ref = g;
      uHold = 0.0f;
      msError = msAtasco = 0;
      atascada = false;
      qAtasco = g;
      reiniciar = false;
    }
    // Referencia hacia el destino a OTRA.vel °/s
    const float paso = OTRA.vel * dt, d = dest - ref;
    ref += d > paso ? paso : (d < -paso ? -paso : d);
    const float e = ref - g;
    uHold += ki * e * dt;
    uHold = constrain(uHold, -OTRA.dutyMax, OTRA.dutyMax);
    float v = constrain(uHold + kp * e, -OTRA.dutyMax, OTRA.dutyMax);
    // Final de carrera apretado (está en el lado -): no empujar contra él
    if (fc && v < 0.0f) {
      v = 0.0f;
      if (uHold < 0.0f) uHold = 0.0f;
    }
    // Atasco: duty alto sin moverse -> deja de empujar (freno) hasta el próximo sostener()
    if (fabsf(v) >= OTRA_ATASCO_DUTY && fabsf(g - qAtasco) < 0.5f) {
      msAtasco += TS_MS;
      if (msAtasco >= OTRA_ATASCO_MS && !atascada) {
        atascada = true;
        avisoPendiente = "ATASCO: deja de empujar (freno). Revisar y mandar \"o <grados>\".";
      }
    } else {
      msAtasco = 0;
      qAtasco = g;
    }
    if (atascada) v = 0.0f;
    // Error grande sostenido: solo avisa
    if (fabsf(e) >= OTRA_ERROR_AVISO) {
      msError += TS_MS;
      if (msError == OTRA_ERROR_MS) avisoPendiente = "no llega a la referencia (error de más de 5° durante 1 s).";
    } else {
      msError = 0;
    }
    aplicar(v);
  }
}

}  // namespace

bool begin() {
  pinMode(PIN_F, INPUT_PULLUP);
  const bool okM = motor.begin(PIN_R, PIN_L, PIN_BRAZO_EN, PIN_BRAZO_EN, CH_R, CH_L, OTRA.pwmHz, art::PWM_BITS);
  const bool okE = enc.begin(PIN_A, PIN_B, PCNT_UNIT_1);
  if (!okM || !okE) return false;
  ceroCuentas = enc.count();
  return xTaskCreatePinnedToCore(tarea, "otra", 4096, nullptr, 2, nullptr, 0) == pdPASS;
}

void cero() { pedidoCero = true; }

void sostener(float g) {
  dest = constrain(g, OTRA.limNeg, OTRA.limPos);
  if (!activo) {
    // El puente pudo quedar deshabilitado por la activa ("x"): coast() pone en cero el estado de la biblioteca y el
    // primer setDuty() vuelve a habilitar el EN
    motor.coast();
    motor.brake();
    reiniciar = true;
    activo = true;
  }
}

void parar() {
  activo = false;
  aplicar(0.0f);
}

bool activa() { return activo; }
float grados() { return q; }
float destino() { return dest; }
float duty() { return u; }
bool finalCarrera() { return fc; }
void ganancias(float p, float i) {
  kp = p;
  ki = i;
}
float khp() { return kp; }
float khi() { return ki; }

const char *aviso() {
  const char *a = (const char *)avisoPendiente;
  avisoPendiente = nullptr;
  return a;
}

}  // namespace otra
