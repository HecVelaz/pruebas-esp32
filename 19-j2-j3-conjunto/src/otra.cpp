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
volatile float ref = 0.0f;        // referencia del lazo (la lee la activa para sus límites: peor caso de la franja)
volatile uint32_t ciclos = 0;     // periodos empezados por la tarea (parar() espera a que termine uno entero)
volatile bool destrabar = false;  // sostener() con la tarea ya sosteniendo: borra un ATASCO
volatile bool llegoV = false, frenadaV = false;
// Cada sostener() es un pedido nuevo: la llegada vale solo para el pedido con que se calculó. Así, apenas se manda
// otro "o" (aunque el destino cambie menos de un paso de la referencia), llego() es false sin esperar a la tarea.
volatile uint32_t pedido = 0, pedidoLlegada = 0;
volatile uint32_t tLlegada = 0;   // millis() al llegar
volatile float uHoldV = 0.0f;
volatile float bandaInt = BANDA_INT_HOLD;
volatile bool frenoBanda = FRENO_EN_BANDA;

int64_t ceroCuentas = 0;
float uHold = 0.0f;

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
  uint32_t msError = 0, msAtasco = 0, msQuieta = 0, pedidoVisto = 0;
  float qAtasco = 0.0f;
  bool atascada = false;
  constexpr int VENTANA = 4;  // velocidad con 4 periodos (40 ms), como la activa
  float hist[VENTANA + 1] = {};
  for (;;) {
    vTaskDelayUntil(&ultimo, pdMS_TO_TICKS(TS_MS));
    ciclos = ciclos + 1;
    if (pedidoCero) {
      ceroCuentas = enc.count();
      ref = 0.0f;
      pedidoCero = false;
    }
    const float g = leerGrados();
    q = g;
    fc = digitalRead(PIN_F) == LOW;
    for (int i = 0; i < VENTANA; i++) hist[i] = hist[i + 1];
    hist[VENTANA] = g;
    const float w = (hist[VENTANA] - hist[0]) / (VENTANA * TS_MS / 1000.0f);
    if (!activo) {
      uHold = uHoldV = 0.0f;
      llegoV = frenadaV = false;
      if (u != 0.0f) aplicar(0.0f);  // por si parar() llegó en medio de un periodo
      continue;
    }
    const float dt = TS_MS / 1000.0f;
    if (reiniciar) {  // arranca donde está, sin salto
      // El EN es común: si la activa lo bajó ("x"), el estado enabled_ de este BTS7960 quedó en "habilitado" y su
      // setDuty() no lo vuelve a subir. Primero PWM en 0 y después EN en alto: las tres quedan frenadas, sin bajar
      // nunca el EN (antes coast() lo bajaba un instante y soltaba la activa y J1). Solo esta tarea toca este motor.
      aplicar(0.0f);
      digitalWrite(PIN_BRAZO_EN, HIGH);
      ref = g;
      uHold = 0.0f;
      msError = msAtasco = msQuieta = 0;
      atascada = false;
      qAtasco = g;
      llegoV = frenadaV = false;
      for (auto &h : hist) h = g;
      reiniciar = destrabar = false;
    }
    if (destrabar) {  // "o" de nuevo: deja de estar atascada y vuelve a empujar
      atascada = false;
      msAtasco = 0;
      qAtasco = g;
      destrabar = false;
    }
    // Referencia hacia el destino a OTRA.vel °/s
    const uint32_t pe = pedido;  // leído antes que dest: si llega uno nuevo en el medio, se ve en el próximo periodo
    const float de = dest, r = ref, paso = OTRA.vel * dt, d = de - r;
    ref = r + (d > paso ? paso : (d < -paso ? -paso : d));
    const float e = ref - g;
    // Llegada (mismo criterio que la activa): referencia en el destino, cerca y quieta LLEGADA_MS; con histéresis
    const float eDest = de - g;
    bool llego = llegoV;
    if (pe != pedidoVisto) {  // pedido nuevo: tiene que volver a llegar
      llego = false;
      msQuieta = 0;
      pedidoVisto = pe;
    }
    if (ref != de) {
      llego = false;
      msQuieta = 0;
    } else if (llego) {
      if (fabsf(eDest) > otra_cfg::TOL_SALIDA) {
        llego = false;
        msQuieta = 0;
      }
    } else {
      msQuieta = (fabsf(eDest) < otra_cfg::TOL_POS && fabsf(w) < 1.0f) ? msQuieta + TS_MS : 0;
      if (msQuieta >= otra_cfg::LLEGADA_MS) {
        llego = true;
        tLlegada = millis();
      }
    }
    pedidoLlegada = pe;
    llegoV = llego;
    // Freno dentro de la banda: llegó -> duty 0 y uHold quieto, hasta que se aleje más de TOL_SALIDA
    const bool frenada = frenoBanda && llego && pe == pedido;
    frenadaV = frenada;
    // Banda del integrador: errores más chicos no integran (no va y viene entre dos puntos por la fricción)
    if (!frenada && fabsf(e) >= bandaInt) uHold += ki * e * dt;
    uHold = constrain(uHold, -OTRA.dutyMax, OTRA.dutyMax);
    uHoldV = uHold;
    float v = frenada ? 0.0f : constrain(uHold + kp * e, -OTRA.dutyMax, OTRA.dutyMax);
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
  pedido = pedido + 1;  // antes que dest: la tarea no puede dar por llegado el destino nuevo con el pedido viejo
  dest = constrain(g, OTRA.limNeg, OTRA.limPos);
  if (!activo) {
    // El EN (por si "x" lo bajó) lo vuelve a subir la tarea al reiniciar, sin pasar por coast(): ver tarea()
    reiniciar = true;
    activo = true;
  } else {
    destrabar = true;
  }
}

// El duty 0 lo aplica la tarea (así solo ella toca este motor). Se espera a que empiece un periodo DESPUÉS de bajar
// "activo" y a que termine: ese ya vio activo = false y aplicó 0, y el que estaba en curso (que pudo haber visto
// true, e incluso subir el EN al reiniciar) ya terminó. Así, el que llama puede bajar el EN sin que la tarea lo
// vuelva a subir. Como mucho ~3 periodos (30 ms; 50 ms de tope por las dudas).
void parar() {
  activo = false;
  const uint32_t c0 = ciclos, t0 = millis();
  while (ciclos - c0 < 2 && millis() - t0 < 5 * TS_MS) delay(1);
}

bool activa() { return activo; }
float grados() { return q; }
float destino() { return dest; }
float referencia() { return ref; }
bool enCamino() { return activo && (reiniciar || !llegoV); }
bool llego() { return activo && !reiniciar && llegoV && pedidoLlegada == pedido; }
uint32_t msDesdeLlegada() { return llego() ? millis() - tLlegada : 0; }
bool frenada() { return llego() && frenadaV; }
float uHold() { return uHoldV; }
void opciones(float b, bool f) {
  bandaInt = b;
  frenoBanda = f;
}
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
