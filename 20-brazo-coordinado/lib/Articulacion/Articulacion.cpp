#include "Articulacion.h"

#include <Arduino.h>

namespace {
constexpr uint32_t TS_MS = 10;
constexpr float RAMPA_DESPEGUE = 300.0f;  // %/s: del feedforward de marcha al de despegue
constexpr uint32_t TRABADA_MS = 100;      // en marcha pero quieta: vuelve a la rampa de despegue
constexpr float VEL_QUIETA = 1.0f;        // °/s
constexpr float FR_RAMPA_GRADOS = 0.3f;   // compensación de fricción: llena desde este error
constexpr float TS_POS = 0.02f;           // s, el P de posición corre cada 2 periodos (50 Hz)
constexpr float VEL_LLEGADA = 2.0f;       // °/s: "llegó" exige además ir más despacio que esto
constexpr float REENTRADA_GRADOS = 1.5f;  // después de llegar, si se aleja más: vuelve a la cascada
// Feedforward suave (2026-11-03, paso 2): ff · tanh(|wFf| / W_FF_SUAVE) en vez del escalón en wFf = 0. Con 2 °/s
// (la aproximación) da el 96 %; en los extremos de cada tramo entra y sale sin salto
constexpr float W_FF_SUAVE = 0.5f;        // °/s
}  // namespace

bool Articulacion::begin(const ConfigArticulacion &c) {
  c_ = c;
  pinMode(c_.pinFc, INPUT_PULLUP);
  const bool okM = motor_.begin(c_.pinRpwm, c_.pinLpwm, c_.pinEn, c_.pinEn, c_.canalR, c_.canalL, c_.pwmHz, c_.pwmBits);
  const bool okE = enc_.begin(c_.pinEncA, c_.pinEncB, c_.unidadPcnt);
  motor_.coast();  // el coordinador sube el EN cuando hace falta
  ceroOffset_ = 0;
  const int64_t c0 = cuentas();
  for (auto &h : hist_) h = c0 + ceroOffset_;
  return okM && okE;
}

// Cuentas con los signos aplicados y el cero
int64_t Articulacion::cuentas() const {
  int64_t c = enc_.count();
  if (c_.encoderInvertido) c = -c;
  if (c_.sentidoInvertido) c = -c;
  return c - ceroOffset_;
}

void Articulacion::aplicar(float pct) {
  u_ = pct;
  const float d = (c_.sentidoInvertido ? -pct : pct) / 100.0f;
  motor_.setDuty(d);
}

uint32_t Articulacion::msFirme() const { return firme() ? millis() - tFirme_ : 0; }

// Un paso del PI de velocidad (de 12-14/19): feedforward de fricción con rampa de despegue, anti-windup.
// El feedforward (signo, rampa de despegue y tamaño) sale de wFf, la velocidad de la TRAYECTORIA, no de wRef (la
// pedida, que lleva la corrección de posición): en la aproximación de J1 la corrección cambiaba de signo cerca de
// velocidad 0 y el ff saltaba +17 -> -19 % unas 25 veces por aproximación (2026-11-03, 1b: ida y vuelta en la punta)
float Articulacion::pasoPI(float wRef, float th, float wFf) {
  (void)th;
  const int signo = wFf > 0 ? 1 : (wFf < 0 ? -1 : 0);
  static_assert(TS_MS == 10, "TS");
  if (signo != signoAnt_) {
    ffRampa_ = 0.0f;
    despegada_ = false;
    signoAnt_ = signo;
  }
  if (wRef == 0.0f) {
    integral_ = 0.0f;
    return 0.0f;
  }
  const float e = wRef - w_;
  const bool quieta = fabsf(w_) < VEL_QUIETA;
  if (despegada_) {
    trabadaTicks_ = quieta ? trabadaTicks_ + 1 : 0;
    if (trabadaTicks_ * TS_MS >= TRABADA_MS) {
      despegada_ = false;
      ffRampa_ = 0.0f;
    }
  } else if (!quieta) {
    despegada_ = true;
    trabadaTicks_ = 0;
  }
  const float marcha = fmaxf(signo > 0 ? c_.ffMarchaPos : c_.ffMarchaNeg, signo > 0 ? c_.ffMinPos : 0.0f);
  const float desp = fmaxf(signo > 0 ? c_.ffDespPos : c_.ffDespNeg, marcha);
  if (!despegada_) ffRampa_ = fminf(ffRampa_ + RAMPA_DESPEGUE * TS, desp - marcha);
  const float ff = signo * (marcha + (despegada_ ? 0.0f : ffRampa_)) * tanhf(fabsf(wFf) / W_FF_SUAVE);
  const float uLibre = ff + c_.kpVel * e + integral_;
  const float u = constrain(uLibre, -c_.dutyMax, c_.dutyMax);
  if (u == uLibre || (e > 0) != (uLibre > 0)) integral_ += c_.kiVel * TS * e;
  return u;
}

// Arrancar el PI de velocidad sin salto desde el duty actual, en el sentido sg (primer periodo: wRef = sg·w0).
// Nunca por debajo del feedforward en el sentido del movimiento: si venía frenada (duty 0), la integral negativa
// anulaba el feedforward y arrancaba desde 0 (2026-10-31: el retroceso de J2 después del flanco tardaba > 15 s).
// El feedforward del primer periodo, con el mismo tanh que pasoPI: al empezar un mv (w0 ~ 0) es casi 0 y la integral
// se queda con el duty que traía (J2 subiendo sigue sosteniendo su peso, sin salto a +25 %).
static float integralSinSalto(const ConfigArticulacion &c, float u, int sg, float w0) {
  const float marcha = sg > 0 ? fmaxf(c.ffMarchaPos, c.ffMinPos) : c.ffMarchaNeg;
  const float i = u - sg * marcha * tanhf(fabsf(w0) / W_FF_SUAVE) - c.kpVel * sg * w0;
  return sg * i > 0.0f ? i : 0.0f;
}

void Articulacion::suelta() {
  modo_ = Modo::Suelta;
  llegado_ = firme_ = frenada_ = false;
  integral_ = 0.0f;
  aplicar(0.0f);
}

void Articulacion::rueda() {
  modo_ = Modo::Suelta;
  llegado_ = firme_ = frenada_ = false;
  integral_ = 0.0f;
  u_ = 0.0f;
  motor_.coast();
}

void Articulacion::sostener(float q) {
  if (modo_ == Modo::Sostener && fabsf(q - ref_) < 1e-4f) return;  // ya está en eso
  const float g = this->q();
  ref_ = q;
  modo_ = Modo::Sostener;
  llegado_ = firme_ = frenada_ = false;
  quietoTicks_ = 0;
  wRefPos_ = 0.0f;
  k_ = 1;  // paso() lo incrementa antes: el P de posición corre ya en el primer periodo (si no, pasoPI(0) borraba
           // la integral precargada y el arranque sin salto se perdía)
  atascoTicks_ = sinCuentasTicks_ = contrarioTicks_ = 0;
  signoIda_ = q > g ? 1 : -1;
  if (fabsf(q - g) >= c_.tolPos) {
    const int sg = q > g ? 1 : -1;
    signoAnt_ = sg;  // que pasoPI no reinicie la rampa en el primer periodo
    despegada_ = false;
    ffRampa_ = 0.0f;
    integral_ = integralSinSalto(c_, u_, sg, c_.aMax * TS_POS);
  }
}

void Articulacion::sostenerAqui() {
  ref_ = q();
  modo_ = Modo::Sostener;
  llegado_ = true;
  firme_ = frenada_ = false;
  quietoTicks_ = 0;
  uHold_ = u_;  // sin salto
  integral_ = 0.0f;
  wRefPos_ = 0.0f;
  atascoTicks_ = sinCuentasTicks_ = contrarioTicks_ = 0;
}

void Articulacion::frenarAqui() {
  aplicar(0.0f);
  sostenerAqui();  // uHold = 0: el PI de posición sostiene desde ahí
}

void Articulacion::seguir(float qRef, float wRef) {
  if (modo_ != Modo::Seguir) {
    const int sg = wRef > 0 ? 1 : (wRef < 0 ? -1 : (qRef >= q() ? 1 : -1));
    signoAnt_ = sg;
    despegada_ = false;
    ffRampa_ = 0.0f;
    integral_ = integralSinSalto(c_, u_, sg, fabsf(wRef));
    modo_ = Modo::Seguir;
    llegado_ = firme_ = frenada_ = false;
    atascoTicks_ = sinCuentasTicks_ = contrarioTicks_ = 0;
  }
  ref_ = qRef;
  wRefSeguir_ = wRef;
}

void Articulacion::homing(bool conCero, float recorridoMax) {
  hConCero_ = conCero;
  error_ = nullptr;
  hEstado_ = Homing::EnCurso;
  hFase_ = fc_ ? 2 : 3;
  hInicio_ = q();
  hRecorridoMax_ = recorridoMax > 0.0f ? recorridoMax : c_.homingRecorridoMax;
  hSoltadoValido_ = false;
  hVentanaC_ = cuentas();
  hVentanaMs_ = millis();
  const int sg = hFase_ == 2 ? 1 : -1;
  signoAnt_ = sg;
  despegada_ = false;
  ffRampa_ = 0.0f;
  integral_ = integralSinSalto(c_, u_, sg, hFase_ == 2 ? c_.homingVelAlejar : c_.homingVelBuscar);
  modo_ = Modo::Homing;
  llegado_ = firme_ = frenada_ = false;
}

void Articulacion::homingCancelar() {
  if (modo_ != Modo::Homing) return;
  hEstado_ = Homing::Error;
  error_ = "cancelado";
  sostenerAqui();
}

void Articulacion::ceroAqui() {
  const int64_t c = cuentas();
  ceroOffset_ += c;
  ref_ -= grados(c);
}

void Articulacion::fijarAngulo(float g) {
  const int64_t c = cuentas();
  const int64_t d = c - (int64_t)llroundf(g * c_.cuentasPorGrado);
  ceroOffset_ += d;
  ref_ -= grados(d);
}

void Articulacion::ceroEnFlanco() {
  ceroOffset_ += flanco_;
  ref_ -= grados(flanco_);
  flanco_ = 0;
}

void Articulacion::reiniciarEstado() {
  error_ = nullptr;
  aviso_ = nullptr;
  avisoAtasco_ = false;
  if (hEstado_ != Homing::EnCurso) hEstado_ = Homing::Nada;
}

void Articulacion::cortar(const char *causa) {
  error_ = causa;
  if (modo_ == Modo::Homing) hEstado_ = Homing::Error;
  aplicar(0.0f);
  sostenerAqui();
}

// Protecciones mientras se mueve (como 12-14/19): sin cuentas, al revés, atasco. Cortan y sostienen donde está.
void Articulacion::protecciones(float wRefMov) {
  const bool mueve = wRefMov != 0.0f;
  sinCuentasTicks_ = (mueve && fabsf(u_) >= c_.arranquePct && hist_[VENTANA] == hist_[VENTANA - 1])
                         ? sinCuentasTicks_ + 1 : 0;
  if (sinCuentasTicks_ * TS_MS >= c_.sinCuentasMs) {
    cortar("sin cuentas del encoder (¿suelto o trabada?)");
    return;
  }
  contrarioTicks_ = (mueve && w_ * (wRefMov > 0 ? 1.0f : -1.0f) < -c_.contrarioVel) ? contrarioTicks_ + 1 : 0;
  if (contrarioTicks_ * TS_MS >= c_.contrarioMs) {
    cortar("se mueve al revés de lo pedido");
    return;
  }
  atascoTicks_ = (fabsf(w_) < c_.atascoVel && fabsf(u_) >= c_.atascoDuty) ? atascoTicks_ + 1 : 0;
  if (atascoTicks_ * TS_MS >= c_.atascoMs) cortar("atasco (duty alto sin moverse)");
}

void Articulacion::paso() {
  // Medición (el historial en cuentas sin el cero: un cambio de cero no da un salto de velocidad)
  const int64_t c = cuentas();
  for (int i = 0; i < VENTANA; i++) hist_[i] = hist_[i + 1];
  hist_[VENTANA] = c + ceroOffset_;
  w_ = grados(hist_[VENTANA] - hist_[0]) / (VENTANA * TS);
  fc_ = digitalRead(c_.pinFc) == LOW;
  const float g = grados(c);
  k_++;

  switch (modo_) {
    case Modo::Suelta:
      return;

    case Modo::Homing: {
      if (hFase_ == 2) {  // apretada al empezar: alejarse hasta soltarlo y "salida" grados más
        if (fc_) {
          hSoltadoValido_ = false;
        } else if (!hSoltadoValido_) {
          hSoltado_ = g;
          hSoltadoValido_ = true;
        }
        if (hSoltadoValido_ && g - hSoltado_ >= c_.homingSalida) {
          hFase_ = 3;
          signoAnt_ = -1;
          despegada_ = false;
          ffRampa_ = 0.0f;
          integral_ = integralSinSalto(c_, u_, -1, c_.homingVelBuscar);
        }
      }
      // Flanco en la etapa rápida: el final de carrera se corrió hacia arriba. No se acepta (se tomó rápido): alejarse
      // y volver a buscarlo, todo despacio
      if (hFase_ == 3 && fc_ && hConCero_ && g > c_.homingZonaLenta) {
        hConCero_ = false;
        hFase_ = 2;
        hSoltadoValido_ = false;
        signoAnt_ = 1;
        despegada_ = false;
        ffRampa_ = 0.0f;
        integral_ = 0.0f;
        aviso_ = "homing: el final de carrera se apretó antes de lo esperado; lo busco de nuevo despacio";
        aplicar(0.0f);
        return;
      }
      if (hFase_ == 3 && fc_) {  // flanco
        flanco_ = c;
        hEstado_ = Homing::Listo;
        aplicar(0.0f);
        sostenerAqui();
        return;
      }
      // Buscando: rápido mientras está lejos del cero anterior (si lo hay), despacio en el último tramo (el flanco se
      // toma siempre a homingVelBuscar)
      const float vBuscar = (hConCero_ && g > c_.homingZonaLenta) ? c_.homingVelRapida : c_.homingVelBuscar;
      const float wRef = hFase_ == 2 ? c_.homingVelAlejar : -vBuscar;
      const float u = constrain(pasoPI(wRef, g, wRef), -c_.homingDutyMax, c_.homingDutyMax);
      aplicar(u);
      if (fabsf(g - hInicio_) > hRecorridoMax_) {
        cortar(hRecorridoMax_ < c_.homingRecorridoMax ? "sin cero y lejos del final de carrera (¿recogido?)"
                                                      : "recorrido máximo sin el final de carrera");
        return;
      }
      // Trabada: con el duty al tope, sin avanzar (antes de llegar al tope, el PI va subiendo)
      if (fabsf(u) < c_.homingDutyMax - 1.0f || llabs(c - hVentanaC_) >= 5) {
        hVentanaC_ = c;
        hVentanaMs_ = millis();
      } else if (millis() - hVentanaMs_ >= c_.homingTrabadaMs) {
        cortar("homing: no avanza con el duty al tope (¿trabada?)");
      }
      return;
    }

    case Modo::Seguir: {
      // Trayectoria del coordinador: velocidad pedida = la de la trayectoria + KPP·error, a la cascada
      const float e = ref_ - g;
      const float wPedida = constrain(wRefSeguir_ + c_.kpp * e, -1.5f * c_.vMax, 1.5f * c_.vMax);
      aplicar(pasoPI(wPedida, g, wRefSeguir_));
      if (g > c_.limPos + 0.5f || g < c_.limNeg - 0.5f) {
        cortar("límite propio");
        return;
      }
      // "Al revés" contra el sentido de la TRAYECTORIA, no contra wPedida: al arrancar un mv la referencia sale de donde
      // quedó el destino anterior y, si la articulación quedó unas décimas adelante (J2 en 74,15 con ref 74,00),
      // wPedida empieza negativa; J2 despegaba de la fricción a +5,7 °/s en el sentido del mv y se cortaba por "al
      // revés" (2026-11-04, trayectoria_20261104_223055_cortado). Con la trayectoria quieta (velocidad 0), wPedida.
      protecciones(wRefSeguir_ != 0.0f ? wRefSeguir_ : wPedida);
      return;
    }

    case Modo::Sostener: {
      const float e = ref_ - g;
      if (!llegado_) {
        // Ir con la cascada (P de posición a 50 Hz con v mínima y aceleración limitada)
        if (k_ % 2 == 0) {
          // Llegó: dentro de la tolerancia Y casi quieta (como 19). Si la daba por llegada pasando rápido por el
          // destino, seguía de largo y el sostener (hecho para décimas) no la volvía: 2026-10-31, el retroceso de J2
          // (0 -> 3°) quedaba en 5-6° trabada por la fricción.
          // Con autobloqueo (J1), también si CRUZÓ el destino (el error cambió de signo) a menos de tolSalida, aunque vaya
          // rápido: frena ahí en vez de invertir. 2026-11-04 (trayectoria_20261104_232850): J1 se pegaba 0,4° antes,
          // despegaba a 7 °/s y se pasaba 0,2-0,3°; si invertía para volver quedaba en 39,75-39,87 y si no, en 40,11-40,16
          // (las dos llegadas de la repetibilidad, RP 0,48°). Invertir además hamaca la punta (correa).
          const bool cruzo = c_.autobloqueo && e * signoIda_ < 0.0f && fabsf(e) < c_.tolSalida;
          if ((fabsf(e) < c_.tolPos && fabsf(w_) < VEL_LLEGADA) || cruzo) {
            llegado_ = true;
            // Arranca del duty que sostiene el peso, estimado del feedforward: el medio entre la marcha subiendo y
            // bajando (J2 +8,5 %, J3 -10 %, J1 0). Antes arrancaba del duty con que venía: al final de un recorrido
            // puede estar frenando (2026-10-31, telemetría: J2 llegaba con -20 % y caía 1,7° antes de volver)
            uHold_ = 0.5f * (c_.ffMarchaPos - c_.ffMarchaNeg);
            integral_ = 0.0f;
          } else {
            const float vP = fmaxf(fabsf(c_.kpp * e), c_.vMin);
            const float deseada = (e > 0 ? 1.0f : -1.0f) * fminf(vP, c_.vMax);
            const float dMax = c_.aMax * TS_POS;
            wRefPos_ = constrain(deseada, wRefPos_ - dMax, wRefPos_ + dMax);
          }
        }
        if (!llegado_) {
          aplicar(pasoPI(wRefPos_, g, wRefPos_));
          // Límite propio: fuera de él y yendo hacia afuera, corta (sobrepaso o caída)
          if ((g > c_.limPos + 0.5f && wRefPos_ > 0.0f) || (g < c_.limNeg - 0.5f && wRefPos_ < 0.0f)) {
            cortar("límite propio");
            return;
          }
          protecciones(wRefPos_);
          return;
        }
      }
      // Se alejó mucho (se pasó o cayó): volver a la cascada para acercarse otra vez, sin salto desde el duty actual.
      // Con autobloqueo (freno, sin PI que corrija) desde tolSalida: si no, entre tolSalida y 1,5° quedaba frenada
      // para siempre, ni firme ni corrigiendo
      if (fabsf(e) > (c_.autobloqueo ? c_.tolSalida : REENTRADA_GRADOS)) {
        llegado_ = firme_ = frenada_ = false;
        quietoTicks_ = 0;
        wRefPos_ = 0.0f;
        k_ = 1;
        const int sg = e > 0 ? 1 : -1;
        signoIda_ = sg;
        signoAnt_ = sg;
        despegada_ = false;
        ffRampa_ = 0.0f;
        integral_ = integralSinSalto(c_, u_, sg, c_.aMax * TS_POS);
        aplicar(u_);
        return;
      }
      // Se autobloquea: al llegar, freno; firme si está a menos de tolSalida y quieta llegadaMs
      if (c_.autobloqueo) {
        quietoTicks_ = (fabsf(e) < c_.tolSalida && fabsf(w_) < VEL_QUIETA) ? quietoTicks_ + 1 : 0;
        if (!firme_ && quietoTicks_ * TS_MS >= c_.llegadaMs) {
          firme_ = true;
          tFirme_ = millis();
        }
        frenada_ = true;
        aplicar(0.0f);
        return;
      }
      // Llegó: PI de posición. Firme = a menos de tolPos y quieta llegadaMs (histéresis tolSalida).
      if (firme_) {
        if (fabsf(e) > c_.tolSalida) {
          firme_ = false;
          quietoTicks_ = 0;
        }
      } else {
        quietoTicks_ = (fabsf(e) < c_.tolPos && fabsf(w_) < VEL_QUIETA) ? quietoTicks_ + 1 : 0;
        if (quietoTicks_ * TS_MS >= c_.llegadaMs) {
          firme_ = true;
          tFirme_ = millis();
        }
      }
      // Apoyada en el final de carrera con la referencia más allá: no empuja más contra él. Solo cerca del cero: si
      // el final de carrera queda apretado más arriba (2026-10-31: el de J2 se arrastraba y seguía apretado a 5-6°),
      // no tiene que impedir bajar a la referencia
      const bool contraFc = fc_ && e < 0.0f && g < c_.limNeg + 1.0f;
      frenada_ = frenoBanda_ && firme_ && !contraFc;
      if (!frenada_ && !contraFc && fabsf(e) >= c_.bandaInt)
        uHold_ = constrain(uHold_ + c_.khi * TS * e, -c_.dutyMax, c_.dutyMax);
      const float comp = c_.friccion * constrain(e / FR_RAMPA_GRADOS, -1.0f, 1.0f);
      float u = frenada_ ? 0.0f
                         : constrain(contraFc ? uHold_ : uHold_ + c_.khp * e + comp, -c_.dutyMax, c_.dutyMax);
      if (contraFc && u < 0.0f) u = fmaxf(u, fmaxf(fminf(uHold_, 0.0f), -c_.fcSostenMax));
      aplicar(u);
      // Sostener el peso le pide mucho duty: avisa una vez (no suelta nunca)
      atascoTicks_ = (fabsf(w_) < c_.atascoVel && fabsf(u) >= c_.atascoDuty) ? atascoTicks_ + 1 : 0;
      if (atascoTicks_ * TS_MS >= c_.atascoMs && !avisoAtasco_) {
        aviso_ = "sostener pide mucho duty";
        avisoAtasco_ = true;
      }
      return;
    }
  }
}
