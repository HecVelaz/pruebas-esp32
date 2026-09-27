#include "ControlArticulacion.h"

#include <math.h>

static float limitar(float v, float lim) { return v > lim ? lim : (v < -lim ? -lim : v); }
static float signo(float v) { return v > 0.0f ? 1.0f : (v < 0.0f ? -1.0f : 0.0f); }

// Filtro de la velocidad medida: con 3200-4000 cuentas por vuelta y 10 ms, una cuenta son ~1,5-1,9 rpm
static constexpr float TF_VEL = 0.02f;          // s
// Por debajo de esto no se aplica el feedforward de zona muerta (evita vibrar alrededor de 0)
static constexpr float W_MIN_FF = 0.5f;         // rpm
// Saltos del encoder: más de este múltiplo de lo posible a rpmMax en un paso se descarta
static constexpr float FACTOR_SALTO = 3.0f;
static constexpr int SALTOS_PARA_FALLA = 5;

void ControlArticulacion::configurar(const ConfigArticulacion &cfg) {
  cfg_ = cfg;
  modo_ = Modo::Libre;
  falla_ = Falla::Ninguna;
  conCero_ = false;
  primerPaso_ = true;
}

float ControlArticulacion::gradosPorCuenta() const {
  return 360.0f / (cfg_.cuentasPorVuelta * cfg_.relacion);
}

void ControlArticulacion::fijarCero(int64_t cuentas, float qDeg) {
  cuentasCero_ = cuentas;
  cuentasPrev_ = cuentas;
  qCero_ = qDeg;
  q_ = qDeg;
  conCero_ = true;
  primerPaso_ = false;
  detener();
}

bool ControlArticulacion::irA(float qDeg) {
  if (falla_ != Falla::Ninguna || !conCero_ || !isfinite(qDeg)) return false;
  if (qDeg < cfg_.qMin) qDeg = cfg_.qMin;
  if (qDeg > cfg_.qMax) qDeg = cfg_.qMax;
  if (modo_ != Modo::Posicion) {
    wRefGps_ = 0.0f;
    integ_ = 0.0f;
  }
  qObj_ = qDeg;
  modo_ = Modo::Posicion;
  enReposo_ = fabsf(qObj_ - q_) < cfg_.tolPos;
  return true;
}

bool ControlArticulacion::velocidad(float rpm) {
  if (falla_ != Falla::Ninguna || !isfinite(rpm)) return false;
  if (modo_ != Modo::Velocidad) integ_ = 0.0f;
  wCmdRpm_ = limitar(rpm, cfg_.rpmMax);
  modo_ = Modo::Velocidad;
  return true;
}

bool ControlArticulacion::duty(float pct) {
  if (falla_ != Falla::Ninguna || !isfinite(pct)) return false;
  uCmd_ = limitar(pct, cfg_.dutyMax);
  modo_ = Modo::Duty;
  return true;
}

void ControlArticulacion::detener() {
  qObj_ = q_;
  wRefGps_ = 0.0f;
  wRefRpm_ = 0.0f;
  integ_ = 0.0f;
  enReposo_ = true;
  modo_ = conCero_ ? Modo::Posicion : Modo::Libre;
}

void ControlArticulacion::liberar() {
  modo_ = Modo::Libre;
  wRefGps_ = 0.0f;
  wRefRpm_ = 0.0f;
  integ_ = 0.0f;
}

void ControlArticulacion::borrarFalla() {
  falla_ = Falla::Ninguna;
  atascoT_ = 0.0f;
  saltosSeguidos_ = 0;
  detener();
}

const char *ControlArticulacion::textoFalla() const {
  switch (falla_) {
    case Falla::Atasco: return "atasco";
    case Falla::FueraDeLimites: return "fuera de limites";
    case Falla::Encoder: return "encoder (saltos)";
    default: return "ninguna";
  }
}

void ControlArticulacion::entrarFalla(Falla f) {
  falla_ = f;
  liberar();
}

float ControlArticulacion::paso(int64_t cuentas, float dt) {
  if (dt <= 0.0f) return u_;
  if (primerPaso_) {
    cuentasPrev_ = cuentas;
    primerPaso_ = false;
  }

  // Descartar lecturas imposibles (ver el salto falso de 32 000 cuentas pendiente en EncoderPCNT)
  const float maxCuentas = FACTOR_SALTO * cfg_.rpmMax / 60.0f * cfg_.cuentasPorVuelta * dt + 2.0f;
  int64_t d = cuentas - cuentasPrev_;
  if (fabsf((float)d) > maxCuentas) {
    saltos_++;
    if (++saltosSeguidos_ >= SALTOS_PARA_FALLA) entrarFalla(Falla::Encoder);
    d = 0;
    cuentas = cuentasPrev_;
  } else {
    saltosSeguidos_ = 0;
  }
  cuentasPrev_ = cuentas;

  // Medición
  const float wInst = (float)d / cfg_.cuentasPorVuelta / dt * 60.0f;
  w_ += dt / (TF_VEL + dt) * (wInst - w_);
  q_ = qCero_ + (float)(cuentas - cuentasCero_) * gradosPorCuenta();

  float u = 0.0f;
  if (falla_ == Falla::Ninguna) {
    switch (modo_) {
      case Modo::Posicion: u = pasoPosicion(dt); break;
      case Modo::Velocidad:
        wRefRpm_ = wCmdRpm_;
        u = pasoVelocidad(wCmdRpm_, dt);
        break;
      case Modo::Duty: u = uCmd_; break;
      case Modo::Libre: u = 0.0f; break;
    }
  }
  u_ = u;
  revisarProtecciones(dt);
  if (falla_ != Falla::Ninguna) u_ = 0.0f;
  return u_;
}

float ControlArticulacion::pasoPosicion(float dt) {
  const float e = qObj_ - q_;
  // Histéresis: se frena dentro de tolPos y se vuelve a mover recién con el doble de error
  if (enReposo_ && fabsf(e) < 2.0f * cfg_.tolPos) {
    wRefGps_ = 0.0f;
    wRefRpm_ = 0.0f;
    integ_ = 0.0f;
    return 0.0f;
  }
  enReposo_ = false;
  if (fabsf(e) < cfg_.tolPos) {
    enReposo_ = true;
    wRefGps_ = 0.0f;
    wRefRpm_ = 0.0f;
    integ_ = 0.0f;
    return 0.0f;
  }
  // P con saturación de velocidad y rampa de aceleración: perfil aproximadamente trapezoidal
  const float wDes = limitar(cfg_.kpPos * e, cfg_.velMax);
  wRefGps_ += limitar(wDes - wRefGps_, cfg_.accMax * dt);
  wRefRpm_ = rpmDeGradosPorSeg(wRefGps_);
  // Cerca del objetivo la velocidad pedida es chica: la zona muerta se compensa igual, en el sentido del
  // error (si no, el motor se clava a ~1° y recién el integrador lo saca, a los tirones)
  return pasoVelocidad(wRefRpm_, dt, signo(e));
}

float ControlArticulacion::pasoVelocidad(float wRef, float dt, float sentidoFf) {
  const float ew = wRef - w_;
  if (sentidoFf == 0.0f && fabsf(wRef) > W_MIN_FF) sentidoFf = signo(wRef);
  const float ff = sentidoFf * cfg_.u0 + wRef / cfg_.K;
  const float uSinI = ff + cfg_.kpVel * ew;
  // Anti-windup condicional: no integrar si ya satura en el mismo sentido del error
  const float uPrueba = uSinI + integ_;
  const bool satura = fabsf(uPrueba) >= cfg_.dutyMax && signo(uPrueba) == signo(ew);
  if (!satura) integ_ += cfg_.kiVel * ew * dt;
  return limitar(uSinI + integ_, cfg_.dutyMax);
}

void ControlArticulacion::revisarProtecciones(float dt) {
  if (falla_ != Falla::Ninguna) return;
  // Atasco: duty alto y el motor no avanza en el sentido pedido (también detecta encoder desconectado)
  if (fabsf(u_) >= cfg_.atascoDuty && signo(u_) * w_ < cfg_.atascoRpm) {
    atascoT_ += dt;
    if (atascoT_ >= cfg_.atascoS) entrarFalla(Falla::Atasco);
  } else {
    atascoT_ = 0.0f;
  }
  // Fuera de límites (solo con cero; en modo duty/velocidad también, si ya hay cero)
  if (conCero_ && (q_ < cfg_.qMin - cfg_.margenLim || q_ > cfg_.qMax + cfg_.margenLim)) {
    entrarFalla(Falla::FueraDeLimites);
  }
}
