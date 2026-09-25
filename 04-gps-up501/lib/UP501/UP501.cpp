#include "UP501.h"

UP501::UP501(Stream &port, uint32_t noDataTimeoutMs, uint32_t fixMaxAgeMs)
    : port_(port),
      noDataTimeoutMs_(noDataTimeoutMs),
      fixMaxAgeMs_(fixMaxAgeMs),
      satsInView_(gps_, "GPGSV", 3),
      ggaQuality_(gps_, "GPGGA", 6),
      geoidSep_(gps_, "GPGGA", 11),
      gsaMode_(gps_, "GPGSA", 2) {}

void UP501::update(Print *echo) {
  while (port_.available() > 0) {
    const char c = (char)port_.read();
    const uint32_t now = millis();
    if (!anyByte_) {
      anyByte_ = true;
      lastValidMs_ = now;  // el plazo para BadData corre desde el primer byte
    }
    lastByteMs_ = now;
    if (echo) echo->write(c);
    // encode() devuelve true al terminar una trama con checksum correcto
    if (gps_.encode(c)) lastValidMs_ = now;
  }
}

UP501::State UP501::state() {
  const uint32_t now = millis();
  if (!anyByte_ || now - lastByteMs_ > noDataTimeoutMs_) return State::NoData;
  if (now - lastValidMs_ > noDataTimeoutMs_) return State::BadData;
  // TinyGPSPlus deja la última posición como válida aunque se pierda el fix: se exige que sea reciente
  if (gps_.location.isValid() && gps_.location.age() <= fixMaxAgeMs_) return State::Fix;
  return State::NoFix;
}

bool UP501::readFix(Fix &out) {
  if (state() != State::Fix) return false;
  out.latDeg = gps_.location.lat();
  out.lonDeg = gps_.location.lng();
  out.ageMs = gps_.location.age();
  out.altMslM = gps_.altitude.isValid() ? gps_.altitude.meters() : NAN;
  out.geoidSepM = customToDouble(geoidSep_);
  out.hdop = gps_.hdop.isValid() ? gps_.hdop.hdop() : NAN;
  out.satsUsed = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
  const double q = customToDouble(ggaQuality_);
  out.quality = isnan(q) ? 0 : (uint8_t)q;
  const double m = customToDouble(gsaMode_);
  out.mode = isnan(m) ? 0 : (uint8_t)m;
  return true;
}

int UP501::satellitesInView() {
  const double n = customToDouble(satsInView_);
  return isnan(n) ? -1 : (int)n;
}

double UP501::customToDouble(TinyGPSCustom &c) {
  // Un campo vacío (por ejemplo, sin fix) también se marca válido: tratarlo como ausente
  if (!c.isValid()) return NAN;
  const char *v = c.value();
  return (v && v[0] != '\0') ? atof(v) : NAN;
}

const char *UP501::stateName(State s) {
  switch (s) {
    case State::NoData: return "sin datos";
    case State::BadData: return "tramas inválidas";
    case State::NoFix: return "sin fix";
    case State::Fix: return "fix";
  }
  return "?";
}
