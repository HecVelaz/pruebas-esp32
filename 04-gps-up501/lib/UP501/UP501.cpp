#include "UP501.h"

UP501::UP501(Stream &port, uint32_t noDataTimeoutMs, uint32_t fixMaxAgeMs)
    : port_(port),
      noDataTimeoutMs_(noDataTimeoutMs),
      fixMaxAgeMs_(fixMaxAgeMs),
      ggaTime_(gps_, "GPGGA", 1),
      ggaLat_(gps_, "GPGGA", 2),
      ggaLatHem_(gps_, "GPGGA", 3),
      ggaLon_(gps_, "GPGGA", 4),
      ggaLonHem_(gps_, "GPGGA", 5),
      ggaQuality_(gps_, "GPGGA", 6),
      ggaSats_(gps_, "GPGGA", 7),
      ggaHdop_(gps_, "GPGGA", 8),
      ggaAlt_(gps_, "GPGGA", 9),
      geoidSep_(gps_, "GPGGA", 11),
      gsaMode_(gps_, "GPGSA", 2),
      satsInView_(gps_, "GPGSV", 3),
      rmcTime_(gps_, "GPRMC", 1),
      rmcStatus_(gps_, "GPRMC", 2),
      rmcDate_(gps_, "GPRMC", 9) {}

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
    if (!gps_.encode(c)) continue;
    lastValidMs_ = now;

    // isUpdated() marca la trama que acaba de terminar (value() lo borra)
    if (ggaTime_.isUpdated()) onGga(now);
    else if (rmcTime_.isUpdated()) onRmc(now);
  }
}

void UP501::onGga(uint32_t now) {
  GgaHalf h = {};
  h.present = true;
  strlcpy(h.utc, ggaTime_.value(), sizeof(h.utc));
  h.rxMs = now;
  const double q = customToDouble(ggaQuality_);
  h.quality = isnan(q) ? 0 : (uint8_t)q;
  // TinyGPSPlus confirma la posición de esta GGA solo si trae fix, pero conserva la latitud o la longitud
  // anterior si ese campo llega vacío: se exigen los cuatro campos (valores y hemisferios) en esta GGA
  const bool positionPresent = customNotEmpty(ggaLat_) && customIs(ggaLatHem_, 'N', 'S') &&
                               customNotEmpty(ggaLon_) && customIs(ggaLonHem_, 'E', 'W');
  h.hasPosition = h.quality > 0 && positionPresent && gps_.location.isValid();
  h.latDeg = h.hasPosition ? gps_.location.lat() : NAN;
  h.lonDeg = h.hasPosition ? gps_.location.lng() : NAN;
  h.altMslM = customToDouble(ggaAlt_);
  h.geoidSepM = customToDouble(geoidSep_);
  const double hdop = customToDouble(ggaHdop_);
  h.hdop = hdop > 0 ? hdop : NAN;  // también descarta HDOP 0
  const double sats = customToDouble(ggaSats_);
  h.satsUsed = isnan(sats) ? 0 : (uint8_t)sats;
  gga_ = h;
  tryPair(now);
}

void UP501::onRmc(uint32_t now) {
  RmcHalf h = {};
  h.present = true;
  strlcpy(h.utc, rmcTime_.value(), sizeof(h.utc));
  h.rxMs = now;
  h.active = rmcStatus_.isValid() && rmcStatus_.value()[0] == 'A';
  strlcpy(h.date, rmcDate_.isValid() ? rmcDate_.value() : "", sizeof(h.date));
  rmc_ = h;
  tryPair(now);
}

void UP501::tryPair(uint32_t now) {
  // Una mitad vieja no se empareja: su pareja se perdió o el GPS se reinició y repite horas
  if (gga_.present && now - gga_.rxMs > HALF_MAX_AGE_MS) gga_.present = false;
  if (rmc_.present && now - rmc_.rxMs > HALF_MAX_AGE_MS) rmc_.present = false;
  if (!gga_.present || !rmc_.present || strcmp(gga_.utc, rmc_.utc) != 0) return;

  Fix e = {};
  e.quality = gga_.quality;
  e.fix = gga_.hasPosition && rmc_.active;
  e.latDeg = e.fix ? gga_.latDeg : NAN;
  e.lonDeg = e.fix ? gga_.lonDeg : NAN;
  e.altMslM = e.fix ? gga_.altMslM : NAN;
  e.geoidSepM = e.fix ? gga_.geoidSepM : NAN;
  e.hdop = e.fix ? gga_.hdop : NAN;
  e.satsUsed = gga_.satsUsed;
  e.satsInView = satellitesInView();
  const double m = customToDouble(gsaMode_);
  e.mode = isnan(m) ? 0 : (uint8_t)m;
  if (!parseUtc(rmc_.utc, e)) e.fix = false;  // sin hora legible no se puede fechar la muestra
  e.dateValid = parseDate(rmc_.date, e);
  e.rxMs = now;

  // Cada mitad se usa una sola vez
  gga_.present = false;
  rmc_.present = false;

  last_ = e;
  hasEpoch_ = true;
  epochCount_++;
  if (queueCount_ == QUEUE_LEN) {
    // Cola llena: se descarta el más viejo
    queueHead_ = (queueHead_ + 1) % QUEUE_LEN;
    queueCount_--;
    droppedEpochs_++;
  }
  queue_[(queueHead_ + queueCount_) % QUEUE_LEN] = e;
  queueCount_++;
}

UP501::State UP501::state() {
  const uint32_t now = millis();
  if (!anyByte_ || now - lastByteMs_ > noDataTimeoutMs_) return State::NoData;
  if (now - lastValidMs_ > noDataTimeoutMs_) return State::BadData;
  // Un epoch sin fix (GGA calidad 0 o RMC 'V') da NoFix de inmediato, aunque la posición anterior sea reciente
  if (hasEpoch_ && last_.fix && now - last_.rxMs <= fixMaxAgeMs_) return State::Fix;
  return State::NoFix;
}

bool UP501::readFix(Fix &out) {
  if (state() != State::Fix) return false;
  out = last_;
  out.ageMs = millis() - last_.rxMs;
  return true;
}

bool UP501::pollEpoch(Fix &out) {
  if (queueCount_ == 0) return false;
  out = queue_[queueHead_];
  queueHead_ = (queueHead_ + 1) % QUEUE_LEN;
  queueCount_--;
  out.ageMs = millis() - out.rxMs;
  return true;
}

int UP501::satellitesInView() {
  const double n = customToDouble(satsInView_);
  return isnan(n) ? -1 : (int)n;
}

// hhmmss o hhmmss.sss
bool UP501::parseUtc(const char *utc, Fix &f) {
  const size_t len = strlen(utc);
  if (len < 6) return false;
  for (int i = 0; i < 6; i++)
    if (!isdigit((unsigned char)utc[i])) return false;
  const int hh = (utc[0] - '0') * 10 + (utc[1] - '0');
  const int mm = (utc[2] - '0') * 10 + (utc[3] - '0');
  const int ss = (utc[4] - '0') * 10 + (utc[5] - '0');
  if (hh > 23 || mm > 59 || ss > 60) return false;  // 60: segundo intercalar
  f.hour = hh;
  f.minute = mm;
  f.second = ss;
  f.msec = 0;
  if (len > 6) {
    if (utc[6] != '.') return false;
    const long ms = lround(atof(utc + 6) * 1000.0);
    if (ms < 0 || ms > 999) return false;
    f.msec = (uint16_t)ms;
  }
  return true;
}

// ddmmyy, con validación de calendario
bool UP501::parseDate(const char *date, Fix &f) {
  if (strlen(date) != 6) return false;
  for (int i = 0; i < 6; i++)
    if (!isdigit((unsigned char)date[i])) return false;
  const int dd = (date[0] - '0') * 10 + (date[1] - '0');
  const int mo = (date[2] - '0') * 10 + (date[3] - '0');
  const int yy = (date[4] - '0') * 10 + (date[5] - '0');
  const int year = yy < 80 ? 2000 + yy : 1900 + yy;
  static const uint8_t DAYS[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if (mo < 1 || mo > 12 || dd < 1) return false;
  const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  if (dd > DAYS[mo - 1] + (mo == 2 && leap ? 1 : 0)) return false;
  f.day = dd;
  f.month = mo;
  f.year = year;
  return true;
}

bool UP501::utcUnixMs(const Fix &f, int64_t &ms) {
  if (!f.dateValid) return false;
  // Días desde 1970-01-01 (algoritmo days_from_civil de H. Hinnant)
  const int y = f.year - (f.month <= 2 ? 1 : 0);
  const int era = y / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned mp = (f.month + 9) % 12;
  const unsigned doy = (153 * mp + 2) / 5 + f.day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = (int64_t)era * 146097 + doe - 719468;
  ms = ((days * 24 + f.hour) * 60 + f.minute) * 60000LL + f.second * 1000LL + f.msec;
  return true;
}

bool UP501::customNotEmpty(TinyGPSCustom &c) { return c.isValid() && c.value()[0] != '\0'; }

bool UP501::customIs(TinyGPSCustom &c, char a, char b) {
  if (!c.isValid()) return false;
  const char *v = c.value();
  return (v[0] == a || v[0] == b) && v[1] == '\0';
}

double UP501::customToDouble(TinyGPSCustom &c) {
  // Un campo vacío también se marca válido: tratarlo como ausente
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
