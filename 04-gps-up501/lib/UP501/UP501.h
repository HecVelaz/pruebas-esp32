#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

// Receptor GPS Fastrax UP501 (chip MediaTek, tramas NMEA con talker "GP" por UART, 9600 baud 8N1 por defecto).
// No abre el puerto ni fija pines: recibe un Stream ya configurado (por ejemplo Serial1.begin(9600, ...)).
// Hay que llamar a update() seguido (en cada loop) para no perder bytes del búfer de la UART.
//
// El receptor manda un epoch por segundo: GGA, GSA, GSV y RMC con la misma hora UTC. El driver arma una
// muestra por epoch cuando llegan la GGA y la RMC de esa hora, con los campos leídos de esas mismas tramas
// (un campo vacío queda en NAN, nunca con el valor de un epoch anterior).
class UP501 {
 public:
  static constexpr uint32_t DEFAULT_BAUD = 9600;

  enum class State : uint8_t {
    NoData,   // no llegó ningún byte en el último timeout: cableado o alimentación
    BadData,  // llegan bytes, pero ninguna trama con checksum válido: baudrate incorrecto o ruido
    NoFix,    // tramas NMEA válidas, sin posición (buscando satélites o fix perdido)
    Fix,      // el último epoch tiene posición válida y es reciente
  };

  // Una muestra completa (un epoch). Sin fix, las coordenadas, la altura y el HDOP quedan en NAN.
  struct Fix {
    bool fix;            // GGA con calidad > 0 y coordenadas, y RMC con estado 'A'
    double latDeg;
    double lonDeg;
    double altMslM;      // altura sobre el nivel del mar (GGA), NAN si el campo vino vacío
    double geoidSepM;    // separación geoide-elipsoide (GGA), NAN si el receptor no la envía
    double hdop;         // NAN si el campo vino vacío
    uint8_t satsUsed;    // satélites usados en la solución (GGA)
    int satsInView;      // satélites en vista (última GSV), -1 si todavía no llegó ninguna
    uint8_t quality;     // calidad de GGA: 0 = sin fix, 1 = GPS, 2 = diferencial
    uint8_t mode;        // modo de la última GSA: 1 = sin fix, 2 = 2D, 3 = 3D, 0 = desconocido
    // Hora UTC del epoch. Sin fix, el reloj del receptor puede no ser real (por ejemplo, año 1980).
    uint8_t hour, minute, second;
    uint16_t msec;
    bool dateValid;      // la RMC trajo una fecha de calendario válida (ddmmyy)
    uint16_t year;
    uint8_t month, day;
    // millis() del ESP32 al procesar la última trama del epoch en update(). Es posterior a la llegada física
    // por la UART en lo que tarde el loop en llamar a update().
    uint32_t rxMs;
    uint32_t ageMs;      // antigüedad del epoch al leerlo
  };

  // noDataTimeoutMs: tiempo sin bytes (o sin tramas válidas) para declarar NoData (o BadData).
  // fixMaxAgeMs: un epoch más viejo que esto no cuenta como fix (el receptor manda uno por segundo).
  explicit UP501(Stream &port, uint32_t noDataTimeoutMs = 3000, uint32_t fixMaxAgeMs = 2500);

  // Procesa todos los bytes pendientes. Si echo no es nullptr, copia ahí las tramas crudas.
  void update(Print *echo = nullptr);

  State state();
  // Devuelve true y completa out con el último epoch solo si state() == Fix
  bool readFix(Fix &out);
  // Entrega los epochs nuevos en orden, uno por llamada (con fix o sin fix). false si no hay pendientes.
  bool pollEpoch(Fix &out);
  // Satélites en vista según GSV; -1 si todavía no llegó ninguna GSV
  int satellitesInView();

  // Hora UTC del epoch en milisegundos Unix. false si el epoch no trae fecha válida.
  static bool utcUnixMs(const Fix &f, int64_t &ms);

  uint32_t charsProcessed() const { return gps_.charsProcessed(); }
  uint32_t passedChecksum() const { return gps_.passedChecksum(); }
  uint32_t failedChecksum() const { return gps_.failedChecksum(); }
  uint32_t epochCount() const { return epochCount_; }      // epochs completos (GGA + RMC) desde el arranque
  uint32_t droppedEpochs() const { return droppedEpochs_; }  // descartados por cola llena (pollEpoch lento)

  // Acceso al parser para velocidad y rumbo
  TinyGPSPlus &parser() { return gps_; }

  static const char *stateName(State s);

 private:
  // Una GGA o RMC a la espera de su pareja. Vence si la pareja no llega pronto (trama perdida o GPS reiniciado).
  struct GgaHalf {
    bool present;
    char utc[12];
    uint32_t rxMs;
    uint8_t quality;
    bool hasPosition;
    double latDeg, lonDeg, altMslM, geoidSepM, hdop;
    uint8_t satsUsed;
  };
  struct RmcHalf {
    bool present;
    char utc[12];
    uint32_t rxMs;
    bool active;  // estado 'A'
    char date[8];
  };
  static constexpr uint32_t HALF_MAX_AGE_MS = 1500;
  static constexpr uint8_t QUEUE_LEN = 4;

  static double customToDouble(TinyGPSCustom &c);
  static bool customNotEmpty(TinyGPSCustom &c);
  static bool customIs(TinyGPSCustom &c, char a, char b);
  static bool parseUtc(const char *utc, Fix &f);
  static bool parseDate(const char *date, Fix &f);
  void onGga(uint32_t now);
  void onRmc(uint32_t now);
  void tryPair(uint32_t now);

  Stream &port_;
  uint32_t noDataTimeoutMs_;
  uint32_t fixMaxAgeMs_;

  TinyGPSPlus gps_;
  // Campos leídos directo de cada trama. Los nombres incluyen el talker del UP501 ("GP").
  TinyGPSCustom ggaTime_;     // GGA, campo 1 (hhmmss.sss)
  TinyGPSCustom ggaLat_;      // GGA, campo 2 (vacío sin posición)
  TinyGPSCustom ggaLatHem_;   // GGA, campo 3 (N/S)
  TinyGPSCustom ggaLon_;      // GGA, campo 4
  TinyGPSCustom ggaLonHem_;   // GGA, campo 5 (E/W)
  TinyGPSCustom ggaQuality_;  // GGA, campo 6
  TinyGPSCustom ggaSats_;     // GGA, campo 7
  TinyGPSCustom ggaHdop_;     // GGA, campo 8
  TinyGPSCustom ggaAlt_;      // GGA, campo 9
  TinyGPSCustom geoidSep_;    // GGA, campo 11
  TinyGPSCustom gsaMode_;     // GSA, campo 2
  TinyGPSCustom satsInView_;  // GSV, campo 3
  TinyGPSCustom rmcTime_;     // RMC, campo 1 (hhmmss.sss)
  TinyGPSCustom rmcStatus_;   // RMC, campo 2 (A = válido, V = inválido)
  TinyGPSCustom rmcDate_;     // RMC, campo 9 (ddmmyy)

  bool anyByte_ = false;
  uint32_t lastByteMs_ = 0;
  uint32_t lastValidMs_ = 0;

  GgaHalf gga_ = {};
  RmcHalf rmc_ = {};

  Fix last_ = {};  // último epoch completo (para state() y readFix())
  bool hasEpoch_ = false;
  Fix queue_[QUEUE_LEN] = {};
  uint8_t queueHead_ = 0;
  uint8_t queueCount_ = 0;
  uint32_t epochCount_ = 0;
  uint32_t droppedEpochs_ = 0;
};
