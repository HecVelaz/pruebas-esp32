#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

// Receptor GPS Fastrax UP501 (chip MediaTek, tramas NMEA con talker "GP" por UART, 9600 baud 8N1 por defecto).
// No abre el puerto ni fija pines: recibe un Stream ya configurado (por ejemplo Serial1.begin(9600, ...)).
// Hay que llamar a update() seguido (en cada loop) para no perder bytes del búfer de la UART.
class UP501 {
 public:
  static constexpr uint32_t DEFAULT_BAUD = 9600;

  enum class State : uint8_t {
    NoData,   // no llegó ningún byte en el último timeout: cableado o alimentación
    BadData,  // llegan bytes, pero ninguna trama con checksum válido: baudrate incorrecto o ruido
    NoFix,    // tramas NMEA válidas, sin posición (buscando satélites)
    Fix,      // posición válida y reciente
  };

  struct Fix {
    double latDeg;
    double lonDeg;
    double altMslM;      // altura sobre el nivel del mar (GGA), NAN si no llegó
    double geoidSepM;    // separación geoide-elipsoide (GGA), NAN si el receptor no la envía
    double hdop;         // NAN si no llegó
    uint8_t satsUsed;    // satélites usados en la solución (GGA)
    uint8_t quality;     // calidad de GGA: 1 = GPS, 2 = DGPS/SBAS
    uint8_t mode;        // modo de GSA: 2 = 2D, 3 = 3D, 0 = desconocido
    uint32_t ageMs;      // antigüedad de la última posición
  };

  // noDataTimeoutMs: tiempo sin bytes (o sin tramas válidas) para declarar NoData (o BadData).
  // fixMaxAgeMs: una posición más vieja que esto no cuenta como fix (el receptor manda una por segundo).
  explicit UP501(Stream &port, uint32_t noDataTimeoutMs = 3000, uint32_t fixMaxAgeMs = 2500);

  // Procesa todos los bytes pendientes. Si echo no es nullptr, copia ahí las tramas crudas.
  void update(Print *echo = nullptr);

  State state();
  // Devuelve true y completa out solo si state() == Fix
  bool readFix(Fix &out);
  // Satélites en vista según GSV; -1 si todavía no llegó ninguna GSV
  int satellitesInView();

  uint32_t charsProcessed() const { return gps_.charsProcessed(); }
  uint32_t passedChecksum() const { return gps_.passedChecksum(); }
  uint32_t failedChecksum() const { return gps_.failedChecksum(); }

  // Acceso al parser para fecha, hora, velocidad y rumbo
  TinyGPSPlus &parser() { return gps_; }

  static const char *stateName(State s);

 private:
  static double customToDouble(TinyGPSCustom &c);

  Stream &port_;
  uint32_t noDataTimeoutMs_;
  uint32_t fixMaxAgeMs_;

  TinyGPSPlus gps_;
  // Campos que TinyGPSPlus no expone. Los nombres de trama incluyen el talker del UP501 ("GP").
  TinyGPSCustom satsInView_;  // GSV, campo 3
  TinyGPSCustom ggaQuality_;  // GGA, campo 6
  TinyGPSCustom geoidSep_;    // GGA, campo 11
  TinyGPSCustom gsaMode_;     // GSA, campo 2

  bool anyByte_ = false;
  uint32_t lastByteMs_ = 0;
  uint32_t lastValidMs_ = 0;
};
