#pragma once

#include <Arduino.h>
#include <Wire.h>

// Límites de la calibración del bias del giroscopio (ver MPU9250::calibrateGyro)
struct GyroCalConfig {
  uint16_t samples = 400;        // lecturas cada 5 ms (400 = 2 s)
  float maxRangeDps = 3.0f;      // variación máxima por eje del giroscopio
  float maxBiasDps = 10.0f;      // bias máximo aceptado por eje (ZRO del datasheet: ±5 °/s a 25 °C)
  float maxAccelRangeG = 0.05f;  // variación máxima por eje del acelerómetro
  float accelNormTolG = 0.1f;    // el módulo de la aceleración debe estar en 1 g ± tol
};

// Driver mínimo para MPU-9250 / MPU-6500 por I2C.
// Acelerómetro y giroscopio: registros del MPU. Magnetómetro: AK8963 (solo en MPU-9250),
// accesible directamente en el bus I2C activando el modo bypass.
class MPU9250 {
 public:
  enum class Model : uint8_t { Unknown, MPU6500, MPU9250 };

  enum class Status : uint8_t {
    Ok,
    I2cError,       // el dispositivo no responde en el bus
    UnknownDevice,  // responde, pero WHO_AM_I no es 0x70 ni 0x71
    ConfigError,    // falló una escritura durante el reset o la configuración
  };

  enum class MagStatus : uint8_t { Ok, NotReady, Overflow, I2cError, NoMag };

  enum class CalStatus : uint8_t {
    Ok,
    I2cError,
    Moving,        // el giroscopio o el acelerómetro variaron más de lo permitido
    NotLevel,      // el módulo de la aceleración no es ~1 g (aceleración lineal)
    BiasTooLarge,  // el promedio supera maxBiasDps (probable giro constante)
  };

  // Ancho de banda del filtro digital pasa bajos (DLPF), igual para giroscopio y acelerómetro.
  // El valor es DLPF_CFG / A_DLPFCFG del register map.
  enum class Dlpf : uint8_t { Hz184 = 1, Hz92 = 2, Hz41 = 3, Hz20 = 4, Hz10 = 5, Hz5 = 6 };

  struct Vec3 {
    float x, y, z;
  };

  static constexpr uint8_t ADDR_AD0_LOW = 0x68;
  static constexpr uint8_t ADDR_AD0_HIGH = 0x69;
  static constexpr uint8_t AK8963_ADDR = 0x0C;

  static constexpr uint8_t WHO_AM_I_MPU6500 = 0x70;
  static constexpr uint8_t WHO_AM_I_MPU9250 = 0x71;

  static constexpr uint16_t SAMPLE_RATE_HZ = 200;

  explicit MPU9250(TwoWire &wire = Wire, uint8_t addr = ADDR_AD0_LOW);

  // Cambia la dirección I2C (0x68 con AD0 a GND, 0x69 con AD0 a VCC). Llamar antes de begin().
  void setAddress(uint8_t addr) { addr_ = addr; }

  // Lee WHO_AM_I, resetea e inicializa el sensor (±2 g, ±250 °/s, 200 Hz, DLPF indicado).
  // Si es MPU-9250, además inicializa el magnetómetro (ver hasMag()).
  Status begin(Dlpf dlpf = Dlpf::Hz41);

  // Devuelve false si falla la lectura I2C (id queda sin modificar).
  bool readWhoAmI(uint8_t &id);
  // Último WHO_AM_I leído por begin() (0 si no se pudo leer).
  uint8_t whoAmI() const { return whoAmI_; }

  Model model() const { return model_; }
  const char *modelName() const;
  bool hasMag() const { return magOk_; }

  bool setDlpf(Dlpf dlpf);
  Dlpf dlpf() const { return dlpf_; }

  // Lee acelerómetro [g], giroscopio [°/s] y temperatura [°C] en una sola transacción.
  // Al giroscopio se le resta el bias de calibración (cero hasta llamar a calibrateGyro()).
  bool readAccelGyro(Vec3 &accel, Vec3 &gyro, float &tempC);

  // Lee el magnetómetro [µT] si hay un dato nuevo. Solo con MagStatus::Ok se escribe mag.
  MagStatus readMag(Vec3 &mag);

  // Promedia el giroscopio con el sensor quieto y guarda el resultado como bias.
  // Corta en cuanto una lectura sale de los límites. Solo con CalStatus::Ok cambia el bias.
  // No distingue un giro lento y constante alrededor de la vertical de un offset: el sensor debe estar quieto.
  CalStatus calibrateGyro(const GyroCalConfig &cfg = GyroCalConfig());
  Vec3 gyroBias() const { return gyroBias_; }
  void setGyroBias(const Vec3 &bias) { gyroBias_ = bias; }

  static const char *statusName(Status s);
  static const char *magStatusName(MagStatus s);
  static const char *calStatusName(CalStatus s);

 private:
  bool writeReg(uint8_t dev, uint8_t reg, uint8_t val);
  bool readRegs(uint8_t dev, uint8_t reg, uint8_t *buf, size_t len);
  bool initMag();

  TwoWire &wire_;
  uint8_t addr_;
  uint8_t whoAmI_ = 0;
  Model model_ = Model::Unknown;
  Dlpf dlpf_ = Dlpf::Hz41;
  bool magOk_ = false;
  float magAdj_[3] = {1.0f, 1.0f, 1.0f};  // ajuste de sensibilidad (ASA) del AK8963
  Vec3 gyroBias_ = {0.0f, 0.0f, 0.0f};    // [°/s]
};
