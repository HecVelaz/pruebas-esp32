#include "MPU9250.h"

namespace {
// Registros del MPU-9250/6500
constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG = 0x37;
constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REG_USER_CTRL = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;
constexpr uint8_t REG_WHO_AM_I = 0x75;

// Registros del AK8963
constexpr uint8_t AK_WIA = 0x00;
constexpr uint8_t AK_ST1 = 0x02;
constexpr uint8_t AK_HXL = 0x03;
constexpr uint8_t AK_CNTL1 = 0x0A;
constexpr uint8_t AK_ASAX = 0x10;
constexpr uint8_t AK_WIA_VALUE = 0x48;
constexpr uint8_t AK_MODE_POWER_DOWN = 0x00;
constexpr uint8_t AK_MODE_FUSE_ROM = 0x0F;
constexpr uint8_t AK_MODE_CONT2_16BIT = 0x16;  // continuo 100 Hz, salida de 16 bits

constexpr float ACCEL_LSB_PER_G = 16384.0f;  // ±2 g
constexpr float GYRO_LSB_PER_DPS = 131.0f;   // ±250 °/s
constexpr float MAG_UT_PER_LSB = 0.15f;      // 16 bits

int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }
int16_t le16(const uint8_t *p) { return (int16_t)((p[1] << 8) | p[0]); }
}  // namespace

MPU9250::MPU9250(TwoWire &wire, uint8_t addr) : wire_(wire), addr_(addr) {}

bool MPU9250::writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
  wire_.beginTransmission(dev);
  wire_.write(reg);
  wire_.write(val);
  return wire_.endTransmission() == 0;
}

bool MPU9250::readRegs(uint8_t dev, uint8_t reg, uint8_t *buf, size_t len) {
  wire_.beginTransmission(dev);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return false;
  if (wire_.requestFrom(dev, (uint8_t)len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = wire_.read();
  return true;
}

bool MPU9250::readWhoAmI(uint8_t &id) { return readRegs(addr_, REG_WHO_AM_I, &id, 1); }

const char *MPU9250::modelName() const {
  switch (model_) {
    case Model::MPU9250: return "MPU-9250";
    case Model::MPU6500: return "MPU-6500";
    default: return "desconocido";
  }
}

MPU9250::Status MPU9250::begin(Dlpf dlpf) {
  model_ = Model::Unknown;
  magOk_ = false;
  whoAmI_ = 0;

  if (!readWhoAmI(whoAmI_)) return Status::I2cError;
  if (whoAmI_ == WHO_AM_I_MPU9250) {
    model_ = Model::MPU9250;
  } else if (whoAmI_ == WHO_AM_I_MPU6500) {
    model_ = Model::MPU6500;
  } else {
    return Status::UnknownDevice;
  }

  if (!writeReg(addr_, REG_PWR_MGMT_1, 0x80)) return Status::ConfigError;  // reset
  delay(100);
  const bool ok = writeReg(addr_, REG_PWR_MGMT_1, 0x01) &&  // reloj: PLL del giroscopio
                  writeReg(addr_, REG_PWR_MGMT_2, 0x00) &&  // habilitar todos los ejes
                  writeReg(addr_, REG_SMPLRT_DIV, 1000 / SAMPLE_RATE_HZ - 1) &&  // 1 kHz / (1 + 4) = 200 Hz
                  writeReg(addr_, REG_GYRO_CONFIG, 0x00) &&                     // ±250 °/s
                  writeReg(addr_, REG_ACCEL_CONFIG, 0x00) &&                    // ±2 g
                  setDlpf(dlpf);
  if (!ok) return Status::ConfigError;
  delay(10);

  if (model_ == Model::MPU9250) magOk_ = initMag();
  return Status::Ok;
}

bool MPU9250::setDlpf(Dlpf dlpf) {
  const uint8_t cfg = (uint8_t)dlpf;
  // CONFIG: DLPF_CFG del giroscopio. ACCEL_CONFIG2: A_DLPFCFG con ACCEL_FCHOICE_B = 0 (DLPF activo)
  if (!writeReg(addr_, REG_CONFIG, cfg) || !writeReg(addr_, REG_ACCEL_CONFIG2, cfg)) return false;
  dlpf_ = dlpf;
  return true;
}

bool MPU9250::initMag() {
  // Modo bypass: el AK8963 queda visible directamente en el bus I2C del ESP32
  if (!writeReg(addr_, REG_USER_CTRL, 0x00)) return false;  // I2C master interno deshabilitado
  if (!writeReg(addr_, REG_INT_PIN_CFG, 0x02)) return false;
  delay(10);

  uint8_t wia = 0;
  if (!readRegs(AK8963_ADDR, AK_WIA, &wia, 1) || wia != AK_WIA_VALUE) return false;

  // Ajustes de sensibilidad de fábrica: se leen en modo Fuse ROM, pasando siempre por power-down
  if (!writeReg(AK8963_ADDR, AK_CNTL1, AK_MODE_POWER_DOWN)) return false;
  delay(10);
  if (!writeReg(AK8963_ADDR, AK_CNTL1, AK_MODE_FUSE_ROM)) return false;
  delay(10);
  uint8_t asa[3];
  if (!readRegs(AK8963_ADDR, AK_ASAX, asa, 3)) return false;
  if (!writeReg(AK8963_ADDR, AK_CNTL1, AK_MODE_POWER_DOWN)) return false;
  delay(10);
  if (!writeReg(AK8963_ADDR, AK_CNTL1, AK_MODE_CONT2_16BIT)) return false;
  delay(10);

  for (int i = 0; i < 3; i++) magAdj_[i] = ((asa[i] - 128) * 0.5f) / 128.0f + 1.0f;
  return true;
}

bool MPU9250::readAccelGyro(Vec3 &accel, Vec3 &gyro, float &tempC) {
  uint8_t b[14];
  if (!readRegs(addr_, REG_ACCEL_XOUT_H, b, sizeof(b))) return false;
  accel = {be16(&b[0]) / ACCEL_LSB_PER_G, be16(&b[2]) / ACCEL_LSB_PER_G, be16(&b[4]) / ACCEL_LSB_PER_G};
  tempC = be16(&b[6]) / 333.87f + 21.0f;
  gyro = {be16(&b[8]) / GYRO_LSB_PER_DPS - gyroBias_.x,
          be16(&b[10]) / GYRO_LSB_PER_DPS - gyroBias_.y,
          be16(&b[12]) / GYRO_LSB_PER_DPS - gyroBias_.z};
  return true;
}

MPU9250::CalStatus MPU9250::calibrateGyro(const GyroCalConfig &cfg) {
  if (cfg.samples == 0) return CalStatus::Moving;
  double sum[3] = {0, 0, 0};
  float gMin[3], gMax[3], aMin[3], aMax[3];

  for (uint16_t i = 0; i < cfg.samples; i++) {
    Vec3 a, g;
    float t;
    if (!readAccelGyro(a, g, t)) return CalStatus::I2cError;

    // Volver al valor crudo, sin el bias actual
    const float gr[3] = {g.x + gyroBias_.x, g.y + gyroBias_.y, g.z + gyroBias_.z};
    const float ar[3] = {a.x, a.y, a.z};

    const float norm = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
    if (fabsf(norm - 1.0f) > cfg.accelNormTolG) return CalStatus::NotLevel;

    for (int k = 0; k < 3; k++) {
      if (i == 0) {
        gMin[k] = gMax[k] = gr[k];
        aMin[k] = aMax[k] = ar[k];
      }
      gMin[k] = min(gMin[k], gr[k]);
      gMax[k] = max(gMax[k], gr[k]);
      aMin[k] = min(aMin[k], ar[k]);
      aMax[k] = max(aMax[k], ar[k]);
      if (gMax[k] - gMin[k] > cfg.maxRangeDps || aMax[k] - aMin[k] > cfg.maxAccelRangeG) return CalStatus::Moving;
      if (fabsf(gr[k]) > cfg.maxBiasDps) return CalStatus::BiasTooLarge;
      sum[k] += gr[k];
    }
    delay(1000 / SAMPLE_RATE_HZ);
  }

  gyroBias_ = {(float)(sum[0] / cfg.samples), (float)(sum[1] / cfg.samples), (float)(sum[2] / cfg.samples)};
  return CalStatus::Ok;
}

MPU9250::MagStatus MPU9250::readMag(Vec3 &mag) {
  if (!magOk_) return MagStatus::NoMag;

  // Secuencia del datasheet del AK8963: leer ST1; solo si DRDY = 1, leer HXL..HZH y ST2.
  // Leer ST2 marca el fin de la lectura y libera el siguiente dato.
  uint8_t st1 = 0;
  if (!readRegs(AK8963_ADDR, AK_ST1, &st1, 1)) return MagStatus::I2cError;
  if (!(st1 & 0x01)) return MagStatus::NotReady;

  uint8_t b[7];  // HXL, HXH, HYL, HYH, HZL, HZH, ST2
  if (!readRegs(AK8963_ADDR, AK_HXL, b, sizeof(b))) return MagStatus::I2cError;
  if (b[6] & 0x08) return MagStatus::Overflow;  // HOFL

  mag = {le16(&b[0]) * magAdj_[0] * MAG_UT_PER_LSB,
         le16(&b[2]) * magAdj_[1] * MAG_UT_PER_LSB,
         le16(&b[4]) * magAdj_[2] * MAG_UT_PER_LSB};
  return MagStatus::Ok;
}

const char *MPU9250::statusName(Status s) {
  switch (s) {
    case Status::Ok: return "OK";
    case Status::I2cError: return "no responde en el bus I2C";
    case Status::UnknownDevice: return "WHO_AM_I no es MPU-9250 (0x71) ni MPU-6500 (0x70)";
    case Status::ConfigError: return "falló una escritura I2C durante la configuración";
  }
  return "?";
}

const char *MPU9250::magStatusName(MagStatus s) {
  switch (s) {
    case MagStatus::Ok: return "OK";
    case MagStatus::NotReady: return "sin dato nuevo";
    case MagStatus::Overflow: return "overflow magnético";
    case MagStatus::I2cError: return "error I2C";
    case MagStatus::NoMag: return "sin magnetómetro";
  }
  return "?";
}

const char *MPU9250::calStatusName(CalStatus s) {
  switch (s) {
    case CalStatus::Ok: return "OK";
    case CalStatus::I2cError: return "error I2C";
    case CalStatus::Moving: return "el sensor se movió";
    case CalStatus::NotLevel: return "aceleración distinta de 1 g";
    case CalStatus::BiasTooLarge: return "bias fuera de rango (¿giro constante?)";
  }
  return "?";
}
