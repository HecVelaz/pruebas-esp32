#include <Arduino.h>
#include <Wire.h>

#include "MPU9250.h"
#include "pins.h"

MPU9250 imu(Wire);
bool imuOk = false;

// Recorre el bus I2C, imprime cada dispositivo encontrado y devuelve la dirección del MPU (0 si no está)
uint8_t scanI2C() {
  Serial.println("Escaneando bus I2C...");
  uint8_t mpuAddr = 0;
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  dispositivo en 0x%02X\n", addr);
      found++;
      if (!mpuAddr && (addr == MPU9250::ADDR_AD0_LOW || addr == MPU9250::ADDR_AD0_HIGH)) mpuAddr = addr;
    }
  }
  Serial.printf("%d dispositivo(s) encontrado(s)\n", found);
  return mpuAddr;
}

void setup() {
  Serial.begin(115200);
  // Con USB CDC, esperar hasta 3 s a que se abra el monitor para no perder los primeros mensajes
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  Serial.println();
  Serial.println("== Prueba IMU MPU-9250 / MPU-6500 ==");
  Serial.printf("I2C: SDA=GPIO%d, SCL=GPIO%d\n", PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);

  uint8_t addr = scanI2C();
  if (!addr) {
    Serial.println("ERROR: no se encontró el MPU en 0x68 ni 0x69. Revisar cableado y alimentación.");
    return;
  }
  if (addr != IMU_I2C_ADDR) {
    Serial.printf("AVISO: se esperaba el MPU en 0x%02X (AD0 a GND) y está en 0x%02X. Revisar AD0.\n",
                  IMU_I2C_ADDR, addr);
  }

  imu.setAddress(addr);
  MPU9250::Status st = imu.begin();
  if (st == MPU9250::Status::I2cError) {
    Serial.printf("ERROR: no se pudo leer WHO_AM_I en 0x%02X (%s).\n", addr, MPU9250::statusName(st));
    return;
  }
  Serial.printf("WHO_AM_I en 0x%02X = 0x%02X\n", addr, imu.whoAmI());
  if (st != MPU9250::Status::Ok) {
    Serial.printf("ERROR: %s.\n", MPU9250::statusName(st));
    return;
  }
  imuOk = true;
  Serial.printf("Modelo detectado: %s\n", imu.modelName());

  if (imu.model() == MPU9250::Model::MPU9250) {
    Serial.printf("Magnetómetro AK8963: %s\n", imu.hasMag() ? "OK" : "no responde");
  } else {
    Serial.println("MPU-6500: sin magnetómetro, solo acelerómetro y giroscopio.");
  }
  Serial.println();
}

void loop() {
  if (!imuOk) {
    delay(1000);
    return;
  }

  MPU9250::Vec3 a, g;
  float t;
  if (!imu.readAccelGyro(a, g, t)) {
    Serial.println("ERROR: fallo de lectura I2C");
    delay(500);
    return;
  }

  Serial.printf("acc[g] %6.2f %6.2f %6.2f | gyr[dps] %7.1f %7.1f %7.1f | T %5.1f C",
                a.x, a.y, a.z, g.x, g.y, g.z, t);

  if (imu.hasMag()) {
    // Solo se imprime el dato si es nuevo; si no, el motivo (nunca se repite una lectura vieja)
    MPU9250::Vec3 m;
    MPU9250::MagStatus ms = imu.readMag(m);
    if (ms == MPU9250::MagStatus::Ok) {
      Serial.printf(" | mag[uT] %6.1f %6.1f %6.1f", m.x, m.y, m.z);
    } else {
      Serial.printf(" | mag: %s", MPU9250::magStatusName(ms));
    }
  }
  Serial.println();
  delay(200);
}
