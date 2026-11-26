#include <Arduino.h>
#include <math.h>
#include "TFminiS.h"
#include "lidar.h"
#include "pins.h"

namespace {
HardwareSerial puertoLidar(1);  // UART0 queda para micro-ROS; no usa GPIO 16/17 de J3.
TFminiS sensor;
uint32_t ultimaLectura = 0;
}

void lidarBegin() {
  puertoLidar.setRxBufferSize(256);
  puertoLidar.begin(115200, SERIAL_8N1, PIN_LIDAR_RX, -1);  // solo RX, TX sin pin
  ultimaLectura = millis();
}

void lidarLoop() {
  const uint32_t ahora = millis();
  const int disponibles = puertoLidar.available();
  // Una pausa larga o un buffer lleno no deben renovar lecturas viejas.
  if (uint32_t(ahora - ultimaLectura) > TFminiS::VIGENCIA_MS || disponibles >= 240) {
    for (int i = 0; i < disponibles; ++i) puertoLidar.read();
    sensor.descartar();
  } else {
    for (int i = 0; i < disponibles; ++i) {
      const int b = puertoLidar.read();
      if (b >= 0) sensor.recibir(static_cast<uint8_t>(b), ahora);
    }
  }
  ultimaLectura = ahora;
}

float lidarMetros() {
  return sensor.valida(millis()) ? sensor.metros() : NAN;
}
