#include <Arduino.h>

#include "UP501.h"
#include "pins.h"

constexpr uint32_t PRINT_PERIOD_MS = 1000;

UP501 gps(Serial1);
bool rawEcho = false;  // 'r' en el monitor alterna entre el resumen y las tramas NMEA crudas
UP501::State lastState = UP501::State::NoData;
bool firstFixReported = false;
uint32_t lastPrintMs = 0;

void printSummary() {
  const uint32_t uptimeS = millis() / 1000;
  const UP501::State st = gps.state();
  Serial.printf("[%5lu s] ", (unsigned long)uptimeS);

  switch (st) {
    case UP501::State::NoData:
      Serial.printf("ERROR: sin datos del GPS. Revisar TXD del GPS -> GPIO%d, GND común y 3V3 en VDD/VDD_B.\n",
                    PIN_GPS_RX);
      break;

    case UP501::State::BadData:
      Serial.printf("ERROR: llegan bytes pero ninguna trama NMEA válida (checksum malo: %lu). ¿Baudrate distinto de %lu?\n",
                    (unsigned long)gps.failedChecksum(), (unsigned long)GPS_BAUD);
      break;

    case UP501::State::NoFix: {
      const int inView = gps.satellitesInView();
      Serial.printf("SIN FIX | satélites en vista: ");
      if (inView < 0) Serial.print("?");
      else Serial.print(inView);
      Serial.printf(" | NMEA ok %lu, checksum malo %lu | buscando posición...\n",
                    (unsigned long)gps.passedChecksum(), (unsigned long)gps.failedChecksum());
      break;
    }

    case UP501::State::Fix: {
      UP501::Fix f;
      gps.readFix(f);
      Serial.printf("FIX %s | sats usados %u, en vista %d | lat %.6f lon %.6f | alt %.1f m (MSL)",
                    f.mode == 3 ? "3D" : (f.mode == 2 ? "2D" : "?"), f.satsUsed, gps.satellitesInView(),
                    f.latDeg, f.lonDeg, f.altMslM);
      if (!isnan(f.geoidSepM)) Serial.printf(", geoide %.1f m", f.geoidSepM);
      Serial.printf(" | HDOP %.2f", f.hdop);
      if (f.quality == 2) Serial.print(" | DGPS/SBAS");
      TinyGPSPlus &p = gps.parser();
      if (p.date.isValid() && p.time.isValid()) {
        Serial.printf(" | %04u-%02u-%02u %02u:%02u:%02u UTC", p.date.year(), p.date.month(), p.date.day(),
                      p.time.hour(), p.time.minute(), p.time.second());
      }
      Serial.println();
      break;
    }
  }

  // Tiempo hasta el primer fix (TTFF), para anotar en Resultados
  if (st == UP501::State::Fix && !firstFixReported) {
    firstFixReported = true;
    Serial.printf("Primer FIX a los %lu s del arranque.\n", (unsigned long)uptimeS);
  } else if (st != UP501::State::Fix && lastState == UP501::State::Fix) {
    Serial.println("AVISO: se perdió el fix.");
  }
  lastState = st;
}

void setup() {
  Serial.begin(115200);
  // Con USB CDC, esperar hasta 3 s a que se abra el monitor para no perder los primeros mensajes
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  Serial.println();
  Serial.println("== Prueba GPS Fastrax UP501 ==");
  Serial.printf("UART1: RX=GPIO%d (<- TXD del GPS), TX=GPIO%d (-> RXD del GPS), %lu baud 8N1\n", PIN_GPS_RX,
                PIN_GPS_TX, (unsigned long)GPS_BAUD);
  Serial.println("Escribir 'r' + Enter en el monitor para alternar entre el resumen y las tramas NMEA crudas.");
  Serial.println();

  // A 9600 baud llegan ~1 kB/s: un búfer de 1 kB cubre cualquier pausa del loop
  Serial1.setRxBufferSize(1024);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == 'r' || c == 'R') {
      rawEcho = !rawEcho;
      Serial.println(rawEcho ? "\n-- Tramas NMEA crudas ('r' para volver al resumen) --"
                             : "\n-- Resumen cada 1 s ('r' para ver las tramas crudas) --");
    }
  }

  gps.update(rawEcho ? &Serial : nullptr);

  if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = millis();
    if (!rawEcho) printSummary();
  }
}
