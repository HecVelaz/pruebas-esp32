#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  // Con USB CDC, esperar hasta 3 s a que se abra el monitor para no perder el primer mensaje
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
}

void loop() {
  Serial.println("Hola mundo desde ESP32-S3");
  delay(1000);
}
