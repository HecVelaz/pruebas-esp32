#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  // Con USB CDC, esperar hasta 3 s a que se abra el monitor para no perder el primer mensaje
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
}

void loop() {
  Serial.printf("Hola mundo desde %s\n", ESP.getChipModel());
  delay(1000);
}
