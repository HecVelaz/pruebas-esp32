// ¿En qué GPIO está el LED RGB de la S3? Alterna un segundo azul por el 48 y un segundo rojo por el 38.
//   parpadea AZUL -> el LED está en el 48 (como la DevKitC-1 v1.0): no choca con el EN de la tracción
//   parpadea ROJO -> el LED está en el 38 (v1.1): comparte pin con PIN_TRACCION_EN del robot
// El 38 es el EN de los IBT-2 de la tracción: correr con la S3 sola, sin drivers ni 12 V.
#include <Arduino.h>

constexpr uint8_t BRILLO = 40;  // de 255: suficiente para verlo sin encandilar

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println("GPIO 48 -> azul");
  neopixelWrite(48, 0, 0, BRILLO);
  delay(1000);
  neopixelWrite(48, 0, 0, 0);

  Serial.println("GPIO 38 -> rojo");
  neopixelWrite(38, BRILLO, 0, 0);
  delay(1000);
  neopixelWrite(38, 0, 0, 0);
}
