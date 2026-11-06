// 24 · Bumpers de la base: frente, izquierdo y derecho (ESP32-S3). No mueve motores: EN y PWM de la tracción en bajo.
// Lee los tres cada 1 ms. Un cambio vale cuando se mantiene LECTURAS_ESTABLES lecturas seguidas (igual que en
// 18-finales-de-carrera); los cambios crudos de más que hubo en el medio se cuentan como rebotes.
// Comandos (Enter): "s" estado de los tres · "r" poner a cero los contadores · "?" ayuda.
#include <Arduino.h>

#include "pins.h"

constexpr int N_FC = 3;  // "fc" = cada bumper (misma lógica que 18-finales-de-carrera)
constexpr const char *NOMBRE[N_FC] = {"frente", "izquierdo", "derecho"};
constexpr int PIN[N_FC] = {PIN_BUMPER_FRENTE, PIN_BUMPER_IZQ, PIN_BUMPER_DER};
constexpr int LECTURAS_ESTABLES = 3;  // a 1 ms: 3 ms seguidos con el mismo valor
constexpr uint32_t PERIODO_US = 1000;

struct FinalCarrera {
  int estable = HIGH;      // último valor confirmado
  int crudo = HIGH;        // última lectura
  int iguales = 0;         // lecturas seguidas distintas de "estable" e iguales entre sí
  uint32_t flancosCrudos = 0;  // cambios crudos desde el último cambio confirmado
  uint32_t apretadas = 0;
  uint32_t rebotesTotal = 0;
  uint32_t tCambioMs = 0;
};

FinalCarrera fc[N_FC];
char linea[32];
size_t largo = 0;

const char *texto(int v) { return v == LOW ? "APRETADO" : "suelto"; }

void mostrarEstado() {
  Serial.printf("Estado:");
  for (int i = 0; i < N_FC; i++) Serial.printf("  %s (GPIO%d) %s", NOMBRE[i], PIN[i], texto(fc[i].estable));
  Serial.println();
  for (int i = 0; i < N_FC; i++)
    Serial.printf("  %-9s apretado %lu veces, %lu rebotes en total\n", NOMBRE[i], (unsigned long)fc[i].apretadas,
                  (unsigned long)fc[i].rebotesTotal);
}

void ayuda() {
  Serial.println("Comandos: s = estado · r = contadores a cero · ? = ayuda. Apretar y soltar cada bumper.");
}

void leer(int i, uint32_t ahoraMs) {
  FinalCarrera &f = fc[i];
  const int v = digitalRead(PIN[i]);
  if (v != f.crudo) f.flancosCrudos++;
  f.crudo = v;
  if (v == f.estable) {
    f.iguales = 0;
    return;
  }
  if (++f.iguales < LECTURAS_ESTABLES) return;
  // Cambio confirmado: el primer flanco crudo es el cambio, el resto son rebotes
  const uint32_t rebotes = f.flancosCrudos > 1 ? f.flancosCrudos - 1 : 0;
  const uint32_t desdeAnterior = ahoraMs - f.tCambioMs;
  f.estable = v;
  f.iguales = 0;
  f.flancosCrudos = 0;
  f.rebotesTotal += rebotes;
  f.tCambioMs = ahoraMs;
  if (v == LOW) f.apretadas++;
  Serial.printf("%9.3f s  %-9s %-8s (rebotes %lu, %lu ms desde el cambio anterior)\n", ahoraMs / 1000.0f, NOMBRE[i],
                texto(v), (unsigned long)rebotes, (unsigned long)desdeAnterior);
  // Dos cambios a menos de 30 ms: más que un rebote, un contacto que falla o ruido
  if (desdeAnterior < 30) Serial.printf("           ^ %s cambió dos veces en %lu ms: ¿contacto flojo o ruido?\n", NOMBRE[i],
                                        (unsigned long)desdeAnterior);
}

void comando(const char *l) {
  if (strcmp(l, "s") == 0) {
    mostrarEstado();
  } else if (strcmp(l, "r") == 0) {
    for (auto &f : fc) f.apretadas = f.rebotesTotal = 0;
    Serial.println(">> Contadores a cero.");
  } else if (strcmp(l, "?") == 0 || l[0] == '\0') {
    ayuda();
  } else {
    Serial.printf("No entiendo \"%s\". ", l);
    ayuda();
  }
}

void setup() {
  // Motores quietos: EN en bajo (rueda libre) y PWM en bajo
  pinMode(PIN_TRACCION_EN, OUTPUT);
  digitalWrite(PIN_TRACCION_EN, LOW);
  for (int p : PINES_PWM) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  for (int i = 0; i < N_FC; i++) pinMode(PIN[i], INPUT_PULLUP);

  Serial.begin(115200);
  // USB nativo: esperar (hasta 3 s) a que se abra el monitor para no perder el encabezado
  for (uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) delay(10);
  delay(300);
  Serial.println("\n== 24 · Bumpers frente / izquierdo / derecho (ESP32-S3) ==");
  Serial.printf("Pines: frente GPIO%d · izq. GPIO%d · der. GPIO%d, pull-up interno (suelto = 1, apretado = 0)\n",
                PIN_BUMPER_FRENTE, PIN_BUMPER_IZQ, PIN_BUMPER_DER);
  Serial.printf("Tracción quieta: EN (GPIO%d) y PWM (4 5 6 7) en bajo.\n", PIN_TRACCION_EN);
  delay(5);
  for (int i = 0; i < N_FC; i++) fc[i].estable = fc[i].crudo = digitalRead(PIN[i]);
  mostrarEstado();
  for (int i = 0; i < N_FC; i++)
    if (fc[i].estable == LOW)
      Serial.printf("   OJO: %s ya está APRETADO al arrancar (¿apretado de verdad, NC en vez de NA o cable a GND?)\n",
                    NOMBRE[i]);
  ayuda();
}

void loop() {
  static uint32_t tick = micros();
  if (micros() - tick >= PERIODO_US) {
    tick += PERIODO_US;
    const uint32_t ahora = millis();
    for (int i = 0; i < N_FC; i++) leer(i, ahora);
  }
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linea[largo] = '\0';
      comando(linea);
      largo = 0;
    } else if (largo < sizeof(linea) - 1) {
      linea[largo++] = c;
    }
  }
}
