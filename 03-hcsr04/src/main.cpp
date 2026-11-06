// 03 · Los tres HC-SR04 de la base (frente, esquina izq., esquina der.) en la ESP32-S3, con el driver de la base
// (lib/HCSR04: ECHO por interrupción, un sensor por vez). No mueve motores: EN y PWM de la tracción en bajo.
// Dispara uno cada PERIODO_MS (cada sensor se mide cada 3 × PERIODO_MS) e imprime las tres distancias cada IMPRIMIR_MS.
// Comandos (Enter): "e [n]" estadística de n mediciones por sensor (20 por defecto) · "p" pausa / sigue · "?" ayuda.
#include <Arduino.h>
#include <math.h>

#include "HCSR04.h"
#include "pins.h"

constexpr int N_US = 3;
constexpr const char *NOMBRE[N_US] = {"frente", "izq.", "der."};
constexpr int TRIG[N_US] = {PIN_US_TRIG_FRENTE, PIN_US_TRIG_IZQ, PIN_US_TRIG_DER};
constexpr int ECHO[N_US] = {PIN_US_ECHO_FRENTE, PIN_US_ECHO_IZQ, PIN_US_ECHO_DER};
constexpr uint32_t PERIODO_MS = 60;    // >= 40 ms: el eco de "nada al alcance" dura hasta ~38 ms
constexpr uint32_t IMPRIMIR_MS = 500;

HCSR04 us;
int disparado = -1;  // sensor que se disparó por última vez (lo cierra el próximo disparar())
bool pausa = false;

// Estadística en curso
struct Estad {
  int validas = 0, sinEco = 0, sinRespuesta = 0;
  double suma = 0, suma2 = 0;
  float min = INFINITY, max = 0;
};
Estad est[N_US];
int estObjetivo = 0;  // 0 = no hay estadística en curso

char linea[32];
size_t largo = 0;

// Texto de una distancia: cm, "sin eco" (nada al alcance) o "NO RESPONDE"
void imprimirDist(float m) {
  if (isnan(m)) Serial.print("  NO RESPONDE");
  else if (isinf(m)) Serial.print(" sin eco (>4m)");
  else Serial.printf("  %7.1f cm  ", m * 100.0f);
}

void ayuda() {
  Serial.println("Comandos: e [n] = estadística de n mediciones por sensor (20) · p = pausa / sigue · ? = ayuda");
}

void nuevaMedicion(int i, float m) {
  if (estObjetivo == 0) return;
  Estad &e = est[i];
  if (e.validas + e.sinEco + e.sinRespuesta >= estObjetivo) return;
  if (isnan(m)) e.sinRespuesta++;
  else if (isinf(m)) e.sinEco++;
  else {
    const float cm = m * 100.0f;
    e.validas++;
    e.suma += cm;
    e.suma2 += (double)cm * cm;
    e.min = min(e.min, cm);
    e.max = max(e.max, cm);
  }
  for (const Estad &x : est)
    if (x.validas + x.sinEco + x.sinRespuesta < estObjetivo) return;
  // Terminó: resumen
  Serial.printf("== Estadística de %d mediciones por sensor ==\n", estObjetivo);
  Serial.println("  sensor   válidas  media cm  desvío cm  mín cm  máx cm  sin eco  no responde");
  for (int k = 0; k < N_US; k++) {
    const Estad &x = est[k];
    if (x.validas > 0) {
      const double media = x.suma / x.validas;
      const double var = x.suma2 / x.validas - media * media;
      Serial.printf("  %-7s  %4d    %7.1f   %7.2f   %6.1f  %6.1f   %4d     %4d\n", NOMBRE[k], x.validas, media,
                    sqrt(var > 0 ? var : 0), x.min, x.max, x.sinEco, x.sinRespuesta);
    } else {
      Serial.printf("  %-7s  %4d         -         -        -       -   %4d     %4d\n", NOMBRE[k], 0, x.sinEco,
                    x.sinRespuesta);
    }
  }
  estObjetivo = 0;
}

void comando(const char *l) {
  if (l[0] == 'e' && (l[1] == '\0' || l[1] == ' ')) {
    const int n = l[1] ? atoi(l + 2) : 20;
    if (n < 1 || n > 1000) {
      Serial.println("n entre 1 y 1000");
      return;
    }
    for (auto &e : est) e = Estad();
    estObjetivo = n;
    Serial.printf(">> Midiendo %d por sensor (~%.1f s). No mover el obstáculo.\n", n, n * N_US * PERIODO_MS / 1000.0f);
  } else if (strcmp(l, "p") == 0) {
    pausa = !pausa;
    Serial.println(pausa ? ">> Pausa (e sigue funcionando; p para seguir)." : ">> Sigue.");
  } else if (strcmp(l, "?") == 0 || l[0] == '\0') {
    ayuda();
  } else {
    Serial.printf("No entiendo \"%s\". ", l);
    ayuda();
  }
}

void setup() {
  // Tracción quieta: EN en bajo (rueda libre) y PWM en bajo
  pinMode(PIN_TRACCION_EN, OUTPUT);
  digitalWrite(PIN_TRACCION_EN, LOW);
  for (int p : PINES_PWM) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.begin(115200);
  // USB nativo: esperar (hasta 3 s) a que se abra el monitor para no perder el encabezado
  for (uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) delay(10);
  delay(300);
  Serial.println("\n== 03 · HC-SR04 frente / esq. izq. / esq. der. (ESP32-S3) ==");
  for (int i = 0; i < N_US; i++)
    Serial.printf("  %-6s TRIG GPIO%d · ECHO GPIO%d (divisor 1 k / 2 k)\n", NOMBRE[i], TRIG[i], ECHO[i]);
  Serial.printf("Tracción quieta: EN (GPIO%d) y PWM (4 5 6 7) en bajo. Un sensor cada %lu ms.\n", PIN_TRACCION_EN,
                (unsigned long)PERIODO_MS);
  // ECHO en reposo: con el sensor alimentado está en 0. En 1 = cable o divisor mal (o sensor colgado).
  us.begin(TRIG, ECHO, N_US);
  delay(5);
  for (int i = 0; i < N_US; i++)
    if (digitalRead(ECHO[i]) == HIGH)
      Serial.printf("   OJO: ECHO de %s (GPIO%d) en 1 sin disparar: ¿divisor, cable o sensor colgado?\n", NOMBRE[i],
                    ECHO[i]);

  // Dos vueltas de prueba para avisar si alguno no responde
  for (int k = 0; k < 2 * N_US + 1; k++) {
    us.disparar();
    delay(PERIODO_MS);
  }
  disparado = 0;  // las 2·3+1 llamadas dejan disparado el sensor 0
  for (int i = 0; i < N_US; i++)
    if (isnan(us.distancia(i)))
      Serial.printf("   OJO: %s no responde (sin pulso de ECHO): revisar 5 V, GND, TRIG GPIO%d, ECHO GPIO%d y el "
                    "divisor\n",
                    NOMBRE[i], TRIG[i], ECHO[i]);
  ayuda();
  Serial.println("      frente           izq.             der.");
}

void loop() {
  static uint32_t tDisparo = millis(), tImprimir = millis();
  const uint32_t ahora = millis();
  if (ahora - tDisparo >= PERIODO_MS) {
    tDisparo += PERIODO_MS;
    const int cerrado = disparado;  // disparar() cierra la medición de este y dispara el siguiente
    us.disparar();
    disparado = (disparado + 1) % N_US;
    nuevaMedicion(cerrado, us.distancia(cerrado));
  }
  if (!pausa && estObjetivo == 0 && ahora - tImprimir >= IMPRIMIR_MS) {
    tImprimir = ahora;
    for (int i = 0; i < N_US; i++) imprimirDist(us.distancia(i));
    Serial.println();
  }
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      linea[largo] = '\0';
      comando(linea);
      largo = 0;
    } else if (largo < sizeof(linea) - 1 && isprint((unsigned char)c)) {
      linea[largo++] = c;
    }
  }
}
