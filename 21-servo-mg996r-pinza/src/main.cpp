// 21-servo-mg996r-pinza: servo MG996R de la pinza (apertura / cierre del dedo).
// Comandos por el monitor serie (115200, una línea por comando). Ver README.md.
#include <Arduino.h>
#include "pins.h"
#include "ServoPWM.h"

// Franja permitida al arrancar: corta a propósito, para no forzar el dedo contra un tope.
// Agrandarla con "l <min> <max>" una vez medidos los extremos reales.
constexpr uint16_t US_MIN_INICIAL = 500;    // = US_ABIERTO: la pinza tiene que poder abrir del todo
constexpr uint16_t US_MAX_INICIAL = 2000;
constexpr uint16_t US_MIN_ABSOLUTO = 500;   // datos del MG996R: ~500–2500 µs
constexpr uint16_t US_MAX_ABSOLUTO = 2500;
// Conversión nominal del MG996R: 500–2500 µs ≈ 0–180° → 2000/180 µs por grado.
// Verificada el 2026-11-06: 1500 → 1000 µs = 45° horario. Grados negativos = horario.
constexpr float US_POR_GRADO = 2000.0f / 180.0f;
constexpr float W_INICIAL = 45;             // °/s
// Cero de este servo de la pinza (medido el 2026-11-06 con "p 1500"). Los ángulos se cuentan desde acá.
constexpr uint16_t US_CERO = 1500;
constexpr uint16_t PASO_US = 20;            // "+" / "-"

ServoPWM servo;

float grados(uint16_t us) { return ((int)us - (int)US_CERO) / US_POR_GRADO; }
// Medido el 2026-11-06. Abierto fijo: la pinza abre siempre a 500 µs (73 mm, ~90° horario desde el cero).
constexpr uint16_t US_ABIERTO = 500;
// Cerrado: 1250 µs aprieta bien un limón de 51 mm de diámetro (~22,5° horario). Se puede cambiar con "C".
uint16_t usCerrado = 1250;
float w = W_INICIAL;  // velocidad angular de la rampa, °/s
bool avisarLlegada = false;

void ayuda() {
  Serial.println(F("Comandos:"));
  Serial.println(F("  p <us>        ir a un ancho de pulso (dentro de la franja)"));
  Serial.println(F("  0             ir al cero (1500 us)"));
  Serial.println(F("  + / -         mover 20 us"));
  Serial.println(F("  a / c         abrir / cerrar (posiciones guardadas)"));
  Serial.println(F("  C             guardar la posición actual como cerrado (abierto es fijo: 500 us)"));
  Serial.println(F("  w <grados/s>  velocidad angular de la rampa (0 = salto directo)"));
  Serial.println(F("  l <min> <max> franja permitida (500..2500)"));
  Serial.println(F("  s             soltar (sin pulsos, el servo no hace fuerza)"));
  Serial.println(F("  e             estado     ? ayuda"));
}

void estado() {
  Serial.printf("ESTADO %s actual=%u us (%+.1f grados) destino=%u us franja=[%u, %u] abierto=%u cerrado=%u w=%.1f grados/s (%.0f us/s)\n",
                servo.suelto() ? "SUELTO" : "ACTIVO", servo.actualUs(), grados(servo.actualUs()), servo.destinoUs(),
                servo.minUs(), servo.maxUs(), US_ABIERTO, usCerrado, w, w * US_POR_GRADO);
}

void ir(int us) {
  if (us < servo.minUs() || us > servo.maxUs())
    Serial.printf("AVISO %d us fuera de la franja [%u, %u]: se recorta\n", us, servo.minUs(), servo.maxUs());
  if (us < 0) us = 0;
  bool estabaSuelto = servo.suelto();
  servo.irA((uint16_t)us);
  Serial.printf("VA %u us (%+.1f grados)%s\n", servo.destinoUs(), grados(servo.destinoUs()), estabaSuelto ? " (estaba suelto: salto directo)" : "");
  avisarLlegada = true;
}

void procesar(String linea) {
  linea.trim();
  if (linea.isEmpty()) return;
  char cmd = linea[0];
  String resto = linea.substring(1);
  resto.trim();

  switch (cmd) {
    case 'p': ir(resto.toInt()); break;
    case '+': ir((servo.suelto() ? US_CERO : servo.destinoUs()) + PASO_US); break;
    case '-': ir((servo.suelto() ? US_CERO : servo.destinoUs()) - PASO_US); break;
    case 'a': ir(US_ABIERTO); break;
    case 'c': ir(usCerrado); break;
    case 'C':
      if (servo.suelto()) { Serial.println(F("ERROR el servo está suelto: moverlo primero")); break; }
      usCerrado = servo.destinoUs();
      Serial.printf("GUARDADO cerrado = %u us\n", usCerrado);
      break;
    case 'w':
      if (resto.isEmpty()) { Serial.printf("W %.1f grados/s\n", w); break; }
      w = resto.toFloat();
      if (w < 0) w = 0;
      servo.velocidad(w * US_POR_GRADO);
      Serial.printf("W %.1f grados/s (%.0f us/s)%s\n", w, w * US_POR_GRADO, w == 0 ? " salto directo" : "");
      break;
    case 'l': {
      int esp = resto.indexOf(' ');
      int mn = resto.substring(0, esp).toInt();
      int mx = esp < 0 ? 0 : resto.substring(esp + 1).toInt();
      if (esp < 0 || mn < US_MIN_ABSOLUTO || mx > US_MAX_ABSOLUTO || !servo.limites(mn, mx)) {
        Serial.printf("ERROR uso: l <min> <max>, con %u <= min < max <= %u\n", US_MIN_ABSOLUTO, US_MAX_ABSOLUTO);
        break;
      }
      Serial.printf("FRANJA [%u, %u] us\n", servo.minUs(), servo.maxUs());
      break;
    }
    case 's': servo.soltar(); avisarLlegada = false; Serial.println(F("SUELTO")); break;
    case '0': ir(US_CERO); break;
    case 'e': estado(); break;
    case '?': ayuda(); break;
    default: Serial.printf("ERROR comando desconocido: %s (\"?\" para ayuda)\n", linea.c_str());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n21-servo-mg996r-pinza"));
  Serial.printf("Chip %s rev %d, %d MHz, señal del servo en GPIO %d\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(), PIN_SERVO);
  if (!servo.begin(PIN_SERVO, CANAL_SERVO, US_MIN_INICIAL, US_MAX_INICIAL)) {
    Serial.println(F("ERROR no se pudo configurar el LEDC"));
    while (true) delay(1000);
  }
  servo.velocidad(w * US_POR_GRADO);
  Serial.println(F("Arranca SUELTO (sin pulsos). El primer comando de posición salta directo."));
  ayuda();
  estado();
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      procesar(buf);
      buf = "";
    } else if (buf.length() < 40) {
      buf += ch;
    }
  }
  servo.update();
  if (avisarLlegada && servo.enDestino()) {
    avisarLlegada = false;
    Serial.printf("LLEGO %u us (%+.1f grados)\n", servo.actualUs(), grados(servo.actualUs()));
  }
}
