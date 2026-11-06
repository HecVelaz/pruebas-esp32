// 22-servo-muneca: servo de la muñeca (giro). Mismo programa de prueba que 21-servo-mg996r-pinza,
// con dos posiciones guardables (A y B) en lugar de abierto / cerrado.
// Comandos por el monitor serie (115200, una línea por comando). Ver README.md.
#include <Arduino.h>
#include "pins.h"
#include "ServoPWM.h"

// Medido el 2026-11-06 con este servo: cero = 1500 µs, 90° horario = 860 µs, 90° antihorario = 2150 µs.
// Grados negativos = horario (pulsos más cortos), como en la pinza.
constexpr uint16_t US_CERO = 1500;
constexpr uint16_t US_90_HORARIO = 860;
constexpr uint16_t US_90_ANTIHORARIO = 2150;
// µs por grado de cada lado (no es simétrico: 640 y 650 µs para 90°). Unos 7,2 µs/°: no es un servo de 180°
// con 500–2500 µs (eso serían 11,1 µs/°); probablemente de 270°.
constexpr float US_POR_GRADO_HORARIO = (US_CERO - US_90_HORARIO) / 90.0f;          // 7,11
constexpr float US_POR_GRADO_ANTIHORARIO = (US_90_ANTIHORARIO - US_CERO) / 90.0f;  // 7,22
constexpr float US_POR_GRADO = (US_POR_GRADO_HORARIO + US_POR_GRADO_ANTIHORARIO) / 2;  // para la velocidad w

// Franja al arrancar: exactamente ±90°. Agrandarla con "l <min> <max>" si hace falta más recorrido.
constexpr uint16_t US_MIN_INICIAL = US_90_HORARIO;
constexpr uint16_t US_MAX_INICIAL = US_90_ANTIHORARIO;
constexpr uint16_t US_MIN_ABSOLUTO = 500;   // típico de servos de hobby
constexpr uint16_t US_MAX_ABSOLUTO = 2500;
constexpr float W_INICIAL = 45;             // °/s
constexpr uint16_t PASO_US = 20;            // "+" / "-" (~2,8°)

ServoPWM servo;

float grados(uint16_t us) {
  int d = (int)us - (int)US_CERO;
  return d / (d < 0 ? US_POR_GRADO_HORARIO : US_POR_GRADO_ANTIHORARIO);
}
// Posiciones A y B: 90° horario y 90° antihorario (se pueden cambiar con "A" / "B" hasta reiniciar).
uint16_t usA = US_90_HORARIO;
uint16_t usB = US_90_ANTIHORARIO;
float w = W_INICIAL;  // velocidad angular de la rampa, °/s
bool avisarLlegada = false;

void ayuda() {
  Serial.println(F("Comandos:"));
  Serial.println(F("  p <us>        ir a un ancho de pulso (dentro de la franja)"));
  Serial.println(F("  0             ir al cero (1500 us)"));
  Serial.println(F("  + / -         mover 20 us (~2,8 grados)"));
  Serial.println(F("  a / b         ir a 90 grados horario (860 us) / antihorario (2150 us)"));
  Serial.println(F("  A / B         guardar la posición actual como A / B"));
  Serial.println(F("  w <grados/s>  velocidad angular de la rampa (0 = salto directo)"));
  Serial.println(F("  l <min> <max> franja permitida (500..2500)"));
  Serial.println(F("  s             soltar (sin pulsos, el servo no hace fuerza)"));
  Serial.println(F("  e             estado     ? ayuda"));
}

void estado() {
  Serial.printf("ESTADO %s actual=%u us (%+.1f grados) destino=%u us franja=[%u, %u] A=%u B=%u w=%.1f grados/s (%.0f us/s)\n",
                servo.suelto() ? "SUELTO" : "ACTIVO", servo.actualUs(), grados(servo.actualUs()), servo.destinoUs(),
                servo.minUs(), servo.maxUs(), usA, usB, w, w * US_POR_GRADO);
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
    case 'a': ir(usA); break;
    case 'b': ir(usB); break;
    case 'A':
    case 'B':
      if (servo.suelto()) { Serial.println(F("ERROR el servo está suelto: moverlo primero")); break; }
      (cmd == 'A' ? usA : usB) = servo.destinoUs();
      Serial.printf("GUARDADO %c = %u us (%+.1f grados)\n", cmd, servo.destinoUs(), grados(servo.destinoUs()));
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
  Serial.println(F("\n22-servo-muneca"));
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
