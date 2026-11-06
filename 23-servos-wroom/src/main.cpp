// 23-servos-wroom: pinza (GPIO 4) y muñeca (GPIO 14) en la WROOM-32D del brazo, con los pines del robot.
// Cada servo tiene los mismos comandos y valores medidos que su prueba suelta:
//   pinza  = 21-servo-mg996r-pinza (cero 1500 us, abierto fijo 500 us, cerrado 1250 us)
//   muñeca = 22-servo-muneca       (±90° en 860 / 2150 us)
// Comandos por el monitor serie (115200): "p <cmd>" para la pinza, "m <cmd>" para la muñeca. Ver README.md.
#include <Arduino.h>
#include "pins.h"
#include "ServoPWM.h"

constexpr uint16_t US_MIN_ABSOLUTO = 500;   // típico de servos de hobby
constexpr uint16_t US_MAX_ABSOLUTO = 2500;
constexpr float W_INICIAL = 45;             // °/s
constexpr uint16_t PASO_US = 20;            // "+" / "-"

// Una posición con nombre: la letra la manda (minúscula) y, si es guardable, la mayúscula la guarda.
struct Posicion {
  char letra;
  uint16_t us;
  bool guardable;
  const char *nombre;
};

struct Junta {
  const char *nombre;
  char prefijo;
  int pin, canal;
  uint16_t cero, minInicial, maxInicial;
  float usPorGradoHorario, usPorGradoAntihorario;  // grados negativos = horario (pulsos más cortos)
  Posicion pos[2];
  ServoPWM servo;
  float w;              // velocidad angular de la rampa, °/s (W_INICIAL en setup)
  bool avisarLlegada;

  float usPorGrado() const { return (usPorGradoHorario + usPorGradoAntihorario) / 2; }  // para la velocidad w
  float grados(uint16_t us) const {
    int d = (int)us - (int)cero;
    return d / (d < 0 ? usPorGradoHorario : usPorGradoAntihorario);
  }
};

Junta juntas[2] = {
    // 21: MG996R, 500–2500 us ≈ 0–180° (11,1 us/°). Mismos valores que 21: cero 1500 us, abierto fijo 500 us
    // (73 mm, −90°), cerrado 1250 us (limón de 51 mm). Franja al arrancar [500, 2000] para no forzar el dedo.
    {"pinza", 'p', PIN_SERVO_PINZA, CANAL_PINZA, 1500, 500, 2000, 2000.0f / 180, 2000.0f / 180,
     {{'a', 500, false, "abierto"}, {'c', 1250, true, "cerrado"}}},
    // 22: cero 1500 us, 90° horario = 860 us, 90° antihorario = 2150 us (unos 7,2 us/°). Franja = ±90°.
    {"muneca", 'm', PIN_SERVO_GIRO, CANAL_GIRO, 1500, 860, 2150, (1500 - 860) / 90.0f, (2150 - 1500) / 90.0f,
     {{'a', 860, true, "90 horario"}, {'b', 2150, true, "90 antihorario"}}},
};

void ayuda() {
  Serial.println(F("Comandos: \"p <cmd>\" = pinza (GPIO 4), \"m <cmd>\" = muneca (GPIO 14). <cmd>:"));
  Serial.println(F("  p <us>        ir a un ancho de pulso (dentro de la franja)"));
  Serial.println(F("  0             ir al cero (1500 us)"));
  Serial.println(F("  + / -         mover 20 us"));
  Serial.println(F("  pinza:  a / c  abrir (500 us, fijo) / cerrar (1250 us);  C  guardar la actual como cerrado"));
  Serial.println(F("  muneca: a / b  90 grados horario (860 us) / antihorario (2150 us);  A / B  guardar la actual"));
  Serial.println(F("  w <grados/s>  velocidad angular de la rampa (0 = salto directo)"));
  Serial.println(F("  l <min> <max> franja permitida (500..2500)"));
  Serial.println(F("  s             soltar (sin pulsos, el servo no hace fuerza)"));
  Serial.println(F("  e             estado"));
  Serial.println(F("Sin prefijo:  s = soltar los dos,  e = estado de los dos,  ? = ayuda"));
  Serial.println(F("Ejemplos: \"p a\" abre la pinza, \"m b\" gira la muneca 90 grados antihorario, \"p p 1300\"."));
}

void estado(Junta &j) {
  Serial.printf("ESTADO %s %s actual=%u us (%+.1f grados) destino=%u us franja=[%u, %u]", j.nombre,
                j.servo.suelto() ? "SUELTO" : "ACTIVO", j.servo.actualUs(), j.grados(j.servo.actualUs()),
                j.servo.destinoUs(), j.servo.minUs(), j.servo.maxUs());
  for (auto &p : j.pos) Serial.printf(" %c(%s)=%u", p.letra, p.nombre, p.us);
  Serial.printf(" w=%.1f grados/s (%.0f us/s)\n", j.w, j.w * j.usPorGrado());
}

void ir(Junta &j, int us) {
  if (us < j.servo.minUs() || us > j.servo.maxUs())
    Serial.printf("AVISO %s: %d us fuera de la franja [%u, %u]: se recorta\n", j.nombre, us, j.servo.minUs(), j.servo.maxUs());
  if (us < 0) us = 0;
  bool estabaSuelto = j.servo.suelto();
  j.servo.irA((uint16_t)us);
  Serial.printf("VA %s %u us (%+.1f grados)%s\n", j.nombre, j.servo.destinoUs(), j.grados(j.servo.destinoUs()),
                estabaSuelto ? " (estaba suelto: salto directo)" : "");
  j.avisarLlegada = true;
}

void soltar(Junta &j) {
  j.servo.soltar();
  j.avisarLlegada = false;
  Serial.printf("SUELTO %s\n", j.nombre);
}

void procesarJunta(Junta &j, String linea) {
  if (linea.isEmpty()) { Serial.printf("ERROR falta el comando para %s (\"?\" para ayuda)\n", j.nombre); return; }
  char cmd = linea[0];
  String resto = linea.substring(1);
  resto.trim();
  uint16_t base = j.servo.suelto() ? j.cero : j.servo.destinoUs();

  for (auto &p : j.pos) {
    if (cmd == p.letra) { ir(j, p.us); return; }
    if (cmd == toupper(p.letra)) {
      if (!p.guardable) break;
      if (j.servo.suelto()) { Serial.printf("ERROR %s está suelto: moverlo primero\n", j.nombre); return; }
      p.us = j.servo.destinoUs();
      Serial.printf("GUARDADO %s %s = %u us (%+.1f grados)\n", j.nombre, p.nombre, p.us, j.grados(p.us));
      return;
    }
  }

  switch (cmd) {
    case 'p': ir(j, resto.toInt()); break;
    case '+': ir(j, base + PASO_US); break;
    case '-': ir(j, base - PASO_US); break;
    case '0': ir(j, j.cero); break;
    case 'w':
      if (resto.isEmpty()) { Serial.printf("W %s %.1f grados/s\n", j.nombre, j.w); break; }
      j.w = resto.toFloat();
      if (j.w < 0) j.w = 0;
      j.servo.velocidad(j.w * j.usPorGrado());
      Serial.printf("W %s %.1f grados/s (%.0f us/s)%s\n", j.nombre, j.w, j.w * j.usPorGrado(), j.w == 0 ? " salto directo" : "");
      break;
    case 'l': {
      int esp = resto.indexOf(' ');
      int mn = resto.substring(0, esp).toInt();
      int mx = esp < 0 ? 0 : resto.substring(esp + 1).toInt();
      if (esp < 0 || mn < US_MIN_ABSOLUTO || mx > US_MAX_ABSOLUTO || !j.servo.limites(mn, mx)) {
        Serial.printf("ERROR uso: %c l <min> <max>, con %u <= min < max <= %u\n", j.prefijo, US_MIN_ABSOLUTO, US_MAX_ABSOLUTO);
        break;
      }
      Serial.printf("FRANJA %s [%u, %u] us\n", j.nombre, j.servo.minUs(), j.servo.maxUs());
      break;
    }
    case 's': soltar(j); break;
    case 'e': estado(j); break;
    default: Serial.printf("ERROR comando desconocido para %s: %s (\"?\" para ayuda)\n", j.nombre, linea.c_str());
  }
}

void procesar(String linea) {
  linea.trim();
  if (linea.isEmpty()) return;
  if (linea == "s") { for (auto &j : juntas) soltar(j); return; }
  if (linea == "e") { for (auto &j : juntas) estado(j); return; }
  if (linea == "?") { ayuda(); return; }
  for (auto &j : juntas) {
    if (linea[0] == j.prefijo && (linea.length() == 1 || linea[1] == ' ')) {
      String resto = linea.substring(1);
      resto.trim();
      procesarJunta(j, resto);
      return;
    }
  }
  Serial.printf("ERROR comando desconocido: %s (empezar con \"p \" o \"m \"; \"?\" para ayuda) bytes:", linea.c_str());
  for (unsigned i = 0; i < linea.length(); i++) Serial.printf(" %02X", (uint8_t)linea[i]);
  Serial.println();
}

void setup() {
  // Primero los motores del brazo: EN y PWM en LOW.
  pinMode(PIN_BRAZO_EN, OUTPUT);
  digitalWrite(PIN_BRAZO_EN, LOW);
  for (int p : PINES_PWM_BRAZO) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n23-servos-wroom: pinza (21) + muneca (22)"));
  Serial.printf("Chip %s rev %d. Motores del brazo deshabilitados (EN %d en LOW).\n", ESP.getChipModel(),
                ESP.getChipRevision(), PIN_BRAZO_EN);
  for (auto &j : juntas) {
    j.w = W_INICIAL;
    j.avisarLlegada = false;
    if (!j.servo.begin(j.pin, j.canal, j.minInicial, j.maxInicial)) {
      Serial.printf("ERROR no se pudo configurar el LEDC del GPIO %d\n", j.pin);
      while (true) delay(1000);
    }
    j.servo.velocidad(j.w * j.usPorGrado());
    Serial.printf("%s en GPIO %d\n", j.nombre, j.pin);
  }
  Serial.println(F("Los dos arrancan SUELTOS (sin pulsos). El primer comando de posición de cada uno salta directo."));
  ayuda();
  for (auto &j : juntas) estado(j);
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { procesar(buf); buf = ""; }
    else if (ch == '\t') { if (buf.length() < 40) buf += ' '; }
    else if (ch >= 0x20 && ch <= 0x7E && buf.length() < 40) buf += ch;  // descarta bytes basura (p. ej. al arrancar)
  }
  for (auto &j : juntas) {
    j.servo.update();
    if (j.avisarLlegada && j.servo.enDestino()) {
      j.avisarLlegada = false;
      Serial.printf("LLEGO %s %u us (%+.1f grados)\n", j.nombre, j.servo.actualUs(), j.grados(j.servo.actualUs()));
    }
  }
}
