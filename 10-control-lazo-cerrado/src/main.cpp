// Paso 0 del lazo cerrado: sentido y signo de J1 (36GP-555 + IBT-2) en la ESP32-WROOM-32D.
// Todavía no hay control: solo pulsos cortos de duty fijo para comprobar que "positivo" es lo mismo
// para el motor, el encoder y la articulación (q1 positivo = antihorario visto desde arriba).
// Si no coinciden, el lazo cerrado empujaría para el lado equivocado y se dispararía.
#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"
#include "config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_R = 0;
constexpr uint8_t LEDC_CH_L = 1;
constexpr uint32_t PULSO_MS_DEF = 200;
constexpr uint32_t FRENO_MS = 300;        // freno después del pulso, antes de dejarlo en rueda libre
constexpr uint32_t MONITOR_CADA_MS = 200;
constexpr uint32_t QUIETO_MS = 100;       // antes de un pulso, la base tiene que estar quieta este tiempo

BTS7960 motorJ1;
EncoderPCNT encJ1;
bool hwOk = false;
bool ceroFijado = false;  // sin "z" no hay pulsos: el cero tiene que estar en la marca, lejos de los topes

// Signos vigentes: arrancan con los de config.h y se pueden cambiar con "ie" e "is" para probar
bool encInvertido = J1_ENCODER_INVERTIDO;
bool sentidoInvertido = J1_SENTIDO_INVERTIDO;

char linea[48];
size_t lineaLen = 0;
bool lineaLarga = false;   // se pasó del buffer: se descarta entera al llegar el Enter
bool ultimoFueCR = false;  // CR+LF cuenta como un solo Enter
bool descartando = false;  // una línea cortó un pulso o el monitor: el resto de esa línea no se ejecuta

// Cuentas de J1 con los signos aplicados
int64_t cuentas() {
  int64_t c = encJ1.count();
  if (encInvertido) c = -c;
  if (sentidoInvertido) c = -c;
  return c;
}

void aplicarDuty(float pct) {
  float d = pct / 100.0f;
  if (sentidoInvertido) d = -d;
  motorJ1.setDuty(d);
}

// Hay algo escrito en el monitor: corta el pulso o el monitor del encoder.
// El LF que sigue al CR del comando anterior no cuenta. Si la línea que corta todavía no terminó,
// el resto se descarta hasta su Enter (así su cola no se ejecuta como otro comando).
bool hayEntrada() {
  bool hay = false;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' && ultimoFueCR) {
      ultimoFueCR = false;
      continue;
    }
    ultimoFueCR = (ch == '\r');
    descartando = !(ch == '\r' || ch == '\n');
    hay = true;
  }
  return hay;
}

void ayuda() {
  Serial.println();
  Serial.println("Comandos (escribir y Enter):");
  Serial.println("  e            ver el encoder (cuentas y niveles A/B) girando a mano; Enter para salir");
  Serial.println("  z            poner las cuentas a cero con la base en su marca (al frente). Habilita los pulsos");
  Serial.printf("  p <%%> [ms]   pulso de duty fijo, |%%| <= %.0f, por defecto %lu ms (máx %lu). Ej: p 25, p -25 300\n",
                J1_DUTY_MAX_PCT, (unsigned long)PULSO_MS_DEF, (unsigned long)J1_PULSO_MAX_MS);
  Serial.println("  c            mostrar las cuentas");
  Serial.println("  ie           invertir el signo del encoder (si con duty + las cuentas bajan)");
  Serial.println("  is           invertir el sentido (motor y encoder juntos) si la base gira al revés de +q1");
  Serial.println("  x            rueda libre (driver deshabilitado)");
  Serial.println("  ?            esta ayuda");
  Serial.printf("Seguridad: corte a más de %ld cuentas del cero (%.0f° de la salida de la reductora) y si no\n",
                (long)J1_LIMITE_CUENTAS, 360.0f * J1_LIMITE_CUENTAS / J1_CUENTAS_POR_VUELTA);
  Serial.printf("llegan cuentas del encoder durante %lu ms. Cualquier Enter durante un pulso lo corta.\n",
                (unsigned long)J1_SIN_CUENTAS_MS);
  Serial.println();
}

void mostrarSignos() {
  Serial.printf("Signos: encoder %s, sentido %s\n", encInvertido ? "INVERTIDO" : "normal",
                sentidoInvertido ? "INVERTIDO" : "normal");
}

void monitorEncoder() {
  motorJ1.coast();
  Serial.println(">> Girá la base a mano (con los 12 V apagados). Enter para salir.");
  int64_t ultima = INT64_MIN;
  uint32_t t = 0;
  while (!hayEntrada()) {
    if (millis() - t < MONITOR_CADA_MS) continue;
    t = millis();
    const int64_t c = cuentas();
    if (c == ultima) continue;  // solo cuando cambia, para no llenar la pantalla
    ultima = c;
    Serial.printf("   cuentas %+6lld | %+7.2f° de salida | A=%d B=%d\n", (long long)c,
                  360.0f * c / J1_CUENTAS_POR_VUELTA, encJ1.levelA(), encJ1.levelB());
  }
  Serial.println(">> Fin del monitor.");
}

// Espera QUIETO_MS sin que la base se mueva (un pulso anterior o la mano pueden dejarla girando).
// Un Enter durante la espera cancela el pulso antes de energizar el motor.
enum class Espera { Quieta, Moviendose, Cancelada };
Espera esperarQuieta() {
  const int64_t c = cuentas();
  const uint32_t t0 = millis();
  while (millis() - t0 < QUIETO_MS) {
    if (hayEntrada()) return Espera::Cancelada;
    if (llabs(cuentas() - c) > 1) return Espera::Moviendose;
    delay(1);
  }
  return Espera::Quieta;
}

void pulso(float pct, uint32_t ms) {
  switch (esperarQuieta()) {
    case Espera::Moviendose:
      Serial.println("La base todavía se mueve: esperar a que se detenga y repetir.");
      return;
    case Espera::Cancelada:
      Serial.println(">> Pulso cancelado.");
      return;
    case Espera::Quieta:
      break;
  }
  const int64_t c0 = cuentas();
  Serial.printf(">> Pulso %+.0f %% durante %lu ms desde %+lld cuentas\n", pct, (unsigned long)ms, (long long)c0);

  enum class Corte { Ninguno, Limite, SinCuentas, Enter };
  Corte corte = Corte::Ninguno;
  // Fuera del límite solo se permite volver: en cuanto entra al rango, el límite vuelve a ser ±LIMITE
  bool dentro = llabs(c0) <= J1_LIMITE_CUENTAS;
  int64_t cAnt = c0;
  const uint32_t t0 = millis();
  uint32_t tUltimaCuenta = t0;
  aplicarDuty(pct);
  while (millis() - t0 < ms) {
    const int64_t c = cuentas();
    const uint32_t ahora = millis();
    if (c != cAnt) {
      cAnt = c;
      tUltimaCuenta = ahora;
    }
    if (llabs(c) <= J1_LIMITE_CUENTAS) dentro = true;
    if (llabs(c) > J1_LIMITE_CUENTAS && (dentro || llabs(c) > llabs(c0))) {
      corte = Corte::Limite;
      break;
    }
    // Sin cuentas: el motor no arrancó, está trabado o el encoder no cuenta. En los tres casos, parar.
    if (ahora - tUltimaCuenta >= J1_SIN_CUENTAS_MS) {
      corte = Corte::SinCuentas;
      break;
    }
    if (hayEntrada()) {
      corte = Corte::Enter;
      break;
    }
    delay(1);
  }
  aplicarDuty(0.0f);  // freno
  const uint32_t tPulso = millis() - t0;
  const int64_t cFin = cuentas();  // el veredicto usa lo que se movió con el duty aplicado
  delay(FRENO_MS);
  motorJ1.coast();

  const int64_t c1 = cuentas();
  const int64_t d = cFin - c0;
  Serial.printf("   cuentas %+lld -> %+lld al cortar el duty (%+lld, %+.1f° de salida); %+lld tras frenar\n",
                (long long)c0, (long long)cFin, (long long)d, 360.0f * d / J1_CUENTAS_POR_VUELTA, (long long)c1);

  switch (corte) {
    case Corte::SinCuentas:
      Serial.printf("   CORTADO: sin cuentas del encoder durante %lu ms (a los %lu ms).\n",
                    (unsigned long)J1_SIN_CUENTAS_MS, (unsigned long)tPulso);
      Serial.println("   O el motor no arrancó (¿12 V? ¿duty en la zona muerta?) o el encoder no cuenta.");
      Serial.println("   Antes de subir el duty, verificar el encoder con \"e\" girando a mano.");
      return;
    case Corte::Limite:
      Serial.printf("   CORTADO por el límite de %ld cuentas a los %lu ms. Resultado no concluyente:\n",
                    (long)J1_LIMITE_CUENTAS, (unsigned long)tPulso);
      Serial.println("   volver hacia el cero con un pulso del signo contrario.");
      return;
    case Corte::Enter:
      Serial.printf("   CORTADO con Enter a los %lu ms. Resultado no concluyente.\n", (unsigned long)tPulso);
      return;
    case Corte::Ninguno:
      break;
  }
  if (llabs(d) < J1_CUENTAS_SIN_GIRO) {
    Serial.println("   Casi no giró: resultado no concluyente. Probar con un poco más de duty o de tiempo.");
  } else if ((d > 0) != (pct > 0)) {
    Serial.println("   Las cuentas van al REVÉS del duty: escribir \"ie\" y repetir el pulso.");
  } else {
    Serial.println("   OK: motor y encoder de acuerdo.");
    Serial.printf("   ¿La base giró %s visto desde arriba? Si fue al revés, escribir \"is\".\n",
                  pct > 0 ? "ANTIHORARIO" : "HORARIO");
  }
}

// Lee "p <pct> [ms]". Rechaza números inválidos o fuera de rango.
void comandoPulso(const char *args) {
  char *fin = nullptr;
  const float pct = strtof(args, &fin);
  if (fin == args || !isfinite(pct) || pct == 0.0f || fabsf(pct) > J1_DUTY_MAX_PCT) {
    Serial.printf("Uso: p <%%> [ms], con 0 < |%%| <= %.0f\n", J1_DUTY_MAX_PCT);
    return;
  }
  uint32_t ms = PULSO_MS_DEF;
  while (*fin == ' ') fin++;
  if (*fin != '\0') {
    char *fin2 = nullptr;
    const long v = strtol(fin, &fin2, 10);
    while (*fin2 == ' ') fin2++;
    if (fin2 == fin || *fin2 != '\0' || v <= 0 || v > (long)J1_PULSO_MAX_MS) {
      Serial.printf("Tiempo inválido: entre 1 y %lu ms\n", (unsigned long)J1_PULSO_MAX_MS);
      return;
    }
    ms = (uint32_t)v;
  }
  if (!ceroFijado) {
    Serial.println("Primero llevar la base a su marca (al frente) y escribir \"z\".");
    return;
  }
  pulso(pct, ms);
}

void ejecutar(char *l) {
  while (*l == ' ') l++;
  if (*l == '\0') return;
  if (strcmp(l, "?") == 0) {
    ayuda();
  } else if (strcmp(l, "x") == 0) {
    motorJ1.coast();
    Serial.println(">> Rueda libre.");
  } else if (!hwOk) {
    Serial.println("Hardware sin inicializar: comandos deshabilitados.");
  } else if (strcmp(l, "e") == 0) {
    monitorEncoder();
  } else if (strcmp(l, "c") == 0) {
    Serial.printf("cuentas %+lld\n", (long long)cuentas());
  } else if (strcmp(l, "z") == 0) {
    encJ1.reset();
    ceroFijado = true;
    Serial.println(">> Cuentas a cero. Pulsos habilitados.");
  } else if (strcmp(l, "ie") == 0) {
    encInvertido = !encInvertido;
    mostrarSignos();
    Serial.printf("   Para dejarlo fijo: J1_ENCODER_INVERTIDO = %s en include/config.h\n",
                  encInvertido ? "true" : "false");
  } else if (strcmp(l, "is") == 0) {
    sentidoInvertido = !sentidoInvertido;
    mostrarSignos();
    Serial.printf("   Para dejarlo fijo: J1_SENTIDO_INVERTIDO = %s en include/config.h\n",
                  sentidoInvertido ? "true" : "false");
  } else if (l[0] == 'p' && (l[1] == ' ' || l[1] == '\0')) {
    comandoPulso(l + 1);
  } else {
    Serial.printf("Comando desconocido: \"%s\" (? para la ayuda)\n", l);
  }
}

void leerSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' && ultimoFueCR) {  // LF de un CR+LF: el Enter ya se procesó
      ultimoFueCR = false;
      continue;
    }
    ultimoFueCR = (ch == '\r');
    if (descartando) {  // cola de la línea que cortó un pulso: se tira hasta su Enter
      if (ch == '\r' || ch == '\n') descartando = false;
      continue;
    }
    if (ch == '\r' || ch == '\n') {
      linea[lineaLen] = '\0';
      if (lineaLarga) {
        Serial.println("Línea demasiado larga: descartada.");
      } else {
        ejecutar(linea);
      }
      lineaLen = 0;
      lineaLarga = false;
    } else if (lineaLen < sizeof(linea) - 1) {
      linea[lineaLen++] = ch;
    } else {
      lineaLarga = true;
    }
  }
}

void setup() {
  // Driver primero y deshabilitado: hasta acá los EN dependen del pull-down de 10 kΩ en GPIO23
  const bool okDriver = motorJ1.begin(PIN_J1_RPWM, PIN_J1_LPWM, PIN_J1_EN, PIN_J1_EN, LEDC_CH_R, LEDC_CH_L,
                                      J1_PWM_FREQ_HZ, J1_PWM_BITS);
  motorJ1.coast();
  // GPIO34/35 no tienen pull-up interno (el driver intenta activarlo y no pasa nada): usan los externos
  const bool okEnc = encJ1.begin(PIN_J1_ENC_A, PIN_J1_ENC_B);
  hwOk = okDriver && okEnc;

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("== 10 · Paso 0: sentido y signo de J1 (36GP-555 + IBT-2, ESP32-WROOM-32D) ==");
  Serial.printf("IBT-2: RPWM=GPIO%d, LPWM=GPIO%d, EN=GPIO%d | PWM %lu Hz\n", PIN_J1_RPWM, PIN_J1_LPWM, PIN_J1_EN,
                (unsigned long)J1_PWM_FREQ_HZ);
  Serial.printf("Encoder: A=GPIO%d, B=GPIO%d | %.0f cuentas por vuelta de salida de la reductora\n", PIN_J1_ENC_A,
                PIN_J1_ENC_B, J1_CUENTAS_POR_VUELTA);
  mostrarSignos();
  if (!okDriver) Serial.println("ERROR: no se pudo configurar el PWM (LEDC).");
  if (!okEnc) Serial.println("ERROR: no se pudo configurar el PCNT del encoder.");
  Serial.printf("Niveles del encoder en reposo: A=%d B=%d (con los pull-up, quieto puede ser 0 o 1)\n",
                encJ1.levelA(), encJ1.levelB());
  Serial.println("Motor en rueda libre. Pulsos deshabilitados hasta \"z\" con la base en su marca.");
  ayuda();
}

void loop() { leerSerial(); }
