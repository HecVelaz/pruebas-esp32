// 15 · Prueba del IBT-2 de J3 (2 x BTS7960B + 74HC244) con una ESP32-C3-MINI-1 y el motor montado en el brazo.
// Solo pulsos cortos de duty fijo en los dos sentidos ("+" y "-"), para ver que el driver mueve el motor.
// Para que el codo no dé una vuelta sin querer:
//  - duty máximo 60 % y pulsos de 500 ms como mucho (a 60 %, ~20° por pulso en el peor caso);
//  - el encoder corta el pulso si se pasa de ±15° desde el cero ("z") y no deja seguir alejándose;
//  - si el encoder no cuenta en 150 ms, corta (motor trabado, driver muerto o encoder mal);
//  - respaldo por tiempo: la suma de pulsos (+ menos -) desde el último "z" no pasa de 3 s (0,6 s con "se");
//  - cualquier tecla durante un pulso lo corta.
// Parámetros en include/config.h y pines en include/pins.h.
#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderCuadratura.h"
#include "config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_R = 0;
constexpr uint8_t LEDC_CH_L = 1;
constexpr uint32_t ASENTAR_MS = 100;  // después del pulso, antes de leer cuánto se movió
constexpr int32_t LIMITE_C = (int32_t)(cfg::LIMITE_GRADOS * cfg::CUENTAS_POR_GRADO);

BTS7960 motor;
EncoderCuadratura enc;
bool hwOk = false;

float dutyPct = cfg::DUTY_DEF_PCT;
uint32_t pulsoMs = cfg::PULSO_DEF_MS;
bool usarEncoder = true;  // "se" lo apaga: sin corte por falta de cuentas, y el respaldo por tiempo más corto
int signoEnc = 0;         // +1: el duty positivo suma cuentas; -1: resta; 0: todavía no se sabe
int32_t netoMs = 0;       // pulsos positivos menos negativos desde el último "z"
float diagPendiente = 0;  // "m <duty>" esperando la confirmación "s" (0: nada pendiente)

char linea[32];
size_t lineaLen = 0;
bool descartando = false;  // una tecla cortó un pulso: el resto de esa línea no se ejecuta
bool lineaSucia = false;   // llegó un byte que no es texto (ruido en el RX): la línea entera se descarta

float grados(int32_t c) { return c / cfg::CUENTAS_POR_GRADO; }

void reposo() {
  if (cfg::FRENO_EN_REPOSO)
    motor.brake();
  else
    motor.coast();
}

void ayuda() {
  Serial.println();
  Serial.println("== 15 · IBT-2 de J3 con ESP32-C3 ==");
  Serial.println("  +  / -      pulso positivo / negativo (cualquier tecla lo corta)");
  Serial.println("  p <duty> [ms]  pulso suelto con duty con signo (p 30 100, p -30 100)");
  Serial.printf("  d <pct>     duty de los pulsos (ahora %.0f %%, máx %.0f %%)\n", dutyPct, cfg::DUTY_MAX_PCT);
  Serial.printf("  t <ms>      duración de los pulsos (ahora %lu ms, máx %lu ms)\n", (unsigned long)pulsoMs,
                (unsigned long)cfg::PULSO_MAX_MS);
  Serial.printf("  z           cero acá (límites ±%.0f°) y borra el tiempo neto\n", cfg::LIMITE_GRADOS);
  Serial.println("  e           encoder: cuentas, grados y niveles de A y B");
  Serial.println("  f / x       freno (EN alto) / rueda libre (EN bajo: el antebrazo cae)");
  Serial.println("  m <duty>    diagnóstico, MOTOR DESCONECTADO: duty fijo hasta una tecla (m 30, m -30)");
  Serial.printf("  se          encoder %s (sin él: sin corte por falta de cuentas, neto máx %lu ms)\n",
                usarEncoder ? "ACTIVO" : "APAGADO", (unsigned long)cfg::NETO_MAX_SIN_ENC_MS);
  Serial.println("  ?           esta ayuda");
}

void mostrarEncoder() {
  const int32_t c = enc.count();
  Serial.printf("encoder: %ld cuentas = %+.2f° | A=%d B=%d | neto %ld ms | signo %s\n", (long)c, grados(c),
                enc.levelA(), enc.levelB(), (long)netoMs,
                signoEnc == 0 ? "sin medir" : (signoEnc > 0 ? "duty + suma cuentas" : "duty + RESTA cuentas"));
}

// Un pulso de duty fijo (en %) en el sentido dir (+1 o -1), de ms milisegundos
void pulso(int dir, float duty, uint32_t ms) {
  if (!hwOk) {
    Serial.println("ERROR: el PWM no arrancó, no se mueve");
    return;
  }
  const int32_t c0 = enc.count();

  // Límite de ángulo: fuera de ±LIMITE solo se deja volver, y para eso hay que saber el signo del encoder
  if (usarEncoder && abs(c0) >= LIMITE_C) {
    if (signoEnc == 0) {
      Serial.println("NO: fuera del límite y todavía no sé hacia dónde vuelve. Moverlo a mano o hacer z");
      return;
    }
    if ((int32_t)(dir * signoEnc) * c0 > 0) {
      Serial.printf("NO: en %+.1f° ese sentido lo aleja más del cero (límite ±%.0f°)\n", grados(c0),
                    cfg::LIMITE_GRADOS);
      return;
    }
  }
  // Respaldo por tiempo
  const int32_t netoMax = usarEncoder ? cfg::NETO_MAX_MS : cfg::NETO_MAX_SIN_ENC_MS;
  const int32_t netoNuevo = netoMs + dir * (int32_t)ms;
  if (abs(netoNuevo) > netoMax && abs(netoNuevo) > abs(netoMs)) {
    Serial.printf("NO: tiempo neto %ld ms, el máximo es ±%ld ms. Volver con el otro sentido o hacer z\n",
                  (long)netoMs, (long)netoMax);
    return;
  }

  while (Serial.available()) Serial.read();
  const char *corte = "tiempo";
  motor.setDuty(dir * duty / 100.0f);
  const uint32_t t0 = millis();
  uint32_t dt = 0;
  while ((dt = millis() - t0) < ms) {
    if (Serial.available()) {
      corte = "TECLA";
      descartando = true;
      break;
    }
    const int32_t c = enc.count();
    // Pasó el límite y se sigue alejando del cero (no depende del signo del encoder)
    if (abs(c) >= LIMITE_C && (c - c0) * (int64_t)c > 0) {
      corte = "LÍMITE";
      break;
    }
    if (usarEncoder && dt >= cfg::SIN_CUENTAS_MS && abs(c - c0) < cfg::CUENTAS_SIN_GIRO) {
      corte = "SIN CUENTAS";
      break;
    }
    delay(1);
  }
  reposo();
  netoMs += dir * (int32_t)dt;

  delay(ASENTAR_MS);
  const int32_t c1 = enc.count();
  const int32_t dc = c1 - c0;
  Serial.printf("pulso %c %.0f %% %lu ms (corte: %s) -> %+ld cuentas = %+.2f° | ahora %+.2f° | neto %ld ms\n",
                dir > 0 ? '+' : '-', duty, (unsigned long)dt, corte, (long)dc, grados(dc), grados(c1),
                (long)netoMs);

  if (abs(dc) >= cfg::CUENTAS_SIN_GIRO) {
    const int s = (dc > 0) == (dir > 0) ? 1 : -1;
    if (s != signoEnc) {
      Serial.printf("  signo: duty %c -> cuentas %s%s\n", dir > 0 ? '+' : '-', dc > 0 ? "suben" : "bajan",
                    signoEnc != 0 ? " (CAMBIÓ: ¿cayó por el peso?)" : "");
      signoEnc = s;
    }
  } else if (strcmp(corte, "SIN CUENTAS") == 0) {
    Serial.println("  no contó: si el motor NO se movió, subir el duty (d) o revisar el driver;");
    Serial.println("  si se movió, revisar el encoder (e girando a mano) o seguir sin él (se)");
  }
}

// Diagnóstico con el motor desconectado: duty fijo hasta una tecla, para medir RPWM, LPWM, EN, M+ y M-
void diagnostico(float duty) {
  if (!hwOk) {
    Serial.println("ERROR: el PWM no arrancó");
    return;
  }
  const char *activo = duty > 0 ? "RPWM (GPIO 4)" : "LPWM (GPIO 5)";
  const char *quieto = duty > 0 ? "LPWM" : "RPWM";
  Serial.printf("diagnóstico %+.0f %%: %s con PWM, %s en 0, EN alto. Cualquier tecla corta (máx %lu s)\n", duty,
                activo, quieto, (unsigned long)(cfg::DIAG_MAX_MS / 1000));
  Serial.printf("  esperado: %s ~%.1f V (multímetro en DC) | M+ a M- ~%+.1f V con 12 V\n", activo,
                3.3f * fabsf(duty) / 100.0f, 12.0f * duty / 100.0f);

  while (Serial.available()) Serial.read();
  const int32_t c0 = enc.count();
  const char *corte = "tiempo";
  motor.setDuty(duty / 100.0f);
  const uint32_t t0 = millis();
  while (millis() - t0 < cfg::DIAG_MAX_MS) {
    if (Serial.available()) {
      corte = "TECLA";
      descartando = true;
      break;
    }
    if (abs(enc.count() - c0) >= cfg::CUENTAS_SIN_GIRO) {
      corte = "ENCODER (¡el motor está conectado!)";
      break;
    }
    delay(1);
  }
  reposo();
  Serial.printf("diagnóstico terminado (corte: %s) tras %lu ms\n", corte, (unsigned long)(millis() - t0));
}

void ejecutar(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') return;
  char *arg = s;
  while (*arg && *arg != ' ') arg++;
  if (*arg) *arg++ = '\0';

  // "m <duty>" pide confirmar: "s" lo ejecuta, cualquier otra cosa lo cancela
  if (diagPendiente != 0) {
    const float d = diagPendiente;
    diagPendiente = 0;
    if (!strcmp(s, "s")) {
      diagnostico(d);
    } else {
      Serial.println("diagnóstico cancelado");
    }
    return;
  }

  if (!strcmp(s, "+")) {
    pulso(+1, dutyPct, pulsoMs);
  } else if (!strcmp(s, "-")) {
    pulso(-1, dutyPct, pulsoMs);
  } else if (!strcmp(s, "p")) {
    // p <duty con signo> [ms]: un pulso suelto, sin cambiar lo de "d" y "t"
    char *fin;
    const float v = strtof(arg, &fin);
    if (fin == arg || v == 0.0f || fabsf(v) > cfg::DUTY_MAX_PCT) {
      Serial.printf("p <duty> [ms]: duty entre -%.0f y %.0f %% (no 0), ms entre 10 y %lu\n", cfg::DUTY_MAX_PCT,
                    cfg::DUTY_MAX_PCT, (unsigned long)cfg::PULSO_MAX_MS);
      return;
    }
    long ms = pulsoMs;
    while (*fin == ' ') fin++;
    if (*fin) ms = atol(fin);
    if (ms < 10 || ms > (long)cfg::PULSO_MAX_MS) {
      Serial.printf("duración entre 10 y %lu ms\n", (unsigned long)cfg::PULSO_MAX_MS);
      return;
    }
    pulso(v > 0 ? +1 : -1, fabsf(v), ms);
  } else if (!strcmp(s, "d")) {
    const float v = atof(arg);
    if (v <= 0.0f || v > cfg::DUTY_MAX_PCT) {
      Serial.printf("duty entre 1 y %.0f %%\n", cfg::DUTY_MAX_PCT);
      return;
    }
    dutyPct = v;
    Serial.printf("duty %.0f %%\n", dutyPct);
  } else if (!strcmp(s, "t")) {
    const long v = atol(arg);
    if (v < 10 || v > (long)cfg::PULSO_MAX_MS) {
      Serial.printf("duración entre 10 y %lu ms\n", (unsigned long)cfg::PULSO_MAX_MS);
      return;
    }
    pulsoMs = v;
    Serial.printf("pulsos de %lu ms\n", (unsigned long)pulsoMs);
  } else if (!strcmp(s, "z")) {
    enc.reset();
    netoMs = 0;
    Serial.printf("cero acá; límites ±%.0f°\n", cfg::LIMITE_GRADOS);
  } else if (!strcmp(s, "e")) {
    mostrarEncoder();
  } else if (!strcmp(s, "f")) {
    motor.brake();
    Serial.println("freno (EN alto, PWM 0)");
  } else if (!strcmp(s, "x")) {
    motor.coast();
    Serial.println("rueda libre (EN bajo): sostener el antebrazo");
  } else if (!strcmp(s, "m")) {
    char *fin;
    const float v = strtof(arg, &fin);
    if (fin == arg || v == 0.0f || fabsf(v) > cfg::DUTY_MAX_PCT) {
      Serial.printf("m <duty>: duty entre -%.0f y %.0f %% (no 0)\n", cfg::DUTY_MAX_PCT, cfg::DUTY_MAX_PCT);
      return;
    }
    diagPendiente = v;
    Serial.println("ATENCIÓN: M+ y M- tienen que estar DESCONECTADOS del motor (si no, el codo gira).");
    Serial.printf("  s = empezar diagnóstico a %+.0f %%, cualquier otra cosa cancela\n", v);
  } else if (!strcmp(s, "se")) {
    usarEncoder = !usarEncoder;
    Serial.printf("encoder %s\n", usarEncoder ? "ACTIVO" : "APAGADO (sin corte por cuentas, neto corto)");
  } else if (!strcmp(s, "?")) {
    ayuda();
  } else {
    Serial.printf("no entiendo \"%s\" (? = ayuda)\n", s);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);  // para que el monitor alcance a abrir

  hwOk = motor.begin(PIN_RPWM, PIN_LPWM, PIN_EN, PIN_EN, LEDC_CH_R, LEDC_CH_L, cfg::PWM_FREQ_HZ, cfg::PWM_BITS);
  enc.begin(PIN_ENC_A, PIN_ENC_B);
  if (hwOk) reposo();

  ayuda();
  Serial.printf("PWM %lu Hz %s | %s en reposo | cero = donde arrancó (hacer z en la posición de prueba)\n",
                (unsigned long)cfg::PWM_FREQ_HZ, hwOk ? "OK" : "FALLÓ",
                cfg::FRENO_EN_REPOSO ? "freno" : "rueda libre");
  mostrarEncoder();
}

void loop() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      linea[lineaLen] = '\0';
      if (lineaSucia)
        Serial.println("línea con ruido descartada (¿algo conectado al RX, GPIO 20?)");
      else if (!descartando)
        ejecutar(linea);
      lineaLen = 0;
      descartando = false;
      lineaSucia = false;
    } else if (ch < ' ' || ch > '~') {
      lineaSucia = true;  // el ruido nunca debe poder armar un "+" o un "-"
    } else if (lineaLen < sizeof(linea) - 1) {
      linea[lineaLen++] = ch;
    }
  }
}
