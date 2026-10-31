// 17-brazo-joystick: J1, J2 y J3 a la vez con un joystick en una página web (tools/joystick.py hace de puente).
//  - Lazo abierto: cada eje del joystick (-1 a 1) da el duty de su articulación, entre dutyMin y dutyMax.
//  - El cero es donde arranca el programa; "Z" lo vuelve a fijar donde está. Además, cada articulación pone su
//    cero sola al tocar su final de carrera (si iba hacia él o estaba quieta).
//  - Con el final de carrera apretado no deja seguir hacia él (estado FC): solo alejarse.
//  - Cuatro barras: J2 y J3 no pueden separarse más de lo medido (config.h); en el borde solo deja moverse hacia
//    adentro de la franja (estado 4B).
//  - Joystick en el centro, hombre muerto (sin comandos en 300 ms) o atasco: duty 0 con el EN alto (freno).
//    Nunca rueda libre: J2 y J3 no se autobloquean.
//  - Límites desde el cero: fuera de ellos solo deja mover hacia adentro. El sentido "hacia afuera" se aprende
//    mientras se mueve (signo del duty contra el de las cuentas), así funciona aunque los signos de config.h estén mal.
//
// Protocolo por serie (115200, una línea por comando):
//   J <e1> <e2> <e3>   ejes del joystick, de -1 a 1 (repetir cada <300 ms mientras se mueve)
//   Z                  cero en la posición actual
//   S                  frenar las tres
//   L <n> <min> <max>  límites de la articulación n (1-3) en grados, en vivo (para medir el rango)
//   I <n>              invertir el sentido de la articulación n (motor y encoder juntos), en vivo
// Sale cada 50 ms:  Q <q1> <q2> <q3> <d1> <d2> <d3> <estado1> <estado2> <estado3> <fc1> <fc2> <fc3>
//                     <min1> <max1> <min2> <max2> <min3> <max3> <inv1> <inv2> <inv3>
//   q en grados, d en % (con signo), estado: OK, LIM (en el límite), FC (final de carrera), 4B (borde del
//   cuatro barras), TRABA (atasco),
//   SIGNO (encoder al revés); fc 1 = apretado; inv 1 = sentido invertido en vivo

#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"
#include "config.h"
#include "pins.h"

namespace {

constexpr int PIN_RPWM[N_ART] = {PIN_J1_RPWM, PIN_J2_RPWM, PIN_J3_RPWM};
constexpr int PIN_LPWM[N_ART] = {PIN_J1_LPWM, PIN_J2_LPWM, PIN_J3_LPWM};
constexpr int PIN_ENC_A[N_ART] = {PIN_J1_ENC_A, PIN_J2_ENC_A, PIN_J3_ENC_A};
constexpr int PIN_ENC_B[N_ART] = {PIN_J1_ENC_B, PIN_J2_ENC_B, PIN_J3_ENC_B};
constexpr pcnt_unit_t UNIDAD[N_ART] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2};
constexpr int PIN_FC[N_ART] = {PIN_FC_J1, PIN_FC_J2, PIN_FC_J3};

struct Articulacion {
  BTS7960 motor;
  EncoderPCNT enc;
  int64_t cero = 0;
  float eje = 0.0f;    // último eje recibido
  float duty = 0.0f;   // % aplicado (positivo = sentido positivo de la articulación)
  int relacion = 1;    // +1 si duty positivo hace subir los grados; se aprende al moverse
  bool signoMal = false;
  bool trabada = false;  // atasco: espera que el joystick vuelva al centro
  bool enLimite = false;
  bool enAcople = false;  // en el borde de la franja del cuatro barras, empujando hacia afuera
  bool soltado = false;   // el joystick está en el centro (para la rampa de bajada)
  int64_t cuentasAtasco = 0;
  uint32_t desdeAtasco = 0;
  int64_t cuentasRelacion = 0;
  uint32_t desdeRelacion = 0;
  float limNeg = 0.0f, limPos = 0.0f;  // límites en vivo (arrancan con los de config.h)
  bool inv = false;      // sentido invertido en vivo ("I"), además de los signos de config.h
  bool fc = false;       // final de carrera apretado (confirmado)
  int fcIguales = 0;     // lecturas seguidas distintas de "fc"
  int dirFC = -1;        // signo del duty que va hacia el final de carrera
  bool enFC = false;     // el joystick empuja hacia el final de carrera apretado
};

Articulacion art[N_ART];
uint32_t ultimoComando = 0;
bool sinComandos = false;  // hombre muerto: frena de golpe, sin rampa
bool okInicio = true;

int64_t cuentas(int i) {
  const int64_t c = art[i].enc.count() - art[i].cero;
  return (CFG[i].encoderInvertido != art[i].inv) ? -c : c;
}

float grados(int i) { return (float)cuentas(i) / CFG[i].cuentasPorGrado; }

void aplicar(int i, float dutyPct) {
  art[i].duty = dutyPct;
  const float d = ((CFG[i].sentidoInvertido != art[i].inv) ? -dutyPct : dutyPct) / 100.0f;
  art[i].motor.setDuty(d);
}

void frenarTodas() {
  for (int i = 0; i < N_ART; i++) {
    art[i].eje = 0.0f;
    aplicar(i, 0.0f);
  }
}

void ceroArt(int i) {
  art[i].cero = art[i].enc.count();
  art[i].enLimite = false;
  art[i].cuentasRelacion = art[i].cuentasAtasco = 0;
}

void cero() {
  for (int i = 0; i < N_ART; i++) ceroArt(i);
}

// Final de carrera: confirma el cambio con FC_LECTURAS lecturas seguidas. Al apretarse anota hacia qué lado iba
// el duty (así el bloqueo funciona aunque los signos estén mal) y, si iba hacia el cero o estaba quieta, pone el cero.
void leerFC(int i) {
  Articulacion &a = art[i];
  const bool apretado = digitalRead(PIN_FC[i]) == LOW;
  if (apretado == a.fc) {
    a.fcIguales = 0;
    return;
  }
  if (++a.fcIguales < FC_LECTURAS) return;
  a.fcIguales = 0;
  a.fc = apretado;
  if (!apretado) {
    Serial.printf("# %s: final de carrera suelto en %+.2f°\n", CFG[i].nombre, grados(i));
    return;
  }
  const int haciaCero = -a.relacion;  // signo del duty que baja los grados
  const int dir = a.duty > 0 ? 1 : (a.duty < 0 ? -1 : 0);
  if (dir != 0) a.dirFC = dir;
  if (dir == 0 || dir == haciaCero) {
    Serial.printf("# %s: final de carrera apretado en %+.2f°: cero acá\n", CFG[i].nombre, grados(i));
    ceroArt(i);
  } else {
    Serial.printf("# %s: final de carrera apretado yendo hacia + (%+.2f°): ¿signo al revés? Cero sin cambiar\n",
                  CFG[i].nombre, grados(i));
  }
}

// Duty pedido por el joystick para la articulación i, con los límites y el atasco.
float dutyPedido(int i) {
  Articulacion &a = art[i];
  const ConfigJoystick &c = CFG[i];
  const float eje = constrain(a.eje, -1.0f, 1.0f);
  a.soltado = fabsf(eje) < ZONA_MUERTA_JOY;
  if (a.soltado) {
    a.trabada = false;  // soltar el joystick borra el atasco
    return 0.0f;
  }
  if (a.trabada) return 0.0f;

  const float sentido = eje > 0 ? 1.0f : -1.0f;
  // Final de carrera apretado: no deja seguir hacia él
  a.enFC = a.fc && (int)sentido == a.dirFC;
  if (a.enFC) return 0.0f;
  const float q = grados(i);
  // Lado del límite en el que está (+1 arriba del máximo, -1 abajo del mínimo) y si el duty pedido lo empuja afuera
  const int lado = q > a.limPos ? 1 : (q < a.limNeg ? -1 : 0);
  a.enLimite = lado != 0 && (int)sentido * a.relacion == lado;
  if (a.enLimite) return 0.0f;
  // Cuatro barras: lado de la franja que bloquea (+1 no deja subir los grados de esta articulación, -1 no deja bajarlos)
  a.enAcople = false;
  if (i == 1 || i == 2) {
    const float q3MenosQ2 = grados(2) - grados(1);
    int ladoAcople = 0;
    if (q3MenosQ2 >= Q3_MENOS_Q2_MAX) ladoAcople = (i == 2) ? 1 : -1;        // codo demasiado abajo
    else if (-q3MenosQ2 >= Q2_MENOS_Q3_MAX) ladoAcople = (i == 1) ? 1 : -1;  // hombro demasiado arriba
    a.enAcople = ladoAcople != 0 && (int)sentido * a.relacion == ladoAcople;
    if (a.enAcople) return 0.0f;
  }

  return sentido * (c.dutyMin + (c.dutyMax - c.dutyMin) * (fabsf(eje) - ZONA_MUERTA_JOY) / (1.0f - ZONA_MUERTA_JOY));
}

void controlar(int i, uint32_t ahora) {
  Articulacion &a = art[i];
  const float objetivo = dutyPedido(i);

  // Rampa: sube despacio, baja (o cambia de sentido) de golpe
  float d = objetivo;
  const float paso = RAMPA_PCT_POR_S * TS_MS / 1000.0f;
  if (objetivo != 0.0f && objetivo * a.duty >= 0.0f && fabsf(objetivo) > fabsf(a.duty) + paso)
    d = a.duty + (objetivo > 0 ? paso : -paso);
  if (objetivo * a.duty < 0.0f) d = 0.0f;  // al invertir pasa por 0 un ciclo
  // Rampa de bajada solo al soltar el joystick, lejos de los límites y con el final de carrera suelto
  const float pasoBajada = RAMPA_BAJADA_PCT_POR_S[i] * TS_MS / 1000.0f;
  const float q = grados(i);
  if (objetivo == 0.0f && a.soltado && !sinComandos && pasoBajada > 0.0f && !a.fc && q > a.limNeg && q < a.limPos &&
      fabsf(a.duty) > pasoBajada)
    d = a.duty - (a.duty > 0 ? pasoBajada : -pasoBajada);
  aplicar(i, d);

  const int64_t n = cuentas(i);
  // Aprende el sentido: con duty firme, cada 100 ms mira hacia dónde fueron las cuentas
  if (fabsf(a.duty) >= CFG[i].dutyMin && ahora - a.desdeRelacion >= 100) {
    const int64_t dn = n - a.cuentasRelacion;
    if (llabs(dn) >= 10) {
      a.relacion = ((dn > 0) == (a.duty > 0)) ? 1 : -1;
      a.signoMal = a.relacion < 0;
    }
    a.cuentasRelacion = n;
    a.desdeRelacion = ahora;
  } else if (fabsf(a.duty) < CFG[i].dutyMin) {
    a.cuentasRelacion = n;
    a.desdeRelacion = ahora;
  }

  // Atasco
  if (fabsf(a.duty) >= ATASCO_DUTY_PCT) {
    if (llabs(n - a.cuentasAtasco) >= ATASCO_CUENTAS) {
      a.cuentasAtasco = n;
      a.desdeAtasco = ahora;
    } else if (ahora - a.desdeAtasco >= ATASCO_MS) {
      a.trabada = true;
      aplicar(i, 0.0f);
    }
  } else {
    a.cuentasAtasco = n;
    a.desdeAtasco = ahora;
  }
}

void leerComandos() {
  static char linea[64];
  static size_t largo = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch != '\n' && ch != '\r') {
      if (largo < sizeof(linea) - 1) linea[largo++] = ch;
      continue;
    }
    linea[largo] = '\0';
    largo = 0;
    float e[N_ART];
    if (linea[0] == 'J' && sscanf(linea + 1, "%f %f %f", &e[0], &e[1], &e[2]) == N_ART) {
      for (int i = 0; i < N_ART; i++) art[i].eje = isfinite(e[i]) ? e[i] : 0.0f;
      ultimoComando = millis();
    } else if (linea[0] == 'Z') {
      cero();
      Serial.println("# cero en la posición actual");
    } else if (linea[0] == 'S') {
      frenarTodas();
      for (int i = 0; i < N_ART; i++) art[i].soltado = false;  // FRENAR: de golpe, sin rampa
      Serial.println("# frenadas");
    } else if (linea[0] == 'L') {
      int n;
      float mn, mx;
      if (sscanf(linea + 1, "%d %f %f", &n, &mn, &mx) == 3 && n >= 1 && n <= N_ART && isfinite(mn) && isfinite(mx) &&
          mn < mx) {
        art[n - 1].limNeg = mn;
        art[n - 1].limPos = mx;
        Serial.printf("# %s: límites %+.0f / %+.0f°\n", CFG[n - 1].nombre, mn, mx);
      } else {
        Serial.println("# uso: L <n> <min> <max>");
      }
    } else if (linea[0] == 'I') {
      int n;
      if (sscanf(linea + 1, "%d", &n) == 1 && n >= 1 && n <= N_ART) {
        Articulacion &a = art[n - 1];
        aplicar(n - 1, 0.0f);
        a.inv = !a.inv;
        a.relacion = 1;
        a.signoMal = false;
        a.dirFC = -1;
        a.cuentasRelacion = a.cuentasAtasco = cuentas(n - 1);
        Serial.printf("# %s: sentido %s (en config.h: encoder %d, sentido %d)\n", CFG[n - 1].nombre,
                      a.inv ? "INVERTIDO en vivo" : "como config.h", CFG[n - 1].encoderInvertido != a.inv,
                      CFG[n - 1].sentidoInvertido != a.inv);
      } else {
        Serial.println("# uso: I <n>");
      }
    }
  }
}

const char *estado(int i) {
  const Articulacion &a = art[i];
  if (a.trabada) return "TRABA";
  if (a.enFC) return "FC";
  if (a.enLimite) return "LIM";
  if (a.enAcople) return "4B";
  if (a.signoMal) return "SIGNO";
  return "OK";
}

}  // namespace

void setup() {
  // Antes que nada, EN y los seis PWM en bajo (hasta acá el EN depende del pull-down de 10 kΩ)
  for (int p : {PIN_BRAZO_EN, PIN_J1_RPWM, PIN_J1_LPWM, PIN_J2_RPWM, PIN_J2_LPWM, PIN_J3_RPWM, PIN_J3_LPWM}) {
    digitalWrite(p, LOW);
    pinMode(p, OUTPUT);
  }
  Serial.begin(115200);
  delay(200);
  for (int i = 0; i < N_ART; i++) {
    pinMode(PIN_FC[i], INPUT_PULLUP);
    art[i].limNeg = CFG[i].limiteNeg;
    art[i].limPos = CFG[i].limitePos;
    const bool okM = art[i].motor.begin(PIN_RPWM[i], PIN_LPWM[i], PIN_BRAZO_EN, PIN_BRAZO_EN, 2 * i, 2 * i + 1,
                                        CFG[i].pwmHz, PWM_BITS);
    const bool okE = art[i].enc.begin(PIN_ENC_A[i], PIN_ENC_B[i], UNIDAD[i]);
    if (!okM || !okE) okInicio = false;
    Serial.printf("# %s: RPWM %d, LPWM %d, encoder %d/%d, PWM %lu Hz, límites %.0f / %.0f°, duty %.0f-%.0f %% %s\n",
                  CFG[i].nombre, PIN_RPWM[i], PIN_LPWM[i], PIN_ENC_A[i], PIN_ENC_B[i], (unsigned long)CFG[i].pwmHz,
                  CFG[i].limiteNeg, CFG[i].limitePos, CFG[i].dutyMin, CFG[i].dutyMax,
                  (okM && okE) ? "" : "FALLA AL INICIAR");
  }
  if (!okInicio) {
    for (int i = 0; i < N_ART; i++) art[i].motor.coast();
    Serial.println("# ERROR: no se pudo iniciar un driver o un encoder. Motores deshabilitados.");
    return;
  }
  frenarTodas();  // EN alto y duty 0: las tres frenadas
  cero();
  delay(5);
  for (int i = 0; i < N_ART; i++) {
    art[i].fc = digitalRead(PIN_FC[i]) == LOW;
    Serial.printf("# %s: final de carrera (GPIO %d) %s\n", CFG[i].nombre, PIN_FC[i], art[i].fc ? "APRETADO" : "suelto");
  }
  Serial.println("# Listo. Cero en la posición de arranque. Comandos: J e1 e2 e3 | Z | S | L n min max | I n");
}

void loop() {
  static uint32_t ultimoLazo = 0, ultimaTelemetria = 0;
  if (!okInicio) {
    delay(1000);
    return;
  }
  leerComandos();
  const uint32_t ahora = millis();
  if (ahora - ultimoLazo >= TS_MS) {
    ultimoLazo = ahora;
    sinComandos = ahora - ultimoComando > HOMBRE_MUERTO_MS;
    if (sinComandos) {
      for (int i = 0; i < N_ART; i++) art[i].eje = 0.0f;
    }
    for (int i = 0; i < N_ART; i++) {
      leerFC(i);
      controlar(i, ahora);
    }
  }
  if (ahora - ultimaTelemetria >= TELEMETRIA_MS) {
    ultimaTelemetria = ahora;
    Serial.printf("Q %.2f %.2f %.2f %.1f %.1f %.1f %s %s %s %d %d %d %.0f %.0f %.0f %.0f %.0f %.0f %d %d %d\n",
                  grados(0), grados(1), grados(2), art[0].duty, art[1].duty, art[2].duty, estado(0), estado(1),
                  estado(2), art[0].fc, art[1].fc, art[2].fc, art[0].limNeg, art[0].limPos, art[1].limNeg,
                  art[1].limPos, art[2].limNeg, art[2].limPos, art[0].inv, art[1].inv, art[2].inv);
  }
}
