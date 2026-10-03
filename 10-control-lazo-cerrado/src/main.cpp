// Pasos 0 y 1 del lazo cerrado de J1 (36GP-555 + IBT-2, base giratoria) en la ESP32-WROOM-32D.
// Todavía no hay control:
//  - Paso 0: pulsos cortos de duty fijo para comprobar que "positivo" es lo mismo para el motor,
//    el encoder y la articulación (q1 positivo = antihorario visto desde arriba). Hecho: ver config.h.
//  - Paso 1: "m <grados>" lleva la base a un ángulo con duty fijo y frena al llegar, para verificar
//    las cuentas por grado (transmisión 50:1 x correa 90/18). Hecho: 44,44 cuentas por grado.
//  - Paso 2: PI de velocidad a 100 Hz con compensación de la zona muerta. "ev <°/s> [ms] [ciclos]" repite
//    +v, pausa, -v, pausa y manda un CSV que tools/escalon_velocidad.py compara con la simulación.
#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"
#include "config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_R = 0;
constexpr uint8_t LEDC_CH_L = 1;
constexpr uint32_t PULSO_MS_DEF = 200;
constexpr uint32_t FRENO_MS = 300;        // freno después de mover, antes de dejarlo en rueda libre
constexpr uint32_t MONITOR_CADA_MS = 200;
constexpr uint32_t QUIETO_MS = 100;       // antes de mover, la base tiene que estar quieta este tiempo
constexpr int64_t LIM_POS_C = (int64_t)(J1_LIMITE_POS_GRADOS * J1_CUENTAS_POR_GRADO);
constexpr int64_t LIM_NEG_C = (int64_t)(J1_LIMITE_NEG_GRADOS * J1_CUENTAS_POR_GRADO);

BTS7960 motorJ1;
EncoderPCNT encJ1;
bool hwOk = false;
bool ceroFijado = false;  // sin "z" no se mueve: el cero tiene que estar en la marca, lejos de los topes

// Signos: los de config.h (paso 0). "ie" e "is" ya no los cambian (ver ejecutar())
const bool encInvertido = J1_ENCODER_INVERTIDO;
const bool sentidoInvertido = J1_SENTIDO_INVERTIDO;

// Ganancias del PI de velocidad: arrancan con las de config.h y se cambian con "kp" y "ki"
float kpVel = J1_KP_VEL;
float kiVel = J1_KI_VEL;

char linea[48];
size_t lineaLen = 0;
bool lineaLarga = false;   // se pasó del buffer: se descarta entera al llegar el Enter
bool ultimoFueCR = false;  // CR+LF cuenta como un solo Enter
bool descartando = false;  // una línea cortó un movimiento o el monitor: el resto de esa línea no se ejecuta
uint8_t escape = 0;        // secuencia de escape de la terminal (flechas): 1 = llegó ESC, 2 = dentro de ESC [

// Cuentas de J1 con los signos aplicados
int64_t cuentas() {
  int64_t c = encJ1.count();
  if (encInvertido) c = -c;
  if (sentidoInvertido) c = -c;
  return c;
}

float grados(int64_t c) { return (float)c / J1_CUENTAS_POR_GRADO; }

// Cuánto se pasó del rango permitido (0 si está dentro)
int64_t excesoLimite(int64_t c) {
  if (c > LIM_POS_C) return c - LIM_POS_C;
  if (c < LIM_NEG_C) return LIM_NEG_C - c;
  return 0;
}
// En el límite o fuera, y moverse en el sentido dir (+1/-1) la alejaría más
bool haciaAfuera(int64_t c, int dir) { return (c >= LIM_POS_C && dir > 0) || (c <= LIM_NEG_C && dir < 0); }

void aplicarDuty(float pct) {
  float d = pct / 100.0f;
  if (sentidoInvertido) d = -d;
  motorJ1.setDuty(d);
}

// Hay algo escrito en el monitor: corta el movimiento o el monitor del encoder.
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
  Serial.println("  e            ver el encoder (cuentas, grados de la base, niveles A/B); Enter para salir");
  Serial.println("  z            poner a cero con la base en su marca (al frente). Habilita los movimientos");
  Serial.printf("  m <°> [%%]    ir a un ángulo de la base con duty fijo (por defecto %.0f %%) y frenar. Ej: m 10, m -30\n",
                J1_DUTY_MOVER_PCT);
  Serial.printf("  p <%%> [ms]   pulso de duty fijo, |%%| <= %.0f, por defecto %lu ms (máx %lu). Ej: p 30 300\n",
                J1_DUTY_MAX_PCT, (unsigned long)PULSO_MS_DEF, (unsigned long)J1_PULSO_MAX_MS);
  Serial.printf("  ev <°/s> [ms] [ciclos]  escalones de velocidad: +v ms, 0 %lu ms, -v ms, 0 %lu ms, repetido (CSV).\n"
                "               Ej: ev 20 1000 40 (2 min)\n", (unsigned long)J1_PAUSA_MS, (unsigned long)J1_PAUSA_MS);
  Serial.println("  kp <x>, ki <x>  cambiar las ganancias del PI de velocidad · g  mostrarlas");
  Serial.println("  c            mostrar cuentas y grados");
  Serial.println("  x            rueda libre (driver deshabilitado)");
  Serial.println("  ?            esta ayuda");
  Serial.printf("Seguridad: corte fuera de %+.0f° / %+.0f° del cero, si avanza menos de %ld cuentas\n",
                J1_LIMITE_POS_GRADOS, J1_LIMITE_NEG_GRADOS, (long)J1_AVANCE_MIN);
  Serial.printf("en %lu ms o con cualquier Enter. Duty máximo %.0f %%.\n", (unsigned long)J1_SIN_CUENTAS_MS,
                J1_DUTY_MAX_PCT);
  Serial.println();
}

void mostrarSignos() {
  Serial.printf("Signos: encoder %s, sentido %s\n", encInvertido ? "INVERTIDO" : "normal",
                sentidoInvertido ? "INVERTIDO" : "normal");
}

void mostrarPosicion() {
  const int64_t c = cuentas();
  Serial.printf("cuentas %+lld | base %+.2f°\n", (long long)c, grados(c));
}

void monitorEncoder() {
  motorJ1.coast();
  Serial.println(">> Girá a mano el disco del imán (con los 12 V apagados). Enter para salir.");
  int64_t ultima = INT64_MIN;
  uint32_t t = 0;
  while (!hayEntrada()) {
    if (millis() - t < MONITOR_CADA_MS) continue;
    t = millis();
    const int64_t c = cuentas();
    if (c == ultima) continue;  // solo cuando cambia, para no llenar la pantalla
    ultima = c;
    Serial.printf("   cuentas %+6lld | base %+7.2f° | A=%d B=%d\n", (long long)c, grados(c), encJ1.levelA(),
                  encJ1.levelB());
  }
  Serial.println(">> Fin del monitor.");
}

// Espera QUIETO_MS sin que la base se mueva (un movimiento anterior o la mano pueden dejarla girando).
// Un Enter durante la espera cancela antes de energizar el motor.
bool listaParaMover() {
  const int64_t c = cuentas();
  const uint32_t t0 = millis();
  while (millis() - t0 < QUIETO_MS) {
    if (hayEntrada()) {
      Serial.println(">> Cancelado.");
      return false;
    }
    if (llabs(cuentas() - c) > 1) {
      Serial.println("La base todavía se mueve: esperar a que se detenga y repetir.");
      return false;
    }
    delay(1);
  }
  return true;
}

// Resultado de un movimiento con duty fijo
enum class Corte { Tiempo, Objetivo, Limite, HaciaAfuera, SinAvance, Enter };
struct Movimiento {
  Corte corte;
  int64_t c0;     // cuentas al empezar
  int64_t cFin;   // al cortar el duty
  int64_t c1;     // después de frenar
  uint32_t ms;    // duración con duty aplicado
};

// Aplica pct durante como mucho msMax. Si conObjetivo, frena al alcanzar o pasar el objetivo.
// Protecciones: límite de ángulo (fuera de él solo se permite volver, y un movimiento hacia afuera
// ni arranca), avance mínimo en el sentido pedido y Enter.
Movimiento mover(float pct, uint32_t msMax, bool conObjetivo, int64_t objetivo) {
  Movimiento m{Corte::Tiempo, cuentas(), 0, 0, 0};
  const int dir = pct > 0 ? 1 : -1;
  if (haciaAfuera(m.c0, dir)) {
    m.corte = Corte::HaciaAfuera;  // sin energizar el motor
    m.cFin = m.c1 = m.c0;
    return m;
  }
  bool dentro = excesoLimite(m.c0) == 0;
  const uint32_t t0 = millis();
  // Ventana de avance: cada J1_SIN_CUENTAS_MS la base tiene que haber avanzado J1_AVANCE_MIN cuentas
  // en el sentido pedido. Cuentas que van y vuelven (una fase del encoder suelta) no alcanzan.
  uint32_t tVentana = t0;
  int64_t cVentana = m.c0;
  // Y además, ninguna cuenta durante J1_SIN_CUENTAS_MS (encoder desconectado del todo) corta enseguida
  uint32_t tUltimaCuenta = t0;
  int64_t cAnt = m.c0;
  aplicarDuty(pct);
  while (millis() - t0 < msMax) {
    const int64_t c = cuentas();
    const uint32_t ahora = millis();
    if (c != cAnt) {
      cAnt = c;
      tUltimaCuenta = ahora;
    }
    if (conObjetivo && (c - objetivo) * dir >= 0) {
      m.corte = Corte::Objetivo;
      break;
    }
    // Fuera del rango solo se permite volver: corta si ya había entrado o si se aleja más
    if (excesoLimite(c) == 0) dentro = true;
    if (excesoLimite(c) > 0 && (dentro || excesoLimite(c) > excesoLimite(m.c0))) {
      m.corte = Corte::Limite;
      break;
    }
    // Sin avance: el motor no arrancó, está trabado o el encoder no cuenta bien. En todos los casos, parar.
    if (ahora - tUltimaCuenta >= J1_SIN_CUENTAS_MS) {
      m.corte = Corte::SinAvance;
      break;
    }
    if (ahora - tVentana >= J1_SIN_CUENTAS_MS) {
      if ((c - cVentana) * dir < J1_AVANCE_MIN) {
        m.corte = Corte::SinAvance;
        break;
      }
      tVentana = ahora;
      cVentana = c;
    }
    if (hayEntrada()) {
      m.corte = Corte::Enter;
      break;
    }
    delay(1);
  }
  aplicarDuty(0.0f);  // freno
  m.ms = millis() - t0;
  m.cFin = cuentas();
  delay(FRENO_MS);
  motorJ1.coast();
  m.c1 = cuentas();
  return m;
}

// Explica un corte por protección. Devuelve true si hubo uno.
bool informarCorte(const Movimiento &m) {
  switch (m.corte) {
    case Corte::SinAvance:
      Serial.printf("   CORTADO: avanzó menos de %ld cuentas en %lu ms (a los %lu ms).\n", (long)J1_AVANCE_MIN,
                    (unsigned long)J1_SIN_CUENTAS_MS, (unsigned long)m.ms);
      Serial.println("   O el motor no arrancó (¿12 V? ¿duty en la zona muerta?), está trabado o el encoder no cuenta bien.");
      Serial.println("   Revisar el encoder con \"e\" (12 V apagados) antes de subir el duty.");
      return true;
    case Corte::HaciaAfuera:
      Serial.printf("   RECHAZADO: está en el límite (%+.0f° / %+.0f°) y eso la alejaría más. Solo hacia el cero.\n",
                    J1_LIMITE_POS_GRADOS, J1_LIMITE_NEG_GRADOS);
      return true;
    case Corte::Limite:
      Serial.printf("   CORTADO por el límite (%+.0f° / %+.0f°) a los %lu ms. Volver hacia el cero.\n",
                    J1_LIMITE_POS_GRADOS, J1_LIMITE_NEG_GRADOS, (unsigned long)m.ms);
      return true;
    case Corte::Enter:
      Serial.printf("   CORTADO con Enter a los %lu ms.\n", (unsigned long)m.ms);
      return true;
    default:
      return false;
  }
}

void pulso(float pct, uint32_t ms) {
  if (!listaParaMover()) return;
  Serial.printf(">> Pulso %+.0f %% durante %lu ms\n", pct, (unsigned long)ms);
  const Movimiento m = mover(pct, ms, false, 0);
  const int64_t d = m.cFin - m.c0;
  Serial.printf("   %+.2f° -> %+.2f° al cortar el duty (%+lld cuentas, %+.2f°); %+.2f° tras frenar\n", grados(m.c0),
                grados(m.cFin), (long long)d, grados(d), grados(m.c1));
  if (informarCorte(m)) {
    Serial.println("   Resultado no concluyente.");
    return;
  }
  if (llabs(d) < J1_CUENTAS_SIN_GIRO) {
    Serial.println("   Casi no giró: resultado no concluyente. Probar con un poco más de duty o de tiempo.");
  } else if ((d > 0) != (pct > 0)) {
    Serial.println("   Las cuentas van al REVÉS del duty: revisar J1_ENCODER_INVERTIDO en config.h y el cableado.");
  } else {
    Serial.println("   OK: motor y encoder de acuerdo.");
    Serial.printf("   ¿La base giró %s visto desde arriba? Si fue al revés, revisar J1_SENTIDO_INVERTIDO.\n",
                  pct > 0 ? "ANTIHORARIO" : "HORARIO");
  }
}

void irA(float objetivoGrados, float pct) {
  const int64_t c = cuentas();
  const int64_t objetivo = (int64_t)lroundf(objetivoGrados * J1_CUENTAS_POR_GRADO);
  if (llabs(objetivo - c) < J1_CUENTAS_SIN_GIRO) {
    Serial.printf("Ya está en %+.2f°.\n", grados(c));
    return;
  }
  if (!listaParaMover()) return;
  const float duty = objetivo > c ? pct : -pct;
  Serial.printf(">> Ir a %+.1f° desde %+.2f° con %+.0f %%\n", objetivoGrados, grados(c), duty);
  const Movimiento m = mover(duty, J1_MOVER_MAX_MS, true, objetivo);
  if (informarCorte(m)) {
    Serial.printf("   Quedó en %+.2f°.\n", grados(m.c1));
    return;
  }
  if (m.corte == Corte::Tiempo) {
    Serial.printf("   No llegó en %lu ms: quedó en %+.2f°.\n", (unsigned long)J1_MOVER_MAX_MS, grados(m.c1));
    return;
  }
  const float vel = grados(m.cFin - m.c0) / (m.ms / 1000.0f);
  Serial.printf("   Llegó en %.2f s (%.1f °/s). Al frenar quedó en %+.2f° (se pasó %.2f°).\n", m.ms / 1000.0f,
                fabsf(vel), grados(m.c1), fabsf(grados(m.c1 - objetivo)));
  Serial.printf("   cuentas %+lld. Comparar con el transportador.\n", (long long)m.c1);
}

// ---------- Paso 2: PI de velocidad ----------

// Un paso del PI (cada J1_TS_VEL_US). Con velocidad pedida 0 devuelve 0 (freno) y borra la integral.
// Feedforward: suma la zona muerta en el sentido pedido. Anti-windup: no integra si está saturado y el
// error lo empujaría más hacia la saturación.
float pasoPI(float wRef, float wMed, float &integral) {
  if (wRef == 0.0f) {
    integral = 0.0f;
    return 0.0f;
  }
  const float e = wRef - wMed;
  const float ff = wRef > 0 ? J1_ZONA_MUERTA_PCT : -J1_ZONA_MUERTA_PCT;
  const float uLibre = ff + kpVel * e + integral;
  const float u = constrain(uLibre, -J1_DUTY_MAX_PCT, J1_DUTY_MAX_PCT);
  if (u == uLibre || (e > 0) != (uLibre > 0)) integral += kiVel * (J1_TS_VEL_US / 1e6f) * e;
  return u;
}

// Escalones de velocidad con el PI: cada ciclo es +v durante ms, 0 durante J1_PAUSA_MS, -v durante ms y
// 0 durante J1_PAUSA_MS (pasar por cero con pausa evita invertir de golpe contra el juego de la correa).
// Al final, 0 durante 300 ms. Manda una línea de CSV por periodo.
// Protecciones: límite de ángulo, deriva, sin cuentas, sentido contrario, atasco y Enter.
void escalonVelocidad(float v, uint32_t ms, uint32_t ciclos) {
  const int64_t c0 = cuentas();
  const float g0 = grados(c0), fin1 = g0 + v * ms / 1000.0f;  // donde termina la ida
  const float maxG = J1_LIMITE_POS_GRADOS - 5.0f, minG = J1_LIMITE_NEG_GRADOS + 5.0f;
  if (fin1 > maxG || fin1 < minG || g0 > maxG || g0 < minG) {
    Serial.printf("La ida iría de %+.1f° a %+.1f°, fuera de %+.0f° / %+.0f° (límites con margen). "
                  "Volver con \"m 0\" o usar menos v o ms.\n", g0, fin1, maxG, minG);
    return;
  }
  if (!listaParaMover()) return;

  const uint32_t nIda = ms * 1000 / J1_TS_VEL_US, nPausa = J1_PAUSA_MS * 1000 / J1_TS_VEL_US;
  const uint32_t nCiclo = 2 * (nIda + nPausa), nTotal = ciclos * nCiclo + 300000 / J1_TS_VEL_US;
  const float ts = J1_TS_VEL_US / 1e6f;
  int64_t hist[J1_VENTANA_VEL + 1];
  for (auto &h : hist) h = c0;
  float integral = 0.0f;
  uint32_t atascoTicks = 0, contrarioTicks = 0, sinCuentasTicks = 0, perdidos = 0;
  const uint32_t tsMs = J1_TS_VEL_US / 1000;
  const char *corte = nullptr;

  Serial.printf("# escalon_vel v=%.1f ms=%lu pausa_ms=%lu ciclos=%lu kp=%.4f ki=%.4f zm=%.1f ts_ms=%.0f ventana=%d "
                "cuentas_por_grado=%.4f\n", v, (unsigned long)ms, (unsigned long)J1_PAUSA_MS, (unsigned long)ciclos, kpVel,
                kiVel, J1_ZONA_MUERTA_PCT, ts * 1000, J1_VENTANA_VEL, J1_CUENTAS_POR_GRADO);
  Serial.println("t_ms,w_ref,w_med,duty,pos");
  uint32_t tick = micros();
  const uint32_t t0 = tick;
  for (uint32_t k = 0; k < nTotal; k++) {
    while (micros() - tick < J1_TS_VEL_US) {
    }
    if (micros() - tick >= 2 * J1_TS_VEL_US) perdidos++;
    tick += J1_TS_VEL_US;

    const int64_t c = cuentas();
    for (int i = 0; i < J1_VENTANA_VEL; i++) hist[i] = hist[i + 1];
    hist[J1_VENTANA_VEL] = c;
    const float wMed = grados(hist[J1_VENTANA_VEL] - hist[0]) / (J1_VENTANA_VEL * ts);
    float wRef = 0.0f;
    if (k < ciclos * nCiclo) {
      const uint32_t r = k % nCiclo;
      wRef = r < nIda ? v : (r < nIda + nPausa ? 0.0f : (r < 2 * nIda + nPausa ? -v : 0.0f));
      // Deriva: al empezar cada ciclo, la ida tiene que entrar en el rango (con margen)
      if (r == 0 && k > 0) {
        const float fin = grados(c) + v * ms / 1000.0f;
        if (fin > maxG || fin < minG || grados(c) > maxG || grados(c) < minG) {
          corte = "deriva: el próximo ciclo saldría del rango";
          break;
        }
      }
    }
    const float u = pasoPI(wRef, wMed, integral);
    aplicarDuty(u);
    Serial.printf("%lu,%.1f,%.2f,%.1f,%.2f\n", (unsigned long)((tick - t0) / 1000), wRef, wMed, u, grados(c));

    // Protecciones. Fuera del rango corta siempre, vaya hacia donde vaya: si el signo del encoder
    // estuviera mal, "hacia afuera" según la consigna no sería hacia afuera de verdad.
    if (excesoLimite(c) > 0) {
      corte = "límite de ángulo";
      break;
    }
    // Sin ninguna cuenta con el motor empujando más que la zona muerta: encoder suelto o motor trabado
    sinCuentasTicks = (wRef != 0.0f && fabsf(u) > J1_ZONA_MUERTA_PCT && c == hist[J1_VENTANA_VEL - 1])
                          ? sinCuentasTicks + 1 : 0;
    if (sinCuentasTicks * tsMs >= J1_SIN_CUENTAS_MS) {
      corte = "sin cuentas del encoder (¿encoder suelto o motor trabado?)";
      break;
    }
    // Se mueve al revés de lo pedido: signo del encoder o del motor mal (¿se usó "ie" o "is"?)
    contrarioTicks = (wRef != 0.0f && wMed * (wRef > 0 ? 1.0f : -1.0f) < -J1_CONTRARIO_VEL) ? contrarioTicks + 1 : 0;
    if (contrarioTicks * tsMs >= J1_CONTRARIO_MS) {
      corte = "se mueve al revés de lo pedido (¿signos mal?)";
      break;
    }
    atascoTicks = (fabsf(wMed) < J1_ATASCO_VEL && fabsf(u) >= J1_ATASCO_DUTY_PCT) ? atascoTicks + 1 : 0;
    if (atascoTicks * tsMs >= J1_ATASCO_MS) {
      corte = "atasco (duty alto sin velocidad: ¿trabado o encoder suelto?)";
      break;
    }
    if (hayEntrada()) {
      corte = "Enter";
      break;
    }
  }
  aplicarDuty(0.0f);
  delay(FRENO_MS);
  motorJ1.coast();
  if (corte) {
    Serial.printf("# cortado %s\n", corte);
    Serial.printf(">> CORTADO por %s.\n", corte);
  } else {
    Serial.println("# fin ok");
  }
  if (perdidos) Serial.printf(">> Aviso: %lu periodos atrasados.\n", (unsigned long)perdidos);
  Serial.printf(">> Escalón terminado. Base en %+.2f°.\n", grados(cuentas()));
}

void mostrarGanancias() {
  Serial.printf("PI de velocidad: kp = %.4f %%/(°/s), ki = %.4f %%/° | zona muerta %.1f %% | %d muestras de %lu ms\n",
                kpVel, kiVel, J1_ZONA_MUERTA_PCT, J1_VENTANA_VEL, (unsigned long)(J1_TS_VEL_US / 1000));
}

// Lee un número que ocupe toda la cadena (se admiten espacios al final). Rechaza NaN e infinito.
// Después del número tiene que haber un espacio o el fin de la línea ("m 10+30" se rechaza).
bool leerNumero(const char *s, const char **resto, float &v) {
  char *fin = nullptr;
  v = strtof(s, &fin);
  if (fin == s || !isfinite(v) || (*fin != ' ' && *fin != '\0')) return false;
  while (*fin == ' ') fin++;
  *resto = fin;
  return true;
}

// "p <pct> [ms]"
void comandoPulso(const char *args) {
  float pct;
  const char *resto;
  if (!leerNumero(args, &resto, pct) || pct == 0.0f || fabsf(pct) > J1_DUTY_MAX_PCT) {
    Serial.printf("Uso: p <%%> [ms], con 0 < |%%| <= %.0f\n", J1_DUTY_MAX_PCT);
    return;
  }
  uint32_t ms = PULSO_MS_DEF;
  if (*resto != '\0') {
    char *fin2 = nullptr;
    const long v = strtol(resto, &fin2, 10);
    while (*fin2 == ' ') fin2++;
    if (fin2 == resto || *fin2 != '\0' || v <= 0 || v > (long)J1_PULSO_MAX_MS) {
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

// "ev <°/s> [ms] [ciclos]"
void comandoEscalon(const char *args) {
  float v;
  const char *resto;
  if (!leerNumero(args, &resto, v) || fabsf(v) < J1_VEL_MIN || fabsf(v) > J1_VEL_MAX) {
    Serial.printf("Uso: ev <°/s> [ms], con %.0f <= |°/s| <= %.0f\n", J1_VEL_MIN, J1_VEL_MAX);
    return;
  }
  // Enteros opcionales: ms y ciclos, separados por espacios
  long enteros[2] = {1000, 1};
  for (int i = 0; i < 2 && *resto != '\0'; i++) {
    char *fin2 = nullptr;
    enteros[i] = strtol(resto, &fin2, 10);
    if (fin2 == resto || (*fin2 != ' ' && *fin2 != '\0')) {
      Serial.println("Uso: ev <°/s> [ms] [ciclos]");
      return;
    }
    while (*fin2 == ' ') fin2++;
    resto = fin2;
  }
  if (*resto != '\0') {
    Serial.println("Uso: ev <°/s> [ms] [ciclos]");
    return;
  }
  if (enteros[0] < 200 || enteros[0] > (long)J1_ESCALON_MAX_MS) {
    Serial.printf("Tiempo inválido: entre 200 y %lu ms\n", (unsigned long)J1_ESCALON_MAX_MS);
    return;
  }
  if (enteros[1] < 1 || enteros[1] > (long)J1_CICLOS_MAX) {
    Serial.printf("Ciclos inválidos: entre 1 y %lu\n", (unsigned long)J1_CICLOS_MAX);
    return;
  }
  if (!ceroFijado) {
    Serial.println("Primero llevar la base a su marca y escribir \"z\".");
    return;
  }
  escalonVelocidad(v, (uint32_t)enteros[0], (uint32_t)enteros[1]);
}

// "kp <x>" / "ki <x>"
void comandoGanancia(const char *args, float &g, const char *nombre) {
  float x;
  const char *resto;
  if (!leerNumero(args, &resto, x) || *resto != '\0' || x < 0.0f || x > 50.0f) {
    Serial.printf("Uso: %s <valor>, entre 0 y 50\n", nombre);
    return;
  }
  g = x;
  mostrarGanancias();
}

// "m <grados> [pct]"
void comandoMover(const char *args) {
  float g;
  const char *resto;
  const float maxG = J1_LIMITE_POS_GRADOS - 3.0f, minG = J1_LIMITE_NEG_GRADOS + 3.0f;  // margen de frenado
  if (!leerNumero(args, &resto, g) || g > maxG || g < minG) {
    Serial.printf("Uso: m <grados> [%%], con %+.0f <= grados <= %+.0f\n", minG, maxG);
    return;
  }
  float pct = J1_DUTY_MOVER_PCT;
  if (*resto != '\0') {
    const char *fin;
    if (!leerNumero(resto, &fin, pct) || *fin != '\0' || pct <= 0.0f || pct > J1_DUTY_MAX_PCT) {
      Serial.printf("Duty inválido: entre 0 y %.0f %%\n", J1_DUTY_MAX_PCT);
      return;
    }
  }
  if (!ceroFijado) {
    Serial.println("Primero llevar la base a su marca (al frente) y escribir \"z\".");
    return;
  }
  irA(g, pct);
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
    mostrarPosicion();
  } else if (strcmp(l, "z") == 0) {
    encJ1.reset();
    ceroFijado = true;
    Serial.println(">> Cero fijado en la marca. Movimientos habilitados.");
  } else if (strcmp(l, "ie") == 0 || strcmp(l, "is") == 0) {
    // Deshabilitados: con límites asimétricos (+135° / -45°), invertir el sentido daría vuelta los
    // límites físicos sin que el firmware lo note. Los signos ya quedaron fijos en config.h (paso 0).
    mostrarSignos();
    Serial.println("   Deshabilitado: los signos están fijos en include/config.h desde el paso 0.");
  } else if (l[0] == 'p' && (l[1] == ' ' || l[1] == '\0')) {
    comandoPulso(l + 1);
  } else if (l[0] == 'm' && (l[1] == ' ' || l[1] == '\0')) {
    comandoMover(l + 1);
  } else if (strncmp(l, "ev", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoEscalon(l + 2);
  } else if (strncmp(l, "kp", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoGanancia(l + 2, kpVel, "kp");
  } else if (strncmp(l, "ki", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoGanancia(l + 2, kiVel, "ki");
  } else if (strcmp(l, "g") == 0) {
    mostrarGanancias();
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
    if (descartando) {  // cola de la línea que cortó un movimiento: se tira hasta su Enter
      if (ch == '\r' || ch == '\n') descartando = false;
      continue;
    }
    // Flechas y otras teclas especiales (ESC [ ... letra): se ignoran
    if (escape == 1) {
      escape = (ch == '[') ? 2 : 0;
      continue;
    }
    if (escape == 2) {
      if (ch >= 0x40 && ch <= 0x7E) escape = 0;  // letra final de la secuencia
      continue;
    }
    if (ch == 0x1B) {
      escape = 1;
      continue;
    }
    // Tecla de borrar: saca el último carácter (y lo borra de la pantalla del monitor)
    if (ch == 0x08 || ch == 0x7F) {
      if (lineaLen > 0 && !lineaLarga) {
        lineaLen--;
        Serial.print("\b \b");
      }
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
  Serial.println("== 10 · J1: pasos 0-1 (signo, transmisión) y 2 (PI de velocidad) · 36GP-555 + IBT-2, WROOM ==");
  Serial.printf("IBT-2: RPWM=GPIO%d, LPWM=GPIO%d, EN=GPIO%d | PWM %lu Hz\n", PIN_J1_RPWM, PIN_J1_LPWM, PIN_J1_EN,
                (unsigned long)J1_PWM_FREQ_HZ);
  Serial.printf("Encoder: A=GPIO%d, B=GPIO%d | reductora 50:1 x correa %.0f:1 = %.2f cuentas por grado de la base\n",
                PIN_J1_ENC_A, PIN_J1_ENC_B, J1_RELACION_CORREA, J1_CUENTAS_POR_GRADO);
  mostrarSignos();
  mostrarGanancias();
  if (!okDriver) Serial.println("ERROR: no se pudo configurar el PWM (LEDC).");
  if (!okEnc) Serial.println("ERROR: no se pudo configurar el PCNT del encoder.");
  Serial.printf("Niveles del encoder en reposo: A=%d B=%d (con los pull-up, quieto puede ser 0 o 1)\n",
                encJ1.levelA(), encJ1.levelB());
  Serial.println("Motor en rueda libre. Movimientos deshabilitados hasta \"z\" con la base en su marca.");
  ayuda();
}

void loop() { leerSerial(); }
