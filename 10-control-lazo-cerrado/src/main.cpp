// Lazo cerrado de una articulación del brazo en la ESP32-WROOM-32D: J1 (36GP-555 + IBT-2, base giratoria,
// -e j1), J2 (5840-31ZY + L298N, hombro, -e j2) o J3 (5840-31ZY + IBT-2, codo, -e j3). Los parámetros de cada una están en include/config.h
// (espacio de nombres "art"). Pasos:
//  - Paso 0: pulsos cortos de duty fijo para comprobar que "positivo" es lo mismo para el motor,
//    el encoder y la articulación (J1: antihorario visto desde arriba; J2: el brazo sube). Ver config.h.
//  - Paso 1: "m <grados>" lleva la articulación a un ángulo con duty fijo y frena al llegar, para verificar
//    las cuentas por grado (J1: 50:1 x correa 90/18 = 44,44; J2: falta medir el engranaje).
//  - Paso 2: PI de velocidad a 100 Hz con compensación de la zona muerta. "ev <°/s> [ms] [ciclos]" repite
//    +v, pausa, -v, pausa y manda un CSV que tools/escalon_velocidad.py compara con la simulación.
//  - Paso 3: P de posición a 50 Hz encima del PI de velocidad (cascada). "a <grados>" va a un ángulo;
//    "ep <A> [ms] [ciclos]" hace escalones 0, +A, 0, -A con CSV para tools/escalon_posicion.py.
#include <Arduino.h>

#if defined(ARTICULACION_L298N)
#include "L298N.h"
#else
#include "BTS7960.h"
#endif
#include "EncoderPCNT.h"
#include "config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_R = 0;
constexpr uint8_t LEDC_CH_L = 1;
constexpr uint32_t FRENO_MS = 300;        // freno después de mover, antes de dejarlo en rueda libre
constexpr uint32_t MONITOR_CADA_MS = 200;
constexpr uint32_t QUIETO_MS = 100;       // antes de mover, la articulación tiene que estar quieta este tiempo
constexpr float VEL_QUIETO = 1.0f;        // °/s: por debajo, el feedforward es el de despegue (J3: 1 cuenta en 40 ms = 0,72)
constexpr float RAMPA_DESPEGUE = 300.0f;  // %/s: del feedforward de marcha al de despegue (3 % cada 10 ms)
constexpr int64_t LIM_POS_C = (int64_t)(art::LIMITE_POS_GRADOS * art::CUENTAS_POR_GRADO);
constexpr int64_t LIM_NEG_C = (int64_t)(art::LIMITE_NEG_GRADOS * art::CUENTAS_POR_GRADO);

#if defined(ARTICULACION_L298N)
L298N motor;
#else
BTS7960 motor;
#endif
EncoderPCNT enc;
bool hwOk = false;
bool ceroFijado = false;  // sin "z" no se mueve: el cero tiene que estar en la marca, lejos de los topes

// Signos: arrancan con los de config.h (paso 0). "ie" e "is" los cambian solo si art::SIGNOS_EN_VIVO
bool encInvertido = art::ENCODER_INVERTIDO;
bool sentidoInvertido = art::SENTIDO_INVERTIDO;

// Ganancias del PI de velocidad: arrancan con las de config.h y se cambian con "kp" y "ki"
float kpVel = art::KP_VEL;
float kiVel = art::KI_VEL;
float kpp = art::KPP;  // P de posición, se cambia con "kpp"
// Feedforward de fricción (despegue y marcha, en cada sentido): arranca con el de config.h y se cambia con "ff"
float ffDespPos = art::DESPEGUE_PCT, ffMarchaPos = art::ZONA_MUERTA_PCT;
float ffDespNeg = art::DESPEGUE_NEG_PCT, ffMarchaNeg = art::ZONA_MUERTA_NEG_PCT;
float ffPendPos = art::FF_PENDIENTE_POS;  // % por grado, subiendo (compensación de gravedad de J3)
constexpr float FF_MIN_PCT = 8.0f;        // piso del feedforward subiendo (abajo del rango la recta daría poco)
// Ajustes contra el traba-suelta al bajar ("bj"): velocidad mínima de llegada bajando, cuánto tiempo trabada antes
// de volver a la rampa de despegue, y vibración (dither) en el duty mientras se mueve
float vminBaja = art::VMIN_BAJA_POS;
uint32_t trabadaMs = art::TRABADA_MS;
float ditherPct = art::DITHER_PCT;

void irAControlado(float g, bool informar);  // paso 3, la usa también "ev" al terminar

char linea[48];
size_t lineaLen = 0;
bool lineaLarga = false;   // se pasó del buffer: se descarta entera al llegar el Enter
bool ultimoFueCR = false;  // CR+LF cuenta como un solo Enter
bool descartando = false;  // una línea cortó un movimiento o el monitor: el resto de esa línea no se ejecuta
uint8_t escape = 0;        // secuencia de escape de la terminal (flechas): 1 = llegó ESC, 2 = dentro de ESC [

// Cuentas de la articulación con los signos aplicados
int64_t cuentas() {
  int64_t c = enc.count();
  if (encInvertido) c = -c;
  if (sentidoInvertido) c = -c;
  return c;
}

float grados(int64_t c) { return (float)c / art::CUENTAS_POR_GRADO; }

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
  motor.setDuty(d);
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
  Serial.println("  e            ver el encoder (cuentas, grados, niveles A/B); Enter para salir");
  Serial.printf("  z            poner a cero en la marca (%s). Habilita los movimientos\n", art::MARCA);
  Serial.printf("  m <°> [%%]    ir a un ángulo con duty fijo (por defecto %.0f %%) y frenar. Ej: m 10, m -30\n",
                art::DUTY_MOVER_PCT);
  Serial.printf("  p <%%> [ms]   pulso de duty fijo, |%%| <= %.0f, por defecto %lu ms (máx %lu). Ej: p 30 100\n",
                art::DUTY_MAX_PCT, (unsigned long)art::PULSO_DEF_MS, (unsigned long)art::PULSO_MAX_MS);
  Serial.printf("  ev <°/s> [ms] [ciclos]  escalones de velocidad: +v ms, 0 %lu ms, -v ms, 0 %lu ms, repetido (CSV).\n"
                "               Ej: ev 20 1000 40 (2 min)\n", (unsigned long)art::PAUSA_MS, (unsigned long)art::PAUSA_MS);
  Serial.println("  kp <x>, ki <x>  cambiar las ganancias del PI de velocidad · g  mostrarlas");
  Serial.println("  ff <desp+> <marcha+> <desp-> <marcha-> [%/°]  feedforward en %: quieta / en marcha, subiendo / bajando,");
  Serial.println("               en 0°; [%/°] = cuánto crece subiendo por grado (compensación de gravedad)");
  Serial.println("  bc <desde> <hasta> [°/s]  barrido lento a velocidad constante (CSV): duty según el ángulo");
  Serial.println("  bj <vmin bajando °/s> <trabada ms> <dither %>  ajustes contra el traba-suelta al bajar");
  Serial.println("  a <°>        ir a un ángulo con el control de posición (cascada). Ej: a 30, a 0");
  Serial.println("  ep <A> [ms] [ciclos]  escalones de posición: A, 0, -A, 0 desde donde está (CSV). Ej: ep 20 2500 4");
  Serial.println("  kpp <x>      cambiar el P de posición");
  Serial.println("  c            mostrar cuentas y grados");
  Serial.println("  x            rueda libre (driver deshabilitado)");
#if defined(ARTICULACION_L298N)
  Serial.println("  dg           diagnóstico del cableado al L298N: un pin en alto por vez (el motor no se mueve)");
#elif defined(ARTICULACION_J3)
  Serial.println("  dg           diagnóstico del cableado al IBT-2: EN, RPWM y LPWM de a uno, y la salida M+/M-");
#endif
  Serial.println("  ?            esta ayuda");
  Serial.printf("Seguridad: corte fuera de %+.0f° / %+.0f° del cero, si avanza menos de %ld cuentas\n",
                art::LIMITE_POS_GRADOS, art::LIMITE_NEG_GRADOS, (long)art::AVANCE_MIN);
  Serial.printf("en %lu ms o con cualquier Enter. Duty máximo %.0f %%.\n", (unsigned long)art::SIN_CUENTAS_MS,
                art::DUTY_MAX_PCT);
  Serial.println();
}

void mostrarSignos() {
  Serial.printf("Signos: encoder %s, sentido %s\n", encInvertido ? "INVERTIDO" : "normal",
                sentidoInvertido ? "INVERTIDO" : "normal");
}

void mostrarPosicion() {
  const int64_t c = cuentas();
  Serial.printf("cuentas %+lld | %s %+.2f°\n", (long long)c, art::PIEZA, grados(c));
}

void monitorEncoder() {
  motor.coast();
  Serial.println(">> Girá a mano el eje del encoder (con los 12 V apagados). Enter para salir.");
  int64_t ultima = INT64_MIN;
  uint32_t t = 0;
  while (!hayEntrada()) {
    if (millis() - t < MONITOR_CADA_MS) continue;
    t = millis();
    const int64_t c = cuentas();
    if (c == ultima) continue;  // solo cuando cambia, para no llenar la pantalla
    ultima = c;
    Serial.printf("   cuentas %+6lld | %s %+7.2f° | A=%d B=%d\n", (long long)c, art::PIEZA, grados(c), enc.levelA(),
                  enc.levelB());
  }
  Serial.println(">> Fin del monitor.");
}

// Espera QUIETO_MS sin que la articulación se mueva (un movimiento anterior o la mano pueden dejarla girando).
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
      Serial.println("Todavía se mueve: esperar a que se detenga y repetir.");
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
  // Ventana de avance: cada art::SIN_CUENTAS_MS la articulación tiene que haber avanzado art::AVANCE_MIN cuentas
  // en el sentido pedido. Cuentas que van y vuelven (una fase del encoder suelta) no alcanzan.
  uint32_t tVentana = t0;
  int64_t cVentana = m.c0;
  // Y además, ninguna cuenta durante art::SIN_CUENTAS_MS (encoder desconectado del todo) corta enseguida
  uint32_t tUltimaCuenta = t0;
  int64_t cAnt = m.c0;
  // Empujón de arranque (art::KICK_PCT): más duty hasta que despega, después el pedido
  bool empujon = art::KICK_PCT > fabsf(pct) && (dir > 0 || !art::KICK_SOLO_POSITIVO);
  aplicarDuty(empujon ? dir * art::KICK_PCT : pct);
  while (millis() - t0 < msMax) {
    const int64_t c = cuentas();
    const uint32_t ahora = millis();
    if (empujon && (llabs(c - m.c0) >= art::KICK_CUENTAS || ahora - t0 >= art::KICK_MS)) {
      empujon = false;
      aplicarDuty(pct);
    }
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
    if (ahora - tUltimaCuenta >= art::SIN_CUENTAS_MS) {
      m.corte = Corte::SinAvance;
      break;
    }
    if (ahora - tVentana >= art::SIN_CUENTAS_MS) {
      if ((c - cVentana) * dir < art::AVANCE_MIN) {
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
  motor.coast();
  m.c1 = cuentas();
  return m;
}

// Explica un corte por protección. Devuelve true si hubo uno.
bool informarCorte(const Movimiento &m) {
  switch (m.corte) {
    case Corte::SinAvance:
      Serial.printf("   CORTADO: avanzó menos de %ld cuentas en %lu ms (a los %lu ms).\n", (long)art::AVANCE_MIN,
                    (unsigned long)art::SIN_CUENTAS_MS, (unsigned long)m.ms);
      Serial.println("   O el motor no arrancó (¿12 V? ¿duty en la zona muerta?), está trabado o el encoder no cuenta bien.");
      Serial.println("   Revisar el encoder con \"e\" (12 V apagados) antes de subir el duty.");
      return true;
    case Corte::HaciaAfuera:
      Serial.printf("   RECHAZADO: está en el límite (%+.0f° / %+.0f°) y eso la alejaría más. Solo hacia el cero.\n",
                    art::LIMITE_POS_GRADOS, art::LIMITE_NEG_GRADOS);
      return true;
    case Corte::Limite:
      Serial.printf("   CORTADO por el límite (%+.0f° / %+.0f°) a los %lu ms. Volver hacia el cero.\n",
                    art::LIMITE_POS_GRADOS, art::LIMITE_NEG_GRADOS, (unsigned long)m.ms);
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
  if (llabs(d) < art::CUENTAS_SIN_GIRO) {
    Serial.println("   Casi no giró: resultado no concluyente. Probar con un poco más de duty o de tiempo.");
  } else if ((d > 0) != (pct > 0)) {
    Serial.println("   Las cuentas van al REVÉS del duty: \"ie\" o ENCODER_INVERTIDO en config.h (o el cableado).");
  } else {
    Serial.println("   OK: motor y encoder de acuerdo.");
    Serial.printf("   ¿%s %s fue %s? Si fue al revés, \"is\" o SENTIDO_INVERTIDO en config.h.\n", art::NOMBRE,
                  art::PIEZA, pct > 0 ? art::SENTIDO_POS : art::SENTIDO_NEG);
  }
}

void irA(float objetivoGrados, float pct) {
  const int64_t c = cuentas();
  const int64_t objetivo = (int64_t)lroundf(objetivoGrados * art::CUENTAS_POR_GRADO);
  if (llabs(objetivo - c) < art::CUENTAS_SIN_GIRO) {
    Serial.printf("Ya está en %+.2f°.\n", grados(c));
    return;
  }
  if (!listaParaMover()) return;
  const float duty = objetivo > c ? pct : -pct;
  Serial.printf(">> Ir a %+.1f° desde %+.2f° con %+.0f %%\n", objetivoGrados, grados(c), duty);
  const Movimiento m = mover(duty, art::MOVER_MAX_MS, true, objetivo);
  if (informarCorte(m)) {
    Serial.printf("   Quedó en %+.2f°.\n", grados(m.c1));
    return;
  }
  if (m.corte == Corte::Tiempo) {
    Serial.printf("   No llegó en %lu ms: quedó en %+.2f°.\n", (unsigned long)art::MOVER_MAX_MS, grados(m.c1));
    return;
  }
  const float vel = grados(m.cFin - m.c0) / (m.ms / 1000.0f);
  Serial.printf("   Llegó en %.2f s (%.1f °/s). Al frenar quedó en %+.2f° (se pasó %.2f°).\n", m.ms / 1000.0f,
                fabsf(vel), grados(m.c1), fabsf(grados(m.c1 - objetivo)));
  Serial.printf("   cuentas %+lld. Comparar con el transportador.\n", (long long)m.c1);
}

// ---------- Paso 2: PI de velocidad ----------

// Un paso del PI (cada art::TS_VEL_US). Con velocidad pedida 0 devuelve 0 (freno) y borra la integral.
// Feedforward: suma la zona muerta en el sentido pedido. Anti-windup: no integra si está saturado y el
// error lo empujaría más hacia la saturación.
// Estado del despegue (ver pasoPI): se reinicia cada vez que la velocidad pedida vuelve a 0 o cambia de signo
float ffRampa = 0.0f;      // feedforward extra que va creciendo mientras la articulación no despega
bool despegada = false;    // ya se movió en este tramo: feedforward de marcha
uint32_t trabadaTicks = 0; // en marcha pero quieta: si dura, vuelve a la rampa de despegue
int signoAnterior = 0;

float pasoPI(float wRef, float wMed, float th, float &integral) {
  const int signo = wRef > 0 ? 1 : (wRef < 0 ? -1 : 0);
  if (signo != signoAnterior) {
    ffRampa = 0.0f;
    despegada = false;
    trabadaTicks = 0;
    signoAnterior = signo;
  }
  if (wRef == 0.0f) {
    integral = 0.0f;
    return 0.0f;
  }
  const float e = wRef - wMed;
  // Feedforward de fricción: quieta hace falta más duty para despegar que para mantenerla en marcha (J3 además
  // es asimétrico por la gravedad). En J1 y J2 despegue = marcha. Para que sea suave:
  //  - El despegue no es un salto: desde el valor de marcha sube en rampa (RAMPA_DESPEGUE) hasta el de despegue, y
  //    la articulación arranca con el duty justo (un salto de golpe la hacía dispararse al soltarse).
  //  - Una vez que se movió, queda el de marcha hasta el fin del tramo (antes alternaba entre los dos cerca de la
  //    llegada, con agujas en el duty). Si se traba más de trabadaMs, vuelve a la rampa.
  //  - Dither (ditherPct): una onda cuadrada de ±ditherPct a 25 Hz sumada al duty mientras se mueve; la vibración
  //    ayuda a que el sin fin no se quede agarrado (fricción estática) a velocidad baja.
  const bool quieta = fabsf(wMed) < VEL_QUIETO;
  if (despegada) {
    trabadaTicks = quieta ? trabadaTicks + 1 : 0;
    if (trabadaTicks * art::TS_VEL_US >= trabadaMs * 1000u) {
      despegada = false;
      ffRampa = 0.0f;
    }
  } else if (!quieta) {
    despegada = true;
    trabadaTicks = 0;
  }
  // Compensación de gravedad (J3): subiendo, el esfuerzo crece con el ángulo (barrido de carga). El de marcha y
  // el de despegue suben ffPendPos % por grado desde su valor en 0° (con un piso de FF_MIN_PCT).
  const float extra = signo > 0 ? ffPendPos * th : 0.0f;
  const float marcha = fmaxf((signo > 0 ? ffMarchaPos : ffMarchaNeg) + extra, signo > 0 ? FF_MIN_PCT : 0.0f);
  const float desp = fmaxf((signo > 0 ? ffDespPos : ffDespNeg) + extra, marcha);
  if (!despegada) ffRampa = fminf(ffRampa + RAMPA_DESPEGUE * (art::TS_VEL_US / 1e6f), fmaxf(desp - marcha, 0.0f));
  const float ff = signo * (marcha + (despegada ? 0.0f : ffRampa));
  static uint32_t tickDither = 0;
  const float dither = ditherPct > 0.0f ? (((tickDither++ / 2) % 2) ? ditherPct : -ditherPct) : 0.0f;
  const float uLibre = ff + kpVel * e + integral + dither;
  const float u = constrain(uLibre, -art::DUTY_MAX_PCT, art::DUTY_MAX_PCT);
  if (u == uLibre || (e > 0) != (uLibre > 0)) integral += kiVel * (art::TS_VEL_US / 1e6f) * e;
  return u;
}

// Escalones de velocidad con el PI: cada ciclo es +v durante ms, 0 durante art::PAUSA_MS, -v durante ms y
// 0 durante art::PAUSA_MS (pasar por cero con pausa evita invertir de golpe contra el juego de la correa).
// Al final, 0 durante 300 ms. Manda una línea de CSV por periodo.
// Protecciones: límite de ángulo, deriva, sin cuentas, sentido contrario, atasco y Enter.
void escalonVelocidad(float v, uint32_t ms, uint32_t ciclos) {
  const int64_t c0 = cuentas();
  const float g0 = grados(c0), fin1 = g0 + v * ms / 1000.0f;  // donde termina la ida
  const float maxG = art::LIMITE_POS_GRADOS - 5.0f, minG = art::LIMITE_NEG_GRADOS + 5.0f;
  if (fin1 > maxG || fin1 < minG || g0 > maxG || g0 < minG) {
    Serial.printf("La ida iría de %+.1f° a %+.1f°, fuera de %+.0f° / %+.0f° (límites con margen). "
                  "Volver con \"m 0\" o usar menos v o ms.\n", g0, fin1, maxG, minG);
    return;
  }
  if (!listaParaMover()) return;

  const uint32_t nIda = ms * 1000 / art::TS_VEL_US, nPausa = art::PAUSA_MS * 1000 / art::TS_VEL_US;
  const uint32_t nCiclo = 2 * (nIda + nPausa), nTotal = ciclos * nCiclo + 300000 / art::TS_VEL_US;
  const float ts = art::TS_VEL_US / 1e6f;
  int64_t hist[art::VENTANA_VEL + 1];
  for (auto &h : hist) h = c0;
  float integral = 0.0f;
  uint32_t atascoTicks = 0, contrarioTicks = 0, sinCuentasTicks = 0, perdidos = 0;
  const uint32_t tsMs = art::TS_VEL_US / 1000;
  const char *corte = nullptr;

  Serial.printf("# escalon_vel v=%.1f ms=%lu pausa_ms=%lu ciclos=%lu kp=%.4f ki=%.4f zm=%.1f zm_neg=%.1f desp=%.1f desp_neg=%.1f pend=%.2f ts_ms=%.0f "
                "ventana=%d cuentas_por_grado=%.4f\n", v, (unsigned long)ms, (unsigned long)art::PAUSA_MS, (unsigned long)ciclos, kpVel,
                kiVel, ffMarchaPos, ffMarchaNeg, ffDespPos, ffDespNeg, ffPendPos, ts * 1000, art::VENTANA_VEL,
                art::CUENTAS_POR_GRADO);
  Serial.println("t_ms,w_ref,w_med,duty,pos");
  uint32_t tick = micros();
  const uint32_t t0 = tick;
  for (uint32_t k = 0; k < nTotal; k++) {
    while (micros() - tick < art::TS_VEL_US) {
    }
    if (micros() - tick >= 2 * art::TS_VEL_US) perdidos++;
    tick += art::TS_VEL_US;

    const int64_t c = cuentas();
    for (int i = 0; i < art::VENTANA_VEL; i++) hist[i] = hist[i + 1];
    hist[art::VENTANA_VEL] = c;
    const float wMed = grados(hist[art::VENTANA_VEL] - hist[0]) / (art::VENTANA_VEL * ts);
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
    const float u = pasoPI(wRef, wMed, grados(c), integral);
    aplicarDuty(u);
    Serial.printf("%lu,%.1f,%.2f,%.1f,%.2f\n", (unsigned long)((tick - t0) / 1000), wRef, wMed, u, grados(c));

    // Protecciones. Fuera del rango corta siempre, vaya hacia donde vaya: si el signo del encoder
    // estuviera mal, "hacia afuera" según la consigna no sería hacia afuera de verdad.
    if (excesoLimite(c) > 0) {
      corte = "límite de ángulo";
      break;
    }
    // Sin ninguna cuenta con el motor empujando más que la zona muerta: encoder suelto o motor trabado
    // (con menos de art::ARRANQUE_PCT el motor no la mueve aunque el encoder esté suelto: no hay peligro, y
    // cerca del objetivo es normal quedar un instante trabada por la fricción)
    sinCuentasTicks = (wRef != 0.0f && fabsf(u) >= art::ARRANQUE_PCT && c == hist[art::VENTANA_VEL - 1])
                          ? sinCuentasTicks + 1 : 0;
    if (sinCuentasTicks * tsMs >= art::SIN_CUENTAS_MS) {
      corte = "sin cuentas del encoder (¿encoder suelto o motor trabado?)";
      break;
    }
    // Se mueve al revés de lo pedido: signo del encoder o del motor mal (¿se usó "ie" o "is"?)
    contrarioTicks = (wRef != 0.0f && wMed * (wRef > 0 ? 1.0f : -1.0f) < -art::CONTRARIO_VEL) ? contrarioTicks + 1 : 0;
    if (contrarioTicks * tsMs >= art::CONTRARIO_MS) {
      corte = "se mueve al revés de lo pedido (¿signos mal?)";
      break;
    }
    atascoTicks = (fabsf(wMed) < art::ATASCO_VEL && fabsf(u) >= art::ATASCO_DUTY_PCT) ? atascoTicks + 1 : 0;
    if (atascoTicks * tsMs >= art::ATASCO_MS) {
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
  motor.coast();
  if (corte) {
    Serial.printf("# cortado %s\n", corte);
    Serial.printf(">> CORTADO por %s.\n", corte);
  } else {
    Serial.println("# fin ok");
  }
  if (perdidos) Serial.printf(">> Aviso: %lu periodos atrasados.\n", (unsigned long)perdidos);
  // Volver al ángulo de inicio, así la deriva no se acumula entre pruebas. Solo si terminó bien: si se cortó
  // (Enter, Ctrl+C del script o una protección), frenar significa quedarse quieta.
  if (!corte) {
    Serial.printf(">> Volviendo a %+.2f°...\n", g0);
    irAControlado(g0, true);
  }
  Serial.printf(">> Escalón terminado. En %+.2f°.\n", grados(cuentas()));
}

void mostrarGanancias() {
  Serial.printf("PI de velocidad: kp = %.4f %%/(°/s), ki = %.4f %%/° | %d muestras de %lu ms\n", kpVel, kiVel,
                art::VENTANA_VEL, (unsigned long)(art::TS_VEL_US / 1000));
  Serial.printf("Feedforward (ff): despegue %+.1f / -%.1f %%, marcha %+.1f / -%.1f %% (en 0°), subiendo +%.2f %%/°\n",
                ffDespPos, ffDespNeg, ffMarchaPos, ffMarchaNeg, ffPendPos);
  Serial.printf("Bajada (bj): vmin %.1f °/s, trabada %lu ms, dither ±%.1f %%\n", vminBaja, (unsigned long)trabadaMs,
                ditherPct);
  Serial.printf("P de posición: kpp = %.3f 1/s | v_max %.0f °/s, a_max %.0f °/s², llega a ±%.2f°, se despierta a "
                "±%.2f° | %lu ms\n", kpp, art::VMAX_POS, art::AMAX_POS, art::TOL_POS, art::TOL_SALIDA,
                (unsigned long)(art::TS_POS_US / 1000));
}

// ---------- Paso 3: P de posición (cascada) ----------

// Ángulo pedido en cada periodo del lazo de velocidad (k = número de periodo)
typedef float (*Referencia)(uint32_t k);

float refFija = 0.0f;  // "a <grados>"
float refConstante(uint32_t) { return refFija; }

// "ep": tramos de nSeg periodos: base + A, base, base - A, base, repetido; después, base
float epBase = 0.0f, epAmp = 0.0f;
uint32_t epNSeg = 1, epCiclos = 1;
float refEscalones(uint32_t k) {
  const uint32_t seg = k / epNSeg;
  if (seg >= 4 * epCiclos) return epBase;
  const uint32_t r = seg % 4;
  return r == 0 ? epBase + epAmp : (r == 2 ? epBase - epAmp : epBase);
}

// Lazo en cascada durante como mucho nTotal periodos de 10 ms. Cada art::TS_POS_US, el P de posición
// calcula la velocidad pedida (limitada a v_max, con su cambio limitado a a_max, y 0 dentro de la
// tolerancia); cada 10 ms, el PI de velocidad la cumple. Con salirAlLlegar, termina cuando está dentro de
// la tolerancia y quieta durante art::LLEGADA_MS. Con csv, manda una línea por periodo.
// Devuelve nullptr si terminó bien, o la causa del corte. Al salir, frena y deja el driver apagado.
const char *cascada(Referencia ref, uint32_t nTotal, bool salirAlLlegar, bool csv, uint32_t &msUsados) {
  const float ts = art::TS_VEL_US / 1e6f;
  const uint32_t nPos = art::TS_POS_US / art::TS_VEL_US, tsMs = art::TS_VEL_US / 1000;
  const uint32_t nLlegada = art::LLEGADA_MS / tsMs;
  int64_t hist[art::VENTANA_VEL + 1];
  const int64_t c0 = cuentas();
  for (auto &h : hist) h = c0;
  float integral = 0.0f, wRef = 0.0f, e = 0.0f, thRefAnt = NAN;
  bool llegado = false;  // histéresis: llega a menos de art::TOL_POS, se despierta a más de art::TOL_SALIDA
  uint32_t atascoTicks = 0, contrarioTicks = 0, sinCuentasTicks = 0, quietoTicks = 0;
  const char *corte = salirAlLlegar ? "no llegó a tiempo" : nullptr;
  uint32_t tick = micros();
  const uint32_t t0 = tick;
  for (uint32_t k = 0; k < nTotal; k++) {
    while (micros() - tick < art::TS_VEL_US) {
    }
    tick += art::TS_VEL_US;
    const int64_t c = cuentas();
    for (int i = 0; i < art::VENTANA_VEL; i++) hist[i] = hist[i + 1];
    hist[art::VENTANA_VEL] = c;
    const float wMed = grados(hist[art::VENTANA_VEL] - hist[0]) / (art::VENTANA_VEL * ts);
    const float thRef = ref(k);

    // Lazo de posición
    if (k % nPos == 0) {
      e = thRef - grados(c);
      if (thRef != thRefAnt) {  // ángulo pedido nuevo: hay que ir
        llegado = false;
        thRefAnt = thRef;
      }
      if (llegado) {
        if (fabsf(e) > art::TOL_SALIDA) llegado = false;  // la alejaron (o se fue): volver
      } else if (fabsf(e) < art::TOL_POS) {
        llegado = true;
      }
      // Mientras no llegó, al menos art::VMIN_POS hacia el objetivo: más despacio la fricción la traba antes
      const float vP = fmaxf(fabsf(kpp * e), e < 0 ? vminBaja : art::VMIN_POS);
      const float deseada = llegado ? 0.0f : (e > 0 ? 1.0f : -1.0f) * fminf(vP, art::VMAX_POS);
      const float dMax = art::AMAX_POS * art::TS_POS_US / 1e6f;
      wRef = deseada == 0.0f ? 0.0f : constrain(deseada, wRef - dMax, wRef + dMax);
    }
    // Lazo de velocidad
    const float u = pasoPI(wRef, wMed, grados(c), integral);
    aplicarDuty(u);
    if (csv) {
      Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.1f\n", (unsigned long)((tick - t0) / 1000), thRef, grados(c), wRef, wMed,
                    u);
    }

    // Protecciones (las mismas que "ev")
    if (excesoLimite(c) > 0) {
      corte = "límite de ángulo";
      break;
    }
    // (con menos de art::ARRANQUE_PCT el motor no la mueve aunque el encoder esté suelto: no hay peligro, y
    // cerca del objetivo es normal quedar un instante trabada por la fricción)
    sinCuentasTicks = (wRef != 0.0f && fabsf(u) >= art::ARRANQUE_PCT && c == hist[art::VENTANA_VEL - 1])
                          ? sinCuentasTicks + 1 : 0;
    if (sinCuentasTicks * tsMs >= art::SIN_CUENTAS_MS) {
      corte = "sin cuentas del encoder (¿encoder suelto o motor trabado?)";
      break;
    }
    // Al revés: con velocidad pedida chica (cerca del objetivo) la medición puede ir y venir; se exige
    // ir al revés a más de art::CONTRARIO_VEL
    contrarioTicks = (wRef != 0.0f && wMed * (wRef > 0 ? 1.0f : -1.0f) < -art::CONTRARIO_VEL) ? contrarioTicks + 1 : 0;
    if (contrarioTicks * tsMs >= art::CONTRARIO_MS) {
      corte = "se mueve al revés de lo pedido (¿signos mal?)";
      break;
    }
    atascoTicks = (fabsf(wMed) < art::ATASCO_VEL && fabsf(u) >= art::ATASCO_DUTY_PCT) ? atascoTicks + 1 : 0;
    if (atascoTicks * tsMs >= art::ATASCO_MS) {
      corte = "atasco (duty alto sin velocidad: ¿trabado o encoder suelto?)";
      break;
    }
    if (hayEntrada()) {
      corte = "Enter";
      break;
    }
    // Llegada
    quietoTicks = (llegado && wRef == 0.0f && fabsf(wMed) < 1.0f) ? quietoTicks + 1 : 0;
    if (salirAlLlegar && quietoTicks >= nLlegada) {
      corte = nullptr;
      break;
    }
  }
  aplicarDuty(0.0f);
  msUsados = (micros() - t0) / 1000;
  delay(FRENO_MS);
  motor.coast();
  return corte;
}

// "a <grados>": ir a un ángulo con el control de posición
void irAControlado(float g, bool informar) {
  refFija = g;
  uint32_t ms = 0;
  const char *corte = cascada(refConstante, art::IR_MAX_MS * 1000 / art::TS_VEL_US, true, false, ms);
  const float fin = grados(cuentas());
  if (corte) {
    Serial.printf(">> CORTADO por %s. En %+.2f°.\n", corte, fin);
  } else if (informar) {
    Serial.printf(">> Llegó a %+.2f° en %.2f s (error %+.2f°).\n", fin, ms / 1000.0f, g - fin);
  }
}

#if defined(ARTICULACION_L298N)
// "dg": diagnóstico del cableado WROOM -> L298N. Pone en alto UN pin por vez (ENA solo, IN1 solo, IN2 solo):
// nunca ENA junto con un IN, así el puente no entrega tensión y el motor no se mueve aunque esté conectado.
// Con el multímetro se mide cada pin del L298N contra su GND: el que está en alto tiene que dar ~3,3 V.
void diagnostico() {
  motor.coast();
  const struct { const char *nombre; int pin; } etapas[] = {
      {"ENA (GPIO25)", PIN_J2_ENA}, {"IN1 (GPIO26)", PIN_J2_IN1}, {"IN2 (GPIO27)", PIN_J2_IN2}};
  Serial.println(">> Diagnóstico: un pin en alto por vez (el motor no puede moverse). Enter para pasar al siguiente.");
  for (const auto &e : etapas) {
    for (int p : {PIN_J2_ENA, PIN_J2_IN1, PIN_J2_IN2}) digitalWrite(p, LOW);
    digitalWrite(e.pin, HIGH);
    Serial.printf("   %s en ALTO, los otros dos en bajo. Medí en el L298N contra GND: %s ~3,3 V, los otros ~0 V.\n",
                  e.nombre, e.nombre);
    const uint32_t t0 = millis();
    while (!hayEntrada() && millis() - t0 < 120000) delay(10);
    descartando = false;
  }
  // Etapa 4: la salida del puente. ENA en alto y 15 % de PWM en IN1: en OUT1-OUT2 tiene que haber ~15 % de
  // la tensión de la fuente (~1,5-1,8 V). Con 15 % el sin fin no arranca (con 20 % tampoco lo hacía en 08);
  // igual se corta si el encoder cuenta movimiento.
  for (int p : {PIN_J2_ENA, PIN_J2_IN1, PIN_J2_IN2}) digitalWrite(p, LOW);
  Serial.println("   Etapa 4: ENA en alto y PWM 15 % en IN1. Medí OUT1-OUT2 (~1,5-1,8 V) y el borne 12V del L298N contra GND.");
  Serial.println("   Si el brazo se mueve: Enter (o apagá la fuente).");
  const int64_t c0 = cuentas();
  motor.setDuty(0.15f);
  // Autodiagnóstico del canal de PWM (leer el pin con la entrada habilitada lo desconectaría del LEDC)
  Serial.printf("   Canal de PWM de IN1: duty %lu de 1023 (esperado ~153), %lu Hz\n", (unsigned long)ledcRead(0),
                (unsigned long)ledcReadFreq(0));
  const uint32_t t0 = millis();
  bool movio = false;
  while (!hayEntrada() && millis() - t0 < 60000) {
    if (llabs(cuentas() - c0) > 10) {
      movio = true;
      break;
    }
    delay(5);
  }
  descartando = false;
  motor.brake();
  delay(FRENO_MS);
  motor.coast();
  if (movio) Serial.println("   CORTADO: el encoder contó movimiento.");
  Serial.println(">> Diagnóstico terminado: todo en bajo.");
}
#elif defined(ARTICULACION_J3)
// "dg": diagnóstico del cableado WROOM -> IBT-2 de J3. Etapas 1-3: nunca EN junto con un PWM, así el puente no
// entrega tensión y el motor no se mueve. Con el multímetro se mide cada pin del conector del IBT-2 contra su GND.
void esperarEnter(uint32_t maxMs) {
  const uint32_t t0 = millis();
  while (!hayEntrada() && millis() - t0 < maxMs) delay(10);
  descartando = false;
}

void diagnostico() {
  motor.coast();
  Serial.println(">> Diagnóstico del IBT-2 (Enter para pasar a la etapa siguiente).");
  // Etapa 1: EN en alto con los dos PWM en 0 = freno: el motor no puede moverse
  motor.brake();
  Serial.printf("   Etapa 1: EN (GPIO%d) en ALTO, RPWM y LPWM en 0 (freno). Medí contra el GND del IBT-2:\n"
                "            R_EN ~3,3 V, L_EN ~3,3 V, VCC ~3,3 V, B+ ~12 V. RPWM y LPWM ~0 V.\n", PIN_J3_EN);
  esperarEnter(120000);
  // Etapas 2 y 3: un PWM al 100 % (nivel fijo) con EN en bajo: salidas en alta impedancia, el motor no se mueve
  motor.coast();
  ledcWrite(LEDC_CH_R, (1u << art::PWM_BITS) - 1);
  Serial.printf("   Etapa 2: RPWM (GPIO%d) en ALTO, EN en bajo. Medí: RPWM ~3,3 V; R_EN, L_EN y LPWM ~0 V.\n",
                PIN_J3_RPWM);
  esperarEnter(120000);
  ledcWrite(LEDC_CH_R, 0);
  ledcWrite(LEDC_CH_L, (1u << art::PWM_BITS) - 1);
  Serial.printf("   Etapa 3: LPWM (GPIO%d) en ALTO, EN en bajo. Medí: LPWM ~3,3 V; R_EN, L_EN y RPWM ~0 V.\n",
                PIN_J3_LPWM);
  esperarEnter(120000);
  ledcWrite(LEDC_CH_L, 0);
  // Etapa 4: la salida del puente. EN en alto y 15 % en RPWM: entre M+ y M- tiene que haber ~15 % de la fuente
  // (~1,8 V). Se corta si el encoder cuenta movimiento.
  Serial.println("   Etapa 4: EN en alto y 15 % en RPWM. Medí M+ contra M- (~1,8 V) y B+ contra B- (~12 V).");
  Serial.println("   Si el brazo se mueve: Enter (o apagá la fuente).");
  const int64_t c0 = cuentas();
  motor.setDuty(0.15f);
  Serial.printf("   Canal de PWM de RPWM: duty %lu de %lu (esperado ~%lu), %lu Hz\n", (unsigned long)ledcRead(LEDC_CH_R),
                (unsigned long)((1u << art::PWM_BITS) - 1), (unsigned long)(0.15f * ((1u << art::PWM_BITS) - 1)),
                (unsigned long)ledcReadFreq(LEDC_CH_R));
  const uint32_t t0 = millis();
  bool movio = false;
  while (!hayEntrada() && millis() - t0 < 60000) {
    if (llabs(cuentas() - c0) > 10) {
      movio = true;
      break;
    }
    delay(5);
  }
  descartando = false;
  motor.brake();
  delay(FRENO_MS);
  motor.coast();
  if (movio) Serial.printf("   CORTADO: el encoder contó movimiento (%+lld cuentas).\n", (long long)(cuentas() - c0));
  Serial.println(">> Diagnóstico terminado: todo en bajo.");
}
#endif

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
  if (!leerNumero(args, &resto, pct) || pct == 0.0f || fabsf(pct) > art::DUTY_MAX_PCT) {
    Serial.printf("Uso: p <%%> [ms], con 0 < |%%| <= %.0f\n", art::DUTY_MAX_PCT);
    return;
  }
  uint32_t ms = art::PULSO_DEF_MS;
  if (*resto != '\0') {
    char *fin2 = nullptr;
    const long v = strtol(resto, &fin2, 10);
    while (*fin2 == ' ') fin2++;
    if (fin2 == resto || *fin2 != '\0' || v <= 0 || v > (long)art::PULSO_MAX_MS) {
      Serial.printf("Tiempo inválido: entre 1 y %lu ms\n", (unsigned long)art::PULSO_MAX_MS);
      return;
    }
    ms = (uint32_t)v;
  }
  if (!ceroFijado) {
    Serial.printf("Primero llevar a la marca (%s) y escribir \"z\".\n", art::MARCA);
    return;
  }
  pulso(pct, ms);
}

// "ev <°/s> [ms] [ciclos]"
void comandoEscalon(const char *args) {
  float v;
  const char *resto;
  if (!leerNumero(args, &resto, v) || fabsf(v) < art::VEL_MIN || fabsf(v) > art::VEL_MAX) {
    Serial.printf("Uso: ev <°/s> [ms], con %.0f <= |°/s| <= %.0f\n", art::VEL_MIN, art::VEL_MAX);
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
  if (enteros[0] < 200 || enteros[0] > (long)art::ESCALON_MAX_MS || enteros[0] % (art::TS_VEL_US / 1000) != 0) {
    Serial.printf("Tiempo inválido: entre 200 y %lu ms, múltiplo de %lu\n", (unsigned long)art::ESCALON_MAX_MS,
                  (unsigned long)(art::TS_VEL_US / 1000));
    return;
  }
  if (enteros[1] < 1 || enteros[1] > (long)art::CICLOS_MAX) {
    Serial.printf("Ciclos inválidos: entre 1 y %lu\n", (unsigned long)art::CICLOS_MAX);
    return;
  }
  if (!ceroFijado) {
    Serial.printf("Primero llevar a la marca (%s) y escribir \"z\".\n", art::MARCA);
    return;
  }
  escalonVelocidad(v, (uint32_t)enteros[0], (uint32_t)enteros[1]);
}

// "a <grados>"
void comandoIrA(const char *args) {
  float g;
  const char *resto;
  const float maxG = art::LIMITE_POS_GRADOS - 3.0f, minG = art::LIMITE_NEG_GRADOS + 3.0f;
  if (!leerNumero(args, &resto, g) || *resto != '\0' || g > maxG || g < minG) {
    Serial.printf("Uso: a <grados>, con %+.0f <= grados <= %+.0f\n", minG, maxG);
    return;
  }
  if (!ceroFijado) {
    Serial.printf("Primero llevar a la marca (%s) y escribir \"z\".\n", art::MARCA);
    return;
  }
  if (!listaParaMover()) return;
  Serial.printf(">> Ir a %+.1f° desde %+.2f° (control de posición)\n", g, grados(cuentas()));
  irAControlado(g, true);
}

// "ep <A> [ms] [ciclos]": escalones de posición desde donde está
void comandoEscalonPos(const char *args) {
  float amp;
  const char *resto;
  if (!leerNumero(args, &resto, amp) || amp < 2.0f || amp > 40.0f) {
    Serial.println("Uso: ep <A> [ms] [ciclos], con 2 <= A <= 40 grados");
    return;
  }
  long enteros[2] = {2500, 1};
  for (int i = 0; i < 2 && *resto != '\0'; i++) {
    char *fin2 = nullptr;
    enteros[i] = strtol(resto, &fin2, 10);
    if (fin2 == resto || (*fin2 != ' ' && *fin2 != '\0')) {
      Serial.println("Uso: ep <A> [ms] [ciclos]");
      return;
    }
    while (*fin2 == ' ') fin2++;
    resto = fin2;
  }
  if (*resto != '\0' || enteros[0] < 1000 || enteros[0] > (long)art::EP_SEG_MAX_MS ||
      enteros[0] % (art::TS_VEL_US / 1000) != 0 || enteros[1] < 1 || enteros[1] > (long)art::EP_CICLOS_MAX) {
    Serial.printf("Uso: ep <A> [ms] [ciclos], ms entre 1000 y %lu (múltiplo de %lu), ciclos entre 1 y %lu\n",
                  (unsigned long)art::EP_SEG_MAX_MS, (unsigned long)(art::TS_VEL_US / 1000),
                  (unsigned long)art::EP_CICLOS_MAX);
    return;
  }
  if (!ceroFijado) {
    Serial.printf("Primero llevar a la marca (%s) y escribir \"z\".\n", art::MARCA);
    return;
  }
  const float base = grados(cuentas());
  const float maxG = art::LIMITE_POS_GRADOS - 5.0f, minG = art::LIMITE_NEG_GRADOS + 5.0f;
  if (base + amp > maxG || base - amp < minG) {
    Serial.printf("Desde %+.1f°, ±%.0f° sale de %+.0f° / %+.0f°. Usar menos amplitud o \"a 0\" antes.\n", base, amp,
                  maxG, minG);
    return;
  }
  if (!listaParaMover()) return;
  epBase = base;
  epAmp = amp;
  epNSeg = (uint32_t)enteros[0] * 1000 / art::TS_VEL_US;
  epCiclos = (uint32_t)enteros[1];
  Serial.printf("# escalon_pos A=%.1f seg_ms=%ld ciclos=%ld base=%.2f kpp=%.4f vmax=%.1f amax=%.1f tol=%.2f "
                "tol_salida=%.2f vmin=%.1f kp=%.4f ki=%.4f zm=%.1f ts_ms=%lu tpos_ms=%lu\n", amp, enteros[0], enteros[1], base, kpp,
                art::VMAX_POS, art::AMAX_POS, art::TOL_POS, art::TOL_SALIDA, art::VMIN_POS, kpVel, kiVel, ffMarchaPos,
                (unsigned long)(art::TS_VEL_US / 1000), (unsigned long)(art::TS_POS_US / 1000));
  Serial.println("t_ms,th_ref,th,w_ref,w_med,duty");
  uint32_t ms = 0;
  const uint32_t nTotal = 4 * epCiclos * epNSeg + 1000000 / art::TS_VEL_US;  // + 1 s quieto al final
  const char *corte = cascada(refEscalones, nTotal, false, true, ms);
  if (corte) {
    Serial.printf("# cortado %s\n", corte);
    Serial.printf(">> CORTADO por %s.\n", corte);
  } else {
    Serial.println("# fin ok");
  }
  Serial.printf(">> Escalones de posición terminados. En %+.2f°.\n", grados(cuentas()));
}

// "kp <x>" / "ki <x>"
// "bc <desde> <hasta> [v]": barrido de carga. Va a <desde> con el control de posición y después recorre hasta
// <hasta> a velocidad constante (PI de velocidad). A velocidad baja y constante, el duty que termina aplicando el
// PI es el esfuerzo que hace falta en cada ángulo (peso a través del cuatro barras + fricción): con eso se arma la
// tabla del feedforward según el ángulo. CSV igual que "ev" (t_ms,w_ref,w_med,duty,pos).
void barridoCarga(float desde, float hasta, float v) {
  irAControlado(desde, true);
  const float g0 = grados(cuentas());
  if (fabsf(g0 - desde) > 1.0f) {
    Serial.println(">> Cancelado: no llegó al punto de partida.");
    return;
  }
  if (!listaParaMover()) return;
  const int dir = hasta > g0 ? 1 : -1;
  const float wRef = dir * v, ts = art::TS_VEL_US / 1e6f;
  const uint32_t tsMs = art::TS_VEL_US / 1000;
  const uint32_t nMax = (uint32_t)((fabsf(hasta - g0) / v + 5.0f) / ts);
  int64_t hist[art::VENTANA_VEL + 1];
  const int64_t c0 = cuentas();
  for (auto &h : hist) h = c0;
  float integral = 0.0f;
  uint32_t atascoTicks = 0, contrarioTicks = 0, sinCuentasTicks = 0;
  const char *corte = "no llegó a tiempo";
  Serial.printf("# barrido desde=%.2f hasta=%.2f v=%.1f kp=%.4f ki=%.4f zm=%.1f zm_neg=%.1f desp=%.1f desp_neg=%.1f "
                "pend=%.2f ts_ms=%.0f ventana=%d cuentas_por_grado=%.4f\n", g0, hasta, wRef, kpVel, kiVel, ffMarchaPos,
                ffMarchaNeg, ffDespPos, ffDespNeg, ffPendPos, ts * 1000, art::VENTANA_VEL, art::CUENTAS_POR_GRADO);
  Serial.println("t_ms,w_ref,w_med,duty,pos");
  uint32_t tick = micros();
  const uint32_t t0 = tick;
  for (uint32_t k = 0; k < nMax; k++) {
    while (micros() - tick < art::TS_VEL_US) {
    }
    tick += art::TS_VEL_US;
    const int64_t c = cuentas();
    for (int i = 0; i < art::VENTANA_VEL; i++) hist[i] = hist[i + 1];
    hist[art::VENTANA_VEL] = c;
    const float wMed = grados(hist[art::VENTANA_VEL] - hist[0]) / (art::VENTANA_VEL * ts);
    const float u = pasoPI(wRef, wMed, grados(c), integral);
    aplicarDuty(u);
    Serial.printf("%lu,%.1f,%.2f,%.1f,%.2f\n", (unsigned long)((tick - t0) / 1000), wRef, wMed, u, grados(c));
    if ((grados(c) - hasta) * dir >= 0.0f) {
      corte = nullptr;
      break;
    }
    if (excesoLimite(c) > 0) {
      corte = "límite de ángulo";
      break;
    }
    // A velocidad baja el sin fin se traba un rato (hasta ~150 ms): acá se tolera más que en "ev"
    sinCuentasTicks = (fabsf(u) >= art::ARRANQUE_PCT && c == hist[art::VENTANA_VEL - 1]) ? sinCuentasTicks + 1 : 0;
    if (sinCuentasTicks * tsMs >= 3 * art::SIN_CUENTAS_MS) {
      corte = "sin cuentas del encoder (¿encoder suelto o motor trabado?)";
      break;
    }
    contrarioTicks = (wMed * dir < -art::CONTRARIO_VEL) ? contrarioTicks + 1 : 0;
    if (contrarioTicks * tsMs >= art::CONTRARIO_MS) {
      corte = "se mueve al revés de lo pedido (¿signos mal?)";
      break;
    }
    atascoTicks = (fabsf(wMed) < art::ATASCO_VEL && fabsf(u) >= art::ATASCO_DUTY_PCT) ? atascoTicks + 1 : 0;
    if (atascoTicks * tsMs >= art::ATASCO_MS) {
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
  motor.coast();
  if (corte) {
    Serial.printf("# cortado %s\n", corte);
    Serial.printf(">> CORTADO por %s.\n", corte);
  } else {
    Serial.println("# fin ok");
  }
  Serial.printf(">> Barrido terminado. En %+.2f°.\n", grados(cuentas()));
}

void comandoBarrido(const char *args) {
  float desde, hasta, v = 4.0f;
  const char *p;
  const float maxG = art::LIMITE_POS_GRADOS - 3.0f, minG = art::LIMITE_NEG_GRADOS + 3.0f;
  if (!leerNumero(args, &p, desde) || !leerNumero(p, &p, hasta) || (*p != '\0' && !leerNumero(p, &p, v)) ||
      *p != '\0' || desde < minG || desde > maxG || hasta < minG || hasta > maxG || fabsf(hasta - desde) < 2.0f ||
      v < 1.0f || v > 10.0f) {
    Serial.printf("Uso: bc <desde> <hasta> [°/s], con %+.0f <= grados <= %+.0f y 1 <= °/s <= 10 (def. 4)\n", minG,
                  maxG);
    return;
  }
  if (!ceroFijado) {
    Serial.printf("Primero llevar a la marca (%s) y escribir \"z\".\n", art::MARCA);
    return;
  }
  barridoCarga(desde, hasta, v);
}

// "ff <desp+> <marcha+> <desp-> <marcha->": los cuatro en % (positivos), entre 0 y el duty máximo
void comandoFF(const char *args) {
  float v[4];
  const char *p = args;
  for (float &x : v) {
    if (!leerNumero(p, &p, x) || x < 0.0f || x > art::DUTY_MAX_PCT) {
      Serial.printf("Uso: ff <desp+> <marcha+> <desp-> <marcha->, cada uno entre 0 y %.0f %%\n", art::DUTY_MAX_PCT);
      return;
    }
  }
  float pend = ffPendPos;
  if (*p != '\0' && (!leerNumero(p, &p, pend) || pend < 0.0f || pend > 5.0f || *p != '\0')) {
    Serial.println("Uso: ff <desp+> <marcha+> <desp-> <marcha-> [%/° subiendo, 0-5]");
    return;
  }
  ffPendPos = pend;
  ffDespPos = v[0];
  ffMarchaPos = v[1];
  ffDespNeg = v[2];
  ffMarchaNeg = v[3];
  mostrarGanancias();
}

// "bj <vmin bajando °/s> <trabada ms> <dither %>": ajustes contra el traba-suelta al bajar
void comandoBajada(const char *args) {
  float v[3];
  const char *p = args;
  for (float &x : v) {
    if (!leerNumero(p, &p, x)) {
      p = nullptr;
      break;
    }
  }
  if (!p || *p != '\0' || v[0] < 1.0f || v[0] > 10.0f || v[1] < 20.0f || v[1] > 500.0f || v[2] < 0.0f || v[2] > 10.0f) {
    Serial.println("Uso: bj <vmin bajando 1-10 °/s> <trabada 20-500 ms> <dither 0-10 %>");
    return;
  }
  vminBaja = v[0];
  trabadaMs = (uint32_t)v[1];
  ditherPct = v[2];
  mostrarGanancias();
}

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
  const float maxG = art::LIMITE_POS_GRADOS - 3.0f, minG = art::LIMITE_NEG_GRADOS + 3.0f;  // margen de frenado
  if (!leerNumero(args, &resto, g) || g > maxG || g < minG) {
    Serial.printf("Uso: m <grados> [%%], con %+.0f <= grados <= %+.0f\n", minG, maxG);
    return;
  }
  float pct = art::DUTY_MOVER_PCT;
  if (*resto != '\0') {
    const char *fin;
    if (!leerNumero(resto, &fin, pct) || *fin != '\0' || pct <= 0.0f || pct > art::DUTY_MAX_PCT) {
      Serial.printf("Duty inválido: entre 0 y %.0f %%\n", art::DUTY_MAX_PCT);
      return;
    }
  }
  if (!ceroFijado) {
    Serial.printf("Primero llevar a la marca (%s) y escribir \"z\".\n", art::MARCA);
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
    motor.coast();
    Serial.println(">> Rueda libre.");
  } else if (!hwOk) {
    Serial.println("Hardware sin inicializar: comandos deshabilitados.");
  } else if (strcmp(l, "e") == 0) {
    monitorEncoder();
  } else if (strcmp(l, "c") == 0) {
    mostrarPosicion();
  } else if (strcmp(l, "z") == 0) {
    enc.reset();
    ceroFijado = true;
    Serial.println(">> Cero fijado en la marca. Movimientos habilitados.");
  } else if (strcmp(l, "ie") == 0 || strcmp(l, "is") == 0) {
    if (!art::SIGNOS_EN_VIVO) {
      // J1: con límites asimétricos (+135° / -45°), invertir el sentido daría vuelta los límites físicos
      // sin que el firmware lo note. Los signos ya quedaron fijos en config.h (paso 0).
      mostrarSignos();
      Serial.println("   Deshabilitado: los signos están fijos en include/config.h desde el paso 0.");
    } else {
      // Con los límites simétricos se puede probar en vivo. Cambia el signo de las cuentas: hay que volver a
      // fijar el cero. Después pasar el valor a config.h (se pierde al reiniciar).
      if (l[1] == 'e') encInvertido = !encInvertido;
      else sentidoInvertido = !sentidoInvertido;
      motor.coast();
      ceroFijado = false;
      mostrarSignos();
      Serial.printf("   Volver a la marca (%s) y escribir \"z\". Pasar el valor a config.h.\n", art::MARCA);
    }
#if defined(ARTICULACION_L298N) || defined(ARTICULACION_J3)
  } else if (strcmp(l, "dg") == 0) {
    diagnostico();
#endif
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
  } else if (strncmp(l, "bj", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoBajada(l + 2);
  } else if (strncmp(l, "bc", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoBarrido(l + 2);
  } else if (strncmp(l, "ff", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoFF(l + 2);
  } else if (strcmp(l, "g") == 0) {
    mostrarGanancias();
  } else if (strncmp(l, "kpp", 3) == 0 && (l[3] == ' ' || l[3] == '\0')) {
    comandoGanancia(l + 3, kpp, "kpp");
  } else if (l[0] == 'a' && (l[1] == ' ' || l[1] == '\0')) {
    comandoIrA(l + 1);
  } else if (strncmp(l, "ep", 2) == 0 && (l[2] == ' ' || l[2] == '\0')) {
    comandoEscalonPos(l + 2);
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

// Las articulaciones que no se prueban comparten la placa: sus pines de potencia van a bajo apenas arranca,
// para que su driver no quede con entradas al aire y mueva el motor solo.
void apagarPin(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

void setup() {
  // Driver primero y deshabilitado: hasta acá el EN/ENA depende del pull-down de 10 kΩ
#if defined(ARTICULACION_J3)
  for (int p : {PIN_J1_EN, PIN_J1_RPWM, PIN_J1_LPWM, PIN_J2_ENA, PIN_J2_IN1, PIN_J2_IN2}) apagarPin(p);
#elif defined(ARTICULACION_L298N)
  for (int p : {PIN_J1_EN, PIN_J1_RPWM, PIN_J1_LPWM, PIN_J3_EN, PIN_J3_RPWM, PIN_J3_LPWM}) apagarPin(p);
#else
  for (int p : {PIN_J2_ENA, PIN_J2_IN1, PIN_J2_IN2, PIN_J3_EN, PIN_J3_RPWM, PIN_J3_LPWM}) apagarPin(p);
#endif
#if defined(ARTICULACION_L298N)
  const bool okDriver = motor.begin(PIN_J2_ENA, PIN_J2_IN1, PIN_J2_IN2, LEDC_CH_R, LEDC_CH_L, L298N::Modo::Freno,
                                    art::PWM_FREQ_HZ, art::PWM_BITS);
  motor.coast();
  const bool okEnc = enc.begin(PIN_J2_ENC_A, PIN_J2_ENC_B);  // pull-up interno (NPN colector abierto)
#elif defined(ARTICULACION_J3)
  const bool okDriver = motor.begin(PIN_J3_RPWM, PIN_J3_LPWM, PIN_J3_EN, PIN_J3_EN, LEDC_CH_R, LEDC_CH_L,
                                    art::PWM_FREQ_HZ, art::PWM_BITS);
  motor.coast();
  const bool okEnc = enc.begin(PIN_J3_ENC_A, PIN_J3_ENC_B);  // pull-up interno (NPN colector abierto)
#else
  const bool okDriver = motor.begin(PIN_J1_RPWM, PIN_J1_LPWM, PIN_J1_EN, PIN_J1_EN, LEDC_CH_R, LEDC_CH_L,
                                    art::PWM_FREQ_HZ, art::PWM_BITS);
  motor.coast();
  // GPIO34/35 no tienen pull-up interno (el driver intenta activarlo y no pasa nada): usan los externos
  const bool okEnc = enc.begin(PIN_J1_ENC_A, PIN_J1_ENC_B);
#endif
  hwOk = okDriver && okEnc;

  Serial.begin(115200);
  delay(1500);

  Serial.println();
#if defined(ARTICULACION_L298N)
  Serial.printf("== 10 · %s (%s): 5840-31ZY + L298N, WROOM ==\n", art::NOMBRE, art::PIEZA);
  Serial.printf("L298N: ENA=GPIO%d, IN1=GPIO%d, IN2=GPIO%d | modo freno, PWM %lu Hz\n", PIN_J2_ENA, PIN_J2_IN1,
                PIN_J2_IN2, (unsigned long)art::PWM_FREQ_HZ);
  Serial.printf("Encoder: A=GPIO%d, B=GPIO%d | 4000 cuentas por vuelta del sin fin x engranaje %.2f:1 = %.2f cuentas "
                "por grado%s\n", PIN_J2_ENC_A, PIN_J2_ENC_B, art::RELACION_ENGRANAJE, art::CUENTAS_POR_GRADO,
                art::RELACION_ENGRANAJE == 1.0f ? " (engranaje SIN MEDIR: grados de la salida del sin fin)" : "");
  Serial.println("Cuatro barras: el otro motor (J2 o J3) queda quieto, no salir de los límites (tools/cuatro_barras.py).");
#elif defined(ARTICULACION_J3)
  Serial.printf("== 10 · %s (%s): 5840-31ZY + IBT-2, WROOM ==\n", art::NOMBRE, art::PIEZA);
  Serial.printf("IBT-2: RPWM=GPIO%d, LPWM=GPIO%d, EN=GPIO%d | PWM %lu Hz\n", PIN_J3_RPWM, PIN_J3_LPWM, PIN_J3_EN,
                (unsigned long)art::PWM_FREQ_HZ);
  Serial.printf("Encoder: A=GPIO%d, B=GPIO%d | 4000 cuentas por vuelta del sin fin x engranaje %.2f:1 = %.2f cuentas "
                "por grado\n", PIN_J3_ENC_A, PIN_J3_ENC_B, art::RELACION_ENGRANAJE, art::CUENTAS_POR_GRADO);
  Serial.println("Cuatro barras: J2 queda quieto, no salir de los límites (tools/cuatro_barras.py).");
#else
  Serial.println("== 10 · J1 (base): 36GP-555 + IBT-2, WROOM ==");
  Serial.printf("IBT-2: RPWM=GPIO%d, LPWM=GPIO%d, EN=GPIO%d | PWM %lu Hz\n", PIN_J1_RPWM, PIN_J1_LPWM, PIN_J1_EN,
                (unsigned long)art::PWM_FREQ_HZ);
  Serial.printf("Encoder: A=GPIO%d, B=GPIO%d | reductora 50:1 x correa %.0f:1 = %.2f cuentas por grado de la base\n",
                PIN_J1_ENC_A, PIN_J1_ENC_B, art::RELACION_CORREA, art::CUENTAS_POR_GRADO);
#endif
  mostrarSignos();
  mostrarGanancias();
  if (!okDriver) Serial.println("ERROR: no se pudo configurar el PWM (LEDC).");
  if (!okEnc) Serial.println("ERROR: no se pudo configurar el PCNT del encoder.");
  Serial.printf("Niveles del encoder en reposo: A=%d B=%d (con los pull-up, quieto puede ser 0 o 1)\n",
                enc.levelA(), enc.levelB());
  Serial.printf("Motor en rueda libre. Movimientos deshabilitados hasta \"z\" en la marca (%s).\n", art::MARCA);
  ayuda();
  // Al abrir el puerto, el CH340 reinicia la placa y su línea TX da saltos mientras se configura: la WROOM
  // recibe bytes de basura sin Enter, que se quedaban en la línea y se pegaban al primer comando ("z").
  while (Serial.available()) Serial.read();
}

void loop() { leerSerial(); }
