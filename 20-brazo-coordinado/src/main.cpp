// 20 · J1, J2 y J3 con el MISMO controlador (lib/Articulacion), en la ESP32-WROOM-32D con el conexionado del robot.
// Fase 5a (ver README.md): un solo lazo NO bloqueante a 100 Hz (tarea "control") que sostiene las tres, homing de
// las tres (J3 -> J2 -> J1) con el cero en el flanco, y movimientos de a una ("j") dentro de la franja del cuatro
// barras. Los comandos llegan por Serial (loop) a una cola; la tarea los ejecuta entre periodos y deja los mensajes
// en otra cola que imprime loop(): la tarea nunca espera a Serial.
// Fase 5b: "mv q1 q2 q3": las tres a la vez, terminan juntas, camino recto en (q2, q3) dentro de la franja, llegada
// siempre con el ángulo creciendo; al asentarse, línea LLEGO.
// Fase 5c: "t": telemetría a 50 Hz (líneas T,...; tools/trayectoria.py la captura y la grafica). Pendiente: zona
// prohibida (la caja).
#include <Arduino.h>

#include "Articulacion.h"
#include "config.h"
#include "pins.h"
#include "servos.h"

namespace {

constexpr int N = 3;
constexpr int I1 = 0, I2 = 1, I3 = 2;
Articulacion art[N];
bool cero[N] = {false, false, false};  // cero hecho ("z" o "h")
bool hwOk = false;
bool frenoBanda = FRENO_EN_BANDA;
const char *ultimoError[N] = {nullptr, nullptr, nullptr};

QueueHandle_t colaComandos;  // líneas de Serial -> tarea
volatile bool xPendiente = false;  // llegó "x": la tarea descarta lo que esté en la cola antes (no sube el EN)
QueueHandle_t colaMensajes;  // tarea -> Serial
struct Linea {
  char t[48];
};
struct Mensaje {
  char t[120];
};

// ---------- Fase 5c: telemetría ----------
// La tarea deja muestras binarias en su propia cola (no compite con los mensajes: un LLEGO no se pierde por la
// telemetría); loop() las formatea. Si la cola se llena se pierden muestras y se cuentan.
constexpr int TELE_DIV = 2;  // una muestra cada 2 periodos: 50 Hz
struct Muestra {
  uint32_t ms;
  uint8_t etapa;       // del mv (1 recorrido, 2 aproximación, 3 llegada, 4 asentamiento), 8 joystick, 9 homing,
                       // 0 nada
  float q[3], r[3], u[3], w[3];
  uint8_t est[3];      // 0 suelta, 1 sosteniendo yendo, 2 llegó, 3 firme, 4 frenada, 5 siguiendo, 6 homing
};
QueueHandle_t colaTele;
volatile bool tele = false;
volatile uint32_t telePerdidas = 0;

void msg(const char *fmt, ...) {
  Mensaje m;
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(m.t, sizeof(m.t), fmt, ap);
  va_end(ap);
  xQueueSend(colaMensajes, &m, 0);  // si la cola está llena, se pierde (no bloquear el control)
}

// El EN es común a los tres IBT-2: en alto mientras alguna se mueve o se sostiene; solo "x" lo baja
void enAlto() { digitalWrite(PIN_BRAZO_EN, HIGH); }

bool cerosJ2J3() { return cero[I2] && cero[I3]; }

// ---------- Franja del cuatro barras (config.h, 2026-11-04): q2 - Q2_MENOS_Q3_MAX <= q3 <= q3MaxFranja(q2) ----------
constexpr int N_FRANJA = sizeof(FRANJA_ARRIBA) / sizeof(FRANJA_ARRIBA[0]);

// Borde de arriba: q3 máximo con J2 en q2 (tabla interpolada; fuera de la tabla, el valor del extremo)
float q3MaxFranja(float q2) {
  if (q2 <= FRANJA_ARRIBA[0].q2) return FRANJA_ARRIBA[0].q3Max;
  for (int k = 1; k < N_FRANJA; k++) {
    const PuntoFranja &a = FRANJA_ARRIBA[k - 1], &b = FRANJA_ARRIBA[k];
    if (q2 <= b.q2) return a.q3Max + (b.q3Max - a.q3Max) * (q2 - a.q2) / (b.q2 - a.q2);
  }
  return FRANJA_ARRIBA[N_FRANJA - 1].q3Max;
}

// Al revés: el q2 mínimo para que q3 quede debajo del borde de arriba (la tabla crece). -inf si cualquier q2 sirve,
// +inf si ninguno (q3 más alto que el último punto)
float q2MinFranja(float q3) {
  if (q3 <= FRANJA_ARRIBA[0].q3Max) return -INFINITY;
  for (int k = 1; k < N_FRANJA; k++) {
    const PuntoFranja &a = FRANJA_ARRIBA[k - 1], &b = FRANJA_ARRIBA[k];
    if (q3 <= b.q3Max) return a.q2 + (b.q2 - a.q2) * (q3 - a.q3Max) / (b.q3Max - a.q3Max);
  }
  return INFINITY;
}

// Borde de abajo medido (2026-11-10 noche): q3 mínimo con J2 en q2 (tabla interpolada). -inf abajo del primer punto
constexpr int N_FRANJA_ABAJO = sizeof(FRANJA_ABAJO) / sizeof(FRANJA_ABAJO[0]);
float q3MinTabla(float q2) {
  if (q2 < FRANJA_ABAJO[0].q2) return -INFINITY;
  for (int k = 1; k < N_FRANJA_ABAJO; k++) {
    const PuntoFranjaAbajo &a = FRANJA_ABAJO[k - 1], &b = FRANJA_ABAJO[k];
    if (q2 <= b.q2) return a.q3Min + (b.q3Min - a.q3Min) * (q2 - a.q2) / (b.q2 - a.q2);
  }
  return FRANJA_ABAJO[N_FRANJA_ABAJO - 1].q3Min;
}
// Al revés: el q2 máximo para que q3 quede arriba de la tabla (la tabla crece). +inf si cualquier q2 sirve
float q2MaxTabla(float q3) {
  if (q3 < FRANJA_ABAJO[0].q3Min) return FRANJA_ABAJO[0].q2;
  for (int k = 1; k < N_FRANJA_ABAJO; k++) {
    const PuntoFranjaAbajo &a = FRANJA_ABAJO[k - 1], &b = FRANJA_ABAJO[k];
    if (q3 <= b.q3Min) return a.q2 + (b.q2 - a.q2) * (q3 - a.q3Min) / (b.q3Min - a.q3Min);
  }
  return INFINITY;
}
// Borde de abajo del camino / freno (recta Q2_MENOS_Q3_MAX y tabla) y de los destinos (recta Q2_MENOS_Q3_DESTINO y tabla)
float q3MinCamino(float q2) { return fmaxf(q2 - Q2_MENOS_Q3_MAX, q3MinTabla(q2)); }
float q3MinDestino(float q2) { return fmaxf(q2 - Q2_MENOS_Q3_DESTINO, q3MinTabla(q2)); }

// Cuánto sale (q3, en °) de la franja: <= 0 adentro. lado: +1 el codo demasiado estirado (q3 sobre el borde de
// arriba), -1 el hombro demasiado arriba (q3 bajo q2 - Q2_MENOS_Q3_MAX)
// "fr 0" (2026-11-10): franja APAGADA para volver a medirla con el joystick (doble helicoidales). Solo quedan los
// límites propios de cada una y los cortes por atasco / sin cuentas. Al reiniciar vuelve prendida
bool franjaActiva = true;

float excesoFranja(float q2, float q3, int *lado = nullptr) {
  if (!franjaActiva) {
    if (lado) *lado = 0;
    return -1000.0f;
  }
  const float arriba = q3 - q3MaxFranja(q2);
  const float abajo = q3MinCamino(q2) - q3;
  if (lado) *lado = arriba >= abajo ? 1 : -1;
  return fmaxf(arriba, abajo);
}
float excesoFranjaAhora(int *lado = nullptr) { return excesoFranja(art[I2].q(), art[I3].q(), lado); }
// Lo mismo para un destino (donde J2/J3 quedan sostenidas): el borde de abajo es Q2_MENOS_Q3_DESTINO (firmeza de J3)
float excesoDestino(float q2, float q3) { return !franjaActiva ? -1000.0f : fmaxf(q3 - q3MaxFranja(q2), q3MinDestino(q2) - q3); }

// ---------- Caja (config.h, 2026-11-10 noche): con J1 >= CAJA_J1_MIN, q3 <= q3MaxCaja(q2) ----------
// Siempre activa (también con "fr 0"). J1 es la del encoder (el modelo convierte a la real)
constexpr int N_CAJA = sizeof(CAJA_ARRIBA) / sizeof(CAJA_ARRIBA[0]);
float q3MaxCaja(float q2) {
  if (q2 < CAJA_ARRIBA[0].q2) return INFINITY;
  for (int k = 1; k < N_CAJA; k++) {
    const PuntoFranja &a = CAJA_ARRIBA[k - 1], &b = CAJA_ARRIBA[k];
    if (q2 <= b.q2) return a.q3Max + (b.q3Max - a.q3Max) * (q2 - a.q2) / (b.q2 - a.q2);
  }
  return CAJA_ARRIBA[N_CAJA - 1].q3Max;
}
// El más bajo de la tabla con J2 entre a y b (la tabla tiene un pozo en J2 68-69: no alcanza con los extremos)
float q3MaxCajaEntre(float a, float b) {
  float m = fminf(q3MaxCaja(a), q3MaxCaja(b));
  for (float x = ceilf(a); x <= b; x += 1.0f) m = fminf(m, q3MaxCaja(x));
  return m;
}
// Cuánto sale J3 (°) de la zona de la caja: <= 0 adentro; -1000 con J1 al costado
float excesoCaja(float q1, float q2, float q3) { return q1 < CAJA_J1_MIN ? -1000.0f : q3 - q3MaxCaja(q2); }
// J1 con la que se revisa la caja en el camino de un "mv": la más alta entre la de partida y la de llegada (J1 tiene
// su propia duración: puede llegar antes que J2/J3). -inf fuera de comandoMv
float q1Camino = -INFINITY;

// El exceso más grande sobre la recta de (a2, a3) a (b2, b3), mirando cada "paso" grados
float excesoTramo(float a2, float a3, float b2, float b3, float paso = 0.25f) {
  const int pasos = 1 + (int)(fmaxf(fabsf(b2 - a2), fabsf(b3 - a3)) / paso);
  float peor = -INFINITY;
  for (int k = 0; k <= pasos; k++) {
    const float t = (float)k / pasos;
    const float q2 = a2 + t * (b2 - a2), q3 = a3 + t * (b3 - a3);
    peor = fmaxf(peor, fmaxf(excesoFranja(q2, q3), excesoCaja(q1Camino, q2, q3)));
  }
  return peor;
}
bool enMovimiento(int i) {
  const auto m = art[i].modo();
  return m == Articulacion::Modo::Seguir || m == Articulacion::Modo::Homing ||
         (m == Articulacion::Modo::Sostener && !art[i].llego());
}

uint8_t estadoTele(const Articulacion &a) {
  switch (a.modo()) {
    case Articulacion::Modo::Suelta: return 0;
    case Articulacion::Modo::Seguir: return 5;
    case Articulacion::Modo::Homing: return 6;
    default: break;
  }
  if (a.frenada()) return 4;
  if (a.firme()) return 3;
  return a.llego() ? 2 : 1;
}

// ---------- Homing de las tres. Con el brazo abajo (J2 sin cero o a HOMING_Q2_JUNTAS o menos) las tres juntas
// (2026-11-03): J2 y J3 bajan a la misma velocidad, q3 - q2 casi no cambia y al terminar una la diferencia va hacia
// 0, dentro de la franja. Si no, J3 -> J2 (con el codo abajo, bajar el hombro no viola la franja) -> J1 ----------
struct {
  bool activo = false;
  int grupo = 0;                 // índice en grupos
  int nGrupos = 0;
  int grupos[N][N];              // articulaciones de cada grupo (-1 = vacío); el grupo siguiente arranca al terminar
  int etapa[N] = {};             // 0 arrancar, 1 buscando, 2 retrocediendo, 3 terminada, 4 J3 en su flanco esperando a J2
  bool preSubida = false;        // J2 sube antes de empezar (J3 arrancó con su final de carrera apretado)
  uint32_t tPre = 0;
  uint32_t tEtapa[N] = {};       // millis() al empezar el retroceso
  uint32_t tFlancoJ3 = 0;        // millis() cuando J3 llegó a su flanco (para medir la espera)
  bool j3Solto = false;          // J3 ya soltó su final de carrera al retroceder
  bool conCero[N] = {};
  float delta[N] = {NAN, NAN, NAN};
} hm;

void homingCortar(const char *causa) {
  for (auto &a : art) {
    a.homingCancelar();
    if (a.modo() != Articulacion::Modo::Suelta) a.sostenerAqui();
  }
  hm.activo = false;
  msg(">> Homing CORTADO: %s. Quedan sosteniendo donde están (repetir \"h\").", causa);
  msg("HOMING_ERROR,%s", causa);
}

// La medición de J2/J3 salió de la franja (la tarea frena una vez por episodio; "zq" lo pone si ya empieza afuera)
bool fueraFranja = false;
bool fueraCaja = false;  // ya avisado (una vez por episodio)

bool fueraFranjaAhora() {
  return cerosJ2J3() && excesoFranjaAhora() > FRANJA_HOLGURA;
}

void homingIniciar() {
  if (fueraFranjaAhora()) {
    msg("Homing: J2 y J3 fuera de la franja (%.2f° afuera): volver adentro con \"j\" antes.", excesoFranjaAhora());
    return;
  }
  for (int i = 0; i < N; i++) hm.conCero[i] = cero[i];
  if (cero[I2] && art[I2].q() > HOMING_Q2_MAX_ARRANQUE) {
    msg("Homing: J2 en %+.1f°; más arriba de %.0f° el codo no llega a su final de carrera sin salir de la franja. "
        "Bajarla antes (j 2 10).", art[I2].q(), HOMING_Q2_MAX_ARRANQUE);
    return;
  }
  if (!cero[I1] || !cero[I2] || !cero[I3])
    msg(">> Alguna sin cero: bajan juntas despacio hasta sus finales de carrera, desde donde estén (mano cerca).");
  enAlto();
  for (auto &a : art) {  // todo quieto donde está antes de empezar (también lo que iba a otro destino)
    a.reiniciarEstado();
    a.sostenerAqui();
  }
  // Orden: las tres juntas si el brazo ya está abajo; si no, J3, J2 y al final J1
  const bool juntas = !cero[I2] || art[I2].q() <= HOMING_Q2_JUNTAS;
  for (auto &g : hm.grupos)
    for (auto &x : g) x = -1;
  if (juntas) {
    hm.grupos[0][0] = I3;
    hm.grupos[0][1] = I2;
    hm.grupos[0][2] = I1;
    hm.nGrupos = 1;
  } else {
    hm.grupos[0][0] = I3;
    hm.grupos[1][0] = I2;
    hm.grupos[2][0] = I1;
    hm.nGrupos = 3;
  }
  hm.activo = true;
  hm.grupo = 0;
  for (auto &e : hm.etapa) e = 0;
  for (auto &d : hm.delta) d = NAN;
  // 2026-11-10 (doble helicoidales, casi sin juego): con J2 abajo J3 no puede alejarse de su final de carrera para
  // volver a buscar el flanco (trayectoria_20261110_141606_cortado: +50 % sin una cuenta). J2 sube antes
  hm.preSubida = art[I3].fc();
  if (hm.preSubida) {
    hm.tPre = millis();
    art[I2].sostener(art[I2].q() + HOMING_PRE_SUBIDA_J2);
    msg("   J3 ya aprieta su final de carrera: J2 sube %.0f° antes de empezar.", HOMING_PRE_SUBIDA_J2);
  }
  msg(">> Homing: %s (J3 %.1f, J2 %.1f, J1 %.1f °/s). Cualquier comando corta.",
      juntas ? "las tres juntas" : "J3, J2 y al final J1 (J2 arriba)", CFG_J3.homingVelBuscar,
      CFG_J2.homingVelBuscar, CFG_J1.homingVelBuscar);
}

// 2026-11-10 noche: J3 retrocede apenas J2 subió a HOMING_RETROCESO_J2 - 1° (antes esperaba a que J2 quedara FIRME:
// a veces J2 tardaba en asentarse y J3 quedaba apretando su final de carrera muchos segundos). Con J2 en 5° el borde de
// arriba está en ~6,5°: J3 tiene lugar para ir a 3°
bool j2DejaLugarAJ3() {
  return hm.etapa[I2] == 3 || (hm.etapa[I2] == 2 && art[I2].q() >= HOMING_RETROCESO_J2 - 1.0f);
}
void j3Retrocede() {
  msg("   J3 retrocede (esperó %.1f s en su final de carrera; J2 en %+.2f°).", (millis() - hm.tFlancoJ3) / 1000.0f,
      art[I2].q());
  hm.j3Solto = false;
}

// Un paso del homing de una articulación: false si cortó (ya llamó a homingCortar)
bool homingPasoArt(int i) {
  Articulacion &a = art[i];
  if (hm.etapa[i] == 0) {
    a.homing(hm.conCero[i], hm.conCero[i] ? 0.0f : HOMING_RECORRIDO_SIN_CERO);
    hm.etapa[i] = 1;
    msg("   %s buscando su final de carrera...", a.cfg().nombre);
    return true;
  }
  if (hm.etapa[i] == 1) {
    if (a.estadoHoming() == Articulacion::Homing::EnCurso) return true;
    if (a.estadoHoming() != Articulacion::Homing::Listo) {
      homingCortar(a.error() ? a.error() : "error");
      return false;
    }
    hm.delta[i] = a.flancoGrados();
    a.ceroEnFlanco();
    cero[i] = true;
    // 2026-11-10: J3 no retrocede hasta que J2 terminó (desde la noche: hasta que J2 pasa HOMING_RETROCESO_J2 - 1°). Con los doble helicoidales (casi sin juego) J3 subiendo a 3°
    // mientras J2 baja a su final de carrera se empujan por el cuatro barras (q3 <= ~3° con J2 en 0) y se traban las
    // dos (trayectoria_20261110_135031_cortado: J3 +69 % y J2 -65 % sin avanzar). Queda sostenida en su flanco
    if (i == I3) hm.tFlancoJ3 = millis();
    if (i == I3 && !j2DejaLugarAJ3()) {
      hm.etapa[i] = 4;
      msg("   J3 en su final de carrera: espera a J2 para retroceder.");
      return true;
    }
    if (i == I3) j3Retrocede();
    a.sostener(i == I2 ? HOMING_RETROCESO_J2 : i == I3 ? HOMING_RETROCESO_J3 : HOMING_RETROCESO);
    hm.etapa[i] = 2;
    hm.tEtapa[i] = millis();
    return true;
  }
  if (hm.etapa[i] == 3) return true;
  if (hm.etapa[i] == 4) {  // J3 esperando a J2
    if (a.error()) {
      homingCortar(a.error());
      return false;
    }
    if (!j2DejaLugarAJ3()) return true;
    j3Retrocede();
    a.sostener(i == I2 ? HOMING_RETROCESO_J2 : i == I3 ? HOMING_RETROCESO_J3 : HOMING_RETROCESO);
    hm.etapa[i] = 2;
    hm.tEtapa[i] = millis();
    return true;
  }
  // etapa 2: retrocediendo
  if (a.error()) {
    homingCortar(a.error());
    return false;
  }
  if (i == I3 && !hm.j3Solto && !a.fc()) {
    hm.j3Solto = true;
    msg("   J3 soltó su final de carrera (%.1f s después de empezar a retroceder).", (millis() - hm.tEtapa[i]) / 1000.0f);
  }
  if (!a.firme()) {
    if (millis() - hm.tEtapa[i] > 15000) {
      msg("   %s no queda firme: en %+.2f° (pedido %+.2f°), duty %+.1f %%, w %+.2f °/s, %s.", a.cfg().nombre, a.q(),
          a.referencia(), a.duty(), a.w(), a.llego() ? "llegó pero no quieta" : "no llegó");
      homingCortar("no termina de retroceder (15 s)");
      return false;
    }
    return true;
  }
  msg("   %s: flanco %+.3f° del cero anterior%s; en %+.2f°.", a.cfg().nombre, hm.delta[i],
      hm.conCero[i] ? "" : " (no había cero)", a.q());
  hm.etapa[i] = 3;
  return true;
}

void homingPaso() {
  if (!hm.activo) return;
  if (hm.preSubida) {
    if (art[I2].error()) {
      homingCortar(art[I2].error());
      return;
    }
    if (!art[I2].firme()) {
      if (millis() - hm.tPre > 15000) homingCortar("J2 no termina de subir antes de empezar (15 s)");
      return;
    }
    hm.preSubida = false;
  }
  // Las del grupo actual y las que ya empezaron (J3 espera a J2, que puede estar en el grupo siguiente)
  bool grupoListo = true;
  for (int i = 0; i < N; i++) {
    bool enGrupo = false;
    if (hm.grupo < hm.nGrupos)
      for (int x : hm.grupos[hm.grupo]) enGrupo = enGrupo || x == i;
    if (!enGrupo && hm.etapa[i] == 0) continue;
    if (!homingPasoArt(i)) return;
    if (enGrupo) grupoListo = grupoListo && hm.etapa[i] >= 3;
  }
  if (hm.grupo < hm.nGrupos && grupoListo) hm.grupo++;
  if (hm.grupo < hm.nGrupos) return;
  for (int i = 0; i < N; i++)
    if (hm.etapa[i] != 3) return;
  hm.activo = false;
  msg(">> Homing listo: cero en los flancos, J1 en %+.1f°, J2 en %+.1f°, J3 en %+.1f°.", HOMING_RETROCESO,
      HOMING_RETROCESO_J2, HOMING_RETROCESO_J3);
  msg("HOMING,%lu,%.3f,%.3f,%.3f,%d%d%d", (unsigned long)millis(), hm.delta[I1], hm.delta[I2], hm.delta[I3],
      hm.conCero[I1] ? 1 : 0, hm.conCero[I2] ? 1 : 0, hm.conCero[I3] ? 1 : 0);
}

// ---------- Fase 5b: movimiento coordinado ("mv") ----------
// Etapas: 1 recorrido (hasta APROX_GRADOS[i] antes del objetivo; J2 y J3 terminan juntas, J1 con su duración), 2 aproximación (el último tramo despacio,
// subiendo; cada una con su duración, J1 más corta), 3 llegada (las tres sostienen el objetivo hasta quedar firmes), 4 asentamiento.
struct {
  bool activo = false;
  int etapa = 0;
  float desde[N], hasta[N], obj[N];
  bool mueve[N];       // en la etapa: esta se mueve
  bool aprox[N];       // hace la aproximación (todas las que se mueven fuera de su tolPos)
  float antes[N];      // dónde termina su recorrido: APROX_GRADOS antes del objetivo, del lado del que llega
  bool esperar[N];     // al final del recorrido, esperar a que esté de verdad de ese lado
  float T = 1.0f;      // s, duración de la etapa
  float Ti[N];         // s, la de cada una (T = la más larga): en la aproximación todas; en el recorrido, las de
                       // MV_DURACION_PROPIA (las demás, T)
  uint32_t k = 0;      // periodos desde el comienzo de la etapa
  uint32_t tEtapa = 0, tFirme = 0;
  // Camino en dos tramos (2026-11-04, franja con tabla): este mv va a un punto intermedio, sin aproximación ni
  // asentamiento, y al llegar la referencia arranca solo el mv a "final" (el único que da LLEGO)
  bool intermedio = false;
  float final[N];
} mv;

// Duración del perfil quíntico para que ninguna pase su velocidad ni su aceleración (con el factor de "mv"):
// pico de velocidad 1,875·D/T, pico de aceleración 5,774·D/T²
// "vel <factor> [aprox]" (2026-11-11): velocidad de los mv en vivo, para medir el tiempo del ciclo sin reflashear
float mvFactorVel = MV_FACTOR_VEL;
float velAproxJ23 = VEL_APROX[1];
uint32_t asentamientoMs = ASENTAMIENTO_MS;

float duracion(const float d[N], float factorVel, float velFija) {
  float T = 0.0f;
  for (int i = 0; i < N; i++) {
    const float D = fabsf(d[i]);
    if (D < 1e-3f) continue;
    const float v = velFija > 0.0f ? velFija : factorVel * art[i].cfg().vMax;
    float a = factorVel * art[i].cfg().aMax;
    if (velFija <= 0.0f && MV_ACEL_MAX[i] > 0.0f) a = fminf(a, MV_ACEL_MAX[i]);  // recorrido: tope propio
    T = fmaxf(T, fmaxf(1.875f * D / v, sqrtf(5.774f * D / a)));
  }
  return T;
}

void mvEtapa(int etapa) {
  mv.etapa = etapa;
  mv.k = 0;
  mv.tEtapa = millis();
  float d[N];
  for (int i = 0; i < N; i++) {
    mv.desde[i] = art[i].referencia();
    if (etapa == 1) {
      mv.hasta[i] = mv.aprox[i] ? mv.antes[i] : mv.obj[i];
    } else {
      mv.hasta[i] = mv.obj[i];
    }
    d[i] = mv.hasta[i] - mv.desde[i];
    mv.mueve[i] = (etapa == 1 || etapa == 2) && fabsf(d[i]) >= 0.01f;
    if (!mv.mueve[i]) d[i] = 0.0f;
  }
  if (etapa == 1) {
    // Las de duración propia, cada una con la suya; las demás juntas (la más lenta manda)
    float dj[N];
    for (int i = 0; i < N; i++) dj[i] = MV_DURACION_PROPIA[i] ? 0.0f : d[i];
    const float Tj = fmaxf(duracion(dj, mvFactorVel, 0.0f), MV_T_MIN);
    mv.T = Tj;
    for (int i = 0; i < N; i++) {
      mv.Ti[i] = Tj;
      if (!MV_DURACION_PROPIA[i] || !mv.mueve[i]) continue;
      float di[N] = {0.0f, 0.0f, 0.0f};
      di[i] = d[i];
      mv.Ti[i] = fmaxf(duracion(di, mvFactorVel, 0.0f), MV_T_MIN);
      mv.T = fmaxf(mv.T, mv.Ti[i]);
    }
  } else if (etapa == 2) {
    mv.T = 0.0f;
    for (int i = 0; i < N; i++) {
      float di[N] = {0.0f, 0.0f, 0.0f};
      di[i] = d[i];
      mv.Ti[i] = duracion(di, 1.0f, i == 0 ? VEL_APROX[0] : velAproxJ23);
      mv.T = fmaxf(mv.T, mv.Ti[i]);
    }
  } else {
    for (int i = 0; i < N; i++) mv.Ti[i] = mv.T;
  }
  if (etapa >= 3)
    for (int i = 0; i < N; i++) art[i].sostener(mv.obj[i]);
}

void mvCortar(const char *causa) {
  // Todas se quedan donde están: las que se movían, frenadas; las que ya sostenían, en su lugar actual (si no,
  // seguían yendo al objetivo del movimiento cortado)
  for (auto &a : art) {
    if (a.modo() == Articulacion::Modo::Seguir || (a.modo() == Articulacion::Modo::Sostener && !a.llego()))
      a.frenarAqui();
    else if (a.modo() == Articulacion::Modo::Sostener)
      a.sostenerAqui();
  }
  mv.activo = false;
  msg(">> mv CORTADO: %s. Quedan sosteniendo donde están.", causa);
  msg("MV_ERROR,%s", causa);
}

void mvIniciar(const float obj[N], bool intermedio = false, const float *final = nullptr) {
  enAlto();
  mv.intermedio = intermedio;
  if (intermedio)
    for (int i = 0; i < N; i++) mv.final[i] = final[i];
  for (int i = 0; i < N; i++) {
    art[i].reiniciarEstado();
    if (art[i].modo() != Articulacion::Modo::Sostener || !art[i].llego()) art[i].sostenerAqui();
    mv.obj[i] = obj[i];
    // Todas las que se mueven llegan con la aproximación (2026-10-31): subiendo directo, J2 llegaba adelantada al
    // frenar y quedaba 0,25° arriba; con el último tramo lento desde abajo, 0,00°. La que ya está dentro de su tolPos
    // no se mueve (2026-11-03, antes 0,05°: en la 1b J2 a 0,137° hacía 2° abajo y vuelta en cada mv de J1, se trababa
    // y se pasaba 0,72°; sostener() con el mismo objetivo no hace nada)
    mv.aprox[i] = !intermedio && fabsf(obj[i] - art[i].q()) > art[i].cfg().tolPos;
    // Del lado del que llega: desde abajo (+1) o, con APROX_MISMO_SENTIDO, del lado del que viene. Si en el mismo
    // sentido está más cerca que el tramo, no retrocede: hace todo en la aproximación (si no, invertía el sentido)
    const float lado = (APROX_MISMO_SENTIDO[i] && obj[i] < art[i].q()) ? -1.0f : 1.0f;
    mv.antes[i] = obj[i] - lado * APROX_GRADOS[i];
    // Las que llegan en el mismo sentido no esperan a estar "del lado": no importa de qué lado queden, solo no
    // invertir (2026-11-03: la vuelta a mv 3 3 3 cortó porque J1 quedó 0,42° corta de su punto previo)
    mv.esperar[i] = mv.aprox[i] && !APROX_MISMO_SENTIDO[i];
    if (APROX_MISMO_SENTIDO[i] && fabsf(obj[i] - art[i].q()) <= APROX_GRADOS[i]) mv.antes[i] = art[i].referencia();
    // El punto previo no sale del margen de los límites (2026-11-03: en la vuelta a "mv 3 3 3" J2 y J3 bajaban a 1°,
    // a 1° del final de carrera, y subían de nuevo): si saldría, va directo al objetivo, sin aproximación
    const ConfigArticulacion &c = art[i].cfg();
    if (mv.aprox[i] && (mv.antes[i] < c.limNeg + MARGEN_LIMITE || mv.antes[i] > c.limPos - MARGEN_LIMITE)) {
      mv.aprox[i] = false;
      mv.esperar[i] = false;
    }
  }
  mv.activo = true;
  bool algunaAprox = false;
  for (bool b : mv.aprox) algunaAprox = algunaAprox || b;
  mvEtapa(1);
  msg(">> mv %s(%+.2f, %+.2f, %+.2f) en %.1f s%s.", intermedio ? "primero a " : "a ", obj[I1], obj[I2], obj[I3],
      mv.T, algunaAprox ? " + la aproximación" : "");
}

void mvPaso() {
  if (!mv.activo) return;
  for (int i = 0; i < N; i++) {
    if (art[i].error()) {
      mvCortar(art[i].error());
      return;
    }
  }
  if (mv.etapa == 1 || mv.etapa == 2) {
    mv.k++;
    const float tau = fminf(mv.k * Articulacion::TS / fmaxf(mv.T, 1e-3f), 1.0f);
    for (int i = 0; i < N; i++) {
      // Todas, también las quietas: ninguna se puede alejar más de MV_ERROR_MAX de su referencia
      if (fabsf(art[i].referencia() - art[i].q()) > MV_ERROR_MAX) {
        mvCortar("no sigue la trayectoria (más de 5° de error)");
        return;
      }
      if (!mv.mueve[i]) continue;
      // Perfil quíntico con la duración de esta (en el recorrido, J2 y J3 la misma: terminan juntas)
      const float Ti = fmaxf(mv.Ti[i], 1e-3f);
      const float ti = fminf(mv.k * Articulacion::TS / Ti, 1.0f);
      if (ti >= 1.0f && Ti < mv.T) {  // terminó su parte antes que las otras: a sostener (J1 frena)
        if (art[i].modo() == Articulacion::Modo::Seguir) art[i].sostener(mv.hasta[i]);
        continue;
      }
      // Aproximación: si se despega y llega (o cruza) antes que la referencia, sostiene el objetivo (freno anticipado,
      // tAnticipo; J2 desde el 2026-11-05). J1 (autobloqueo) no: lo hace en su llegada. Ya sostenida: no seguir más
      if (mv.etapa == 2 && !art[i].cfg().autobloqueo) {
        if (art[i].modo() == Articulacion::Modo::Sostener) continue;
        if (art[i].anticipar(mv.obj[i], mv.antes[i] <= mv.obj[i] ? 1 : -1)) continue;
      }
      const float t2 = ti * ti, t3 = t2 * ti;
      const float sp = t3 * (10.0f - 15.0f * ti + 6.0f * t2);           // posición normalizada
      const float vp = 30.0f * t2 * (1.0f - 2.0f * ti + t2) / Ti;       // su derivada (1/s)
      const float D = mv.hasta[i] - mv.desde[i];
      art[i].seguir(mv.desde[i] + D * sp, D * vp);
    }
    if (tau >= 1.0f) {
      if (mv.etapa == 1) {
        // Antes de subir el último tramo, todas las que se aproximan tienen que estar de verdad por debajo del objetivo (si no,
        // la llegada podía terminar siendo desde arriba): esperar en el punto previo, hasta 3 s
        for (int i = 0; i < N; i++) {
          // todavía no está del lado del que llega (al menos APROX_ABAJO_MIN antes del objetivo; antes la mitad del tramo)
          const float lado = mv.antes[i] <= mv.obj[i] ? 1.0f : -1.0f;
          if (mv.esperar[i] && lado * (mv.obj[i] - art[i].q()) < APROX_ABAJO_MIN) {
            if (mv.k * Articulacion::TS > mv.T + 3.0f) {
              mvCortar("no bajó por debajo del objetivo para llegar desde abajo");
            }
            return;  // la referencia ya está en el punto previo (tau = 1): sigue ahí
          }
        }
        bool algunaAprox = false;
        for (bool b : mv.aprox) algunaAprox = algunaAprox || b;
        mvEtapa(algunaAprox ? 2 : 3);
      } else {
        mvEtapa(3);
      }
      // Las que terminaron su parte se quedan donde las dejó la trayectoria (sostener sin salto)
      for (int i = 0; i < N; i++)
        if (!mv.mueve[i] && art[i].modo() == Articulacion::Modo::Seguir) art[i].sostener(art[i].referencia());
      // Tramo intermedio: con la referencia en el punto intermedio, el segundo tramo arranca de ahí (sin esperar)
      if (mv.intermedio) {
        float fin[N];
        for (int i = 0; i < N; i++) fin[i] = mv.final[i];
        mvIniciar(fin);
      }
    }
    return;
  }
  // 3: las tres sostienen el objetivo hasta quedar firmes; 4: firmes ASENTAMIENTO_MS
  bool firmes = true;
  for (auto &a : art) firmes = firmes && a.firme();
  if (mv.etapa == 3) {
    if (firmes) {
      mv.etapa = 4;
      mv.tFirme = millis();
    } else if (millis() - mv.tEtapa > MV_LLEGADA_MAX_MS) {
      mvCortar("no quedan firmes");
    }
    return;
  }
  if (!firmes) {
    mv.etapa = 3;
    return;
  }
  if (millis() - mv.tFirme < asentamientoMs) return;
  mv.activo = false;
  float q[N];
  for (int i = 0; i < N; i++) q[i] = art[i].q();
  msg(">> Llegó: J1 %+.2f° J2 %+.2f° J3 %+.2f° (errores %+.2f %+.2f %+.2f°).", q[I1], q[I2], q[I3],
      mv.obj[I1] - q[I1], mv.obj[I2] - q[I2], mv.obj[I3] - q[I3]);
  msg("LLEGO,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", (unsigned long)millis(), q[I1], q[I2], q[I3], mv.obj[I1] - q[I1],
      mv.obj[I2] - q[I2], mv.obj[I3] - q[I3]);
}

// Exceso de un mv de J2/J3 de s a o: la recta hasta el punto previo de la aproximación (APROX_GRADOS abajo, si esa
// articulación la hace, como en mvIniciar()) y de ahí el último tramo
float excesoMv(float s2, float s3, float o2, float o3, float paso = 0.25f) {
  const float s[2] = {s2, s3}, o[2] = {o2, o3};
  float pr[2];
  for (int k = 0; k < 2; k++) {
    const int i = I2 + k;
    const ConfigArticulacion &c = art[i].cfg();
    const bool aprox = fabsf(o[k] - s[k]) > c.tolPos && o[k] - APROX_GRADOS[i] >= c.limNeg + MARGEN_LIMITE;
    pr[k] = aprox ? o[k] - APROX_GRADOS[i] : o[k];
  }
  return fmaxf(excesoTramo(s2, s3, pr[0], pr[1], paso), excesoTramo(pr[0], pr[1], o2, o3, paso));
}

// Punto intermedio P para ir de s a o en dos tramos dentro de la franja (s -> P sin aproximación, P -> o con ella).
// Grilla de 5° más las dos esquinas (primero J2 o primero J3); el de camino más corto. ~1-10 ms
bool buscarIntermedio(float s2, float s3, float o2, float o3, float tolerado, float &p2, float &p3) {
  const ConfigArticulacion &c2 = art[I2].cfg(), &c3 = art[I3].cfg();
  const float mn2 = c2.limNeg + MARGEN_LIMITE, mx2 = c2.limPos - MARGEN_LIMITE;
  const float mn3 = c3.limNeg + MARGEN_LIMITE, mx3 = c3.limPos - MARGEN_LIMITE;
  float mejor = INFINITY;
  const auto probar = [&](float x, float y) {
    if (x < mn2 || x > mx2 || y < mn3 || y > mx3 || excesoFranja(x, y) > -MARGEN_FRANJA ||
        excesoCaja(q1Camino, x, y) > -MARGEN_FRANJA)
      return;
    const float largo = fmaxf(fabsf(x - s2), fabsf(y - s3)) + fmaxf(fabsf(o2 - x), fabsf(o3 - y));
    if (largo >= mejor) return;
    // los dos tramos con MARGEN_CAMINO (el primero, con lo que se tolere si arranca más cerca del borde)
    if (excesoTramo(s2, s3, x, y, 1.0f) > tolerado || excesoMv(x, y, o2, o3, 1.0f) > -MARGEN_CAMINO) return;
    if (excesoTramo(s2, s3, x, y) > tolerado || excesoMv(x, y, o2, o3) > -MARGEN_CAMINO) return;
    mejor = largo;
    p2 = x;
    p3 = y;
  };
  probar(o2, s3);  // primero J2
  probar(s2, o3);  // primero J3
  for (float x = 0.0f; x <= mx2; x += 5.0f)
    for (float y = 0.0f; y <= mx3; y += 5.0f) probar(x, y);
  return isfinite(mejor);
}

// "mv <q1> <q2> <q3>": valida el destino (límites con margen, franja con MARGEN_FRANJA) y el camino de J2 y J3 (recta
// hasta el punto previo de la aproximación y el último tramo, como en mvIniciar()) dentro de la franja
void comandoMv(const char *args) {
  float obj[N];
  if (sscanf(args, "%f %f %f", &obj[I1], &obj[I2], &obj[I3]) != 3) {
    msg("Uso: mv <q1> <q2> <q3>  (grados)");
    return;
  }
  for (int i = 0; i < N; i++) {
    if (!cero[i]) {
      msg("%s sin cero: primero \"h\".", art[i].cfg().nombre);
      return;
    }
    const ConfigArticulacion &c = art[i].cfg();
    if (!isfinite(obj[i]) || obj[i] < c.limNeg + MARGEN_LIMITE || obj[i] > c.limPos - MARGEN_LIMITE) {
      msg("Fuera de rango: %s entre %+.1f° y %+.1f°.", c.nombre, c.limNeg + MARGEN_LIMITE, c.limPos - MARGEN_LIMITE);
      return;
    }
  }
  if (excesoDestino(obj[I2], obj[I3]) > -MARGEN_FRANJA) {
    msg("Fuera de la franja: con J2 en %+.1f°, J3 entre %+.1f° y %+.1f° (con margen; abajo, J3 firme).", obj[I2],
        q3MinDestino(obj[I2]) + MARGEN_FRANJA, q3MaxFranja(obj[I2]) - MARGEN_FRANJA);
    return;
  }
  if (excesoCaja(obj[I1], obj[I2], obj[I3]) > -MARGEN_FRANJA) {
    msg("Sobre la caja (J1 desde %.0f°): con J2 en %+.1f°, J3 hasta %+.1f° (con margen).", CAJA_J1_MIN, obj[I2],
        q3MaxCaja(obj[I2]) - MARGEN_FRANJA);
    return;
  }
  if (fueraFranjaAhora()) {
    msg("J2 y J3 fuera de la franja ahora (%.2f° afuera): volver adentro con \"j\" antes.", excesoFranjaAhora());
    return;
  }
  const float s2 = art[I2].q(), s3 = art[I3].q();
  // Caja en todo el camino con la J1 más alta del mv; si ya arranca con J3 alta para esa J1, no hay camino seguro
  // (J1 puede llegar antes que J3 baje): primero bajar J3 con "j" o mover J1 por separado
  q1Camino = fmaxf(art[I1].q(), obj[I1]);
  struct Restaurar {
    ~Restaurar() { q1Camino = -INFINITY; }
  } restaurar;
  if (excesoCaja(q1Camino, s2, s3) > -MARGEN_CAMINO) {
    msg("Sobre la caja: con J1 hasta %+.1f°, J3 tiene que estar debajo de %+.1f° antes de empezar. Bajar J3 con \"j\" "
        "(o mover J1 sola).", q1Camino, q3MaxCaja(s2) - MARGEN_CAMINO);
    return;
  }
  // Todo el camino a MARGEN_CAMINO del borde; si arranca más cerca, que no se acerque más (2026-11-05)
  const float tolerado = fmaxf(-MARGEN_CAMINO, excesoFranjaAhora());
  if (excesoMv(s2, s3, obj[I2], obj[I3]) <= tolerado) {
    mvIniciar(obj);
    return;
  }
  // La recta sale de la franja (con J2 bajo es muy angosta): buscar un punto intermedio P (J1 quieta en el primer
  // tramo) con los dos tramos adentro y el camino más corto. Primero cada 1° en una grilla de 5°; el elegido, cada 0,25°
  float p2, p3;
  if (!buscarIntermedio(s2, s3, obj[I2], obj[I3], tolerado, p2, p3)) {
    msg("Fuera de la franja: el camino de J2 y J3 sale y no encontré un punto intermedio. Ir en pasos (\"j\").");
    return;
  }
  const float p[N] = {art[I1].q(), p2, p3};
  msg(">> El camino recto sale de la franja: pasa por J2 %+.1f° J3 %+.1f°.", p2, p3);
  mvIniciar(p, true, obj);
}

// ---------- Comandos (los ejecuta la tarea) ----------

void ayuda() {
  msg("Comandos (escribir y Enter):");
  msg("  h            homing: las tres (abajo, juntas) a sus finales de carrera despacio, cero en el flanco, retroceden %.0f°",
      HOMING_RETROCESO);
  msg("  j <n> <°>    mover una articulación (1, 2 o 3) a un ángulo, dentro de sus límites y de la franja");
  msg("  mv <q1> <q2> <q3>  las tres a la vez, terminan juntas (recta en q2-q3), llegan con el ángulo creciendo;");
  msg("               al asentarse: LLEGO,t,q1,q2,q3,e1,e2,e3. Caja: con J1 >= %.0f°, J3 con tope según J2",
      CAJA_J1_MIN);
  msg("  jv <a1> <a2> <a3>  joystick: cada eje (-1 a 1) es la velocidad de su articulación; repetir cada");
  msg("               < %lu ms (si no, frena); al soltar se sostiene. tools/joystick.py --placa 20",
      (unsigned long)JOY_HOMBRE_MUERTO_MS);
  msg("  t [1|0]      telemetría a 50 Hz (T,ms,etapa,q,ref,duty,w,estado de cada una); no corta el mv ni el homing");
  msg("  s            sostener las tres donde están");
  msg("  z            cero de las tres acá (tienen que estar en sus finales de carrera; mejor \"h\")");
  msg("  c            estado de las tres");
  msg("  fr <1|0>     franja del cuatro barras prendida / apagada (apagada solo para medirla con el joystick)");
  msg("  fb           freno dentro de la banda sí/no (al llegar, duty 0 hasta que se aleje)");
  msg("  ff <n> <desp+> <marcha+> <desp-> <marcha->  feedforward de una articulación (%%)");
  msg("  zq <q1> <q2> <q3>  el brazo está en estos ángulos (cero estimado, para bajarlo sin cero; después \"h\")");
  msg("  x            rueda libre de las TRES (el brazo cae; mano debajo)");
  msg("  vel <f> [va] [as]  mv: fracción de vMax/aMax (def. %.2f), aproximación de J2/J3 (def. %.1f °/s), asentamiento (def. %.1f s)",
      MV_FACTOR_VEL, VEL_APROX[1], ASENTAMIENTO_MS / 1000.0f);
  msg("  pz a|c|0|s|e|<us>|w <°/s>  pinza: abrir (500 us) / cerrar (1250) / cero (1500) / soltar / estado / pulso / rampa");
  msg("  mn a|b|0|s|e|<us>|w <°/s>  muñeca: 90° horario (860 us) / antihorario (2150) / cero (1500) / ...");
  msg("               pz y mn no cortan un mv; arrancan sueltos (el primero salta directo); SERVO_LLEGO al llegar");
  msg("  ?            esta ayuda");
}

void estado() {
  for (int i = 0; i < N; i++) {
    const Articulacion &a = art[i];
    static const char *const MODOS[] = {"SUELTA", "sosteniendo", "siguiendo", "HOMING"};
    msg("%s %+8.2f° | ref %+8.2f° | duty %+6.1f %% | %s%s%s | final de carrera %s | %s", a.cfg().nombre, a.q(),
        a.referencia(), a.duty(), MODOS[(int)a.modo()], a.llego() ? (a.firme() ? ", firme" : ", llegó") : "",
        a.frenada() ? ", frenada" : "", a.fc() ? "apretado" : "suelto", cero[i] ? "con cero" : "SIN CERO");
  }
  if (cerosJ2J3())
    msg("   franja: con J2 en %+.2f°, J3 entre %+.1f° y %+.1f° (%+.2f° del borde); destinos desde %+.1f°", art[I2].q(),
        q3MinCamino(art[I2].q()), q3MaxFranja(art[I2].q()), -excesoFranjaAhora(),
        q3MinDestino(art[I2].q()) + MARGEN_FRANJA);
  if (cero[I1] && cerosJ2J3())
    msg("   caja: J1 desde %.0f°: con J2 en %+.2f°, J3 hasta %+.1f°%s", CAJA_J1_MIN, art[I2].q(),
        q3MaxCaja(art[I2].q()), art[I1].q() >= CAJA_J1_MIN ? " (J1 sobre la caja: rige)" : " (J1 al costado: no rige)");
}

// Rango permitido para la articulación i con las otras donde están (o yendo a): límites propios con MARGEN_LIMITE y
// franja de los destinos (borde de abajo Q2_MENOS_Q3_DESTINO) con MARGEN_FRANJA
// La caja en el rango de una (2026-11-10 noche). Las otras, en el peor caso entre su medido y su referencia
void rangoCaja(int i, float &mn, float &mx) {
  const auto peor = [](int k, float &lo, float &hi) {
    lo = fminf(art[k].q(), art[k].referencia());
    hi = fmaxf(art[k].q(), art[k].referencia());
  };
  float q1lo, q1hi, q2lo, q2hi, q3lo, q3hi;
  peor(I1, q1lo, q1hi);
  peor(I2, q2lo, q2hi);
  peor(I3, q3lo, q3hi);
  if (i == I1) {
    // J1 no entra sobre la caja si J3 está más alta que la tabla (0,3° de tolerancia: J3 quieta en el borde del
    // joystick no tiene que bloquear a J1 por el ruido de la medición)
    if (q3hi > q3MaxCajaEntre(q2lo, q2hi) - MARGEN_FRANJA + 0.3f) mx = fminf(mx, CAJA_J1_MIN - 0.5f);
    return;
  }
  if (q1hi < CAJA_J1_MIN - 1.0f) return;  // al costado (1° antes: J1 puede estar llegando)
  if (i == I3) {
    mx = fminf(mx, q3MaxCajaEntre(q2lo, q2hi) - MARGEN_FRANJA);
    return;
  }
  // J2: desde donde está, hasta donde la tabla deje a J3 (su valor más alto) con margen, de a 0,25°
  const float lim = q3hi + MARGEN_FRANJA, q2 = art[I2].q();
  float a = q2, b = q2;
  while (a + 0.25f <= mx && q3MaxCaja(a + 0.25f) >= lim) a += 0.25f;
  while (b - 0.25f >= mn && q3MaxCaja(b - 0.25f) >= lim) b -= 0.25f;
  mx = fminf(mx, a);
  mn = fmaxf(mn, b);
}

bool rango(int i, float &mn, float &mx) {
  const ConfigArticulacion &c = art[i].cfg();
  mn = c.limNeg + MARGEN_LIMITE;
  mx = c.limPos - MARGEN_LIMITE;
  if ((i == I2 || i == I3) && !cerosJ2J3()) return false;
  if ((i == I2 || i == I3) && franjaActiva) {
    const int o = i == I2 ? I3 : I2;
    // La otra, en el peor caso entre su medido y su referencia
    const float qo1 = art[o].q(), qo2 = art[o].referencia();
    const float lo = fminf(qo1, qo2), hi = fmaxf(qo1, qo2);
    // (la tabla del borde de arriba crece: el peor caso de la otra es su valor más alto o más bajo)
    if (i == I2) {  // q2 entre q2MinFranja(q3) y q3 + Q2_MENOS_Q3_DESTINO
      mn = fmaxf(mn, q2MinFranja(hi + MARGEN_FRANJA));
      mx = fminf(mx, fminf(lo + Q2_MENOS_Q3_DESTINO, q2MaxTabla(lo)) - MARGEN_FRANJA);
    } else {        // q3 entre q2 - Q2_MENOS_Q3_DESTINO y q3MaxFranja(q2)
      mn = fmaxf(mn, q3MinDestino(hi) + MARGEN_FRANJA);
      mx = fminf(mx, q3MaxFranja(lo) - MARGEN_FRANJA);
    }
  }
  if (cero[I1] && cerosJ2J3()) rangoCaja(i, mn, mx);
  return true;
}

void comandoMover(const char *args) {
  int n = 0;
  float g = 0.0f;
  if (sscanf(args, "%d %f", &n, &g) != 2 || n < 1 || n > 3 || !isfinite(g)) {
    msg("Uso: j <1|2|3> <grados>");
    return;
  }
  const int i = n - 1;
  if (!cero[i]) {
    msg("%s sin cero: primero \"h\" (o \"z\" en los finales de carrera).", art[i].cfg().nombre);
    return;
  }
  float mn, mx;
  if (!rango(i, mn, mx)) {
    msg("J2 y J3 necesitan cero las dos (la franja depende de las dos): \"h\".");
    return;
  }
  if (g < mn || g > mx) {
    msg("Fuera de rango: %s puede ir de %+.1f° a %+.1f° (límites con %.0f° de margen y franja con %.0f°).",
        art[i].cfg().nombre, mn, mx, MARGEN_LIMITE, MARGEN_FRANJA);
    return;
  }
  enAlto();
  for (int k = 0; k < N; k++) {  // de a una: las otras se quedan donde están (también si iban a otro lado)
    art[k].reiniciarEstado();
    if (k != i && (art[k].modo() == Articulacion::Modo::Suelta || enMovimiento(k))) art[k].sostenerAqui();
  }
  if (art[i].modo() == Articulacion::Modo::Suelta) art[i].sostenerAqui();
  art[i].sostener(g);
  msg(">> %s hacia %+.2f°.", art[i].cfg().nombre, g);
}

// ---------- Joystick ("jv"): velocidad de cada una en lazo cerrado ----------
struct {
  bool activo = false;
  float a[N] = {};        // ejes pedidos (-1 a 1)
  float w[N] = {};        // °/s de la referencia (aceleración limitada)
  float ref[N] = {};      // referencia de las que se mueven
  bool mueve[N] = {};     // siguiendo la referencia del joystick
  uint32_t tUltimo = 0;   // millis() del último "jv"
  bool bloqueado = false; // cortado por una protección: no se reactiva hasta un "jv 0 0 0" (palanca soltada)
} joy;

// Las que se movían con el joystick: sostener donde están (otro comando) o frenadas (un corte)
void joyParar(bool frenar) {
  for (int i = 0; i < N; i++) {
    if (joy.mueve[i]) frenar ? art[i].frenarAqui() : art[i].sostenerAqui();
    joy.mueve[i] = false;
    joy.a[i] = joy.w[i] = 0.0f;
  }
  joy.activo = false;
  // Un corte (frenar) no se rearma con el siguiente "jv": con la palanca sostenida volvía a empujar contra el mismo
  // obstáculo y reiniciarEstado() borraba el error (revisión de Codex, 2026-11-03). Hay que soltarla
  if (frenar) joy.bloqueado = true;
}

// Lee los tres ejes de "jv" (con la zona muerta y entre -1 y 1); false si no son tres números
bool joyEjes(const char *args, float e[N]) {
  if (sscanf(args, "%f %f %f", &e[I1], &e[I2], &e[I3]) != 3) return false;
  for (int i = 0; i < N; i++) {
    if (!isfinite(e[i])) return false;
    e[i] = fabsf(e[i]) < JOY_ZONA_MUERTA ? 0.0f : constrain(e[i], -1.0f, 1.0f);
  }
  return true;
}

void comandoJoy(const char *args) {
  float e[N];
  static uint32_t tAviso = 0;  // el puente manda "jv" 10 veces por segundo: los avisos, cada 2 s como mucho
  const bool avisar = millis() - tAviso > 2000;
  if (!joyEjes(args, e)) {
    if (avisar) msg("Uso: jv <a1> <a2> <a3>  (cada eje de -1 a 1; repetir cada < %lu ms)",
                    (unsigned long)JOY_HOMBRE_MUERTO_MS);
    tAviso = millis();
    return;
  }
  if (!cero[I1] || !cero[I2] || !cero[I3]) {
    if (avisar) msg("Joystick: alguna sin cero, primero \"h\".");
    tAviso = millis();
    return;
  }
  if (!joy.activo) {
    enAlto();
    for (int k = 0; k < N; k++) {
      art[k].reiniciarEstado();
      // También las que iban a otro lado (un "j" en curso): si no, la del eje centrado seguía sola hacia su destino
      // aunque se soltara la palanca (revisión de Codex, 2026-11-03)
      if (art[k].modo() == Articulacion::Modo::Suelta || enMovimiento(k)) art[k].sostenerAqui();
    }
    joy.activo = true;
  }
  for (int i = 0; i < N; i++) joy.a[i] = e[i];
  joy.tUltimo = millis();
}

// Un período: la referencia de cada una avanza a la velocidad del eje (acelera y frena con JOY_FACTOR_ACEL·aMax) y no
// pasa los límites ni la franja con margen (frena a tiempo para quedar en el borde); al soltar, sostiene donde quedó
void joyPaso() {
  if (!joy.activo) return;
  const bool vivo = millis() - joy.tUltimo <= JOY_HOMBRE_MUERTO_MS;
  bool alguna = false;
  for (int i = 0; i < N; i++) {
    if (art[i].error()) {
      joyParar(true);
      msg(">> Joystick cortado: %s %s. Quedan sosteniendo donde están.", art[i].cfg().nombre, art[i].error());
      return;
    }
    // Otra parte del control la sacó de la trayectoria (la franja la frena): no insistir
    if (joy.mueve[i] && art[i].modo() != Articulacion::Modo::Seguir) {
      joyParar(true);
      msg(">> Joystick cortado: %s dejó de seguir (¿franja?). Quedan sosteniendo donde están.", art[i].cfg().nombre);
      return;
    }
    const ConfigArticulacion &c = art[i].cfg();
    const float wObj = (vivo ? joy.a[i] : 0.0f) * JOY_FACTOR_VEL * c.vMax;
    if (!joy.mueve[i]) {
      if (wObj == 0.0f) continue;
      joy.mueve[i] = true;
      joy.ref[i] = art[i].q();
      joy.w[i] = 0.0f;
    }
    float mn, mx;
    rango(i, mn, mx);  // con las tres con cero siempre da true
    const float acel = JOY_FACTOR_ACEL * c.aMax;
    float w = constrain(wObj, joy.w[i] - acel * Articulacion::TS, joy.w[i] + acel * Articulacion::TS);
    // Frenar a tiempo: con la aceleración acel, la velocidad que todavía para antes del borde
    if (w > 0.0f) w = fminf(w, sqrtf(fmaxf(2.0f * acel * (mx - joy.ref[i]), 0.0f)));
    if (w < 0.0f) w = fmaxf(w, -sqrtf(fmaxf(2.0f * acel * (joy.ref[i] - mn), 0.0f)));
    // Con la MEDICIÓN de J2/J3 ya fuera de la franja (moviéndose la otra, la referencia de esta puede seguir adentro):
    // solo hacia adentro, y la referencia no queda del lado de afuera de la medición (si no, el PI seguía empujando
    // hacia afuera sin llegar a los 5° de error que cortan; revisión de Codex, 2026-11-03)
    if ((i == I2 || i == I3) && fueraFranjaAhora()) {
      // Codo demasiado estirado (lado +1): J3 hacia abajo y J2 hacia arriba; hombro demasiado arriba: al revés
      int lado = 0;
      excesoFranjaAhora(&lado);
      const float adentro = (lado > 0 ? -1.0f : 1.0f) * (i == I3 ? 1.0f : -1.0f);
      if (w * adentro < 0.0f) w = 0.0f;
      if ((joy.ref[i] - art[i].q()) * adentro < 0.0f) joy.ref[i] = art[i].q();
    }
    joy.w[i] = w;
    joy.ref[i] += w * Articulacion::TS;
    if (fabsf(joy.ref[i] - art[i].q()) > MV_ERROR_MAX) {
      joyParar(true);
      msg(">> Joystick cortado: %s no sigue (más de %.0f° de error). Quedan sosteniendo donde están.", c.nombre,
          MV_ERROR_MAX);
      return;
    }
    if (wObj == 0.0f && w == 0.0f) {  // soltada y ya frenada: sostener donde quedó la referencia
      art[i].sostener(joy.ref[i]);
      joy.mueve[i] = false;
      continue;
    }
    art[i].seguir(joy.ref[i], w);
    alguna = true;
  }
  if (!alguna && !vivo) joy.activo = false;
}

void comandoFF(const char *args) {
  int n = 0;
  float v[4];
  if (sscanf(args, "%d %f %f %f %f", &n, &v[0], &v[1], &v[2], &v[3]) != 5 || n < 1 || n > 3) {
    msg("Uso: ff <1|2|3> <desp+> <marcha+> <desp-> <marcha->  (%%)");
    return;
  }
  for (float x : v) {
    if (!(x >= 0.0f && x <= 70.0f)) {  // así también rechaza nan (con NaN toda comparación da falso)
      msg("Cada valor entre 0 y 70 %%.");
      return;
    }
  }
  art[n - 1].ajustarFeedforward(v[0], v[1], v[2], v[3]);
  msg(">> %s feedforward: despegue %+.1f / -%.1f %%, marcha %+.1f / -%.1f %% (hasta reiniciar; pasar a config.h).",
      art[n - 1].cfg().nombre, v[0], v[2], v[1], v[3]);
}

// "zq q1 q2 q3": el brazo está en estos ángulos (cero ESTIMADO). Para sacarlo con "j" o el joystick cuando quedó
// lejos de los finales de carrera sin cero (al abrir el puerto la WROOM se reinicia y lo pierde; el brazo se queda
// donde estaba). Después, abajo y recogido, "h" toma el cero de verdad en los flancos.
void comandoZq(const char *args) {
  float g[N];
  if (sscanf(args, "%f %f %f", &g[I1], &g[I2], &g[I3]) != 3) {
    msg("Uso: zq <q1> <q2> <q3>  (grados donde está el brazo ahora)");
    return;
  }
  for (int i = 0; i < N; i++) {
    const ConfigArticulacion &c = art[i].cfg();
    if (!isfinite(g[i]) || g[i] < c.limNeg || g[i] > c.limPos) {
      msg("zq: %s entre %+.0f° y %+.0f°.", c.nombre, c.limNeg, c.limPos);
      return;
    }
  }
  for (int i = 0; i < N; i++) {
    art[i].fijarAngulo(g[i]);
    cero[i] = true;
  }
  msg(">> Cero ESTIMADO: J1 %+.1f° J2 %+.1f° J3 %+.1f°. Bajar el brazo (j o joystick) y hacer \"h\".", g[I1], g[I2],
      g[I3]);
  fueraFranja = fueraFranjaAhora();  // ya avisado acá: que la tarea no frene el primer "j" que vuelve adentro
  if (fueraFranja)
    msg("   Fuera de la franja (%.1f° afuera): solo deja volver hacia adentro.", excesoFranjaAhora());
}

void ejecutar(char *l) {
  while (*l == ' ') l++;
  if (*l == '\0') return;
  const bool noCorta = strcmp(l, "c") == 0 || l[0] == 't';  // "c" y "t" no cortan nada
  const bool esJoy = strncmp(l, "jv ", 3) == 0;
  if (esJoy && joy.bloqueado) {  // después de un corte del joystick: hasta soltar la palanca, nada
    float e[N];
    if (joyEjes(l + 3, e) && e[I1] == 0.0f && e[I2] == 0.0f && e[I3] == 0.0f) {
      joy.bloqueado = false;
      msg(">> Joystick listo otra vez.");
      return;
    }
    static uint32_t tAviso = 0;
    if (millis() - tAviso > 2000) {
      msg("Joystick bloqueado por el corte: soltar la palanca (centro) para seguir.");
      tAviso = millis();
    }
    return;
  }
  if (esJoy && !joy.activo) {  // el puente manda "jv 0 0 0" siempre: con el joystick en el centro no corta nada
    float e[N];
    if (joyEjes(l + 3, e) && e[I1] == 0.0f && e[I2] == 0.0f && e[I3] == 0.0f) return;
  }
  if (joy.activo && !noCorta && !esJoy) joyParar(false);  // otro comando: las del joystick se sostienen donde están
  if (hm.activo && !noCorta) {  // cualquier otro comando corta el homing
    homingCortar("comando");
    if (strcmp(l, "x") != 0) return;
  }
  if (mv.activo && !noCorta) {  // y el movimiento coordinado
    mvCortar("comando");
    if (strcmp(l, "x") != 0) return;
  }
  if (strcmp(l, "?") == 0) {
    ayuda();
  } else if (strcmp(l, "x") == 0) {
    for (auto &a : art) a.rueda();
    msg(">> Rueda libre: las tres sueltas (el brazo cae).");
  } else if (!hwOk) {
    msg("Hardware sin inicializar: comandos deshabilitados.");
  } else if (strcmp(l, "c") == 0) {
    estado();
  } else if (l[0] == 't' && (l[1] == '\0' || l[1] == ' ')) {
    const char *a = l + 1;
    while (*a == ' ') a++;
    tele = *a == '1' ? true : (*a == '0' ? false : !tele);
    msg(">> Telemetría %s.", tele ? "PRENDIDA (50 Hz)" : "APAGADA");
    if (tele) {
      telePerdidas = 0;
      msg("TELE_CFG,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", art[I1].cfg().tolPos, art[I2].cfg().tolPos, art[I3].cfg().tolPos,
          art[I1].cfg().bandaInt, art[I2].cfg().bandaInt, art[I3].cfg().bandaInt);
    } else {
      msg("TELE_FIN,%lu", (unsigned long)telePerdidas);
    }
  } else if (strcmp(l, "h") == 0) {
    homingIniciar();
  } else if (strcmp(l, "s") == 0) {
    enAlto();
    for (auto &a : art) {
      a.reiniciarEstado();
      a.sostenerAqui();
    }
    msg(">> Sosteniendo las tres donde están.");
  } else if (strcmp(l, "z") == 0) {
    for (int i = 0; i < N; i++) {
      art[i].ceroAqui();
      cero[i] = true;
    }
    msg(">> Cero de las tres acá. Finales de carrera: J1 %s, J2 %s, J3 %s.", art[I1].fc() ? "apretado" : "SUELTO",
        art[I2].fc() ? "apretado" : "SUELTO", art[I3].fc() ? "apretado" : "SUELTO");
    if (!art[I1].fc() || !art[I2].fc() || !art[I3].fc())
      msg("   OJO: alguno suelto. El cero bueno es en los finales de carrera: mejor \"h\".");
    enAlto();
    for (auto &a : art) a.sostenerAqui();
  } else if (strcmp(l, "fr 0") == 0 || strcmp(l, "fr 1") == 0) {
    franjaActiva = l[3] == '1';
    fueraFranja = fueraFranjaAhora();
    msg(">> Franja del cuatro barras %s.", franjaActiva ? "PRENDIDA" : "APAGADA: J2 y J3 solo con sus límites propios "
        "(para medirla de nuevo con el joystick; mano en la fuente). \"fr 1\" la prende");
  } else if (strcmp(l, "fb") == 0) {
    frenoBanda = !frenoBanda;
    for (auto &a : art) a.frenoEnBanda(frenoBanda);
    msg(">> Freno dentro de la banda %s (las tres).", frenoBanda ? "PRENDIDO" : "APAGADO");
  } else if (l[0] == 'j' && l[1] == ' ') {
    comandoMover(l + 2);
  } else if (esJoy) {
    comandoJoy(l + 3);
  } else if (strncmp(l, "mv ", 3) == 0) {
    comandoMv(l + 3);
  } else if (strncmp(l, "zq ", 3) == 0) {
    comandoZq(l + 3);
  } else if (strncmp(l, "ff ", 3) == 0) {
    comandoFF(l + 3);
  } else if (strncmp(l, "vel", 3) == 0 && (l[3] == ' ' || l[3] == '\0')) {
    float f = mvFactorVel, va = velAproxJ23, as = asentamientoMs / 1000.0f;
    const int n = sscanf(l + 3, "%f %f %f", &f, &va, &as);
    if (n >= 1 && (f < 0.2f || f > 1.0f || (n >= 2 && (va < 1.0f || va > 6.0f)) || (n == 3 && (as < 0.0f || as > 5.0f)))) {
      msg("Uso: vel <factor 0.2-1.0> [aprox J2/J3 1-6 °/s] [asentamiento 0-5 s]");
    } else {
      mvFactorVel = f;
      velAproxJ23 = va;
      asentamientoMs = (uint32_t)(as * 1000.0f + 0.5f);
      msg(">> mv a %.2f de vMax/aMax (J2/J3 %.0f °/s, J1 %.0f °/s); aproximación de J2/J3 a %.1f °/s; asentamiento "
          "%.1f s. Se pierde al reiniciar (config.h: %.2f, %.1f, %.1f).", mvFactorVel, mvFactorVel * CFG_J2.vMax,
          mvFactorVel * CFG_J1.vMax, velAproxJ23, asentamientoMs / 1000.0f, MV_FACTOR_VEL, VEL_APROX[1],
          ASENTAMIENTO_MS / 1000.0f);
    }
  } else {
    msg("Comando desconocido: \"%s\" (? para la ayuda)", l);
  }
}

// ---------- Tarea de control: 100 Hz ----------
void control(void *) {
  TickType_t ultimo = xTaskGetTickCount();
  uint32_t perdidos = 0;
  for (;;) {
    if (xTaskDelayUntil(&ultimo, pdMS_TO_TICKS(10)) == pdFALSE) perdidos++;
    Linea ln;
    if (xPendiente) {  // "x" manda: lo anterior en la cola se descarta (podía volver a subir el EN)
      while (xQueueReceive(colaComandos, &ln, 0) == pdTRUE) {
      }
      xPendiente = false;
      char x[] = "x";
      ejecutar(x);
    }
    while (xQueueReceive(colaComandos, &ln, 0) == pdTRUE) ejecutar(ln.t);
    // La referencia con la que paso() calcula el duty de este período (mvPaso() ya pone la del siguiente): es la que
    // va a la telemetría, si no ref − q tenía un adelanto falso de 10 ms (~0,1° a 10 °/s)
    float refUsada[N];
    for (int i = 0; i < N; i++) {
      refUsada[i] = art[i].referencia();
      art[i].paso();
    }
    homingPaso();
    mvPaso();
    joyPaso();
    static uint32_t kTele = 0;
    if (tele && ++kTele % TELE_DIV == 0) {
      Muestra m;
      m.ms = millis();
      m.etapa = mv.activo ? (uint8_t)mv.etapa : (hm.activo ? 9 : (joy.activo ? 8 : 0));
      for (int i = 0; i < N; i++) {
        m.q[i] = art[i].q();
        m.r[i] = refUsada[i];
        m.u[i] = art[i].duty();
        m.w[i] = art[i].w();
        m.est[i] = estadoTele(art[i]);
      }
      if (xQueueSend(colaTele, &m, 0) != pdTRUE) telePerdidas++;
    }
    // Franja del cuatro barras: si la medición sale (con holgura) con J2 o J3 controladas (moviéndose o sosteniendo:
    // sosteniendo también pueden corregir una caída), frenar las dos donde están. Una vez por episodio.
    if (cerosJ2J3() && (art[I2].modo() != Articulacion::Modo::Suelta || art[I3].modo() != Articulacion::Modo::Suelta)) {
      const float d = excesoFranjaAhora();
      const bool fuera = d > FRANJA_HOLGURA;
      // Al salir, y mientras siga afuera, cualquier movimiento de J2/J3 que la aleje se frena (no solo la primera vez).
      // Un "j" que vuelve adentro sí se permite (su destino está adentro: lo valida rango()).
      // Las que mueve el joystick no: joyPaso() ya no las deja alejarse (solo volver hacia adentro, 2026-11-03)
      const auto mueveSola = [](int i) {
        return !joy.mueve[i] && (art[i].modo() == Articulacion::Modo::Homing ||
                                 art[i].modo() == Articulacion::Modo::Seguir);
      };
      const bool alejando = fuera && (mueveSola(I2) || mueveSola(I3));
      if ((fuera && !fueraFranja) || alejando) {
        if (hm.activo) homingCortar("saldría de la franja del cuatro barras");
        if (mv.activo) mvCortar("saldría de la franja del cuatro barras");
        if (!joy.mueve[I2]) art[I2].frenarAqui();
        if (!joy.mueve[I3]) art[I3].frenarAqui();
        if (!fueraFranja)
          msg(">> FRANJA: %.2f° fuera de la franja (J2 %+.2f°, J3 %+.2f°). J2 y J3 frenadas donde están: volver con "
              "\"j\".", d, art[I2].q(), art[I3].q());
      }
      fueraFranja = fuera;
    }
    // Caja: si la medición sale (con holgura) estando J1 sobre la caja, frenar las tres (las del joystick no: rango()
    // ya no las deja alejarse, solo volver). Una vez por episodio
    if (cero[I1] && cerosJ2J3()) {
      const float d = excesoCaja(art[I1].q(), art[I2].q(), art[I3].q());
      const bool fuera = d > FRANJA_HOLGURA;
      if (fuera && !fueraCaja) {
        if (hm.activo) homingCortar("bajaría sobre la caja");
        if (mv.activo) mvCortar("bajaría sobre la caja");
        for (int i = 0; i < N; i++)
          if (!joy.mueve[i] && art[i].modo() != Articulacion::Modo::Suelta) art[i].frenarAqui();
        msg(">> CAJA: J3 %.2f° más alta que lo permitido sobre la caja (J1 %+.2f°, J2 %+.2f°, J3 %+.2f°). Frenadas: "
            "bajar J3 o volver J1 con \"j\".", d, art[I1].q(), art[I2].q(), art[I3].q());
      }
      fueraCaja = fuera;
    }
    // Cortes y avisos de cada una (una vez)
    for (int i = 0; i < N; i++) {
      const char *e = art[i].error();
      if (e && e != ultimoError[i] && !hm.activo) msg(">> %s CORTADA: %s. Sostiene donde está (%+.2f°).",
                                                     art[i].cfg().nombre, e, art[i].q());
      ultimoError[i] = e;
      if (const char *av = art[i].aviso()) msg(">> %s: %s.", art[i].cfg().nombre, av);
    }
    (void)perdidos;
  }
}

// ---------- Serial (loop) ----------
char linea[48];
size_t lineaLen = 0;
bool ultimoFueCR = false;

void leerSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' && ultimoFueCR) {
      ultimoFueCR = false;
      continue;
    }
    ultimoFueCR = ch == '\r';
    if (ch == 0x08 || ch == 0x7F) {
      if (lineaLen) lineaLen--;
      continue;
    }
    if (ch == '\r' || ch == '\n') {
      linea[lineaLen] = '\0';
      Linea l;
      strncpy(l.t, linea, sizeof(l.t));
      l.t[sizeof(l.t) - 1] = '\0';
      if (lineaLen && servosComando(l.t)) {
        // pinza y muñeca: se atienden acá (loop), no pasan por la tarea de control ni cortan un mv
      } else if (lineaLen) {
        // "x": el EN se baja acá mismo (no depende de la cola); la tarea después las pone sueltas
        if (strcmp(l.t, "x") == 0) {
          digitalWrite(PIN_BRAZO_EN, LOW);
          xPendiente = true;  // la tarea descarta la cola y suelta las tres
        } else if (xQueueSend(colaComandos, &l, pdMS_TO_TICKS(50)) != pdTRUE)
          Serial.printf("Comando perdido (la placa está ocupada): \"%s\". Repetirlo.\n", l.t);
      }
      lineaLen = 0;
    } else if (lineaLen < sizeof(linea) - 1 && ch >= ' ') {
      linea[lineaLen++] = ch;
    }
  }
}

}  // namespace

void setup() {
  // Primero todo en bajo: los PWM de las tres y el EN (con su pull-down hasta acá)
  for (int p : {PIN_BRAZO_EN, PIN_J1_RPWM, PIN_J1_LPWM, PIN_J2_RPWM, PIN_J2_LPWM, PIN_J3_RPWM, PIN_J3_LPWM}) {
    digitalWrite(p, LOW);
    pinMode(p, OUTPUT);
  }
  const ConfigArticulacion *cfg[N] = {&CFG_J1, &CFG_J2, &CFG_J3};
  hwOk = true;
  for (int i = 0; i < N; i++) {
    hwOk = art[i].begin(*cfg[i]) && hwOk;
    art[i].frenoEnBanda(frenoBanda);
  }
  Serial.setTxBufferSize(2048);  // la telemetría (~5 kB/s) no bloquea loop() en cada línea
  Serial.begin(115200);
  delay(1500);
  colaComandos = xQueueCreate(16, sizeof(Linea));
  colaMensajes = xQueueCreate(24, sizeof(Mensaje));
  colaTele = xQueueCreate(50, sizeof(Muestra));  // 1 s de muestras
  Serial.println();
  Serial.println("== 20 · J1, J2 y J3 con el mismo controlador (5a: homing, \"j\"; 5b: \"mv\"; 5c: \"t\") · WROOM, 20 kHz ==");
  Serial.printf("Franja (manivela de 95 mm): q2 - %.1f° <= q3 <= tabla (destinos desde q2 - %.1f°):", Q2_MENOS_Q3_MAX,
                Q2_MENOS_Q3_DESTINO);
  for (const PuntoFranja &p : FRANJA_ARRIBA) Serial.printf(" %.0f->%.1f", p.q2, p.q3Max);
  Serial.printf("; caja con J1 >= %.0f°, J3 máx:", CAJA_J1_MIN);
  for (const PuntoFranja &p : CAJA_ARRIBA) Serial.printf(" %.0f->%.1f", p.q2, p.q3Max);
  Serial.printf("; abajo, además, tabla:");
  for (const PuntoFranjaAbajo &p : FRANJA_ABAJO) Serial.printf(" %.0f->%.1f", p.q2, p.q3Min);
  Serial.printf(". J1 0-%.0f° (PROVISORIO: estirado choca antes), J2 0-%.0f°, J3 0-%.1f°.\n", CFG_J1.limPos,
                CFG_J2.limPos, CFG_J3.limPos);
  if (!servosIniciar()) Serial.println("ERROR: no se pudo configurar el LEDC de la pinza o de la muñeca.");
  if (!hwOk) Serial.println("ERROR: no se pudo configurar algún PWM o encoder.");
  Serial.println("Las tres sueltas (EN en bajo) hasta \"h\", \"s\" o \"z\". Para el cero: \"h\".");
  while (Serial.available()) Serial.read();  // basura del CH340 al abrir el puerto
  xTaskCreatePinnedToCore(control, "control", 6144, nullptr, 5, nullptr, 1);
  Linea l = {"?"};
  xQueueSend(colaComandos, &l, 0);
}

void loop() {
  leerSerial();
  servosActualizar();
  Mensaje m;
  while (xQueueReceive(colaMensajes, &m, 0) == pdTRUE) Serial.println(m.t);
  Muestra x;
  while (xQueueReceive(colaTele, &x, 0) == pdTRUE) {
    char b[200];
    int n = snprintf(b, sizeof(b), "T,%lu,%u", (unsigned long)x.ms, x.etapa);
    for (int i = 0; i < N; i++)
      n += snprintf(b + n, sizeof(b) - n, ",%.3f,%.3f,%.1f,%.2f,%u", x.q[i], x.r[i], x.u[i], x.w[i], x.est[i]);
    Serial.println(b);
  }
  delay(2);
}
