// 20 · J1, J2 y J3 con el MISMO controlador (lib/Articulacion), en la ESP32-WROOM-32D con el conexionado del robot.
// Fase 5a (ver README.md): un solo lazo NO bloqueante a 100 Hz (tarea "control") que sostiene las tres, homing de
// las tres (J3 -> J2 -> J1) con el cero en el flanco, y movimientos de a una ("j") dentro de la franja del cuatro
// barras. Los comandos llegan por Serial (loop) a una cola; la tarea los ejecuta entre periodos y deja los mensajes
// en otra cola que imprime loop(): la tarea nunca espera a Serial.
// Fase 5b: "mv q1 q2 q3": las tres a la vez, terminan juntas, camino recto en (q2, q3) dentro de la franja, llegada
// siempre con el ángulo creciendo; al asentarse, línea LLEGO. Pendiente: 5c telemetría; zona prohibida (la caja).
#include <Arduino.h>

#include "Articulacion.h"
#include "config.h"
#include "pins.h"

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
float dFranja() { return art[I3].q() - art[I2].q(); }  // q3 - q2, entre -Q2_MENOS_Q3_MAX y +Q3_MENOS_Q2_MAX
bool enMovimiento(int i) {
  const auto m = art[i].modo();
  return m == Articulacion::Modo::Seguir || m == Articulacion::Modo::Homing ||
         (m == Articulacion::Modo::Sostener && !art[i].llego());
}

// ---------- Homing de las tres: J3 -> J2 -> J1 (con el codo abajo, bajar el hombro no viola la franja; J1 al final,
// con el brazo recogido) ----------
struct {
  bool activo = false;
  int paso = 0;          // índice en ORDEN
  int etapa = 0;         // 0 arrancar, 1 buscando, 2 retrocediendo
  uint32_t tEtapa = 0;   // millis() al empezar el retroceso
  bool conCero[N] = {};
  float delta[N] = {NAN, NAN, NAN};
} hm;
constexpr int ORDEN[N] = {I3, I2, I1};

void homingCortar(const char *causa) {
  for (auto &a : art) {
    a.homingCancelar();
    if (a.modo() != Articulacion::Modo::Suelta) a.sostenerAqui();
  }
  hm.activo = false;
  msg(">> Homing CORTADO: %s. Quedan sosteniendo donde están (repetir \"h\").", causa);
  msg("HOMING_ERROR,%s", causa);
}

bool fueraFranjaAhora() {
  const float d = dFranja();
  return cerosJ2J3() && (d < -Q2_MENOS_Q3_MAX - FRANJA_HOLGURA || d > Q3_MENOS_Q2_MAX + FRANJA_HOLGURA);
}

void homingIniciar() {
  if (fueraFranjaAhora()) {
    msg("Homing: J2 y J3 fuera de la franja (q3 - q2 = %+.2f°): volver adentro con \"j\" antes.", dFranja());
    return;
  }
  for (int i = 0; i < N; i++) hm.conCero[i] = cero[i];
  if (cero[I2] && art[I2].q() > HOMING_Q2_MAX_ARRANQUE) {
    msg("Homing: J2 en %+.1f°; más arriba de %.0f° el codo no llega a su final de carrera sin salir de la franja. "
        "Bajarla antes (j 2 10).", art[I2].q(), HOMING_Q2_MAX_ARRANQUE);
    return;
  }
  if (!cero[I1] || !cero[I2] || !cero[I3])
    msg(">> Alguna sin cero: el brazo tiene que estar abajo y recogido (cerca de los finales de carrera).");
  enAlto();
  for (auto &a : art) {  // todo quieto donde está antes de empezar (también lo que iba a otro destino)
    a.reiniciarEstado();
    a.sostenerAqui();
  }
  hm.activo = true;
  hm.paso = hm.etapa = 0;
  for (auto &d : hm.delta) d = NAN;
  msg(">> Homing: J3 (%.1f °/s), J2 (%.1f °/s), J1 (%.1f °/s) hacia sus finales de carrera. Cualquier comando corta.",
      CFG_J3.homingVelBuscar, CFG_J2.homingVelBuscar, CFG_J1.homingVelBuscar);
}

void homingPaso() {
  if (!hm.activo) return;
  const int i = ORDEN[hm.paso];
  Articulacion &a = art[i];
  if (hm.etapa == 0) {
    a.homing(hm.conCero[i]);
    hm.etapa = 1;
    msg("   %s buscando su final de carrera...", a.cfg().nombre);
    return;
  }
  if (hm.etapa == 1) {
    if (a.estadoHoming() == Articulacion::Homing::EnCurso) return;
    if (a.estadoHoming() != Articulacion::Homing::Listo) {
      homingCortar(a.error() ? a.error() : "error");
      return;
    }
    hm.delta[i] = a.flancoGrados();
    a.ceroEnFlanco();
    cero[i] = true;
    a.sostener(HOMING_RETROCESO);
    hm.etapa = 2;
    hm.tEtapa = millis();
    return;
  }
  // etapa 2: retrocediendo
  if (a.error()) {
    homingCortar(a.error());
    return;
  }
  if (!a.firme()) {
    if (millis() - hm.tEtapa > 15000) {
      msg("   %s no queda firme: en %+.2f° (pedido %+.2f°), duty %+.1f %%, w %+.2f °/s, %s.", a.cfg().nombre, a.q(),
          a.referencia(), a.duty(), a.w(), a.llego() ? "llegó pero no quieta" : "no llegó");
      homingCortar("no termina de retroceder (15 s)");
    }
    return;
  }
  msg("   %s: flanco %+.3f° del cero anterior%s; en %+.2f°.", a.cfg().nombre, hm.delta[i],
      hm.conCero[i] ? "" : " (no había cero)", a.q());
  if (++hm.paso < N) {
    hm.etapa = 0;
    return;
  }
  hm.activo = false;
  msg(">> Homing listo: cero en los flancos, las tres en %+.1f°.", HOMING_RETROCESO);
  msg("HOMING,%lu,%.3f,%.3f,%.3f,%d%d%d", (unsigned long)millis(), hm.delta[I1], hm.delta[I2], hm.delta[I3],
      hm.conCero[I1] ? 1 : 0, hm.conCero[I2] ? 1 : 0, hm.conCero[I3] ? 1 : 0);
}

// ---------- Fase 5b: movimiento coordinado ("mv") ----------
// Etapas: 1 recorrido (hasta el objetivo, o APROX_GRADOS por debajo para las que bajan), 2 aproximación (esas suben
// el último tramo despacio), 3 llegada (las tres sostienen el objetivo hasta quedar firmes), 4 asentamiento.
struct {
  bool activo = false;
  int etapa = 0;
  float desde[N], hasta[N], obj[N];
  bool mueve[N];       // en la etapa: esta se mueve
  bool aprox[N];       // llega desde abajo (bajaba)
  float T = 1.0f;      // s, duración de la etapa
  uint32_t k = 0;      // periodos desde el comienzo de la etapa
  uint32_t tEtapa = 0, tFirme = 0;
} mv;

// Duración del perfil quíntico para que ninguna pase su velocidad ni su aceleración (con el factor de "mv"):
// pico de velocidad 1,875·D/T, pico de aceleración 5,774·D/T²
float duracion(const float d[N], float factorVel, float velFija) {
  float T = 0.0f;
  for (int i = 0; i < N; i++) {
    const float D = fabsf(d[i]);
    if (D < 1e-3f) continue;
    const float v = velFija > 0.0f ? velFija : factorVel * art[i].cfg().vMax;
    const float a = factorVel * art[i].cfg().aMax;
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
      mv.hasta[i] = mv.aprox[i] ? mv.obj[i] - APROX_GRADOS : mv.obj[i];
    } else {
      mv.hasta[i] = mv.obj[i];
    }
    d[i] = mv.hasta[i] - mv.desde[i];
    mv.mueve[i] = (etapa == 1 || etapa == 2) && fabsf(d[i]) >= 0.01f;
    if (!mv.mueve[i]) d[i] = 0.0f;
  }
  if (etapa == 1) mv.T = fmaxf(duracion(d, MV_FACTOR_VEL, 0.0f), MV_T_MIN);
  if (etapa == 2) mv.T = duracion(d, 1.0f, VEL_APROX);
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

void mvIniciar(const float obj[N]) {
  enAlto();
  for (int i = 0; i < N; i++) {
    art[i].reiniciarEstado();
    if (art[i].modo() != Articulacion::Modo::Sostener || !art[i].llego()) art[i].sostenerAqui();
    mv.obj[i] = obj[i];
    mv.aprox[i] = obj[i] < art[i].q() - 0.05f;  // baja: llega desde abajo
  }
  mv.activo = true;
  bool algunaAprox = false;
  for (bool b : mv.aprox) algunaAprox = algunaAprox || b;
  mvEtapa(1);
  msg(">> mv a (%+.2f, %+.2f, %+.2f) en %.1f s%s.", obj[I1], obj[I2], obj[I3], mv.T,
      algunaAprox ? " + la aproximación desde abajo" : "");
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
    const float t2 = tau * tau, t3 = t2 * tau;
    const float sp = t3 * (10.0f - 15.0f * tau + 6.0f * t2);                 // posición normalizada
    const float vp = 30.0f * t2 * (1.0f - 2.0f * tau + t2) / fmaxf(mv.T, 1e-3f);  // su derivada (1/s)
    for (int i = 0; i < N; i++) {
      // Todas, también las quietas: ninguna se puede alejar más de MV_ERROR_MAX de su referencia
      if (fabsf(art[i].referencia() - art[i].q()) > MV_ERROR_MAX) {
        mvCortar("no sigue la trayectoria (más de 5° de error)");
        return;
      }
      if (!mv.mueve[i]) continue;
      const float D = mv.hasta[i] - mv.desde[i];
      art[i].seguir(mv.desde[i] + D * sp, D * vp);
    }
    if (tau >= 1.0f) {
      if (mv.etapa == 1) {
        // Antes de subir el último tramo, las que bajan tienen que estar de verdad por debajo del objetivo (si no,
        // la llegada podía terminar siendo desde arriba): esperar en el punto previo, hasta 3 s
        for (int i = 0; i < N; i++) {
          if (mv.aprox[i] && art[i].q() > mv.obj[i] - 0.5f * APROX_GRADOS) {
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
  if (millis() - mv.tFirme < ASENTAMIENTO_MS) return;
  mv.activo = false;
  float q[N];
  for (int i = 0; i < N; i++) q[i] = art[i].q();
  msg(">> Llegó: J1 %+.2f° J2 %+.2f° J3 %+.2f° (errores %+.2f %+.2f %+.2f°).", q[I1], q[I2], q[I3],
      mv.obj[I1] - q[I1], mv.obj[I2] - q[I2], mv.obj[I3] - q[I3]);
  msg("LLEGO,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", (unsigned long)millis(), q[I1], q[I2], q[I3], mv.obj[I1] - q[I1],
      mv.obj[I2] - q[I2], mv.obj[I3] - q[I3]);
}

// "mv <q1> <q2> <q3>": valida el destino (límites con margen, franja con margen entre los destinos de J2 y J3)
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
  const float d = obj[I3] - obj[I2];
  if (d < -Q2_MENOS_Q3_MAX + MARGEN_LIMITE || d > Q3_MENOS_Q2_MAX - MARGEN_LIMITE) {
    msg("Fuera de la franja: q3 - q2 = %+.1f°, tiene que estar entre %+.1f° y %+.1f° (con margen).", d,
        -Q2_MENOS_Q3_MAX + MARGEN_LIMITE, Q3_MENOS_Q2_MAX - MARGEN_LIMITE);
    return;
  }
  if (fueraFranjaAhora()) {
    msg("J2 y J3 fuera de la franja ahora (q3 - q2 = %+.2f°): volver adentro con \"j\" antes.", dFranja());
    return;
  }
  mvIniciar(obj);
}

// ---------- Comandos (los ejecuta la tarea) ----------

void ayuda() {
  msg("Comandos (escribir y Enter):");
  msg("  h            homing: J3, J2 y J1 a sus finales de carrera despacio, cero en el flanco, retroceden %.0f°",
      HOMING_RETROCESO);
  msg("  j <n> <°>    mover una articulación (1, 2 o 3) a un ángulo, dentro de sus límites y de la franja");
  msg("  mv <q1> <q2> <q3>  las tres a la vez, terminan juntas (recta en q2-q3), llegan con el ángulo creciendo;");
  msg("               al asentarse: LLEGO,t,q1,q2,q3,e1,e2,e3. Ojo: no esquiva la caja (zona prohibida pendiente)");
  msg("  s            sostener las tres donde están");
  msg("  z            cero de las tres acá (tienen que estar en sus finales de carrera; mejor \"h\")");
  msg("  c            estado de las tres");
  msg("  fb           freno dentro de la banda sí/no (al llegar, duty 0 hasta que se aleje)");
  msg("  ff <n> <desp+> <marcha+> <desp-> <marcha->  feedforward de una articulación (%%)");
  msg("  x            rueda libre de las TRES (el brazo cae; mano debajo)");
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
  if (cerosJ2J3()) msg("   franja: q3 - q2 = %+.2f° (entre %+.1f° y %+.1f°)", dFranja(), -Q2_MENOS_Q3_MAX, Q3_MENOS_Q2_MAX);
}

// Rango permitido para la articulación i con las otras donde están (o yendo a): límites propios y franja, con margen
bool rango(int i, float &mn, float &mx) {
  const ConfigArticulacion &c = art[i].cfg();
  mn = c.limNeg + MARGEN_LIMITE;
  mx = c.limPos - MARGEN_LIMITE;
  if (i == I2 || i == I3) {
    if (!cerosJ2J3()) return false;
    const int o = i == I2 ? I3 : I2;
    // La otra, en el peor caso entre su medido y su referencia
    const float qo1 = art[o].q(), qo2 = art[o].referencia();
    const float lo = fminf(qo1, qo2), hi = fmaxf(qo1, qo2);
    if (i == I2) {  // q2 entre q3 - Q3_MENOS_Q2_MAX y q3 + Q2_MENOS_Q3_MAX
      mn = fmaxf(mn, hi - Q3_MENOS_Q2_MAX + MARGEN_LIMITE);
      mx = fminf(mx, lo + Q2_MENOS_Q3_MAX - MARGEN_LIMITE);
    } else {        // q3 entre q2 - Q2_MENOS_Q3_MAX y q2 + Q3_MENOS_Q2_MAX
      mn = fmaxf(mn, hi - Q2_MENOS_Q3_MAX + MARGEN_LIMITE);
      mx = fminf(mx, lo + Q3_MENOS_Q2_MAX - MARGEN_LIMITE);
    }
  }
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
    msg("Fuera de rango: %s puede ir de %+.1f° a %+.1f° (límites y franja, con %.0f° de margen).", art[i].cfg().nombre,
        mn, mx, MARGEN_LIMITE);
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

void comandoFF(const char *args) {
  int n = 0;
  float v[4];
  if (sscanf(args, "%d %f %f %f %f", &n, &v[0], &v[1], &v[2], &v[3]) != 5 || n < 1 || n > 3) {
    msg("Uso: ff <1|2|3> <desp+> <marcha+> <desp-> <marcha->  (%%)");
    return;
  }
  for (float x : v) {
    if (x < 0.0f || x > 70.0f) {
      msg("Cada valor entre 0 y 70 %%.");
      return;
    }
  }
  art[n - 1].ajustarFeedforward(v[0], v[1], v[2], v[3]);
  msg(">> %s feedforward: despegue %+.1f / -%.1f %%, marcha %+.1f / -%.1f %% (hasta reiniciar; pasar a config.h).",
      art[n - 1].cfg().nombre, v[0], v[2], v[1], v[3]);
}

void ejecutar(char *l) {
  while (*l == ' ') l++;
  if (*l == '\0') return;
  if (hm.activo && strcmp(l, "c") != 0) {  // cualquier comando corta el homing (menos "c")
    homingCortar("comando");
    if (strcmp(l, "x") != 0) return;
  }
  if (mv.activo && strcmp(l, "c") != 0) {  // y el movimiento coordinado
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
  } else if (strcmp(l, "fb") == 0) {
    frenoBanda = !frenoBanda;
    for (auto &a : art) a.frenoEnBanda(frenoBanda);
    msg(">> Freno dentro de la banda %s (las tres).", frenoBanda ? "PRENDIDO" : "APAGADO");
  } else if (l[0] == 'j' && l[1] == ' ') {
    comandoMover(l + 2);
  } else if (strncmp(l, "mv ", 3) == 0) {
    comandoMv(l + 3);
  } else if (strncmp(l, "ff ", 3) == 0) {
    comandoFF(l + 3);
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
    for (auto &a : art) a.paso();
    homingPaso();
    mvPaso();
    // Franja del cuatro barras: si la medición sale (con holgura) con J2 o J3 controladas (moviéndose o sosteniendo:
    // sosteniendo también pueden corregir una caída), frenar las dos donde están. Una vez por episodio.
    static bool fueraFranja = false;
    if (cerosJ2J3() && (art[I2].modo() != Articulacion::Modo::Suelta || art[I3].modo() != Articulacion::Modo::Suelta)) {
      const float d = dFranja();
      const bool fuera = d < -Q2_MENOS_Q3_MAX - FRANJA_HOLGURA || d > Q3_MENOS_Q2_MAX + FRANJA_HOLGURA;
      // Al salir, y mientras siga afuera, cualquier movimiento de J2/J3 que la aleje se frena (no solo la primera vez).
      // Un "j" que vuelve adentro sí se permite (su destino está adentro: lo valida rango()).
      const bool alejando = fuera && (art[I2].modo() == Articulacion::Modo::Homing ||
                                      art[I3].modo() == Articulacion::Modo::Homing ||
                                      art[I2].modo() == Articulacion::Modo::Seguir ||
                                      art[I3].modo() == Articulacion::Modo::Seguir);
      if ((fuera && !fueraFranja) || alejando) {
        if (hm.activo) homingCortar("saldría de la franja del cuatro barras");
        if (mv.activo) mvCortar("saldría de la franja del cuatro barras");
        art[I2].frenarAqui();
        art[I3].frenarAqui();
        if (!fueraFranja)
          msg(">> FRANJA: q3 - q2 = %+.2f°, fuera de la franja. J2 y J3 frenadas donde están: volver con \"j\".", d);
      }
      fueraFranja = fuera;
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
      if (lineaLen) {
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
  Serial.begin(115200);
  delay(1500);
  colaComandos = xQueueCreate(16, sizeof(Linea));
  colaMensajes = xQueueCreate(24, sizeof(Mensaje));
  Serial.println();
  Serial.println("== 20 · J1, J2 y J3 con el mismo controlador (5a: homing, \"j\"; 5b: \"mv\") · WROOM, 20 kHz ==");
  Serial.printf("Franja: q2 - %.1f° <= q3 <= q2 + %.1f°. J1 0-%.0f° (PROVISORIO: estirado choca antes), J2 0-%.0f°, "
                "J3 0-%.0f°.\n", Q2_MENOS_Q3_MAX, Q3_MENOS_Q2_MAX, CFG_J1.limPos, CFG_J2.limPos, CFG_J3.limPos);
  if (!hwOk) Serial.println("ERROR: no se pudo configurar algún PWM o encoder.");
  Serial.println("Las tres sueltas (EN en bajo) hasta \"h\", \"s\" o \"z\". Para el cero: \"h\".");
  while (Serial.available()) Serial.read();  // basura del CH340 al abrir el puerto
  xTaskCreatePinnedToCore(control, "control", 6144, nullptr, 5, nullptr, 1);
  Linea l = {"?"};
  xQueueSend(colaComandos, &l, 0);
}

void loop() {
  leerSerial();
  Mensaje m;
  while (xQueueReceive(colaMensajes, &m, 0) == pdTRUE) Serial.println(m.t);
  delay(2);
}
