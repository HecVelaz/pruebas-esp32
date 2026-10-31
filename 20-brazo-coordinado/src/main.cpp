// 20 · J1, J2 y J3 con el MISMO controlador (lib/Articulacion), en la ESP32-WROOM-32D con el conexionado del robot.
// Fase 5a (ver README.md): un solo lazo NO bloqueante a 100 Hz (tarea "control") que sostiene las tres, homing de
// las tres (J3 -> J2 -> J1) con el cero en el flanco, y movimientos de a una ("j") dentro de la franja del cuatro
// barras. Los comandos llegan por Serial (loop) a una cola; la tarea los ejecuta entre periodos y deja los mensajes
// en otra cola que imprime loop(): la tarea nunca espera a Serial.
// Pendiente: 5b movimientos coordinados ("mv", llegada desde el mismo lado), 5c telemetría; zona prohibida (la caja).
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

void homingIniciar() {
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
    a.homing();
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
    if (millis() - hm.tEtapa > 15000) homingCortar("no termina de retroceder (15 s)");
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

// ---------- Comandos (los ejecuta la tarea) ----------

void ayuda() {
  msg("Comandos (escribir y Enter):");
  msg("  h            homing: J3, J2 y J1 a sus finales de carrera despacio, cero en el flanco, retroceden %.0f°",
      HOMING_RETROCESO);
  msg("  j <n> <°>    mover una articulación (1, 2 o 3) a un ángulo, dentro de sus límites y de la franja");
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
    while (xQueueReceive(colaComandos, &ln, 0) == pdTRUE) ejecutar(ln.t);
    for (auto &a : art) a.paso();
    homingPaso();
    // Franja del cuatro barras: si la medición sale (con holgura) con J2 o J3 controladas (moviéndose o sosteniendo:
    // sosteniendo también pueden corregir una caída), frenar las dos donde están. Una vez por episodio.
    static bool fueraFranja = false;
    if (cerosJ2J3() && (art[I2].modo() != Articulacion::Modo::Suelta || art[I3].modo() != Articulacion::Modo::Suelta)) {
      const float d = dFranja();
      const bool fuera = d < -Q2_MENOS_Q3_MAX - FRANJA_HOLGURA || d > Q3_MENOS_Q2_MAX + FRANJA_HOLGURA;
      if (fuera && !fueraFranja) {
        if (hm.activo) homingCortar("saldría de la franja del cuatro barras");
        art[I2].sostenerAqui();
        art[I3].sostenerAqui();
        msg(">> FRANJA: q3 - q2 = %+.2f°, fuera de la franja. J2 y J3 sostenidas donde están: volver con \"j\".", d);
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
        if (strcmp(l.t, "x") == 0) digitalWrite(PIN_BRAZO_EN, LOW);
        if (xQueueSend(colaComandos, &l, pdMS_TO_TICKS(50)) != pdTRUE)
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
  Serial.println("== 20 · J1, J2 y J3 con el mismo controlador (fase 5a: sostener, homing, \"j\") · WROOM, 20 kHz ==");
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
