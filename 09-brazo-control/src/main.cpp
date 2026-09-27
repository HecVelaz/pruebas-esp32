// Firmware del brazo (J1 base, J2 hombro, J3 codo + pinza) en la ESP32-WROOM-32D: control en cascada
// de posición y velocidad en lazo cerrado, con los drivers y el encoder de 06/08 y lib/ControlArticulacion.
//
// Esqueleto, primera versión: comandos por el monitor serial (115200). Más adelante el mismo control se
// comandará por micro-ROS (/brazo/joint_cmd, /brazo/joint_states), cambiando solo la capa de comandos.
//
// Estructura:
//   - tareaControl (núcleo 0, cada 10 ms): lee encoders, corre el control, aplica los duty. No usa Serial.
//   - loop (núcleo 1): comandos, mensajes y el CSV del comando log.
// Todo el estado compartido (ctl, homing, cortes, watchdog, log) se toca dentro de la sección crítica, y
// las marcas de tiempo se toman adentro, para que las dos tareas nunca resten un tiempo "del futuro".
#include <Arduino.h>

#include "BTS7960.h"
#include "ControlArticulacion.h"
#include "EncoderPCNT.h"
#include "L298N.h"
#include "brazo_config.h"
#include "esp_task_wdt.h"
#include "pins.h"

// Canales LEDC: de a pares comparten timer (misma frecuencia)
constexpr uint8_t CH_J1_R = 0, CH_J1_L = 1;    // 20 kHz
constexpr uint8_t CH_J2_A = 2, CH_J2_B = 3;    // 1 kHz
constexpr uint8_t CH_J3_A = 4, CH_J3_B = 5;    // 1 kHz
constexpr uint8_t CH_PINZA = 6, CH_GIRO = 7;   // 50 Hz

BTS7960 motorJ1;
L298N motorJ2, motorJ3;
EncoderPCNT enc[N_ART];
ControlArticulacion ctl[N_ART];
const int PIN_FC[N_ART] = {PIN_FC_J1, PIN_FC_J2, PIN_FC_J3};
bool hwOk = false;
bool servosOk = false;
bool servosArmados = false;  // los servos no reciben pulsos hasta el primer comando pinza/giro

// Estado compartido entre la tarea de control y el loop: se toca solo dentro de la sección crítica
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool ruedaLibre[N_ART] = {true, true, true};  // al arrancar los puentes quedan deshabilitados
uint32_t cortarEnMs[N_ART] = {0, 0, 0};       // fin del comando d / v (0 = sin corte)
bool homing[N_ART] = {false, false, false};
uint32_t homingDesdeMs[N_ART] = {0, 0, 0};
int64_t homingC0[N_ART] = {0, 0, 0};          // cuentas al empezar el homing (para el recorrido máximo)
int fcLecturas[N_ART] = {0, 0, 0};            // lecturas seguidas del final de carrera en bajo
uint32_t ultimoCmdMs = 0;
uint32_t watchdogMs = 0;  // 0 = desactivado
uint32_t ticksAtrasados = 0;

// CSV del comando log: la tarea deja muestras en una cola y el loop las imprime
struct Muestra {
  uint32_t tMs;
  float qObj, q, wRef, w, u;
};
QueueHandle_t colaLog;
int logArt = -1;  // articulación registrada (-1 = apagado)
uint32_t logCadaTicks = 1;

// Mensajes de la tarea al loop (sin Serial en la tarea)
enum class Evento : uint8_t { HomingOk, HomingTimeout, HomingRecorrido, HomingFalla, Watchdog };
struct Msg {
  Evento ev;
  uint8_t art;
};
QueueHandle_t colaMsg;

// ---------------------------------------------------------------- hardware

int64_t cuentas(int i) {
  const int64_t c = enc[i].count();
  return ENC_INVERTIDO[i] ? -c : c;
}

void aplicar(int i, float pct, bool libre) {
  const float d = pct / 100.0f;
  switch (i) {
    case 0: libre ? motorJ1.coast() : (void)motorJ1.setDuty(d); break;
    case 1: libre ? motorJ2.coast() : (void)motorJ2.setDuty(d); break;
    case 2: libre ? motorJ3.coast() : (void)motorJ3.setDuty(d); break;
  }
}

void servoUs(uint8_t ch, float us) {
  const float periodoUs = 1e6f / SERVO_HZ;
  ledcWrite(ch, (uint32_t)(us / periodoUs * ((1u << SERVO_BITS) - 1)));
}

// Los servos se conectan recién con el primer comando: al arrancar no se mueven solos
void armarServos() {
  if (servosArmados) return;
  ledcAttachPin(PIN_SERVO_PINZA, CH_PINZA);
  ledcAttachPin(PIN_SERVO_GIRO, CH_GIRO);
  servoUs(CH_PINZA, PINZA_US_ABIERTA);
  servoUs(CH_GIRO, GIRO_US_CENTRO);
  servosArmados = true;
}

bool finalDeCarrera(int i) { return PIN_FC[i] >= 0 && digitalRead(PIN_FC[i]) == LOW; }

// Cancela lo pendiente de una articulación (homing, corte de d/v). Llamar dentro de la sección crítica.
void cancelarPendientes(int i) {
  homing[i] = false;
  cortarEnMs[i] = 0;
  fcLecturas[i] = 0;
}

// ---------------------------------------------------------------- tarea de control

void tareaControl(void *) {
  esp_task_wdt_add(nullptr);
  TickType_t ultimo = xTaskGetTickCount();
  const TickType_t periodo = pdMS_TO_TICKS((uint32_t)(TS * 1000));
  int64_t tPrevUs = esp_timer_get_time();
  uint32_t tick = 0;
  for (;;) {
    // Si el paso anterior tardó más de un periodo, vTaskDelayUntil vuelve en el acto: contarlo
    const bool atrasado = xTaskGetTickCount() - ultimo >= periodo;
    vTaskDelayUntil(&ultimo, periodo);
    esp_task_wdt_reset();
    tick++;

    // dt real (no siempre TS): tras un atraso, la velocidad y el descarte de saltos siguen siendo correctos
    const int64_t tUs = esp_timer_get_time();
    float dt = (tUs - tPrevUs) * 1e-6f;
    tPrevUs = tUs;
    if (dt < 0.5f * TS) dt = 0.5f * TS;
    if (dt > 5.0f * TS) dt = 5.0f * TS;

    int64_t c[N_ART];
    bool fc[N_ART];
    for (int i = 0; i < N_ART; i++) {
      c[i] = cuentas(i);
      fc[i] = finalDeCarrera(i);
    }

    float u[N_ART];
    bool libre[N_ART];
    Msg msgs[2 * N_ART + 1];
    int nMsgs = 0;
    Muestra m{};
    bool hayMuestra = false;

    portENTER_CRITICAL(&mux);
    const uint32_t ahora = millis();  // adentro: el loop no puede haber escrito un tiempo posterior
    if (atrasado) ticksAtrasados++;

    // Watchdog de comandos: si el que manda (más adelante, la Pi) se calla, todo se detiene
    if (watchdogMs > 0 && ahora - ultimoCmdMs > watchdogMs) {
      bool algoSeMovia = false;
      for (int i = 0; i < N_ART; i++) {
        const bool moviendo = homing[i] || cortarEnMs[i] ||
                              (ctl[i].modo() == ControlArticulacion::Modo::Posicion && !ctl[i].llego());
        if (moviendo) {
          algoSeMovia = true;
          cancelarPendientes(i);
          ctl[i].detener();
        }
      }
      if (algoSeMovia) msgs[nMsgs++] = {Evento::Watchdog, 0};
      ultimoCmdMs = ahora;  // un aviso por vencimiento
    }

    for (int i = 0; i < N_ART; i++) {
      // Fin de los comandos d / v
      if (cortarEnMs[i] && (int32_t)(ahora - cortarEnMs[i]) >= 0) {
        cortarEnMs[i] = 0;
        ctl[i].detener();
      }
      // Homing: velocidad constante hasta el final de carrera (estable varias lecturas seguidas)
      if (homing[i]) {
        fcLecturas[i] = fc[i] ? fcLecturas[i] + 1 : 0;
        const float recorrido = fabsf((float)(c[i] - homingC0[i])) * 360.0f /
                                (CFG_ART[i].cuentasPorVuelta * CFG_ART[i].relacion);
        const float recorridoMax = CFG_ART[i].qMax - CFG_ART[i].qMin + HOME_MARGEN_DEG;
        Evento fin = Evento::HomingOk;
        bool termino = true;
        if (ctl[i].falla() != ControlArticulacion::Falla::Ninguna) fin = Evento::HomingFalla;
        else if (fcLecturas[i] >= HOME_FC_LECTURAS) fin = Evento::HomingOk;
        else if (recorrido > recorridoMax) fin = Evento::HomingRecorrido;
        else if (ahora - homingDesdeMs[i] > (uint32_t)(HOME_TIMEOUT_S * 1000)) fin = Evento::HomingTimeout;
        else termino = false;
        if (termino) {
          cancelarPendientes(i);
          if (fin == Evento::HomingOk) ctl[i].fijarCero(c[i], HOME_Q[i]);
          else if (fin != Evento::HomingFalla) ctl[i].detener();
          msgs[nMsgs++] = {fin, (uint8_t)i};
        }
      }
      u[i] = ctl[i].paso(c[i], dt);
      // Una falla cancela lo pendiente (por ejemplo, un homing que se atascó)
      if (ctl[i].falla() != ControlArticulacion::Falla::Ninguna && (homing[i] || cortarEnMs[i])) {
        if (homing[i]) msgs[nMsgs++] = {Evento::HomingFalla, (uint8_t)i};
        cancelarPendientes(i);
      }
      libre[i] = ruedaLibre[i] && ctl[i].modo() == ControlArticulacion::Modo::Libre;
    }
    if (logArt >= 0 && tick % logCadaTicks == 0) {
      const ControlArticulacion &k = ctl[logArt];
      m = {ahora, k.qObjetivo(), k.q(), k.wRef(), k.w(), k.u()};
      hayMuestra = true;
    }
    portEXIT_CRITICAL(&mux);

    for (int i = 0; i < N_ART; i++) aplicar(i, u[i], libre[i]);
    for (int k = 0; k < nMsgs; k++) xQueueSend(colaMsg, &msgs[k], 0);
    if (hayMuestra) xQueueSend(colaLog, &m, 0);  // si la cola está llena se pierde la muestra
  }
}

// ---------------------------------------------------------------- comandos

const char *textoModo(ControlArticulacion::Modo m) {
  switch (m) {
    case ControlArticulacion::Modo::Posicion: return "posicion";
    case ControlArticulacion::Modo::Velocidad: return "velocidad";
    case ControlArticulacion::Modo::Duty: return "duty";
    default: return "libre";
  }
}

void ayuda() {
  Serial.println();
  Serial.println("Comandos (articulaciones 1..3 = J1 base, J2 hombro, J3 codo; ángulos en grados):");
  Serial.println("  e               estado de las articulaciones");
  Serial.println("  z [n]           cero manual: el brazo ESTÁ en la pose de referencia (todas o la n)");
  Serial.println("  home <n>        homing con final de carrera, de a una articulación");
  Serial.println("  m <q1> <q2> <q3> mover las tres (posición). Ej: m 0 90 0");
  Serial.println("  j <n> <q>       mover una articulación. Ej: j 2 45");
  Serial.printf("  v <n> <rpm> [s] lazo de velocidad solo, rpm del motor (máx %.0f), por s segundos (def %.0f, máx %.0f)\n",
                VEL_CMD_MAX_RPM, VEL_CMD_S_DEF, VEL_CMD_S_MAX);
  Serial.printf("  d <n> <pct>     lazo abierto; se corta a los %lu ms si no se repite (J1 máx %.0f %%, J2/J3 %.0f %%)\n",
                (unsigned long)JOG_MS, JOG_DUTY_MAX[0], JOG_DUTY_MAX[1]);
  Serial.println("  s               detener: frena y, ya quieto, sostiene la posición");
  Serial.println("  l               liberar: puentes deshabilitados (rueda libre)");
  Serial.println("  r               borrar fallas (queda detenido)");
  Serial.println("  pinza <0-100>   0 = abierta, 100 = cerrada (el primer comando conecta los servos)");
  Serial.println("  giro <grados>   giro de la pinza (J4), -90..90");
  Serial.println("  log <n> [hz]    CSV de la articulación n (t_ms,q_obj,q,wref_rpm,w_rpm,u_pct); log 0 apaga");
  Serial.println("  wd <ms>         watchdog de comandos (0 = apagado; usarlo cuando mande la Pi)");
  Serial.println("  ?               esta ayuda");
  Serial.println("Mover (m, j) exige cero. Con falla la articulación queda frenada hasta r. Un comando nuevo");
  Serial.println("cancela lo pendiente de esa articulación (homing, d, v).");
  Serial.println();
}

void estado() {
  ControlArticulacion copia[N_ART];
  uint32_t atrasados;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < N_ART; i++) copia[i] = ctl[i];
  atrasados = ticksAtrasados;
  portEXIT_CRITICAL(&mux);
  for (int i = 0; i < N_ART; i++) {
    const ControlArticulacion &k = copia[i];
    const bool enPos = k.modo() == ControlArticulacion::Modo::Posicion;
    Serial.printf("J%d %-9s %s q=%7.2f obj=%7.2f w=%7.1f rpm wref=%7.1f u=%6.1f %% | falla: %s | saltos enc: %lu%s\n",
                  i + 1, textoModo(k.modo()), k.conCero() ? "cero" : "SIN CERO", k.q(), k.qObjetivo(), k.w(),
                  k.wRef(), k.u(), k.textoFalla(), (unsigned long)k.saltosEncoder(),
                  enPos && k.frenando() ? " | frenando" : (enPos && k.llego() ? " | llegó" : ""));
  }
  Serial.printf("Ticks atrasados: %lu | watchdog: %lu ms | servos: %s\n", (unsigned long)atrasados,
                (unsigned long)watchdogMs, servosArmados ? "conectados" : "sin conectar");
}

// Lee un número que ocupe todo el token. Rechaza NaN e infinito.
bool leerNum(const char *s, float &v) {
  if (!s) return false;
  char *fin = nullptr;
  v = strtof(s, &fin);
  return fin != s && *fin == '\0' && isfinite(v);
}

bool leerArt(const char *s, int &i) {
  float v;
  if (!leerNum(s, v) || v < 1 || v > N_ART || v != (int)v) {
    Serial.printf(">> Articulación inválida (1..%d).\n", N_ART);
    return false;
  }
  i = (int)v - 1;
  return true;
}

// Mensaje de rechazo con la falla leída bajo el candado (la tarea puede estar cambiándola)
void rechazo(int i, const char *cmd) {
  portENTER_CRITICAL(&mux);
  const ControlArticulacion::Falla f = ctl[i].falla();
  portEXIT_CRITICAL(&mux);
  if (f != ControlArticulacion::Falla::Ninguna)
    Serial.printf(">> J%d no acepta '%s': falla '%s' (r para borrarla).\n", i + 1, cmd, ControlArticulacion::texto(f));
  else
    Serial.printf(">> J%d no acepta '%s': falta el cero (z o home).\n", i + 1, cmd);
}

// Registra un comando aceptado para el watchdog. Llamar dentro de la sección crítica.
void alimentarWatchdog() { ultimoCmdMs = millis(); }

void ejecutar(char *linea) {
  char *cmd = strtok(linea, " ");
  if (!cmd) return;
  char *a1 = strtok(nullptr, " ");
  char *a2 = strtok(nullptr, " ");
  char *a3 = strtok(nullptr, " ");
  float v1, v2, v3;
  int i;
  bool ok = true;

  if (!strcmp(cmd, "?") || !strcmp(cmd, "h")) {
    ayuda();
  } else if (!strcmp(cmd, "e")) {
    estado();
  } else if (!strcmp(cmd, "s") || !strcmp(cmd, "l") || !strcmp(cmd, "r")) {
    portENTER_CRITICAL(&mux);
    for (int k = 0; k < N_ART; k++) {
      cancelarPendientes(k);
      if (cmd[0] == 'l') {
        ctl[k].liberar();
        ruedaLibre[k] = true;
      } else if (cmd[0] == 'r') {
        ctl[k].borrarFalla();  // queda detenido: frena y después sostiene
      } else {
        ctl[k].detener();
      }
    }
    alimentarWatchdog();
    portEXIT_CRITICAL(&mux);
    Serial.println(cmd[0] == 'l' ? ">> Liberado (rueda libre)." : cmd[0] == 'r' ? ">> Fallas borradas." : ">> Detenido.");
  } else if (!strcmp(cmd, "z")) {
    int desde = 0, hasta = N_ART;
    if (a1) {
      if (!leerArt(a1, i)) return;
      desde = i;
      hasta = i + 1;
    }
    for (int k = desde; k < hasta; k++) {
      portENTER_CRITICAL(&mux);
      cancelarPendientes(k);
      ctl[k].fijarCeroUltima(POSE_CERO[k]);  // última lectura validada por la tarea
      ruedaLibre[k] = false;
      alimentarWatchdog();
      portEXIT_CRITICAL(&mux);
      Serial.printf(">> J%d: cero fijado, q = %.1f°.\n", k + 1, POSE_CERO[k]);
    }
  } else if (!strcmp(cmd, "home")) {
    // De a una: si un final de carrera falla, solo se mueve esa articulación
    if (!leerArt(a1, i)) return;
    if (PIN_FC[i] < 0) {
      Serial.printf(">> J%d no tiene final de carrera: usar z en la pose de referencia.\n", i + 1);
      return;
    }
    if (finalDeCarrera(i)) {
      Serial.printf(">> J%d: el final de carrera ya está apretado (o en corto). Alejarlo con d y repetir.\n", i + 1);
      return;
    }
    const int64_t c0 = cuentas(i);
    portENTER_CRITICAL(&mux);
    ok = ctl[i].falla() == ControlArticulacion::Falla::Ninguna;
    if (ok) {
      ctl[i].quitarCero();  // el cero anterior no debe frenar el homing en qMin antes del final de carrera
      ctl[i].velocidad(HOME_RPM[i]);
      cancelarPendientes(i);
      homing[i] = true;
      homingDesdeMs[i] = millis();
      homingC0[i] = c0;
      ruedaLibre[i] = false;
      alimentarWatchdog();
    }
    portEXIT_CRITICAL(&mux);
    if (ok) Serial.printf(">> J%d: homing a %.0f rpm...\n", i + 1, HOME_RPM[i]);
    else rechazo(i, "home");
  } else if (!strcmp(cmd, "m")) {
    if (!leerNum(a1, v1) || !leerNum(a2, v2) || !leerNum(a3, v3)) {
      Serial.println(">> Uso: m <q1> <q2> <q3>");
      return;
    }
    const float q[N_ART] = {v1, v2, v3};
    for (int k = 0; k < N_ART; k++) {
      portENTER_CRITICAL(&mux);
      ok = ctl[k].irA(q[k]);
      if (ok) {
        cancelarPendientes(k);
        ruedaLibre[k] = false;
        alimentarWatchdog();
      }
      portEXIT_CRITICAL(&mux);
      if (!ok) rechazo(k, "m");
    }
  } else if (!strcmp(cmd, "j")) {
    if (!leerArt(a1, i)) return;
    if (!leerNum(a2, v2)) {
      Serial.println(">> Uso: j <n> <grados>");
      return;
    }
    portENTER_CRITICAL(&mux);
    ok = ctl[i].irA(v2);
    if (ok) {
      cancelarPendientes(i);
      ruedaLibre[i] = false;
      alimentarWatchdog();
    }
    portEXIT_CRITICAL(&mux);
    if (!ok) rechazo(i, "j");
  } else if (!strcmp(cmd, "v")) {
    if (!leerArt(a1, i)) return;
    v3 = VEL_CMD_S_DEF;
    if (!leerNum(a2, v2) || fabsf(v2) > VEL_CMD_MAX_RPM || (a3 && (!leerNum(a3, v3) || v3 <= 0 || v3 > VEL_CMD_S_MAX))) {
      Serial.printf(">> Uso: v <n> <rpm, hasta %.0f> [segundos, hasta %.0f]\n", VEL_CMD_MAX_RPM, VEL_CMD_S_MAX);
      return;
    }
    portENTER_CRITICAL(&mux);
    ok = ctl[i].velocidad(v2);
    if (ok) {
      cancelarPendientes(i);
      cortarEnMs[i] = (millis() + (uint32_t)(v3 * 1000)) | 1;
      ruedaLibre[i] = false;
      alimentarWatchdog();
    }
    portEXIT_CRITICAL(&mux);
    if (!ok) rechazo(i, "v");
  } else if (!strcmp(cmd, "d")) {
    if (!leerArt(a1, i)) return;
    if (!leerNum(a2, v2)) {
      Serial.println(">> Uso: d <n> <pct>");
      return;
    }
    if (fabsf(v2) > JOG_DUTY_MAX[i]) v2 = v2 > 0 ? JOG_DUTY_MAX[i] : -JOG_DUTY_MAX[i];
    portENTER_CRITICAL(&mux);
    ok = ctl[i].duty(v2);
    if (ok) {
      cancelarPendientes(i);
      cortarEnMs[i] = (millis() + JOG_MS) | 1;
      ruedaLibre[i] = false;
      alimentarWatchdog();
    }
    portEXIT_CRITICAL(&mux);
    if (!ok) rechazo(i, "d");
  } else if (!strcmp(cmd, "pinza")) {
    if (!servosOk || !leerNum(a1, v1) || v1 < 0 || v1 > 100) {
      Serial.println(servosOk ? ">> Uso: pinza <0-100>" : ">> Servos sin configurar.");
      return;
    }
    armarServos();
    servoUs(CH_PINZA, PINZA_US_ABIERTA + (PINZA_US_CERRADA - PINZA_US_ABIERTA) * v1 / 100.0f);
  } else if (!strcmp(cmd, "giro")) {
    if (!servosOk || !leerNum(a1, v1) || fabsf(v1) > GIRO_MAX_DEG) {
      if (servosOk) Serial.printf(">> Uso: giro <grados> (-%.0f..%.0f)\n", GIRO_MAX_DEG, GIRO_MAX_DEG);
      else Serial.println(">> Servos sin configurar.");
      return;
    }
    armarServos();
    servoUs(CH_GIRO, GIRO_US_CENTRO + GIRO_US_POR_GRADO * v1);
  } else if (!strcmp(cmd, "log")) {
    if (a1 && !strcmp(a1, "0")) {
      portENTER_CRITICAL(&mux);
      logArt = -1;
      portEXIT_CRITICAL(&mux);
      return;
    }
    if (!leerArt(a1, i)) return;
    v2 = 100;
    if (a2 && (!leerNum(a2, v2) || v2 < 1 || v2 > 100)) {
      Serial.println(">> Uso: log <n> [hz 1..100]");
      return;
    }
    portENTER_CRITICAL(&mux);
    logArt = i;
    logCadaTicks = (uint32_t)(100.0f / v2 + 0.5f);
    portEXIT_CRITICAL(&mux);
    Serial.println("t_ms,q_obj,q,wref_rpm,w_rpm,u_pct");
  } else if (!strcmp(cmd, "wd")) {
    if (!leerNum(a1, v1) || v1 < 0 || v1 > 10000) {
      Serial.println(">> Uso: wd <ms> (0 = apagado, hasta 10000)");
      return;
    }
    portENTER_CRITICAL(&mux);
    watchdogMs = (uint32_t)v1;
    alimentarWatchdog();
    portEXIT_CRITICAL(&mux);
    Serial.printf(">> Watchdog: %lu ms.\n", (unsigned long)watchdogMs);
  } else {
    Serial.printf(">> Comando desconocido: %s (? para ayuda)\n", cmd);
  }
}

char linea[64];
size_t lineaLen = 0;
bool descartando = false;

void leerSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (descartando) {
        Serial.println(">> Línea demasiado larga: descartada.");
      } else if (lineaLen > 0 && hwOk) {
        linea[lineaLen] = '\0';
        ejecutar(linea);
      }
      lineaLen = 0;
      descartando = false;
    } else if (lineaLen < sizeof(linea) - 1) {
      linea[lineaLen++] = ch;
    } else {
      descartando = true;
    }
  }
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  // Drivers primero y deshabilitados: hasta acá los EN flotan (pull-down de 10 kΩ recomendado)
  bool ok = motorJ1.begin(PIN_J1_RPWM, PIN_J1_LPWM, PIN_J1_EN, PIN_J1_EN, CH_J1_R, CH_J1_L, PWM_BTS_HZ, PWM_BITS);
  ok &= motorJ2.begin(PIN_J2_ENA, PIN_J2_IN1, PIN_J2_IN2, CH_J2_A, CH_J2_B, L298N::Modo::Freno, PWM_L298_HZ, PWM_BITS);
  ok &= motorJ3.begin(PIN_J3_ENB, PIN_J3_IN3, PIN_J3_IN4, CH_J3_A, CH_J3_B, L298N::Modo::Freno, PWM_L298_HZ, PWM_BITS);
  motorJ1.coast();
  motorJ2.coast();
  motorJ3.coast();
  const bool okPwm = ok;

  const bool okEnc = enc[0].begin(PIN_J1_ENC_A, PIN_J1_ENC_B, PCNT_UNIT_0) &&
                     enc[1].begin(PIN_J2_ENC_A, PIN_J2_ENC_B, PCNT_UNIT_1) &&
                     enc[2].begin(PIN_J3_ENC_A, PIN_J3_ENC_B, PCNT_UNIT_2);

  // Los canales de los servos se configuran, pero los pines se conectan con el primer comando pinza/giro
  servosOk = ledcSetup(CH_PINZA, SERVO_HZ, SERVO_BITS) != 0 && ledcSetup(CH_GIRO, SERVO_HZ, SERVO_BITS) != 0;

  for (int i = 0; i < N_ART; i++) {
    ctl[i].configurar(CFG_ART[i]);
    if (PIN_FC[i] >= 0) pinMode(PIN_FC[i], INPUT);  // 34-39: sin pull-up interno, va uno externo
  }

  colaLog = xQueueCreate(64, sizeof(Muestra));
  colaMsg = xQueueCreate(8, sizeof(Msg));
  hwOk = okPwm && okEnc && colaLog && colaMsg;

  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("== 09-brazo-control: J1 (36GP-555 + IBT-2), J2/J3 (5840-31ZY + L298N), pinza (2 servos) ==");
  Serial.printf("Control en cascada a %.0f Hz en el núcleo 0. Brazo en rueda libre y SIN CERO.\n", 1.0f / TS);
  if (!okPwm) Serial.println("ERROR: no se pudo configurar el PWM de algún driver.");
  if (!okEnc) Serial.println("ERROR: no se pudo configurar el PCNT de algún encoder.");
  if (!servosOk) Serial.println("AVISO: no se pudo configurar el PWM de los servos (pinza y giro deshabilitados).");
  if (!hwOk) {
    Serial.println("Comandos DESHABILITADOS hasta corregir el error y reiniciar.");
    return;
  }
  Serial.println("Para empezar: llevar el brazo a la pose de referencia (d) y fijar el cero (z), o usar home.");
  ayuda();

  ultimoCmdMs = millis();
  // Watchdog de tareas: si la tarea de control se cuelga, la placa se reinicia (y los puentes se deshabilitan)
  esp_task_wdt_init(TWDT_S, true);
  xTaskCreatePinnedToCore(tareaControl, "control", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 0);
}

void loop() {
  leerSerial();

  Msg msg;
  while (colaMsg && xQueueReceive(colaMsg, &msg, 0) == pdTRUE) {
    const int n = msg.art + 1;
    switch (msg.ev) {
      case Evento::HomingOk: Serial.printf(">> J%d: homing OK, q = %.1f°.\n", n, HOME_Q[msg.art]); break;
      case Evento::HomingTimeout: Serial.printf(">> J%d: homing sin tocar el final de carrera a tiempo: detenido.\n", n); break;
      case Evento::HomingRecorrido: Serial.printf(">> J%d: homing superó el recorrido máximo sin tocar el final de carrera: detenido.\n", n); break;
      case Evento::HomingFalla: Serial.printf(">> J%d: homing cancelado por una falla.\n", n); break;
      case Evento::Watchdog: Serial.println(">> Watchdog: sin comandos, brazo detenido."); break;
    }
  }

  // Avisar fallas nuevas
  static ControlArticulacion::Falla previa[N_ART] = {};
  for (int i = 0; i < N_ART; i++) {
    portENTER_CRITICAL(&mux);
    const ControlArticulacion::Falla f = ctl[i].falla();
    portEXIT_CRITICAL(&mux);
    if (f != previa[i] && f != ControlArticulacion::Falla::Ninguna)
      Serial.printf(">> J%d: FALLA %s. Frenada; r para borrar.\n", i + 1, ControlArticulacion::texto(f));
    previa[i] = f;
  }

  Muestra m;
  while (colaLog && xQueueReceive(colaLog, &m, 0) == pdTRUE)
    Serial.printf("%lu,%.2f,%.2f,%.1f,%.1f,%.1f\n", (unsigned long)m.tMs, m.qObj, m.q, m.wRef, m.w, m.u);

  delay(1);
}
