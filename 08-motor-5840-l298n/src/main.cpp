// Prueba en lazo abierto del motor 5840-31ZY (sin fin, 12 V, 160 rpm) con el driver L298N, en una
// ESP32-WROOM-32D. Adaptado de 06-motor-36gp555 (mismos comandos y formatos de salida, para el análisis).
// El encoder externo (1000 PPR) está en el eje de salida: todas las rpm son de salida (REDUCCION = 1).
// El duty se fija a mano por el monitor serial; el encoder solo se lee para medir la velocidad
// y para las protecciones (esperar a que el motor se detenga antes de invertir, corte por atasco).
// No corrige la velocidad: eso sería lazo cerrado.
#include <Arduino.h>

#include "L298N.h"
#include "EncoderPCNT.h"
#include "motor_config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_A = 0;  // IN1 (modo freno) o ENA (modo rueda libre)
constexpr uint8_t LEDC_CH_B = 1;  // IN2 (modo freno)

constexpr uint32_t TICK_US = 10000;          // periodo de la rampa, las protecciones y el CSV del escalón (10 ms)
constexpr uint32_t TELEMETRIA_TICKS = 20;    // telemetría cada 200 ms
constexpr float CUENTAS_POR_VUELTA = ENCODER_PPR * ENCODER_X;  // del eje del motor
constexpr int VEL_N = 10;                    // la velocidad de las protecciones se mide en 10 ticks (100 ms)

// Barrido automático
constexpr int BARRIDO_PASO_PCT = 10;
constexpr uint32_t BARRIDO_ASENTAR_MS = 2000;  // espera a que la velocidad se estabilice
constexpr uint32_t BARRIDO_MEDIR_MS = 1000;    // ventana de medición
constexpr uint32_t BARRIDO_PAUSA_MS = 1000;    // pausa en duty 0 antes de invertir o terminar
// Escalón
constexpr uint32_t ESCALON_MS = 3000;
// Perfil en escalera (comando p)
constexpr int PERFIL_MAX_NIVELES = 8;
constexpr uint32_t PERFIL_SEG_MS_DEF = 5000;
constexpr float PERFIL_NIVELES_DEF[] = {20, 40, 60, 80, 100};
constexpr uint32_t PERFIL_PARADA_MS = 1000;  // freno entre la mitad positiva y la negativa

L298N driver;
EncoderPCNT enc;
bool hwOk = false;  // si el PWM o el encoder no se inicializan, no se aceptan comandos al motor

enum class Modo { Manual, Barrido, Escalon, Contar, Perfil };
Modo modo = Modo::Manual;

float dutyObj = 0.0f;  // % pedido (-100..100)
float dutyAct = 0.0f;  // % aplicado (sigue a dutyObj con la rampa)
bool rampa = true;     // el escalón la desactiva
uint32_t ceroDesdeMs = 0;       // desde cuándo el duty aplicado es 0
bool esperaInformada = false;   // ya se avisó que se espera a que el motor se detenga

uint32_t tickUs = 0;
uint32_t ticks = 0;
uint32_t ticksPerdidos = 0;

// Velocidad para las protecciones: ventana móvil de VEL_N ticks
int64_t velC[VEL_N];
uint32_t velT[VEL_N];
int velI = 0;
float rpmMedida = 0.0f;  // rpm medidas por el encoder (de salida), con signo
uint32_t atascoDesdeMs = 0;

// Telemetría
int64_t cuentasTel = 0;
uint32_t tTelUs = 0;
bool detenidoInformado = true;

// Barrido
enum class FaseBarrido { Rampa, Asentar, Medir, Pausa };
FaseBarrido faseB;
int barridoPct = 0;
int barridoSentido = 1;
uint32_t faseInicioMs = 0;
int64_t cuentasInicioMedida = 0;
uint32_t tInicioMedidaUs = 0;
int zonaMuertaPct = -1;

// Escalón
uint32_t escalonInicioUs = 0;
uint32_t tPrevEscalonUs = 0;
int64_t cuentasInicioEscalon = 0;
int64_t cuentasPrevEscalon = 0;

// Contar vueltas
uint32_t contarUltimoMs = 0;

// Perfil
float perfilNiv[PERFIL_MAX_NIVELES];
int perfilN = 0;
uint32_t perfilSegMs = PERFIL_SEG_MS_DEF;
int perfilIdx = 0;
int perfilSentido = 1;
bool perfilEnParada = false;
uint32_t perfilSegInicioMs = 0;
uint32_t perfilInicioUs = 0;
int64_t perfilC0 = 0;

char linea[64];
size_t lineaLen = 0;
bool ultimoFueCR = false;

// Cuentas del encoder con el signo corregido
int64_t cuentas() {
  const int64_t c = enc.count();
  return ENCODER_INVERTIDO ? -c : c;
}

float rpmMotor(int64_t dCuentas, uint32_t dtUs) {
  if (dtUs == 0) return 0.0f;
  return (float)dCuentas / CUENTAS_POR_VUELTA * 60.0e6f / (float)dtUs;
}

float limitar(float pct) {
  if (pct > DUTY_MAX_PCT) return DUTY_MAX_PCT;
  if (pct < -DUTY_MAX_PCT) return -DUTY_MAX_PCT;
  return pct;
}

// Lee un duty en % que ocupe toda la cadena (se admiten espacios al final). Rechaza NaN e infinito.
bool leerPct(const char *s, float &pct) {
  char *fin = nullptr;
  const float v = strtof(s, &fin);
  if (fin == s) return false;
  while (*fin == ' ') fin++;
  if (*fin != '\0' || !isfinite(v)) return false;
  pct = v;
  return true;
}

void reiniciarVelocidad() {
  const int64_t c = cuentas();
  const uint32_t t = micros();
  for (int i = 0; i < VEL_N; i++) {
    velC[i] = c;
    velT[i] = t;
  }
  rpmMedida = 0.0f;
  cuentasTel = c;
  tTelUs = t;
}

void actualizarVelocidad() {
  const int64_t c = cuentas();
  const uint32_t t = micros();
  // velI apunta a la muestra más vieja de la ventana
  rpmMedida = rpmMotor(c - velC[velI], t - velT[velI]);
  velC[velI] = c;
  velT[velI] = t;
  velI = (velI + 1) % VEL_N;
}

// Quieto: duty 0 desde hace PARADA_MIN_MS y el encoder casi no se mueve
bool motorQuieto() {
  return dutyAct == 0.0f && millis() - ceroDesdeMs >= PARADA_MIN_MS && fabsf(rpmMedida) < RPM_QUIETO;
}

void aplicarDuty(float pct) {
  if (!driver.setDuty(pct / 100.0f)) return;  // driver sin inicializar o valor inválido: no se toca nada
  if (pct == 0.0f && dutyAct != 0.0f) ceroDesdeMs = millis();
  dutyAct = pct;
}

void ruedaLibre() {
  if (dutyAct != 0.0f) ceroDesdeMs = millis();
  dutyObj = 0.0f;
  dutyAct = 0.0f;
  driver.coast();
}

void ayuda() {
  Serial.println();
  Serial.println("Comandos (escribir y Enter):");
  Serial.println("  <n>    duty en % de -100 a 100 (signo = sentido), con rampa. Ej: 30, -50");
  Serial.println("  0, s   parar con rampa (queda frenado)");
  Serial.println("  x      parar YA (freno inmediato). Enter o cualquier línea también corta el barrido y el escalón");
  Serial.println("  l      rueda libre (driver deshabilitado)");
  Serial.println("  a      barrido automático: duty vs rpm, adelante y atrás");
  Serial.println("  e<n>   escalón a n % desde el reposo, sin rampa, CSV cada 10 ms durante 3 s. Ej: e50");
  Serial.println("  n      contar cuentas girando el eje del encoder a mano (verificar el encoder)");
  Serial.println("  p      perfil en escalera sin rampa: +20..+100 % y -20..-100 %, 5 s por nivel, CSV cada 10 ms");
  Serial.println("         Opcional: p <ms por nivel> <niveles %>. Ej: p 3000 25 50 75 100");
  Serial.println("  ?      esta ayuda");
  Serial.println("Para invertir el sentido, el motor primero baja a 0 y espera a detenerse.");
  Serial.printf("Corte por atasco: duty >= %.0f %% y menos de %.0f rpm de salida durante %lu ms.\n", ATASCO_DUTY_PCT,
                ATASCO_RPM, (unsigned long)ATASCO_MS);
  Serial.println();
}

void pararYa(const char *motivo) {
  modo = Modo::Manual;
  rampa = true;
  dutyObj = 0.0f;
  aplicarDuty(0.0f);  // freno
  Serial.printf(">> Parado (%s).\n", motivo);
}

void iniciarBarrido() {
  modo = Modo::Barrido;
  rampa = true;
  barridoSentido = 1;
  barridoPct = BARRIDO_PASO_PCT;
  zonaMuertaPct = -1;
  dutyObj = barridoPct;
  faseB = FaseBarrido::Rampa;
  Serial.println(">> Barrido automático. Enter o cualquier línea lo corta.");
  Serial.println("   duty %  | rpm motor | rpm salida | cuentas/s");
}

void iniciarEscalon(float pct) {
  pct = limitar(pct);
  if (pct == 0.0f) {
    Serial.println(">> El escalón necesita un duty distinto de 0. Ej: e50");
    return;
  }
  // Parte del reposo: si el motor gira (también en rueda libre), primero hay que pararlo
  if (dutyObj != 0.0f || !motorQuieto()) {
    Serial.println(">> El escalón parte del reposo: parar el motor (0), esperar a que se detenga y repetir.");
    return;
  }
  modo = Modo::Escalon;
  rampa = false;
  dutyObj = pct;
  aplicarDuty(pct);
  ticksPerdidos = 0;
  escalonInicioUs = micros();
  tPrevEscalonUs = escalonInicioUs;
  cuentasInicioEscalon = cuentas();
  cuentasPrevEscalon = cuentasInicioEscalon;
  Serial.printf(">> Escalón a %.0f %% sin rampa, %lu ms. CSV (cuentas desde el inicio del escalón):\n", pct,
                (unsigned long)ESCALON_MS);
  Serial.println("t_ms,duty_pct,cuentas,rpm_motor,rpm_salida");
  Serial.printf("0,%.1f,0,0.0,0.00\n", pct);
}

void aplicarPerfil(float pct) {
  dutyObj = pct;
  aplicarDuty(pct);
  perfilSegInicioMs = millis();
  Serial.printf(">> Perfil: nivel %+.0f %%\n", pct);
}

// "p" o "p <ms> <nivel1> <nivel2> ...": los niveles son positivos; la mitad negativa es la misma escalera
void iniciarPerfil(char *args) {
  uint32_t segMs = PERFIL_SEG_MS_DEF;
  float niv[PERFIL_MAX_NIVELES];
  int n = 0;
  char *tok = strtok(args, " ");
  if (tok) {
    float v;
    if (!leerPct(tok, v) || v < 500 || v > 60000) {
      Serial.println(">> Perfil inválido: el primer número son los ms por nivel (500..60000). Ej: p 5000 20 40 60");
      return;
    }
    segMs = (uint32_t)v;
    while ((tok = strtok(nullptr, " ")) != nullptr) {
      if (n == PERFIL_MAX_NIVELES || !leerPct(tok, v) || v <= 0.0f) {
        Serial.printf(">> Perfil inválido: hasta %d niveles positivos. Ej: p 5000 20 40 60\n", PERFIL_MAX_NIVELES);
        return;
      }
      niv[n++] = limitar(v);
    }
  }
  if (n == 0) {
    for (float v : PERFIL_NIVELES_DEF) niv[n++] = limitar(v);
  }
  if (dutyObj != 0.0f || !motorQuieto()) {
    Serial.println(">> El perfil parte del reposo: parar el motor (0), esperar a que se detenga y repetir.");
    return;
  }
  for (int i = 0; i < n; i++) perfilNiv[i] = niv[i];
  perfilN = n;
  perfilSegMs = segMs;
  perfilIdx = 0;
  perfilSentido = 1;
  perfilEnParada = false;
  modo = Modo::Perfil;
  rampa = false;
  ticksPerdidos = 0;
  Serial.printf(">> Perfil: %d niveles x %lu ms por sentido, sin rampa. Enter o cualquier línea lo corta.\n", n,
                (unsigned long)segMs);
  Serial.printf("# cuentas_por_vuelta_motor=%.0f reduccion=%.1f\n", CUENTAS_POR_VUELTA, REDUCCION);
  Serial.println("t_ms,duty_pct,cuentas");
  perfilInicioUs = micros();
  perfilC0 = cuentas();
  aplicarPerfil(perfilNiv[0]);
}

void procesarLinea(char *s) {
  while (*s == ' ') s++;
  size_t n = strlen(s);
  while (n > 0 && s[n - 1] == ' ') s[--n] = '\0';

  // Enter o cualquier línea corta el barrido, el escalón o el perfil, y termina el conteo.
  // La línea no se ejecuta: se avisa para que se vuelva a escribir.
  if (modo == Modo::Barrido || modo == Modo::Escalon || modo == Modo::Perfil) {
    pararYa("cortado por el usuario");
    if (*s) Serial.printf("   Se descartó '%s': volver a escribirlo.\n", s);
    return;
  }
  if (modo == Modo::Contar) {
    modo = Modo::Manual;
    reiniciarVelocidad();
    Serial.println(">> Fin del conteo.");
    if (*s) Serial.printf("   Se descartó '%s': volver a escribirlo.\n", s);
    return;
  }
  if (*s == '\0') return;

  if (s[0] == '?' || s[0] == 'h') {
    ayuda();
    return;
  }
  if (!hwOk) {
    Serial.println(">> ERROR de inicialización (ver el arranque): comandos del motor deshabilitados.");
    return;
  }

  if (s[1] == '\0') {
    switch (s[0]) {
      case 'x':
        pararYa("comando x");
        return;
      case 's':
        dutyObj = 0.0f;
        rampa = true;
        Serial.println(">> Parando con rampa.");
        return;
      case 'l':
        ruedaLibre();
        Serial.println(">> Rueda libre (driver deshabilitado).");
        return;
      case 'a':
        iniciarBarrido();
        return;
      case 'n':
        ruedaLibre();
        modo = Modo::Contar;
        enc.reset();
        reiniciarVelocidad();
        contarUltimoMs = 0;
        Serial.println(">> Conteo (rueda libre). Girar el eje del encoder a mano (el sin fin no se puede girar");
        Serial.println("   desde la salida): una vuelta = 4000 cuentas. Enter para terminar.");
        return;
    }
  }

  if (s[0] == 'p' && (s[1] == '\0' || s[1] == ' ')) {
    iniciarPerfil(s + 1);
    return;
  }

  float pct;
  if (s[0] == 'e') {
    if (leerPct(s + 1, pct)) iniciarEscalon(pct);
    else Serial.printf(">> Escalón inválido: '%s'. Ej: e50\n", s);
    return;
  }
  if (!leerPct(s, pct)) {
    Serial.printf(">> Comando desconocido: '%s'. '?' para la ayuda.\n", s);
    return;
  }
  dutyObj = limitar(pct);
  rampa = true;
  esperaInformada = false;
  Serial.printf(">> Duty objetivo %+.0f %% (rampa %.0f %%/s).\n", dutyObj, RAMPA_PCT_S);
}

void leerSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' && ultimoFueCR) {  // CRLF: una sola línea
      ultimoFueCR = false;
      continue;
    }
    ultimoFueCR = ch == '\r';
    if (ch == '\n' || ch == '\r') {
      linea[lineaLen] = '\0';
      procesarLinea(linea);  // también las líneas vacías (Enter solo)
      lineaLen = 0;
    } else if (lineaLen < sizeof(linea) - 1) {
      linea[lineaLen++] = ch;
    }
  }
}

// Mueve dutyAct hacia dutyObj como máximo RAMPA_PCT_S por segundo. Al invertir el sentido,
// se detiene en 0 hasta que el motor esté quieto.
void actualizarRampa() {
  if (!rampa || dutyAct == dutyObj) return;  // incluye parado (frenado o en rueda libre): no tocar

  if (dutyAct == 0.0f) {
    // Salir de 0: permitido con el motor quieto, o si ya gira en el sentido pedido (tras la rueda libre)
    const bool mismoSentido = (dutyObj > 0.0f && rpmMedida >= RPM_QUIETO) ||
                              (dutyObj < 0.0f && rpmMedida <= -RPM_QUIETO);
    if (!motorQuieto() && !mismoSentido) {
      if (!esperaInformada) {
        Serial.println(">> Esperando que el motor se detenga antes de arrancar en ese sentido...");
        esperaInformada = true;
      }
      return;
    }
    esperaInformada = false;
  }

  const float paso = RAMPA_PCT_S * TICK_US / 1e6f;
  float d = dutyObj - dutyAct;
  if (d > paso) d = paso;
  if (d < -paso) d = -paso;
  float nuevo = dutyAct + d;
  // No cruzar el 0 en un solo paso: frenar en 0 y esperar a que se detenga
  if ((dutyAct > 0.0f && nuevo < 0.0f) || (dutyAct < 0.0f && nuevo > 0.0f)) nuevo = 0.0f;
  aplicarDuty(nuevo);
}

// Duty alto y el motor no gira en ese sentido: trabado, encoder desconectado o invertido
void revisarAtasco() {
  const float rpmEnSentido = dutyAct > 0.0f ? rpmMedida : -rpmMedida;
  if (fabsf(dutyAct) < ATASCO_DUTY_PCT || rpmEnSentido >= ATASCO_RPM) {
    atascoDesdeMs = 0;
    return;
  }
  const uint32_t ahora = millis();
  if (atascoDesdeMs == 0) {
    atascoDesdeMs = ahora;
    return;
  }
  if (ahora - atascoDesdeMs >= ATASCO_MS) {
    atascoDesdeMs = 0;
    const float dutyAlCortar = dutyAct;  // pararYa() lo pone en 0
    pararYa("ATASCO");
    Serial.printf("   Duty %.0f %% pero %.1f rpm de salida en ese sentido durante %lu ms.\n", dutyAlCortar,
                  rpmEnSentido, (unsigned long)ATASCO_MS);
    Serial.println("   Revisar: eje trabado, encoder desconectado, o rpm negativas (ENCODER_INVERTIDO).");
  }
}

void telemetria() {
  const uint32_t t = micros();
  const int64_t c = cuentas();
  const float rpm = rpmMotor(c - cuentasTel, t - tTelUs);
  cuentasTel = c;
  tTelUs = t;

  const bool quieto = dutyAct == 0.0f && dutyObj == 0.0f && fabsf(rpm) < 0.2f;
  if (quieto) {
    if (!detenidoInformado) {
      Serial.printf("[%7.1f s] detenido (%s) | cuentas %lld\n", millis() / 1000.0f,
                    driver.enabled() ? "frenado" : "rueda libre", (long long)c);
      detenidoInformado = true;
    }
    return;
  }
  detenidoInformado = false;
  Serial.printf("[%7.1f s] duty %+6.1f %% (obj %+4.0f) | salida %+7.1f rpm | cuentas %lld\n", millis() / 1000.0f,
                dutyAct, dutyObj, rpm / REDUCCION, (long long)c);
}

void pasoBarrido() {
  const uint32_t ahora = millis();
  switch (faseB) {
    case FaseBarrido::Rampa:
      if (dutyAct == dutyObj) {
        faseB = FaseBarrido::Asentar;
        faseInicioMs = ahora;
      }
      break;
    case FaseBarrido::Asentar:
      if (ahora - faseInicioMs >= BARRIDO_ASENTAR_MS) {
        faseB = FaseBarrido::Medir;
        faseInicioMs = ahora;
        cuentasInicioMedida = cuentas();
        tInicioMedidaUs = micros();
      }
      break;
    case FaseBarrido::Medir:
      if (ahora - faseInicioMs >= BARRIDO_MEDIR_MS) {
        const uint32_t dt = micros() - tInicioMedidaUs;
        const int64_t dc = cuentas() - cuentasInicioMedida;
        const float rpm = rpmMotor(dc, dt);
        Serial.printf("   %+5.0f   | %+9.0f | %+10.1f | %9.0f\n", dutyAct, rpm, rpm / REDUCCION,
                      dc * 1e6f / dt);
        if (barridoSentido > 0 && zonaMuertaPct < 0 && fabsf(rpm) > RPM_QUIETO) zonaMuertaPct = barridoPct;

        if (barridoPct + BARRIDO_PASO_PCT <= DUTY_MAX_PCT) {
          barridoPct += BARRIDO_PASO_PCT;
          dutyObj = barridoSentido * barridoPct;
          faseB = FaseBarrido::Rampa;
        } else {
          dutyObj = 0.0f;  // bajar con rampa antes de invertir o terminar
          faseB = FaseBarrido::Pausa;
          faseInicioMs = ahora;
        }
      }
      break;
    case FaseBarrido::Pausa:
      // La pausa se cuenta desde que el duty llega a 0, y además el motor tiene que estar quieto
      if (dutyAct != 0.0f) {
        faseInicioMs = ahora;
        break;
      }
      if (ahora - faseInicioMs >= BARRIDO_PAUSA_MS && motorQuieto()) {
        if (barridoSentido > 0) {
          barridoSentido = -1;
          barridoPct = BARRIDO_PASO_PCT;
          dutyObj = -barridoPct;
          faseB = FaseBarrido::Rampa;
          Serial.println("   --- atrás ---");
        } else {
          modo = Modo::Manual;
          Serial.println(">> Fin del barrido.");
          if (zonaMuertaPct > 0) {
            Serial.printf("   Zona muerta: el motor empieza a girar entre %d %% y %d %% de duty.\n",
                          zonaMuertaPct - BARRIDO_PASO_PCT, zonaMuertaPct);
          }
        }
      }
      break;
  }
}

void pasoEscalon() {
  const uint32_t t = micros();
  const uint32_t tRel = t - escalonInicioUs;
  const int64_t c = cuentas();
  const float rpm = rpmMotor(c - cuentasPrevEscalon, t - tPrevEscalonUs);
  cuentasPrevEscalon = c;
  tPrevEscalonUs = t;
  Serial.printf("%lu,%.1f,%lld,%.1f,%.2f\n", (unsigned long)(tRel / 1000), dutyAct,
                (long long)(c - cuentasInicioEscalon), rpm, rpm / REDUCCION);
  if (tRel >= ESCALON_MS * 1000UL) {
    modo = Modo::Manual;
    rampa = true;
    dutyObj = 0.0f;  // baja con rampa
    Serial.println(">> Fin del escalón, bajando con rampa.");
    if (ticksPerdidos > 0) {
      Serial.printf("   Aviso: se perdieron %lu muestras (loop atrasado); ver los saltos en t_ms.\n",
                    (unsigned long)ticksPerdidos);
    }
  }
}

void pasoPerfil() {
  const uint32_t t = micros();
  Serial.printf("%lu,%.1f,%lld\n", (unsigned long)((t - perfilInicioUs) / 1000), dutyAct,
                (long long)(cuentas() - perfilC0));
  if (modo != Modo::Perfil) return;  // el atasco pudo cortarlo en este tick

  const uint32_t ahora = millis();
  if (perfilEnParada) {
    // Entre las dos mitades: frenado hasta que el motor está quieto (no se invierte girando)
    if (ahora - perfilSegInicioMs >= PERFIL_PARADA_MS && motorQuieto()) {
      perfilEnParada = false;
      perfilSentido = -1;
      perfilIdx = 0;
      aplicarPerfil(-perfilNiv[0]);
    }
    return;
  }
  if (ahora - perfilSegInicioMs < perfilSegMs) return;

  perfilIdx++;
  if (perfilIdx < perfilN) {
    aplicarPerfil(perfilSentido * perfilNiv[perfilIdx]);
  } else if (perfilSentido > 0) {
    perfilEnParada = true;
    aplicarPerfil(0.0f);  // freno
  } else {
    modo = Modo::Manual;
    rampa = true;
    dutyObj = 0.0f;
    aplicarDuty(0.0f);
    Serial.println(">> Fin del perfil.");
    if (ticksPerdidos > 0) {
      Serial.printf("   Aviso: se perdieron %lu muestras (loop atrasado); ver los saltos en t_ms.\n",
                    (unsigned long)ticksPerdidos);
    }
  }
}

void pasoContar() {
  const uint32_t ahora = millis();
  if (ahora - contarUltimoMs < 500) return;
  contarUltimoMs = ahora;
  const int64_t c = cuentas();
  Serial.printf("   cuentas %+lld | vueltas del encoder %+.3f | A=%d B=%d\n", (long long)c, c / CUENTAS_POR_VUELTA,
                enc.levelA(), enc.levelB());
}

void setup() {
  // Driver primero y deshabilitado, antes de cualquier espera: hasta acá ENA flota
  // (por eso el README recomienda un pull-down de 10 kΩ en ENA)
  const bool okDriver =
      driver.begin(PIN_ENA, PIN_IN1, PIN_IN2, LEDC_CH_A, LEDC_CH_B, PWM_MODO, PWM_FREQ_HZ, PWM_BITS);
  driver.coast();
  const bool okEnc = enc.begin(PIN_ENC_A, PIN_ENC_B);
  hwOk = okDriver && okEnc;

  Serial.begin(115200);
  delay(1500);  // dar tiempo a que abra el monitor

  Serial.println();
  Serial.println("== Prueba en lazo abierto: motor 5840-31ZY + L298N (ESP32-WROOM-32D) ==");
  Serial.printf("L298N: ENA=GPIO%d, IN1=GPIO%d, IN2=GPIO%d | PWM %lu Hz, %u bits, %s\n", PIN_ENA, PIN_IN1, PIN_IN2,
                (unsigned long)PWM_FREQ_HZ, PWM_BITS,
                PWM_MODO == L298N::Modo::Freno ? "PWM en IN1/IN2 con freno (ENA fijo)" : "PWM en ENA con rueda libre");
  Serial.printf("Encoder externo en la salida: A=GPIO%d, B=GPIO%d | %.0f PPR x%.0f = %.0f cuentas por vuelta\n",
                PIN_ENC_A, PIN_ENC_B, ENCODER_PPR, ENCODER_X, CUENTAS_POR_VUELTA);
  Serial.printf("Límites: duty máx %.0f %%, rampa %.0f %%/s\n", DUTY_MAX_PCT, RAMPA_PCT_S);
  if (!okDriver) Serial.println("ERROR: no se pudo configurar el PWM (LEDC). Revisar PWM_FREQ_HZ / PWM_BITS.");
  if (!okEnc) Serial.println("ERROR: no se pudo configurar el PCNT del encoder.");
  if (!hwOk) Serial.println("Comandos del motor DESHABILITADOS hasta corregir el error y reiniciar.");
  Serial.printf("Niveles del encoder en reposo: A=%d B=%d (con el motor quieto pueden ser 0 o 1)\n", enc.levelA(),
                enc.levelB());
  Serial.println("Motor en rueda libre. Empezar con un duty bajo, por ejemplo 20.");
  ayuda();

  reiniciarVelocidad();
  ceroDesdeMs = millis();
  tickUs = micros();
}

void loop() {
  leerSerial();

  const uint32_t ahora = micros();
  if (ahora - tickUs < TICK_US) return;
  // Si el loop se atrasó (por ejemplo, el Serial bloqueado), descartar los ticks vencidos
  // en lugar de ejecutarlos en ráfaga: la rampa nunca sube más rápido que RAMPA_PCT_S.
  if (ahora - tickUs >= 2 * TICK_US) {
    ticksPerdidos += (ahora - tickUs) / TICK_US - 1;
    tickUs = ahora;
  } else {
    tickUs += TICK_US;
  }
  ticks++;

  if (!hwOk) return;

  actualizarVelocidad();
  actualizarRampa();
  revisarAtasco();

  // Fuera del modo manual no hay telemetría: mantener al día su referencia para que la primera
  // lectura al volver no promedie todo el barrido o el conteo.
  if (modo != Modo::Manual) {
    cuentasTel = cuentas();
    tTelUs = ahora;
  }

  switch (modo) {
    case Modo::Barrido:
      pasoBarrido();
      break;
    case Modo::Escalon:
      pasoEscalon();
      return;  // el escalón imprime su propio CSV
    case Modo::Contar:
      pasoContar();
      return;
    case Modo::Perfil:
      pasoPerfil();
      return;  // imprime su propio CSV
    case Modo::Manual:
      break;
  }

  if (ticks % TELEMETRIA_TICKS == 0 && modo == Modo::Manual) telemetria();
}
