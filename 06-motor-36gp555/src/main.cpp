// Prueba en lazo abierto del motor 36GP-555 (12 V, 160 rpm) con el driver IBT-2 (BTS7960).
// El duty se fija a mano por el monitor serial; el encoder solo se lee para medir la velocidad
// y para las protecciones (esperar a que el motor se detenga antes de invertir, corte por atasco).
// No corrige la velocidad: eso sería lazo cerrado.
#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"
#include "motor_config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_R = 0;
constexpr uint8_t LEDC_CH_L = 1;

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

BTS7960 driver;
EncoderPCNT enc;
bool hwOk = false;  // si el PWM o el encoder no se inicializan, no se aceptan comandos al motor

enum class Modo { Manual, Barrido, Escalon, Contar };
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
float rpmMedida = 0.0f;  // rpm del motor, con signo
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

char linea[32];
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
  Serial.println("  n      contar vueltas girando la salida a mano (para verificar la reducción)");
  Serial.println("  ?      esta ayuda");
  Serial.println("Para invertir el sentido, el motor primero baja a 0 y espera a detenerse.");
  Serial.printf("Corte por atasco: duty >= %.0f %% y menos de %.0f rpm del motor durante %lu ms.\n", ATASCO_DUTY_PCT,
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

void procesarLinea(char *s) {
  while (*s == ' ') s++;
  size_t n = strlen(s);
  while (n > 0 && s[n - 1] == ' ') s[--n] = '\0';

  // Enter o cualquier línea corta el barrido o el escalón, y termina el conteo
  if (modo == Modo::Barrido || modo == Modo::Escalon) {
    pararYa("cortado por el usuario");
    return;
  }
  if (modo == Modo::Contar) {
    modo = Modo::Manual;
    reiniciarVelocidad();
    Serial.println(">> Fin del conteo.");
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
        Serial.println(">> Conteo de vueltas (rueda libre). Marcar el eje de salida, girarlo a mano N vueltas");
        Serial.println("   y leer 'reducción medida' (con N vueltas exactas). Enter para terminar.");
        return;
    }
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
    pararYa("ATASCO");
    Serial.printf("   Duty %.0f %% pero %.0f rpm del motor en ese sentido durante %lu ms.\n", dutyAct,
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

  const bool quieto = dutyAct == 0.0f && dutyObj == 0.0f && fabsf(rpm) < 1.0f;
  if (quieto) {
    if (!detenidoInformado) {
      Serial.printf("[%7.1f s] detenido (%s) | cuentas %lld\n", millis() / 1000.0f,
                    driver.enabled() ? "frenado" : "rueda libre", (long long)c);
      detenidoInformado = true;
    }
    return;
  }
  detenidoInformado = false;
  Serial.printf("[%7.1f s] duty %+6.1f %% (obj %+4.0f) | motor %+7.0f rpm | salida %+7.1f rpm | cuentas %lld\n",
                millis() / 1000.0f, dutyAct, dutyObj, rpm, rpm / REDUCCION, (long long)c);
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
        if (barridoSentido > 0 && zonaMuertaPct < 0 && fabsf(rpm) > 20.0f) zonaMuertaPct = barridoPct;

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

void pasoContar() {
  const uint32_t ahora = millis();
  if (ahora - contarUltimoMs < 500) return;
  contarUltimoMs = ahora;
  const int64_t c = cuentas();
  const float vueltasMotor = c / CUENTAS_POR_VUELTA;
  Serial.printf("   cuentas %+lld | vueltas del motor %+.2f | salida (con reducción %.1f) %+.3f | A=%d B=%d\n",
                (long long)c, vueltasMotor, REDUCCION, vueltasMotor / REDUCCION, enc.levelA(), enc.levelB());
  Serial.printf("   reducción medida = vueltas del motor / vueltas de salida girada (1: %.1f, 2: %.1f, 5: %.1f)\n",
                fabsf(vueltasMotor), fabsf(vueltasMotor) / 2, fabsf(vueltasMotor) / 5);
}

void setup() {
  // Driver primero y deshabilitado, antes de cualquier espera: hasta acá los EN flotan
  // (por eso el README pide pull-down de 10 kΩ en R_EN y L_EN)
  const bool okDriver = driver.begin(PIN_RPWM, PIN_LPWM, PIN_R_EN, PIN_L_EN, LEDC_CH_R, LEDC_CH_L,
                                     PWM_FREQ_HZ, PWM_BITS);
  driver.coast();
  const bool okEnc = enc.begin(PIN_ENC_A, PIN_ENC_B);
  hwOk = okDriver && okEnc;

  Serial.begin(115200);
  delay(1500);  // dar tiempo a que abra el monitor por USB

  Serial.println();
  Serial.println("== Prueba en lazo abierto: motor 36GP-555 + IBT-2 (BTS7960) ==");
  Serial.printf("IBT-2: RPWM=GPIO%d, LPWM=GPIO%d, R_EN=GPIO%d, L_EN=GPIO%d | PWM %lu Hz, %u bits\n", PIN_RPWM,
                PIN_LPWM, PIN_R_EN, PIN_L_EN, (unsigned long)PWM_FREQ_HZ, PWM_BITS);
  Serial.printf("Encoder: A=GPIO%d, B=GPIO%d | %.0f PPR x%.0f = %.0f cuentas por vuelta del motor | reducción %.1f:1\n",
                PIN_ENC_A, PIN_ENC_B, ENCODER_PPR, ENCODER_X, CUENTAS_POR_VUELTA, REDUCCION);
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
    case Modo::Manual:
      break;
  }

  if (ticks % TELEMETRIA_TICKS == 0 && modo == Modo::Manual) telemetria();
}
