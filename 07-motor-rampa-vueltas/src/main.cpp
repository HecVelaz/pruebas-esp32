// Rampa de PWM por tiempo que termina al completar N vueltas de salida (lazo abierto).
// Motor 36GP-555 + IBT-2 (BTS7960), mismo cableado y drivers que 06-motor-36gp555.
//
//   r [segundos] [vueltas]   rampa 0 -> 100 % en 'segundos'; termina al llegar a 'vueltas' de salida.
//                            Vueltas negativas: gira hacia atrás. Por defecto: r 3 3
//   x o Enter                freno inmediato (corta la rampa)
//   ?                        ayuda
//
// Durante la rampa imprime un CSV cada 10 ms: t_ms,duty_pct,cuentas (cuentas desde el inicio).
#include <Arduino.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"
#include "motor_config.h"
#include "pins.h"

constexpr uint8_t LEDC_CH_R = 0;
constexpr uint8_t LEDC_CH_L = 1;
constexpr uint32_t TICK_US = 10000;
constexpr float CUENTAS_POR_VUELTA = ENCODER_PPR * ENCODER_X;  // del eje del motor
constexpr int VEL_N = 10;                                      // ventana de velocidad: 100 ms

BTS7960 driver;
EncoderPCNT enc;
bool hwOk = false;

bool enRampa = false;
float rampaS = RAMPA_S_DEF;
float vueltasObj = VUELTAS_DEF;
int sentido = 1;
float duty = 0.0f;  // % aplicado, con signo
uint32_t inicioUs = 0;
int64_t c0 = 0;
uint32_t llego100Ms = 0;  // cuándo llegó al 100 % (0: todavía no)

uint32_t tickUs = 0;
uint32_t ticksPerdidos = 0;
int64_t velC[VEL_N];
uint32_t velT[VEL_N];
int velI = 0;
float rpmMotor = 0.0f;
uint32_t atascoDesdeMs = 0;

char linea[64];
size_t lineaLen = 0;
bool ultimoFueCR = false;

int64_t cuentas() {
  const int64_t c = enc.count();
  return ENCODER_INVERTIDO ? -c : c;
}

void actualizarVelocidad() {
  const int64_t c = cuentas();
  const uint32_t t = micros();
  const uint32_t dt = t - velT[velI];
  rpmMotor = dt ? (float)(c - velC[velI]) / CUENTAS_POR_VUELTA * 60.0e6f / (float)dt : 0.0f;
  velC[velI] = c;
  velT[velI] = t;
  velI = (velI + 1) % VEL_N;
}

void frenar() {
  duty = 0.0f;
  driver.brake();
}

void terminar(const char *motivo) {
  const uint32_t tMs = (micros() - inicioUs) / 1000;
  const float dutyFinal = duty;
  const float vueltas = (float)(cuentas() - c0) / (CUENTAS_POR_VUELTA * REDUCCION);
  frenar();
  enRampa = false;
  Serial.printf(">> Fin (%s): %.2f vueltas de salida en %.2f s, duty final %+.1f %%.\n", motivo, vueltas,
                tMs / 1000.0f, dutyFinal);
  if (ticksPerdidos > 0) {
    Serial.printf("   Aviso: se perdieron %lu muestras (loop atrasado).\n", (unsigned long)ticksPerdidos);
  }
}

void ayuda() {
  Serial.println();
  Serial.println("Comandos (escribir y Enter):");
  Serial.println("  r [segundos] [vueltas]  rampa 0 -> 100 % en 'segundos', termina a 'vueltas' de salida.");
  Serial.println("                          Vueltas negativas: hacia atrás. Por defecto: r 3 3");
  Serial.println("  x o Enter               freno inmediato");
  Serial.println("  ?                       esta ayuda");
  Serial.printf("Si llega al 100 %% sin completar las vueltas, sigue en 100 %% hasta %lu ms más.\n",
                (unsigned long)ESPERA_EN_100_MS);
  Serial.println();
}

void iniciarRampa(char *args) {
  float seg = RAMPA_S_DEF, vue = VUELTAS_DEF;
  char *tok = strtok(args, " ");
  char *fin;
  if (tok) {
    seg = strtof(tok, &fin);
    if (*fin || !isfinite(seg) || seg < 0.5f || seg > 120.0f) {
      Serial.println(">> Rampa inválida: segundos entre 0.5 y 120. Ej: r 3 3");
      return;
    }
    if ((tok = strtok(nullptr, " ")) != nullptr) {
      vue = strtof(tok, &fin);
      if (*fin || !isfinite(vue) || fabsf(vue) < 0.1f || fabsf(vue) > 1000.0f) {
        Serial.println(">> Rampa inválida: vueltas entre 0.1 y 1000 (negativas: hacia atrás). Ej: r 3 3");
        return;
      }
    }
  }
  if (fabsf(rpmMotor) >= RPM_QUIETO) {
    Serial.println(">> La rampa parte del reposo: esperar a que el motor se detenga y repetir.");
    return;
  }
  rampaS = seg;
  sentido = vue < 0 ? -1 : 1;
  vueltasObj = fabsf(vue);
  enRampa = true;
  llego100Ms = 0;
  atascoDesdeMs = 0;
  ticksPerdidos = 0;
  Serial.printf(">> Rampa 0 -> %+.0f %% en %.1f s, hasta %.2f vueltas de salida. Enter lo corta.\n",
                sentido * 100.0f, rampaS, vueltasObj);
  Serial.printf("# cuentas_por_vuelta_motor=%.0f reduccion=%.1f rampa_s=%.2f vueltas=%.2f\n", CUENTAS_POR_VUELTA,
                REDUCCION, rampaS, sentido * vueltasObj);
  Serial.println("t_ms,duty_pct,cuentas");
  inicioUs = micros();
  c0 = cuentas();
  duty = 0.0f;
  driver.setDuty(0.0f);  // habilita el puente (duty 0 = freno) hasta que la rampa suba
}

void procesarLinea(char *s) {
  while (*s == ' ') s++;
  size_t n = strlen(s);
  while (n > 0 && s[n - 1] == ' ') s[--n] = '\0';

  if (enRampa) {  // cualquier línea (también Enter solo) corta la rampa
    terminar("cortada por el usuario");
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
  if (s[0] == 'x' && s[1] == '\0') {
    frenar();
    Serial.println(">> Frenado.");
    return;
  }
  if (s[0] == 'r' && (s[1] == '\0' || s[1] == ' ')) {
    iniciarRampa(s + 1);
    return;
  }
  Serial.printf(">> Comando desconocido: '%s'. '?' para la ayuda.\n", s);
}

void leerSerial() {
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' && ultimoFueCR) {
      ultimoFueCR = false;
      continue;
    }
    ultimoFueCR = ch == '\r';
    if (ch == '\n' || ch == '\r') {
      linea[lineaLen] = '\0';
      procesarLinea(linea);
      lineaLen = 0;
    } else if (lineaLen < sizeof(linea) - 1) {
      linea[lineaLen++] = ch;
    }
  }
}

void pasoRampa() {
  const uint32_t t = micros() - inicioUs;
  const int64_t c = cuentas() - c0;
  Serial.printf("%lu,%.1f,%lld\n", (unsigned long)(t / 1000), duty, (long long)c);

  // ¿Completó las vueltas? (en el sentido pedido)
  const float vueltas = sentido * (float)c / (CUENTAS_POR_VUELTA * REDUCCION);
  if (vueltas >= vueltasObj) {
    terminar("vueltas completas");
    return;
  }

  // Corte por atasco: duty alto y el motor no avanza en el sentido pedido
  if (fabsf(duty) >= ATASCO_DUTY_PCT && sentido * rpmMotor < ATASCO_RPM) {
    const uint32_t ahora = millis();
    if (atascoDesdeMs == 0) {
      atascoDesdeMs = ahora;
    } else if (ahora - atascoDesdeMs >= ATASCO_MS) {
      terminar("ATASCO");
      Serial.println("   Carga demasiado alta, eje trabado o encoder desconectado.");
      return;
    }
  } else {
    atascoDesdeMs = 0;
  }

  // Duty de la rampa: lineal en el tiempo, tope 100 %
  float pct = 100.0f * (t / 1e6f) / rampaS;
  if (pct >= 100.0f) {
    pct = 100.0f;
    if (llego100Ms == 0) llego100Ms = millis();
    if (millis() - llego100Ms >= ESPERA_EN_100_MS) {
      terminar("tiempo agotado en 100 %");
      return;
    }
  }
  duty = sentido * pct;
  driver.setDuty(duty / 100.0f);
}

void setup() {
  // Driver primero y deshabilitado (los EN flotan hasta acá)
  const bool okDriver = driver.begin(PIN_RPWM, PIN_LPWM, PIN_R_EN, PIN_L_EN, LEDC_CH_R, LEDC_CH_L, PWM_FREQ_HZ,
                                     PWM_BITS);
  driver.coast();
  const bool okEnc = enc.begin(PIN_ENC_A, PIN_ENC_B);
  hwOk = okDriver && okEnc;

  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("== Rampa por vueltas en lazo abierto: motor 36GP-555 + IBT-2 (BTS7960) ==");
  Serial.printf("Encoder: %.0f cuentas por vuelta del motor, reducción %.1f:1 -> %.0f cuentas por vuelta de salida\n",
                CUENTAS_POR_VUELTA, REDUCCION, CUENTAS_POR_VUELTA * REDUCCION);
  if (!okDriver) Serial.println("ERROR: no se pudo configurar el PWM (LEDC).");
  if (!okEnc) Serial.println("ERROR: no se pudo configurar el PCNT del encoder.");
  if (!hwOk) Serial.println("Comandos del motor DESHABILITADOS hasta corregir el error y reiniciar.");
  Serial.println("Motor en rueda libre. Listo.");
  ayuda();

  const int64_t c = cuentas();
  for (int i = 0; i < VEL_N; i++) {
    velC[i] = c;
    velT[i] = micros();
  }
  tickUs = micros();
}

void loop() {
  leerSerial();

  const uint32_t ahora = micros();
  if (ahora - tickUs < TICK_US) return;
  if (ahora - tickUs >= 2 * TICK_US) {  // loop atrasado: descartar ticks vencidos
    ticksPerdidos += (ahora - tickUs) / TICK_US - 1;
    tickUs = ahora;
  } else {
    tickUs += TICK_US;
  }
  if (!hwOk) return;

  actualizarVelocidad();
  if (enRampa) pasoRampa();
}
