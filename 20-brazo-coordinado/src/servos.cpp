// Pinza (GPIO 4) y muñeca (GPIO 14) en 20, con el driver de 21 y los valores de 23.
//   pz a | c | 0 | s | e | <us>     abrir (500) / cerrar (1250) / cero (1500) / soltar / estado / pulso
//   mn a | b | 0 | s | e | <us>     90° horario (860) / 90° antihorario (2150) / cero (1500) / soltar / estado / pulso
//   pz w <°/s>, mn w <°/s>          velocidad de la rampa (0 = salto)
// Avisa SERVO_LLEGO,<nombre>,<us> al terminar la rampa.
#include "servos.h"

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

#include "ServoPWM.h"
#include "config.h"
#include "pins.h"

extern Print *out;  // main.cpp: Serial o, con micro-ROS, /brazo/respuesta

namespace {

struct Servo {
  const char *nombre;
  const char *prefijo;
  int pin, canal;
  uint16_t cero, a, b, minUs, maxUs;
  float usPorGrado;
  ServoPWM s;
  float w;
  bool avisar;
};

Servo servos[2] = {
    {"pinza", "pz", PIN_SERVO_PINZA, CANAL_SERVO_PINZA, PINZA_CERO_US, PINZA_ABIERTO_US, PINZA_CERRADO_US,
     PINZA_MIN_US, PINZA_MAX_US, PINZA_US_POR_GRADO, {}, SERVO_W_INICIAL, false},
    {"muneca", "mn", PIN_SERVO_GIRO, CANAL_SERVO_GIRO, MUNECA_CERO_US, MUNECA_A_US, MUNECA_B_US, MUNECA_MIN_US,
     MUNECA_MAX_US, MUNECA_US_POR_GRADO, {}, SERVO_W_INICIAL, false},
};

void estado(Servo &v) {
  out->printf("SERVO %s %s actual=%u us destino=%u us franja=[%u, %u] w=%.0f °/s\n", v.nombre,
                v.s.suelto() ? "SUELTO" : "ACTIVO", v.s.actualUs(), v.s.destinoUs(), v.s.minUs(), v.s.maxUs(), v.w);
}

void ir(Servo &v, long us) {
  if (us < v.minUs || us > v.maxUs)
    out->printf("SERVO %s: %ld us fuera de [%u, %u]: se recorta\n", v.nombre, us, v.minUs, v.maxUs);
  if (us < 0) us = 0;
  const bool suelto = v.s.suelto();
  v.s.irA((uint16_t)us);
  out->printf("SERVO_VA,%s,%u%s\n", v.nombre, v.s.destinoUs(), suelto ? " (estaba suelto: salto directo)" : "");
  v.avisar = true;
}

}  // namespace

bool servosIniciar() {
  bool ok = true;
  for (auto &v : servos) {
    ok = v.s.begin(v.pin, v.canal, v.minUs, v.maxUs) && ok;
    v.s.velocidad(v.w * v.usPorGrado);
  }
  return ok;
}

bool servosComando(const char *l) {
  for (auto &v : servos) {
    const size_t n = strlen(v.prefijo);
    if (strncmp(l, v.prefijo, n) != 0 || (l[n] != ' ' && l[n] != '\0')) continue;
    const char *a = l + n;
    while (*a == ' ') a++;
    if (*a == '\0' || strcmp(a, "e") == 0) {
      estado(v);
    } else if (strcmp(a, "s") == 0) {
      v.s.soltar();
      v.avisar = false;
      out->printf("SERVO %s SUELTO\n", v.nombre);
    } else if (strcmp(a, "a") == 0) {
      ir(v, v.a);
    } else if ((strcmp(a, "b") == 0 && v.b != 0 && strcmp(v.prefijo, "mn") == 0) ||
               (strcmp(a, "c") == 0 && strcmp(v.prefijo, "pz") == 0)) {
      ir(v, v.b);
    } else if (strcmp(a, "0") == 0) {
      ir(v, v.cero);
    } else if (a[0] == 'w' && (a[1] == ' ' || a[1] == '\0')) {
      if (a[1] == ' ') {
        v.w = atof(a + 2);
        if (v.w < 0) v.w = 0;
        v.s.velocidad(v.w * v.usPorGrado);
      }
      out->printf("SERVO %s w=%.0f °/s%s\n", v.nombre, v.w, v.w == 0 ? " (salto directo)" : "");
    } else {
      char *fin;
      const long us = strtol(a, &fin, 10);
      if (fin == a || *fin != '\0')
        out->printf("SERVO %s: comando desconocido \"%s\" (? para la ayuda)\n", v.nombre, a);
      else
        ir(v, us);
    }
    return true;
  }
  return false;
}

void servosActualizar() {
  for (auto &v : servos) {
    v.s.update();
    if (v.avisar && v.s.enDestino()) {
      v.avisar = false;
      out->printf("SERVO_LLEGO,%s,%u\n", v.nombre, v.s.actualUs());
    }
  }
}

void servosTexto(char *b, size_t cap) {
  snprintf(b, cap, "pz=%d;mn=%d", servos[0].s.suelto() ? -1 : (int)servos[0].s.actualUs(),
           servos[1].s.suelto() ? -1 : (int)servos[1].s.actualUs());
}
