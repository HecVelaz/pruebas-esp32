#pragma once

#include <stdint.h>

#include "BTS7960.h"
#include "EncoderPCNT.h"

// Una articulación del brazo (J1, J2 o J3) con el MISMO controlador para las tres; cambia solo la configuración.
// Viene de 19-j2-j3-conjunto (fases 1-4) y de 12-j1-base:
//  - Cascada: P de posición (50 Hz) -> PI de velocidad (100 Hz) con feedforward de fricción (despegue / marcha,
//    con rampa de despegue) y anti-windup.
//  - Sostener: va al ángulo con la cascada y al llegar pasa a un PI de posición (uHold + KHP·e + fricción) con banda
//    del integrador y freno dentro de la banda (al quedar firme, duty 0 hasta que se aleje).
//  - Seguir: la trayectoria la da el coordinador en cada periodo (qRef, wRef) -> velocidad pedida = wRef + KPP·e.
//  - Homing: si el final de carrera (lado -) está apretado se aleja hasta soltarlo y "salida" grados más; después
//    baja a homingVelBuscar hasta el flanco y queda sosteniéndose ahí.
// Sin Serial ni pines fijos: los pines vienen en la configuración. paso() se llama cada TS (10 ms) desde UNA sola
// tarea; todo lo demás también desde esa tarea (no es seguro entre núcleos).
// Signos: + = alejarse del final de carrera; la configuración dice si el encoder o el motor van al revés.

struct ConfigArticulacion {
  const char *nombre;
  // Hardware (el EN es común a los tres IBT-2: lo maneja el coordinador)
  int pinRpwm, pinLpwm, pinEn, pinFc;
  uint8_t canalR, canalL;
  int pinEncA, pinEncB;
  pcnt_unit_t unidadPcnt;
  uint32_t pwmHz;
  uint8_t pwmBits;
  // Escala y signos
  float cuentasPorGrado;
  bool encoderInvertido, sentidoInvertido;
  // Límites propios desde el final de carrera (°) y duty máximo (%)
  float limNeg, limPos, dutyMax;
  // PI de velocidad y feedforward (%)
  float kpVel, kiVel;
  float ffDespPos, ffMarchaPos, ffDespNeg, ffMarchaNeg, ffMinPos;
  // P de posición
  float kpp, vMax, aMax, vMin, tolPos, tolSalida;
  uint32_t llegadaMs;
  // Sostener
  float khp, khi, friccion, bandaInt;
  // Homing
  float homingVelBuscar, homingVelAlejar, homingSalida, homingDutyMax, homingRecorridoMax;
  uint32_t homingTrabadaMs;
  float homingVelRapida;      // °/s buscando mientras está más arriba que homingZonaLenta (solo con cero previo)
  float homingZonaLenta;      // ° sobre el cero anterior desde donde busca a homingVelBuscar
  // Protecciones
  float atascoVel, atascoDuty;
  uint32_t atascoMs;
  float arranquePct;          // con menos duty no actúa el corte "sin cuentas" (sostener el peso pide duty sin moverse)
  uint32_t sinCuentasMs;
  float contrarioVel;
  uint32_t contrarioMs;
  float fcSostenMax;          // apoyada en el final de carrera, hacia él como mucho esto (%)
  // Se autobloquea (J1: correa + 50:1, sin peso): al llegar frena (duty 0) en vez del PI de posición, y "firme" con
  // tolSalida. Con el PI, el integrador empujaba contra el juego de la correa y J1 iba y venía alrededor del destino.
  bool autobloqueo;
};

class Articulacion {
 public:
  enum class Modo : uint8_t { Suelta, Sostener, Seguir, Homing };
  enum class Homing : uint8_t { Nada, EnCurso, Listo, Error };

  static constexpr float TS = 0.01f;  // s, periodo de paso()

  bool begin(const ConfigArticulacion &c);
  void paso();

  // Órdenes
  void suelta();                       // duty 0 (freno con el EN alto); quien llama maneja el EN
  void rueda();                        // rueda libre: baja el EN (común: suelta las tres; solo para "x")
  void sostener(float q);              // ir a q con la cascada y quedarse; sin salto desde el duty actual
  void sostenerAqui();                 // sostener donde está
  void frenarAqui();                   // duty 0 y sostener donde está desde ahí (sin el duty con que se movía)
  void seguir(float qRef, float wRef); // un periodo de trayectoria (el coordinador lo llama en cada paso)
  void homing(bool conCero = false);  // conCero: rápido hasta homingZonaLenta sobre el cero anterior, después lento
  void homingCancelar();
  void ceroAqui();                     // el ángulo actual pasa a ser 0
  void ceroEnFlanco();                 // el flanco del último homing pasa a ser 0
  void fijarAngulo(float g);           // el ángulo actual pasa a ser g (cero estimado, "zq")
  void frenoEnBanda(bool si) { frenoBanda_ = si; }
  void ajustarFeedforward(float despPos, float marchaPos, float despNeg, float marchaNeg) {
    c_.ffDespPos = despPos;
    c_.ffMarchaPos = marchaPos;
    c_.ffDespNeg = despNeg;
    c_.ffMarchaNeg = marchaNeg;
  }
  void reiniciarEstado();              // borra el error/aviso de la última vez

  // Estado
  const ConfigArticulacion &cfg() const { return c_; }
  float q() const { return grados(cuentas()); }
  float w() const { return w_; }
  float duty() const { return u_; }
  float referencia() const { return ref_; }
  bool fc() const { return fc_; }
  bool llego() const { return modo_ == Modo::Sostener && llegado_; }
  bool firme() const { return modo_ == Modo::Sostener && firme_; }
  bool frenada() const { return frenada_; }
  uint32_t msFirme() const;            // desde cuándo está firme (0 si no)
  Modo modo() const { return modo_; }
  Homing estadoHoming() const { return hEstado_; }
  float flancoGrados() const { return grados(flanco_); }  // dónde quedó el flanco, con el cero que había
  const char *error() const { return error_; }            // último corte (nullptr si ninguno)
  const char *aviso() { const char *a = aviso_; aviso_ = nullptr; return a; }
  float uHold() const { return uHold_; }

 private:
  int64_t cuentas() const;
  float grados(int64_t c) const { return (float)c / c_.cuentasPorGrado; }
  void aplicar(float pct);
  float pasoPI(float wRef, float th);
  void cortar(const char *causa);      // protección: deja de mover y sostiene donde está
  void protecciones(float wRefMov);

  ConfigArticulacion c_{};
  BTS7960 motor_;
  EncoderPCNT enc_;
  int64_t ceroOffset_ = 0;
  Modo modo_ = Modo::Suelta;
  // Medición
  static constexpr int VENTANA = 4;
  int64_t hist_[VENTANA + 1] = {};
  float w_ = 0.0f, u_ = 0.0f;
  bool fc_ = false;
  // PI de velocidad
  float integral_ = 0.0f, ffRampa_ = 0.0f;
  bool despegada_ = false;
  int signoAnt_ = 0;
  uint32_t trabadaTicks_ = 0;
  // Posición / sostener
  float ref_ = 0.0f, wRefPos_ = 0.0f, wRefSeguir_ = 0.0f;
  int signoIda_ = 0;  // sentido hacia el destino al empezar: si el error cambia de signo, lo cruzó (= llegó)
  uint32_t k_ = 0;
  bool llegado_ = false, firme_ = false, frenada_ = false, frenoBanda_ = true;
  float uHold_ = 0.0f;
  uint32_t quietoTicks_ = 0, tFirme_ = 0;
  // Homing
  Homing hEstado_ = Homing::Nada;
  int hFase_ = 0;  // 2 alejarse, 3 buscar
  float hSoltado_ = 0.0f, hInicio_ = 0.0f;
  bool hSoltadoValido_ = false, hConCero_ = false;
  int64_t flanco_ = 0;
  int64_t hVentanaC_ = 0;
  uint32_t hVentanaMs_ = 0;
  // Protecciones
  uint32_t atascoTicks_ = 0, sinCuentasTicks_ = 0, contrarioTicks_ = 0;
  bool avisoAtasco_ = false;
  const char *error_ = nullptr;
  const char *aviso_ = nullptr;
};
