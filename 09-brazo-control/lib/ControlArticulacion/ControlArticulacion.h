#pragma once

#include <stdint.h>

// Control en cascada de una articulación del brazo, sin Arduino ni hardware (se prueba en la PC: test/).
//
//   posición objetivo ──► [P posición + límite de velocidad y aceleración] ──► velocidad ref
//   velocidad ref ──► [PI + feedforward (modelo de la Entrega 5)] ──► duty % ──► driver
//
// El lazo interno trabaja en rpm del EJE DE SALIDA DEL MOTOR (donde se identificó G(s) = K/(τs+1) en la E5).
// El externo trabaja en grados de la ARTICULACIÓN. Los une la relación de transmisión (correas/poleas).
// paso() se llama cada dt (10 ms) con las cuentas del encoder y devuelve el duty a aplicar.
// Duty 0 significa freno: con el sin fin de J2/J3 el brazo queda sostenido sin corriente.

struct ConfigArticulacion {
  const char *nombre;
  // Encoder y transmisión
  float cuentasPorVuelta;  // cuentas del encoder por vuelta del eje de salida del motor (x4 incluido)
  float relacion;          // vueltas del eje del motor por vuelta de la articulación (1 = directo)
  float rpmMax;            // velocidad máxima del motor (para descartar saltos falsos del encoder)
  // Límites de la articulación [grados]
  float qMin, qMax;
  // Modelo del motor (E5): rpm = K * (duty - zona muerta), constante de tiempo tau
  float K;        // rpm/%
  float tau;      // s
  float u0;       // % de zona muerta girando (feedforward)
  // Lazo de velocidad (PI)
  float kpVel;    // %/rpm
  float kiVel;    // %/(rpm·s)
  // Lazo de posición
  float kpPos;    // (grados/s) por grado de error
  float velMax;   // grados/s de la articulación
  float accMax;   // grados/s² de la articulación
  float tolPos;   // grados: dentro de esta banda se frena y se da por llegado
  float dutyMax;  // % máximo que puede pedir el control
  // Protecciones
  float atascoDuty;  // % a partir del cual se vigila el atasco
  float atascoRpm;   // rpm mínimas en el sentido pedido
  float atascoS;     // s sostenido para declarar atasco
  float margenLim;   // grados fuera de [qMin, qMax] que se toleran antes de declarar falla
};

class ControlArticulacion {
 public:
  enum class Modo { Libre, Posicion, Velocidad, Duty };
  enum class Falla { Ninguna, Atasco, FueraDeLimites, Encoder };

  void configurar(const ConfigArticulacion &cfg);

  // Fija el cero: la posición actual del encoder pasa a valer qDeg. Habilita los modos con límites.
  void fijarCero(int64_t cuentas, float qDeg);
  // Igual, con la última lectura ya validada por paso() (descarta un salto falso del encoder justo en el cero).
  void fijarCeroUltima(float qDeg) { fijarCero(cuentasPrev_, qDeg); }
  bool conCero() const { return conCero_; }
  // Olvida el cero (antes de un homing, que lo vuelve a fijar): sin límites ni modo posición hasta entonces.
  void quitarCero() {
    conCero_ = false;
    if (modo_ == Modo::Posicion) liberar();
  }

  // Comandos. Devuelven false si no se aceptan (falla activa o sin cero cuando hace falta).
  bool irA(float qDeg);                 // modo posición (el objetivo se recorta a los límites)
  bool velocidad(float rpm);            // modo velocidad (lazo interno solo), rpm del motor
  bool duty(float pct);                 // lazo abierto
  void detener();                       // frena (duty 0) y, recién quieto, sostiene esa posición
  void liberar();                       // modo Libre: duty 0 (el main decide si freno o rueda libre)
  void borrarFalla();

  // Un paso del control. Devuelve el duty en % (-dutyMax..dutyMax; 0 = freno).
  float paso(int64_t cuentas, float dt);

  // Estado
  Modo modo() const { return modo_; }
  Falla falla() const { return falla_; }
  const char *textoFalla() const { return texto(falla_); }
  static const char *texto(Falla f);
  float q() const { return q_; }            // grados de la articulación
  float qObjetivo() const { return qObj_; }
  float wRef() const { return wRefRpm_; }   // rpm del motor pedidas por el lazo de posición
  float w() const { return w_; }            // rpm del motor medidas (filtradas)
  float u() const { return u_; }            // duty aplicado, %
  bool llego() const { return enReposo_ && !frenando_; }
  bool frenando() const { return frenando_; }
  uint32_t saltosEncoder() const { return saltos_; }

  // Conversiones
  float gradosPorCuenta() const;
  float rpmDeGradosPorSeg(float gps) const { return gps * cfg_.relacion / 6.0f; }

 private:
  float pasoPosicion(float dt);
  // sentidoFf: sentido del feedforward de zona muerta (0 = según wRefRpm, solo si supera un umbral)
  float pasoVelocidad(float wRefRpm, float dt, float sentidoFf = 0.0f);
  void revisarProtecciones(float dt);
  void entrarFalla(Falla f);

  ConfigArticulacion cfg_{};
  Modo modo_ = Modo::Libre;
  Falla falla_ = Falla::Ninguna;
  bool conCero_ = false;
  bool primerPaso_ = true;

  int64_t cuentasCero_ = 0;
  int64_t cuentasPrev_ = 0;
  float qCero_ = 0.0f;

  float q_ = 0.0f;
  float qObj_ = 0.0f;
  float wRefGps_ = 0.0f;   // velocidad de referencia de la articulación (con límite de aceleración)
  float wRefRpm_ = 0.0f;
  float wCmdRpm_ = 0.0f;   // consigna del modo velocidad
  float w_ = 0.0f;
  float integ_ = 0.0f;
  float u_ = 0.0f;
  float uCmd_ = 0.0f;      // consigna del modo duty
  bool enReposo_ = true;
  bool frenando_ = false;  // detener(): duty 0 hasta que el eje quede quieto, sin volver hacia atrás
  int quietoTicks_ = 0;

  float atascoT_ = 0.0f;
  uint32_t saltos_ = 0;
  int saltosSeguidos_ = 0;
};
