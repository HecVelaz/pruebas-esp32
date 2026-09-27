// Pruebas del control en la PC (sin placa): pio test -e native
// Simula cada motor con el modelo de la Entrega 5 (primer orden + zona muerta) y un encoder ideal.
#include <math.h>
#include <unity.h>

#include "ControlArticulacion.h"
#include "brazo_config.h"

// Motor simulado: w' = (K·(|u| - zona muerta)·signo(u) - w) / tau, integra la posición en cuentas
struct MotorSim {
  float K, tau, zonaMuerta, cuentasPorVuelta;
  float w = 0.0f;          // rpm
  double cuentas = 0.0;
  bool trabado = false;

  void paso(float u, float dt) {
    float uEf = fabsf(u) > zonaMuerta ? (fabsf(u) - zonaMuerta) * (u > 0 ? 1.0f : -1.0f) : 0.0f;
    const float wObj = trabado ? 0.0f : K * uEf;
    w += dt / tau * (wObj - w);
    cuentas += w / 60.0f * cuentasPorVuelta * dt;
  }
  int64_t leer() const { return (int64_t)llround(cuentas); }
};

static MotorSim simDe(int i) {
  const ConfigArticulacion &c = CFG_ART[i];
  // La zona muerta real (arranque) algo mayor que la del feedforward, para que el PI tenga que corregir
  return MotorSim{c.K, c.tau, c.u0 + 3.0f, c.cuentasPorVuelta};
}

// Corre el lazo t segundos. Devuelve el mayor |duty| visto.
static float correr(ControlArticulacion &k, MotorSim &m, float t) {
  float uMax = 0.0f;
  for (int n = 0; n < (int)(t / TS); n++) {
    const float u = k.paso(m.leer(), TS);
    uMax = fmaxf(uMax, fabsf(u));
    m.paso(u, TS);
  }
  return uMax;
}

static void llegaAlObjetivo(int i, float desde, float hasta) {
  ControlArticulacion k;
  k.configurar(CFG_ART[i]);
  MotorSim m = simDe(i);
  k.fijarCero(m.leer(), desde);
  TEST_ASSERT_TRUE(k.irA(hasta));
  const float tMov = fabsf(hasta - desde) / CFG_ART[i].velMax + 3.0f;
  correr(k, m, tMov);
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::Ninguna, k.falla());
  TEST_ASSERT_TRUE_MESSAGE(k.llego(), CFG_ART[i].nombre);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, hasta, k.q());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, k.u());  // en reposo frena (duty 0)
}

void test_j1_llega() { llegaAlObjetivo(0, 0.0f, 60.0f); }
void test_j2_llega() { llegaAlObjetivo(1, 90.0f, 10.0f); }
void test_j3_llega() { llegaAlObjetivo(2, 0.0f, -40.0f); }

void test_sin_cero_no_mueve() {
  ControlArticulacion k;
  k.configurar(CFG_ART[1]);
  TEST_ASSERT_FALSE(k.irA(30.0f));
}

void test_objetivo_recortado_a_limites() {
  ControlArticulacion k;
  k.configurar(CFG_ART[0]);
  k.fijarCero(0, 0.0f);
  TEST_ASSERT_TRUE(k.irA(500.0f));
  TEST_ASSERT_EQUAL_FLOAT(CFG_ART[0].qMax, k.qObjetivo());
}

void test_velocidad_sigue_consigna() {
  ControlArticulacion k;
  k.configurar(CFG_ART[0]);
  MotorSim m = simDe(0);
  TEST_ASSERT_TRUE(k.velocidad(60.0f));
  correr(k, m, 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 60.0f, k.w());
}

void test_atasco() {
  ControlArticulacion k;
  k.configurar(CFG_ART[1]);
  MotorSim m = simDe(1);
  m.trabado = true;
  k.fijarCero(m.leer(), 90.0f);
  k.irA(0.0f);
  correr(k, m, 3.0f);
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::Atasco, k.falla());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, k.u());
  TEST_ASSERT_FALSE(k.irA(10.0f));  // no acepta comandos hasta borrar la falla
  k.borrarFalla();
  TEST_ASSERT_TRUE(k.irA(10.0f));
}

void test_descarta_salto_de_encoder() {
  ControlArticulacion k;
  k.configurar(CFG_ART[1]);
  k.fijarCero(1000, 0.0f);
  k.paso(1000, TS);
  k.paso(1000 + 32000, TS);  // lectura falsa aislada
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, k.q());
  TEST_ASSERT_EQUAL_UINT32(1, k.saltosEncoder());
  k.paso(1000, TS);
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::Ninguna, k.falla());
}

void test_saltos_seguidos_son_falla() {
  ControlArticulacion k;
  k.configurar(CFG_ART[1]);
  k.fijarCero(0, 0.0f);
  for (int n = 1; n <= 6; n++) k.paso((int64_t)n * 40000, TS);
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::Encoder, k.falla());
}

void test_fuera_de_limites() {
  // Con cero, una lectura fuera de [qMin - margen, qMax + margen] es falla (por ejemplo, un cero mal fijado)
  ControlArticulacion k;
  k.configurar(CFG_ART[2]);
  k.fijarCero(0, 45.0f);
  const float cuentasPorGrado = 1.0f / k.gradosPorCuenta();
  for (int n = 1; n <= 20; n++) k.paso((int64_t)(n * 1.0f * cuentasPorGrado), TS);  // +20° en 0,2 s
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::FueraDeLimites, k.falla());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, k.u());
}

void test_manual_no_empuja_afuera_del_limite() {
  // En duty y velocidad (sin lazo de posición), con cero, al llegar al límite se corta en ese sentido
  ControlArticulacion k;
  k.configurar(CFG_ART[2]);
  MotorSim m = simDe(2);
  k.fijarCero(m.leer(), 45.0f);
  TEST_ASSERT_TRUE(k.duty(45.0f));
  correr(k, m, 3.0f);
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::Ninguna, k.falla());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, k.u());
  TEST_ASSERT_TRUE(k.q() < CFG_ART[2].qMax + CFG_ART[2].margenLim);
  TEST_ASSERT_TRUE(k.duty(-45.0f));  // hacia adentro sí
  correr(k, m, 0.5f);
  TEST_ASSERT_TRUE(k.q() < CFG_ART[2].qMax);
}

void test_detener_no_vuelve_atras() {
  // Frenar en pleno movimiento: duty 0 hasta quedar quieto y recién ahí sostener, sin retroceder
  ControlArticulacion k;
  k.configurar(CFG_ART[0]);
  MotorSim m = simDe(0);
  k.fijarCero(m.leer(), -60.0f);
  k.irA(60.0f);
  correr(k, m, 1.5f);  // a velocidad máxima
  TEST_ASSERT_TRUE(fabsf(k.w()) > 3.0f);
  k.detener();
  const float qAlDetener = k.q();
  float uMin = 0.0f;
  for (int n = 0; n < 300; n++) {
    const float u = k.paso(m.leer(), TS);
    uMin = fminf(uMin, u);
    m.paso(u, TS);
  }
  TEST_ASSERT_EQUAL_FLOAT(0.0f, uMin);  // nunca empujó hacia atrás
  TEST_ASSERT_TRUE(k.q() >= qAlDetener);
  TEST_ASSERT_FALSE(k.frenando());
  TEST_ASSERT_TRUE(k.llego());
}

void test_cero_con_ultima_lectura_validada() {
  // Si justo al fijar el cero la lectura cruda es un salto falso, fijarCeroUltima usa la anterior buena
  ControlArticulacion k;
  k.configurar(CFG_ART[1]);
  k.paso(1000, TS);
  k.paso(1000 + 32000, TS);  // salto falso: descartado
  k.fijarCeroUltima(0.0f);
  k.paso(1000, TS);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, k.q());
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::Ninguna, k.falla());
}

void test_quitar_cero_para_homing() {
  ControlArticulacion k;
  k.configurar(CFG_ART[0]);
  k.fijarCero(0, CFG_ART[0].qMin);  // parado justo en el límite
  k.quitarCero();
  TEST_ASSERT_TRUE(k.velocidad(-10.0f));
  const float u = k.paso(0, TS);
  TEST_ASSERT_TRUE(u < 0.0f);  // sin cero, el límite no frena el homing
}

void setUp() {}
void tearDown() {}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_j1_llega);
  RUN_TEST(test_j2_llega);
  RUN_TEST(test_j3_llega);
  RUN_TEST(test_sin_cero_no_mueve);
  RUN_TEST(test_objetivo_recortado_a_limites);
  RUN_TEST(test_velocidad_sigue_consigna);
  RUN_TEST(test_atasco);
  RUN_TEST(test_descarta_salto_de_encoder);
  RUN_TEST(test_saltos_seguidos_son_falla);
  RUN_TEST(test_fuera_de_limites);
  RUN_TEST(test_manual_no_empuja_afuera_del_limite);
  RUN_TEST(test_detener_no_vuelve_atras);
  RUN_TEST(test_cero_con_ultima_lectura_validada);
  RUN_TEST(test_quitar_cero_para_homing);
  return UNITY_END();
}
