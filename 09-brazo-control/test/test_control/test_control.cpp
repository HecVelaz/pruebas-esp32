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
  ControlArticulacion k;
  k.configurar(CFG_ART[2]);
  MotorSim m = simDe(2);
  k.fijarCero(m.leer(), 45.0f);
  TEST_ASSERT_TRUE(k.duty(60.0f));  // lazo abierto hacia afuera del límite de +50°
  correr(k, m, 2.0f);
  TEST_ASSERT_EQUAL(ControlArticulacion::Falla::FueraDeLimites, k.falla());
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
  return UNITY_END();
}
