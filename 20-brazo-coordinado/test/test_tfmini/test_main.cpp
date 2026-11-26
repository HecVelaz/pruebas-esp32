#include <initializer_list>
#include <unity.h>
#include "TFminiS.h"

void setUp() {}
void tearDown() {}

bool trama(TFminiS &s, uint16_t cm, uint16_t fuerza, uint32_t t, bool corrupta = false) {
  uint8_t b[] = {0x59, 0x59, uint8_t(cm), uint8_t(cm >> 8), uint8_t(fuerza), uint8_t(fuerza >> 8), 0, 8, 0};
  for (unsigned i = 0; i < 8; ++i) b[8] += b[i];
  if (corrupta) ++b[8];
  bool lista = false;
  for (uint8_t v : b) lista = s.recibir(v, t);
  return lista;
}
void test_distancia_y_vencimiento() {
  TFminiS s;
  TEST_ASSERT_FALSE(s.valida(0));
  TEST_ASSERT_TRUE(trama(s, 73, 250, 100));
  TEST_ASSERT_FLOAT_WITHIN(.0001f, .73f, s.metros());
  TEST_ASSERT_EQUAL_UINT16(250, s.intensidad());
  TEST_ASSERT_TRUE(s.valida(400));
  TEST_ASSERT_FALSE(s.valida(401));
}
void test_invalidas_reemplazan_la_lectura_anterior() {
  const uint16_t datos[][2] = {{9, 150}, {1201, 150}, {65535, 10}, {65534, 65535}, {65532, 150}, {50, 99}, {50, 65535}};
  for (const auto &d : datos) {
    TFminiS s;
    trama(s, 50, 200, 0);
    TEST_ASSERT_TRUE(trama(s, d[0], d[1], 10));
    TEST_ASSERT_FALSE(s.valida(10));
  }
}
void test_checksum_no_renueva_y_recupera() {
  TFminiS s;
  trama(s, 50, 200, 0);
  TEST_ASSERT_FALSE(trama(s, 70, 200, 300, true));
  TEST_ASSERT_FALSE(s.valida(301));
  for (uint8_t b : {uint8_t(0), uint8_t(0x59), uint8_t(0x58), uint8_t(0xff)}) s.recibir(b, 302);
  TEST_ASSERT_TRUE(trama(s, 80, 200, 303));
  TEST_ASSERT_TRUE(s.valida(303));
  TEST_ASSERT_FLOAT_WITHIN(.0001f, .80f, s.metros());
}
void test_cabecera_solapada_y_trama_parcial() {
  TFminiS s;
  s.recibir(0x59, 0);  // byte extra antes de una trama válida
  TEST_ASSERT_TRUE(trama(s, 89, 200, 1));
  TEST_ASSERT_FLOAT_WITHIN(.0001f, .89f, s.metros());
  s.recibir(0x59, 10); s.recibir(0x59, 10); s.recibir(50, 10);
  TEST_ASSERT_TRUE(trama(s, 60, 200, 100));  // descarta parcial tras 50 ms
  s.descartar();
  TEST_ASSERT_FALSE(s.valida(101));
}
void test_limites_y_wrap_millis() {
  TFminiS s;
  trama(s, 10, 100, UINT32_MAX - 100);
  TEST_ASSERT_TRUE(s.valida(100));
  TEST_ASSERT_FALSE(s.valida(201));
  trama(s, 1200, 100, 300);
  TEST_ASSERT_TRUE(s.valida(300));
}
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_distancia_y_vencimiento);
  RUN_TEST(test_invalidas_reemplazan_la_lectura_anterior);
  RUN_TEST(test_checksum_no_renueva_y_recupera);
  RUN_TEST(test_cabecera_solapada_y_trama_parcial);
  RUN_TEST(test_limites_y_wrap_millis);
  return UNITY_END();
}
