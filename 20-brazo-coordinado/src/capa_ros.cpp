// Capa micro-ROS de 20: tópicos en capa_ros.h. Solo se compila en el entorno `ros`. Conexión con el agente con
// EnlaceMicroRos (recolector_de_frutas/firmware/lib, el mismo del brazo y la base del robot).
#ifdef USAR_MICROROS

#include "capa_ros.h"

#include <Arduino.h>
#include <std_msgs/msg/string.h>

#include "EnlaceMicroRos.h"

extern Print *out;
void encolar(const char *texto);
void textoEstado(char *b, size_t cap);

namespace {

template <typename T>
inline void ignorar(T) {}

constexpr uint32_t PERIODO_MS = 100;  // /brazo/estado a 10 Hz
constexpr int PUB_TIMEOUT_MS = 10;
constexpr size_t N_HANDLES = 2;  // 1 suscripción + 1 timer

EnlaceMicroRos enlace;
BufferTexto salida;

rcl_publisher_t pubEstado, pubResp;
rcl_subscription_t subComando;
rcl_timer_t timer;
std_msgs__msg__String estadoMsg, respMsg, comandoIn;
char comandoBuf[64];
char estadoBuf[160];

void fijarTexto(rosidl_runtime_c__String &s, char *buf, size_t cap) {
  s.data = buf;
  s.size = 0;
  s.capacity = cap;
}

void alRecibirComando(const void *m) {
  const auto *msg = static_cast<const std_msgs__msg__String *>(m);
  char l[48];
  const size_t n = msg->data.size < sizeof(l) - 1 ? msg->data.size : sizeof(l) - 1;
  memcpy(l, msg->data.data, n);
  l[n] = '\0';
  encolar(l);
}

void alTimer(rcl_timer_t *t, int64_t) {
  if (!t) return;
  textoEstado(estadoBuf, sizeof(estadoBuf));
  estadoMsg.data.size = strlen(estadoBuf);
  ignorar(rcl_publish(&pubEstado, &estadoMsg, nullptr));
}

bool crear(rcl_node_t *node, rclc_support_t *support, rclc_executor_t *ex) {
  pubEstado = rcl_get_zero_initialized_publisher();
  pubResp = rcl_get_zero_initialized_publisher();
  subComando = rcl_get_zero_initialized_subscription();
  timer = rcl_get_zero_initialized_timer();
  const auto *tStr = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  if (rclc_publisher_init_default(&pubEstado, node, tStr, "brazo/estado") != RCL_RET_OK) return false;
  if (rclc_publisher_init_default(&pubResp, node, tStr, "brazo/respuesta") != RCL_RET_OK) return false;
  for (rcl_publisher_t *p : {&pubEstado, &pubResp}) EnlaceMicroRos::timeoutPublicador(p, PUB_TIMEOUT_MS);
  if (rclc_subscription_init_default(&subComando, node, tStr, "brazo/comando") != RCL_RET_OK) return false;
  if (rclc_timer_init_default2(&timer, support, RCL_MS_TO_NS(PERIODO_MS), alTimer, true) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(ex, &subComando, &comandoIn, alRecibirComando, ON_NEW_DATA) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_timer(ex, &timer) != RCL_RET_OK) return false;
  return true;
}

void destruir(rcl_node_t *node) {
  ignorar(rcl_timer_fini(&timer));
  ignorar(rcl_subscription_fini(&subComando, node));
  for (rcl_publisher_t *p : {&pubEstado, &pubResp}) ignorar(rcl_publisher_fini(p, node));
}

}  // namespace

void rosBegin() {
  Serial.begin(115200);
  out = &salida;
  comandoIn = {};
  fijarTexto(comandoIn.data, comandoBuf, sizeof(comandoBuf));
  estadoMsg = {};
  fijarTexto(estadoMsg.data, estadoBuf, sizeof(estadoBuf));
  respMsg = {};
  enlace.begin(Serial, "brazo", N_HANDLES, crear, destruir);
}

void rosLoop() {
  enlace.loop();
  if (enlace.seCorto()) encolar("s");  // sin agente: las tres se sostienen donde están
  if (enlace.conectado() && !salida.vacio()) {
    respMsg.data.data = const_cast<char *>(salida.texto());
    respMsg.data.size = salida.largo();
    respMsg.data.capacity = salida.largo() + 1;
    ignorar(rcl_publish(&pubResp, &respMsg, nullptr));
    salida.limpiar();
  }
}

#endif  // USAR_MICROROS
