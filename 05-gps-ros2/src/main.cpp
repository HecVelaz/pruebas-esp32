// Publica el GPS Fastrax UP501 en ROS 2 (micro-ROS) a 1 Hz:
//   /gps/fix    sensor_msgs/NavSatFix (estándar, para robot_localization, mapviz, etc.)
//   /gps/status std_msgs/String (diagnóstico legible: sin datos, sin fix o posición)
// Con transporte serial, el USB (/dev/ttyACM0) lo usa el micro-ROS Agent: no imprimir nada por Serial.
// El GPS entra por UART1 (Serial1), que es independiente del USB.
#include <Arduino.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <std_msgs/msg/string.h>

#include "UP501.h"
#include "pins.h"

#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
#if !__has_include("wifi_config.h")
#error "Falta include/wifi_config.h: copiar include/wifi_config.example.h y completar SSID, clave e IP del agente"
#endif
#include "wifi_config.h"
#elif !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error "Transporte micro-ROS no soportado: usar board_microros_transport = serial o wifi"
#endif

namespace {

// Descarta a propósito el código de retorno (rcl/rmw usan warn_unused_result y (void) no lo silencia en GCC)
template <typename T>
inline void ignoreRet(T) {}

constexpr uint32_t PUBLISH_PERIOD_MS = 1000;  // 1 Hz, igual que las tramas del UP501
constexpr char FIX_TOPIC[] = "gps/fix";
constexpr char STATUS_TOPIC[] = "gps/status";
constexpr char NODE_NAME[] = "gps_node";
constexpr char FRAME_ID[] = "gps_link";

// Covarianza aproximada a partir del HDOP: σ horizontal = HDOP × error de rango típico de un GPS sin
// corrección; la vertical suele ser del doble.
constexpr double UERE_M = 4.0;

// Tiempos de la comunicación con el agente. Mientras dura cada espera, los bytes del GPS se acumulan en
// el búfer de 1 kB de la UART (~1 s a 9600 baud), así que no se pierden.
constexpr int PUBLISH_TIMEOUT_MS = 50;  // espera máxima de confirmación de cada publisher reliable
constexpr uint32_t AGENT_PING_PERIOD_MS = 500;
constexpr int AGENT_PING_TIMEOUT_MS = 20;
constexpr uint8_t AGENT_MAX_MISSED_PINGS = 3;  // ~1,5 s sin respuesta = agente desconectado
constexpr uint32_t TIME_SYNC_RETRY_MS = 500;   // reintento mientras no hay hora válida
constexpr uint32_t TIME_SYNC_PERIOD_MS = 60000;
constexpr int TIME_SYNC_TIMEOUT_MS = 50;

UP501 gps(Serial1);

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t fixPublisher;
rcl_publisher_t statusPublisher;
rcl_timer_t timer;
rclc_executor_t executor;
sensor_msgs__msg__NavSatFix fixMsg;
std_msgs__msg__String statusMsg;
char statusBuf[160];

enum class AgentState { WaitingAgent, AgentAvailable, AgentConnected, AgentDisconnected };
AgentState state = AgentState::WaitingAgent;
uint32_t lastPingMs = 0;
uint8_t missedPings = 0;
uint32_t lastSyncMs = 0;
bool timeSynced = false;  // hubo una sincronización válida en la sesión actual
int64_t lastStampNs = 0;

bool syncTime(int timeoutMs) {
  if (rmw_uros_sync_session(timeoutMs) == RMW_RET_OK && rmw_uros_epoch_synchronized()) timeSynced = true;
  return timeSynced;
}

// Completa /gps/fix y /gps/status con el estado actual del receptor. El texto va sin tildes para que
// ros2 topic echo lo muestre sin escapes.
void fillMessages() {
  UP501::Fix f;
  const UP501::State st = gps.state();
  const bool hasFix = gps.readFix(f);

  if (hasFix) {
    if (f.quality == 2) fixMsg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX;
    else fixMsg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
    fixMsg.latitude = f.latDeg;
    fixMsg.longitude = f.lonDeg;
    // NavSatFix pide altura sobre el elipsoide WGS 84 = altura MSL de GGA + separación del geoide.
    // Si el receptor no envía la separación, queda la altura MSL (error de decenas de metros).
    fixMsg.altitude = isnan(f.geoidSepM) ? f.altMslM : f.altMslM + f.geoidSepM;
    if (!isnan(f.hdop)) {
      const double h = f.hdop * UERE_M;
      fixMsg.position_covariance[0] = h * h;
      fixMsg.position_covariance[4] = h * h;
      fixMsg.position_covariance[8] = 4.0 * h * h;
      fixMsg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;
    } else {
      fixMsg.position_covariance[0] = fixMsg.position_covariance[4] = fixMsg.position_covariance[8] = 0.0;
      fixMsg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
    }
  } else {
    fixMsg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
    fixMsg.latitude = fixMsg.longitude = fixMsg.altitude = NAN;
    fixMsg.position_covariance[0] = fixMsg.position_covariance[4] = fixMsg.position_covariance[8] = 0.0;
    fixMsg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_UNKNOWN;
  }

  int n = 0;
  switch (st) {
    case UP501::State::NoData:
      n = snprintf(statusBuf, sizeof(statusBuf), "ERROR: sin datos del GPS | Revisar TX/RX y alimentacion");
      break;
    case UP501::State::BadData:
      n = snprintf(statusBuf, sizeof(statusBuf), "ERROR: tramas NMEA invalidas | Revisar baudrate (%lu)",
                   (unsigned long)GPS_BAUD);
      break;
    case UP501::State::NoFix:
      n = snprintf(statusBuf, sizeof(statusBuf), "SIN FIX | Satelites visibles: %d | Buscando posicion...",
                   max(gps.satellitesInView(), 0));
      break;
    case UP501::State::Fix:
      n = snprintf(statusBuf, sizeof(statusBuf),
                   "GPS OK | Satelites: %u | Lat: %.6f | Lon: %.6f | Alt: %.1f m | HDOP: %.2f", f.satsUsed,
                   f.latDeg, f.lonDeg, f.altMslM, f.hdop);
      break;
  }
  if (!timeSynced && n >= 0 && (size_t)n < sizeof(statusBuf)) {
    n += snprintf(statusBuf + n, sizeof(statusBuf) - n, " | /gps/fix espera la hora del agente");
  }
  statusMsg.data.size = strnlen(statusBuf, sizeof(statusBuf) - 1);
}

void timerCallback(rcl_timer_t *t, int64_t /*lastCallTime*/) {
  if (t == nullptr) return;
  fillMessages();

  // /gps/status no tiene header: se publica siempre, para diagnosticar aunque no haya hora
  ignoreRet(rcl_publish(&statusPublisher, &statusMsg, nullptr));

  // /gps/fix solo con la hora del agente, con stamps siempre crecientes
  if (!timeSynced) return;
  const int64_t ns = rmw_uros_epoch_nanos();
  if (ns <= lastStampNs) return;
  lastStampNs = ns;
  fixMsg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
  fixMsg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  ignoreRet(rcl_publish(&fixPublisher, &fixMsg, nullptr));
}

void initMessages() {
  fixMsg = {};
  // frame_id y el texto de estado apuntan a buffers estáticos: no requieren memoria dinámica
  fixMsg.header.frame_id.data = const_cast<char *>(FRAME_ID);
  fixMsg.header.frame_id.size = sizeof(FRAME_ID) - 1;
  fixMsg.header.frame_id.capacity = sizeof(FRAME_ID);
  // El UP501 solo recibe GPS (no GLONASS, Galileo ni BeiDou)
  fixMsg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  statusMsg = {};
  statusBuf[0] = '\0';
  statusMsg.data.data = statusBuf;
  statusMsg.data.size = 0;
  statusMsg.data.capacity = sizeof(statusBuf);
}

bool createEntities() {
  allocator = rcl_get_default_allocator();
  support = {};
  node = rcl_get_zero_initialized_node();
  fixPublisher = rcl_get_zero_initialized_publisher();
  statusPublisher = rcl_get_zero_initialized_publisher();
  timer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) return false;
  // Reliable: compatible con suscriptores reliable y best effort
  if (rclc_publisher_init_default(&fixPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
                                  FIX_TOPIC) != RCL_RET_OK)
    return false;
  if (rclc_publisher_init_default(&statusPublisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                  STATUS_TOPIC) != RCL_RET_OK)
    return false;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&fixPublisher), PUBLISH_TIMEOUT_MS) !=
      RMW_RET_OK)
    return false;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&statusPublisher), PUBLISH_TIMEOUT_MS) !=
      RMW_RET_OK)
    return false;
  if (rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timerCallback, true) !=
      RCL_RET_OK)
    return false;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;

  timeSynced = false;
  lastStampNs = 0;
  missedPings = 0;
  syncTime(1000);
  lastSyncMs = millis();
  return true;
}

void destroyEntities() {
  // El agente ya no responde: no esperar confirmaciones al destruir
  rmw_context_t *rmwContext = rcl_context_get_rmw_context(&support.context);
  if (rmwContext) ignoreRet(rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0));

  ignoreRet(rclc_executor_fini(&executor));
  ignoreRet(rcl_timer_fini(&timer));
  ignoreRet(rcl_publisher_fini(&statusPublisher, &node));
  ignoreRet(rcl_publisher_fini(&fixPublisher, &node));
  ignoreRet(rcl_node_fini(&node));
  ignoreRet(rclc_support_fini(&support));
}

bool pingAgent(int timeoutMs, uint8_t attempts) { return rmw_uros_ping_agent(timeoutMs, attempts) == RMW_RET_OK; }

void setupTransport() {
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  static char ssid[] = WIFI_SSID;
  static char password[] = WIFI_PASSWORD;
  set_microros_wifi_transports(ssid, password, IPAddress(AGENT_IP), AGENT_PORT);
#else
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
#endif
}

}  // namespace

void setup() {
  // A 9600 baud llegan ~1 kB/s: un búfer de 1 kB cubre las esperas de la comunicación con el agente
  Serial1.setRxBufferSize(1024);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);

  initMessages();
  setupTransport();
}

void loop() {
  // El GPS se lee en cada vuelta, haya o no agente
  gps.update();

  const uint32_t now = millis();

  switch (state) {
    case AgentState::WaitingAgent:
      if (now - lastPingMs >= AGENT_PING_PERIOD_MS) {
        lastPingMs = now;
        if (pingAgent(AGENT_PING_TIMEOUT_MS, 1)) state = AgentState::AgentAvailable;
      }
      break;

    case AgentState::AgentAvailable:
      if (createEntities()) {
        state = AgentState::AgentConnected;
      } else {
        destroyEntities();
        state = AgentState::WaitingAgent;
      }
      break;

    case AgentState::AgentConnected:
      // Un ping por periodo con timeout corto; se declara desconexión tras varios fallos seguidos
      if (now - lastPingMs >= AGENT_PING_PERIOD_MS) {
        lastPingMs = now;
        missedPings = pingAgent(AGENT_PING_TIMEOUT_MS, 1) ? 0 : missedPings + 1;
        if (missedPings >= AGENT_MAX_MISSED_PINGS) {
          state = AgentState::AgentDisconnected;
          break;
        }
      }
      if (now - lastSyncMs >= (timeSynced ? TIME_SYNC_PERIOD_MS : TIME_SYNC_RETRY_MS)) {
        lastSyncMs = now;
        syncTime(TIME_SYNC_TIMEOUT_MS);
      }
      ignoreRet(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
      break;

    case AgentState::AgentDisconnected:
      destroyEntities();
      state = AgentState::WaitingAgent;
      break;
  }
}
