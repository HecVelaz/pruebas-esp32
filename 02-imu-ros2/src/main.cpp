// Publica el MPU-9250/6500 en ROS 2 (micro-ROS) como sensor_msgs/Imu en /imu/data_raw a 50 Hz.
// Con transporte serial, el USB (/dev/ttyACM0) lo usa el micro-ROS Agent: no imprimir nada por Serial.
#include <Arduino.h>
#include <Wire.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/imu.h>

#include "MPU9250.h"
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

constexpr uint32_t PUBLISH_PERIOD_MS = 20;  // 50 Hz
constexpr char TOPIC[] = "imu/data_raw";
constexpr char NODE_NAME[] = "imu_node";
constexpr char FRAME_ID[] = "imu_link";

constexpr float GRAVITY = 9.80665f;  // [m/s²] por g

// DLPF de 20 Hz: por debajo de la frecuencia de Nyquist de la salida (25 Hz) para limitar el aliasing
// al tomar una muestra cada 20 ms de los 200 Hz internos
constexpr MPU9250::Dlpf IMU_DLPF = MPU9250::Dlpf::Hz20;

// Calibración del giroscopio al arrancar: hasta 5 intentos de 2 s (límites por defecto de GyroCalConfig)
constexpr int GYRO_CAL_ATTEMPTS = 5;

// Varianzas aproximadas a partir de la densidad de ruido del datasheet con DLPF 20 Hz
constexpr double ACCEL_VARIANCE = 3e-4;  // [(m/s²)²], ~0,017 m/s² RMS
constexpr double GYRO_VARIANCE = 1e-6;   // [(rad/s)²], ~0,06 °/s RMS
// Sin calibración, el bias puede llegar a ±5 °/s (ZRO del datasheet): (0,087 rad/s)²
constexpr double GYRO_VARIANCE_UNCALIBRATED = 7.6e-3;

// Tiempos de la comunicación con el agente. Cada espera bloquea el loop y, con él, al timer de 50 Hz:
// se mantienen muy por debajo del periodo de 20 ms.
constexpr int PUBLISH_TIMEOUT_MS = 10;        // espera máxima de confirmación del publisher reliable
constexpr uint32_t AGENT_PING_PERIOD_MS = 500;
constexpr int AGENT_PING_TIMEOUT_MS = 20;
constexpr uint8_t AGENT_MAX_MISSED_PINGS = 3;  // ~1,5 s sin respuesta = agente desconectado
constexpr uint32_t TIME_SYNC_RETRY_MS = 500;   // reintento mientras no hay hora válida
constexpr uint32_t TIME_SYNC_PERIOD_MS = 60000;
constexpr int TIME_SYNC_TIMEOUT_MS = 50;

MPU9250 imu(Wire, IMU_I2C_ADDR);

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
sensor_msgs__msg__Imu imuMsg;

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

void timerCallback(rcl_timer_t *t, int64_t /*lastCallTime*/) {
  if (t == nullptr) return;
  // Solo se publica con la hora del agente: un stamp desde el arranque rompería el dt de Madgwick y la TF
  if (!timeSynced) return;

  MPU9250::Vec3 a, g;
  float tempC;
  if (!imu.readAccelGyro(a, g, tempC)) return;

  // Una resincronización puede mover el reloj hacia atrás: descartar la muestra antes que publicar dt <= 0
  const int64_t ns = rmw_uros_epoch_nanos();
  if (ns <= lastStampNs) return;
  lastStampNs = ns;
  imuMsg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
  imuMsg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  imuMsg.linear_acceleration.x = a.x * GRAVITY;
  imuMsg.linear_acceleration.y = a.y * GRAVITY;
  imuMsg.linear_acceleration.z = a.z * GRAVITY;
  imuMsg.angular_velocity.x = g.x * DEG_TO_RAD;
  imuMsg.angular_velocity.y = g.y * DEG_TO_RAD;
  imuMsg.angular_velocity.z = g.z * DEG_TO_RAD;

  ignoreRet(rcl_publish(&publisher, &imuMsg, nullptr));
}

void initImuMsg(bool gyroCalibrated) {
  imuMsg = {};
  // frame_id apunta a un buffer estático: no requiere memoria dinámica
  imuMsg.header.frame_id.data = const_cast<char *>(FRAME_ID);
  imuMsg.header.frame_id.size = sizeof(FRAME_ID) - 1;
  imuMsg.header.frame_id.capacity = sizeof(FRAME_ID);

  // Sin estimación de orientación (la calcula imu_filter_madgwick): covarianza[0] = -1
  imuMsg.orientation_covariance[0] = -1.0;
  for (int i = 0; i < 3; i++) {
    imuMsg.angular_velocity_covariance[i * 4] = gyroCalibrated ? GYRO_VARIANCE : GYRO_VARIANCE_UNCALIBRATED;
    imuMsg.linear_acceleration_covariance[i * 4] = ACCEL_VARIANCE;
  }
}

bool createEntities() {
  allocator = rcl_get_default_allocator();
  support = {};
  node = rcl_get_zero_initialized_node();
  publisher = rcl_get_zero_initialized_publisher();
  timer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, NODE_NAME, "", &support) != RCL_RET_OK) return false;
  // Reliable: compatible con suscriptores reliable y best effort
  if (rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), TOPIC) !=
      RCL_RET_OK)
    return false;
  if (rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&publisher), PUBLISH_TIMEOUT_MS) !=
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
  ignoreRet(rcl_publisher_fini(&publisher, &node));
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
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);

  // Sin IMU no hay nada que publicar: reintentar hasta que responda
  while (imu.begin(IMU_DLPF) != MPU9250::Status::Ok) delay(1000);

  // La placa debe estar quieta durante la calibración (~2 s por intento).
  // Si falla en todos los intentos, se publica sin corregir el bias y con la covarianza del gyro ampliada.
  bool gyroCalibrated = false;
  for (int i = 0; i < GYRO_CAL_ATTEMPTS && !gyroCalibrated; i++) {
    gyroCalibrated = imu.calibrateGyro() == MPU9250::CalStatus::Ok;
    if (!gyroCalibrated) delay(500);
  }

  initImuMsg(gyroCalibrated);
  setupTransport();
}

void loop() {
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
