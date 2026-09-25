@AGENTS.md

## Estado actual

- **Funciona (probado en placa):** toolchain y flasheo (`00-hola-mundo`). IMU en ROS 2 (`02-imu-ros2`): `/imu/data_raw` a 50,0 Hz por micro-ROS serial y por **WiFi UDP** (también sin USB, con otra fuente), con hora sincronizada con el agente y calibración del bias del gyro OK. Probado con `imu_filter_madgwick` y RViz2 (orientación OK) en la notebook y en la PC de escritorio. `01-imu-mpu9250`: el IMU es un **MPU-6500** (WHO_AM_I `0x70`, **sin magnetómetro**), así que el yaw deriva y no hay `/imu/mag`.
- **GPS:** Fastrax UP501 por UART1 a 9600 8N1. **NMEA OK en la S3 (2026-09-25, interiores, sin fix)** con `04 -e uart` por el conector CH343. Si se invierten TX/RX, el GPS queda mudo hasta cortarle la alimentación. `04-gps-up501` (resumen o NMEA crudo por serial) y `05-gps-ros2` (`/gps/fix` NavSatFix + `/gps/status` String a 1 Hz). Driver `lib/UP501` sobre TinyGPSPlus, compartido por `symlink://`. Prueba previa con Arduino en un ESP32 clásico: NMEA OK, sin fix en interiores.
- **Falta:** fix del GPS al aire libre y probar `05-gps-ros2` (por WiFi). Opcional, un magnetómetro externo por I2C (por ejemplo QMC5883L en 0x0D) para corregir el yaw. `03-hcsr04` (idea: `sensor_msgs/Range` por micro-ROS para la altura). Lidar por UART2 (pines reservados).
- **Pines:** ver la tabla de abajo. I2C del IMU a 400 kHz, AD0→GND (0x68). NCS y FSYNC del IMU sin conectar.

### Pines del ESP32-S3 (Freenove N16R8)

| GPIO | Uso | Estado |
|------|-----|--------|
| 8 | I2C SDA del IMU (MPU-6500) | ocupado |
| 9 | I2C SCL del IMU | ocupado |
| 41 | RX1 ← TXD del GPS (pin 2 del UP501) | ocupado |
| 42 | TX1 → RXD del GPS (pin 1 del UP501) | ocupado |
| 47 | PPS del GPS | reservado (sin conectar) |
| 14 | RX2 ← TX del lidar | reservado |
| 21 | TX2 → RX del lidar | reservado |
| 19, 20 | USB nativo (`/dev/ttyACM0`, micro-ROS serial) | prohibido |
| 26–37 | flash y PSRAM octal | prohibido |
| 0, 3, 45, 46 | strapping | evitar |
| 43, 44 | UART0, conectado al CH343 del segundo USB de la placa | evitar |
| 4–7, 10–13, 15–18 | cámara (8/9 también, compartidos con el IMU) | libres si no se usa la cámara |
| 38–40 | ranura SD | libres si no se usa la SD |
| 2, 48 | LED y LED RGB de la placa | evitar |
| 1 | — | libre |

Alimentación: IMU y GPS en 3V3 (el UP501 en VDD y VDD_B). UARTs: UART0 libre (CH343), UART1 = GPS, UART2 = lidar.
- **Decisiones:**
  - PlatformIO + Arduino core 2.0.17, placa `esp32-s3-devkitc-1` con N16R8 (`qio_opi`, 16 MB).
  - Driver `MPU9250` propio, compartido por `symlink://` (no se copia).
  - micro-ROS Agent (Jazzy) compilado desde el código fuente en `~/microros_ws`: no hay paquete apt.
  - Publisher reliable con timeout de 10 ms. No se publica sin hora sincronizada. DLPF de 20 Hz contra el aliasing.
  - Solo orientación: la posición no se integra del acelerómetro porque deriva. La altura necesita otro sensor.
  - `autoRebuildAutocompleteIndex: false` en el workspace, para que VS Code no rompa el build de micro-ROS.
  - WiFi: `AGENT_IP` fija en `include/wifi_config.h` (no versionado); cambiar de PC o de IP exige recompilar `-e wifi`.
  - Codex solo revisa (`codex exec --sandbox read-only`).
