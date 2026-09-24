@AGENTS.md

## Estado actual

- **Funciona (probado en placa):** toolchain y flasheo (`00-hola-mundo`). IMU en ROS 2 (`02-imu-ros2`): `/imu/data_raw` a 50,0 Hz por micro-ROS serial y por **WiFi UDP** (también sin USB, con otra fuente), con hora sincronizada con el agente y calibración del bias del gyro OK. Probado con `imu_filter_madgwick` y RViz2 (orientación OK) en la notebook y en la PC de escritorio. `01-imu-mpu9250`: el IMU es un **MPU-6500** (WHO_AM_I `0x70`, **sin magnetómetro**), así que el yaw deriva y no hay `/imu/mag`.
- **Falta:** opcional, un magnetómetro externo por I2C (por ejemplo QMC5883L en 0x0D) para corregir el yaw. `03-hcsr04` (idea: `sensor_msgs/Range` por micro-ROS para la altura). `04-gps`.
- **Pines:** I2C del IMU: SDA = GPIO8, SCL = GPIO9, 400 kHz, AD0→GND (0x68). NCS y FSYNC sin conectar. Prohibidos: GPIO19/20 (USB) y 26–37 (flash/PSRAM). En la Freenove, GPIO8/9 comparten líneas con la cámara.
- **Decisiones:**
  - PlatformIO + Arduino core 2.0.17, placa `esp32-s3-devkitc-1` con N16R8 (`qio_opi`, 16 MB).
  - Driver `MPU9250` propio, compartido por `symlink://` (no se copia).
  - micro-ROS Agent (Jazzy) compilado desde el código fuente en `~/microros_ws`: no hay paquete apt.
  - Publisher reliable con timeout de 10 ms. No se publica sin hora sincronizada. DLPF de 20 Hz contra el aliasing.
  - Solo orientación: la posición no se integra del acelerómetro porque deriva. La altura necesita otro sensor.
  - `autoRebuildAutocompleteIndex: false` en el workspace, para que VS Code no rompa el build de micro-ROS.
  - WiFi: `AGENT_IP` fija en `include/wifi_config.h` (no versionado); cambiar de PC o de IP exige recompilar `-e wifi`.
  - Codex solo revisa (`codex exec --sandbox read-only`).
