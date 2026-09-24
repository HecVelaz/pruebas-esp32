@AGENTS.md

## Estado actual

- **Funciona (probado en placa):** toolchain y flasheo (`00-hola-mundo`). IMU en ROS 2 (`02-imu-ros2`): `/imu/data_raw` a 50,0 Hz por micro-ROS serial, con hora sincronizada con el agente y calibración del bias del gyro OK. Probado con `imu_filter_madgwick` y RViz2 (orientación OK).
- **Compila, sin prueba en placa:** `01-imu-mpu9250` (escáner I2C, WHO_AM_I, magnetómetro AK8963) y el entorno `-e wifi` de `02` (no se compiló: falta `wifi_config.h`).
- **Falta:** confirmar el modelo del IMU (0x71 MPU-9250 o 0x70 MPU-6500) con `01`. Publicar `/imu/mag` si hay AK8963. Probar WiFi UDP. `03-hcsr04` (idea: `sensor_msgs/Range` por micro-ROS para la altura). `04-gps`.
- **Pines:** I2C del IMU: SDA = GPIO8, SCL = GPIO9, 400 kHz, AD0→GND (0x68). NCS y FSYNC sin conectar. Prohibidos: GPIO19/20 (USB) y 26–37 (flash/PSRAM). En la Freenove, GPIO8/9 comparten líneas con la cámara.
- **Decisiones:**
  - PlatformIO + Arduino core 2.0.17, placa `esp32-s3-devkitc-1` con N16R8 (`qio_opi`, 16 MB).
  - Driver `MPU9250` propio, compartido por `symlink://` (no se copia).
  - micro-ROS Agent (Jazzy) compilado desde el código fuente en `~/microros_ws`: no hay paquete apt.
  - Publisher reliable con timeout de 10 ms. No se publica sin hora sincronizada. DLPF de 20 Hz contra el aliasing.
  - Solo orientación: la posición no se integra del acelerómetro porque deriva. La altura necesita otro sensor.
  - `autoRebuildAutocompleteIndex: false` en el workspace, para que VS Code no rompa el build de micro-ROS.
  - Codex solo revisa (`codex exec --sandbox read-only`).
