@AGENTS.md

## Estado actual

- **Funciona (probado en placa):** toolchain y flasheo (`00-hola-mundo`). IMU en ROS 2 (`02-imu-ros2`): `/imu/data_raw` a 50,0 Hz por micro-ROS serial y por **WiFi UDP** (también sin USB, con otra fuente), con hora sincronizada con el agente y calibración del bias del gyro OK. Probado con `imu_filter_madgwick` y RViz2 (orientación OK) en la notebook y en la PC de escritorio. `01-imu-mpu9250`: el IMU es un **MPU-6500** (WHO_AM_I `0x70`, **sin magnetómetro**), así que el yaw deriva y no hay `/imu/mag`.
- **GPS:** Fastrax UP501 por UART1 a 9600 8N1. **NMEA OK en la S3 (2026-09-25, interiores, sin fix)** con `04 -e uart` por el conector CH343. Si se invierten TX/RX, el GPS queda mudo hasta cortarle la alimentación. `04-gps-up501` (resumen o NMEA crudo por serial) y `05-gps-ros2` (`/gps/fix` NavSatFix + `/gps/status` String a 1 Hz). Driver `lib/UP501` sobre TinyGPSPlus, compartido por `symlink://`. Prueba previa con Arduino en un ESP32 clásico: NMEA OK, sin fix en interiores.
- **GPS en ROS 2 por WiFi OK (2026-09-25):** `/gps/fix` con fix al aire libre, primer fix en frío ~2 min, 4–5 satélites, HDOP ~1,4, posición verificada en el mapa. La protoboard daba falsos contactos: el GPS va conectado directo a la S3. Firmware con un `/gps/fix` por epoch (stamp = hora UTC del GPS) y diagnósticos en `/gps/status`; con `WiFi.setSleep(false)`, 0 pérdidas y 0 reconexiones en 2 min (interiores). **`02-imu-ros2` no desactiva el ahorro de energía del WiFi.** El UP501 tiene el **rollover de semana GPS** (fecha 1024 semanas atrás, corregido en `05`) y, tras cada arranque en frío, **UTC ~3 s adelantada** durante ~11 min (segundos intercalares de fábrica): mientras tanto el stamp es la hora de llegada.
- **Pendiente GPS:**
  - Correcciones sugeridas por Codex (sin aplicar) en `05-gps-ros2/src/main.cpp`: (1) ventana **asimétrica** para aceptar la UTC del GPS como stamp (llegada − UTC entre −0,2 s y +1,5 s, en vez de ±2 s), para no publicar stamps futuros durante los ~10 min con segundos intercalares viejos; (2) mostrar fecha y `rollover` en `/gps/status` también cuando el stamp es `UTC GPS`.
  - `05-gps-ros2 -e serial` (cable) compila pero no se probó con el GPS: se decidió usar solo WiFi.
  - Soldar o fijar los cables del GPS: se desconecta al mover la placa.
- **Motor en lazo abierto OK en placa (2026-09-25):** `06-motor-36gp555`. Motor 36GP-555 de 12 V, 160 rpm, eje de 8 mm, reductora 50:1 (confirmada por el máximo medido), encoder Hall 16 PPR (x4 por PCNT = 3200 cuentas por vuelta de la salida), placa del encoder **SCX-555** (colores: rojo = Signal B, negro = Signal A, verde = Encoder+, azul = Encoder−, amarillo = Motor−, blanco = Motor+). Driver IBT-2 (BTS7960) con VCC a 3V3, PWM 20 kHz, fuente 12 V / 3 A. Barrido: lineal, **1,61 rpm/%, zona muerta ~11 %, máx 142 rpm**, simétrico. Escalón 50 %: **τ ≈ 74 ms**, 62,6 rpm. Desde parado no arranca con 20 % (sí con 30 %). Script `tools/graficar.py` (barrido y escalón → CSV + PNG en `resultados/`). Protecciones: rampa 50 %/s, corte por atasco, no invierte sin el motor quieto. Revisado por Codex (10 hallazgos, aplicados). **Pendiente:** pull-down de 10 kΩ en R_EN/L_EN (sin ellos: apagar los 12 V antes de flashear o resetear); avisar cuando se descarta la línea que corta `n`/barrido/escalón; lazo cerrado (PID con feedforward de la zona muerta).
- **Falta:** opcional, un magnetómetro externo por I2C (por ejemplo QMC5883L en 0x0D) para corregir el yaw. `03-hcsr04` (idea: `sensor_msgs/Range` por micro-ROS para la altura). Lidar por UART2 (pines reservados).
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
| 10–13, 17, 18 | cámara (también 4–7, 15, 16 y 8/9, ya usados) | libres si no se usa la cámara |
| 38–40 | ranura SD | libres si no se usa la SD |
| 2, 48 | LED y LED RGB de la placa | evitar |
| 4 | RPWM del IBT-2 (motor) | ocupado |
| 5 | LPWM del IBT-2 | ocupado |
| 6 | R_EN del IBT-2 | ocupado |
| 7 | L_EN del IBT-2 | ocupado |
| 15 | Fase A del encoder del motor | ocupado |
| 16 | Fase B del encoder del motor | ocupado |
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
  - Roles: en modo normal Codex solo revisa (`codex exec --sandbox read-only`). Si el límite semanal de Claude Code se agota, Codex puede editar y hacer commits (modo respaldo) hasta que se reinicie. Ver "Roles" y "Continuidad entre agentes" en `AGENTS.md`: revisar `git log` antes de asumir continuidad de contexto.
