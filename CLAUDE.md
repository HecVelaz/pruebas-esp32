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
- **Perfil (`06 -e`, comando `p`, `graficar.py perfil`) OK en placa (2026-09-25, fuente de laboratorio APS3005SI):** 4 perfiles repetibles: **K ≈ 1,58 rpm/%, zona muerta ~8 %, τ ≈ 60–70 ms, máx ~145 rpm**. La APS3005SI con límite de corriente bajo entra en CC y el barrido "cae" desde el 40 % (salta el atasco): límite en 3 A.
- **`07-motor-rampa-vueltas` OK en placa (2026-09-25):** rampa 0 → 100 % por tiempo que termina a N vueltas de salida; por defecto **3 vueltas en 3 s** (segundos = vueltas: sin carga llega a ~99 % en la última vuelta). Firmware propio (comando `r [s] [vueltas]`) con los drivers de 06 por symlink; `tools/rampa.py` (4 gráficos, arranque, duty por vuelta, K y zona muerta compensando τ, `--comparar`). Sin carga: arranca con ~16 %, K ≈ 1,6 rpm/%. Con carga: arranca con 25 %, misma pendiente pero zona muerta ~23 %, −21 rpm a 50 % y máx ~120 rpm, velocidad ondulada. Arriba de ~90 % de PWM la curva salta y se aplana (probable límite de conmutación del BTS7960 a 20 kHz): rango lineal ~20–85 %.
- **`08-motor-5840-l298n` en placa (2026-09-25):** segundo motor, **ESP32-WROOM-32D** (DevKit con CH340, `/dev/ttyUSB0`, `board = esp32dev`). Motor 5840-31ZY (sin fin autobloqueante, 12 V, 160 rpm, 100 kg·cm), driver **L298N** (ENA=GPIO25, IN1=26, IN2=27; jumper de ENA **afuera**; 2 A máx; **9,6 V en el motor a 100 %** con 12 V), encoder externo **38S6G5-B-G24N** en la salida (1000 PPR → 4000 cuentas por vuelta, NPN colector abierto: rojo a 5V/VIN, negro GND, blanco A=GPIO32, verde B=GPIO33, pull-ups internos: no hay de 4,7 kΩ; **cuenta al revés con este acople → `ENCODER_INVERTIDO = true`**). Abrir el puerto no reinicia el ESP32. Perfil con PWM en ENA (rueda libre): curva **saturada** (40 % → 86 rpm, 100 % → 141 rpm), modelo lineal inválido, velocidad ondulada (¿acople?). Por eso `lib/L298N` tiene ahora `Modo::Freno` (por defecto: ENA fijo y PWM en IN1/IN2, freno en la parte apagada) y `Modo::RuedaLibre`; sin pulsos al cambiar de estado. Aplicados los arreglos de la revisión de Codex en 08: Ctrl+C siempre frena, datos guardados si la prueba se corta (`*_cortado.csv`), τ solo con giro estable, mensaje de atasco con el duty real, README con la pérdida real del L298N. **Pendiente:** perfil con `Modo::Freno` (flasheado, sin medir); pull-down de 10 kΩ en ENA; arreglo del salto falso de 32 000 cuentas en `EncoderPCNT` (biblioteca compartida con 06/07, sin aplicar); los mismos arreglos en `graficar.py`/`main.cpp` de 06 (sin aplicar, a pedido).
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
