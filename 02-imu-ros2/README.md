# 02-imu-ros2 — IMU MPU-9250/6500 en ROS 2 Jazzy con micro-ROS

El ESP32-S3 lee el IMU y publica `sensor_msgs/Imu` en **`/imu/data_raw`** a **50 Hz**. La aceleración va en m/s², la velocidad angular en rad/s y el `frame_id` es `imu_link`. El bias del giroscopio se calibra al arrancar.

## Arquitectura

```
┌──────────── ESP32-S3 (micro-ROS) ────────────┐          ┌──────────────── PC Ubuntu 24.04 (ROS 2 Jazzy) ────────────────┐
│ MPU-9250 ─I2C─> driver MPU9250               │          │                                                                │
│               └> nodo imu_node (rclc)        │  XRCE-   │ micro_ros_agent ──DDS──> /imu/data_raw ──> imu_filter_madgwick │
│                  timer 50 Hz -> publisher    │ ─DDS───> │  (proxy)                                   │                   │
│                  rmw_microxrcedds            │  serial  │                                            ├─> /imu/data       │
│                  transporte serial / UDP     │  o UDP   │                                            └─> TF odom→imu_link│
└──────────────────────────────────────────────┘          │                                   ros2 topic echo/hz, RViz2   │
                                                          └────────────────────────────────────────────────────────────────┘
```

- **micro-ROS** es ROS 2 para microcontroladores. El firmware usa la API `rclc` (nodos, publishers y timers, como en `rclcpp`), pero en lugar de DDS usa **Micro XRCE-DDS**, un protocolo liviano cliente-servidor.
- El **micro-ROS Agent** corre en la PC y hace de puente: recibe el tráfico XRCE-DDS por serial o UDP y crea el nodo y el publisher equivalentes en DDS. Para el resto de ROS 2, `imu_node` aparece como un nodo más.
- El agente también le da la hora al ESP32 (`rmw_uros_sync_session`), así los `header.stamp` quedan en la misma base de tiempo que la PC. Sin eso, TF y RViz2 descartan los mensajes por timestamps viejos.
- **imu_filter_madgwick** fusiona acelerómetro y giroscopio y calcula la orientación. La publica en `/imu/data` y como TF `odom → imu_link`, que es lo que se ve en RViz2.

## Conexiones

Mismo cableado que `01-imu-mpu9250`, en la placa Freenove ESP32-S3-WROOM (N16R8) con breakout Freenove v1.2:

| Módulo MPU | Breakout Freenove v1.2 | Nota |
|------------|------------------------|------|
| VCC        | **3V3**                | |
| GND        | **GND**                | |
| SDA        | **GPIO8**              | `PIN_I2C_SDA` en `include/pins.h` |
| SCL        | **GPIO9**              | `PIN_I2C_SCL` en `include/pins.h` |
| AD0        | **GND**                | Dirección I2C `0x68` (`IMU_I2C_ADDR`) |

Ver las notas de pines del README de `01-imu-mpu9250`: GPIO 26–37 están prohibidos en el N16R8, y GPIO 8/9 comparten líneas con la cámara.

## Firmware

| Parámetro | Valor |
|-----------|-------|
| Nodo | `imu_node` |
| Tópico | `/imu/data_raw` (`sensor_msgs/msg/Imu`), QoS reliable, compatible con suscriptores reliable y best effort |
| Frecuencia | **50 Hz nominales** (timer rclc de 20 ms). Ver "Frecuencia real" más abajo |
| Muestreo del sensor | 200 Hz internos, **DLPF 20 Hz**: queda por debajo de los 25 Hz de Nyquist de la salida y limita el aliasing |
| Unidades | `linear_acceleration` en m/s² (incluye la gravedad), `angular_velocity` en rad/s |
| `orientation` | no se estima: `orientation_covariance[0] = -1` |
| Covarianzas | diagonales aproximadas según el datasheet con DLPF 20 Hz: acel. 3e-4 (m/s²)², gyro 1e-6 (rad/s)². Si la calibración falló, gyro 7,6e-3 (bias de hasta ±5 °/s) |
| `header.stamp` | hora del agente. **No se publica nada hasta tener una sincronización válida** (reintento cada 500 ms; después se resincroniza cada 60 s). Los stamps son siempre crecientes: si una resincronización mueve el reloj hacia atrás, se descarta esa muestra |
| Driver | `01-imu-mpu9250/lib/MPU9250` enlazado con `symlink://` en `lib_deps` (no se copia) |

**Calibración del giroscopio:** al arrancar, el firmware promedia 400 lecturas en 2 s y guarda el resultado como bias.
- **Dejar la placa quieta y apoyada** unos segundos después de conectarla o pulsar RST.
- Un intento se rechaza en cuanto pasa algo de esto (límites de `GyroCalConfig` en el driver):
  - Un eje del giroscopio varía más de 3 °/s, o uno del acelerómetro más de 0,05 g: hubo movimiento.
  - El módulo de la aceleración se aleja más de 0,1 g de 1 g: hay aceleración lineal.
  - Algún eje supera 10 °/s: el datasheet admite hasta ±5 °/s de offset, así que un valor mayor indica casi seguro un giro constante.
- Reintenta hasta 5 veces. Si no lo logra, publica sin corregir el bias y con la covarianza del giroscopio ampliada, para que los consumidores sepan que es menos confiable.
- Limitación: un giro lento y constante alrededor de la vertical (menos de 10 °/s) no altera el acelerómetro y no se distingue de un offset. Por eso la placa tiene que estar quieta.
- Para comprobarlo: en reposo, `angular_velocity` debería quedar en unos ±0,005 rad/s, y `angular_velocity_covariance[0]` en `1e-06`.

**Reconexión:** el firmware hace un ping al agente cada 500 ms y da la conexión por perdida después de 3 pings sin respuesta (~1,5 s). Entonces destruye las entidades y vuelve a esperarlo. Se puede reiniciar el agente sin resetear la placa.

**Frecuencia real:** el timer, la comunicación y la lectura I2C comparten el mismo `loop()`, así que cada espera retrasa al timer. Por eso cada espera está acotada muy por debajo de los 20 ms del periodo:

| Espera | Máximo |
|--------|--------|
| Confirmación del publisher reliable (`rmw_uros_set_publisher_session_timeout`) | 10 ms |
| Ping al agente (uno cada 500 ms) | 20 ms |
| Resincronización de hora (una cada 60 s) | 50 ms |

Con el agente respondiendo, la frecuencia debería estar muy cerca de 50 Hz. Si se pierde la confirmación de un mensaje, esa muestra puede llegar tarde o no llegar. Medir con `ros2 topic hz /imu/data_raw` (promedio y `min`/`max` del periodo) y anotarlo en **Resultados**. Si hace falta una adquisición continua sin huecos, el paso siguiente es leer el sensor a 200 Hz en otra tarea de FreeRTOS con un búfer, separado de la comunicación.

Con transporte serial, el USB es del agente: **el firmware no imprime nada por `Serial`** y `pio device monitor` no muestra texto útil. Para diagnosticar el sensor, flashear `01-imu-mpu9250`.

## Compilar y flashear

```bash
cd ~/pruebas-esp32/02-imu-ros2
~/.platformio/penv/bin/pio run -e serial -t upload
```

- La primera compilación descarga y compila micro-ROS Jazzy para el ESP32-S3, y tarda varios minutos. Queda en caché en `.pio/libdeps/`.
- **Detener el micro-ROS Agent antes de flashear**, porque tiene abierto `/dev/ttyACM0`.
- Si cambia `board_microros_distro` o `board_microros_transport`, borrar `.pio/libdeps/<entorno>/micro_ros_platformio/libmicroros` para forzar la recompilación.

## Instalar el micro-ROS Agent (Jazzy)

No hay paquete apt de `micro_ros_agent` para Jazzy, así que se compila una sola vez con `micro_ros_setup`:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/microros_ws/src && cd ~/microros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash   # carga el agente recién compilado en esta terminal
```

Para no repetir el `source` en cada terminal:

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source ~/microros_ws/install/local_setup.bash' >> ~/.bashrc
```

## Correr el agente

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

- Con el USB nativo del S3 la velocidad en baudios no importa, pero el agente la exige.
- Al conectar, el agente muestra `session established` y luego `create_publisher`/`create_datawriter`.
- Si no aparece, pulsar RST en la placa, dejarla quieta durante la calibración y esperar unos segundos.
- Para más detalle, agregar `-v6` al comando del agente.

## Verificar con la CLI de ROS 2

En otra terminal (con `source /opt/ros/jazzy/setup.bash`):

```bash
ros2 node list                    # debe aparecer /imu_node
ros2 topic list                   # debe aparecer /imu/data_raw
ros2 topic info /imu/data_raw -v  # tipo sensor_msgs/msg/Imu, QoS RELIABLE
ros2 topic hz /imu/data_raw       # ~50 Hz
ros2 topic echo /imu/data_raw     # un mensaje cada 20 ms
ros2 topic echo /imu/data_raw --field linear_acceleration
```

Valores esperados con la placa quieta y plana:
- `linear_acceleration.z ≈ +9.8` y `x, y ≈ 0`: el acelerómetro mide la reacción a la gravedad.
- `angular_velocity ≈ 0`, gracias a la calibración del bias.
- `header.frame_id: imu_link` y `header.stamp` con la hora actual (comparar con `date +%s`).

## Visualizar con imu_filter_madgwick y RViz2

Instalar las herramientas (filtro y plugin de RViz2):

```bash
sudo apt install ros-jazzy-imu-tools
```

Terminal 1 (agente, ver arriba). Terminal 2, el filtro, que lee `imu/data_raw` y publica `imu/data` y la TF:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  -p use_mag:=false \
  -p world_frame:=enu \
  -p publish_tf:=true \
  -p fixed_frame:=odom
```

Terminal 3:

```bash
rviz2
```

En RViz2:
1. **Global Options → Fixed Frame:** `odom`.
2. **Add → TF**: se ve el eje `imu_link` rotando con la placa.
3. **Add → By topic → `/imu/data` → Imu** (plugin `rviz_imu_plugin`): muestra la orientación como caja o ejes, y la aceleración como vector.

Sin magnetómetro (`use_mag:=false`), roll y pitch son estables, pero el **yaw deriva** lentamente: es normal. Si el módulo es MPU-9250 con AK8963, habría que publicar además `/imu/mag` (`sensor_msgs/MagneticField`) y usar `use_mag:=true`. Esa parte todavía no está implementada.

## Cambiar a WiFi UDP (preparado, sin probar)

El entorno `wifi` de `platformio.ini` ya está definido y `main.cpp` elige el transporte según `board_microros_transport`.

1. Copiar la configuración y completarla con la red y la IP de la PC:
   ```bash
   cp include/wifi_config.example.h include/wifi_config.h   # está en .gitignore
   ```
2. Compilar y flashear el entorno WiFi (la primera vez recompila micro-ROS para UDP):
   ```bash
   ~/.platformio/penv/bin/pio run -e wifi -t upload
   ```
3. Correr el agente en UDP:
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
   ```
4. Abrir el puerto si hay firewall: `sudo ufw allow 8888/udp`.

El ESP32-S3 solo usa WiFi de 2,4 GHz. `set_microros_wifi_transports` bloquea hasta conectarse a la red. Con WiFi, `Serial` queda libre y se pueden agregar mensajes de depuración.

## Solución de problemas

| Síntoma | Causa probable |
|---------|----------------|
| `pio run -t upload`: *port is busy* | El agente o un monitor tiene abierto `/dev/ttyACM0`. |
| El agente no muestra `session established` | El IMU no responde (el firmware reintenta `begin()` y no llega a conectar). Flashear `01-imu-mpu9250` para diagnosticar. |
| El tópico existe pero `ros2 topic echo` no muestra nada | Todavía no hay sincronización de hora con el agente (se reintenta cada 500 ms). Revisar el log del agente con `-v6`. |
| `ros2 topic list` no muestra el tópico | La terminal no tiene `source`, o hay un `ROS_DOMAIN_ID` distinto entre terminales. |
| `ros2 topic hz` muy por debajo de 50 Hz | El agente corre con `-v6` (log pesado) o hay errores I2C. |
| RViz2: *No transform from imu_link to odom* | `imu_filter_madgwick` no está corriendo, o no recibe `/imu/data_raw`. |
| `angular_velocity` con offset en reposo y `angular_velocity_covariance[0] = 0.0076` | La calibración falló en los 5 intentos (la placa se movió o no estaba apoyada). Pulsar RST y dejarla quieta. |

## Resultados

| Fecha | Transporte | `ros2 topic hz` (promedio / min / max) | Bias gyro en reposo | Observaciones |
|-------|------------|----------------------------------------|---------------------|---------------|
| 2026-09-23 | serial (USB CDC) | 50,00 Hz / 0,000 s / 0,041 s (σ 3,3 ms, ventana 608) | < 0,0025 rad/s por eje (una muestra en reposo; calibración OK, covarianza 1e-6) | `ros2 topic hz` mide la hora de llegada a la PC. El promedio exacto indica que no se perdieron mensajes; el máximo de 41 ms seguido de un 0 indica que a veces llegan dos mensajes juntos (jitter del USB o del agente, no del stamp). Placa casi plana: acel. (-0,57, 0,24, 9,69) m/s². |
| 2026-09-24 | serial (USB CDC), PC de escritorio | 50,00 Hz / 0,000 s / 0,040 s (σ 2,2 ms, ventana 506) | < 0,0011 rad/s por eje (una muestra en reposo) | Segunda PC: agente compilado de nuevo en `~/microros_ws` y usuario agregado a `dialout`. Después de flashear hay que pulsar RST, porque el reset automático no arranca el firmware. RViz2 + Madgwick: orientación OK. El IMU es un MPU-6500 (ver `01`), así que no hay `/imu/mag` y el yaw deriva. Acel. (-0,86, 0,07, 9,73) m/s². |
