# 05-gps-ros2 — GPS Fastrax UP501 en ROS 2 Jazzy con micro-ROS

El ESP32-S3 lee el GPS por UART1 y publica **una vez por segundo**:

| Tópico | Tipo | Para qué |
|--------|------|----------|
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | Mensaje estándar de GPS: lo usan `robot_localization`, mapviz, Foxglove, etc. |
| `/gps/status` | `std_msgs/msg/String` | Diagnóstico legible para una persona |

El nodo es `gps_node` y el `frame_id` es `gps_link`. La arquitectura (agente, transporte serial o UDP y sincronización de hora) es la misma que en `02-imu-ros2`.

## Conexiones

Mismo cableado que `04-gps-up501`:

| Pin del UP501 | Breakout Freenove v1.2 |
|---------------|------------------------|
| 1 RXD         | **GPIO42** (TX1) |
| 2 TXD         | **GPIO41** (RX1) |
| 3 GND         | **GND** |
| 4 VDD         | **3V3** |
| 5 VDD_B       | **3V3** |
| 6 PPS         | sin conectar |

El IMU (GPIO8/9) puede quedar conectado: no comparte pines con el GPS.

## Firmware

| Parámetro | Valor |
|-----------|-------|
| Nodo | `gps_node` |
| Frecuencia | `/gps/fix`: **uno por epoch** del receptor (1 Hz), apenas llegan la GGA y la RMC de la misma hora UTC (cada mitad vence a los 1,5 s). No se repiten epochs; si el loop se demora, hasta 4 esperan en cola. `/gps/status`: timer de 1 s |
| QoS | reliable en los dos tópicos, compatible con suscriptores reliable y best effort |
| `/gps/fix` → `status.service` | `SERVICE_GPS` (1) |
| `/gps/fix` → `status.status` | `STATUS_NO_FIX` (-1) sin fix, `STATUS_FIX` (0) con fix y `STATUS_SBAS_FIX` (1) si GGA informa calidad 2 |
| `/gps/fix` sin fix | `latitude`, `longitude` y `altitude` en `NaN`; covarianza `UNKNOWN` |
| `altitude` | Sobre el **elipsoide WGS 84**, como pide `NavSatFix`: altura MSL de GGA + separación del geoide. Si el receptor no envía la separación, queda la altura MSL |
| `position_covariance` | Diagonal, `APPROXIMATED`: σ horizontal = HDOP × 4 m y σ vertical = el doble. Si un epoch con fix no trae HDOP, se usa HDOP = 10 (σ = 40 m) en vez de 0, porque `robot_localization` usa la matriz tal cual |
| `header.stamp` | **Hora de la medición**: la hora UTC del epoch (fecha y hora de RMC) si difiere menos de 2 s de la hora de llegada en el reloj del agente; si no (sin fix, fecha errónea del receptor o reloj de la PC desfasado), la hora de llegada. Un stamp que no avanza se descarta; si retrocede más de 1 s (resincronización del reloj) se acepta y se sigue desde ahí. `/gps/fix` **no se publica hasta sincronizar la hora con el agente**. `/gps/status` se publica siempre |
| Driver | `04-gps-up501/lib/UP501`, enlazado con `symlink://` (no se copia) |

Textos de `/gps/status` (van sin tildes para que `ros2 topic echo` no los muestre con escapes):

```
ERROR: sin datos del GPS | Revisar TX/RX y alimentacion
ERROR: tramas NMEA invalidas | Revisar baudrate (9600)
SIN FIX | Satelites visibles: 0 | Buscando posicion...
GPS OK | Satelites: 8 | Lat: -25.324521 | Lon: -57.512346 | Alt: 112.4 m | HDOP: 1.20
```

En `/gps/status`, la altura es la MSL (sobre el nivel del mar), la que se compara con un mapa. Con fix, el texto termina en `| Stamp: UTC GPS (latencia N ms)` (cuánto tardaron las tramas en llegar después de la medición) o en `| Stamp: llegada`. Mientras no hay hora del agente, se le agrega `| /gps/fix espera la hora del agente`.

**Diagnóstico en `/gps/status`:** al final del texto van los contadores desde el arranque: `epochs` completos del driver, `fix pub` y `err pub` (publicaciones de `/gps/fix` confirmadas y fallidas), `err NMEA` (checksum malo), `err UART` (desbordes o errores de la UART), `cola` (epochs descartados por cola llena), `stamp desc` (descartados por stamp que no avanza), `saltos reloj` y `loop max` (vuelta de `loop()` más larga en el último segundo). Si `epochs` deja de subir y los errores siguen en 0, el GPS dejó de transmitir: cableado o alimentación.

**Validez del fix:** un epoch cuenta como fix solo si la GGA tiene calidad > 0 y la RMC estado `A`. Si el receptor informa que perdió el fix, el siguiente `/gps/fix` ya sale con `STATUS_NO_FIX`: nunca se publica una posición vieja con stamp nuevo.

**Reconexión:** un ping cada 500 ms (timeout de 100 ms) y, después de 3 sin respuesta, destruye las entidades y vuelve a esperar al agente. Con WiFi, el firmware desactiva el ahorro de energía (`WiFi.setSleep(false)`): con él activo, los picos de latencia hacían fallar los pings y la sesión se reconectaba 2 a 5 veces cada 2 minutos, perdiendo epochs. El GPS se sigue leyendo en todo momento. Durante las esperas de la comunicación, los bytes se acumulan en un búfer de 1 kB de la UART (~1 s a 9600 baud), así que no se pierden tramas.

Con transporte serial, el USB es del agente: el firmware no imprime nada por `Serial`. Para diagnosticar el GPS, flashear `04-gps-up501`.

## Compilar y flashear

```bash
cd ~/pruebas-esp32/05-gps-ros2
~/.platformio/penv/bin/pio run -e serial -t upload
```

- La primera compilación genera micro-ROS Jazzy para este proyecto (~5 min). Antes de compilar, revisar que no haya otro `pio` corriendo.
- **Detener el micro-ROS Agent antes de flashear**, porque tiene abierto `/dev/ttyACM0`.
- **Después de flashear, pulsar RST.**

## Correr el agente y verificar

El agente es el mismo de `02-imu-ros2` (compilado en `~/microros_ws`).

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

En otra terminal:

```bash
ros2 node list                    # /gps_node
ros2 topic list                   # /gps/fix y /gps/status
ros2 topic echo /gps/status       # diagnóstico legible
ros2 topic echo /gps/fix          # NavSatFix
ros2 topic hz /gps/fix            # ~1 Hz
```

Qué revisar:
- En interiores: `/gps/status` muestra `SIN FIX` y `/gps/fix` trae `status: -1` y `latitude: .nan`.
- Al aire libre: `/gps/status` pasa a `GPS OK` y `/gps/fix` trae `status: 0` (o 1 con SBAS), coordenadas reales y `position_covariance_type: 1`.
- `header.stamp` coincide con la hora de la PC (comparar con `date +%s`).

Para ver la posición en un mapa: Foxglove Studio (panel Map, con `/gps/fix`) o `mapviz` (`sudo apt install ros-jazzy-mapviz ros-jazzy-mapviz-plugins`).

## Cambiar a WiFi UDP

Igual que en `02-imu-ros2`. `include/wifi_config.h` no se versiona: se crea en cada PC a partir de `wifi_config.example.h`.

```bash
cp include/wifi_config.example.h include/wifi_config.h   # completar SSID, clave e IP de la PC
~/.platformio/penv/bin/pio run -e wifi -t upload
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

Con WiFi la placa puede ir alimentada con una batería, sin USB. Así es más fácil sacarla al exterior para conseguir fix.

## Solución de problemas

| Síntoma | Causa probable |
|---------|----------------|
| `/gps/status`: `ERROR: sin datos del GPS` | Cableado de la UART o alimentación. Probar con `04-gps-up501`. |
| Existe `/gps/status`, pero `/gps/fix` no publica nada | Todavía no hay hora del agente: el texto de estado termina en `/gps/fix espera la hora del agente`. |
| Nunca pasa de `SIN FIX` | Interiores o cielo tapado. Llevar la placa afuera (conviene usar WiFi y una batería). |
| El agente no muestra `session established` | Falta pulsar RST después de flashear, o el agente no corre en el puerto correcto. |

## Resultados

| Fecha | Transporte | `ros2 topic hz /gps/fix` | `/gps/status` | Observaciones |
|-------|------------|--------------------------|---------------|---------------|
| 2026-09-25 | WiFi UDP (placa por el conector CH343 solo para flashear y alimentar; PC por WiFi, misma red) | 0,996 Hz / 0,926 s / 1,061 s (ventana 10) | `SIN FIX \| Satelites visibles: 1` | Interiores. Sesión con el agente sin pulsar RST: el reset del CH343 después de flashear alcanzó. `/gps/fix`: `status -1`, `service 1`, coordenadas `.nan`, stamp sincronizado con la PC. Falta el fix al aire libre. |
| 2026-09-25 | WiFi UDP, placa con batería al aire libre | ~1 Hz | `SIN FIX` → **`GPS OK`** | Primer fix en frío ~2 min (02:39:35 → 02:41:36): 3 satélites, HDOP 2,42. A los 3 min: 4–5 satélites, HDOP 1,4–1,6, posición estable en ~7 m, **verificada en el mapa**. `/gps/fix`: `status 0`, `altitude` = MSL + 15 m de geoide, covarianza `APPROXIMATED` (σ ≈ 9,6 m con HDOP 2,4). Hubo cortes de `sin datos del GPS` por falso contacto en la protoboard al mover la placa. |
| 2026-09-25 | WiFi UDP, interiores, GPS conectado directo (sin protoboard) | 125 epochs / 125 publicados / 120 de 120 recibidos en 120 s, sin huecos | `SIN FIX` | Firmware con un `/gps/fix` por epoch. Antes de desactivar el ahorro de energía del WiFi: 2–5 reconexiones cada 2 min y huecos de 2–5 s. Con la protoboard, el GPS se cortaba (epochs detenidos, luego `tramas invalidas`). Contadores de error en 0 y `loop max` ≤ 119 ms. Falta repetir al aire libre para ver el stamp UTC GPS y la latencia. |
