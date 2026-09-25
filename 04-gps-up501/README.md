# 04-gps-up501 — GPS Fastrax UP501 por UART

Lee las tramas NMEA del receptor por **UART1** y, una vez por segundo, imprime por serial un resumen del estado:
- **ERROR sin datos**: no llega nada por la UART.
- **ERROR tramas inválidas**: llegan bytes, pero ningún checksum es correcto.
- **SIN FIX**: llegan tramas NMEA válidas, pero todavía no hay posición.
- **FIX 2D/3D**: posición con satélites, latitud, longitud, altura, HDOP y hora UTC.

También puede mostrar las tramas NMEA crudas, como la prueba anterior con Arduino (`gps.md`). Para ROS 2 ver `05-gps-ros2`.

## Módulo

| Dato | Valor |
|------|-------|
| Modelo | Fastrax **UP501** (marcado UP501A03), chip MediaTek, antena cerámica integrada |
| Interfaz | UART, NMEA 0183 con talker `GP` (solo GPS) |
| Puerto serie | **9600 baud, 8N1** (valor de fábrica) |
| Tramas | `GGA`, `RMC`, `GSA`, `GSV`, una vez por segundo |
| Alimentación | **3,3 V** en VDD y en VDD_B. **No usar 5 V.** |

Pines del módulo, con el texto FASTRAX / UP501 hacia arriba y los 6 pads abajo, de izquierda a derecha: `6 PPS · 5 VDD_B · 4 VDD · 3 GND · 2 TXD · 1 RXD`.

## Conexiones

**Placa:** Freenove ESP32-S3-WROOM (N16R8) sobre el breakout Freenove v1.2.

| Pin del UP501 | Breakout Freenove v1.2 | Nota |
|---------------|------------------------|------|
| 1 RXD         | **GPIO42** (TX1)       | `PIN_GPS_TX` en `include/pins.h` |
| 2 TXD         | **GPIO41** (RX1)       | `PIN_GPS_RX` en `include/pins.h` |
| 3 GND         | **GND**                | |
| 4 VDD         | **3V3**                | |
| 5 VDD_B       | **3V3**                | Respaldo del reloj y las efemérides |
| 6 PPS         | sin conectar           | Si se usa más adelante: GPIO47 |

- TX y RX van **cruzados**: lo que sale de cada placa entra en la otra.
- Los niveles lógicos del UP501 son de 3,3 V: se conecta directo, sin divisor.
- Con VDD_B en el mismo 3V3, el respaldo se pierde al desconectar la placa y cada arranque es en frío. Para arranques en caliente, VDD_B necesita alimentación propia (por ejemplo, una pila).
- Pines elegidos para no chocar con el resto (ver la tabla del `CLAUDE.md` de la raíz):
  - GPIO8/9 son del IMU.
  - GPIO14/21 quedan reservados para el lidar (UART2).
  - GPIO43/44 (UART0) van al chip USB-serie CH343 de la placa.
  - También se evitaron los pines de strapping (0, 3, 45 y 46), el USB (19/20), la flash y la PSRAM (26–37), la cámara, la SD (38–40) y los LED (2 y 48).
  - GPIO41/42 son también pines de JTAG, pero la placa usa el JTAG por USB, así que están libres.

## Cómo probar

```bash
cd ~/pruebas-esp32/04-gps-up501
~/.platformio/penv/bin/pio run -t upload -t monitor
```

Cerrar cualquier otro monitor serial y detener el micro-ROS Agent antes de flashear. Después de flashear, pulsar RST.

**Alternativa por el conector del CH343 (entorno `uart`):** si el USB nativo deja la placa en modo descarga (`boot:0x0 ... waiting for download`), conectar el cable al otro USB-C de la placa (chip CH343, `1a86:55d3`) y usar:

```bash
~/.platformio/penv/bin/pio run -e uart -t upload -t monitor
```

`Serial` sale por UART0 (GPIO43/44) y el reset lo hace el CH343 con EN/GPIO0, sin pasar por el USB nativo. Así se probó el 2026-09-25.

Escribir `r` + Enter en el monitor alterna entre el resumen y las tramas NMEA crudas.

Salida esperada en interiores, sin satélites (lo que se vio con Arduino):

```
== Prueba GPS Fastrax UP501 ==
UART1: RX=GPIO41 (<- TXD del GPS), TX=GPIO42 (-> RXD del GPS), 9600 baud 8N1
Escribir 'r' + Enter en el monitor para alternar entre el resumen y las tramas NMEA crudas.

[    1 s] SIN FIX | satélites en vista: 0 | NMEA ok 4, checksum malo 0 | buscando posición...
[    2 s] SIN FIX | satélites en vista: 0 | NMEA ok 8, checksum malo 0 | buscando posición...
```

Con fix (al aire libre, antena mirando al cielo):

```
Primer FIX a los 42 s del arranque.
[   43 s] FIX 3D | sats usados 8, en vista 11 | lat -25.324521 lon -57.512346 | alt 112.4 m (MSL), geoide 12.3 m | HDOP 1.20 | 2026-09-24 15:32:10 UTC
```

En modo crudo aparecen las tramas tal como llegan:

```
$GPGGA,235950.036,,,,,0,0,,,M,,M,,*45
$GPGSA,A,1,,,,,,,,,,,,,,,*1E
$GPGSV,1,1,00*79
$GPRMC,235950.036,V,,,,,0.00,0.00,050180,,,N*4C
```

Sin fix, la hora y la fecha de las tramas vienen del reloj interno (por ejemplo `050180`, año 1980) y no son reales: el resumen solo muestra la hora cuando hay fix.

### Verificaciones

- **Comunicación:** `NMEA ok` sube unas 4 tramas por segundo y `checksum malo` queda en 0 o casi 0.
- **Satélites:** en interiores lo normal es 0. Al aire libre, sin techos ni árboles encima, en unos minutos debería ver 6 o más.
- **Primer fix:** en frío suele tardar de medio minuto a unos minutos. Anotar el tiempo en **Resultados**.
- **Posición:** pegar `lat, lon` en un mapa (por ejemplo `https://www.openstreetmap.org/?mlat=LAT&mlon=LON&zoom=18`) y comprobar que cae en el lugar de la prueba.
- **HDOP:** menor a 2 es bueno; mayor a 5, la posición es poco confiable.
- **Quieto:** con la antena quieta, la posición varía unos pocos metros. Es normal.

## Driver (`lib/UP501`)

- Envoltorio de [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) (1.1.0) que recibe un `Stream` ya abierto, sin pines ni `Serial`.
- Arma **una muestra por epoch** cuando llegan la GGA y la RMC con la misma hora UTC: todos los campos (posición, altura, HDOP, satélites, hora) son del mismo instante. `pollEpoch()` la entrega una sola vez; `utcUnixMs()` da su hora UTC.
- Un epoch tiene fix solo si la GGA trae calidad > 0 y la RMC estado `A`. Si el receptor informa que perdió el fix, el estado pasa a "sin fix" en ese mismo epoch. TinyGPSPlus, en cambio, mantiene como válida la última posición.
- Estado de la comunicación: sin datos o tramas inválidas si pasan 3 s sin bytes o sin tramas con checksum correcto; fix si el último epoch con fix tiene menos de 2,5 s.
- Lee campos que TinyGPSPlus no expone: satélites en vista (`GSV`), calidad del fix y separación del geoide (`GGA`), y modo 2D/3D (`GSA`).
- Lo reutiliza `05-gps-ros2` con `symlink://`: un cambio en el driver afecta a los dos proyectos.

## Solución de problemas

| Síntoma | Causa probable |
|---------|----------------|
| `ERROR: sin datos del GPS` | TX y RX sin cruzar, falta GND común o el módulo no tiene 3V3. |
| `ERROR: ... ninguna trama NMEA válida` | Baudrate distinto de 9600 (alguien lo cambió con un comando PMTK) o ruido en la línea. |
| TX del GPS en alto pero 0 bytes, también después de corregir el cableado | El GPS quedó trabado (pasó después de invertir TX y RX). **Cortarle la alimentación ~10 s**: resetear el ESP32 no alcanza. |
| El monitor no muestra nada y el log de la ROM dice `waiting for download` | El USB nativo dejó la S3 en modo descarga (ModemManager o la apertura del puerto). Ver `AGENTS.md` o usar el entorno `uart`. |
| Por el conector del CH343 solo aparece el log de la ROM | El firmware del entorno por defecto escribe en el USB nativo: flashear `-e uart`. |
| `SIN FIX` con 0 satélites durante mucho tiempo | Está en interiores o la antena no mira al cielo. |
| Satélites en vista pero nunca hay fix | Cielo parcialmente tapado. Esperar más, o alejarse de paredes y árboles. |

## Resultados

| Fecha | Lugar | Satélites (usados / en vista) | Primer fix | HDOP | Observaciones |
|-------|-------|-------------------------------|------------|------|---------------|
| 2026-09-25 | Interiores (escritorio) | 0 / 0 | — | — | NMEA OK en UART1 (GPIO41/42): 4 tramas/s, 0 errores de checksum, entorno `uart` por el CH343. Antes se verificó el módulo en un ESP32-WROOM-32D (136 B/s). Falta probar al aire libre. |
