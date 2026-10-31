# pruebas-esp32

## Proyecto

Pruebas de sensores y motores **de a uno**, cada uno en su carpeta y su proyecto PlatformIO, para el robot recolector
de frutas. El **firmware del robot** (lo que ya no es una prueba de un componente suelto) vive en
`~/Documentos/recolector_de_frutas` (`firmware/brazo`, `firmware/lib`, …); `09-brazo-control` se movió allá el 2026-09-27.

- **Placas:** Freenove ESP32-S3-WROOM **N16R8** (16 MB flash QIO, 8 MB PSRAM octal, `/dev/ttyACM0`) y una
  ESP32-WROOM-32D (DevKit con CH340, `/dev/ttyUSB0`, `board = esp32dev`) para `08`, `10` y `12`–`14`.
- **Stack:** PlatformIO (`~/.platformio/penv/bin/pio`) + Arduino core 2.0.17; ROS 2 Jazzy + micro-ROS en `02`/`05`;
  scripts de PC en Python (`tools/`). Host Ubuntu 24.04. Las dos placas también están en la Pi 5 (flasheo con esptool).

## Reglas

- **El estado va en `notas/ESTADO.md`, nunca en `CLAUDE.md`. Este archivo solo cambia si cambia una regla.**
- Al cerrar una sesión: `/cerrar`; al empezar: `/retomar`. **En este repo** bitácora, estado, decisiones y pendientes
  van en `notas/` (`BITACORA.md`, `ESTADO.md`, `DECISIONES.md`, `PENDIENTES.md`), **no en `docs/`**: `docs/` es un
  symlink a `../recolector_de_frutas/docs` (en `.gitignore`), solo para leer.
- Carpeta por prueba: `NN-<tipo>-<modelo>` en minúsculas, con la estructura estándar de `notas/FLUJO.md` (referencia
  `01-imu-mpu9250`). Agregarla a `folders` en `pruebas-esp32.code-workspace` y a la tabla de abajo.
- Drivers en `lib/<Sensor>/` sin `Serial` ni pines fijos, con `library.json`. **Reutilizar, no copiar:**
  `lib_deps = symlink://../<carpeta>/lib/<Driver>`; si cambia un driver, recompilar todos los que lo usan.
- GPIO solo en `include/pins.h`. Resultados (CSV, PNG) en `resultados/`, versionados, con fecha y hora en el nombre.
- Compilar y flashear desde la carpeta de la prueba: `$PIO run`, `$PIO run -t upload`, `$PIO device monitor`
  (115200). Cerrar el monitor (y los scripts de `tools/`) antes de flashear: el puerto no se abre dos veces.
- **Motores:** apagar la fuente de 12 V antes de flashear o pulsar RST/EN (sin pull-down en EN/ENA el puente flota).
  IBT-2 con VCC a 5 V en `12`–`14` y en el robot; si un módulo no responde a duty bajo, probar su VCC a 3V3.
- **S3:** GPIO de 3,3 V (no toleran 5 V); 19/20 son el USB nativo; **nunca usar 26–37** (flash y PSRAM octal).
  Pines ocupados en `notas/PINES_S3.md`. ModemManager y modo bootloader: ver `notas/FLUJO.md`.
- **micro-ROS** (`-e serial`): el firmware no imprime por `Serial`, detener el agente antes de flashear, pulsar RST
  después, no correr dos `pio` a la vez sobre el mismo proyecto; `include/wifi_config.h` (no versionado) por PC.
- **WROOM de la Pi:** tiene el firmware micro-ROS del brazo. Antes de flashear otra cosa, apagar los 12 V y respaldar
  la flash (`esptool --baud 230400 read-flash 0 0x400000 ...`); después restaurarla con `write-flash 0x0`.
- **Roles:** el usuario dice el modo; ante la duda, normal. **Normal:** Claude Code escribe, compila, flashea y hace
  los commits; Codex solo revisa (`codex exec --sandbox read-only`), no toca archivos ni la placa.
  **Respaldo** (límite semanal de Claude Code agotado y el usuario lo pide): Codex edita, compila, flashea y commitea
  con estas mismas reglas, hasta que el límite se reinicia.
- **Continuidad:** el contexto no pasa entre agentes: vive en git y en `notas/`. Al empezar: `git log --oneline -5` y
  `git status`. No revertir ni pisar cambios recientes sin mirar el historial. Al terminar: commit y `notas/ESTADO.md`
  al día, con lo sin terminar anotado. Al volver al modo normal, Claude Code revisa lo que hizo Codex.

## Qué leer para cada cosa

| Para… | Leer |
|---|---|
| Saber en qué quedó cada prueba | `notas/ESTADO.md` |
| Historia, mediciones y nombres de los CSV de cada prueba | `notas/BITACORA.md` |
| Qué sigue | `notas/PENDIENTES.md` |
| Por qué se eligió algo | `notas/DECISIONES.md` |
| Estructura de carpetas, `platformio.ini` de la S3, ModemManager, bootloader, micro-ROS | `notas/FLUJO.md` |
| Pines del ESP32-S3 | `notas/PINES_S3.md` |
| Control del brazo (J1–J3), cascada, scripts de `tools/` | `10-control-lazo-cerrado/CLAUDE.md`, `10-control-lazo-cerrado/README.md` |
| Plan y cableado de las pruebas definitivas | `12-j1-base/README.md`, `13-j2-hombro/README.md`, `14-j3-codo/README.md` |
| Arquitectura y conexionado del robot (solo leer) | `docs/ARQUITECTURA.md`, `docs/html/placa_wroom.html` |
| Cómo se prueba un sensor (ejemplo) | `01-imu-mpu9250/README.md` |

## Estructura

| Carpeta          | Prueba                          |
|------------------|---------------------------------|
| `00-hola-mundo`  | Serial "hola mundo" (verificación de toolchain y flasheo) |
| `01-imu-mpu9250` | IMU MPU-9250 / MPU-6500 por I2C |
| `02-imu-ros2`    | IMU publicado en ROS 2 Jazzy con micro-ROS (`/imu/data_raw`, 50 Hz) |
| `03-hcsr04`      | Sensor ultrasónico HC-SR04 (pendiente, todavía sin carpeta) |
| `04-gps-up501`   | GPS Fastrax UP501 por UART (NMEA) |
| `05-gps-ros2`    | GPS publicado en ROS 2 Jazzy con micro-ROS (`/gps/fix`, `/gps/status`, 1 Hz) |
| `06-motor-36gp555` | Motor DC 36GP-555 (12 V, 160 rpm, encoder Hall) con driver IBT-2 (BTS7960), lazo abierto |
| `07-motor-rampa-vueltas` | Mismo motor: rampa de PWM 0 → 100 % que termina a N vueltas de salida, para comparar con y sin carga |
| `08-motor-5840-l298n` | Motor 5840-31ZY (sin fin, 12 V, 160 rpm) con L298N y encoder externo de 1000 PPR, en una **ESP32-WROOM-32D**, lazo abierto |
| `10-control-lazo-cerrado` | Lazo cerrado (cascada posición → velocidad) de J1 (36GP-555 + IBT-2) y J2 (5840-31ZY + L298N) del brazo, en la **ESP32-WROOM-32D** con los pines del robot |
| `11-led-rgb-s3` | LED RGB de la S3: ¿GPIO 48 o 38? |
| `12-j1-base` | Prueba definitiva de J1 (base, 36GP-555 + IBT-2) montada, con el conexionado v0.5 del robot (EN común, las tres conectadas), en la WROOM |
| `13-j2-hombro` | Prueba definitiva de J2 (hombro, 36-555 58 rpm + engranajes 15 → 75 + IBT-2), igual que 12 |
| `14-j3-codo` | Prueba definitiva de J3 (codo, 36-555 58 rpm + engranajes 15 → 75 + IBT-2), igual que 12 |
| `15-ibt2-j3-c3` | IBT-2 de J3 (driver 3) con el motor montado, en una **ESP32-C3-MINI-1**: pulsos cortos limitados a ±15° |
| `16-l298n-j3-c3` | Lo mismo que 15 con un **L298N** en lugar del IBT-2 (mismo C3, pines y comandos), para descartar el driver |
| `17-brazo-joystick` | J1, J2 y J3 a la vez con un joystick en el navegador (lazo abierto; `tools/joystick.py` hace de puente serie), en la WROOM |
| `18-finales-de-carrera` | Los tres finales de carrera del brazo (J1 · J2 · J3 en GPIO 32 · 27 · 33): suelto / apretado y rebotes, sin mover motores, en la WROOM |
| `19-j2-j3-conjunto` | Lazo cerrado de J2 y J3 juntas (cuatro barras): una activa con los comandos de 12–14 y la otra sostenida sola en segundo plano, en la WROOM |
| `notas/` | Estado, decisiones, pendientes, pines y flujo de trabajo (no es una prueba) |

`12`–`14`: `tools/` es un enlace a `10-control-lazo-cerrado/tools`.
