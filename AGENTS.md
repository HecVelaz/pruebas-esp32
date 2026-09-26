# AGENTS.md — pruebas-esp32

## Contexto

- **Placa:** Freenove ESP32-S3-WROOM **N16R8** (16 MB flash QIO, 8 MB PSRAM octal) sobre breakout Freenove v1.2
- **Segunda placa (solo `08-motor-5840-l298n`):** ESP32-WROOM-32D (ESP32 clásico, 4 MB) en una DevKit con USB-serie **CH340** (`1a86:7523`, `/dev/ttyUSB0`). Botones **BOOT** y **EN** (reset). GPIO a evitar: 6–11 (flash), 0/2/5/12/15 (strapping), 1/3 (UART0 = USB), 34–39 (solo entrada, sin pull-up).
- **Host:** Ubuntu 24.04
- **Puerto:** `/dev/ttyACM0` (USB nativo del S3, `303a:1001`; el usuario está en el grupo `dialout`)
- **Toolchain:** PlatformIO en `~/.platformio/penv/bin/pio` (framework Arduino)

## Objetivo

Probar sensores de a uno, cada uno en su propia carpeta y proyecto PlatformIO independiente:

| Carpeta          | Prueba                          |
|------------------|---------------------------------|
| `00-hola-mundo`  | Serial "hola mundo" (verificación de toolchain y flasheo) |
| `01-imu-mpu9250` | IMU MPU-9250 / MPU-6500 por I2C |
| `02-imu-ros2`    | IMU publicado en ROS 2 Jazzy con micro-ROS (`/imu/data_raw`, 50 Hz) |
| `03-hcsr04`      | Sensor ultrasónico HC-SR04 (pendiente) |
| `04-gps-up501`   | GPS Fastrax UP501 por UART (NMEA) |
| `05-gps-ros2`    | GPS publicado en ROS 2 Jazzy con micro-ROS (`/gps/fix`, `/gps/status`, 1 Hz) |
| `06-motor-36gp555` | Motor DC 36GP-555 (12 V, 160 rpm, encoder Hall) con driver IBT-2 (BTS7960), lazo abierto |
| `07-motor-rampa-vueltas` | Mismo motor: rampa de PWM 0 → 100 % que termina a N vueltas de salida, para comparar con y sin carga |
| `08-motor-5840-l298n` | Motor 5840-31ZY (sin fin, 12 V, 160 rpm) con L298N y encoder externo de 1000 PPR, en una **ESP32-WROOM-32D**, lazo abierto |

Nombre de carpeta: `NN-<tipo>-<modelo>` en minúsculas (por ejemplo `03-hcsr04`, `04-gps-neo6m`).

### Estructura estándar de cada sensor

Tomar `01-imu-mpu9250` como referencia:

```
pruebas-esp32/
├── AGENTS.md
├── CLAUDE.md
├── pruebas-esp32.code-workspace
├── 00-hola-mundo/
└── 01-imu-mpu9250/
    ├── platformio.ini      ← placa, puerto, librerías
    ├── README.md           ← conexiones, pines, resultados
    ├── src/
    │   └── main.cpp        ← programa de prueba
    ├── lib/
    │   └── MPU9250/        ← código del sensor, reutilizable
    │       ├── MPU9250.h
    │       └── MPU9250.cpp
    ├── include/
    │   └── pins.h          ← pines en un solo lugar
    ├── tools/              ← scripts de PC en Python (opcional; por ejemplo graficar.py en los motores)
    ├── resultados/         ← CSV y PNG de las pruebas (versionados, nombre con fecha y hora)
    └── test/               ← pruebas unitarias (opcional)
```

- **`lib/<Sensor>/`**: el driver del sensor, sin `Serial` ni pines fijos (recibe el bus o los pines por parámetro). Incluye un `library.json` para que otros proyectos lo puedan enlazar.
- **Reutilizar drivers, no copiarlos:** si el driver ya existe en otra carpeta, enlazarlo en `platformio.ini` con `lib_deps = symlink://../01-imu-mpu9250/lib/MPU9250` (así lo hace `02-imu-ros2`). Un cambio en el driver afecta a todos los proyectos que lo usan: recompilarlos.
- **`include/pins.h`**: único lugar donde se definen los GPIO. `main.cpp` y el README los toman de ahí.
- **`src/main.cpp`**: primero detecta el sensor (escáner I2C, ID de chip o primera trama válida) e informa por serial si no responde; después imprime lecturas con unidades.
- **`README.md`**: tabla de conexiones, comando para probar, salida esperada, cómo verificar que los valores son razonables y una tabla de **Resultados** para completar después de probar.
- Al crear la carpeta, agregarla a `folders` en `pruebas-esp32.code-workspace`.

La configuración de placa de `platformio.ini` es la misma en todas las pruebas con la ESP32-S3 (excepción: `08-motor-5840-l298n` usa una ESP32-WROOM-32D, `board = esp32dev`, puerto `/dev/ttyUSB0` del CH340):

```ini
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_upload.flash_size = 16MB
board_build.partitions = default_16MB.csv
board_build.arduino.memory_type = qio_opi
build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_CDC_ON_BOOT=1   ; Serial sale por /dev/ttyACM0
```

## Comandos

Ejecutar dentro de la carpeta de la prueba (por ejemplo `cd 00-hola-mundo`):

```bash
PIO=~/.platformio/penv/bin/pio

$PIO run                            # compilar
$PIO run -t upload                  # compilar y flashear
$PIO device monitor                 # monitor serial (115200); salir con Ctrl+C
$PIO run -t upload -t monitor       # flashear y abrir el monitor
```

- Cerrar el monitor antes de flashear: si otro proceso tiene abierto `/dev/ttyACM0`, el upload falla.
- **ModemManager** (activo por defecto en Ubuntu) sondea cada `/dev/ttyACM*` nuevo y, al mover DTR/RTS, deja la S3 en modo descarga (`rst:0x15 (USB_UART_CHIP_RESET), boot:0x0 ... waiting for download`): el firmware no arranca, de forma intermitente. En cada PC, una vez:
  ```bash
  echo 'ATTRS{idVendor}=="303a", ENV{ID_MM_DEVICE_IGNORE}="1"' | sudo tee /etc/udev/rules.d/99-esp32-mm-ignore.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
- Si el upload no conecta, entrar en modo bootloader: mantener **BOOT**, pulsar **RST** y soltar **BOOT**. Después de flashear, pulsar **RST** otra vez. (En la DevKit de la ESP32-WROOM-32D el reset se llama **EN**; el puerto es `/dev/ttyUSB0`.)
- **Motores:** el firmware no fija el driver hasta arrancar, y sin pull-down en los EN/ENA el puente flota durante el flasheo y el reset. **Apagar la fuente del motor (12 V) antes de flashear o pulsar RST/EN.** Abrir el puerto con pyserial o el monitor no reinicia ninguna de las dos placas.
- **Scripts de los motores** (`tools/graficar.py`, `tools/rampa.py`): cerrar antes el monitor de PlatformIO (el puerto no se puede abrir dos veces). Con `--archivo` vuelven a graficar un CSV guardado sin tocar la placa.

### Proyectos micro-ROS (ROS 2 Jazzy)

- ROS 2 Jazzy está instalado en `/opt/ros/jazzy`. El micro-ROS Agent se compila en `~/microros_ws` (ver `02-imu-ros2/README.md`).
- Con `board_microros_transport = serial`, el USB lo usa el agente: el firmware **no debe imprimir por `Serial`**, y hay que **detener el agente antes de flashear**.
- La primera compilación genera `libmicroros.a` (~5 min). Si en paralelo corre otro `pio` sobre el mismo proyecto (por ejemplo, la reindexación de la extensión de VS Code), se pisan y falla con errores como `file INSTALL cannot find librmw.a`. Por eso el workspace tiene `platformio-ide.autoRebuildAutocompleteIndex: false`. Antes de compilar, revisar que no haya otro `pio` corriendo.
- Tienen varios entornos (`-e serial`, `-e wifi`). `default_envs = serial`. Los datos de WiFi van en `include/wifi_config.h`, que está en `.gitignore`: **hay que crearlo en cada PC** (a partir de `wifi_config.example.h`) con la IP de esa PC y re-flashear con `-e wifi`.
- **Después de flashear, pulsar RST** en la placa. El reset automático de esptool arranca el firmware, pero la sesión con el agente queda sin crear el nodo (visto por serial y por WiFi, en las dos PCs).

## Notas de hardware

- Los GPIO del ESP32-S3 son de **3,3 V** y no toleran 5 V. El pin ECHO del HC-SR04 (5 V) necesita un divisor de tensión o un conversor de nivel.
- GPIO19/20 son el USB nativo y no se usan para sensores. GPIO26–37 están reservados para la flash y la PSRAM octal en el N16R8 (**nunca usar 35, 36 ni 37**, aunque aparezcan en el pinout de la DevKit).
- I2C por defecto: SDA = GPIO8, SCL = GPIO9.

## Roles

Hay dos modos. El usuario indica cuál está vigente; ante la duda, es el **normal**.

- **Modo normal** (Claude Code tiene límite disponible):
  - **Claude Code:** escribe el código, compila, flashea la placa y hace los commits.
  - **Codex:** actúa **solo como revisor** (`codex exec --sandbox read-only`). No modifica, crea ni borra archivos, y no ejecuta uploads a la placa. Únicamente sugiere cambios (en el chat o como diff propuesto) para que Claude Code o el usuario los apliquen.
- **Modo respaldo** (el límite semanal de Claude Code se agotó y el usuario pide seguir con Codex):
  - **Codex:** puede editar archivos, compilar, flashear y hacer commits, siguiendo las mismas convenciones de este archivo y de `CLAUDE.md`.
  - Cuando el límite de Claude Code se reinicia, se vuelve al modo normal.

## Continuidad entre agentes

El contexto no pasa de un agente a otro: el estado vive en **git** y en **`CLAUDE.md`**.

- **Al empezar** (cualquier agente): correr `git log --oneline -5` y `git status` para ver en qué quedó el trabajo. No asumir que se conoce lo que hizo el otro agente.
- **No revertir ni pisar cambios recientes** sin revisar antes el historial (`git log`, `git show`).
- **Al terminar una sesión:** hacer commit de lo terminado y actualizar "Estado actual" en `CLAUDE.md` con lo hecho, lo probado en placa y lo pendiente. Lo que quede sin terminar, anotarlo explícitamente.
- **Al volver al modo normal:** Claude Code revisa lo que hizo Codex en modo respaldo (`git log`, `git diff`) antes de continuar.
