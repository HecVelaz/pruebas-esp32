# Estado de pruebas-esp32 (al 2026-11-18, cierre)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Recolección autónoma (lo actual)

- **`27-aruco-arbol/tools/recolectar.py` hace todo (13/11: agarró el limón, 95 s):** homing del brazo → reposo → mira la
  marca de la maceta → va a la parada (girar con el giróscopo, avanzar con las ruedas, girar) → mide el limón 5 s →
  `agarrar`. 3 Enter: 12 V apagados (abre la WROOM), 12 V encendidos (homing), ir. En la Pi:
  `cd ~/scripts/27-aruco-arbol; python3 tools/recolectar.py` (visión y agente de la base corriendo; WROOM env `brazo`).
- **Parada** (`26`): tronco a **70 cm a la izquierda de J1 y 15 cm adelante**; **4 paradas cada 90° = 100 %** de los
  limones del arbolito medido. Validada: desde ahí `agarrar` funciona.
- **Marcas ArUco** (`27`): DICT_4X4_50, 15 cm, ID 0 pegada en la maceta (cara a 16,5 cm del tronco: `--radio-maceta
  165`); faltan pegar las IDs 1–3 (una por cara, antihorario). Se detecta de ~0,9 a ~4 m; en la parada queda cortada
  abajo (ahí mandan los limones). `ver_marcas.py --tf nav` para mirarla.
- **Dos calibraciones de la cámara:** agarre = la del brazo del 12/11 (`ajuste_camara.py`, < 70 cm); navegación =
  `27/resultados/tf_navegacion.json` (~3 cm a 1–2 m).
- **GPS** UP501 en la Pi (`/dev/ttyAMA0`, 9600, `nmea_navsat_driver -r fix:=/gps/fix`): ±20 m, solo para la rúbrica.

## Teleop afuera (25, 17/11)

- **Sin la red del lab:** la Pi entra al **hotspot del celular `POCO X6 Pro 5G` en 5 GHz** (en 2,4 GHz el celular usa
  el canal 13 y la Pi no lo ve: sin país del wifi). Afuera se conecta sola; en el lab, `sudo nmcli con up "POCO X6 Pro
  5G"`. Página `http://<IP en el hotspot>:8000` (17/11: 10.244.71.243; ver "dispositivos conectados", MAC
  `2c:cf:67:14:b2:cb`) o, con datos, Tailscale. `lanzar.sh` lo arranca todo (SSH desde la notebook o el celular).
- `lanzar.sh` pregunta "¿12 V APAGADOS?" y resetea la WROOM por EN (`n` si los 12 V están encendidos). Página:
  botón "Servos: suaves / RÁPIDOS" (rampa 45 °/s o salto directo) y "Pinza cerrar" = `pz 1400`. Funciona.

## Base (S3, `25-teleop-pi`)

- S3 en la Pi con el env `ros` de `recolector_de_frutas/firmware/base` (flasheo con `--after watchdog-reset` y el
  puerto by-id: vuelve a veces como `ttyACM1`). Agente: `bash ~/scripts/agente_base.sh`; comandos: `bash ~/scripts/base.sh "<cmd>"` (`sens`, `e`, `cal`,
  `odo0`, `armar`, `r`). Con micro-ROS el watchdog es 300 ms: mover solo con `/cmd_vel` continuo.
- **S3 con `base_ros_20261117_imu`** (18/11): **la IMU se recupera sola** de un I2C colgado (probado: la seta corta los
  5 V de la IMU → `imu_rec=1` al soltarla). Commit `cc96d1f` en `recolector_de_frutas`.
- **Seguridad con motores probada (18/11, `25-teleop-pi/README.md`):** frena solo el HC-SR04 del medio (tope lineal
  50 → 20 cm; en el piso para a 20 cm justos); bumpers, seta y vuelco (25°) OK, enclavados (`r` por `/base/comando`).
  Los tres HC-SR04 andan (derecho cambiado); los de las esquinas solo publican (decisión del usuario, por ahora).
- Batería ~11,2 V (LiPo 3S sin BMS, corte 10,5 V). Odometría: recto < 1 %; giros con el giróscopo (~±1°).
- **A vigilar:** encoder izquierdo con un `atasco` aislado el 18/11 (falso contacto); enlace USB de la S3 con cortes
  (3 en la sesión, con la seta y moviendo la base). Si la S3 no se conecta al agente tras reiniciar la Pi:
  `pkill -f micro_ros_agent`, `esptool ... --after watchdog-reset chip-id`, `agente_base.sh`.

## Brazo (`20-brazo-coordinado`, WROOM)

- **La WROOM de la Pi tiene el env `ros`** (teleop, 17/11). Para `punta.py` / `recolectar.py`: `write-flash 0x10000
  ~/firmware/brazo20_serie.bin` (env `brazo`); de vuelta: `write-flash 0x0 ~/firmware/respaldo_wroom_20261112.bin`.
- **Abrir el puerto serie reinicia la WROOM** (cero perdido): siempre `h` después (los scripts lo hacen).
- `punta.py`: `p`, `ap`, `ret`, `tarea`, `anotar`, **`agarrar` / `veragarrar`** (pinza 1300 us, 1 cm más abajo, 3° de
  margen a la franja); `punta.agarrar` y `Vision` se usan desde otros programas.
- J1, J2, J3 con `lib/Articulacion`, 100 Hz, 20 kHz; homing `h` → J1 3°, J2 6°, J3 3°; reposo `mv 3 17 4`; franja con
  tablas (`FRANJA_ARRIBA` / `FRANJA_ABAJO`), caja con J1 ≥ 53,5 (sin probar); repetibilidad RP 0,13 / 0,20 / 0,06°.
- Cinemática (`tools/cinematica.py`): ~10 mm de media; en z el brazo real se mueve ~1,26–1,43 × el modelo (recalibrar).
- Cero en los finales de carrera (J1 32, J2 27, J3 33; un solo apoyo: el golpe los gira). Límites J1 0–118°, J2 0–83°,
  J3 0–101,5°. Fuente directa a la bornera del bus (sin fusible ni seta por el adaptador XT60 malo).
- `recolector_de_frutas/firmware/brazo` sigue con franja, homing y reposo viejos: falta pasarle lo de `20`.

## Pinza y muñeca (servos)

- En 20 (`pz`, `mn`): pinza GPIO 4 (abierta 500 µs, cerrada 1250; `agarrar` 1300; el teleop 1400), muñeca GPIO 14
  (cero 1500, ±90° en 860 / 2150 µs). El brazo del servo de la pinza está ~45° corrido (recalzar ~3 dientes).

## Visión (Pi)

- `bash ~/scripts/vision.sh` = `start_limon.sh` con el v3 (`/limon/posicion`); no arranca sola. Ubica limones a 0,40–0,95 m.

## Máquinas

- Pi 5 `r2d2@100.116.96.71` (Tailscale): las dos placas por USB (S3 `/dev/ttyACM0`, WROOM CH340 `/dev/ttyUSB0`), el GPS
  por la UART0. Se compila en la notebook y se flashea con `~/venvs/esptool/bin/esptool` desde la Pi. Scripts en
  `~/scripts/` (copias en el repo), logs en `~/logs/`. Al apagar: `sudo shutdown now` (sin `now` espera 1 min).
  Después de reiniciar: `agente_base.sh` y `vision.sh` a mano. `pgrep -f` por ssh se encuentra a sí mismo.
- Binarios en `~/firmware/`: WROOM `respaldo_wroom_20261112.bin` (20 `ros`), `brazo20_serie.bin` (20 `brazo`),
  `brazo/respaldo_20261001_183959.bin` (robot); S3 `base_ros_20261113`, `base_ros_20261117_imu` (cargado), `hcsr04_03*`.

## Pruebas terminadas

IMU MPU-6500 (01, 02), GPS UP501 (04, 05), lazo abierto 06/07 y 08, HC-SR04 (03; `d` = diagnóstico del ECHO, envs
`trig40` / `echo40`), bumpers (24), finales de carrera (18).
