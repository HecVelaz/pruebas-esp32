# Estado de pruebas-esp32 (al 2026-11-17, cierre)

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
- **HC-SR04 derecho mal** (17/11: `NO RESPONDE` / `sin eco` / 3,3 cm fijo; placa, pines 42 / 13 y alimentación
  descartados): cambiarlo. Solo el del frente frena la base. **IMU: el firmware de la Pi no se recupera de un I2C
  colgado** (en el pasto se trancó con SIN_DATOS hasta apagar); la recuperación está hecha en `firmware/base`
  (`~/firmware/base_ros_20261117_imu/`, sin probar en placa; la S3 tiene `base_ros_20261113`).
- Todo responde con 12 V: batería ~11,2 V (LiPo 3S sin BMS, corte 10,5 V), HC-SR04 ×3, bumpers ×3, seta suelta, IMU
  (era un cable; calibra el giróscopo sola al arrancar). **Odometría:** recto < 1 % con las ruedas; giros con el
  giróscopo (~±1°; las ruedas se pasan ~2° cada 90°). `25-teleop-pi/tools/movimiento.py` (frenado proporcional),
  `prueba_odom.py recto|giro|tiempo`.
- Sin probar: frenado por HC-SR04 (< 20 cm), bumper y vuelco con motores; cortes del USB (se desarma al reconectar).

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
  `brazo/respaldo_20261001_183959.bin` (robot); S3 `base_ros_20261113`, `base_ros_20261117_imu`, `hcsr04_03*`.

## Pruebas terminadas

IMU MPU-6500 (01, 02), GPS UP501 (04, 05), lazo abierto 06/07 y 08, HC-SR04 (03; `d` = diagnóstico del ECHO, envs
`trig40` / `echo40`), bumpers (24), finales de carrera (18).
