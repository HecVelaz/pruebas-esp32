# Estado de pruebas-esp32 (al 2026-11-13, cierre)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Recolección autónoma (lo actual)

- **`27-aruco-arbol/tools/recolectar.py` hace todo (13/11: agarró el limón, 95 s):** homing del brazo → reposo → mira la
  marca de la maceta → va a la parada (girar con el giróscopo, avanzar con las ruedas, girar) → mide el limón 5 s →
  `agarrar`. 3 Enter: 12 V apagados (abre la WROOM), 12 V encendidos (homing), ir. En la Pi:
  `cd ~/scripts/27-aruco-arbol; python3 tools/recolectar.py` (con la visión y el agente de la base corriendo).
  Hoy arranca viendo la marca (a ~0,9–4 m a la izquierda) y agarra un limón desde una parada.
- **Parada** (`26-sim-alcance-arbolito`): tronco a **70 cm a la izquierda de J1 y 15 cm adelante**; agarra a 55–70 cm
  de J1 y 30–66 cm del piso; con el arbolito medido (60 cm, limones a 10–14 cm del tronco, de 32 a 54 cm del piso),
  **4 paradas cada 90° = 100 %**. Validada: desde ahí `agarrar` funciona.
- **Marcas ArUco** (`27`): DICT_4X4_50, 15 cm, ID 0 pegada en la maceta (cara a 16,5 cm del tronco: `--radio-maceta
  165`); faltan pegar las IDs 1–3 (una por cara, antihorario). Se detecta de ~0,9 a ~4 m; en la parada queda cortada
  abajo (ahí mandan los limones). `ver_marcas.py --tf nav` para mirarla.
- **Dos calibraciones de la cámara:** agarre = la del brazo del 12/11 (`ajuste_camara.py`, sirve < 70 cm; compensa con
  14° el error en z del modelo); navegación = `27/resultados/tf_navegacion.json` (6 puntos en el piso, ~3 cm a 1–2 m;
  cámara a 57,4 cm del piso, 6,5° hacia abajo).
- **GPS** UP501 en la Pi (UART0, `/dev/ttyAMA0`, 9600; `nmea_navsat_driver ... -r fix:=/gps/fix`): ±20 m, solo para
  la rúbrica (zona, registro, `robot_localization`). Adentro sin fix.

## Base (S3, `25-teleop-pi`)

- S3 en la Pi con el env `ros` de `recolector_de_frutas/firmware/base` (flasheo con `--after watchdog-reset`: arranca
  sola). Agente: `bash ~/scripts/agente_base.sh`; comandos: `bash ~/scripts/base.sh "<cmd>"` (`sens`, `e`, `cal`,
  `odo0`, `armar`, `r`). Con micro-ROS el watchdog es 300 ms: mover solo con `/cmd_vel` continuo.
- Todo responde con 12 V: batería ~11,2 V (LiPo 3S sin BMS, corte 10,5 V), HC-SR04 ×3, bumpers ×3, seta suelta, IMU
  (era un cable; calibra el giróscopo sola al arrancar). **Odometría:** recto < 1 % con las ruedas; giros con el
  giróscopo (~±1°; las ruedas se pasan ~2° cada 90°). `25-teleop-pi/tools/movimiento.py` (frenado proporcional),
  `prueba_odom.py recto|giro|tiempo`.
- Sin probar: frenado por HC-SR04 (< 20 cm), bumper y vuelco con los motores andando. El enlace USB tuvo un corte con
  los motores en marcha (la base se desarma al reconectar, por diseño).

## Brazo (`20-brazo-coordinado`, WROOM)

- **La WROOM de la Pi tiene el env `brazo` (serie)**, el de `punta.py` y `recolectar.py`. Para la misión en ROS / el
  teleop: `write_flash 0x0 ~/firmware/respaldo_wroom_20261112.bin` (env `ros`; 12 V apagados).
- **Abrir el puerto serie reinicia la WROOM** (cero perdido): siempre `h` después (los scripts lo hacen).
- `punta.py`: `p`, `ap` (entra de frente), `ret`, `tarea`, `anotar`, **`agarrar` / `veragarrar`** (mediana de
  `/limon/posicion`, calibración del 12/11, pinza **1300 us**, **1 cm más abajo**, salida y reposo con **3° de margen** a
  la franja). `punta.agarrar` y `Vision` se usan desde otros programas.
- J1, J2, J3 con `lib/Articulacion`, 100 Hz, 20 kHz; homing `h` → J1 3°, J2 6°, J3 3°; reposo `mv 3 17 4`; franja con
  tablas (`FRANJA_ARRIBA` / `FRANJA_ABAJO`), caja con J1 ≥ 53,5 (sin probar); repetibilidad RP 0,13 / 0,20 / 0,06°.
- Cinemática (`tools/cinematica.py`, video del 11/11): ~10 mm de media; hombro 165 mm; J1 real `(q1 − 2,8)·90/83,3`; el
  brazo real sube y baja ~1,26–1,43 × lo que dice el modelo en z (pendiente recalibrar).
- Cero en los finales de carrera (J1 32, J2 27, J3 33; un solo apoyo: el golpe los gira). Límites J1 0–118°, J2 0–83°,
  J3 0–101,5°. Fuente directa a la bornera del bus (sin fusible ni seta por el adaptador XT60 malo).
- `recolector_de_frutas/firmware/brazo` sigue con franja, homing y reposo viejos: falta pasarle lo de `20`.

## Pinza y muñeca (servos)

- En 20 (`pz`, `mn`): pinza GPIO 4 (abierta 500 µs, cerrada 1250; `agarrar` usa 1300), muñeca GPIO 14 (cero 1500,
  ±90° en 860 / 2150 µs). El brazo del servo de la pinza está ~45° corrido (recalzar ~3 dientes).

## Visión (Pi)

- `bash ~/scripts/vision.sh` = `start_limon.sh` con el v3 (`/limon/posicion`, `/left/image_rect`). No arranca sola al
  reiniciar la Pi. En la parada ve el limón entero; la visión ubica limones entre 0,40 y 0,95 m (u ≥ 280 px).

## Máquinas

- Pi 5 `r2d2@100.116.96.71` (Tailscale): las dos placas por USB (S3 `/dev/ttyACM0`, WROOM CH340 `/dev/ttyUSB0`), el GPS
  por la UART0. Se compila en la notebook y se flashea con `~/venvs/esptool/bin/esptool` desde la Pi. Scripts en
  `~/scripts/` (copias en el repo), logs en `~/logs/`. Al apagar: `sudo shutdown now` (sin `now` espera 1 min).
  Después de reiniciar: `agente_base.sh` y `vision.sh` a mano. `pgrep -f` por ssh se encuentra a sí mismo.
- Respaldos de la WROOM: `~/firmware/brazo/respaldo_20261001_183959.bin` (micro-ROS del robot), `respaldo_wroom_20261112.bin`
  (20 env `ros`); `~/firmware/brazo20_serie.bin` (20 env `brazo`).

## Pruebas terminadas

IMU MPU-6500 (01, 02), GPS UP501 (04, 05), lazo abierto 06/07 y 08, HC-SR04 (03), bumpers (24), finales de carrera (18).
