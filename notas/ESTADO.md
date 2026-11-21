# Estado de pruebas-esp32 (al 2026-11-21, cierre de la madrugada)

Foto de hoy. Historia y CSV en `BITACORA.md`; lo que sigue en `PENDIENTES.md`; pines de la S3 en `PINES_S3.md`.

## Recolección desde la app (lo actual)

- **App Arturito** (`recolector_de_frutas/app/android-app`, 0.11.1, Xiaomi Pad 5) + **puente** (`app/robot-bridge`,
  en la Pi `~/arturito-bridge-0.9.0`, `127.0.0.1:18006`). Tablet por USB a la notebook: `adb reverse tcp:18006
  tcp:18006` y túnel `ssh -N -L 127.0.0.1:18006:127.0.0.1:18006 r2d2@100.116.96.71`. Pi: `bash
  ~/arturito-bridge-0.9.0/arrancar_pi.sh` (agentes, visión v4 y puente; datos y clave en `~/arturito-bridge-0.4.0/data`).
- Flujo en **Misión**: **Hacer homing** (un botón; si el firmware lo rechaza por la franja, `j 2 6` y reintenta) →
  **Calcular cero con el limón** (pose de la base respecto del limón más cercano, 3 s de estéreo) → **Iniciar
  recolección** (1–5 limones) / **Volver al cero**. Probado: 1 limón a la canasta y vuelta al cero (0,9 cm / 4°);
  con 2, **1 de 2** (el bajo, a ~30 cm del piso, falló dos veces).
- Misión (`collection_mission.py`): limón a **50–55 cm de J1** (si ya está y tiene plan no se mueve; si no, se acerca en
  su dirección hasta 53,5 cm, nunca retrocede); en la parada solo vale el limón a < 12 cm de donde se esperaba;
  agarre de `punta.py` (+2 cm, `GRASP_RAISE_MM`) a la canasta; siguiente limón medido desde ahí; al final (o tras un
  fallo) brazo al reposo y base al cero por la odometría (girar, retroceder recto a ≤ 6 cm/s, girar). Usa
  `~/scripts/27-aruco-arbol/tools` y `~/scripts/20-brazo-coordinado/tools` de la Pi (planificadores en otro proceso).
- Registros: `~/arturito-bridge-0.4.0/data/missions/` (eventos + rosbag); pedidos con tiempos en
  `~/arturito-bridge-0.9.0/bridge.log`. Reiniciar el puente borra el cero (se puede pasar con `ARTURITO_RESTORE_ORIGIN_JSON`).
- **`27-aruco-arbol/tools/recolectar.py`** (marca → parada → `cosechar`) sigue en el repo pero usa el firmware **serie**
  del brazo: con la WROOM en micro-ROS no le habla.

## Visión (Pi)

- **Detector v4** (`~/vision_estereo/modelos/limon_yolo26n_v4_ncnn_model`, NCNN **fijo a 640**, `IMGSZ=640` en
  `~/scripts/vision.sh`): sin el falso limón sobre el panel de metal; ~2,5–4 img/s, `/limon/todos` ~1,8/s,
  disparidad ~1,5 Hz. Volver al v3: la línea comentada de `vision.sh` (`IMGSZ=480`).
- Estéreo a 0,40–0,95 m. A veces mide el follaje delante del limón (42 vs 77 cm). Si tras reiniciar la visión una vista
  queda sin imagen: segmento de Fast DDS corrupto en `/dev/shm/fastrtps_port*` (borrarlo, ver bitácora 20–21/11).

## Base (S3, `25-teleop-pi`)

- S3 con **`base_ros_20261117_imu`** (env `ros`): la IMU se recupera sola de un I2C colgado (`imu_rec`). Agente:
  `bash ~/scripts/agente_base.sh` (va antes que el del brazo); watchdog 300 ms.
- Seguridad con motores probada: frena el HC-SR04 del medio (reduce desde 50 cm, `no_avanza` a 20 cm), bumpers,
  seta, vuelco 25°, enclavados (`r` por `/base/comando`). Los de las esquinas solo publican.
- Batería ~11,1 V (LiPo 3S sin BMS, corte 10,5 V). Odometría: recto < 1 %; giros con el giróscopo (~±1°; al volver
  al cero queda ~4° de rumbo).

## Brazo (`20-brazo-coordinado`, WROOM)

- **WROOM de la Pi: env `ros` (micro-ROS) de la fuente `750ba4c`** (`~/arturito-build/20261120/firmware.bin`,
  `62695d4a…`; respaldo de antes `respaldo_wroom_20261120.bin` = el serie del 19/11). Agente:
  `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`; se conecta
  solo; si no, pulso en EN por software (RTS, DTR suelto) **con los 12 V apagados**. Respuestas por `/brazo/respuesta`.
- **Pose plegada** (`SOLTAR_*`): J1 hasta 130 solo con J2 42–51 y J3 ≥ 78. J3 cerca de 80 está dura: con destino 80 a
  veces corta por "sin cuentas" (la misión ya no se lo pide al girar J1 hacia la canasta).
- Cosecha (`punta.py`): entra de frente 2,8 cm más allá del centro, cierra, sube 5 y retrocede 5 cm, gira la muñeca,
  arrastra con J1, sale y va a la canasta (J1 130 / J2 46,7 / J3 80). Necesita r > ~50 cm para entrar desde 10 cm.
  Visión → brazo: `z = 23,1 + 0,718·z_visión` + ajuste fino de 9 puntos.
- Homing → J1 3°, J2 6°, J3 3°; reposo `mv 3 17 4`; repetibilidad 0,13 / 0,20 / 0,06°. Abrir el puerto serie reinicia
  la WROOM.

## Pinza y muñeca (servos)

- Pinza RDS3218, GPIO 4: abierta `pz a` = 500 µs, **cerrada `pz c` = 1200**. Muñeca GPIO 14 (cero 1500, ±90° en
  860 / 2150 µs).

## Teleop (25)

- Página del celular (`25/tools/teleop.py`, puerto 8000) con el env `ros` del brazo: ahora la WROOM ya lo tiene, con la
  pose plegada. Afuera la Pi entra al hotspot `POCO X6 Pro 5G`.

## Máquinas

- Pi 5 `r2d2@100.116.96.71` (Tailscale), **con la fuente oficial de 27 W** (con la powerbank se reiniciaba por bajo
  voltaje; `vcgencmd get_throttled` → `0x0`). Placas por USB (S3 `/dev/ttyACM0`, WROOM `/dev/ttyUSB0`). Se compila en
  la notebook y se flashea con `~/venvs/esptool/bin/esptool` desde la Pi. Apagar: `sudo shutdown now`.
- Binarios: WROOM `~/arturito-build/20261120/firmware.bin` (cargado, `ros`), `~/firmware/brazo20_serie_20261119.bin`
  (serie, para `recolectar.py`/`punta.py`), `respaldo_wroom_20261112.bin`; S3 `base_ros_20261117_imu` (cargado).
- Notebook: SDK Android en `~/Android/Sdk` (sin Android Studio); regla udev `/etc/udev/rules.d/51-android.rules`.

## Pruebas terminadas

IMU MPU-6500 (01, 02), GPS UP501 (04, 05), lazo abierto 06/07 y 08, HC-SR04 (03), bumpers (24), finales de carrera
(18), seguridad de la base con motores (25), recolección de un limón desde la app (cero, acercamiento, canasta, vuelta).
