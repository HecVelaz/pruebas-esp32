# Estado de pruebas-esp32 (al 2026-11-13)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **Agarre automático con la visión (13/11): `agarrar` en `punta.py` funciona** (Pi, `start_limon.sh` v3, WROOM con el
  env `brazo`): mide 5 s, calibración del 12/11, pinza a 1300 us, 1 cm más abajo, salida con 3° de margen. Ahora la
  WROOM de la Pi tiene el env **`brazo`** (serie); volver al `ros` con `respaldo_wroom_20261112.bin`.
- **Simulación de la estación (`26-sim-alcance-arbolito`, 13/11):** agarra a 55–70 cm de J1 y 30–66 cm del piso. Con el
  arbolito medido, **4 paradas cada 90° = 100 %**; parada: eje del árbol a 70 cm a la izquierda de J1 y 15 cm
  adelante. Sin validar con el robot.
- **Visión → brazo (12/11 noche): agarró todos los limones pedidos.** El usuario cuelga el limón; `medir_limon.py` (Pi,
  con la visión) → `ajuste_camara.py --sin P5 --radio 25 --limon x y z` → `ap` en `punta.py` (a mano). TF
  `brazo_base -> stereo_camera_link`: x 0,1961 y −0,2694 z 0,3833 m, roll 0,0425 pitch 0,2415 yaw 1,5810 rad
  (cámaras ~14° hacia abajo; `radio_limon:=0.025`); error dejando cada punto afuera ~22 mm, 45 máx. Límite: el modelo
  de la punta en z (el brazo sube/baja ~1,26 × lo calculado). Alcance con `ap`: r ≳ 50 cm de J1; cerca y alto no llega.
  `ap` gira J1 primero; `ret` sin números retrocede lo máximo. Datos: `resultados/camara_brazo_20261112_203642_*`.
- **La WROOM de la Pi tiene el env `ros`** (restaurado el 12/11 al cierre). Para `punta.py` hace falta el serie:
  `write_flash 0x0 ~/firmware/brazo20_serie.bin`; volver con `respaldo_wroom_20261112.bin` (12 V apagados).
- **Visión en la Pi:** apagada al cierre del 12/11 (para el agarre: `start_limon.sh` con el v3, `MODELO=... CLASE=limon`).
- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`), lazo no
  bloqueante a 100 Hz, 20 kHz. **Flasheado (11/11 tarde): caja desde J1 53,5 (sin probar), pinza y muñeca (`pz`,
  `mn`) y `vel <factor> [aprox] [asentamiento s]`** (en vivo; al reiniciar vuelve a 0,5 / 2 / 2 de `config.h`).
  - **Agarre automático** (`punta.py`: `ap`, `ret`, `tarea`): ciclo completo OK con un limón de juguete (A =
    (−550, 100, 100), B = (−450, 350, 100)): entra y sale de frente y horizontal con J1 quieta, `pz 1250`, `mn a`. Con
    `vel 0.75 2 0.5`: **76 s** (con la de `config.h`, 96); "ir frente a A" tarda 30 s (sin diagnosticar). J2 a veces
    no llega al salir con el limón (un reintento).
  - **Mecánica (10/11):** J2 y J3 con doble helicoidales 15 → 75, en seco; J3 con manivela de 95 mm; eje de J1 nuevo.
  - **Franja (10/11 noche, medida entera con el joystick):** tablas `FRANJA_ARRIBA` (desde J2 60, traba − 2°) y
    `FRANJA_ABAJO` más las rectas `q2 − 49` (camino) / `q2 − 25` (destinos), en `config.h` (valores en la bitácora
    del 10/11). Probada en placa. Margen 1° al destino y al camino, 3° a los límites. `fr 0` / `fr 1`.
  - **Homing `h`:** termina en **J1 3°, J2 6°, J3 3°** (se leen 2,8 / 6,0 / 3,2). J2 con cero busca a 5 °/s hasta 4°
    del cero anterior (flanco a 1,5). J3 espera en su flanco hasta que J2 pasa 5° subiendo. `zq` para bajar sin cero.
    Reposo de los scripts: **`mv 3 17 4`**.
  - `mv`: quíntico, J2/J3 juntas, J1 con su duración; intermedio si la recta sale de la franja; llegada desde abajo.
  - **Caja (11/11):** J3 máximo según J2 con J1 ≥ `CAJA_J1_MIN` = 53,5 (sin probar; tabla en la bitácora del 11/11).
  - **Repetibilidad** (10/11): RP J1 0,13°, J2 0,20°, J3 0,06° (J2 la peor).
  - **Cinemática** (`tools/cinematica.py`, **recalibrada con video el 11/11**, 19 poses, error 10 mm de media y
    25 máx): hombro (J2 y J3) a **165 mm**; J1 real `(q1 − 2,8) · 90 / 83,3` (**confirmada** con cinta a 0°, 45° y
    90°; el firmware sigue con la escala de la polea de 84). Marco: x a la derecha del robot, y adelante; **J1 a la
    izquierda** (0 = −x), mástil a la derecha (300, 240, a ojo). Tapa en **T**: ancha (40) adelante, angosta (34)
    atrás con J1 a 9 cm del borde de atrás y del izquierdo. Inversa con `punta.py` probada con cinta: bien. La punta no
    llega sobre la tapa (mínimo 36 cm de J1). La caja del firmware sigue siendo empírica.
  - Joystick `jv` (`17/tools/joystick.py --placa 20 [--sin-franja]`; botones Homing y Anotar).
- **Cero del brazo:** la pose con los tres finales de carrera apretados (foto del 30/10); + = alejarse de cada uno.
  Límites: J1 0–118° (brazo recogido), J2 0–83°, J3 0–101,5°; franja con tablas (ver arriba). Finales de carrera:
  J1 32, J2 27, J3 33. **Tienen un solo punto de apoyo y el golpe los gira**: llegar despacio (`h`) y fijarlos
  mejor (pendiente).
- **20 kHz en J2 y J3** (31/10, ver `DECISIONES.md`); compensación de gravedad apagada (`gm`).
- **Abrir el puerto serie reinicia la WROOM** (cero perdido, brazo suelto): después del monitor, `h`.
- **Fuente:** conectada a la bornera del bus de 12 V (el adaptador al XT60 tiene un contacto malo). Así no pasa por el
  fusible ni la seta: límite de 3 A y apagar la fuente como parada.
- **`12`:** J1 lista (0–5), repetir el paso 3 con el cero nuevo. **`17`:** pega fuerte en los FC. **`18`:** OK.
- **Robot:** `recolector_de_frutas/firmware/brazo` sigue con franja, homing y reposo viejos: falta pasarle lo de `20`.

## Pinza y muñeca (servos)

- **En 20 (`pz`, `mn`) y `23-servos-wroom`:** pinza GPIO 4, muñeca GPIO 14, valores de 21 y 22 (cero 1500 µs).
  - **Pinza con el brazo del servo ~45° corrido** al montarla: cero visual en 1000 µs, `p a` abre 45°. Falta
    recalzarlo (~3 dientes) para que 500 µs vuelva a ser 90° (73 mm). Muñeca: se mueve con los valores de 22.
- Sueltas en la ESP32 "ESP32S" (CP2102): **`21`** (GPIO 18) cero 1500, abierto 500 µs = 73 mm, cerrado 1250 µs;
  **`22`** (GPIO 19) cero 1500, ±90° en 860 / 2150 µs (~7,2 µs/°). Detalle en `BITACORA.md`.

## Teleop (`25-teleop-pi`, ROS 2 + micro-ROS en la Pi)

- `bash ~/scripts/25-teleop-pi/tools/lanzar.sh` (dos agentes + teleop) → `http://100.116.96.71:8000`, horizontal.
  Con 20 env `ros` en la WROOM: homing, joystick, pinza y muñeca OK desde el celular .
- **Base:** la S3 está **en la notebook con `03-hcsr04`** (los tres HC-SR04 OK); falta flashear el `ros` completo y
  volver a la Pi. Tracción: la rueda izquierda se escapaba (sin cuentas): **era una tierra suelta, arreglada**.
  IMU real sin confirmar (con `ros` daba SIN_DATOS; con `ros_motores` no se mira). La S3 a veces no arranca después
  de flashear o reconectar el USB: RST; el by-id cambia de ttyACM0 a ACM1.
- `24` (bumpers 14/21/39) y `03` (HC-SR04 10-11 / 41-12 / 42-13) OK; IMU (`01`) respondía el 06/11 (SDA 8, SCL 9).

## Máquinas

- Notebook y Pi 5 (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`) ordenadas igual: repos en `~/Documentos` (Pi:
  `~/firmware`, `~/scripts`, `~/logs`), workspaces en `~/ros/`, venvs en `~/venvs/`. Las dos placas están en la Pi
  (se compila en la notebook y se flashea con esptool desde la Pi: `~/venvs/esptool/bin/esptool` o el 4.8.1 de
  `~/keepon_env`). Respaldos de la WROOM: `~/firmware/brazo/respaldo_20261001_183959.bin` (micro-ROS del robot),
  `respaldo_wroom_20261111.bin` y `respaldo_wroom_20261112.bin` (20 env `ros`).

## Pruebas terminadas: IMU MPU-6500 (01, 02) OK, sin magnetómetro; GPS UP501 (04, 05) OK por WiFi; lazo abierto 06/07 y 08 (`Freno`) OK.
