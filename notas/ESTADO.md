# Estado de pruebas-esp32 (al 2026-11-11, cierre)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`), lazo no
  bloqueante a 100 Hz, 20 kHz. **Flasheado (11/11 tarde): caja desde J1 53,5 (sin probar), pinza y muñeca (`pz`,
  `mn`) y `vel <factor> [aprox] [asentamiento s]`** (en vivo; al reiniciar vuelve a 0,5 / 2 / 2 de `config.h`).
  - **Agarre automático** (`punta.py`: `ap`, `ret`, `tarea`): ciclo completo OK con un limón de juguete (A =
    (−550, 100, 100), B = (−450, 350, 100)): entra y sale de frente y horizontal con J1 quieta, `pz 1250`, `mn a`. Con
    `vel 0.75 2 0.5`: **76 s** (con la de `config.h`, 96); "ir frente a A" tarda 30 s (sin diagnosticar). J2 a veces
    no llega al salir con el limón (un reintento).
  - **Mecánica (10/11):** J2 y J3 con **doble helicoidales** 15 → 75, en seco. J3 con manivela de 95 mm. **Eje de J1
    reemplazado** (misma pieza) el 10/11 a la noche: cero, recorrido 3–115° y repetibilidad bien.
  - **Franja (10/11 noche, medida entera con el joystick):** tablas `FRANJA_ARRIBA` (desde J2 60, traba − 2°) y
    `FRANJA_ABAJO` más las rectas `q2 − 49` (camino) / `q2 − 25` (destinos), en `config.h` (valores en la bitácora
    del 10/11). Probada en placa. Margen 1° al destino y al camino, 3° a los límites. `fr 0` / `fr 1`.
  - **Homing `h`:** termina en **J1 3°, J2 6°, J3 3°** (se leen 2,8 / 6,0 / 3,2). J2 con cero busca a 5 °/s hasta 4°
    del cero anterior (flanco a 1,5). J3 espera en su flanco hasta que J2 pasa 5° subiendo. `zq` para bajar sin cero.
    Reposo de los scripts: **`mv 3 17 4`**.
  - `mv`: quíntico, J2/J3 terminan juntas, J1 con su duración; si la recta sale de la franja, por un intermedio.
    Llegada desde abajo; `tolLlegada` 0,15° y freno anticipado (J1 0,03 s, J2 0,055 s, J3 0,045 s).
  - **Caja (11/11):** con J1 ≥ `CAJA_J1_MIN`, J3 máximo según J2 (`CAJA_ARRIBA`: 64 → 73,2 · 68–69 → 65,0 ·
    70 → 66,2 · 74 → 69,5 · 80 → 74,2; muñeca a ≥ 3 cm de la tapa). `mv` revisa con la J1 más alta del camino; con
    J3 alta J1 no entra; protección `>> CAJA`. Desde J1 64: frenó en 65,2 con la muñeca a 4 cm, pero con J1 63,5 la
    muñeca llegó a 1 cm de la tapa → **53,5** (flasheado, sin probar). Activa también con `fr 0`.
  - **Repetibilidad** (`repetibilidad_20261110_224409`, 10 ciclos, con J3 70): RP **J1 0,13°, J2 0,20°, J3 0,06°**; 5 ciclos con
    la franja completa (`_230137`): 0,06 / 0,20 / 0,08. J2 la peor (ganancias y feedforward de los engranajes
    viejos; tarda ~0,4 s en despegar).
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
- **Robot:** la WROOM tiene el firmware de `20`; `firmware/brazo` sigue con franja, homing y reposo viejos.

- **Cámara → brazo (12/11 noche, medida):** 9 puntos, TF `brazo_base -> stereo_camera_link` x 0,1944 y −0,2736
  z 0,3856 m, roll 0,0422 pitch 0,2425 yaw 1,5794 rad (cámaras ~14° hacia abajo); error dejando cada punto afuera
  ~23 mm de media, 46 máx. El modelo de la punta tiene error en z (el brazo sube/baja ~1,26 × lo que calcula): eso
  limita la TF. **La WROOM de la Pi tiene el env `brazo` (serie)**; respaldo del `ros` en `~/firmware/respaldo_wroom_20261112.bin`.

## Pinza y muñeca (servos)

- **En 20 (`pz`, `mn`) y `23-servos-wroom`:** pinza GPIO 4, muñeca GPIO 14, valores de 21 y 22 (cero 1500 µs).
  - **Pinza con el brazo del servo ~45° corrido** al montarla: cero visual en 1000 µs, `p a` abre 45°. Falta
    recalzarlo (~3 dientes) para que 500 µs vuelva a ser 90° (73 mm). Muñeca: se mueve con los valores de 22.
- Sueltas en la ESP32 "ESP32S" (CP2102): **`21`** (GPIO 18) cero 1500, abierto 500 µs = 73 mm, cerrado 1250 µs;
  **`22`** (GPIO 19) cero 1500, ±90° en 860 / 2150 µs (~7,2 µs/°). Detalle en `BITACORA.md`.

## Teleop (`25-teleop-pi`, ROS 2 + micro-ROS en la Pi)

- `bash ~/scripts/25-teleop-pi/tools/lanzar.sh` (dos agentes + teleop) → `http://100.116.96.71:8000`, horizontal.
  **WROOM con 20 env `ros`** (flasheado; homing, joystick, pinza y muñeca OK desde el celular).
- **Base:** la S3 está **en la notebook con `03-hcsr04`** (los tres HC-SR04 OK); falta flashear el `ros` completo y
  volver a la Pi. Tracción: la rueda izquierda se escapaba (sin cuentas): **era una tierra suelta, arreglada**.
  IMU real sin confirmar (con `ros` daba SIN_DATOS; con `ros_motores` no se mira). La S3 a veces no arranca después
  de flashear o reconectar el USB: RST; el by-id cambia de ttyACM0 a ACM1.
- `24` (bumpers 14/21/39) y `03` (HC-SR04 10-11 / 41-12 / 42-13) OK; IMU (`01`) respondía el 06/11 (SDA 8, SCL 9).

## Máquinas

- Notebook y Pi 5 (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`) ordenadas igual: repos en `~/Documentos` (Pi:
  `~/firmware`, `~/scripts`, `~/logs`), workspaces en `~/ros/`, venvs en `~/venvs/`. Las dos placas están en la Pi
  (se compila en la notebook y se flashea con esptool desde la Pi: `~/venvs/esptool/bin/esptool`). Respaldos de la
  WROOM: `~/firmware/brazo/respaldo_20261001_183959.bin` (micro-ROS del robot) y `respaldo_wroom_20261111.bin` (20).

## Pruebas terminadas: IMU MPU-6500 (01, 02) OK, sin magnetómetro; GPS UP501 (04, 05) OK por WiFi; lazo abierto 06/07 y 08 (`Freno`) OK.

## Firmware del robot

- El brazo vive en `~/Documentos/recolector_de_frutas/firmware/brazo`; sus pendientes están en el `CLAUDE.md` de ese
  repo. Falta pasarle lo de `20` (ver `PENDIENTES.md`).
