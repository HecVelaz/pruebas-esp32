# Estado de pruebas-esp32 (al 2026-11-06, cierre de la noche)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`), lazo no
  bloqueante a 100 Hz, 20 kHz, sin el resorte de la pinza. Firmware flasheado: el del 05/11 noche (`073b4bd`:
  destinos con q3 − q2 ≥ −25°, freno anticipado de J3).
  - **Mecánica:** J3 con manivela de 95 mm; J2 y J3 con helicoidales impresos 15 → 75, en seco. **J3 tiene ~25–30° de
    juego en el antebrazo** (un diente entre el piñón y el engranaje de 75) y J2 algunos grados; sin corriente el brazo
    se queda donde está. **No se puede cambiar la mecánica** (decisión del usuario): todo por software.
  - **Franja:** `q2 − 49° ≤ q3 ≤ tabla(q2)` (0 → 3,2 · 15 → 10,8 · 30 → 27,9 · 45 → 54,2 · 60 → 72,2 · 75 → 82,7 ·
    90 → 91,9 · 105 → 101,6) para el camino y el freno; **destinos** (`mv`, `j`, joystick) además q3 − q2 ≥ −24°
    (`Q2_MENOS_Q3_DESTINO` 25 + 1°: más abajo J3 no queda firme). Margen 1° al destino y a todo el camino, 3° a los
    límites. J2 0–83° (destinos hasta 80°), J3 0–101,5°.
  - `h` (sin cero baja las tres juntas desde donde estén; arrancar con J3 ≤ J2); `zq` para bajar rápido sin cero.
    `mv`: quíntico, J2/J3 terminan juntas, J1 con su duración; si la recta sale de la franja, por un intermedio (desde
    el reposo, (15, 5)). Llegada desde abajo (J2/J3 al menos 0,3° bajo el objetivo antes del último tramo); J1 en el
    sentido en que viene. `tolLlegada` 0,15° y freno anticipado (J1 0,03 s, J2 0,055 s, J3 0,045 s). Reposo `mv 3 10 5`.
  - **Repetibilidad** (`repetibilidad_20261105_201209`, 30 ciclos, firmware final): RP **J1 0,18°, J2 0,05°, J3
    0,12°**, las tres dentro de ±0,2° el 97 % y de ±0,3° el 100 %. El juego no la afecta: el peso lo apoya siempre del
    mismo lado en las poses quietas. Error fijo ~0,14° (J1 en 40/60°, J2 en 20/30°), < 1 mm en la punta.
  - **Cinemática (calibración con video, `trayectoria_20261105_164644`):** **J2 gira 1,170° reales por grado del
    encoder** (reductora ~120:1; confirmado con inclinómetro; el firmware sigue en grados de encoder). Brazo =
    113,0 − 1,087·q2 − 0,094·q3; manivela = 171,3 − 0,851·q3 − 0,133·q2 (lineal); antebrazo 335 mm; pinza horizontal
    a (179, 29) mm de la muñeca. Modelo de la punta con 12 poses: **~9 mm de error medio, hasta ~3 cm en los bordes.**
  - **`tools/cinematica.py`** (verificado con cinta el 05/11: ±1 cm, la pose estirada (600, 150) 2 cm afuera;
    hombro 175 mm): directa e inversa de la punta en el marco del robot (origen en J1 a la altura de la tapa, x
    derecha, y adelante, z arriba), con límites, franja y choques con la caja, el mástil y el piso.
    **`tools/punta.py`** (probado en placa): `p x y z` lleva la punta ahí y revisa los choques en todo el camino (el
    firmware no conoce la caja: con `mv` a mano no hay protección).
  - `tools/trayectoria.py`: si un `mv` se corta, vuelve solo al reposo y hace `h` (`--sin-rescate`); acepta `zq` primero.
  - Joystick `jv` (`17/tools/joystick.py --placa 20`). J1 0–118° PROVISORIO. La caja no está en el firmware.
- **Cero del brazo:** la pose con los tres finales de carrera apretados (foto del 30/10); + = alejarse de cada uno.
  Límites: J1 0–118° (brazo recogido), J2 0–83°, J3 0–101,5°; franja con tabla (04/11, ver arriba). Finales de carrera:
  J1 32, J2 27, J3 33. **Tienen un solo punto de apoyo y el golpe los gira** (el 31/10 el cero se corrió ~9° en J3 y
  ~30° en J2 con pulsos de 25 %): llegar despacio (`h`) y fijarlos mejor (pendiente).
- **20 kHz en J2 y J3** (decisión del 31/10, sin pitido): más duty por la misma fuerza (J2 arranca subiendo con 35 %);
  J3 sostiene con compensación de fricción 15 %. A 20 kHz la fricción sostiene el brazo en un rango ancho de duty: el
  mapa de gravedad (`19/resultados/mapa_gravedad_20261031_135400.*`) no sirve como modelo (`gm` apagado).
- **Abrir el puerto serie reinicia la WROOM** (cero perdido, brazo suelto): después del monitor, `h`.
- **Fuente:** conectada a la bornera del bus de 12 V (el adaptador al XT60 tiene un contacto malo). Así no pasa por el
  fusible ni la seta: límite de 3 A y apagar la fuente como parada.
- **`12`:** J1 lista (0–5), repetir el paso 3 con el cero nuevo. **`17`:** pega fuerte en los FC. **`18`:** OK.
- **Robot:** `firmware/brazo` sigue con la polea de 90, cero y signos viejos de J1, FC de J2/J3 al revés y 1 kHz.

## Pinza y muñeca (servos)

- **`23-servos-wroom` (lo actual):** los dos en la **WROOM-32D del brazo** con los pines del robot: **pinza GPIO 4,
  muñeca GPIO 14** (verificado contra `todo.html` / `placa_wroom.html`; los dos responden). `p <cmd>` / `m <cmd>` con
  los comandos y valores de 21 y 22; EN 23 de los IBT-2 en LOW. Código con los valores de 21 (cero 1500 µs).
  - **Pinza con el brazo del servo ~45° corrido** al montarla: cero visual en 1000 µs, `p a` abre 45°. Falta
    recalzarlo (~3 dientes) para que 500 µs vuelva a ser 90° (73 mm). Muñeca: se mueve con los valores de 22.
- Sueltas en la ESP32 "ESP32S" (CP2102): **`21`** (GPIO 18) cero 1500, abierto 500 µs = 73 mm, cerrado 1250 µs;
  **`22`** (GPIO 19) cero 1500, ±90° en 860 / 2150 µs (~7,2 µs/°). Detalle en `BITACORA.md`.

## Base (S3)

- **`24-bumpers-s3`:** los tres responden (frente 14, izq. 21, der. 39; 0 rebotes, 06/11); falta el paragolpes montado.
- **`03-hcsr04`:** los tres HC-SR04 responden (TRIG 10/41/42, ECHO 11/12/13, driver de la base; 06/11); falta la cinta.
- **IMU (`01`):** responde en la base (SDA 8, SCL 9; gyro y −5 °/s sin calibrar); `02` compilado, RViz sin probar.
  Firmware en la S3: incierto (`24` o `03`). `01`–`03` y `24` usan el puerto by-id.

## Máquinas

- Notebook y Pi 5 (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`) ordenadas igual: repos en `~/Documentos` (Pi:
  `~/firmware`, `~/scripts`, `~/logs`), workspaces en `~/ros/`, venvs en `~/venvs/`. Las dos placas están en la Pi
  (se compila en la notebook y se flashea con esptool desde la Pi); la WROOM del brazo, hoy, en la notebook
  (`/dev/ttyUSB1`; `tools/monitor.sh` la abre por id).
- Respaldo del firmware micro-ROS del brazo de la WROOM de la Pi: `~/firmware/brazo/respaldo_20261001_183959.bin`.

## Pruebas terminadas
- IMU MPU-6500 (01, 02) OK, sin magnetómetro; GPS UP501 (04, 05) OK por WiFi; lazo abierto 06/07 y 08 (`Freno`) OK.

## Firmware del robot

- El brazo vive en `~/Documentos/recolector_de_frutas/firmware/brazo`; sus pendientes están en el `CLAUDE.md` de ese
  repo. Falta pasarle lo de hoy (ver `PENDIENTES.md`).
