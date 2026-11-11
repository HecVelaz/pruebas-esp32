# Estado de pruebas-esp32 (al 2026-11-10, noche)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`), lazo no
  bloqueante a 100 Hz, 20 kHz, sin el resorte de la pinza. **Flasheado: el del 10/11 noche** (franja completa).
  - **Mecánica (10/11):** J2 y J3 con **doble helicoidales** 15 → 75, en seco. J3 con manivela de 95 mm. **Eje de J1
    reemplazado** (misma pieza) el 10/11 a la noche: cero, recorrido 3–115° y repetibilidad bien.
  - **Franja (10/11 noche, medida entera con el joystick):** borde de arriba, tabla `FRANJA_ARRIBA` (0 → 0,5 est. ·
    3,4 → 1,05 · 5,8 → 8,1 · 10,6 → 10,3 · 15,1 → 15,4 · 21,0 → 20,3 · 30,4 → 32,4 · 45,2 → 51,9 · 60,1 → 70,8 ·
    74,6 → 83,3 · 79,8 → 88,1; hasta 21 lo alcanzado sin nada, 30–45 arrastre, desde 60 traba − 2°). Borde de abajo:
    rectas `q2 − 49` (camino) / `q2 − 25` (destinos) **y** tabla `FRANJA_ABAJO` (50 → 1,0 · 55,1 → 22,5 ·
    60,9 → 40,3 · 65,2 → 44,2 · 70,0 → 45,2 · 74,7 → 47,8 · 80,0 → 57,5, arrastre sin margen). Probada en placa
    (frena en ~45 y en 82,3). Margen 1° al destino y al camino, 3° a los límites. `fr 0` / `fr 1` apaga / prende.
  - **Homing `h`:** termina en **J1 3°, J2 6°, J3 3°** (se leen 2,8 / 6,0 / 3,2). J2 con cero busca a 5 °/s hasta 4°
    del cero anterior (flanco a 1,5). J3 espera en su flanco hasta que J2 pasa 5° subiendo. `zq` para bajar sin cero.
    Reposo de los scripts: **`mv 3 17 4`**.
  - `mv`: quíntico, J2/J3 terminan juntas, J1 con su duración; si la recta sale de la franja, por un intermedio.
    Llegada desde abajo; `tolLlegada` 0,15° y freno anticipado (J1 0,03 s, J2 0,055 s, J3 0,045 s).
  - **Repetibilidad** (`repetibilidad_20261110_224409`, 10 ciclos): RP **J1 0,13°, J2 0,20°, J3 0,06°**; 5 ciclos con
    la franja completa (`_230137`): 0,06 / 0,20 / 0,08. J2 la peor (ganancias y feedforward de los engranajes
    viejos; tarda ~0,4 s en despegar).
  - **Cinemática: DESACTUALIZADA** (modelo del 05/11, con el juego viejo). Con la franja nueva J3 llega más alto:
    **caminos de las poses sin revisar contra la caja** (lo primero de la próxima sesión).
  - Joystick `jv` (`17/tools/joystick.py --placa 20 [--sin-franja]`; botones Homing y Anotar). J1 0–118°
    PROVISORIO. La caja no está en el firmware.
- **Cero del brazo:** la pose con los tres finales de carrera apretados (foto del 30/10); + = alejarse de cada uno.
  Límites: J1 0–118° (brazo recogido), J2 0–83°, J3 0–101,5°; franja con tablas (ver arriba). Finales de carrera:
  J1 32, J2 27, J3 33. **Tienen un solo punto de apoyo y el golpe los gira**: llegar despacio (`h`) y fijarlos
  mejor (pendiente).
- **20 kHz en J2 y J3** (decisión del 31/10, sin pitido): más duty por la misma fuerza (J2 arranca subiendo con 35 %);
  J3 sostiene con compensación de fricción 15 %. A 20 kHz la fricción sostiene el brazo en un rango ancho de duty: el
  mapa de gravedad (`19/resultados/mapa_gravedad_20261031_135400.*`) no sirve como modelo (`gm` apagado).
- **Abrir el puerto serie reinicia la WROOM** (cero perdido, brazo suelto): después del monitor, `h`.
- **Fuente:** conectada a la bornera del bus de 12 V (el adaptador al XT60 tiene un contacto malo). Así no pasa por el
  fusible ni la seta: límite de 3 A y apagar la fuente como parada.
- **`12`:** J1 lista (0–5), repetir el paso 3 con el cero nuevo. **`17`:** pega fuerte en los FC. **`18`:** OK.
- **Robot:** la WROOM tiene hoy el firmware de `20` (hay que volver a flashear `firmware/brazo`, que sigue con la
  franja, el homing y el reposo viejos).

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
  repo. Falta pasarle lo de `20` (ver `PENDIENTES.md`).
