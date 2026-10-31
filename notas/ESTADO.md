# Estado de pruebas-esp32 (al 2026-10-31, tarde, segunda sesión)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`) en un lazo no
  bloqueante a 100 Hz, a 20 kHz. **5a, 5b y 5c probadas en placa.** `h` (homing J3 → J2 → J1, cero en el flanco)
  repite al nivel de una cuenta del encoder; `mv q1 q2 q3` mueve las tres juntas (perfil quíntico, recta en (q2, q3)),
  **todas llegan con la aproximación desde abajo** (2° por debajo y el último tramo a 2 °/s) y el sostener arranca
  del duty que sostiene el peso (J2 +8,5 %, J3 −10 %). Repetir `mv 20 20 24` desde poses distintas: J1 0,00°, J2
  0,008°, J3 0,04°; firmes en 0,2–0,5 s. Tolerancias: J1 y J2 `tolPos` 0,3° / `bandaInt` 0,2°, J3 0,15° / 0,1°;
  `khi` 8 (con 4 no quedaban firmes). Queda: J3 se traba y se pasa en la subida lenta (hasta 0,55°). Telemetría:
  `t` y `tools/trayectoria.py h "mv ..." ...` (CSV y PNG en `resultados/`). J1 0–90° PROVISORIO; feedforward de J3
  en cascada medido con la telemetría ≈ el de `config.h`; `MV_FACTOR_VEL` 0,5 PROVISORIO. No esquiva la caja.
- **`19-j2-j3-conjunto`:** banco de pruebas de J2 y J3 (una activa, la otra sostenida). Fases 1–4 del plan probadas
  en placa (franja, sostener con banda 0,2° / compensación de fricción / freno en banda, mapa de gravedad, homing);
  20 kHz. La fase 5 siguió en `20`.
- **Cero del brazo:** la pose con los tres finales de carrera apretados (foto del 30/10); + = alejarse de cada uno.
  Límites: J1 0–118° (brazo recogido), J2 0–97°, J3 0–113°; franja `q2 − 25° ≤ q3 ≤ q2 + 7,5°`. Finales de carrera:
  J1 32, J2 27, J3 33. **Tienen un solo punto de apoyo y el golpe los gira** (el 31/10 el cero se corrió ~9° en J3 y
  ~30° en J2 con pulsos de 25 %): llegar despacio (`h`) y fijarlos mejor (pendiente).
- **20 kHz en J2 y J3** (decisión del 31/10, sin pitido): más duty por la misma fuerza (J2 arranca subiendo con 35 %);
  J3 sostiene con compensación de fricción 15 %. A 20 kHz la fricción sostiene el brazo en un rango ancho de duty: el
  mapa de gravedad (`19/resultados/mapa_gravedad_20261031_135400.*`) no sirve como modelo (`gm` apagado).
- **Abrir el puerto serie reinicia la WROOM** (cero perdido, brazo suelto): después de abrir el monitor, `h`.
- **Fuente:** conectada a la bornera del bus de 12 V (el adaptador al XT60 tiene un contacto malo). Así no pasa por el
  fusible ni la seta: límite de 3 A y apagar la fuente como parada.
- **`12-j1-base`:** J1 terminada (pasos 0–5); con el cero en el final de carrera falta repetir el paso 3.
- **`17-brazo-joystick`:** probado en placa (también desde el celular). Ojo: pega fuerte contra los finales de
  carrera. **`18-finales-de-carrera`:** los tres responden.
- **Robot:** `recolector_de_frutas/firmware/brazo` sigue con la polea de 90, el cero y los signos viejos de J1, los
  pines de los finales de carrera de J2/J3 al revés y 1 kHz.

## Máquinas

- Notebook y Pi 5 (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`) ordenadas igual: repos en `~/Documentos` (Pi:
  `~/firmware`, `~/scripts`, `~/logs`), workspaces en `~/ros/`, venvs en `~/venvs/`. Las dos placas están en la Pi
  (se compila en la notebook y se flashea con esptool desde la Pi); la WROOM del brazo, hoy, en la notebook
  (`/dev/ttyUSB1`; `tools/monitor.sh` la abre por id).
- Respaldo del firmware micro-ROS del brazo de la WROOM de la Pi: `~/firmware/brazo/respaldo_20261001_183959.bin`.

## Sensores en la S3

- **IMU (01, 02): OK.** MPU-6500 (sin magnetómetro: el yaw deriva). `/imu/data_raw` a 50 Hz por micro-ROS.
- **GPS UP501 (04, 05): OK por WiFi.** `/gps/fix` y `/gps/status`; `-e serial` sin probar; cables sueltos.

## Motores en lazo abierto

- **06 / 07 (36GP-555 + IBT-2, S3): OK.** K ≈ 1,6 rpm/%, zona muerta ~8–11 %, τ ≈ 60–74 ms.
- **08 (5840-31ZY + L298N, WROOM): OK.** Modo `Freno` casi lineal entre 40 y 100 %.

## Brazo en `10-control-lazo-cerrado` (historia, antes de 12–14)

- J1 terminada ahí (cascada P → PI, Kpp 2, Kp 0,549, Ki 4); J2/J3 con el 5840 (sin fin) descartadas; simulaciones e
  informe en `simulaciones/`. `tools/` de `10` lo comparten `12`–`14` y `19` (scripts, `mapa_gravedad.py`).

## Firmware del robot

- El brazo vive en `~/Documentos/recolector_de_frutas/firmware/brazo`; sus pendientes están en el `CLAUDE.md` de ese
  repo. Falta pasarle lo de hoy (ver `PENDIENTES.md`).
