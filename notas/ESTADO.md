# Estado de pruebas-esp32 (al 2026-11-05)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`), lazo no
  bloqueante a 100 Hz, 20 kHz, sin el resorte de la pinza. Firmware flasheado: el del commit del 05/11 (freno anticipado de J1).
  - **Mecánica (04/11):** J3 con **manivela de 95 mm** (antes 80); J2 y J3 con engranajes 15 → 75 (las correas GT2 se
    probaron y se sacaron). La calibración del cuatro barras del 03/11 (80 mm) ya no vale.
  - **Franja:** `q2 − 49° ≤ q3 ≤ tabla(q2)` (J2 → J3 máx: 0 → 3,2 · 15 → 10,8 · 30 → 27,9 · 45 → 54,2 · 60 → 72,2 ·
    75 → 82,7 · 90 → 91,9 · 105 → 101,6), margen 1° a la franja y 3° a los límites. **J2 0–83° (destinos hasta 80°)**:
    cerca de 90° J2 se pasa ~0,7° y no puede volver. J3 0–101,5° (dejar aire: cerca de la traba no queda firme).
  - `h`: con cero, despacio a los flancos; **sin cero baja las tres juntas desde donde estén** (sin límite de
    búsqueda; arrancar con J3 ≤ J2). `zq q1 q2 q3` para rescatarlo. `mv`: perfil quíntico, J2/J3 terminan juntas;
    **si la recta sale de la franja va en dos tramos** (por (10, 5) desde el reposo). **Reposo `mv 3 10 5`** (vuelta
    de los scripts). J2/J3 llegan desde abajo; **J1 en el sentido en que viene y frena antes de cruzar el destino**
    (no invierte; llegada con 0,15° o anticipando el deslizamiento). "Sin cuentas" de J2/J3 a 60 %. No esquiva la caja.
  - **Repetibilidad** (`repetibilidad_20261105_111907`, 10 ciclos × 5 poses, sin cortes): RP **J1 0,13°, J2 0,43°,
    J3 0,08°** (04/11: 0,16 / 0,21 / 0,10). Las tres dentro de ±0,3° el 100 % (04/11: 60 %). **J1 resuelta** (05/11:
    `tolLlegada` 0,15° y freno anticipado; videos de la punta: la punta sigue al encoder, juego de la correa 0,04°).
    **Lo que queda es J2 en 74°:** se pasó dos veces en 10 (74,27 / 74,29); en 20° y 30°, error fijo ±0,2°.
    Siguiente: lubricar los engranajes de J2/J3 y repetir la repetibilidad antes de tocar el control de J2.
  - Joystick `jv` (puente `17/tools/joystick.py --placa 20`). `17 -e explorar`: sin franja, para medir límites.
  - **J1 0–118° PROVISORIO** (libre hasta 115° recogido; falta estirado y más allá). La caja no está en el firmware
    (geometría a ojo en la bitácora del 03/11). `MV_FACTOR_VEL` 0,5 PROVISORIO.
- **`19-j2-j3-conjunto`:** banco de pruebas de J2 y J3 (una activa, la otra sostenida). Fases 1–4 del plan probadas
  en placa (franja, sostener con banda 0,2° / compensación de fricción / freno en banda, mapa de gravedad, homing);
  20 kHz. La fase 5 siguió en `20`.
- **Cero del brazo:** la pose con los tres finales de carrera apretados (foto del 30/10); + = alejarse de cada uno.
  Límites: J1 0–118° (brazo recogido), J2 0–83°, J3 0–101,5°; franja con tabla (04/11, ver arriba). Finales de carrera:
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
