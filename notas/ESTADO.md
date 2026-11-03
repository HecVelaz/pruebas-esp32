# Estado de pruebas-esp32 (al 2026-11-03)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`) en un lazo no
  bloqueante a 100 Hz, a 20 kHz; 5a, 5b, 5c y la primera parte de la fase 6 probadas en placa (03/11, **sin el
  resorte de la pinza**). Firmware flasheado: `1c8a0ea` (lo posterior son scripts y notas).
  - `h`: las tres juntas si el brazo está abajo (3,5 s desde ~3°), cero en el flanco; sin cero corta si una avanza
    15° sin su final de carrera (el brazo tiene que estar recogido). `zq q1 q2 q3` para rescatarlo sin cero.
  - `mv`: perfil quíntico, J2 y J3 terminan juntas (recta en (q2, q3)); **J1 con su duración y a ≤ 10 °/s²**
    (`MV_DURACION_PROPIA`, `MV_ACEL_MAX`). Aproximación: J2 y J3 desde abajo (2° a 2 °/s); **J1 0,5° a 5 °/s en el
    sentido en que viene** (invertir hamacaba la punta ±1,4 cm a 1,5 Hz: el encoder de J1 está en el motor y no lo
    ve). La que ya está en `tolPos` no se mueve; si el punto previo sale del margen de 3° de un límite, va directo.
    Feedforward multiplicado por tanh(|w|/0,5 °/s): inversiones del duty 0. No esquiva la caja.
  - Resultado (video y capturas): **sin balanceo de la punta**, `LLEGO` ≤ 0,2° (J1 una vez 0,41° corta), firmes en
    ≤ 0,5 s; deriva 0,000° en 30 s con y sin limón (las tres frenadas).
  - **Repetibilidad articular** (`tools/repetibilidad.py`, 10 ciclos × 5 poses): RP J1 0,33°, J2 0,32°, J3 0,17°.
    J1 cae en dos valores (un salto de stick-slip de 0,2–0,3°); J2 con un sesgo fijo por pose.
  - Joystick `jv` (puente `17/tools/joystick.py --placa 20`) usado en placa; **3 problemas de seguridad de la
    revisión de Codex sin arreglar** (ver `PENDIENTES.md`): usarlo con cuidado.
  - J1 0–90° PROVISORIO; `MV_FACTOR_VEL` 0,5 PROVISORIO (más velocidad: no por ahora, ver la bitácora).
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
