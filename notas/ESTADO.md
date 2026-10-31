# Estado de pruebas-esp32 (al 2026-10-30)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Lo próximo

- **Cero del brazo (2026-10-30):** la pose con los **tres finales de carrera apretados**; + = alejarse de cada uno.
  Límites medidos con el joystick: J1 0–118°, J2 0–97°, J3 0–113° y la franja del cuatro barras
  `q2 − 25° ≤ q3 ≤ q2 + 7,5°`. Finales de carrera: J1 32, **J2 27, J3 33** (cruzados respecto de v0.5).
- **`12-j1-base`: J1 terminada (pasos 0–5 con el cero al frente).** Polea de la base de **84** dientes (41,48
  cuentas/°); posición: ~1,8 s, sobrepico 0, error ≤ 0,31°, igual con el limón. Ya con el cero en el final de
  carrera (+ hacia el mástil, 0–118°): compila, **falta repetir el paso 3**.
- **`19-j2-j3-conjunto` (nueva, compila, sin probar):** J2 y J3 juntas; una activa (`-e j2`/`-e j3`, comandos de
  12–14) y la otra sostenida sola (`o <grados>`); límites con la franja. Falta el paso 6 (movimientos coordinados).
  Reemplaza a `13`/`14` (quedan como historia: 1 kHz, todo provisorio).
- **`17-brazo-joystick`: probado en placa**, también desde el celular (`--host 0.0.0.0`). Finales de carrera, cero
  automático, límites en vivo, "Anotar", franja (`4B`). Sin probar en placa: rampa de bajada de J1, barra de J3 al
  revés y límite de J1 en 118°.
- **`18-finales-de-carrera`:** los tres responden.
- **Robot:** `recolector_de_frutas/firmware/brazo` sigue con la polea de 90, los signos, límites y cero viejos de J1
  y los pines de los finales de carrera de J2/J3 al revés.
- **`15`/`16` (J3 con un C3, IBT-2 o L298N):** preparadas el 22–23/10, resultados sin anotar.

## Máquinas

- Notebook y Pi 5 (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`) ordenadas igual: repos en `~/Documentos` (Pi:
  `~/firmware`, `~/scripts`, `~/logs`), workspaces en `~/ros/`, venvs en `~/venvs/`; las rutas viejas quedan como
  symlinks. Las dos placas están en la Pi: se compila en la notebook y se flashea con esptool desde la Pi (OK).
- Respaldo del firmware micro-ROS del brazo de la WROOM de la Pi: `~/firmware/brazo/respaldo_20261001_183959.bin`
  (en la Pi, restaurado y verificado).

## Sensores en la S3

- **IMU (01, 02): OK.** Es un MPU-6500 (sin magnetómetro: el yaw deriva). `/imu/data_raw` a 50 Hz por micro-ROS
  serial y WiFi, probado con Madgwick y RViz2.
- **GPS UP501 (04, 05): OK por WiFi.** `/gps/fix` con fix al aire libre (primer fix en frío ~2 min, HDOP ~1,4) y
  `/gps/status`. Rollover de semana corregido; tras un arranque en frío la UTC viene ~3 s adelantada ~11 min.
  `-e serial` sin probar. Cables sueltos (se desconecta al mover la placa).

## Motores en lazo abierto

- **06 / 07 (36GP-555 + IBT-2, S3): OK.** K ≈ 1,6 rpm/%, zona muerta ~8–11 %, τ ≈ 60–74 ms, máx ~142–145 rpm;
  rango lineal ~20–85 %. Con carga arranca con 25 % y la zona muerta sube a ~23 %.
- **08 (5840-31ZY + L298N, WROOM): OK.** Modo `Freno` casi lineal entre 40 y 100 % (K ≈ 1,33 rpm/%, τ ≈ 50 ms);
  umbral de arranque sin medir (entre 20 y 40 %).

## Brazo en `10-control-lazo-cerrado` (antes de 12–14)

- **J1 (base, 36GP-555, correa 5:1): terminada.** Cascada P de posición (Kpp 2) → PI de velocidad (Kp 0,549,
  Ki 4): escalones de ±20° en ~1,75 s, sobrepico 0, error ≤ 0,31°, aguanta perturbación y el limón. Rango −45° /
  +135° desde el cero al costado.
- **J3 y J2 con el motor 5840 (sin fin): probadas y descartadas** por los 36-555 de 58 rpm. Con compensación de
  gravedad, dither y `bj` llegaban sin sobrepico, pero bajando temblaban (stick-slip del sin fin con carga a favor).
- **Hombro con el motor de J1 (`-e j2_160_en_j1`): levanta el brazo y lo sostiene** en el ángulo pedido con el
  sostener automático (PI de posición que aprende el duty). Ese motor no se autobloquea: sin corriente el brazo cae.
- `10` ya tiene los pines del robot (v0.4) y los tres motores como 36-555; `cfg_j2`/`cfg_j3` nuevos sin probar.
- Simulaciones (J1 y J3 con el 5840), informe para Overleaf y secciones para Colab en `simulaciones/`.

## Firmware del robot

- El brazo vive en `~/Documentos/recolector_de_frutas/firmware/brazo` (movido el 2026-09-27); sus pendientes están
  en el `CLAUDE.md` de ese repo. Falta pasarle las ganancias de J1 y el cambio de J3 al IBT-2.
