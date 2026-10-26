# Estado de pruebas-esp32 (al 2026-10-26)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Lo próximo

- **`12-j1-base` (J1 montada, en placa):** paso 0 (signos) OK; motor y encoder andan (falta `dg`). PWM en 20 kHz
  (arranca con ~20 %; a 1 kHz con ~10 %, queda como recurso). **Escala en duda:** al ojo 41,5 cuentas por grado en
  vez de 44,44 (¿polea de 84 dientes?): contar dientes o prueba de 180° antes de los pasos 2–5.
- **`13-j2-hombro`, `14-j3-codo`:** a 1 kHz (a 20 kHz J2 no arrancaba con menos de 40 %); resto PROVISORIO (signos
  sin medir, límites ±15°, duty máx 60 %, ganancias de J1, ff 17/8 %, khp 3 / khi 8). Conexionado v0.5 (EN común 23,
  VCC de los IBT-2 a 5 V).
- **`15`/`16` (J3 con un C3, IBT-2 o L298N):** preparadas el 22–23/10, resultados sin anotar.
- **`17-brazo-joystick`:** las tres articulaciones con un joystick en Firefox (lazo abierto, puente
  `tools/joystick.py`, cero al abrir el puente). Compila; puente probado con un puerto simulado; **sin probar en
  placa**. Para fusionar después con la tracción en `recolector_de_frutas`.

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
