# Bitácora de pruebas-esp32

Una entrada por sesión, con la fecha (la crea y completa `/cerrar`). Las entradas hasta el 2026-10-20 son el antiguo "Estado actual"
de `CLAUDE.md`, movido tal cual el 2026-10-22 (algunas cubren varios días).

## Antes del 2026-09-25 (sin fecha registrada) — IMU y toolchain (00–02)

- **Funciona (probado en placa):** toolchain y flasheo (`00-hola-mundo`). IMU en ROS 2 (`02-imu-ros2`): `/imu/data_raw` a 50,0 Hz por micro-ROS serial y por **WiFi UDP** (también sin USB, con otra fuente), con hora sincronizada con el agente y calibración del bias del gyro OK. Probado con `imu_filter_madgwick` y RViz2 (orientación OK) en la notebook y en la PC de escritorio. `01-imu-mpu9250`: el IMU es un **MPU-6500** (WHO_AM_I `0x70`, **sin magnetómetro**), así que el yaw deriva y no hay `/imu/mag`.

## 2026-09-25 — GPS (04, 05)

- **GPS:** Fastrax UP501 por UART1 a 9600 8N1. **NMEA OK en la S3 (2026-09-25, interiores, sin fix)** con `04 -e uart` por el conector CH343. Si se invierten TX/RX, el GPS queda mudo hasta cortarle la alimentación. `04-gps-up501` (resumen o NMEA crudo por serial) y `05-gps-ros2` (`/gps/fix` NavSatFix + `/gps/status` String a 1 Hz). Driver `lib/UP501` sobre TinyGPSPlus, compartido por `symlink://`. Prueba previa con Arduino en un ESP32 clásico: NMEA OK, sin fix en interiores.
- **GPS en ROS 2 por WiFi OK (2026-09-25):** `/gps/fix` con fix al aire libre, primer fix en frío ~2 min, 4–5 satélites, HDOP ~1,4, posición verificada en el mapa. La protoboard daba falsos contactos: el GPS va conectado directo a la S3. Firmware con un `/gps/fix` por epoch (stamp = hora UTC del GPS) y diagnósticos en `/gps/status`; con `WiFi.setSleep(false)`, 0 pérdidas y 0 reconexiones en 2 min (interiores). **`02-imu-ros2` no desactiva el ahorro de energía del WiFi.** El UP501 tiene el **rollover de semana GPS** (fecha 1024 semanas atrás, corregido en `05`) y, tras cada arranque en frío, **UTC ~3 s adelantada** durante ~11 min (segundos intercalares de fábrica): mientras tanto el stamp es la hora de llegada.

## 2026-09-25 — Motores en lazo abierto (06, 07, 08)

- **Motor en lazo abierto OK en placa (2026-09-25):** `06-motor-36gp555`. Motor 36GP-555 de 12 V, 160 rpm, eje de 8 mm, reductora 50:1 (confirmada por el máximo medido), encoder Hall 16 PPR (x4 por PCNT = 3200 cuentas por vuelta de la salida), placa del encoder **SCX-555** (colores: rojo = Signal B, negro = Signal A, verde = Encoder+, azul = Encoder−, amarillo = Motor−, blanco = Motor+). Driver IBT-2 (BTS7960) con VCC a 3V3, PWM 20 kHz, fuente 12 V / 3 A. Barrido: lineal, **1,61 rpm/%, zona muerta ~11 %, máx 142 rpm**, simétrico. Escalón 50 %: **τ ≈ 74 ms**, 62,6 rpm. Desde parado no arranca con 20 % (sí con 30 %). Script `tools/graficar.py` (barrido y escalón → CSV + PNG en `resultados/`). Protecciones: rampa 50 %/s, corte por atasco, no invierte sin el motor quieto. Revisado por Codex (10 hallazgos, aplicados). **Pendiente:** pull-down de 10 kΩ en R_EN/L_EN (sin ellos: apagar los 12 V antes de flashear o resetear); lazo cerrado (PID con feedforward de la zona muerta). (Ya resuelto: el firmware avisa cuando descarta la línea que corta `n`/barrido/escalón/perfil.)
- **Perfil (`06 -e`, comando `p`, `graficar.py perfil`) OK en placa (2026-09-25, fuente de laboratorio APS3005SI):** 4 perfiles repetibles: **K ≈ 1,58 rpm/%, zona muerta ~8 %, τ ≈ 60–70 ms, máx ~145 rpm**. La APS3005SI con límite de corriente bajo entra en CC y el barrido "cae" desde el 40 % (salta el atasco): límite en 3 A.
- **`07-motor-rampa-vueltas` OK en placa (2026-09-25):** rampa 0 → 100 % por tiempo que termina a N vueltas de salida; por defecto **3 vueltas en 3 s** (segundos = vueltas: sin carga llega a ~99 % en la última vuelta). Firmware propio (comando `r [s] [vueltas]`) con los drivers de 06 por symlink; `tools/rampa.py` (4 gráficos, arranque, duty por vuelta, K y zona muerta compensando τ, `--comparar`). Sin carga: arranca con ~16 %, K ≈ 1,6 rpm/%. Con carga: arranca con 25 %, misma pendiente pero zona muerta ~23 %, −21 rpm a 50 % y máx ~120 rpm, velocidad ondulada. Arriba de ~90 % de PWM la curva salta y se aplana (probable límite de conmutación del BTS7960 a 20 kHz): rango lineal ~20–85 %.
- **`08-motor-5840-l298n` en placa (2026-09-25):** segundo motor, **ESP32-WROOM-32D** (DevKit con CH340, `/dev/ttyUSB0`, `board = esp32dev`). Motor 5840-31ZY (sin fin autobloqueante, 12 V, 160 rpm, 100 kg·cm), driver **L298N** (ENA=GPIO25, IN1=26, IN2=27; jumper de ENA **afuera**; 2 A máx; **9,6 V en el motor a 100 %** con 12 V), encoder externo **38S6G5-B-G24N** en la salida (1000 PPR → 4000 cuentas por vuelta, NPN colector abierto: rojo a 5V/VIN, negro GND, blanco A=GPIO32, verde B=GPIO33, pull-ups internos: no hay de 4,7 kΩ; **cuenta al revés con este acople → `ENCODER_INVERTIDO = true`**). Abrir el puerto no reinicia el ESP32.
  - `lib/L298N` con dos modos de PWM. **`RuedaLibre`** (PWM en ENA): curva **saturada** (40 % → 86 rpm, 100 % → 141 rpm), modelo lineal inválido. **`Freno`** (por defecto: ENA fijo, PWM en IN1/IN2): **casi lineal entre 40 y 100 %** (40 → ~58, 60 → ~87, 80 → ~115, 100 → ~140 rpm), **K ≈ 1,33 rpm/%**, simétrico, **τ ≈ 40–70 ms** (mediana ~50 ms), velocidad estable (1–3 %). Con 20 % no arranca: el umbral real está **entre 20 y 40 %** (el ajuste lineal da una zona muerta negativa porque extrapola la recta de 40–100 %). 2 perfiles buenos (`perfil_20260925_215823`, `_220412`) y uno cortado por atasco con 0 rpm en 20–60 % (`_215340_cortado`, causa no registrada: probablemente sin 12 V).
  - Aplicados los arreglos de la revisión de Codex en 08: Ctrl+C siempre frena, datos guardados si la prueba se corta (`*_cortado.csv`), τ solo con giro estable, mensaje de atasco con el duty real, README con la pérdida real del L298N.

## 2026-09-27 — 09-brazo-control movido a recolector_de_frutas

- **`09-brazo-control` → movido (2026-09-27)** a `~/Documentos/recolector_de_frutas/firmware/brazo`, con copias de `BTS7960`, `L298N` y `EncoderPCNT` en `firmware/lib`. Su estado y pendientes están en el `CLAUDE.md` de ese repo. Los arreglos de los drivers para el robot se hacen allá.

## 2026-10-01 — Flasheo desde la Pi 5 y notebook ordenada

- **Flasheo desde la Pi 5 OK (2026-10-01):** las dos placas están conectadas a la Pi (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`), que no tiene PlatformIO. Se compila en la notebook, se copian los `.bin` por `scp` a `~/firmware/<proyecto>` y se flashea con `~/venvs/esptool/bin/esptool` (S3: `0x0` bootloader, `0x8000` partitions, `0xe000` boot_app0, `0x10000` firmware). `00-hola-mundo` OK en la S3. Tras el flasheo la S3 queda **en modo descarga** (`boot:0x0 ... waiting for download`), y abrir el puerto con pyserial también la manda ahí: arrancarla con `esptool --port <S3> --after watchdog-reset read-mac` o con RST. Al re-enumerar pasa de `ttyACM0` a `ttyACM1`: usar `/dev/serial/by-id/`. Regla udev de ModemManager creada en la Pi (2026-10-01, con `sudo bash ~/scripts/instalar_regla_modemmanager.sh`; la S3 tiene `ID_MM_DEVICE_IGNORE=1`). `00-hola-mundo -e wroom` OK en la WROOM de la Pi (`ttyUSB0`, `ESP32-D0WD-V3`; mensaje con `ESP.getChipModel()`). Esa WROOM tiene el firmware micro-ROS del brazo: antes de flashear otra cosa, apagar los 12 V y respaldar la flash (`esptool --baud 230400 read-flash 0 0x400000 ...`, ~3 min; a 460800 el CH340 pierde datos) y después restaurarla con `write-flash 0x0`. Respaldo: `~/firmware/brazo/respaldo_20261001_183959.bin` (restaurado y verificado). La Pi se ordenó: `~/firmware`, `~/scripts`, `~/logs`, `~/venvs/{esptool,yolo,keepon}` y los workspaces en `~/ros/`. `~/camera_ws`, `~/microros_ws`, `~/yolo_ws`, `~/yolo_env` y `~/keepon_env` quedan como symlinks, porque keepon, vision_estereo, los `install/` y los shebangs de los venvs usan esas rutas.
- **Notebook ordenada como la Pi (2026-10-01):** repos en `~/Documentos` (`pruebas-esp32`, `recolector_de_frutas`, `vision_estereo`, `Keepon pro `), workspaces en `~/ros/` (`microros_ws`, `yolo_ws`) y venv en `~/venvs/yolo`. `~/microros_ws`, `~/yolo_ws`, `~/yolo_env` y `~/vision_estereo` quedan como symlinks (los usan scripts y READMEs de `vision_estereo`, `recolector_de_frutas`, `02-imu-ros2` y Keepon). Verificado: agente micro-ROS, nodos de `yolo_ros` e imports del venv.

## 2026-10-01 a 2026-10-04 — 10-control-lazo-cerrado: J1, J3 (5840) y J2 (5840), simulaciones e informe

- **`10-control-lazo-cerrado` (empezado 2026-10-01):** lazo cerrado de **J1** (36GP-555 + IBT-2) y **J2** (5840-31ZY + L298N) del brazo, **montados en el brazo**, todo en la **WROOM** con los pines del robot (`recolector_de_frutas/docs/conexiones.html`: J1 21/22/23 y encoder 34/35 con pull-up externo; J2 25/26/27 y 32/33; finales de carrera en 36/39). Cascada posición (P, 50 Hz) → velocidad (PI + zona muerta, 100 Hz), como en `ARQUITECTURA.md` §5. Hechos el README (plan de 7 pasos y casillas de estado), `include/pins.h`, `conexiones.html` + `CONEXIONES.md` (cableado, revisado por Codex 2026-10-02: fuente 3 A con J1, **2 A con J2** o fusible de 2 A en el L298N si van juntos; la SCX-555 parece traer pull-ups de 5,1 kΩ (R1/R2 "512"), los 10 kΩ externos en 34/35 quedan como recomendados) y `plan.html` (hoja de ruta con tareas por paso). **Por ahora solo J1** (J2 sin conectar): `plan_j1.html` (pasos A, 0–5, con especificaciones, diseño teórico + simulación vs. real, video y CSV, tomado de la Entrega 6 de 2025 en `docs/entrega6_v2.py`). **Firmware del paso 0 de J1** (`src/main.cpp`, `include/config.h`: comandos `e`, `z`, `p <%> [ms]`, `ie`, `is`, `x`; corte por límite de ±400 cuentas, sin cuentas en 150 ms, reposo previo, `z` obligatorio; revisado dos veces por Codex) **Prueba 0 de J1 OK en placa (2026-10-02):** encoder normal, **sentido INVERTIDO** (`J1_SENTIDO_INVERTIDO = true`, flasheado y verificado: `p 30 300` antihorario, `p -30 300` horario); con 25 % / 100 ms apenas se mueve (13 cuentas), con 30 % bien. **Paso 1 de J1 OK (2026-10-02):** correa GT2 18 → 90 dientes (5:1), **44,44 cuentas por grado de la base**, confirmado a ojo con `m <grados>` (duty fijo, frena al llegar) contra un doblez de 45° (no hay transportador). A 30 % la base va a ~25 °/s (zona muerta en marcha ≈ 17 %); se pasa ~1° al frenar. Cero de J1 = brazo hacia el **costado** del robot. **Decisión del usuario (2026-10-02): rango de J1 −45° (horario) / +135° (antihorario) desde el cero al costado** (no los ±75° de la E4, que eran con otro cero); `ie`/`is` deshabilitados porque invertirían los límites asimétricos. **Paso 2 (PI de velocidad), sin probar en placa:** especificación ts = 0,25 s, sobrepico 0, error 0; diseño por cancelación de polo (Ti = τ) en `tools/diseno_velocidad.py` → **Kp = 0,549 %/(°/s), Ki = 8,44 %/°**, zona muerta 17 % como feedforward, velocidad medida en 4 muestras de 10 ms (con 8 la simulación da 22 % de sobrepico). Firmware: `ev <°/s> [ms]` (+v, −v y 0, CSV con `# fin ok` / `# cortado <causa>`), `kp`, `ki`, `g`; cortes por límite (siempre), sin cuentas 150 ms, sentido contrario 100 ms, atasco. `tools/escalon_velocidad.py` corre el escalón y lo grafica contra la simulación. **Paso 2 de J1 OK (2026-10-02):** el juego de la correa (no está en el modelo) daba 18 % de sobrepico con el diseño; `ev <v> [ms] [ciclos]` con pausa en 0 entre +v y −v: **Kp 0,549, Ki 4 → ts 0,33–0,38 s, sobrepico 4–7 %**; Kalman probado offline y descartado. **Paso de posición de J1 OK (2026-10-02)** (paso 4 del README, 3 de `plan_j1.html`): cascada P de posición 50 Hz → PI de velocidad 100 Hz, **Kpp = 2** (teoría: sin sobrepico con Kpp ≤ 2,78), v_max 30 °/s, a_max 60 °/s², velocidad mínima 3 °/s, llega a ±0,3° y se despierta a ±0,6° (histéresis contra el zumbido), corte "sin cuentas" solo con duty ≥ 25 % (fricción de arranque). 20 escalones de ±20°: **llegada 1,71–1,77 s, sobrepico 0, error 0,27–0,31°, sin zumbido**, coincide con la simulación. Comandos `a <°>`, `ep <A> [ms] [ciclos]`, `kpp`; `ev` vuelve al inicio solo si terminó bien. **Perturbación y carga de J1 OK (2026-10-02):** la base no se mueve a mano (250:1; forzarla puede saltar un diente de la correa); frenada a mano en movimiento: el PI sube el duty 22 → 26 % y recupera en ~0,8 s; con un limón en la pinza, los escalones dan igual que sin carga. Pendiente: video para la Entrega 6; opcional tolerancia 0,1° / 0,3°; pasar ganancias a `recolector_de_frutas/firmware/brazo/include/brazo_config.h`; después J2. Cada flasheo borra el cero: con la base en la marca, `z`. El usuario no conoce mucho de control: explicar cada paso. `docs/` (copia de los documentos del robot) está en `.gitignore`: es solo para leer. **J3 (codo) pasa al IBT-2 (2026-10-03):** 5840-31ZY + encoder 38S6G5, RPWM 16, LPWM 17, EN 13 (+10 kΩ a GND), encoder 18/19 (pull-up interno), PWM 20 kHz; con la fuente directo al motor (12 V, probado en los dos sentidos invirtiendo los cables), mover el brazo pide **1,1 A** como máximo → límite de la fuente en **2 A**; `conexiones_j3.html`, `-e j3` sin L298N (compila, **sin probar en placa**). Duty de `cfg_j3` = los del L298N × 0,8 (provisorios); repetir el paso 0 (sentido, arranque subiendo/bajando, si hace falta el empujón). **Paso 0 con el IBT-2 (2026-10-03), en curso:** desde quieta no arranca con 30-70 % (0 cuentas) y sí con **75-80 %** (fricción estática; con el L298N hacía falta el empujón de 75 %); en `dg` etapa 4 (15 %) se midieron 0,6 V en M+/M− en lugar de ~1,8 V. Duty máx 80 % (temporal). **Límites +70° / −15°** a pedido del usuario (flasheado 2026-10-03; +70° pasa el punto muerto calculado de arriba: verificar midiendo la verde); `cuatro_barras.py --q2 60 --theta 112` (roja a 112° en la marca: el inclinómetro marca 68° medido del otro lado) da los puntos muertos a ~−22° y ~+60° y recomienda −15°/+45°. Con empujón de 80 % (`KICK_PCT`, flasheado): **sentido OK** (positivo = sube, sin `is`); subiendo, 30 y 40 % se clavan tras el empujón y 50 % sigue a ~2 °/s; bajando, −20 % no arranca, −30 % ~18 °/s, −40 % ~32 °/s. Asimetría fuerte por el peso: umbral ~50 % subiendo y ~25 % bajando. **PWM de J3 a 1 kHz (flasheado): rinde ~2-3 veces más que a 20 kHz** (a 20 kHz el IBT-2 entregaba menos: 0,6 V en vez de 1,8 V a 15 %). A 1 kHz: subiendo 30 % OK apenas (empujón), 40 % se clavó, 50 % ~7 °/s; bajando −20 % ~22 °/s, −30 % ~37 °/s; `m 0 80` subió a ~84 °/s. Segunda tanda a 1 kHz: bajando −10 y −15 % no arrancan; subiendo 40 % se clava (dos veces), 45 % ~2 °/s, 60 % ~19 °/s, 70 % ~26 °/s → **umbral subiendo ~44 %, bajando ~18 %**, ~1,1 (°/s)/% subiendo y ~1,5 bajando. Flasheado: feedforward **asimétrico** (`ZONA_MUERTA_PCT` 44 subiendo, `ZONA_MUERTA_NEG_PCT` 18 bajando; J1/J2 simétricos), K 1,3 → Kp 0,73, Ki 12,3 provisorios, `m` por defecto 60 %, atasco a 70 %. Los simuladores de `tools/` todavía usan una sola zona muerta. **Paso 1 de J3 con el IBT-2 OK (2026-10-03):** `m 10 55` de −2,49° a +10,10° (12,6° por el encoder) contra **13°** con el inclinómetro sobre la roja: 34,57 cuentas por grado (piñón 18, engranaje 56) confirmado dentro de ~0,5°. `m` sube con 55 % a ~32-35 °/s y se pasa ~0,1°; para subir hace falta ≥ 44 % (con 20-25 % se clava tras el empujón). **Paso 0 con el IBT-2 (2026-10-03), en curso:** desde quieta no arranca con 30-70 % (0 cuentas) y sí con **75-80 %** (fricción estática; con el L298N hacía falta el empujón de 75 %); en `dg` etapa 4 (15 %) se midieron 0,6 V en M+/M− en lugar de ~1,8 V. **Paso 2 de J3, primer `ev 10 1000 3` (2026-10-03, `# fin ok`, CSV no guardado):** con feedforward fijo 44/18 %, al arrancar se dispara a ~50 °/s subiendo y ~30 °/s bajando (sobrepico 200-400 %) y después oscila trabándose y soltándose (fricción estática > dinámica); en régimen el promedio sí sigue 10 °/s, con duty ~40-46 % subiendo y ~−9 % bajando. Hecho: feedforward de **despegue** (quieta, |w| < 1 °/s: 48 / 18 %) y de **marcha** (33 / 3 %), flasheado; en J1/J2 despegue = marcha. Resultado (`resultados/escalon_vel_20261003_184307`): **bajando mucho mejor** (promedio −10 °/s, sin el pico de −30, oscila ±3 °/s); **subiendo todavía se dispara a 30-46 °/s en los primeros 50 ms** aunque el duty baja enseguida (al despegar, la fricción del sin fin cae: efecto Stribeck), después promedio ~8,3 °/s con duty ~40 %. Flasheado: comando `ff <desp+> <marcha+> <desp-> <marcha->` para cambiar el feedforward en vivo y `--ff` en `escalon_velocidad.py`. Despegue subiendo 42 y 38 % (`_185352`, `_185455`): pico ~23-25 °/s (igual entre los dos), pero oscila todo el escalón (~8 Hz) con Kp 0,73 / Ki 12,3. Con `ff 40 33 18 3`: Kp 0,5 / Ki 5 (`_190957`) oscila menos y llega a ~10 °/s; Ki 3 (`_191028`) queda lento subiendo (5-6 °/s). **Elegido Kp 0,5, Ki 5, despegue subiendo 40 %** (en `config.h`, compila, sin flashear; en la placa se pasan con `--kp/--ki/--ff`). Subiendo la carga cambia con la posición y es repetible (duty para 10 °/s de ~25 % a +3° a ~37 % a +8°). `escalon_posicion.py` ahora acepta `--kp --ki --ff --cero`. **Paso 3 (posición) de J3 OK en placa (2026-10-03, `resultados/escalon_pos_20261003_191345`):** Kpp 2, v_max 15 °/s, a_max 60 °/s², 12 escalones de ±10°: **llegada 1,3-1,7 s** (sim 1,35), **sobrepico 0** (peor 0,14°), **error final ≤ 0,28°**, sin zumbido; al subir desde −10° hay un pico de velocidad de ~50 °/s al despegar que no afecta la posición. Para que sea más suave (flasheado): el despegue ya no es un salto: el feedforward sube en **rampa** (300 %/s) desde el de marcha hasta el de despegue y, una vez que se movió, queda el de marcha hasta el fin del tramo (vuelve a la rampa si se traba 100 ms); en J1/J2 no cambia nada. Flasheado también con Kp 0,5, Ki 5, ff 40/33/18/3 de `config.h`. Con la rampa (`resultados/escalon_pos_20261003_192009`): saltos de duty > 10 % de 142 a **21**, sobrepico 0, error ≤ 0,26°, sin zumbido; llegada algo más lenta (1,4-1,7 s, y **2,1 s bajando a −10°**, que se arrastra al final). Queda el pico de ~53 °/s al subir desde −10°: el duty cae a 2-8 % y la barra igual avanza 3° en 80 ms → es el **juego (backlash) del piñón y el engranaje** (bajando el motor empuja hacia abajo, al invertir cruza el juego sin carga), no fricción. Con `--ff 40 33 18 6` (`_192304`): casi igual (llegada a −10° en 2,05 s en vez de 2,14; error ≤ 0,29°, sobrepico 0); marcha bajando 6 % queda en `config.h` (sin flashear). La bajada se arrastra de −7° a −10° porque ahí la carga cambia: para ~2 °/s hace falta −16 a −20 % (de 0 a −5° alcanzaba con −10 %): el cuatro barras se endurece al acercarse al punto muerto de abajo (~−22°). Video del usuario (`docs/WhatsApp Video 2026-10-03 at 19.25.31.mp4`, prueba `_192528`): **tambaleo al bajar de +10° a 0°**. En el CSV es stick-slip: va a −10/−22 °/s, el P baja el duty a −4/−7 %, se traba 100-150 ms, la rampa de despegue sube a −17 % y se suelta de golpe (−8 a −22 °/s); se repite 3 veces por bajada. Trabas de > 50 ms seguidas de un salto > 8 °/s, por bajada de +10° a 0°: 2-3 con `ff 40 33 18 6` / Kp 0,5 (`_192009`, `_192304`, `_192528`); **1 con `ff 40 33 14 9` / Kp 0,35** (`_193657`, mejor; sobrepico 0, error ≤ 0,32°). Subiendo (0 → +10°) también tambalea: hasta +5° va a 17-20 °/s (pedido 10-14) con ~25 % y a +6,5° se clava ~150 ms y sigue a saltitos hasta +10° con 33-36 %. **Causa de fondo: la carga cambia mucho con el ángulo** (peso a través del cuatro barras), y un feedforward fijo no sirve en todo el rango. Propuesto: medir el duty necesario en función del ángulo con barridos lentos y usar un **feedforward por tabla según el ángulo** (compensación de gravedad). **Rango de trabajo de J3 elegido: −12° a +45°** (límites −15° / +48°, con 3° de margen). Flasheado: comando `bc <desde> <hasta> [°/s]` (barrido a velocidad constante con el PI, CSV como `ev`) y `tools/barrido_carga.py` (corre −12 → +45 → −12 a 4 °/s, guarda `resultados/barrido_*.csv/png` e imprime la mediana del duty por tramos de 3°, subiendo y bajando); `config.h` de J3 con Kp 0,35 y ff 40/33/14/9. **Primer barrido (`resultados/barrido_20261003_194419_cortado`):** subiendo, el duty necesario crece casi **lineal con el ángulo: ~13 % a −4,5°, ~22 % a +1,5°, ~37 % a +10,5°, ~53 % a +19,5°, ~64 % a +28,5°, ~70 % a +31,5°** (≈ 19,5 + 1,45·θ), con dientes de sierra de ~5-8 %; **cortó por atasco a +33,7° con ~80 % y la fuente marcando 3,3 A** (el usuario tenía el límite por encima de 2 A). Al arrancar desde −11,8° saltó a ~50 °/s hasta −7° con el duty cayendo a ~7 % (juego + poca carga abajo). No llegó a medir bajando. **Segundo barrido −12 → +25 → −12 (`resultados/barrido_20261003_195629`, fuente 2 A, llega a 2 A arriba):** subiendo ~23 % a −4,5°, ~33 % a +1,5°, ~47 % a +10,5°, ~62 % a +22,5° (~1,4 %/°, ~10 % más que el primer barrido en todo el rango); **bajando casi constante, −5 a −7 %**, pero con traba–suelta continuo (27 trabas en 37°, velocidad de 0 a −15 °/s): carga que arrastra al sin fin (gravedad a favor) + engranajes impresos secos. Subiendo, dientes de sierra cada ~3,3°. Flasheado: **feedforward con compensación de gravedad** subiendo: marcha 24 % + 1,4 %/°·θ, despegue 31 % + 1,4 %/°·θ (piso 8 %), bajando igual que antes (14/9); `ff` acepta un 5.º número (%/°) y los scripts `--ff` con 4 o 5. **Límite de arriba bajado a +28°** (rango útil −12° / +25°). **Con la compensación de gravedad (`resultados/escalon_pos_20261003_200605`):** el disparo al subir desde −10° bajó de **54 a ~18 °/s**, 0 → +10° llega en **1,48 s** (antes 1,71) siguiendo a la simulación, sin trabas subiendo; sobrepico 0, error ≤ 0,28°, sin zumbido. Bajando sigue el traba–suelta (+10 → 0°: 2-3 trabas por bajada): es mecánico (carga que arrastra al sin fin). Bajando, fila por fila (`_200605`, +10 → 0°): después de soltarse va a −15/−20 °/s (pedido −9/−11), el P baja el duty a ~−6 %, la barra frena y **se clava con −8 a −12 % aplicados**; recién con −14/−20 % (rampa de despegue tras 100 ms) se suelta de golpe y el ciclo se repite (~130-150 ms parada cada vez). A velocidad baja el sin fin con carga a favor necesita más duty para seguir que para ir rápido (efecto Stribeck), así que no hay un duty que dé una bajada lenta y pareja. Flasheado: comando `bj <vmin bajando> <trabada ms> <dither %>` (defaults 3 °/s, 100 ms, 0) y `--bj` en `escalon_posicion.py`, para probar en vivo: llegada más rápida bajando, re-despegue antes y vibración (dither ±% a 25 Hz). Con `--bj 6 40 0` (`resultados/escalon_pos_20261003_202249`): **paradas de ~110-130 ms a ~50-60 ms**; paradas bajando +10 → 0°: de 2-3 a **1**, 0 → −10°: de 6 a **0-2**; llegadas 1,14 s (+10 → 0) y 1,58 s (0 → −10, antes 1,93); subiendo desde −10° más paradas cortas (6-7 de ~54 ms); apareció un sobrepico de 0,49° en una bajada a 0° (vmin 6 bajando). El usuario midió como máximo 4,4 V en el motor con el multímetro: es el promedio del PWM que elige el control (duty máx usado ~45 % → ~5 V), no un tope; el tope es DUTY_MAX 80 % (~9,6 V) y el límite de 2 A de la fuente. Flasheado en J3: **VMAX_POS 25 °/s** (pedido del usuario), vmin bajando 6 °/s y trabada 40 ms como valores por defecto. **Con VMAX 25 °/s y bj 6/40/0 por defecto (`resultados/escalon_pos_20261003_211535` y `_212157`):** 12 escalones de ±10° cada una, **sobrepico 0, error ≤ 0,29°, sin zumbido**; llegadas 0 → +10° en 1,30-1,37 s, +10 → 0° en 1,13-1,15 s, 0 → −10° en 1,86-1,92 s, −10 → 0° en 0,96-1,04 s. Foto del codo en `10-control-lazo-cerrado/imagenes/j3_codo/j3_foto_codo_montado.jpeg`. Pendiente: probar `--bj 6 40 3` (dither), lubricar los engranajes, prueba con el limón, duty máx a ~70 %, resorte o contrapeso, pasar J3 al IBT-2 en `recolector_de_frutas`. **J2 en el IBT-2 de J3 (2026-10-03):** el usuario desconectó J3 y puso el motor y el encoder de J2 en el mismo IBT-2 y los mismos pines (RPWM 16, LPWM 17, EN 13, encoder 18/19). Nuevo entorno **`-e j2ibt`** (`-DARTICULACION_J2 -DJ2_IBT2`, macro `IBT2_EN_PINES_J3` en `main.cpp`): usa `cfg_j2` con el IBT-2 a 1 kHz y apaga los pines del L298N y de J1; flasheado. **Paso 0 de J2 con el IBT-2 (2026-10-03):** bajando se mueve (`p -30 100`: −0,5°, OK, sentido correcto sin `is`); **subiendo no se mueve ni con 70 %: se traba con 3,2 A** (pita). Con la mano aliviando el peso, sube con `p 50 200` → **al motor no le alcanza el par para levantar el brazo completo**; el mecanismo funciona. Hace falta **compensación mecánica en el hombro (resorte o contrapeso)**; pendiente medir con una balanza de mano la fuerza para sostener el brazo y la geometría para dimensionarla. Duty máx de J2 en 80 % (temporal); la fuente tiene que volver a 2 A. `RELACION_ENGRANAJE` de J2 sigue sin medir. Después: `p -50 100` desde la marca bajó 15° (de la salida del sin fin) en 81 ms, cortó por el límite y **siguió cayendo hasta −32,9° después de frenar**; en reposo (driver en rueda libre) **siguió bajando solo** (−32,94 → −33,39°): con el peso del brazo **el sin fin de J2 no se autobloquea del todo**. `p 60 100` desde −33° sí subió 2°. **Postura: el cero de J2 (barra verde en la marca) está a 30° de la horizontal** → `cuatro_barras.py --q2 30 --theta 112`: con J3 quieto J2 puede ir de −19,9° a +42,3° (grados del brazo, con 10° de margen); con J2 quieto la roja de −42,3° a +19,9° (borde ~+30°: explica que J3 se trabara a +34°). Los cálculos anteriores de J3 usaban q2 = 60° por error. **J2: piñón 18, engranaje 56** (igual que J3, 34,57 cuentas por grado del brazo). En `cfg_j2` (flasheado): `RELACION_ENGRANAJE` 56/18, límites **+40° / −18°** (del cálculo de arriba, con J3 en su marca) y `FRENO_EN_REPOSO = true` (al terminar un movimiento queda frenado en vez de rueda libre, para que el brazo no se vaya cayendo; J1 y J3 siguen en rueda libre). **El usuario le bajó peso al brazo y le puso un resorte** que tira del eslabón para alivianar a J2 (2026-10-03): paso 0 de J2 con el resorte (flasheado, grados del brazo): **subiendo no arranca con 30-60 % y sí con 70 %** (1,65° en 100 ms); **bajando no arranca con −15/−20 % y sí con −30 %** (3,1° en 100 ms y **3,5° más después de frenar**); en reposo, frenado, no se cae (`c` igual). `barrido_carga.py` acepta `--ff`. Confirmado por el usuario: **sube con 60-70 % y baja con −30/−40 %**. En `cfg_j2` (compila, sin flashear): despegue 65/32, marcha 50/10 (provisoria), `ARRANQUE_PCT` 30. **Barrido de carga de J2 (`resultados/barrido_20261003_225314`, −10 → +20 → −10°):** subiendo ~59 % a −10°, ~55 % en 0°, ~48 % a +20° (**baja ~0,35 %/°**: más arriba el brazo está más vertical y el resorte ayuda); bajando ~−9/−10 % en todo el rango, con traba–suelta de 0 a −20 °/s (como J3). En `cfg_j2` (flasheado): marcha 52 % − 0,35 %/°·θ subiendo y 9 % bajando, despegue 62/30, Kp 0,35, Ki 5, vmin bajando 6 °/s, trabada 40 ms; `ff` acepta pendiente negativa. **Posición de J2 (`resultados/escalon_pos_20261003_225858`):** 12 escalones de ±10°: llegada 1,1-1,65 s, sobrepico ≤ 0,38°, error ≤ 0,29°, sin zumbido; **bajando tiembla**: 3-6 paradas cortas (~45 ms) por bajada y picos de 28-34 °/s (pedido ≤ 15); subiendo 1-4 paradas y picos ~21-24 °/s. Hasta el 2026-10-04 la simulación de los scripts era siempre la planta de J1 (con la zona muerta del CSV). **Modelo de J3 identificado (2026-10-04, `tools/identificar_j3.py`, sin la placa):** τ 53 ms, K 1,62 (°/s)/%, zona muerta subiendo 23,6 + 1,42·θ %, bajando 3,2 %, despegue +5,9 / −6,1 % (en `PLANTAS` de `diseno_velocidad.py`); los scripts de diseño y de escalones toman `-a j1/j2/j3` (J2 sin modelo: sin simulación) y todos los gráficos de J3 y J2 se rehicieron. Placa contra simulación de J3: subiendo coincide (1,30 contra 1,29 s); de 0 a −10° no (1,92 contra 1,06 s, endurecimiento cerca del punto muerto de abajo). Simulaciones de J1 y J3 (imágenes con nombre por paso) e informe sencillo en `10-control-lazo-cerrado/simulaciones/` (`informe_simulaciones.tex` + zip para Overleaf, explicación en una página; PDF de vista previa con `tools/informe_simulaciones.py`, pandoc + Firefox porque no hay LaTeX). **Secciones del informe para Colab (2026-10-04)** en `10-control-lazo-cerrado/simulaciones/*_colab.md` (caracterización, modelo de la planta, simulación, firmware, validación): Markdown para pegar en las celdas de texto, con imágenes enlazadas desde GitHub (raw), centradas con `<p align="center">` y pie `<b>Fig. N.</b>` (7a–7c, 10–18). El usuario las copia desde el archivo (copiar desde la terminal pierde el formato). **No renombrar imágenes enlazadas** (rompe los enlaces de Colab). Usa J1/J2/J3 en lugar de q1/q2/q3; `firmware_colab.md` todavía tiene q. Imágenes de escalones para el informe en `10-control-lazo-cerrado/imagenes/informe/` (con su README). Pendiente: escalones de velocidad de J2 y J3 con Kp 0,35 / Ki 5; modelo de J2; el CSV de `ep` debería guardar todo el feedforward y `bj` (hoy se pasan con `--ff`/`--bj` al rehacer). Comparación (promedio por escalón; bajando = +10 → 0 y 0 → −10): base `_225858`: paradas 4,3, pico 30 °/s, error de velocidad rms 8,2 °/s; `--ff 62 52 20 13 -0.35` (`_230155`): 3,8 / 24 / 7,2 pero sobrepico de hasta 0,70° bajando; **dither ±3 % (`--bj 6 40 3`, `_230354`): 3,5 / 23 / 6,5, y subiendo paradas 4,7 → 2,5, llegada 1,08-1,33 s, sobrepico ≤ 0,23°, error ≤ 0,26° (la mejor)**. Dither 3 % queda en `cfg_j2` (flasheado). Lo que queda del temblor es mecánico (fricción del sin fin con carga a favor, posible rebote del resorte): lubricar. Con el dither flasheado (`resultados/escalon_pos_20261003_230802`): llegada 1,09-1,44 s, sobrepico ≤ 0,35°, error ≤ 0,35°, sin zumbido (igual que `_230354`). **El temblor al bajar es peor con el resorte**: el brazo colgado del resorte rebota con cada suelta del sin fin. Más pruebas en vivo (promedio por escalón, bajando / subiendo): dither ±5 % (`_231754`): peor (paradas bajando 4,8, pico 27 °/s, sobrepico 0,65°, 5 zumbidos); **Kp 0,2 (`_231844`): bajando mejor (paradas 3,3, pico 23, error de velocidad rms 5,9 °/s, sobrepico 0) pero subiendo peor (rms 8,7, pico 28 °/s, llegadas 0,7-0,9 s muy variables)** → idea: Kp distinto para subir (0,35) y bajar (0,2). Pendiente probar bajar con duty fijo sin PI (`m -8 12`, `m -8 15`, `m -8 20`, volviendo con `m 0 60`): si alguno baja parejo, bajar con duty fijo y corregir solo al final. Mecánico: resorte a gas (amortiguado) en lugar del resorte común, o fricción en el eje del hombro; lubricar. Después: limón, duty máx a ~55 %, límites definitivos. Pendiente: cambiar J3 a IBT-2 en `recolector_de_frutas/firmware/brazo` y sus docs.

## 2026-10-10 — 10 con los pines del robot (conexionado v0.4)

- **`10` con los pines del robot (2026-10-10):** J1, J2 y J3 con el conexionado v0.4 de `recolector_de_frutas`: un IBT-2 cada una, **EN común en el GPIO 23** (una sola 10 kΩ a GND), J2 RPWM 25 / LPWM 26 / encoder 32-33, J3 RPWM 16 / LPWM 17 / encoder 18-19, FC J3 en 27, GPIO 13 libre. Salen el L298N, `-e j2ibt` y el encoder 38S6G5: **los tres motores son 36-555 con encoder SCX-555 a 3V3** (J2/J3 de 58 rpm, 139:1, correa 20 → 100, 240 mm, 54 mm entre centros). `cfg_j2` nuevo para el 36-555 de 58 rpm (123,6 cuentas por grado); **`-e j2_160` = prueba aislada del motor de J1 (36GP-555 160 rpm, 50:1) en el hombro, cableado como J2, solo para ver si levanta el brazo** (`REDUCTORA` 50 → 44,4 cuentas por grado; en los dos: signos sin medir; límites ±15° para el paso 0; duty máx 70 %; PWM 20 kHz; ganancias y zona muerta de J1; `FRENO_EN_REPOSO`; **no se autobloquea: el brazo cae sin corriente**). `cfg_j3`: cuentas del 36-555 de 58 rpm (123,6 por grado), el resto sigue siendo del 5840 (rehacer pasos). `pins.h`, `main.cpp` (pines por articulación, `dg` para las tres, `MOTOR`/`REDUCTORA` en config), `CONEXIONES.md`, `README.md`, `conexiones.html` actualizados; `conexiones_j3.html` queda como registro. Compila (j1, j2, j3), **sin flashear ni probar en placa**.

## 2026-10-13 — Hombro con el motor de J1, en los pines de J1

- **Hombro con el motor de J1, en los pines de J1 (2026-10-13):** entornos **`-e j2_160_en_j1`** (36GP-555 160 rpm de J1 en el hombro como J2, cableado en el IBT-2 y el encoder de J1: RPWM 21, LPWM 22, EN 23, encoder 34/35; macro `J2_EN_PINES_J1`) y `-e j2_en_j1` (el 36-555 de J2 en esos pines, compila, sin probar). Flasheado y **probado en placa**: el motor levanta el brazo; el 36GP-555 **no se autobloquea** (sin corriente el brazo cae). Hecho en el firmware (todas las articulaciones, `SOSTENER_AUTO` solo `true` en `cfg_j2`): comando **`s [°]`** (sostener con informe del duty cada 1 s) y **sostener automático (`sa`)**: el lazo corre en `loop()` entre comando y comando; al llegar pasa a un PI de posición (`khp` 6 %/°, `khi` 15 %/(°·s)) cuyo integrador aprende el duty que sostiene el peso; **no suelta nunca por sí solo** (si se traba, no llega, cae o sale del límite, sostiene donde está; solo `sa`/`x` lo sueltan); `z` mientras sostiene fija el cero y sigue sosteniendo; con el sostener, `a` no frena 300 ms al terminar; fuera del límite `a` deja volver hacia adentro. En `cfg_j2`: límites **±25°** (con ±60° se trababa entre −34° y −41°; punto muerto calculado −19,9° / +42,3°), feedforward bajando **8 %** (con 17 % se pasaba bajando, con 0 % no frenaba subiendo), `ARRANQUE_PCT` 50 (con 25 % saltaba "sin cuentas" a ~1° del objetivo). **Resultado: se mantiene bien en el ángulo pedido.** Al flashear se pierden el cero y el sostener: sostener el brazo, `z`, `a 0`. Pendiente: medir con `bc` el duty según el ángulo y hacer compensación de gravedad en el hombro; la USB se desconectó una vez con un pulso fuerte (revisar masas).

## 2026-10-20 — Pruebas definitivas 12–14 preparadas (conexionado v0.5)

- **Pruebas definitivas de J1, J2 y J3 (2026-10-20, preparadas, sin probar en placa):** `12-j1-base`, `13-j2-hombro`, `14-j3-codo`, con el **conexionado v0.5** de `recolector_de_frutas` (las tres conectadas a la vez, EN común 23; J1 21/22 + encoder 34/35, J2 25/26 + **36/39**, J3 16/17 + 18/19; finales de carrera 32/33/27, no usados). Firmware copiado de `10` con una sola articulación por carpeta (`RELACION_CORREA` → `RELACION_TRANSMISION`, `khp`/`khi` iniciales en `config.h`); `tools/` es un enlace a `10/tools` y los scripts guardan en `resultados/` de la carpeta desde donde se llaman (`Path(__file__).absolute()`). J1: valores de 10, pero `FRENO_EN_REPOSO = true` (con el EN común, rueda libre suelta J2/J3, que no se autobloquean). J2 y J3: **36-555 58 rpm + engranajes helicoidales 15 → 75 (123,6 cuentas por grado)**, 20 kHz, signos sin medir, límites ±15°, duty máx 60 %, ganancias de J1, ff 17/8 %, sostener automático, khp 3 / khi 8: todo PROVISORIO. En `10/tools` el modelo del codo viejo pasó de `j3` a **`j3_5840`** (`-a j3` = J3 nuevo, sin simulación). Compilan las tres. Plan por pasos en cada README (A, 0 signos, 1 cuentas y cuatro barras, 2 barrido de carga, 3 PI, 4 posición, 5 carga, 6 al robot). **VCC de los IBT-2 del brazo a 5 V** (riel de 5 V = VIN del shield, como `recolector_de_frutas/docs/html/placa_wroom.html`; en `10` iba a 3V3), R_IS/L_IS sin conectar; verificado contra esa página (2026-10-20): todos los GPIO coinciden. Con VCC a 5 V el alto de 3,3 V queda justo para un 74HC244 (pide ~3,5 V), pero los IBT-2 de tracción de la S3 andan así (prueba del 20/10): **decisión del usuario, se queda en 5 V**; si un IBT-2 del brazo falla en `dg` o en el paso 0, probar ese con VCC a 3V3.

## 2026-10-22 a 2026-10-23 — Driver de J3 en una ESP32-C3 (15, 16)

- **`15-ibt2-j3-c3` y `16-l298n-j3-c3` (2026-10-22/23, sin commitear hasta el 26/10):** el motor de J3 montado, movido desde una **ESP32-C3-MINI-1** suelta (CH340, entorno `c3_uart`) con pulsos cortos de duty fijo limitados a ±15° por encoder, para descartar el driver: `15` con el IBT-2 de J3 (driver 3, 2 × BTS7960B + 74HC244D), `16` con un **L298N** (mismos pines y comandos; GPIO 4/5 IN1/IN2, 6 ENA, encoder 7/10). Plan y `conexiones.html` en cada carpeta; las tablas de Resultados quedaron vacías (no se anotó lo medido).

## 2026-10-26 — J1: PWM, signos y escala (12); J2/J3 a 1 kHz (13, 14); joystick del brazo (17)

- **Frecuencia del PWM de J1 (`12-j1-base`, 2026-10-26, probado en placa):** pulsos `p <duty> 200` desde el cero, con los 12 V y el brazo armado, cambiando solo `PWM_FREQ_HZ`. **1 kHz:** 5 % no se mueve; 7 % apenas en + (+0,18°, 8 cuentas), en − no; **10 % se mueve en los dos sentidos (+1,22° / −0,79°)**. **20 kHz:** 10 % y 15 % no se mueven (15 %: 2 cuentas); **20 % se mueve en los dos sentidos (+1,19° / −0,95°)**. **A 1 kHz hace falta la mitad del duty para arrancar** (picos de corriente: el período de 1 ms es parecido a L/R del motor; a 20 kHz además el IBT-2 pierde parte de los pulsos cortos en la conmutación). Ese día a 20 kHz arrancó con 20 %, no con los 25–30 % del 22/10 (cuando `DUTY_MAX_PCT` pasó de 40 a 70 %). **Decisión: queda en 20 kHz** (no chilla, zona muerta, arranque y ganancias están medidos a 20 kHz); 1 kHz (o 5 kHz como término medio, sin probar) queda como solución si J1 vuelve a trabarse. Ojo: `pio device monitor` hay que correrlo desde la carpeta de la prueba; desde `~` no lee `platformio.ini` (9600 baudios, sin eco) y parece que no acepta comandos.
- **J1 paso 0 (signos, `12`):** `p 30 200` → +3,44° (153 cuentas), **antihorario**; `p -30 200` → −3,04°, **horario**. Signos de `config.h` correctos. Después de frenar sigue ~0,7–0,9° (inercia + juego de la correa).
- **J1 paso 1 (escala, en duda):** `m 90` (desde −1,28°) llegó en 2,84 s y frenó en +91,33° (se pasa ~1,3° con duty fijo); `m 85` frenó en +83,99° (3733 cuentas) y **al ojo eso son 90° reales** → 41,5 cuentas por grado en vez de 44,44 (−7 %). Coincide con una polea de **84 dientes** en la base en vez de 90. Sin cambiar: falta contar los dientes o la prueba de 180° (`m -42` → `m 126`: si queda una recta, la escala es 41,5; si faltan ~12°, la de 44,44 está bien y el error es del cero o del ojímetro).
- **J2 y J3 a 1 kHz (`13`, `14`, sin commitear de otra sesión del 26/10):** según el comentario de `13/include/config.h`, en el paso 0 a 20 kHz J2 arrancaba recién con 40 % subiendo y bajando (35 % no movía) → `PWM_FREQ_HZ = 1000` en `13` y en `14`. Sin resultados anotados a 1 kHz.
- **`17-brazo-joystick` (nueva, compila, sin probar en placa):** J1, J2 y J3 a la vez desde un joystick en el navegador, en la WROOM con los pines v0.5. **Lazo abierto:** cada eje (−1…1) da el duty entre `dutyMin` y `dutyMax` (J1 18–40 % a 20 kHz, J2/J3 15–50 % a 1 kHz), rampa de subida 200 %/s. Protocolo serie: `J e1 e2 e3` (repetir cada < 300 ms), `Z` cero, `S` frenar; sale `Q q1 q2 q3 d1 d2 d3 est1 est2 est3` a 20 Hz. Soltar el joystick, hombre muerto (300 ms) o `S` = freno (EN alto, duty 0), nunca rueda libre. Límites desde el cero (J1 −45/+135°, J2/J3 ±15°) con el sentido "hacia afuera" aprendido al moverse (funciona con los signos mal; estado `SIGNO`); atasco ≥ 30 % y < 5 cuentas en 500 ms → `TRABA` hasta soltar. Firefox no tiene Web Serial: **`tools/joystick.py`** (pyserial + `http.server`) sirve `tools/joystick.html` en `http://localhost:8000`, abre el puerto sin DTR/RTS y manda `Z` al arrancar (**cero = donde está el brazo al abrir el puente**). Página: joystick redondo J1 (←→) / J2 (↑↓), barra J3, teclado A/D W/S I/K, gamepad, espacio = frenar, escala de 20–100 %. Puente probado con un pty simulado (llegan `S`, `Z`, `J …` y vuelve la telemetría). Idea: fusionarlo en `recolector_de_frutas` con la tracción por joystick.


## 2026-10-30 — J1 terminada en el brazo montado (12): cero al frente, polea de 84, pasos 0–5

- **Cero nuevo (pedido del usuario):** brazo **hacia el frente del robot** (fotos en `12-j1-base/CERO/`). Positivo = alejándose del mástil de la visión, negativo = hacia el mástil. Límites primero +90° / −45°; en el paso 4 `a -40` **chocó con el soporte de la visión a −38,2°** (cortó la protección "sin cuentas") → **−30° / +90°** (`a`/`m` llegan a −27° / +87° por el margen de 3°). Mismos límites y signos en `17-brazo-joystick`.
- **Paso 0 (signos):** con `SENTIDO_INVERTIDO = true`, `p 30 300` iba **hacia el mástil** → `false` (invierte motor y encoder juntos). Después: `p 30 300` → +3,17° alejándose del mástil, motor y encoder de acuerdo. Ojo: `is` en `12` no cambia nada (signos fijos en `config.h`), solo imprime los actuales. Los mensajes del monitor hablan del mástil en vez de horario/antihorario. (El "antihorario" del 26/10 era con otra referencia; la que vale es el mástil.)
- **Paso 1 (escala resuelta): polea de 84 dientes en la base**, no 90. Con la escala vieja (90/18), `m 84 30` frenó en +84,69° (3764 cuentas) y a ojo la base quedó a 90° (frente → costado) = 41,8 cuentas/°. `RELACION_TRANSMISION = 84.0f / 18.0f` → **41,48 cuentas/°** (en `12` y `17`). Verificación: `m 87 30` → +88,45° (3669 cuentas), a ojo 90°.
- **Paso 2 (PI de velocidad, Kp 0,549 Ki 4, ff 17 %), `escalon_vel_20261030_123837`:** ±20 °/s ×10. ts 0,645 / 0,612 s (ida / vuelta, peor 0,78), sobrepico 14,5 %, error final −0,57 °/s; en `10` daba 0,33–0,38 s y 4–7 %. La velocidad sube a ~15 °/s, baja entre 0,1 y 0,2 s (la correa se estira con la inercia del brazo completo) y el integral la pasa a ~22,9 °/s. Las dos primeras corridas (`_123528_cortado`, `_123749_cortado`) se cortaron a los 150 ms: el duty subió de 28 a 39 % sin una cuenta (roce de arranque con el brazo quieto); al reintentar arrancó.
- **Paso 3 (posición, Kpp 2, v_max 30 °/s, a_max 60 °/s²), `escalon_pos_20261030_124228`:** ±20° ×5 desde −1,64°. Llegada 1,81–1,89 s, **sobrepico 0** en los 20 escalones, error ≤ 0,31°, zumbido 0 (como en `10`). Mucho más suave que `m` (duty fijo y frenazo, que rebota y se pasa 0,7–1,45°). Al final, **saltitos de stick-slip** a la velocidad mínima (`VMIN_POS` 3 °/s) antes de entrar en ±0,3°: inofensivos. Mejora posible: `VMIN_POS` 5 °/s o tolerancia 0,5°.
- **Paso 4 (rango):** `a 45` (+44,74°), `a 0`, `a 85` (+84,86°, 4,3 s), `a -27` (−26,88°) con error ≤ 0,26°; `a -35` rechazado.
- **Paso 5 (limón en la pinza), `escalon_pos_20261030_135849`:** llegada 1,77–1,85 s, sobrepico ≤ 0,06°, error ≤ 0,27°, zumbido 0: igual que sin carga, con el mismo stick-slip al final. (`escalon_pos_20261030_134230`: corrida anterior antes de desconectar la placa, sin analizar.)
- **Velocidad:** queda en 30 °/s (decisión del usuario). Con 70 % de duty la base llegaría a ~120 °/s; si se sube, de a poco (45 / 90 °/s²) repitiendo el paso 3.
- **Sin pasar al robot:** `recolector_de_frutas/firmware/brazo/include/brazo_config.h` sigue con relación 5 (44,44 cuentas/°), el factor ×1,2 de las ganancias en rpm (con 84/18 es ×1,286), límites 45° / 225° en el marco de `brazo_ik` y los signos viejos. Antes hay que confirmar que el "frente" de hoy es el +X de `brazo_ik` y dónde queda el mástil (el comentario dice "adelante-derecha"); lo mismo en `cinematica.py` y el URDF.

## 2026-10-30 (tarde y noche) — Joystick por celular, finales de carrera, límites del cuatro barras, J1 con cero en el final de carrera, 19 nueva

- **J2 sin cuentas (13):** `p 40 500` y `p -60 500` daban 5–20 cuentas y el motor no giraba según el encoder. Era **un canal del encoder de J2 desconectado** (con un solo canal el contador va y viene). Reconectado: `p 40 500` → +837 cuentas (+6,77°), motor y encoder de acuerdo.
- **`17` (joystick):** `--host 0.0.0.0` para abrirlo desde el celular (`http://192.168.1.114:8000`; cualquiera en la red mueve el brazo); **tres barras independientes** (J1 horizontal, J2/J3 verticales; la de J3 al revés: arriba = el codo sube); finales de carrera en la telemetría, **cero automático al tocar cada uno**, bloqueo hacia el final de carrera apretado (estado `FC`); límites en vivo ("máx ±10", comando `L`); "Invertir" (comando `I`); **"Anotar" / "Borrar la última"** → `resultados/limites_<fecha>_<hora>.csv`; **rampa de bajada de J1** al soltar (120 %/s; corta de golpe con FRENAR, hombre muerto, límites, FC y atasco); franja del cuatro barras (estado `4B`). Puente probado con un pty simulado.
- **`18-finales-de-carrera` (nueva):** lee los tres (pull-up interno, 3 lecturas a 1 ms, cuenta rebotes) sin mover motores. Los tres responden. Así se vio que **J2 y J3 estaban cruzados: J2 en el GPIO 27 y J3 en el 33** (v0.5 decía al revés) → cambiado en `12`–`14`, `17`, `18` (falta `recolector_de_frutas`). Motores y encoders sí como v0.5.
- **Cero nuevo del brazo:** la pose con los **tres finales de carrera apretados** (fotos en `17-brazo-joystick/LIMITES/ceros_bumpers`, sin commitear: aparecen personas). + = alejarse de cada uno: J1 hacia el frente y el mástil (el final de carrera queda del lado contrario al mástil, cerca del +90° de la mañana), J2 antihorario visto desde atrás, J3 el codo baja. En `17` J3 iba al revés → signos `true/true`.
- **Límites medidos con el joystick:** J1 llegó a 121,8° cerca del soporte de la visión con el brazo recogido (`limites_20261030_221357.csv`) → **118°**. **J2/J3 (`limites_20261030_210733.csv`):** borde "el codo no baja más" (q2; q3) = (−0,6; 11,9) (9,9; 21,2) (20,0; 29,6) (30,0; 40,1) (53,4; 66,5) (79,2; 91,8) (97,3; 112,9) → q3 − q2 entre +9,7 y +15,6; borde "el hombro arrastra a J3" = (30,0; 0,1) (53,4; 20,0) (79,2; 45,1) (97,3; 70,0) → q2 − q3 entre 27,3 y 34,2. **Franja: `q2 − 25° ≤ q3 ≤ q2 + 7,5°`** (peores casos con 2° de margen), J2 ≤ 97°, J3 ≤ 113° (el usuario cortó ahí). Probada en placa: con J3 en 10,1° J2 frena (4B) en 35,8°; con J2 en 35,8° J3 frena en 44,6° (se pasa 0,7–1,3° al frenar). `limites_20261030_205619.csv`: una fila de prueba (q2 56,8°, q3 46,1°).
- **`12` con el cero nuevo (compila, sin probar):** `SENTIDO_INVERTIDO = true` (+ hacia el mástil), límites 0° / 118°, `z` avisa si el final de carrera de J1 (GPIO 32) no está apretado. Las ganancias no cambian. (Un `pinMode` del final de carrera había quedado dentro de `reposo()`; corregido a `setup()` antes de flashear.)
- **`19-j2-j3-conjunto` (nueva, compila `-e j2` y `-e j3`, sin probar en placa):** firmware de `12`–`14` con una articulación **activa** (todos los comandos y scripts de siempre) y la **otra sostenida sola** en una tarea del núcleo 0 (`src/otra.cpp`: referencia a 5 °/s hacia `o <grados>`, duty = uHold + khp·e, no empuja contra su final de carrera, corta por atasco). Límites de la activa = propios ∩ franja alrededor del ángulo pedido de la otra. `z` pone el cero de las dos. `liberar()`: freno en vez de rueda libre mientras la otra sostiene (EN común); `dg` solo con la otra suelta; `x` suelta las dos. Falta: movimientos coordinados (paso 6). Reemplaza a `13`/`14`.

## 2026-10-31 — 19: fase 1, seguridad de la franja

- Plan de seis fases para `19-j2-j3-conjunto` (antes del test de repetibilidad estilo ISO 9283): 1 seguridad de la
  franja, 2 sostener sin oscilación y criterio de llegada común, 3 mapa de gravedad, 4 homing, 5 control conjunto
  (propuesta: carpeta nueva `20`), 6 preparar el test.
- **Fase 1 (compila `-e j2` y `-e j3`, sin probar en placa):** los límites de la activa usaban el destino de la otra,
  que viaja a 5 °/s: con `-e j2`, J2 en 50°, J3 en 30° y `o 55`, J2 quedaba habilitada hasta 80° con J3 cerca de
  30° (q2 − q3 ≈ 50°; el ejemplo del enunciado, J2 en 30° y `o 60`, ya lo rechazaba `o`). Ahora: peor caso entre el medido, la referencia y el destino de la otra (`intervaloOtra()`; con
  la otra sin sostener, el medido, después de `z`); los comandos que mueven la activa esperan a la otra sosteniendo
  la activa (`esperarOtra()`, Enter cancela, 30 s rechaza); `o` valida con el medido y la referencia de la activa;
  `oc` muestra referencia, destino y `EN CAMINO`. `otra::sostener()` ya no llama a `coast()` (bajaba el EN común un
  instante): la tarea pone el PWM en 0 y sube el EN (arregla `enabled_` desfasado después de `x` sin tocar la
  biblioteca); solo la tarea toca el motor de la otra (`parar()` espera a que aplique 0).
- Revisión de Codex (solo lectura) de la fase 1, tres arreglos: `o` valida también con la referencia del sostener de
  la activa de antes del comando (`ejecutar()` lo apaga mientras corre); `parar()` espera a que la tarea termine un
  periodo entero con `activo = false` (si no, la tarea podía volver a subir el EN después de `x`); `x` escrito
  durante la espera se ejecuta (antes solo cancelaba y las dos seguían sosteniendo).
- **Prueba en placa de la fase 1 (`-e j2`):** `z` (los dos apretados) → `o 7`, `a 28` (28,02° en 2,26 s), `o 30`,
  `a 50` (50,21° en 1,92 s); `a 75` rechazado (J3 en 30° → J2 ≤ 52°); `o 52`, `a 48` (48,21°); `a 26` rechazado
  porque J3 todavía estaba en ~31,5° (el límite usó el medido, atrasado 1,5° de la referencia); `o 7` rechazado con J2
  en 48° (J3 ≥ 23°); `x` → `o 23`: J3 vuelve a sostener. Sosteniendo, J2 quedó 0,9–1,4° abajo de lo pedido (49,12°
  con 50°; 46,6° con 48°) y el duty de J3 quieta en ~30° fue de −19,9 a −0,2 % (el integrador va y viene). No se vio
  la espera (los `a` se escribieron con J3 ya llegada). Después de `x`, J2 se quedó en 48° sin caer (la fricción de la
  reductora la sostiene, al menos ahí). El sostener pita (PWM de 1 kHz).
- **J1 y J3 no se movían con la fuente y sí con la batería:** la fuente entraba por el XT60 con un adaptador; con la
  fuente marcando 12 V, J2 (bajando, poca corriente) se movía y J1/J3 no. Con la fuente directo a la bornera del bus,
  se mueven las tres → el adaptador fuente→XT60 tiene un contacto con resistencia. El ATASCO de J3 en la prueba
  anterior (no subía de 7,5° a 0°) era eso. Con la fuente en la bornera, la seta y el fusible quedan afuera: la
  protección es el límite de 3 A y la parada, apagar la fuente.
- **Fase 2 (compila `-e j2` y `-e j3`, sin probar en placa):** banda del integrador `BANDA_INT_HOLD` 0,1° (`bi`) en
  los dos sostener; la otra con el mismo "llegó" que la activa (referencia en el destino, < `TOL_POS`, quieta
  `LLEGADA_MS`, histéresis `TOL_SALIDA`) y `esperarOtra()` espera la llegada real; freno dentro de la banda opcional
  (`fb`, `FRENO_EN_BANDA` = false): al llegar, duty 0 hasta salir de `TOL_SALIDA`; informe `Asentado` a los
  `ASENTAMIENTO_MS` (2 s) sin bloquear; `o` destraba un ATASCO (antes hacía falta `x`); mensajes de rechazo con el
  rango y la franja, con un decimal (antes `Uso:` redondeado: decía +18 y rechazaba 18). Todo PROVISORIO.
- Revisión de Codex de la fase 2, cinco arreglos: la llegada de la otra queda atada al pedido vigente (contador
  `pedido`: un `o` nuevo la anula enseguida, aunque el destino cambie menos de un paso de la referencia); la activa
  usa una llegada **firme** (< `TOL_POS` y quieta `LLEGADA_MS`, histéresis `TOL_SALIDA`) para `fb` y `Asentado`;
  los movimientos con `sa` apagado también piden el informe; si a los 15 s no se asentó, informa `SIN ASENTAR` con
  los mismos datos; `barrido_carga.py`, `escalon_posicion.py` y `escalon_velocidad.py` (en `10/tools`) reconocen los
  rechazos `Fuera de rango` y `>> RECHAZADO`.
- **Prueba en placa de la fase 2 (`-e j2`)**, con `z` hecho con J2 `SUELTO` (~4° arriba de su final de carrera: los
  ángulos de J2 de esta prueba marcan ~4° menos que los reales; hay que rehacer el cero antes de la fase 3). Andan
  `Asentado`, la espera (`a 25` → `Esperando a J3` sin mover J2), los estados `EN CAMINO` / `LLEGANDO` / `LLEGÓ`, `fb`
  (las dos frenadas, sin pitido; congela el error al frenar: J3 con 0,39°) y los mensajes de rechazo. **Banda del
  integrador: con 0,1° J3 oscila ±1° (stick-slip, no termina de llegar); con 0,2° queda a 0,04° con duty fijo →
  `BANDA_INT_HOLD` = 0,2° (PROVISORIO).** J2 en 25° subió sola a 27,1° con el `uHold` aprendido en otra pose (lo
  ataca la fase 3). J2 no puede apretar su final de carrera con `a` (3° de margen sobre el límite de 0°; la otra sí,
  con `o 0`, porque no tiene margen y no empuja con el final de carrera apretado): lo resuelve la fase 4 (homing).
- **Fase 3a (compila `-e j2` y `-e j3`, sin probar en placa):** comando `an [nota]`: con las dos sosteniéndose,
  espera el asentamiento, promedia el duty 1 s (una muestra por periodo, sin dejar de sostener) e imprime
  `MAPA,t_ms,q2,q3,duty2,std2,duty3,std3,uhold2,uhold3,err2,err3,lado2,lado3,estable,nota` (J2 y J3 en ese orden con
  cualquier activa; `MAPA_ERROR,...` si no se asienta en 20 s o con `fb`). El firmware guarda desde qué lado llegó
  cada una. Script nuevo `10-control-lazo-cerrado/tools/mapa_gravedad.py`: `capturar` (14 poses × desde abajo y
  desde arriba con 2° de aproximación; mueve de a una sin salir de la franja, con el margen de 3° de `a` y 0,7° de
  holgura), `ajustar` (por pose, promedio de los dos lados = gravedad, mitad de la diferencia = fricción; modelos
  constante / q propio / plano / `c + a·cos(q2+φ) + b·cos(q3+ψ)` con RMS y validación cruzada; PNG; imprime
  `MAPA_J2`/`MAPA_J3`) y `plan`. Probado sin placa: plan con `-a j2` sin ningún movimiento fuera de las reglas;
  el ajuste recupera un modelo sintético (fricción 3 % y 2 %, RMS 0,2–0,3 %).
- Revisión de Codex de la 3a, cuatro arreglos en `mapa_gravedad.py`: después de cada `o` espera que la otra diga
  `LLEGÓ` (con `oc`; si no, `o 15 → o 13` podía dar la vuelta antes de llegar a 15 y llegar del lado contrario) y el
  firmware calcula el lado de la otra contra su ángulo medido; `ATASCO` y `saca el cuatro barras` cortan la captura
  (antes un aviso `>> J3: ATASCO` pasaba como confirmación del `o`); el ajuste deja afuera las mediciones que se
  movieron durante el promedio y las poses con un solo lado (tienen la fricción adentro), salvo
  `--incluir-inestables` / `--incluir-un-lado`.
- **Abrir el puerto serie reinicia la WROOM (probable):** después de cerrar y volver a abrir `tools/monitor.sh`, `c`
  dio J2 en 0 cuentas, J3 `SIN SOSTENER` y `fb` (que estaba prendido) se prendió de nuevo: arrancó de cero (el brazo
  quedó suelto; en esa pose baja la fricción lo aguantó). `mapa_gravedad.py capturar --cero`: abre sin tocar DTR/RTS,
  espera que termine de arrancar, hace `z` (corta si un final de carrera está suelto) y apaga `fb`; puerto por id
  (la placa estaba en `ttyUSB1` y el script buscaba `ttyUSB0`).
- **Comando `pwm <Hz>`** (500–25000; compila `-e j2` y `-e j3`): cambia en vivo la frecuencia del PWM de las dos
  (`ledcChangeFrequency`; la otra la cambia su tarea), sin tocar `PWM_FREQ_HZ` (1 kHz). Para probar de nuevo 20 kHz
  contra el pitido de 1 kHz: el 26/10 J2 pedía 40 % para arrancar a 20 kHz, pero con la fuente por el adaptador XT60
  con contacto malo. La captura del mapa queda en pausa hasta decidir la frecuencia (el mapa está en % de duty).
- Revisión de Codex de `pwm`, dos arreglos: `ejecutar()` reconoce los movimientos por el comando entero
  (`mueveActiva()` + `dg`), no por la primera letra (`pwm` cambiaba el ángulo del sostener por el actual); `pwm` no
  toca el puente si está deshabilitado (después de `x`, `setDuty(0)` subía el EN común y frenaba las tres).
- **Prueba de PWM (`-e j2`, fuente a la bornera):** a 1 kHz (al arrancar), pulsos de 200 ms con J2 en 20° y J3 en 10°:
  subiendo, 15 % y 20 % no la mueven, **25 % sí** (+0,62°); bajando, 5 % y 10 % no, **15 % sí** (−1,00°). A 1 kHz
  `o 14` y `a 4` terminaron `SIN ASENTAR` (J2 con −0,65° y duty de −11 a +18 %). Con `pwm 20000`: `a 12` → 11,80°
  (error 0,20°), `a 4` → 3,88° (0,12°), `a 20` → 20,02° (0,02°), duty de sostener estable 16-17 %. **Decisión del
  usuario: J2 y J3 a 20 kHz** (`PWM_FREQ_HZ`, `DECISIONES.md`); zona muerta y arranque quedan PROVISORIOS (de 1 kHz)
  hasta medirlos a 20 kHz. J3 a 20 kHz y el arranque a 20 kHz (pulsos), sin medir.
- **Primera captura del mapa (20 kHz), cortada en la 2.ª medición:** (10°, 3°) desde abajo → J2 +27,4 %, J3 −34,0 %
  (a 1 kHz, en poses parecidas, ~17 % y ~−23 %: a 20 kHz el IBT-2 rinde menos por % de duty). Después `a 12` se cortó
  por "sin cuentas": la cascada arrancaba desde 0 con el feedforward de 1 kHz (17 %), por debajo del 27 % que ya
  pedía sostener J2, y a los 150 ms con más de `ARRANQUE_PCT` (35 %) sin moverse cortó. CSV
  `resultados/mapa_gravedad_20261031_131027_cortado.csv` (1 medición).
- **Arreglos (compila `-e j2` y `-e j3`, sin probar en placa):** arranque sin salto (si el comando empieza con la
  activa sosteniéndose, la primera cascada arranca desde el duty que la sostenía: integral = uHold − feedforward de
  marcha del sentido; `p` y `ev` consumen la marca); `p` negativo con el final de carrera de la activa suelto puede
  pasar de 0° hasta apretarlo (`HASTA_FC_MAX_GRADOS` = 10° más abajo como mucho, la franja vale igual) y avisa
  `FINAL DE CARRERA APRETADO` → `z`. Falta: medir el arranque a 20 kHz y subir zona muerta, `ARRANQUE_PCT`, atasco y
  duty máximo antes de capturar el mapa.
- Revisión de Codex del arranque sin salto y de `p` hasta el final de carrera, cinco arreglos: el límite de abajo
  extendido se recalcula en cada vuelta (la otra puede moverse) y también corta arriba; con el final de carrera ya
  apretado `p` negativo no empuja (avisa); la precarga descuenta también el proporcional del primer periodo
  (`kp·a_max·Ts_pos`); el sostener debajo de 0° no corre la referencia hacia donde cae y suelta (freno) si pasa el
  límite extendido; el corte en el límite extendido informa ese límite (no 0°).
- **Prueba a 20 kHz (`-e j2`):** pulsos de 200 ms con J2 en 20° y J3 en 10°: subiendo, 20 % y 25 % no, **35 % sí**
  (+1,05°); bajando, **−15 % no** (0 cuentas). `a 20` desde 21° y `a 12` llegan (arranque sin salto). **J3 a 20 kHz va
  y viene**: pidiendo 10° queda entre 9,4° y 11,8° y no termina de llegar (`Esperando a J3` hasta cancelar,
  `SIN ASENTAR`); el duty con que sostiene va de +13 % (en 10°) a −33 % (en 14°): la franja de duty en que la
  fricción no la deja moverse es mucho más ancha que a 1 kHz y el integrador la cruza de a saltos. **Decisión del
  usuario: seguir con 20 kHz y ajustar; si no anda, 5 kHz con `fb` (ruido al moverse, silencio quieta).**
- **Compensación de fricción al sostener (compila `-e j2` y `-e j3`):** las dos suman
  `FR·clamp(error / 0,3°, −1, 1)` al duty (`FR_SOSTENER` = 0, apagada; en vivo `fr` y `ofr`). Umbrales para 20 kHz
  (PROVISORIOS): `ARRANQUE_PCT` 35 → 45 %, `ATASCO_DUTY_PCT` 50 → 60 %, `DUTY_MAX_PCT` 60 → 70 %, la otra: duty
  máximo 50 → 60 % y atasco 45 → 55 %.
- **Ajuste a 20 kHz en placa (`-e j2`, J2 en 20°):** J3 de 14° a 10°: con `ofr 0` se pasa hasta 8,53° y queda en
  −0,20°; `ofr 10` +0,22°; **`ofr 15` no se pasa (10,16 → 9,91°), +0,09°, `LLEGÓ` enseguida**; `ofr 20` en 14°,
  −0,24°. J2 no necesita compensación (`a 25` / `a 20`: +0,19° / −0,05°, duty ~0 %: la fricción la sostiene). Con
  `fb`: las dos `frenada` (duty 0) y **sin ruido**. Quedan `FR_SOSTENER` J3 15 %, J2 0 % (por articulación,
  PROVISORIO). **20 kHz se queda** (no hace falta 5 kHz).
- **J3 no "llegaba" a 0°:** su final de carrera se aprieta en ~0,6° y la otra no empuja hacia él apretado → error
  0,6° > `TOL_POS`, `LLEGANDO` para siempre y la activa esperando 30 s. Arreglo: con el final de carrera apretado y
  el destino más allá, el error de llegada cuenta 0.
- **Captura del mapa de gravedad a 20 kHz (`-e j2`, `ofr 15`): 27 de 28** (falta (90°, 93°) desde arriba; ninguna
  inestable), `resultados/mapa_gravedad_20261031_135400.csv/.png`. J2: +20 a +25 % con q2 bajo, ~0 % en q2 ≈ 50°,
  −15 a −25 % en q2 = 70–90° (el peso pasa del otro lado del eje del hombro, cambia de signo cerca de 54°). J3: ~−17 %
  sin tendencia clara. **La fricción pesa como la gravedad:** desde abajo y desde arriba el duty cambia hasta ±7 % y
  en varias poses J2 sostiene con el mismo duty (+2,02 %: integrador quieto dentro de la banda; a 20 kHz y con la
  reductora ~695:1 la fricción sola sostiene el brazo). Validación cruzada: J2 solo q2 8,6 % (mejor), plano 8,9 %,
  cos q2 + cos q3 10,1 % (sobreajusta; extrapola a −50 % en las esquinas); J3 plano 6,2 %, constante 7,2 %. Modelos
  simples: J2 `u ≈ 31,4 − 0,58·q2`, J3 ≈ −17 % (constante). `mapa_gravedad_20261031_123942_cortado.csv`: 4
  mediciones a 1 kHz (captura cortada).
- **J3 iba y venía en su final de carrera** al terminar la captura (vuelta a (5°, 0°)): con el final de carrera
  apretado la otra cortaba todo el empuje, también el que sostiene el peso; el codo caía, se soltaba, volvía a subir.
  Arreglo: apretado y con la referencia más allá, sostiene con `uHold` sin P ni I hacia él y nunca empuja más que eso.
  Además `o` puede ir hasta 10° debajo de 0° con el final de carrera suelto (`o -5` lo busca y se queda ahí).
- Los finales de carrera se aprietan ~2° antes del cero anterior (J2 en 1,9°, J3 en 0,6–1,7°): el cero hecho con el
  brazo apoyado a fondo no es repetible; la fase 4 (homing) pone el cero en el flanco.
- Revisión de Codex del arreglo, dos más: apoyada en el final de carrera empuja hacia él como mucho
  `OTRA_FC_SOSTEN_MAX` = 30 % (el `uHold` pudo crecer viajando hasta −60 %; PROVISORIO) y el freno dentro de la banda
  no actúa ahí (con `fb` volvía el ir y venir).
- **Fase 3b mínima (compila `-e j2` y `-e j3`, sin probar en placa):** el sostener de las dos suma un modelo según la
  pose, `u0 = SOST_C + SOST_K2·q2 + SOST_K3·q3` (J2 `31,4 − 0,58·q2`, J3 −16,8 % constante; del mapa del 31/10,
  PROVISORIO) y `uHold` aprende solo la diferencia (antes se arrastraba el duty de la pose anterior). Solo al
  sostener: la cascada no lo usa (el modelo con cosenos sobreajustaba). `gm` lo prende/apaga sin salto.
- Revisión de Codex de la 3b, cuatro arreglos: `gm` no pasa por el corte del sostener (antes, un instante de freno);
  con la otra parada, `gm` no deja una compensación fantasma; al empezar a sostener, el modelo de la otra entra en
  rampa de 1 s (antes de golpe: J3 −16,8 %, J2 +31,4 %); `z` sosteniendo ajusta el residuo al cambio del modelo (en
  J2 desde 50° eran 29 puntos de más), también en la otra.
- **Prueba F3b (`-e j2`, `gm` prendido): el modelo empeora el sostener de J2.** J2 20° (J3 0°): el modelo dice
  +19,8 %, se asienta con −11,6 %; 31°: +13,4 % → −17,5 %; 50° (J3 30°): `SIN ASENTAR`, J2 cae 1,2° (48,78°) con
  −13,1 %. El modelo empuja hacia arriba, el integrador lo compensa y ese residuo se arrastra a la pose siguiente. A
  20 kHz la fricción sostiene J2 en todo un rango de duty (~−15 a +25 %): el "duty que sostiene" no es la gravedad.
  Propuesta: `MODELO_SOSTEN` = false y `FRENO_EN_BANDA` = true por defecto (sin decidir todavía).
- **Ceros corridos (2026-10-31, tarde):** el usuario movió el final de carrera de J3 (se aprieta ~9° debajo del cero
  viejo, ±1°: `o -9`) y, bajando J2 con pulsos hasta su final de carrera haciendo `z` entre pulso y pulso, J2 bajó
  26,1° desde ~4,0° → **su final de carrera se aprieta ~30° debajo del cero viejo** (no recuerda haberlo movido).
  Con el cero nuevo, la franja de `config.h` (medida desde el cero viejo y solo con J2 ≥ 0°) ya no vale: pasada al
  cero nuevo sería ~`q2 − 46° ≤ q3 ≤ q2 − 13,6°` y la pose del cero nuevo queda fuera de lo medido. Hay que revisar el
  final de carrera de J2 y volver a medir la franja antes de mover el brazo a poses altas.
- **Causa de los ceros corridos (usuario):** los finales de carrera tienen un solo punto de apoyo y el golpe al
  llegar rápido (pulsos de 25 %, joystick) los hace girar. La pose del cero nuevo es la misma de la foto del 30/10:
  el brazo y los encoders estaban bien, se movían los finales de carrera. Con este cero, la franja de `config.h`
  vuelve a valer. Pendiente mecánico: fijarlos con un segundo punto de apoyo.
- **Fase 4, homing `h` (compila `-e j2` y `-e j3`, sin probar en placa):** primero J3 (codo) y después J2 (hombro);
  cada una, si está apretada, se aleja hasta soltarlo y 2° más, después baja a 1 °/s hasta el flanco (la activa con el
  lazo de velocidad y el duty limitado a 50 %, mirando el final de carrera cada ~0,2 ms; la otra, su tarea mueve la
  referencia y guarda el contador al apretarse), cero en el flanco, retrocede 3° y queda sosteniendo. Informa cuánto
  se corrió cada flanco respecto del cero anterior (`HOMING,t,dq2,dq3,con_cero`). Con cero: si J2 está más arriba de
  20° no arranca. La otra busca su final de carrera con `o` negativo a
  2 °/s. Por defecto: `FRENO_EN_BANDA` = true, `MODELO_SOSTEN` = false. No limité `p` hacia el final de carrera a 15 %
  (a 20 kHz J2 no baja con 15 %): para el cero, usar `h`.
- Revisión de Codex de la fase 4, siete arreglos: sin acercamiento rápido (si el flanco se corrió, a velocidad
  normal podía golpearlo; busca a 1 °/s desde donde esté); `irAControlado()` devuelve el corte y el homing lo
  respeta; al empezar, las dos se sostienen donde están (con `-e j3` J2 quedaba suelta); cancelar lo hace la tarea
  (carrera con el estado del homing); con Enter la otra se queda donde está; con el cero conocido, corta si
  activa − otra sale de la franja; `ceroFijado` recién cuando terminan las dos.
- **Prueba F4 (homing, `-e j2`, 20 kHz):** sin cero, `h` desde el arranque: J3 encontró su flanco 8,6° más arriba
  de donde había caído al reiniciar y J2 0,45° (estaba apoyada más allá del flanco). Después, 5 veces `h` seguidas
  (flanco respecto del anterior): **J3 −0,03 / −0,06 / −0,02 / 0,00° (σ ≈ 0,02°)**; **J2 +0,32 / −0,16 / +0,11 /
  +0,15° (σ ≈ 0,20°)**; la 4.ª se cortó en el retroceso de J2 ("sin cuentas": subir a 20 kHz pide 35 % y la cascada
  arrancaba con el feedforward de 1 kHz, 17 %). J3 a 1 °/s ya no mueve su final de carrera.
- **J2 a 20 kHz:** feedforward despegue subiendo 35 % (medido), bajando 22 %, marcha 25 % / 8 % (PROVISORIOS) y
  homing de J2 a 0,5 °/s (`HOMING_VEL_BUSCAR` por articulación; J3 sigue en 1 °/s). J1: va con la fase 5 (`20`); el
  test de repetibilidad, primero en el plano de J2 y J3 con J1 frenada en su cero.
- **Homing con J2 a 0,5 °/s y feedforward de 20 kHz (`-e j2`):** flanco respecto del anterior, 4 veces (la última
  después de mover J2 a 20°, 10°, 18°, 20°): **J2 0,00 / −0,01 / +0,15 / −0,09° (σ ≈ 0,10°, antes 0,20°)**; J3 −0,08 /
  +0,02 / +0,05 / −0,05° (σ ≈ 0,06°). Repetibilidad de cada flanco ≈ σ/√2: ~0,07° J2, ~0,04° J3. Sin cortes en el
  retroceso. `a 20` / `a 10` / `a 18` / `a 20` llegan sin cortarse (error ≤ 0,3°) y quedan frenadas (`fb`).

## 2026-10-31 (noche) — 20-brazo-coordinado: fase 5a

- Plan de la fase 5 aprobado: carpeta nueva `20-brazo-coordinado` con J1 (y su homing); llegada siempre con el ángulo
  creciendo (+) en las tres; J1 0–90° PROVISORIO (estirado choca antes con el soporte de la visión, sin medir); la
  zona prohibida (que el brazo no choque con la caja) queda para después.
- **5a (compila, sin probar en placa):** `lib/Articulacion` (sin `Serial` ni pines fijos): el mismo controlador para
  las tres (cascada, PI de velocidad con feedforward y rampa de despegue, sostener con banda del integrador,
  compensación de fricción y freno en banda, homing en el flanco, protecciones), portado de `12`/`19`. `src/main.cpp`:
  tarea de control a 100 Hz que ejecuta los comandos (cola desde `loop()`) y deja los mensajes en otra cola; homing
  J3 → J2 → J1; `j <n> <°>` dentro de los límites y la franja (3° de margen); la franja se vigila en cada periodo.
  J3 en cascada con feedforward PROVISORIO (en 19 se sostenía con su lazo simple; `ff 3 ...` en vivo).
- Revisión de Codex de la 5a, seis arreglos: el primer periodo de `sostener()` ya corre el P de posición (si no,
  `pasoPI(0)` borraba la integral precargada: se perdía el arranque sin salto); límites propios mientras va al
  destino; la franja se vigila también sosteniendo (una vez por episodio); `j` frena a las otras si iban a otro lado;
  `h` deja todo quieto antes de empezar; `x` baja el EN desde `loop()` sin depender de la cola (que pasó a 16).
- **Primera prueba de la 5a:** J3 homing bien (flanco +0,016° y −0,049° respecto del anterior). J2 encontró su
  flanco pero no terminó de subir a 3° en 15 s: el arranque sin salto partía del duty que tenía (0 %, frenada en el
  flanco) y anulaba el feedforward (35 %); el integrador tardaba. Arreglo: la integral precargada nunca va contra el
  sentido del movimiento (arranca al menos con el feedforward, como 19). En el 3.er `h`, J2 bajó ~5° debajo de su
  flanco anterior sin apretar el final de carrera (−4,92°, q3 − q2 = 8° fuera de la franja; los `h` siguientes se
  cortan enseguida, bien): probablemente el final de carrera de J2 se movió otra vez (apoyada varios segundos).
- Homing de J2 apoyada en su final de carrera: no se pudo alejar (subió 1,5° y "no avanza con el duty al tope"): el
  homing tenía el duty limitado a 50 % y levantar J2 desde abajo a 20 kHz pide más (~25 % para sostenerla + el
  despegue). `homingDutyMax` de J2 → 65 % (PROVISORIO).
- Retroceso de J2 después del flanco (0 → 3°) se pasó a 5,03° y no quedaba firme: el despegue de 35 % a 20 kHz la
  lanza en un tramo corto y la cascada (que mira la llegada cada 20 ms con ±0,3°) podía cruzar el destino sin verlo
  y volver. Arreglo: cruzar el destino (el error cambia de signo) cuenta como llegada; el sostener corrige el pasado.
  `j 2 5` con la cascada (70 %) subió J2 sin problema.
- Retroceso de J2 todavía sin quedar firme: quedaba en 5–6° (pedido 3°), duty 3–9 %, quieta: el sostener no la
  empujaba hacia abajo porque **el final de carrera de J2 seguía apretado** varios grados arriba del flanco (se
  arrastra / se traba; mecánico). Arreglo: "apoyada en el final de carrera" solo cerca del cero (< 1° sobre el límite
  de abajo). Homing en dos velocidades: con cero previo, rápido (J2 y J3 2 °/s, J1 3 °/s) hasta 1,5° del cero
  anterior y después a la velocidad lenta (el flanco siempre se toma despacio).
- **Corrección:** no era el final de carrera (el usuario lo revisó: no se traba). Con la protección ya limitada al
  cero, J2 seguía en 6,1° (pedido 3°) empujando −13,6 % sin moverse: la cascada la daba por "llegada" al pasar por
  3° todavía rápido (y la regla de "cruzar" lo empeoraba), seguía de largo y el sostener —hecho para décimas— no la
  bajaba 3° contra la fricción de 20 kHz. Arreglo (como 19): "llegó" exige < 0,3° **y** < 2 °/s; si después de
  llegar se aleja más de 1,5°, vuelve a la cascada. Se saca la regla de "cruzar".
- **Homing de las tres anda (20, 5a):** flanco respecto del anterior, 6 vueltas: **J3 σ ≈ 0,013°, J2 σ ≈ 0,014°**
  (antes 0,10°; ahora con la llegada arreglada y la búsqueda en dos velocidades), **J1 σ ≈ 0,03°** (3 vueltas): del
  orden de 1–2 cuentas del encoder. En 3 vueltas J1 no quedó firme después del retroceso (iba y venía alrededor de
  3°: el PI de posición del sostener empuja contra el juego de la correa; J1 no carga peso y se autobloquea).
  Arreglo: `autobloqueo` en la configuración (J1): al llegar, freno (duty 0), como 12; firme con `tolSalida`.
- Revisión completa de Codex de la `20` (HEAD `00fde16`), cinco arreglos: con autobloqueo vuelve a la cascada desde
  `tolSalida` (entre 0,6° y 1,5° quedaba frenada para siempre); si el final de carrera se aprieta en la etapa rápida
  del homing (se corrió), el flanco no se acepta: se aleja y lo busca de nuevo despacio; el corte por la franja frena
  (duty 0) en vez de seguir con el duty del movimiento; mientras siga fuera de la franja frena cualquier movimiento de
  J2/J3 y `h` no arranca; `x` descarta los comandos que estaban en la cola (podían volver a subir el EN).
- **5a probada en placa (`a89d415`):** homing de las tres, 4 vueltas con cero previo, flanco respecto del anterior:
  **J1 0,000° (4/4; 1 cuenta = 0,024°), J2 ±0,008°, J3 ±0,010° (1 cuenta = 0,008°)**: repite al nivel de una cuenta
  del encoder (a la mañana J2 daba ±0,20°). `j` de a una: J2 15° (14,88), J3 10° (10,23; se pasó ~1° y volvió, con
  el feedforward PROVISORIO), J1 30° (29,77) y 3° (3,13), J3 3° (3,09), J2 3° (2,75): firmes y frenadas, errores
  ≤ 0,25°. Se ve el acople: J2 cambia su duty para sostenerse mientras J3 se mueve.
- **5b (compila, sin probar en placa):** `mv q1 q2 q3`: las tres siguen el mismo perfil quíntico normalizado
  (terminan juntas; J2 y J3 al mismo ritmo → recta en (q2, q3) dentro de la franja); duración según vMax/aMax de cada
  una × `MV_FACTOR_VEL` (0,5, PROVISORIO), mínimo 1 s; llegada siempre con el ángulo creciendo (las que bajan terminan
  2° por debajo y suben a 2 °/s); después sostienen el objetivo hasta quedar firmes y a los 2 s: `LLEGO,t,q1,q2,q3,
  e1,e2,e3`. Corta con error de seguimiento > 5°, franja o cualquier comando. No esquiva la caja (pendiente).
- Revisión de Codex de la 5b, tres arreglos: al cortar un `mv` todas se quedan donde están (las que ya sostenían el
  objetivo seguían yendo a él); antes de subir el último tramo, las que bajan tienen que estar de verdad por debajo
  del objetivo (espera hasta 3 s en el punto previo; si no, podía terminar llegando desde arriba); el error de
  seguimiento (> 5°) se controla en las tres, también las quietas.
- **Cierre del 31/10:** 32 commits (`b21854b` … `b5a0763`), todos con revisión de Codex (solo lectura) salvo los de
  constantes, notas y arreglos de una línea. Revisión de Codex sin trabarse: `codex exec --sandbox read-only -o
  <salida.md> "..." < /dev/null` (sin el `< /dev/null` se queda esperando la entrada). Útiles: `h` (homing de las
  tres en `20`), `c`, `j <n> <°>`, `mv <q1> <q2> <q3>`; para volver J2 a su final de carrera en `19`, `p -25 200`
  repetido; en `19`, `o -5` busca el de la otra. Abrir el puerto serie reinicia la WROOM: después de abrir el monitor,
  `h` (en `20`) o `z` con el brazo en los finales de carrera (en `19`).

## 2026-10-31 (tarde, segunda sesión) — 20: 5b probada en placa, telemetría (5c), llegadas sin caída

- **5b probada en placa (`79bb390`):** los `mv` arrancan y terminan juntos; las que bajan se pasan ~2° y vuelven
  subiendo despacio. Serie `mv 20 15 10` → `3 3 3` → `40 30 20` → `20 20 24` → `3 3 3`: errores ≤ 0,46° (J3), el resto
  ≤ 0,32°. Cualquier comando corta el `mv`: esperar siempre el `LLEGO` antes de mandar el siguiente.
- **Repetir `mv 20 20 24` desde poses distintas** (adelanto de la fase 6). Con `tolPos` 0,3° en las tres: rango J1
  0,09°, J2 0,06°, **J3 0,30°** (cae en cualquier lugar de la tolerancia: al quedar firme frena y no corrige más).
  Ajustes, de a uno:
  - J2 y J3 `tolPos` 0,15° / `bandaInt` 0,1°: J3 0,03° (mejor), J2 0,29° (peor) y una vez `no quedan firmes`.
  - J2 vuelve a 0,3° / 0,2°, J3 queda en 0,15° / 0,1°: 4 llegadas, J1 0,00°, J2 0,22°, **J3 0,008° (una cuenta)**.
  - `khi` 4 en J2/J3 (en vez de 8) para que despeguen más suave: peor (3 de 7 `mv` con `no quedan firmes`, J2 con
    0,30° de error) → vuelve a 8.
  - Al final de cada `mv` se veía un tambaleo de la punta (video del usuario, no versionado: aparece una persona).
- **5c, telemetría (`3c08116`):** `t [1|0]` imprime a 50 Hz `T,ms,etapa,` y por articulación `q,ref,duty,w,estado`,
  por su propia cola (no tapa los `LLEGO`; `TELE_FIN,<perdidas>`), y no corta el `mv` ni el homing.
  `tools/trayectoria.py h "mv ..." ...` reemplaza al monitor: manda los comandos esperando cada `LLEGO`, guarda
  `resultados/trayectoria_*.csv` y `.png` (referencia contra medido, error con `tolPos`/`bandaInt`, duty, camino
  (q2, q3) sobre la franja) e imprime por `mv` el tiempo hasta quedar firmes y las correcciones del final. Primera
  captura (`trayectoria_20261031_175424`, 6990 muestras, ninguna perdida):
  1. **El tambaleo era J2 cayéndose 1,7° al llegar:** subiendo directo llegaba adelantada y frenando con −20 %; el
     sostener arrancaba de ese duty y la gravedad la bajaba hasta que el integrador juntaba +22 %.
  2. En la subida final a 2 °/s, stick-slip: J2 hasta 1,4° y J3 0,9° atrás de la referencia hasta despegar.
  3. `mv 3 3 3`: J3 trabada justo en el borde de `tolPos` 0,15°, 18,5 s hasta quedar firme (el corte es a los 20 s).
  Feedforward medido en el recorrido (ajuste duty = a + b·w): J2 +24,4 / −10 %, J3 +11,9 / −33 %, como `config.h`
  (25 / 8, 10 / 30): no era eso.
- **El sostener arranca del duty que sostiene el peso (`75b1b29`):** `uHold = (ffMarchaPos − ffMarchaNeg) / 2`
  (J2 +8,5 %, J3 −10 %, J1 0) en vez del duty con que venía. J2 ya no cae (error máx al final del primer `mv` 1,68 →
  0,40°); de la llegada a firmes 0,3–0,4 s en los cuatro (antes hasta 18,5 s); 4 `mv` en 28 s (antes 48); a la vista,
  menos tambaleo. Subiendo directo J2 seguía terminando 0,25–0,27° arriba (adelantada al frenar, dentro de `tolPos`).
- **Todas las que se mueven llegan con la aproximación** (commit del cierre; captura
  `trayectoria_20261031_180843`): J2 en el primer `mv 20 20 24` 20,00 (antes 20,27); dos llegadas a `20 20 24`: J1
  0,00°, J2 0,008°, J3 0,04° de diferencia; firmes en 0,2–0,5 s; 4 `mv` en 30 s. Queda J3 pasándose en la subida a
  2 °/s (error máx 0,46° y 0,55°; `mv 3 3 3` termina 0,18° arriba): siguiente, `VEL_APROX` 2 → 3 °/s.
- Se habló de un perfil trapezoidal: el quíntico de `mv` ya es más suave (aceleración sin saltos); el tambaleo no
  venía del perfil sino del paso al sostener y del stick-slip. Si hace falta, `MV_FACTOR_VEL` 0,5 → 0,3.
- Commits: `79bb390`, `3c08116`, `75b1b29` (sin revisión de Codex en esta sesión). Útil: abrir el puerto reinicia
  la WROOM, por eso la lista de `trayectoria.py` empieza con `h`; `--archivo` vuelve a graficar sin la placa.

## 2026-11-02 — 20: revisión de Codex de la sesión del 31/10 (sin placa)

- Codex (`--sandbox read-only`) sobre `20-brazo-coordinado`, mirando `79bb390..36708bf`. Tres problemas, verificados:
  1. **Alta:** `ff` aceptaba `nan` (`x < 0 || x > 70` da falso con NaN). Con `uHold = (ffMarchaPos − ffMarchaNeg) / 2`,
     el sostener de J2 quedaba en NaN al llegar. **Arreglado:** `!(x >= 0 && x <= 70)`.
  2. **Media:** `mv.aprox` se decide con 0,05°, menos que `tolPos`: una que ya está firme en su objetivo (p. ej. J1 en
     19,94° con objetivo 20°) igual baja 2° y vuelve. **Se deja así (decisión del usuario):** llegan siempre igual.
  3. **Media:** la telemetría juntaba el duty del período con la referencia del siguiente (`mvPaso()` la actualiza
     después de `paso()`): `ref − q` con 10 ms de adelanto falso (~0,1° a 10 °/s, comparable a `tolPos` 0,15° de J3).
     **Arreglado:** se guarda la referencia antes de `paso()`. Los gráficos de antes (`trayectoria_20261031_*`)
     exageran el atraso en el recorrido; el error al quedar firme no cambia.
- Compila; **sin flashear ni probar** (no estaba la placa): pendiente en `PENDIENTES.md`.

## 2026-11-03 — 20: sin el resorte de la pinza; resumen de capturas; paso 0

- **Mecánico:** el usuario sacó el resorte de la pinza (compensaba un juego para que no quedara inclinada hacia
  abajo; ahora la gravedad carga ese juego). Problema a resolver: con el brazo estirado y el limón, la punta oscila al
  llegar (video del 02/11, no versionado). Plan de la sesión (pasos 0–5): base sin resorte, diagnóstico de la
  oscilación (J1 recogido / estirado, J2-J3 solas), feedforward suave (signo desde la velocidad de la trayectoria),
  J1 más suave, `VEL_APROX` 3 °/s, deriva.
- **`tools/trayectoria.py`** (`4fd8b78`): resumen por `mv` y articulación (inversiones del duty con cambio de signo y
  salto > 15 % en el recorrido y en la aproximación, |error| máx de seguimiento, error del `LLEGO`, correcciones,
  movimiento ya firme, tiempo de la llegada a firmes), totales, cola; `--comparar` y `--solo-resumen`. Sobre 180843
  da J1 20 + 61 = 81, J2 7 + 8 = 15, J3 28 + 12 = 40: **las inversiones de J1 son sobre todo de la aproximación** a
  2 °/s.
- **Paso 0** (firmware 37d8827 flasheado, con los arreglos de Codex; lista fija, `--cola 5`):
  `trayectoria_20261103_092501`. Inversiones J1 81 → 38, J2 15 → 13, J3 40 → 34; |e| máx en el recorrido 0,90 /
  2,48 / 1,28° (J2 +0,57°: más carga sin el resorte); |e LLEGO| máx 0,105 / 0,084 / 0,062°; de la llegada a firmes
  0,2–0,5 s; encoders quietos en la cola. **J2 y J3 no caen ni tardan:** no se cambia el feedforward (duty medido en
  el recorrido J2 +34 / −24 %, J3 +19 / −41 %).
- **Homing:** 16,6 s (J3 3,5 s, J2 6,3 s a 0,5 °/s, J1 2,3 s buscando, más ~4,5 s de retrocesos). Lento porque al
  abrir el puerto la WROOM se reinicia y pierde el cero: busca todo a velocidad lenta. Propuesto (sin hacer): buscar
  más rápido (J2 1 °/s, J3 y J1 1,5 °/s), J1 junto con J3, y que `trayectoria.py` termine siempre en `mv 3 3 3`.
- **Homing más rápido** (`c0c44ca`: buscar a J2 1 °/s, J3 y J1 1,5 °/s) y `trayectoria.py` vuelve a `mv 3 3 3` al
  final de cada serie (`b2e428b`). `trayectoria_20261103_094631`: homing desde ~3° **16,6 → 12,1 s** (J2 buscando
  3,4 s; J1 no llega a 1,5 °/s, mediana 0,6). Los `mv` igual que en 092501. `ff 2 35 nan 22 8` → "Cada valor entre
  0 y 70 %" (arreglo de Codex verificado en placa). El usuario pide J2 todavía más rápido.
- **J2 a 1,5 °/s** (`b5f1648`) y **J1 junto con J3** si el brazo está abajo (`bb9dfae`; J2 sin cero o ≤ 5°; si no,
  J1 al final). `trayectoria_20261103_095335`: homing **7,7 s** (J3 y J1 2,1 s juntas, J2 2,3 s). Buscando hay
  stick-slip: J1 toca el flanco a 4–5 °/s (ya pasaba a 1 °/s), J2 a 3 °/s (antes 0,8). Falta ver si el flanco repite
  (un segundo `h` con cero da el corrimiento) y si el golpe mueve los finales de carrera. `mv` sin cambios.
  Intermedia: `trayectoria_20261103_094915`, J2 a 1,5 °/s con J1 todavía al final: 10,9 s.
- **Joystick en `20`** (para medir la caja sin cambiar de firmware): `jv a1 a2 a3` (`14768d4`), velocidad de cada una en
  lazo cerrado (a fondo J1 12 °/s, J2 y J3 8 °/s), al soltar se sostiene donde quedó, frena antes de los límites y la
  franja (3° de margen), hombre muerto 300 ms; `jv 0 0 0` no corta el homing ni un `mv`. Puente:
  `17-brazo-joystick/tools/joystick.py --placa 20` (`798f4b0`): botón Homing (`h`), estado por la telemetría y `c`,
  "Anotar" en `20/resultados/limites_*.csv`. Probado con una placa simulada (pty); **sin probar en placa**.
- **Joystick usado en placa** para el paso 1 (`20/resultados/limites_20261103_100457.csv`): con el brazo estirado
  (J2 74,3°, J3 69,2–70,2°) J1 en 20°, 40° y 60° **libre** de la caja. Poses del paso 1b: `mv 20 74 70` / `mv 60 74 70`.
  J3 pasó de 69,84° a 69,22° mientras solo se movía J1 (¿se tocó la barra de J3?). La zona prohibida (anotar el
  borde, lo que no se alcanza) queda para después.
- **Homing que trabó el brazo:** el joystick dejó el brazo estirado (60 / 74 / 69); al abrir el puerto la WROOM se
  reinició sin cero y el homing (que lo supone recogido) subió J3 28° con J2 en 74°: fuera de la franja, trabado,
  cortado a los 22 s por "no avanza" (`trayectoria_20261103_102001_cortado`). Los motores sostienen el brazo sin
  corriente (no se puede mover a mano: se rompe). Rescate: `zq q1 q2 q3` (`92b2ec5`, cero estimado de la anotación
  menos lo que movió el homing: 27,3 / 74,3 / 41,3) y `joystick.py --placa 20 --zq` (`5248171`); el joystick solo
  deja volver hacia adentro de la franja. Bajado y con homing: OK. Protección: sin cero el homing corta a los 15°
  (`f24114c`, sin probar en placa).
- **Paso 1a** (`trayectoria_20261103_103503`, J1 20 ↔ 60° con el brazo recogido): J2 y J3 no se mueven (0,01°). J1
  en la aproximación a 2 °/s tiene stick-slip: trabada con 16–18 % de duty, salta 1,7° a ~6 °/s y se vuelve a
  trabar; al llegar y frenar retrocede 0,1° (elasticidad de la correa) y queda 0,38° corta en 60° (firme por
  `tolSalida` 0,6°). El encoder de J1 está en el motor: una oscilación de la base por la correa no se ve en él.
  **Hipótesis:** con el brazo estirado, esos saltos de J1 en la aproximación sacuden la punta.
- **Homing de las tres juntas** (`42e5575`): 3,5 s (`trayectoria_20261103_104057`).
- **Paso 1b** (`trayectoria_20261103_104300`, J1 20 ↔ 60° con el brazo estirado y el limón): J3 no se mueve (0,02°);
  J1 llega bien, con saltos chicos (0,4°) en la aproximación. **J2 se mueve aunque solo se pida J1**: tras el primer
  `mv` quedó a 0,137° (> 0,05°) y hace la aproximación (2° abajo y vuelta). En la aproximación se traba 1 s (1,5° de
  atraso, duty 11 → 27 %), despega a 6,7 °/s, se pasa 0,72° y baja despacio 0,43° en 1,4 s: probablemente la
  "corrección de la punta" del video. La vuelta a `mv 3 3 3` tardó 29 s: J2 quedó 0,313° abajo (> `tolPos`),
  trabada; el integrador del sostener (khi 8) subió de 10 a 38 % en 11 s para moverla 0,016°.
- **La que ya está en `tolPos` no hace la aproximación** (`a4d4a99`). 1b de nuevo (`trayectoria_20261103_104918`):
  J2 quieta (inversiones 92 → 14, correcciones 16 → 0), de la llegada a firmes ≤ 0,3 s. En el video ahora corrige
  **J1**: en la aproximación a 2 °/s se traba, salta a 5–7 °/s, se pasa 0,3–0,6° y el feedforward cambia de signo
  (+17 → −19 %) porque sale de la velocidad pedida con la corrección: vuelve a −3,6 °/s. Es lo que ataca el paso 2.
- **Transmisión de J3** (dato del usuario): engranajes 75:15 + manivela 80 mm / pivote a 40 mm del eje del antebrazo:
  q3 es el ángulo del engranaje; el antebrazo gira ~2 veces eso (anotado en el README de 20 y en PENDIENTES, fase 6).
- **Paso 2, feedforward suave** (`c61c108`; `trayectoria_20261103_105648`): inversiones de las tres a 0. J1 sigue
  a saltos a 2 °/s (fricción de la correa y la caja) y queda 0,38° corta (frena a 0,3°, la correa la retrocede 0,1°).
- **J1 (opción A):** último tramo propio de 0,5° a 5 °/s, cada una con su duración en la aproximación (`6ae1805`) y
  `tolPos` 0,15° (`be11c83`). Sin probar en placa. Después: medir la frecuencia de la oscilación de la punta (video o
  IMU en la pinza) para un input shaping (ZV) de J1; si no alcanza, menos aceleración de J1 (paso 3).
- **J1 `tolPos` 0,15° no sirve** (`trayectoria_20261103_110204_cortado`): en el retroceso del homing J1 quedó en un
  ciclo límite ±0,3° alrededor de 3° (la cascada pide ≥ 3 °/s y J1 avanza a saltos de 0,3–0,6°; nunca queda quieta
  dentro de 0,15°) y el homing cortó a los 15 s. Revertido (vuelve a 0,3°). La aproximación propia de J1
  (`6ae1805`) queda, sin probar todavía.
- **J1 con aproximación propia** (`trayectoria_20261103_110513`): llega en un solo empujón (se traba, despega y salta
  ~0,7° a 6 °/s), se pasa 0,1–0,2° (antes quedaba 0,38° corta), inversiones 0, cada `mv` 7,7 s (antes 9). Con el
  encoder en el motor y la fricción de la correa, ±0,2° en la base parece el límite sin cambiar la mecánica.
- **Video fijo de la 1b** (`VID_20261103_113530.mp4`, 1080×1920 a 30 fps, fuera del repo; captura
  `trayectoria_20261103_113641`, el video va ~1,2 s detrás de la captura). Limón seguido por color con OpenCV
  (~1,1 mm por píxel, el limón mide ~52 px). **Llegando a 60° (J1 sigue en el mismo sentido): sin oscilación**, se
  acomoda en ~1,2 s. **Llegando a 20° (J1 baja a 19,5° y SUBE el último tramo: invierte el sentido): oscila
  ±12 px (~±1,4 cm) a ~1,5 Hz** (período ~0,67 s) y se apaga en ~1,5 s, **con los tres encoders quietos** (J1
  frenada en 19,89°, J2 y J3 frenadas): es la correa y la estructura, el encoder de J1 (en el motor) no lo ve. La
  primera subida (3 → 20 / 74 / 70) oscila igual a ~1,5 Hz (±45 px, amortiguamiento ~0,15–0,2). La vuelta a
  `mv 3 3 3` cortó con "no bajó por debajo del objetivo" (quedó en 2,9 / 0,9 / 1,3, abajo).
- **J1 llega en el sentido en que viene** (`a71f355`; `trayectoria_20261103_120620` con video
  `VID_20261103_120507.mp4`, 1920×1080 a 60 fps, de costado, fuera del repo; antes `trayectoria_20261103_120141`, la
  misma lista sin video). Llegadas de J1 sola (20 → 60 → 20 → 60): **la punta ya no se hamaca**, desvío 1,4–1,8 px
  (ruido del video, igual a 60° que a 20°), errores del LLEGO −0,22 a +0,06°. **Queda una oscilación en el primer
  `mv`** (desde abajo a 20 / 74 / 70): J1 tiene que durar lo mismo que J2/J3 (12,9 s para 17°), el final de su
  perfil es lentísimo y se traba 1,6 s en 18,68° (15–17 %, w = 0); al empezar la aproximación sube a 28 %, se despega
  a 16 °/s, se pasa a 21,0° y vuelve con −18 % a 19,86°. En el video: ±35 px a ~1,6 Hz (incluye el ida y vuelta de
  J1), se apaga en ~1,5 s. Arreglo: J1 con su propia duración en el recorrido (`MV_DURACION_PROPIA`).
