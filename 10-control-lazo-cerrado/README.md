# 10-control-lazo-cerrado — Control en lazo cerrado de J1 y J2 del brazo (ESP32-WROOM-32D)

Ajuste del control de **posición angular** de dos articulaciones del brazo del recolector, de a un lazo y con
gráficas, como en el lazo abierto (`06`, `07`, `08`). Las ganancias que salgan de acá se pasan a
`~/Documentos/recolector_de_frutas/firmware/brazo`, que ya tiene la cascada escrita pero **sin probar con motores**.

## Qué se controla

| | J1: base giratoria | J2: hombro |
|---|---|---|
| Motor | 36GP-555, 160 rpm, reductora 50:1 | 5840-31ZY, sin fin (no cae sin corriente) |
| Driver | IBT-2 (BTS7960), PWM 20 kHz | L298N canal A, modo freno, PWM 1 kHz |
| Encoder | Hall 16 PPR x4 → 3200 cuentas por vuelta de salida del motor | Óptico 1000 PPR x4 → 4000 cuentas por vuelta |
| Modelo de velocidad (R1 E5) | G = 1,58 / (0,065 s + 1) [rpm/%] | G = 1,33 / (0,059 s + 1) [rpm/%] |
| Zona muerta (arranque) | 16 % sin carga, 25 % con carga | entre 20 y 40 % (falta medir) |
| Límites (R1 E4) | q1 = ±75° | q2 = −25…135° |
| Transmisión motor → articulación | **falta medir** | **falta medir** |

Control en cascada (`docs/ARQUITECTURA.md` §5 del repo del robot):

```
ángulo pedido → [P de posición, 50 Hz] → velocidad pedida → [PI + zona muerta, 100 Hz] → PWM → motor
                       ↑                                          ↑
                       └──────────────── encoder ─────────────────┘
```

## Conexiones

Las del brazo del robot (`recolector_de_frutas/docs/conexiones.html`), todo en la **WROOM**. Pines en
`include/pins.h`. **Cableado completo** (motor → driver → ESP32, tracción, bumpers y lista de resistencias) en
[`CONEXIONES.md`](CONEXIONES.md). Diagrama para cablear (mapa de la placa, esquemas de J1 y J2 con las
resistencias y lista para tildar): [`conexiones.html`](conexiones.html), abrir en el navegador. Fuente:
laboratorio a 12 V: límite de 3 A con J1, 2 A con J2 sola y fusible de 2 A en el L298N si van los dos juntos
(ver `CONEXIONES.md`).

| GPIO | Conecta a | Nota |
|------|-----------|------|
| 21 · 22 | J1 IBT-2 RPWM · LPWM | 20 kHz |
| 23 | J1 IBT-2 R_EN + L_EN | pull-down 10 kΩ |
| 34 · 35 | J1 encoder A · B | pull-up **externo** 10 kΩ a 3V3 (sin pull-up interno) |
| 25 · 26 · 27 | J2 L298N ENA · IN1 · IN2 | jumper de ENA afuera, pull-down 10 kΩ en ENA |
| 32 · 33 | J2 encoder A · B | pull-up interno |
| 36 · 39 | Finales de carrera J1 · J2 | todavía no instalados |

- **Apagar los 12 V antes de flashear o pulsar EN** mientras falten los pull-down de los EN.
- J1 cambia de placa respecto de `06` (S3 → WROOM): revisar el signo del encoder.

## Plan paso a paso

Cada paso se prueba en la placa antes de pasar al siguiente. Hoja de ruta con las tareas de cada paso, para tildar: [`plan.html`](plan.html). Plan **solo de J1**, más simple y para seguir paso a paso: [`plan_j1.html`](plan_j1.html). Plan **solo de J2** (hombro, con los límites del cuatro barras): [`plan_j2.html`](plan_j2.html); `tools/cuatro_barras.py` calcula cuánto puede girar J2 con J3 quieto (balancín rojo de 60 mm: Grashof, γ₄ entre −112° y −30°). Plan **solo de J3** (codo): [`plan_j3.html`](plan_j3.html). **J3 pasó al IBT-2 (2026-10-03):** RPWM 16, LPWM 17, EN 13 (+10 kΩ a GND), encoder en 18/19; cableado en [`conexiones_j3.html`](conexiones_j3.html). Los duty de `cfg_j3` son los del L298N × 0,8, **provisorios** hasta repetir el paso 0.

| Paso | Qué | Para qué |
|------|-----|----------|
| 0 | Verificar el cableado de J1 y J2 en la WROOM (sentido de giro y signo del encoder) | Que "positivo" sea lo mismo para el motor y el encoder |
| 1 | Medir la **transmisión** motor → articulación de J1 y J2 | Pasar de cuentas a grados de la articulación |
| 2 | **PI de velocidad** en J1: escalones de rpm, gráficas, ajustar Kp y Ki | Lazo interno |
| 3 | PI de velocidad en J2 (zona muerta del sin fin) | Lazo interno |
| 4 | **P de posición** encima, primero J1 y después J2 | Ir a un ángulo y quedarse ahí sin pasarse |
| 5 | Perturbación y carga (frenar con la mano, peso en el brazo) | Ver que el lazo corrige |
| 6 | Pasar las ganancias a `firmware/brazo/include/brazo_config.h` | Integración |

La numeración de esta tabla es la del plan de J1 y J2 juntos. En `plan_j1.html` (solo J1) los pasos son
A (preparar), 0 (signo), 1 (transmisión), 2 (velocidad), **3 (posición = paso 4 de acá)**, **4 (perturbación =
paso 5 de acá)** y 5 (al robot).

**Firmware por articulación:** `pio run -e j1 -t upload` (base), `pio run -e j2 -t upload` (hombro) o `pio run -e j3 -t upload` (codo, por defecto). Los parámetros de cada una están en `include/config.h` (`cfg_j1`, `cfg_j2`). J2 arranca con valores **provisorios**: límites ±15° desde la marca, engranaje sin medir (grados de la salida del sin fin), `ie`/`is` habilitados en vivo.

**Comandos del firmware (los dos):** `z` cero en la marca · `c` posición · `e` encoder a mano · `p <%> [ms]` pulso ·
`m <°> [%]` ir con duty fijo · `ev <°/s> [ms] [ciclos]` escalones de velocidad (CSV) · `a <°>` ir con el control de
posición · `ep <A> [ms] [ciclos]` escalones de posición (CSV) · `kp`, `ki`, `kpp` ganancias · `g` mostrarlas ·
`x` rueda libre · `?` ayuda. Rango permitido **−45° / +135°** desde el cero (brazo al costado). Scripts en `tools/`:
`diseno_velocidad.py`, `diseno_posicion.py` (diseño y simulación), `escalon_velocidad.py`, `escalon_posicion.py`
(prueba en placa contra la simulación).

**Cero de posición:** el encoder es incremental, así que al encender la placa la posición arranca en 0 donde
esté el brazo. Hasta que haya finales de carrera, el cero se fija a mano: llevar la articulación a una marca
conocida y mandar un comando de cero.

## Estado

- [x] Paso 0 J1 (2026-10-02): encoder normal, sentido INVERTIDO (`J1_SENTIDO_INVERTIDO = true`). J2: pendiente
- [x] Paso 1 J1 (2026-10-02): reductora 50:1 × correa 90/18 = **44,44 cuentas por grado de la base** (16 000 por vuelta), confirmado a ojo con un doblez de 45°. J2: pendiente
- [x] Paso 2 J1 (2026-10-02): PI de velocidad **Kp = 0,549, Ki = 4** (diseño Ki = 8,44 daba 18 % de sobrepico por el juego de la correa); ts 0,33–0,38 s, sobrepico 4–7 %. Kalman descartado (no mejora la medición). J2: pendiente
- [ ] Paso 3: PI de velocidad J2 (Kp = ?, Ki = ?, zona muerta = ?)
- [x] Paso 4 J1 (2026-10-02): P de posición **Kpp = 2** (v_max 30 °/s, a_max 60 °/s², llega a ±0,3°, se despierta a ±0,6°, velocidad mínima 3 °/s): 20 escalones de ±20° → **llegada 1,71–1,77 s, sobrepico 0, error 0,27–0,31°, sin zumbido**. J2: pendiente
- [x] Paso 5 J1 (2026-10-02): la base **no se mueve a mano** (250:1, la mecánica la sostiene quieta); frenada con la mano en movimiento → el PI subió el duty de 22 a 26 % y recuperó 15 °/s en ~0,8 s; con un limón en la pinza, los escalones de posición dan lo mismo que sin carga. Falta el video. J2: pendiente
- [~] Paso 6 J1 (2026-10-02): parámetros pasados a `recolector_de_frutas/firmware/brazo/include/brazo_config.h` (commit `836cf95` de ese repo; en rpm de la reductora: kpVel 0,659, kiVel 4,8), con `velMin`, `ffSoloZonaMuerta` y `MOTOR_INVERTIDO` nuevos. **Falta:** decidir cómo se traduce el cero al costado y el rango −45/+135 al marco de `brazo_ik`, y probar ese firmware en la placa
- [ ] Finales de carrera de J1 y J2 (GPIO 36 y 39, pull-up externo)
- [ ] Pull-down de 10 kΩ en los EN (GPIO 23 y 25)

## Resultados

| Fecha | Paso | Resultado |
|-------|------|-----------|
| 2026-10-02 | 0 (J1) | Con duty + las cuentas suben (encoder normal), pero la base giraba horario → `J1_SENTIDO_INVERTIDO = true`. Verificado tras reflashear: `p 30 300` antihorario OK, `p -30 300` horario OK. `p 25 100`: solo 13 cuentas (1,5° de salida) en 0,1 s; con 30 % se mueve bien. Encoder en reposo A=0 B=1 |
| 2026-10-02 | 1 (J1) | Sin transportador: `m 45` quedó paralelo a un doblez de 45° de una hoja (error de pocos grados como mucho, a ojo); `m 0` vuelve a la marca sin diferencia visible (juego no medible a ojo). Repetible: 45,59° y 45,79° según el encoder. Con 30 % la base va a 22–28 °/s (→ zona muerta en marcha ≈ 17 % con K de la E5); al cortar el duty se pasa 0,6–0,8° (+) y 1,2–1,3° (−), y hacia − va algo más rápido. Cero de J1 = brazo hacia el **costado** del robot. Límite de software subido a ±60° |
| 2026-10-02 | 2 (J1) | Diseño por cancelación de polo para ts 0,25 s: Kp 0,549, Ki 8,44 (`tools/diseno_velocidad.py`). En placa (`ev`, `tools/escalon_velocidad.py`): un escalón con Ki 8,44 → ts 0,47 s y **18 % de sobrepico**; la meseta al arrancar y el golpe al invertir son el **juego de la correa** (el encoder está en el motor). Con pausa en 0 entre +v y −v y 40 ciclos (80 escalones): **Ki 4 → ts 0,33/0,38 s, sobrepico 3,9/7,1 % (ida/vuelta), ±0,9 %**; Ki 6 → ts 0,37/0,42 s, sobrepico 9,8/12,4 %. Elegido Ki 4. Para 20 °/s hace falta 26 % (+) y 24 % (−). Con los 12 V apagados, `ev` cortó a los 150 ms por "sin cuentas" (protección verificada) |
| 2026-10-02 | 2 (J1) | **Kalman descartado** con los datos de la prueba de Ki 4 (`resultados/escalon_vel_20261002_215350.csv`), contra una derivada centrada sin atraso: ventana de 40 ms → error 1,6 °/s, atraso 20 ms, ruido 0,36 °/s; ventana de 20 ms → 1,2 °/s, 10 ms, 0,56 °/s; Kalman de velocidad constante (mejor q) → 1,2 °/s, 10 ms, 0,49 °/s (igual que una ventana más corta); Kalman con el modelo del motor → 1,3 °/s, ruido 1,29 °/s (peor: el modelo no tiene el juego ni la asimetría). El límite es la resolución del encoder (1 cuenta en 10 ms = 2,25 °/s). Se sigue con la ventana de 40 ms |
| 2026-10-02 | 2 (J1) | Deriva: el lazo de velocidad no controla posición; en 40 ciclos la base se corrió hasta −4° (Ki 6) y usar `--cero` lejos de la marca corrió el cero ~20°. Hay que volver a fijar el cero en la marca |
| 2026-10-02 | 4 (J1) | Diseño (`tools/diseno_posicion.py`): lazo de velocidad cerrado ≈ 1/(0,09 s + 1) → sin sobrepico con Kpp ≤ 1/(4·0,09) = 2,78; elegido **Kpp = 2** (margen por el juego; igual que `brazo_config.h`). Velocidad pedida limitada a 30 °/s y su cambio a 60 °/s² (perfil trapezoidal, sin golpes); simulado 0 → 20° en 1,7 s sin sobrepico |
| 2026-10-02 | 4 (J1) | `ep 20 2500 5` (20 escalones ±20°, `tools/escalon_posicion.py`): llegada 1,83–2,00 s, **sobrepico 0**, error 0,25–0,31°, coincide con la simulación (~0,15 s más lento: juego). **Zumbido** al llegar: 67 arranques del motor (el error iba y venía en el borde de la tolerancia de 0,3°); el usuario sentía un golpe leve al frenar |
| 2026-10-02 | 4 (J1) | Con **histéresis** (llega a 0,3°, se despierta a más de 0,6°): zumbido 0, pero se cortó por "sin cuentas": a 0,81° del objetivo pedía 1,6 °/s → 17 % de duty y la fricción de arranque (~25 %) la trabó |
| 2026-10-02 | 4 (J1) | Con **velocidad mínima de 3 °/s** mientras no llegó y el corte "sin cuentas" solo con duty ≥ 25 %: **llegada 1,71–1,77 s (±0,01), sobrepico 0 en los 20, error 0,27–0,31°, zumbido 0, sin cortes** (`resultados/escalon_pos_20261002_223821.*`). El error queda en el borde de la tolerancia porque la fricción y el freno la detienen en el acto (la simulación, sin esa fricción al frenar, da 0,06°): para menos error, bajar la tolerancia (p. ej. 0,1° / 0,3°). Cumple la especificación (sobrepico 0, llegada ≤ 2 s, error ≤ 0,5°) |
| 2026-10-02 | 5 (J1) | **No se puede mover a mano** con los 12 V apagados: reductora 50:1 × correa 5:1 = 250:1; forzarla podría hacer saltar un diente de la correa (y el encoder, que está en el motor, perdería el cero). Quieta, la mecánica la sostiene: el control solo trabaja en movimiento. Por eso no hace falta un modo "mantener" |
| 2026-10-02 | 5 (J1) | **Perturbación en movimiento** (`ev 15 3000`, frenada suave con la mano en la ida, `resultados/escalon_vel_20261002_225238.*`): antes 14,9 °/s con 21,7 % de duty; al frenarla bajó a **11,8 °/s** (t = 1,19–1,41 s) y el PI subió el duty hasta **25,8 %**; al soltarla se pasó a **19,7 °/s** (lo que la integral acumuló durante la frenada) y volvió a 15 ± 1,2 °/s en t = 1,97 s (**~0,8 s** desde que empezó la frenada); después 15,0 °/s con 20 %. El "sobrepico de 31 %" que imprime el script en esa ida es el de la perturbación, no el del escalón (la vuelta, sin tocarla: 16 %, ts 0,53 s) |
| 2026-10-02 | 5 (J1) | **Con carga** (un limón en la pinza, `ep 20 2500 5`, `resultados/escalon_pos_20261002_225437.*`): llegada **1,67–1,75 s**, sobrepico 0 en los 20, error 0,22–0,32°, zumbido 0: **igual que sin carga** (1,71–1,77 s). Con 250:1, la inercia del limón vista desde el motor se divide por 250² y casi no cambia nada |
