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
laboratorio a 12 V con límite de 3 A.

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

Cada paso se prueba en la placa antes de pasar al siguiente.

| Paso | Qué | Para qué |
|------|-----|----------|
| 0 | Verificar el cableado de J1 y J2 en la WROOM (sentido de giro y signo del encoder) | Que "positivo" sea lo mismo para el motor y el encoder |
| 1 | Medir la **transmisión** motor → articulación de J1 y J2 | Pasar de cuentas a grados de la articulación |
| 2 | **PI de velocidad** en J1: escalones de rpm, gráficas, ajustar Kp y Ki | Lazo interno |
| 3 | PI de velocidad en J2 (zona muerta del sin fin) | Lazo interno |
| 4 | **P de posición** encima, primero J1 y después J2 | Ir a un ángulo y quedarse ahí sin pasarse |
| 5 | Perturbación y carga (frenar con la mano, peso en el brazo) | Ver que el lazo corrige |
| 6 | Pasar las ganancias a `firmware/brazo/include/brazo_config.h` | Integración |

**Cero de posición:** el encoder es incremental, así que al encender la placa la posición arranca en 0 donde
esté el brazo. Hasta que haya finales de carrera, el cero se fija a mano: llevar la articulación a una marca
conocida y mandar un comando de cero.

## Estado

- [ ] Paso 0: cableado verificado
- [ ] Paso 1: transmisión J1 = ? · J2 = ?
- [ ] Paso 2: PI de velocidad J1 (Kp = ?, Ki = ?)
- [ ] Paso 3: PI de velocidad J2 (Kp = ?, Ki = ?, zona muerta = ?)
- [ ] Paso 4: P de posición J1 / J2
- [ ] Paso 5: pruebas con perturbación y carga
- [ ] Paso 6: ganancias pasadas al firmware del brazo
- [ ] Finales de carrera de J1 y J2 (GPIO 36 y 39, pull-up externo)
- [ ] Pull-down de 10 kΩ en los EN (GPIO 23 y 25)

## Resultados

| Fecha | Paso | Resultado |
|-------|------|-----------|
| | | |
