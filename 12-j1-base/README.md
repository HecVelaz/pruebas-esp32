# 12-j1-base — Prueba definitiva de J1 (base giratoria), montada en el brazo

Confirmar el lazo cerrado de **J1** con el **conexionado definitivo del robot** (v0.5 de `recolector_de_frutas`):
las tres articulaciones conectadas a la vez en la WROOM, con el **EN común** en el GPIO 23. J1 ya pasó todos los
pasos en `10-control-lazo-cerrado` (2026-10-02) con el mismo motor, la misma correa y los mismos pines: acá se
repiten rápido con el brazo completo armado, para tener el resultado final.

Pruebas hermanas: [`13-j2-hombro`](../13-j2-hombro/README.md) y [`14-j3-codo`](../14-j3-codo/README.md). Orden
recomendado: **J1 → J2 → J3**.

## Qué se controla

| | J1: base giratoria |
|---|---|
| Motor | 36GP-555, 12 V, 160 rpm, reductora 50:1 |
| Transmisión | correa GT2, polea de 18 dientes en el motor y **84** en la base (4,67:1; se creía 90) |
| Encoder | SCX-555, 16 PPR x4 = 64 cuentas por vuelta del motor → **41,48 cuentas por grado de la base** (polea de 84, medido el 2026-10-30) |
| Driver | IBT-2 (BTS7960), PWM 20 kHz |
| Rango | **−30° (hacia el mástil de la visión; choca a −38°) / +90° (alejándose)** desde el cero: brazo hacia el frente del robot (fotos en [`CERO/`](CERO/), 2026-10-30) |
| Ganancias (de 10) | PI de velocidad Kp 0,549, Ki 4, zona muerta 17 %; P de posición Kpp 2, v_max 30 °/s |

## Conexiones (v0.5, definitivas)

Detalle cable por cable: `recolector_de_frutas/docs/CONEXIONES.md` y `docs/html/placa_wroom.html`. Pines en
[`include/pins.h`](include/pins.h).

| GPIO | Conecta a | Nota |
|---|---|---|
| 21 · 22 | **J1** IBT-2 RPWM · LPWM | lo que mueve esta prueba |
| 34 · 35 | **J1** encoder A (negro) · B (rojo) | SCX-555 a **3V3**; pull-up de su placa |
| 23 | R_EN + L_EN de los **3** IBT-2 | pull-down 10 kΩ a GND |
| 25 · 26 / 36 · 39 | J2 IBT-2 / encoder | conectados, quietos |
| 16 · 17 / 18 · 19 | J3 IBT-2 / encoder | conectados, quietos |
| 32 · 27 · 33 | Finales de carrera J1 · J2 · J3 | no se usan acá (J2 y J3 cruzados respecto de v0.5, 2026-10-30) |

IBT-2: **VCC al riel de 5 V** (VIN del shield; en `10` iba a 3V3), GND al riel GND, R_IS y L_IS sin conectar; B+ / B− a 12 V. Motor: blanco = M+, amarillo = M−. Encoder:
verde = 3V3, azul = GND.
Con el VCC a 5 V, el alto de 3,3 V de la WROOM queda justo para la entrada del IBT-2 (si es un 74HC244 pide
3,5 V; suele andar igual). Si en `dg` o en el paso 0 el motor no responde o va a los saltos, probar el VCC de ese
IBT-2 a 3V3, como en `10` (`recolector_de_frutas/docs/CONEXIONES.md`, "IBT-2 a 3,3 V").

**Con el EN común, J2 y J3 se frenan o se sueltan junto con J1.** Mientras J1 se mueve o está quieta (EN alto,
`FRENO_EN_REPOSO = true`), J2 y J3 quedan **frenadas**; con `x` o al arrancar la placa (EN bajo) quedan **sueltas**
y, como no se autobloquean, el brazo cae. Durante esta prueba: **brazo apoyado en su tope** (o atado), antes de
encender los 12 V.

## Seguridad

- **Fuente de laboratorio a 12 V con límite de 3 A**, no la LiPo (nada de pruebas con motores trabados con la LiPo).
- **Apagar los 12 V antes de flashear o pulsar EN** si todavía falta el pull-down de 10 kΩ del GPIO 23.
- Cada flasheo o reset borra el cero: llevar la base a la marca y mandar `z`.
- Ctrl+C o Enter en cualquier prueba → frena.

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/12-j1-base
~/.platformio/penv/bin/pio run -t upload     # 12 V APAGADOS
tools/monitor.sh                             # monitor sin tocar DTR/RTS; salir con Ctrl+]
```

Al arrancar imprime `== 12 · J1 (base): 36GP-555 160 rpm + IBT-2, WROOM ==`, los pines, los signos y las
ganancias. Los scripts de `tools/` (enlace a `../10-control-lazo-cerrado/tools`) guardan en **`resultados/` de esta
carpeta**; cerrar el monitor antes de correrlos.

## Pasos

Orden de la fuente en cada paso: **12 V apagados → monitor → base en la marca → `z` → 12 V encendidos → comandos →
12 V apagados**.

| Paso | Qué hacer | Qué se espera (lo de 10) |
|---|---|---|
| A | Revisar el cableado con la tabla de arriba. Monitor abierto, sin 12 V: `e` y girar la polea del motor a mano | las cuentas cambian; A/B nunca a mitad de camino (medir con el multímetro: 0 o 3,3 V) |
| A | Con 12 V: `dg` (diagnóstico del IBT-2, etapa por etapa) | EN ~3,3 V, RPWM/LPWM según la etapa, M+/M− con tensión |
| 0 | `p 30 300` y `p -30 300` | **+ = antihorario** visto desde arriba, − = horario; las cuentas suben con + |
| 1 | `m 45`, `m 0`, `m -30`, `m 0` | 45° contra una escuadra o un doblez de 45°; vuelve a la marca |
| 2 | `python3 tools/escalon_velocidad.py -a j1 20 --ciclos 10` | ts 0,33–0,38 s, sobrepico 4–7 % |
| 3 | `python3 tools/escalon_posicion.py -a j1 20 --ciclos 5` | llegada ~1,7 s, sobrepico 0, error ≤ 0,3°, sin zumbido |
| 4 | `a 45`, `a 0`, `a 85`, `a -27`, `a 0` | todo el rango sin tocar nada (cables, mástil) |
| 5 | Paso 3 otra vez con un limón en la pinza | igual que sin carga |

Si el paso 0 da al revés: el motor está cableado al revés que en 10 (cambiar `SENTIDO_INVERTIDO` en
`include/config.h`, o invertir M+/M−). Si con duty + las cuentas bajan: `ENCODER_INVERTIDO`. En J1 `ie`/`is` están
deshabilitados porque los límites son asimétricos.

**Comandos:** `?` muestra todos. Los más usados: `z` cero · `c` posición · `e` encoder · `p <%> [ms]` pulso ·
`m <°> [%]` duty fijo · `a <°>` control de posición · `ev`/`ep` escalones (CSV) · `kp`, `ki`, `kpp`, `g` ganancias ·
`x` rueda libre · `dg` diagnóstico.

## Estado

- [ ] A: cableado, encoder y `dg` (2026-10-26: motor y encoder andan; falta `dg`)
- [x] 0: signos (2026-10-30, cero al frente: `SENTIDO_INVERTIDO = false`, + se aleja del mástil, encoder de acuerdo)
- [x] 1: 41,48 cuentas por grado (2026-10-30: polea de 84; `m 87` queda a ~90° a ojo)
- [x] 2: PI de velocidad (2026-10-30: funciona, peor que en 10: ts ~0,63 s, sobrepico 14,5 %)
- [x] 3: posición (2026-10-30: igual que en 10)
- [x] 4: rango completo −30° / +90° (2026-10-30)
- [x] 5: con carga (2026-10-30, limón)

## Resultados

Los CSV y PNG quedan en `resultados/` (nombre con fecha y hora).

| Fecha | Paso | Resultado |
|---|---|---|
| 2026-10-26 | PWM | `p` 200 ms: a 1 kHz arranca con ~8–10 %, a 20 kHz con ~20 % (15 % no). Queda en 20 kHz |
| 2026-10-26 | 0 | `p 30 200`: +3,44° antihorario, `p -30 200`: −3,04° horario. Después de frenar sigue ~0,7–0,9° |
| 2026-10-30 | cero | Cero nuevo: brazo hacia el frente (fotos en `CERO/`). Con `SENTIDO_INVERTIDO = true`, + iba hacia el mástil: pasa a `false` |
| 2026-10-30 | 1 | `m 84` con la escala de 90/18: la base quedó a 90° a ojo con 3764 cuentas (41,8 por grado) → polea de **84** dientes (41,48 por grado). Con la escala nueva, `m 87` queda en +88,45° y a ojo a 90° |
| 2026-10-30 | 2 | `escalon_vel_20261030_123837`: ±20 °/s ×10, Kp 0,549 Ki 4. ts 0,65 / 0,61 s (ida / vuelta, peor 0,78), sobrepico 14,5 %, error −0,6 °/s. Baja a ~15 °/s a los 0,1–0,2 s (correa con la inercia del brazo) y el integral se pasa. Dos corridas cortadas al arrancar por "sin cuentas" (28 → 39 % en 150 ms sin moverse); al reintentar arranca |
| 2026-10-30 | 3 | `escalon_pos_20261030_124228`: ±20° ×5. Llegada 1,81–1,89 s, sobrepico 0, error ≤ 0,31°, zumbido 0. Al final, saltitos de stick-slip a la velocidad mínima (3 °/s) antes de entrar en ±0,3° |
| 2026-10-30 | 4 | `a 45`, `a 85`, `a 0` con error ≤ 0,26°. `a -40` chocó con el soporte de la visión a −38,2° (corte por "sin cuentas") → límite negativo a **−30°** (comandos hasta −27°). `a -27` OK, `a -35` rechazado |
| 2026-10-30 | 5 | `escalon_pos_20261030_135849` con un limón en la pinza: llegada 1,77–1,85 s, sobrepico ≤ 0,06°, error ≤ 0,27°, zumbido 0. Igual que sin carga; mismo stick-slip al final |
