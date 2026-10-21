# 14-j3-codo — Prueba definitiva de J3 (codo), montada en el brazo

Caracterizar y cerrar el lazo de **J3** con su **motor definitivo** (36-555 de 58 rpm con engranajes 15 → 75) y el
**conexionado definitivo del robot** (v0.5 de `recolector_de_frutas`): las tres articulaciones conectadas a la vez
en la WROOM, con el **EN común** en el GPIO 23. Hasta el 2026-10-10 el codo tenía un 5840-31ZY con sin fin (lo medido en `10` no sirve para este motor): todo lo de
`include/config.h` marcado **PROVISORIO** se reemplaza con lo que se mida acá.

Pruebas hermanas: [`12-j1-base`](../12-j1-base/README.md) y [`13-j2-hombro`](../13-j2-hombro/README.md). Conviene
hacer J2 antes: así se sabe cuánto se mueve el hombro frenado y la postura de la verde en su marca.

## Qué se controla

| | J3: codo (balancín rojo del cuatro barras, que lleva el antebrazo) |
|---|---|
| Motor | 36-555, 12 V, 58 rpm, reductora 139:1, planetario: **no se autobloquea** (sin corriente el antebrazo cae) |
| Transmisión | engranajes helicoidales, 15 dientes en el motor y 75 en el codo (5:1) |
| Encoder | SCX-555, 16 PPR x4 → **123,6 cuentas por grado del codo** (verificar en el paso 1) |
| Driver | IBT-2 (BTS7960), PWM 20 kHz (si rinde poco a duty bajo, probar 1 kHz) |
| Velocidad máxima | ~70 °/s del codo a 100 % (58 rpm / 5) |
| Rango | provisorio **±15°** desde la marca; el definitivo sale del cuatro barras (paso 1) |
| Sostener | `sa` activo desde `z`: el lazo de posición mantiene el antebrazo entre comando y comando |

## Conexiones (v0.5, definitivas)

Detalle cable por cable: `recolector_de_frutas/docs/CONEXIONES.md` y `docs/html/placa_wroom.html`. Pines en
[`include/pins.h`](include/pins.h).

| GPIO | Conecta a | Nota |
|---|---|---|
| 16 · 17 | **J3** IBT-2 RPWM · LPWM | lo que mueve esta prueba |
| 18 · 19 | **J3** encoder A (negro) · B (rojo) | SCX-555 a **3V3**; pull-up de su placa + interno |
| 23 | R_EN + L_EN de los **3** IBT-2 | pull-down 10 kΩ a GND |
| 21 · 22 / 34 · 35 | J1 IBT-2 / encoder | conectados, quietos |
| 25 · 26 / 36 · 39 | J2 IBT-2 / encoder | conectados, quietos (frenados con el EN alto) |
| 32 · 33 · 27 | Finales de carrera J1 · J2 · J3 | no se usan acá |

IBT-2: **VCC al riel de 5 V** (VIN del shield; en `10` iba a 3V3), GND al riel GND, R_IS y L_IS sin conectar; B+ / B− a 12 V. Motor: blanco = M+, amarillo = M−. Encoder:
verde = 3V3, azul = GND.
Con el VCC a 5 V, el alto de 3,3 V de la WROOM queda justo para la entrada del IBT-2 (si es un 74HC244 pide
3,5 V; suele andar igual). Si en `dg` o en el paso 0 el motor no responde o va a los saltos, probar el VCC de ese
IBT-2 a 3V3, como en `10` (`recolector_de_frutas/docs/CONEXIONES.md`, "IBT-2 a 3,3 V").

**Antes del primer encendido:** con el encoder a 3V3 y sin girar, medir A y B en 18 y 19: tienen que estar en 0 o
3,3 V, nunca a mitad.

## Seguridad

- **Fuente de laboratorio a 12 V con límite de 3 A**, no la LiPo. Trabado, el motor pide hasta 10 A.
- **J3 no sostiene el antebrazo sin corriente.** Hasta hacer `z`, y siempre que se mande `x` o se apaguen los 12 V,
  el antebrazo cae: tenerlo con la mano o apoyado en un tope.
- **J2 queda frenada, no sostenida:** con el peso de todo el brazo puede bajar despacio y eso cambia la postura del
  cuatro barras. Apoyar el hombro en un tope (o sostenerlo) durante toda la prueba.
- **Apagar los 12 V antes de flashear o pulsar EN** si todavía falta el pull-down de 10 kΩ del GPIO 23.
- Cada flasheo o reset borra el cero y el sostener: sostener el antebrazo con la roja en la marca, `z`.
- El sostener solo funciona con los signos bien: **en el paso 0, con la mano en el antebrazo**.

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/14-j3-codo
~/.platformio/penv/bin/pio run -t upload     # 12 V APAGADOS
tools/monitor.sh                             # monitor sin tocar DTR/RTS; salir con Ctrl+]
```

Los scripts de `tools/` (enlace a `../10-control-lazo-cerrado/tools`) guardan en **`resultados/` de esta carpeta**;
siempre con `-a j3` (J3 con este motor todavía no tiene modelo: los gráficos salen sin simulación). Cerrar el monitor antes.

## Pasos

Orden de la fuente en cada paso: **12 V apagados → monitor → roja en la marca (antebrazo sostenido) → `z` → 12 V encendidos
→ comandos → antebrazo sostenido o apoyado → 12 V apagados**.

| Paso | Qué hacer | Para qué / qué anotar |
|---|---|---|
| A | (Opcional, **en banco**, antes de montar) motor suelto: `z`, `e` y girarlo a mano; `p 30 300`, `p -30 300` | motor, encoder e IBT-2 andan; una vuelta de la salida del motor = 8896 cuentas |
| A | Montado, sin 12 V: `e` y mover el antebrazo a mano un poco. Con 12 V: `dg` | las cuentas cambian; el IBT-2 entrega tensión en cada etapa |
| 0 | `p 30 100` y `p -30 100` (subir de a 10 % si no se mueve; **mano en el antebrazo**). Si sale al revés: `is` (o `ie` si las cuentas van al revés del movimiento) | **+ = la pinza sube** y las cuentas suben. Anotar con qué duty arranca subiendo y bajando. Copiar `ENCODER_INVERTIDO` / `SENTIDO_INVERTIDO` a `config.h` |
| 0 | `s` y soltar el antebrazo; `khp`/`khi` si oscila o cae | que sostenga quieto; anotar el duty que informa cada 1 s |
| 1 | `m 10`, `m 0` y medir el ángulo de la roja con el inclinómetro | **123,6 cuentas por grado** dentro de ~0,5° |
| 1 | Medir en la marca el ángulo de la verde (q2) y de la roja (θ) y correr `python3 tools/cuatro_barras.py --q2 <q2> --theta <θ>` | límites definitivos de J3 con J2 quieta (10° de margen; con el 5840 era −12° / +25° útil): pasarlos a `LIMITE_POS/NEG_GRADOS` |
| 2 | `python3 tools/barrido_carga.py --cero --desde <límite de abajo + 3> --hasta <límite de arriba − 3> --v 4` | duty necesario según el ángulo, subiendo y bajando → feedforward (`ff`) y compensación de gravedad (%/°) |
| 3 | `python3 tools/escalon_velocidad.py -a j3 10 --ciclos 3`, después con `--kp`/`--ki`/`--ff` | PI de velocidad: subida sin sobrepico grande, bajada sin traba–suelta |
| 4 | `python3 tools/escalon_posicion.py -a j3 10 --ciclos 3` | llegada < 2 s, sobrepico < 0,5°, error ≤ 0,3°, sin zumbido |
| 5 | Paso 4 con un limón en la pinza; frenar con la mano en movimiento | el lazo corrige; anotar el duty de sostener con carga |
| 6 | Pasar signos, ganancias, feedforward y límites a `recolector_de_frutas/firmware/brazo/include/brazo_config.h` | integración |

Con los valores del paso 2 en la mano, se puede identificar un modelo de J3 (adaptando `tools/identificar_j3.py`, que
lo hizo con el codo viejo: queda como `PLANTAS["j3"]`) y diseñar Kp/Ki con `tools/diseno_velocidad.py`.

**Comandos:** `?` muestra todos. `z` cero · `c` posición · `e` encoder · `p <%> [ms]` pulso · `m <°> [%]` duty fijo
· `ie`/`is` invertir encoder/sentido en vivo · `s [°]` sostener · `sa` sostener automático sí/no · `khp`, `khi` ·
`bc` barrido · `ev`/`ep` escalones (CSV) · `ff`, `bj`, `kp`, `ki`, `kpp`, `g` · `a <°>` · `x` rueda libre (**el antebrazo
cae**) · `dg` diagnóstico.

## Estado

- [ ] A: cableado, encoder y `dg`
- [ ] 0: signos y duty de arranque subiendo / bajando; sostener
- [ ] 1: cuentas por grado; límites del cuatro barras
- [ ] 2: barrido de carga → feedforward
- [ ] 3: PI de velocidad
- [ ] 4: posición
- [ ] 5: con carga y perturbación
- [ ] 6: valores al firmware del robot

## Resultados

Los CSV y PNG quedan en `resultados/` (nombre con fecha y hora).

| Fecha | Paso | Resultado |
|---|---|---|
| | | |
