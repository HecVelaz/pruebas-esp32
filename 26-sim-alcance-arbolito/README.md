# 26-sim-alcance-arbolito — simulación de la estación de trabajo

Simulación en Python, **sin placa**: el robot parado al lado del arbolito, ¿qué limones puede agarrar con el comando
`agarrar` de `20-brazo-coordinado/tools/punta.py`? Y de ahí: **dónde tiene que parar la navegación** (distancia y
posición respecto del árbol) y **cuántas paradas** hacen falta alrededor del árbol para agarrar todos.

## Qué tiene en cuenta

| Qué | De dónde |
|---|---|
| Alcance del **agarre** (entrar de frente desde 10 cm antes, cerrar, salir con 3° de margen, volver al reposo), con límites, franja del cuatro barras y choques con caja, mástil y piso | el mismo planificador que usa el brazo: `planear_agarre` de `punta.py` y el modelo medido de `cinematica.py` (reutilizados, no copiados) |
| Robot: tapa en T 40 × 23 cm adelante y 34 × 17 atrás, tapa a 23 cm del piso, J1 a 9 cm del borde de atrás y del izquierdo, hombro 16,5 cm sobre la tapa, mástil a la derecha y adelante | `recolector_de_frutas` (`arturito.urdf.xacro`, `docs/RESUMEN_PROYECTO.md`), `~/Imágenes/brazo/tapa.jpeg`, `medidas.jpeg`, `vista_lateral.jpeg` |
| Cámara: dónde está y hacia dónde mira, zona con disparidad (u ≥ 280 px, 0,40–0,95 m), imagen de 640 × 480 | calibración cámara → brazo del 12/11 (`ajuste_camara.py`) y `calibracion_mastil/left.yaml` de la Pi |
| Arbolito: 60 cm de alto, limones colgados en la superficie de la copa (cilindro) entre `--h-min` y 54 cm | usuario (13/11); el radio de la copa y el limón más bajo son **supuestos** (`--radio-copa 200`, `--h-min 300`): medirlos |
| Alturas: el modelo de la punta sube y baja menos que el brazo real; la altura del piso se pasa a la z del modelo con la recta de las mediciones con cinta del 12/11 (piso ≈ 256 + 1,43·z mm) | `notas/BITACORA.md` (12/11, calibración cámara → brazo) |

Un limón cuenta como **agarrable** si el planificador llega, la cámara lo ve en la zona con disparidad, y está del lado
de la copa que mira al robot (la pinza entra en recta desde J1 y no puede atravesar el árbol).

## Uso

```
cd 26-sim-alcance-arbolito
python3 tools/alcance.py          # una vez (~15 min, 4 procesos): resultados/mapa_alcance_<fecha>.csv
python3 tools/simular.py          # segundos: figuras y resumen en resultados/sim_<fecha>_*
python3 tools/simular.py --radio-copa 150 --h-min 280 --estacion 600 150   # otro árbol / otra estación
```

Salidas (`resultados/`):
- `sim_<fecha>_mapa_rz.png`: alcance del agarre (distancia a J1 × altura del piso) para tres direcciones de J1.
- `sim_<fecha>_estaciones.png`: % de limones agarrables según dónde para el robot (D, Y) y con 1–6 paradas.
- `sim_<fecha>_estacion_3d.png` y `_planta.png`: la mejor estación (o la de `--estacion`) en 3D y en planta.
- `sim_<fecha>_resumen.txt`: los números.

## Resultados

Primera corrida (13/11, `resultados/sim_20261113_163252_*`, mapa `mapa_alcance_20261113_161717.csv`: 972 puntos,
15 min). Arbolito supuesto: copa de 20 cm de radio, limones de 30 a 54 cm del piso.

- **Alcance del agarre** (`_mapa_rz.png`): agarra con el limón a **55–70 cm de J1** (horizontal) y de **~30 a ~66 cm
  del piso**: toda la franja de los limones del arbolito (30–54 cm) entra. Casi igual en las tres direcciones de J1
  (5°, 25°, 45°). Más cerca de 55 cm no: el punto previo de la entrada (10 cm antes) cae fuera de la franja de J2/J3;
  más lejos de ~72 cm J2 pasa de su límite (80°).
- **Una estación** (`_estaciones.png`): lo mejor es el eje del árbol a **D = 70 cm a la izquierda de J1** y **Y = 32 cm
  adelante de J1** (más o menos a la altura del frente del robot): la copa queda a ~38 cm del costado izquierdo.
  Desde ahí agarra el **28 %** de los limones (9 de los 13 del lado que mira al robot, en todas las alturas). La zona
  buena es angosta: ±5 cm en D y en Y bajan rápido el porcentaje.
- **Varias estaciones**: 2 → 56 %, 3 → 83 %, **4 → 100 %** (cada 90° alrededor del árbol, D 75 cm, Y 17 cm).
- La cámara no limita en la mejor estación: todo lo que el brazo alcanza cae en la zona con disparidad.

Limitaciones: la copa es un cilindro (sin ramas ni hojas que tapen); los limones del otro lado se descartan con un
ángulo fijo (60° al brazo, 75° a la cámara); la recta de alturas de la cinta vale entre ~33 y ~52 cm del piso; el
modelo de la punta tiene ~1–2 cm de error y la calibración de la cámara ~2 cm.
