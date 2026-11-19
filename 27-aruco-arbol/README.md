# 27-aruco-arbol — marcas ArUco para encontrar el arbolito

El GPS (UP501) tiene ±20 m: sirve para la rúbrica (zona de trabajo, registro del recorrido, fusión con
`robot_localization`), no para llegar al árbol. El profesor acepta un solo arbolito, que puede estar en cualquier lugar
del aula, y acepta marcas (13/11). Plan de navegación:

```
BUSCAR (girar en el lugar de a ~20°; si no ve, avanzar 1 m esquivando con los HC-SR04 y volver a girar)
  └─ ve una marca ─> ACERCARSE (odometría + giróscopo, el árbol a la izquierda, hasta ~1 m)
       └─> PARADA k: corregir con la marca (y los limones) hasta el eje del árbol a 70 cm a la izquierda de J1 y
           15 cm adelante (26-sim-alcance-arbolito) ─> agarrar ─> rodear 90° (antihorario) ─> PARADA k+1 ... (4)
```

## Marcas

- `tools/generar_marcas.py` → `resultados/marcas_aruco_4x4_150mm.pdf`: 4 hojas A4, **DICT_4X4_50, IDs 0–3, cuadrado
  negro de 150 mm**, dibujadas como vectores (tamaño exacto). Verificado: OpenCV 4.6 (notebook y Pi) detecta los 4 IDs
  en el PDF renderizado, con 149,9 mm de lado.
- Imprimir al **100 % / tamaño real** (no "ajustar a la página"), medir el cuadrado con regla, recortar dejando el
  margen blanco de 5 cm (sin él no se detecta) y pegar sobre cartón rígido y plano (sin curvar: la maceta redonda no
  sirve tal cual).
- **Dónde:** una por cara de una caja o cubo alrededor de la base del arbolito, el centro a ~20–40 cm del piso (a 1 m la
  cámara ve el piso hasta 68 cm de altura), mirando hacia afuera. ID 0 = parada 1, ID 1 = parada 2, … en sentido
  antihorario visto desde arriba (el robot rodea el árbol con el árbol a su izquierda, donde mira la cámara).
- Tamaño (cámara del mástil, f = 672 px, 640 × 480): la marca de 15 cm mide ~100 px a 1 m, ~50 a 2 m, ~25 a 4 m
  (ArUco detecta bien desde ~25–30 px): alcanza para un aula buscando desde el medio.

## Detector y calibración para la navegación (13/11)

- `tools/ver_marcas.py` (Pi, con `start_limon.sh`): `/left/image_rect` → `cv2.aruco` → `solvePnP` (IPPE_SQUARE, 150 mm)
  → posición de la marca, rumbo y hacia dónde mira, en el marco del brazo. `--tf nav` usa la calibración de abajo
  (`--tf brazo`, la del 12/11, solo sirve < 70 cm). `--radio-maceta <mm>` estima el eje del árbol y cuánto le falta al
  robot para la parada. `--csv`; foto cada 2 s en `~/logs/aruco_ultima.jpg` (puede salir gente: no versionarla).
- **La calibración del brazo no sirve lejos:** a 1 m de J1 la marca salía a 89 cm y 15 cm más alta (compensa con
  14° de inclinación el error en z del modelo de la punta; la cámara real está casi horizontal).
- **Calibración de navegación:** `tools/medir_marca.py <punto> <x> <y> <h>` (mediana de 5 s de la pose en el marco de
  la cámara + lo medido con cinta) y `tools/ajuste_nav.py` (Kabsch → `resultados/tf_navegacion.json`). 6 puntos en el
  piso (`resultados/marca_nav_20261113.csv`): de costado 1 / 1,5 / 2 m × adelante 0 / 30 cm, centro a 22 cm del piso
  (marca ID 0, 80 / 56 / 44 px, dispersión ≤ 0,4 mm). **Residuo medio 27 mm (máx 44), dejando cada uno afuera 36 mm
  (máx 52).** Cámara a 57,4 cm del piso, 6,5° hacia abajo, mira +3,8° hacia adelante. En vivo en P6: (−2017, −17)
  contra (−2000, 0). El error mayor es a lo largo del robot (y); x ±2 cm.

## Banderín único y BUSCAR (19/11, sin probar en el robot)

Una sola marca (la ID 0 que ya está impresa) en un palo, en cualquier lugar, y en `banderin.json` dónde está el **eje
del arbolito** respecto de ella, medido en el piso con cinta y **mirando la marca de frente**:

- `arbol_derecha_mm`: hacia la derecha de quien mira la marca (izquierda: negativo).
- `arbol_frente_mm`: hacia quien la mira; el árbol detrás de la marca: negativo.
- `parada_lado_deg`: de qué lado del árbol para el robot en la parada 1 (0 = del lado al que mira la marca, 90 = un
  cuarto de vuelta antihorario visto de arriba). El robot queda con el árbol a su izquierda, como siempre.

`python3 tools/ir_a_parada.py --banderin --buscar [--rodear N]`: BUSCAR gira en el lugar de a 30° a la izquierda
(giróscopo) y mira 1,5 s en cada paso, hasta 360°. Cuando ve la marca, la mide 3 s y calcula el árbol y el plan
(girar / avanzar / girar; los rectos largos en tramos de 1,5 m, hasta 3 m). **No va si el camino pasa J1 a menos de
45 cm del banderín o del tronco** (mover el robot, el banderín o `parada_lado_deg`). RODEAR con banderín va a la
parada siguiente solo con la odometría (girar 34° izq., avanzar 112 cm, girar 56° izq.; J1 pasa a 48 cm del
tronco). `--arbol D F` pisa lo del JSON; `--simular X Y MIRA` prueba la cuenta sin ROS (X, Y del banderín).

El banderín no puede quedar donde se para o pasa el robot (anillo de ~0,5–1,5 m alrededor del árbol): mejor a un
costado, o atrás del árbol visto desde la parada. Error de rumbo del banderín: sin medir (lo que más pesa si está lejos
del árbol).

## Falta

1. Imprimir y armar las marcas alrededor del arbolito.
2. Detector en la Pi (nodo ROS): imagen rectificada izquierda → `cv2.aruco` → pose de cada marca (`solvePnP` con el
   lado de 150 mm) → posición del eje del árbol en el marco del brazo (TF del 12/11). Medir contra cinta a 1, 2, 3 y
   4 m y a varios ángulos.
3. En `recolector_de_frutas/ros2_ws/src/mision`: estados BUSCAR / ACERCARSE / PARADA / RODEAR (primero en Gazebo,
   con las marcas en el mundo). Necesita la base en ROS con `/odom` medido (S3 con el `ros` completo).
