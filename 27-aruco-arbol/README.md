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

## Falta

1. Imprimir y armar las marcas alrededor del arbolito.
2. Detector en la Pi (nodo ROS): imagen rectificada izquierda → `cv2.aruco` → pose de cada marca (`solvePnP` con el
   lado de 150 mm) → posición del eje del árbol en el marco del brazo (TF del 12/11). Medir contra cinta a 1, 2, 3 y
   4 m y a varios ángulos.
3. En `recolector_de_frutas/ros2_ws/src/mision`: estados BUSCAR / ACERCARSE / PARADA / RODEAR (primero en Gazebo,
   con las marcas en el mundo). Necesita la base en ROS con `/odom` medido (S3 con el `ros` completo).
