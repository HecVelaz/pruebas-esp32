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

## Falta

1. Imprimir y armar las marcas alrededor del arbolito.
2. Detector en la Pi (nodo ROS): imagen rectificada izquierda → `cv2.aruco` → pose de cada marca (`solvePnP` con el
   lado de 150 mm) → posición del eje del árbol en el marco del brazo (TF del 12/11). Medir contra cinta a 1, 2, 3 y
   4 m y a varios ángulos.
3. En `recolector_de_frutas/ros2_ws/src/mision`: estados BUSCAR / ACERCARSE / PARADA / RODEAR (primero en Gazebo,
   con las marcas en el mundo). Necesita la base en ROS con `/odom` medido (S3 con el `ros` completo).
