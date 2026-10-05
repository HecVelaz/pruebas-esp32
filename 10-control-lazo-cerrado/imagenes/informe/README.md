# Imágenes para el informe

Copias de las imágenes elegidas de `../j1_base`, `../j2_hombro` y `../j3_codo` (el original sigue allá).
En cada par, el escalón de posición usa el mismo lazo de velocidad (Kp, Ki) que el escalón de velocidad.

| Imagen | Qué muestra |
|--------|-------------|
| `j1_07_escalon_velocidad_20261002_215350.png` | J1, PI de velocidad (Kp 0,549, Ki 4): 40 escalones de ±20 °/s superpuestos, contra la simulación |
| `j1_12_escalon_posicion_20261002_223821.png` | J1, posición en cascada (Kpp 2, mismo PI): 20 escalones de ±20°, sobrepico 0, error 0,27–0,31°, llegada 1,71–1,77 s |
| `j3_07_escalon_velocidad_20261003_190957.png` | J3, PI de velocidad (Kp 0,5, Ki 5): escalones de ±10 °/s; subiendo se dispara al despegar (fricción) |
| `j3_09_escalon_posicion_20261003_191345.png` | J3, posición (Kpp 2, mismo PI): 12 escalones de ±10°, sobrepico 0, error ≤ 0,28°; picos de velocidad al despegar subiendo |
| `j3_18_barrido_carga_20261003_195629.png` | J3, duty necesario según el ángulo (barrido a 4 °/s, Kp 0,35, Ki 5): subiendo crece ~1,4 %/° por el peso → compensación de gravedad |
| `j3_28_escalon_posicion_20261003_212157.png` | J3, resultado final (Kp 0,35, Ki 5, compensación de gravedad, v_max 25 °/s): sobrepico 0, error ≤ 0,29°, llegada 1,0–1,9 s |
| `j2_07_escalon_posicion_20261003_230802.png` | J2 con resorte, posición (Kp 0,35, Ki 5, dither ±3 %): 12 escalones de ±10°, sobrepico ≤ 0,35°, error ≤ 0,35°. La línea "simulado" no vale para J2 (la simulación usa una sola zona muerta) |

Faltan: escalón de velocidad de J2 y de J3 con las ganancias finales (Kp 0,35, Ki 5), para completar esos pares.
