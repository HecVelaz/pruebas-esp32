# Imágenes para el informe

Copias de las imágenes elegidas de `../j1_base`, `../j2_hombro` y `../j3_codo` (el original sigue allá).
En cada par, el escalón de posición usa el mismo lazo de velocidad (Kp, Ki) que el escalón de velocidad.

**Las simulaciones de J1 y J3 (diseño, modelo de J3 y comparación con la placa) y el informe en `.tex` están en
[`../../simulaciones/`](../../simulaciones/README.md).**

**Simulación** = el modelo de la planta (motor + reducción, una ecuación con K, τ y la zona muerta) más el mismo
controlador del firmware (PI de velocidad a 100 Hz, P de posición a 50 Hz, feedforward, límites, encoder), resueltos
paso a paso en Python (`tools/diseno_velocidad.py`, `tools/diseno_posicion.py`). Sirve para diseñar (de la
especificación salen las ganancias) y para validar (la línea azul de los escalones contra la medida en la placa).

## J1 (base)

Modelo: `τ·dω/dt = −ω + K·(u − 17 %)`, K = 1,90 (°/s)/%, τ = 65 ms (pruebas de lazo abierto de 06 y paso 1),
despega con 25 %.

| Imagen | Qué muestra |
|--------|-------------|
| `j1_07_escalon_velocidad_20261002_215350.png` | Placa contra simulación, PI de velocidad (Kp 0,549, **Ki 4**: con 8,44 el juego de la correa daba 18 % de sobrepico): 40 escalones de ±20 °/s superpuestos |
| `j1_12_escalon_posicion_20261002_223821.png` | Placa contra simulación, posición (Kpp 2, mismo PI): 20 escalones de ±20°, sobrepico 0, error 0,27–0,31°, llegada 1,71–1,77 s |

## J3 (codo)

Modelo identificado con los datos guardados (`tools/identificar_j3.py`): una sola K (es un solo motor) y el peso del
antebrazo corre la zona muerta subiendo según el ángulo; bajando el peso ayuda y el sin fin no deja que se caiga:
`τ·dω/dt = −ω + K·(u − c+(θ))` subiendo, `−ω + K·(u + c−)` bajando, y `−ω` en el medio, con
K = 1,62 (°/s)/%, τ = 53 ms, c+(θ) = 23,6 + 1,42·θ %, c− = 3,2 %; quieta despega con 5,9 % más subiendo y 6,1 % bajando.

| Imagen | Qué muestra |
|--------|-------------|
| `j3_18_barrido_carga_20261003_195629.png` | Duty necesario según el ángulo (barrido a 4 °/s): subiendo crece ~1,4 %/° por el peso |
| `j3_07_escalon_velocidad_20261003_190957.png` | Placa contra simulación, PI de velocidad (Kp 0,5, Ki 5): bajando coincide; subiendo la placa se dispara a ~20 °/s al despegar (juego del engranaje, no está en el modelo) |
| `j3_09_escalon_posicion_20261003_191345.png` | Placa contra simulación, posición (Kpp 2, mismo PI): llegadas 1,60 / 1,30 / 1,35 s contra 1,60 / 1,31 / 1,36 simuladas; de 0 a −10° 1,67 s contra 1,32 |
| `j3_28_escalon_posicion_20261003_212157.png` | Resultado final (Kp 0,35, Ki 5, compensación del peso, v_max 25 °/s): sobrepico 0, error ≤ 0,29°. Subiendo 1,30 s contra 1,29 simulado; **de 0 a −10° 1,92 s contra 1,06**: cerca del punto muerto de abajo (~−22°) el cuatro barras se endurece y el modelo supone una fricción constante al bajar |

Lo que el modelo de J3 no tiene, y explica las diferencias: el juego del piñón y el engranaje (picos al despegar
subiendo), el traba-suelta al bajar despacio (Stribeck) y el endurecimiento cerca del punto muerto de abajo.

## J2 (hombro)

| Imagen | Qué muestra |
|--------|-------------|
| `j2_07_escalon_posicion_20261003_230802.png` | J2 con resorte, posición (Kp 0,35, Ki 5, dither ±3 %): 12 escalones de ±10°, sobrepico ≤ 0,35°, error ≤ 0,35°. Sin simulación: J2 no tiene modelo |

Faltan: escalón de velocidad de J2 y de J3 con las ganancias finales (Kp 0,35, Ki 5), y un modelo de J2.
