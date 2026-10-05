La simulación se realiza en **Python** (`numpy`, `scipy` y `matplotlib`) sobre los modelos de la sección 5, con los mismos períodos de muestreo que el firmware: $T_s = 10$ ms para el lazo de velocidad y $20$ ms para el de posición. El simulador usa el **mismo controlador que corre en la placa** (mismas ganancias, límites y compensaciones), así que la simulación se puede comparar directamente con las pruebas.

#### Especificaciones de diseño

| Especificación | Valor | J1 (base), placa | J3 (codo), placa |
| :--- | :--- | :--- | :--- |
| Sobrepico de posición | 0 | 0 | 0 |
| Tiempo de llegada (a menos de 0,5 °) | ≤ 2 s | 1,71 a 1,77 s (±20 °) | 0,96 a 1,92 s (±10 °) |
| Error en régimen permanente | ≤ 0,5 ° | ≤ 0,30 ° | ≤ 0,29 ° |

El lazo de velocidad se diseñó para un tiempo de establecimiento de 0,25 s sin sobrepico.

#### Controlador

Se utiliza un control **en cascada**: un lazo de posición (P) pide una velocidad y un lazo de velocidad (PI) calcula el PWM.

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/diagrama_control.png" width="750"><br>
  <b>Fig. 10.</b>  Diagrama del control en cascada. El lazo de posición (P) pide una velocidad y el lazo de velocidad (PI) calcula el PWM del motor; el encoder mide el ángulo y, con él, la velocidad.
</p>

**Lazo de velocidad (PI, cada 10 ms):**

$$
u[k] = u_{ff} + K_p\, e[k] + K_i\, T_s \sum_{j=0}^{k} e[j], \qquad e[k] = \omega_{ref}[k] - \omega[k]
$$

**Lazo de posición (P, cada 20 ms):**

$$
\omega_{ref} = K_{pp}\,(\theta_{ref} - \theta)
$$

con las siguientes características:

- **Compensación de zona muerta ($u_{ff}$):** se suma el PWM mínimo para que el motor se mueva, en el sentido pedido. En J3 también se suma la fuerza para levantar el peso del antebrazo según el ángulo: $u_{ff} = 24 + 1{,}4\,\theta$ % subiendo y 9 % bajando.
- **Diseño por cancelación de polo:** con $K_p / K_i = \tau$, el lazo de velocidad queda $\dfrac{1}{\tau_{lc}s + 1}$ con $\tau_{lc} = \dfrac{\tau}{K K_p}$.
- **Sin sobrepico de posición** si $K_{pp} \le \dfrac{1}{4\,\tau_v}$, donde $\tau_v$ es lo que tarda en responder el lazo de velocidad.
- **Perfil de movimiento:** la velocidad pedida se limita a $v_{max}$ y su cambio a 60 °/s², para no golpear la mecánica.
- **Tolerancia de llegada** de ±0,3 °: dentro de ella el motor frena (y no vuelve a moverse hasta que se aleja más de 0,6 °), para que no zumbe alrededor del objetivo.
- **Anti-windup:** la integral no crece mientras el PWM está saturado.

#### Ganancias

| | $K_p$ | $K_i$ | $K_{pp}$ | $v_{max}$ | Límite sin sobrepico |
| :--- | :--- | :--- | :--- | :--- | :--- |
| J1 (base) | 0,549 | 4 | 2 | 30 °/s | $K_{pp} \le 2{,}78$ |
| J3 (codo) | 0,35 | 5 | 2 | 25 °/s | $K_{pp} \le 2{,}08$ |

Las ganancias de diseño eran más altas ($K_i = 8{,}44$ en J1; $K_p = 0{,}52$ y $K_i = 9{,}9$ en J3). En la placa se bajaron: en J1 por el juego de la correa, que daba 18 % de sobrepico de velocidad, y en J3 porque la fricción del tornillo sin fin lo hacía oscilar.

#### Efectos incluidos en el simulador

- Zona muerta (en J3, distinta subiendo y bajando, y dependiente del ángulo por el peso) y fricción para arrancar desde quieto.
- Saturación de PWM.
- Cuantización del encoder (0,0225 ° en la base de J1 y 0,029 ° en el codo de J3) y velocidad medida con la diferencia de cuentas en 40 ms, igual que el firmware.
- Muestreo a 100 Hz (velocidad) y 50 Hz (posición), y el perfil de movimiento con la tolerancia de llegada.

**No incluidos:** la holgura (juego) de la correa y de los engranajes, el "traba-suelta" del tornillo sin fin al bajar despacio y el endurecimiento del mecanismo del codo cerca de su posición más baja. Son la causa de las diferencias con la placa.

#### Escenarios

1. Escalones de velocidad: ±20 °/s en J1 y ±10 °/s en J3.
2. Escalones de posición: ±20 ° en J1, y ±10 ° y +20 ° en J3.
3. Las mismas secuencias de escalones de las pruebas en la placa, para comparar la simulación con la medición.

Para cada escenario se grafican la referencia, la posición, la velocidad y el PWM, y se calculan el sobrepico, el tiempo de llegada y el error final.

#### Resultados de la simulación

**J1 (base):** escalones de posición de 20 ° y 5 °. Llega en 1,64 s sin sobrepico.

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j1/j1_3_diseno_posicion.png" width="700"><br>
  <b>Fig. 11.</b>  Simulación del lazo de posición de J1 (base) con escalones de 20° y 5°: llega en 1,64 s sin sobrepico (Kpp = 2, v<sub>max</sub> = 30 °/s).
</p>


**J3 (codo):** escalones de 10 ° y 20 °. Llega en 1,3 a 1,7 s sin sobrepico; el PWM al subir crece con el ángulo por el peso.

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j3/j3_4_diseno_posicion.png" width="700"><br>
  <b>Fig. 12.</b>  Simulación del lazo de posición de J3 (codo) con escalones de 10° y 20°: llega en 1,3–1,7 s sin sobrepico (Kpp = 2, v<sub>max</sub> = 25 °/s). Al subir, el PWM crece con el ángulo por el peso del antebrazo.
</p>

#### Simulación contra la placa

| | Llegada en la placa | Llegada simulada | Sobrepico (placa / sim.) |
| :--- | :--- | :--- | :--- |
| J1, escalones de ±20 ° | 1,71 a 1,77 s | 1,64 s | 0 / 0 |
| J3, sube de 0 a +10 ° | 1,30 s | 1,29 s | 0 / 0 |
| J3, baja de 0 a −10 ° | 1,92 s | 1,06 s | 0 / 0 |

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j1/j1_5_comparacion_posicion_sim_vs_real.png" width="700"><br>
  <b>Fig. 13.</b>  Posición de J1 (base), placa (rojo) contra simulación (azul), en 20 escalones de ±20°. La placa llega en 1,71–1,77 s y la simulación en 1,64 s; ninguna se pasa del objetivo.
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j3/j3_7_comparacion_posicion_sim_vs_real_final.png" width="700"><br>
  <b>Fig. 14.</b>   Posición de J3 (codo), placa (rojo) contra simulación (azul), en 12 escalones de ±10°. Subiendo coinciden (1,30 s contra 1,29 s); bajando hacia −10° la placa tarda más (1,92 s contra 1,06 s) porque el mecanismo se endurece.
</p>

En J1 la simulación acierta: la placa llega 0,1 s más tarde por el juego de la correa. En J3 coincide subiendo, pero bajando hacia −10 ° la placa tarda casi el doble porque ahí el mecanismo se endurece, y el modelo no lo tiene.

El código del simulador está en el repositorio: `10-control-lazo-cerrado/tools/diseno_velocidad.py`, `diseno_posicion.py`, `escalon_velocidad.py`, `escalon_posicion.py` e `identificar_j3.py`. **J2 queda pendiente:** todavía no tiene modelo con el brazo montado.
