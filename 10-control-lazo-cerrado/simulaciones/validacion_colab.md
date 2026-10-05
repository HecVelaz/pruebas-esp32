La validación compara la respuesta **medida** en el brazo con la **simulada**, para la misma referencia, las mismas ganancias y la misma compensación. Se validaron J1 (base) y J3 (codo), montados en el brazo.

#### Ensayos

| Ensayo | Descripción | J1 (base) | J3 (codo) |
| :--- | :--- | :--- | :--- |
| Escalones de velocidad | $+v$, 0, $-v$, 0 repetido | ±20 °/s, 80 escalones | ±10 °/s, 6 escalones ($K_p$ = 0,5) |
| Escalones de posición | 0 → $+A$ → 0 → $-A$ → 0 repetido | ±20 °, 20 escalones | ±10 °, 2 pruebas de 12 escalones |
| Con carga | Escalones de posición con un limón en la pinza | ±20 °, 20 escalones | — |
| Perturbación | Frenar la base con la mano mientras se mueve | escalón de 15 °/s | — |
| Repetibilidad | Dispersión entre repeticiones del mismo escalón | 5 por tipo de escalón | 3 por tipo, en 2 pruebas |
| Verificación del ángulo | Medir el ángulo con un instrumento externo | doblez de 45 ° en una hoja | inclinómetro |

Las ganancias no se modificaron durante ningún ensayo. Todos se hicieron con las ganancias finales de cada articulación, salvo los escalones de velocidad de J3 y su primera prueba de posición, que usaron $K_p$ = 0,5 en lugar de 0,35.

#### Métricas

- **Sobrepico:** $M_p = \dfrac{\theta_{max} - \theta_{ref}}{\theta_{ref}} \times 100\,\%$
- **Tiempo de llegada:** $t_s$ hasta entrar y permanecer a menos de **0,5 °** de la referencia. No se usa la banda del 2 % porque el control frena al llegar a ±0,3 ° del objetivo: con un escalón de 10 °, la banda del 2 % (±0,2 °) es más chica que esa tolerancia.
- **Error en régimen permanente:** $e_{ss} = \theta_{ref} - \theta_{final}$.
- **RMSE** entre la simulación y la medición, en el primer ciclo de cada prueba: $\sqrt{\dfrac{1}{N}\sum_k \left(\theta_{sim}[k] - \theta_{med}[k]\right)^2}$.

#### Resultados

| Motor | Ensayo | $M_p$ [%] | $t_s$ [s] | $e_{ss}$ [°] | RMSE [°] |
| :--- | :--- | :--- | :--- | :--- | :--- |
| J1 | Posición ±20 ° (20 escalones) | 0 | 1,71 – 1,77 | 0,27 – 0,30 | 0,49 |
| J1 | Posición ±20 ° con carga (20 escalones) | 0 | 1,67 – 1,75 | 0,22 – 0,32 | — |
| J1 | Velocidad ±20 °/s (80 escalones) | 3,9 – 7,1 | 0,33 – 0,38 | 0,14 – 0,54 °/s | — |
| J3 | Posición ±10 °, primera versión (12 escalones) | 0 – 1,4 | 1,30 – 1,67 | ≤ 0,28 | 0,45 |
| J3 | Posición ±10 °, versión final (2 × 12 escalones) | 0 | 0,96 – 1,92 | ≤ 0,29 | 1,07 – 1,26 |

**Repetibilidad:** entre repeticiones del mismo escalón, el tiempo de llegada varía ±0,01 s en J1 y ±0,01 a 0,02 s en J3.

**Perturbación (J1):** al frenar la base con la mano a 15 °/s, la velocidad bajó a 11,8 °/s y el PI subió el PWM de 21,7 % a 25,8 %. Al soltarla, la base volvió a 15 °/s en unos 0,8 s (figura 16). Quieta, la base no se puede mover a mano: la reducción total es de 250:1.

**Verificación del ángulo:** en J1, la orden de ir a 45 ° quedó paralela a un doblez de 45 ° en una hoja (verificado a ojo). En J3, el encoder marcó 12,6 ° y el inclinómetro 13 °: el factor de 34,57 cuentas por grado del codo es correcto con una diferencia de unos 0,5 °.

Las figuras 13 y 14 muestran la simulación superpuesta a la medición de la posición de J1 y J3. Las figuras 15 y 16 muestran la velocidad de J1, y las figuras 17 y 18, la velocidad y la primera prueba de posición de J3.

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j1/j1_4_comparacion_velocidad_sim_vs_real.png" width="750"><br>
  <b>Fig. 15.</b>  Velocidad de J1 (base), placa contra simulación: 80 escalones de ±20 °/s superpuestos (rojo claro), su promedio (rojo) y la simulación (azul). La placa llega a la velocidad pedida un poco antes y se pasa 4–7 %, contra 1 % en la simulación.
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/imagenes/j1_base/j1_15_escalon_velocidad_20261002_225238.png" width="750"><br>
  <b>Fig. 16.</b>  Perturbación en J1: escalón de velocidad de 15 °/s con la base frenada a mano entre 1,2 y 1,4 s. La velocidad cae a 11,8 °/s, el PI aumenta el PWM y la base vuelve a 15 °/s en unos 0,8 s.
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j3/j3_5_comparacion_velocidad_sim_vs_real.png" width="750"><br>
  <b>Fig. 17.</b>  Velocidad de J3 (codo), placa contra simulación: 3 escalones de ±10 °/s superpuestos (Kp = 0,5, Ki = 5). Bajando coinciden; subiendo, la placa tiene un pico de ~20 °/s al arrancar por el juego del piñón y el engranaje, que el modelo no tiene.
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/10-control-lazo-cerrado/simulaciones/j3/j3_6_comparacion_posicion_sim_vs_real_kp05.png" width="700"><br>
  <b>Fig. 18.</b>  Posición de J3 (codo), primera versión (Kp = 0,5, Ki = 5, v<sub>max</sub> = 15 °/s), placa (rojo) contra simulación (azul), en 12 escalones de ±10°. Las llegadas coinciden (1,60 / 1,30 / 1,35 s contra 1,60 / 1,31 / 1,36 s), salvo bajando de 0 a −10° (1,67 s contra 1,32 s).
</p>

#### Discusión

**Cumplimiento de las especificaciones de la sección 6:** en J1 y J3 la posición se cumple en todos los ensayos: sin sobrepico, llegada en menos de 2 s y error final por debajo de 0,5 °. El lazo de velocidad de J1 quedó en $t_s$ = 0,33 – 0,38 s y sobrepico de 4 – 7 %. Es más lento que los 0,25 s del diseño, porque $K_i$ se bajó de 8,44 a 4 para reducir el sobrepico que causaba el juego de la correa.

**Diferencias entre simulación y medición:**

- **J1:** la placa llega unos 0,1 s más tarde que la simulación (RMSE de 0,49 °). La causa es el juego de la correa: el encoder está en el motor, y al invertir el giro el motor recorre el juego antes de mover la base. El error final real (0,27 – 0,30 °) es mayor que el simulado (0,05 – 0,09 °), porque en la placa la fricción y el freno la detienen apenas entra en la tolerancia de ±0,3 °.
- **J3:** subiendo coinciden (1,30 s contra 1,29 s). Bajando hacia −10 ° la placa tarda casi el doble (1,92 s contra 1,06 s): el mecanismo de cuatro barras se endurece cerca de su posición más baja, y el modelo supone una fricción constante al bajar. Por eso el RMSE de J3 (1,1 – 1,3 °) es mayor que el de J1. También hay picos de velocidad al empezar a subir, por el juego del piñón y el engranaje, y un "traba-suelta" del tornillo sin fin al bajar despacio.
- **Carga:** el limón no cambia la respuesta de J1, porque con la reducción de 250:1 su inercia casi no se nota en el motor.

**Limitaciones:**

- J1 usa el modelo del motor con el eje libre, que alcanzó para predecir la respuesta con el brazo montado. J3 se re-identificó con el brazo montado (sección 5). J2 todavía no tiene modelo con carga ni validación.
- Las verificaciones del ángulo fueron simples (un doblez de papel y un nivelador). Falta una verificación con transportador o con escala en video.
- No se hizo el ensayo de perturbación en J3.
