Para un motor de corriente continua controlado por tensión de armadura, la función de transferencia entre la posición angular y la tensión es

$$
G(s) = \frac{\theta(s)}{V(s)} = \frac{K_t}{s\left[(Js + b)(Ls + R) + K_t K_b\right]}
$$

**Donde:**

- $K_t$: constante de torque [N·m/A]; $K_b$: constante de fuerza contraelectromotriz [V·s/rad].
- $J$: inercia [kg·m²]; $b$: fricción viscosa [N·m·s/rad].
- $L$: inductancia de armadura [H]; $R$: resistencia de armadura [Ω].

Con la inductancia despreciable ($L \approx 0$) se obtiene el modelo de segundo orden

$$
G(s) = \frac{K_m}{s(\tau_m s + 1)}, \qquad K_m = \frac{K_t}{R\,b + K_t K_b}, \qquad \tau_m = \frac{R\,J}{R\,b + K_t K_b}
$$

Como no se dispone de todos los parámetros físicos, el modelo se **identificó experimentalmente** (sección 3). La velocidad responde como un sistema de primer orden,

$$
G_\omega(s) = \frac{\Omega(s)}{U(s)} = \frac{K}{\tau s + 1}
$$

y la posición es su integral:

$$
G_\theta(s) = \frac{\theta(s)}{U(s)} = \frac{K_\theta}{s(\tau s + 1)}
$$

con la entrada $U$ en **% de PWM** y $K_\theta$ expresada en °/s por % de PWM ($K_\theta = 6\,K$ con $K$ en rpm/%).

#### Modelos identificados

| Articulación | $K$ [rpm/%] | $K_\theta$ [°/s por %] | $\tau$ [s] | Modelo de posición |
| :--- | :--- | :--- | :--- | :--- |
| J1 (36GP-555), eje libre | 1,58 | 9,5 | 0,065 | $\dfrac{9{,}5}{s(0{,}065\,s + 1)}$ |
| J1 en la base, montado | 0,32 | 1,90 | 0,065 | $\dfrac{1{,}90}{s(0{,}065\,s + 1)}$ |
| J2 y J3 (5840-31ZY), eje libre | 1,33 | 8,0 | 0,059 | $\dfrac{8{,}0}{s(0{,}059\,s + 1)}$ |
| J3 en el codo, brazo montado | 0,27 | 1,62 | 0,053 | $\dfrac{1{,}62}{s(0{,}053\,s + 1)}$ |

Los modelos "eje libre" se midieron con el motor solo. El de J1 en la base sale de dividir por la correa (5:1). El de J3 en el codo se **identificó con el brazo montado**, con los datos guardados de 8 escalones de velocidad y 2 barridos de carga: se le aplicó al modelo el mismo PWM que se aplicó al motor y se ajustaron $K$ y $\tau$ hasta que la velocidad simulada coincidiera con la medida.

#### No linealidades y limitaciones

- **Zona muerta:** el motor no se mueve por debajo de cierto PWM: 17 % en J1 (en la base) y 20 a 40 % en J2 con el eje libre. En J3 con el brazo montado depende del ángulo por el peso del antebrazo: subiendo $u_c = 23{,}6 + 1{,}42\,\theta$ % y bajando 3,2 %. Un controlador proporcional dejaría un error final, por lo que el diseño incluye compensación de zona muerta (en J3, también del peso según el ángulo) y acción integral.
- **Saturación:** el PWM está limitado a $\pm 100\,\%$.
- **Holgura y fricción** del reductor, en especial del tornillo sin fin.
- **Carga:** en J2 y J3 el torque gravitatorio depende del ángulo. J3 ya se re-identificó con el brazo montado (fila "J3 en el codo"); J2 todavía falta.
