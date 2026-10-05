En la Entrega 5 se caracterizaron ambos motores en lazo abierto aplicando un PWM conocido y midiendo con el encoder la respuesta del eje, sin realimentación. Los ensayos fueron:

- **Barrido de PWM**, para obtener la curva velocidad-PWM, la ganancia y la zona muerta.
- **Escalón de PWM**, para obtener la constante de tiempo.
- **Perfil en escalera** ($\pm 20\dots 100\,\%$), con ajuste de un modelo de primer orden a cada tramo mediante `scipy.optimize.curve_fit`.
- **Rampa con y sin carga** (motor base), para evaluar el efecto de la carga.

La adquisición se realizó con un período de muestreo de 10 ms y lectura del encoder por PCNT.

#### Resultados resumidos

| Parámetro | Motor base (36GP-555 + IBT-2) | Motor articulación (5840-31ZY + L298N) |
| :--- | :--- | :--- |
| Ganancia estática $K$ | 1,58 rpm/% | 1,33 rpm/% |
| Constante de tiempo $\tau$ | 65 ms | 59 ms |
| Rango lineal | 20 a 85 % | 40 a 100 % |
| Zona muerta (arranque) | 16 % (vacío), 25 % (con carga) | 20 a 40 % |
| Velocidad máxima | ~145 rpm | ~140 rpm |
| Simetría y repetibilidad | error < 2,5 rpm / ±1,5 rpm | error < 5 rpm / ±5 rpm |

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/06-motor-36gp555/resultados/barrido_20260925_172703.png" width="650"><br>
  <b>Fig. 7a.</b>  Barrido de PWM del motor base (36GP-555 + IBT-2), en los dos sentidos: la velocidad crece en forma lineal con el PWM (~1,6 rpm/%) desde ~11 % y es simétrica. Arriba de ~85 % la curva salta y se aplana en ~145 rpm.
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/06-motor-36gp555/resultados/perfil_20260925_175338.png" width="750"><br>
  <b>Fig. 7b.</b>  Perfil en escalera del motor base (±20 a 100 % de PWM, 5 s por escalón): posición, PWM aplicado, respuesta en cada nivel y velocidad. Cada escalón llega a su velocidad en unos 65 ms (τ) y la respuesta es simétrica.
</p>

<p align="center">
  <img src="https://raw.githubusercontent.com/HecVelaz/pruebas-esp32/main/08-motor-5840-l298n/resultados/perfil_20260925_215823.png" width="750"><br>
  <b>Fig. 7c.</b>  Perfil en escalera del motor de articulación (5840-31ZY + L298N): con ±20 % no se mueve (zona muerta) y desde 40 % la velocidad crece casi en forma lineal hasta ~140 rpm a 100 %.
</p>

#### Limitaciones de esta caracterización

- Los ensayos se hicieron con el **eje libre** (en el motor base, también con una carga de prueba). Con el brazo armado, la gravedad y la inercia de los eslabones cambian la zona muerta, $K$ y $\tau$. Por eso se repitió la caracterización con el brazo montado: en J3 (codo) se identificó un modelo nuevo (sección 5), y en J1 (base) el modelo con el eje libre alcanzó para predecir la respuesta.
- **Zona muerta del 5840 con el brazo montado:** se midió con barridos lentos a velocidad constante (4 °/s), en los dos sentidos y en todo el rango de ángulos, con el driver IBT-2 a 1 kHz. En **J3 (codo)**, el PWM para subir crece con el ángulo por el peso del antebrazo: $26{,}1 + 1{,}42\,\theta$ % (dispersión ±4,9 %); para bajar alcanza con −5,7 % en todo el rango. En **J2 (hombro, con un resorte que compensa parte del peso)**, para subir hacen falta ~59 % a −10 °, ~55 % a 0 ° y ~48 % a +20 °, y para bajar, −9 a −10 % en todo el rango. Así, la zona muerta dejó de ser un rango amplio (20 a 40 %) y pasó a ser un valor conocido según el ángulo y el sentido, que el firmware compensa.
- **Por encima del 85 % de PWM** la velocidad del motor base deja de crecer de forma lineal. La causa más probable es el **driver**: el BTS7960 del IBT-2 no alcanza a conmutar bien a 20 kHz. En J3, con el mismo driver a 20 kHz, el motor recibía menos tensión de la esperada (0,6 V en lugar de 1,8 V con 15 % de PWM), y al bajar a 1 kHz rendía 2 a 3 veces más. En el motor base no se usa más de 40 % de PWM, así que no afecta al control. Para confirmarlo, falta repetir el barrido del motor base a 1 kHz o medir la tensión de la fuente durante el barrido.
