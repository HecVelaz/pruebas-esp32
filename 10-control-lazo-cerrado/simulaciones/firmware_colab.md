#### Estructura del firmware

El firmware se desarrolla en **PlatformIO** (Arduino-ESP32) en el proyecto `10-control-lazo-cerrado`, reutilizando los drivers de la Entrega 5 (enlazados, no copiados) y agregando el control en cascada:

| Módulo | Función |
| :--- | :--- |
| `lib/BTS7960/` | Driver del IBT-2 (q1 y q3): PWM con signo, freno y rueda libre |
| `lib/L298N/` | Driver del L298N en modo freno (q2, primera versión) |
| `lib/EncoderPCNT/` | Encoder en cuadratura x4 por el contador de pulsos (PCNT) del ESP32 |
| `include/config.h` | Parámetros de cada articulación: ganancias, compensación de zona muerta y de peso, límites y protecciones |
| `include/pins.h` | Pines de cada articulación (los mismos del robot) |
| `src/main.cpp` | Comandos, lazo de velocidad (PI), lazo de posición (P), protecciones y envío de datos |

Todo corre en una **ESP32-WROOM-32D**. Se compila una articulación por vez, eligiendo el entorno: `pio run -e j1` (base), `-e j2` (hombro) o `-e j3` (codo). Cuando una articulación queda probada, sus parámetros pasan al firmware del robot (`recolector_de_frutas/firmware/brazo`).

#### Lazo de control

El lazo corre con un período fijo de **10 ms**, medido con `micros()`. En cada período el firmware:

1. Lee el conteo del encoder y calcula la velocidad con la diferencia de cuentas de los últimos 40 ms.
2. Cada 20 ms (uno de cada dos períodos), el **lazo de posición** calcula la velocidad pedida: $\omega_{ref} = K_{pp}(\theta_{ref} - \theta)$, limitada a $v_{max}$ y a 60 °/s², y 0 si ya llegó (±0,3 °).
3. El **lazo de velocidad** calcula el PWM: compensación de zona muerta (y de peso en q3) más el PI, con anti-windup.
4. Limita el PWM y lo aplica al driver.
5. Revisa las protecciones y envía por serie una línea con el tiempo, la referencia, la posición, la velocidad y el PWM.

#### Código del controlador

Lazo de velocidad (PI), simplificado de `pasoPI()` en `src/main.cpp`. Unidades: velocidad en °/s y PWM en %.

```cpp
float pasoPI(float wRef, float wMed, float th, float &integral) {
  if (wRef == 0.0f) { integral = 0.0f; return 0.0f; }      // velocidad pedida 0: frena
  const float e = wRef - wMed;                              // error de velocidad [°/s]
  const int signo = wRef > 0 ? 1 : -1;
  // compensación de zona muerta; en q3 subiendo, también del peso según el ángulo
  const float ff = signo > 0 ? ffMarchaPos + ffPendPos * th : -ffMarchaNeg;
  const float uLibre = ff + kpVel * e + integral;           // PWM sin limitar [%]
  const float u = constrain(uLibre, -DUTY_MAX_PCT, DUTY_MAX_PCT);
  if (u == uLibre || (e > 0) != (uLibre > 0))               // anti-windup
    integral += kiVel * 0.01f * e;                          // Ts = 10 ms
  return u;
}
```

Lazo de posición (P), de `cascada()`:

```cpp
if (k % 2 == 0) {                                     // cada 20 ms
  const float e = thRef - grados(c);                  // error de posición [°]
  // llegado: dentro de ±0,3° (y no se aleja más de 0,6°)
  const float v = fminf(fmaxf(fabsf(kpp * e), VMIN_POS), VMAX_POS);
  const float deseada = llegado ? 0.0f : (e > 0 ? v : -v);
  const float dMax = AMAX_POS * 0.02f;                // aceleración máxima
  wRef = deseada == 0.0f ? 0.0f : constrain(deseada, wRef - dMax, wRef + dMax);
}
const float u = pasoPI(wRef, wMed, grados(c), integral);   // cada 10 ms
aplicarDuty(u);
```

#### Ganancias utilizadas

Las mismas que en la simulación, con las mismas unidades:

| | $K_p$ [%/(°/s)] | $K_i$ [%/°] | $K_{pp}$ [1/s] | Zona muerta ($u_{ff}$) | $v_{max}$ | PWM máx. |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| q1 (base) | 0,549 | 4 | 2 | 17 % | 30 °/s | 40 % |
| q3 (codo) | 0,35 | 5 | 2 | $24 + 1{,}4\,\theta$ % subiendo, 9 % bajando | 25 °/s | 80 % |

#### Comandos por puerto serie

| Comando | Función |
| :--- | :--- |
| `z` | Fija el ángulo actual como cero (con la articulación en su marca). Sin `z` no se mueve |
| `a <grados>` | Va a un ángulo con el control de posición |
| `ep <A> [ms] [ciclos]` | Escalones de posición $+A$, 0, $-A$, 0 (envía CSV) |
| `ev <°/s> [ms] [ciclos]` | Escalones de velocidad (envía CSV) |
| `bc <desde> <hasta> [°/s]` | Barrido lento a velocidad constante, para medir el PWM que hace falta según el ángulo (CSV) |
| `kp`, `ki`, `kpp <valor>` | Cambian las ganancias sin recargar el programa; `g` las muestra |
| `ff <valores>` | Cambia la compensación de zona muerta y de peso |
| `m <grados> [%]` | Va a un ángulo con PWM fijo, sin control (pruebas) |
| `p <%> [ms]` | Pulso de PWM fijo (pruebas) |
| `c`, `e` | Muestran la posición y el encoder |
| `x` o Enter | Frena: rueda libre / corta cualquier prueba |

#### Protecciones

- **Límite de ángulo** por software (q1: −45 ° a +135 °; q3: −15 ° a +28 °).
- **PWM máximo** configurable (40 % en q1, 80 % en q3).
- **Sin cuentas del encoder:** corta si con PWM aplicado el encoder no cuenta durante 150 ms (encoder suelto o motor trabado).
- **Sentido contrario:** corta si se mueve al revés de lo pedido durante 100 ms (signos mal configurados).
- **Atasco:** corta si hay PWM alto y no se mueve durante 300 ms.
- **Cero obligatorio:** no se mueve hasta hacer `z`, y cualquier Enter frena.
- **Límite de corriente en la fuente:** 3 A con q1, 2 A con q2 y q3.

#### Captura y procesamiento de datos

Los datos se reciben por serie en la PC con scripts de Python (`tools/escalon_velocidad.py`, `tools/escalon_posicion.py`, `tools/barrido_carga.py`), que mandan el comando, guardan el **CSV** en `resultados/` y lo grafican contra la simulación. Cada CSV empieza con una línea con los parámetros de la prueba y termina con `# fin ok` o `# cortado <causa>`. Columnas:

- Escalones de posición: `t_ms, th_ref, th, w_ref, w_med, duty`
- Escalones de velocidad y barridos: `t_ms, w_ref, w_med, duty, pos`

Como Colab no tiene acceso al puerto serie local, los CSV se suben al cuaderno (o se leen desde el repositorio de GitHub) para su análisis y graficación.
