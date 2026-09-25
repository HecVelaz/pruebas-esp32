# 06-motor-36gp555 — Motor 36GP-555 con driver IBT-2 (BTS7960), lazo abierto

Primera prueba del motor: el duty del PWM se fija a mano desde el monitor serial y el motor gira sin realimentación (**lazo abierto**). El encoder se lee solo para **medir** la velocidad (no corrige nada). Estos datos (zona muerta, curva duty–rpm, respuesta al escalón) sirven después para diseñar el control en lazo cerrado.

## Hardware

| Pieza | Dato |
|-------|------|
| Motor | **36GP-555**, 12 V, **160 rpm** en la salida, eje de 8 mm, reductora planetaria metálica (**50:1**, deducida: el 555 gira a ~8000 rpm a 12 V) |
| Encoder | Hall en el eje del motor (antes de la reductora), fases A/B, **16 PPR** → 64 cuentas por vuelta del motor en x4 → **3200 cuentas por vuelta de la salida** |
| Driver | Módulo **IBT-2**: dos medios puentes BTS7960 (43 A, 6–27 V) y un buffer 74HC244 en las entradas |
| Fuente | 12 V / 3 A para el motor. El ESP32 se alimenta por USB |

## Conexiones

```
 Fuente 12 V ──(+)──────────────► B+  ┐
             ──(−)──────────────► B−  │ bornera del IBT-2
 Motor+ (blanco)  ──────────────► M+  │
 Motor− (amarillo) ─────────────► M−  ┘

 IBT-2 (tira de 8 pines)          ESP32-S3 (breakout Freenove)
   1 RPWM  ◄──────────────────── GPIO4
   2 LPWM  ◄──────────────────── GPIO5
   3 R_EN  ◄────────┬─────────── GPIO6
                    └─ 10 kΩ ─ GND     ← pull-down OBLIGATORIO
   4 L_EN  ◄────────┬─────────── GPIO7
                    └─ 10 kΩ ─ GND     ← pull-down OBLIGATORIO
   5 R_IS                         (sin conectar)
   6 L_IS                         (sin conectar)
   7 VCC   ◄──────────────────── 3V3
   8 GND   ◄──────────────────── GND   ← GND común, obligatorio

 Encoder del motor                ESP32-S3
   Encoder+ (verde) ◄─────────── 3V3
   Encoder− (azul)  ◄─────────── GND
   Signal A (negro) ────────────► GPIO15
   Signal B (rojo)  ────────────► GPIO16
```

### Motor (6 cables)

Según la placa del encoder (**SCX-555**, foto del anuncio del motor), el conector de 6 pines tiene este orden. **Guiarse por la etiqueta de la placa, no por el color**: los colores no siguen ninguna convención (el rojo y el negro son **señales**, no alimentación) y pueden cambiar entre lotes.

| Pin del conector (de arriba hacia abajo en la foto) | Color en la foto | Va a |
|------------------|------------------|------|
| Signal B | Rojo | **GPIO16** |
| Signal A | Negro | **GPIO15** |
| Encoder+ | Verde | **3V3** del ESP32 |
| Encoder− | Azul | **GND** del ESP32 |
| Motor− | Amarillo | **M−** del IBT-2 |
| Motor+ | Blanco | **M+** del IBT-2 |

Comprobar con el multímetro antes de conectar (sin nada conectado):
1. **Motor:** entre Motor+ y Motor− se miden **pocos ohmios** (~1–10 Ω), y el valor cambia al girar el eje a mano. Entre cualquier otro par no se mide eso.
2. **Encoder:** entre Encoder+ y Encoder− no hay cortocircuito (más de unos kΩ).
3. Recién entonces conectar. **Nunca poner 12 V en los cables del encoder**, y nunca un cable del motor en un GPIO: se queman.

Si A y B quedan cruzados, el motor funciona igual pero las rpm salen negativas con duty positivo: poner `ENCODER_INVERTIDO = true` (o intercambiar GPIO15 y GPIO16).

### Por qué así

- **VCC del IBT-2 a 3V3, no a 5 V.** Ese VCC alimenta el 74HC244 de las entradas. A 5 V, el 74HC necesita al menos ~3,15 V para leer un "1", y el ESP32 da 3,3 V: queda al límite. A 3,3 V lee bien las señales del ESP32, y el BTS7960 acepta sus salidas (umbral de ~2 V). El motor sigue recibiendo los 12 V de B+.
- **GND común:** el GND de la tira de pines del IBT-2 es el mismo que B−. Sin unirlo al GND del ESP32, las señales no tienen referencia.
- **R_IS y L_IS** (sensado de corriente) quedan sin conectar: pueden subir por encima de 3,3 V ante una falla. Para medir corriente más adelante hace falta un divisor.
- **Encoder a 3V3:** así sus salidas son de 3,3 V y van directo a los GPIO. Los GPIO del S3 **no toleran 5 V**.
- **Pines:** GPIO4–7 y 15/16 no chocan con el IMU (8/9), el GPS (41/42/47) ni el lidar (14/21), así se pueden usar todos juntos más adelante. Solo se pierde la cámara, que no se usa. No son pines de strapping.
- **Pull-down de 10 kΩ en R_EN y L_EN (obligatorios).** Durante el reset, el bootloader y al flashear, los GPIO del ESP32 quedan como entradas flotantes, y el 74HC244 del IBT-2 podría leer un "1" y encender el puente con los 12 V conectados. Con las resistencias a GND, el driver queda deshabilitado (rueda libre) hasta que el firmware lo habilita. El firmware configura el driver apagado como primera instrucción del `setup()`. Si el módulo ya trae pull-down en esas entradas (medir con el multímetro, sin alimentar, entre EN y GND), no hacen falta.

### Fuente de 12 V / 3 A

- En vacío el motor consume unos cientos de mA, pero **al arrancar o si se traba** puede pedir varias veces más (el 555 es un motor de alta corriente). El programa sube el duty con una **rampa** (0 → 100 % en 2 s), que reduce el pico al arrancar con el motor libre.
- **La rampa no es un límite de corriente.** No hay medición de corriente (R_IS/L_IS sin conectar), y la protección del BTS7960 actúa recién cerca de 40 A. Si el eje se traba, la corriente la limitan la fuente y la resistencia del motor. Para eso hay un **corte por atasco**: con duty ≥ 40 %, si el encoder mide menos de 300 rpm del motor en el sentido pedido durante 1 s, frena. También salta si el encoder está desconectado o invertido (ver `include/motor_config.h`).
- **Cambio de sentido:** el programa nunca invierte con el motor girando. Baja a 0, frena, y espera a que el encoder marque el motor quieto (y al menos 300 ms en 0) antes de arrancar en el otro sentido. El escalón también exige el motor quieto.
- **No trabar el eje con la mano:** con la reductora 50:1 el par de salida es muy alto. Fijar el motor a la mesa (tiene 4 agujeros M3 al frente).
- Poner un fusible de 3–5 A en el +12 V si es posible.

## Cómo probar

1. Conectar todo **con la fuente de 12 V apagada**.
2. Flashear y abrir el monitor (el ESP32 por USB):
   ```bash
   cd ~/pruebas-esp32/06-motor-36gp555
   ~/.platformio/penv/bin/pio run -t upload -t monitor
   ```
   Después de flashear, pulsar **RST**. Si el USB nativo deja la placa en modo descarga, usar el otro conector (CH343) con `pio run -e uart -t upload -t monitor` (ver `04-gps-up501/README.md`).
3. **Verificar el encoder sin la fuente:** escribir `n` + Enter y girar el eje de salida a mano. Las cuentas deben cambiar y los niveles `A`/`B` alternar. Enter para salir.
4. Encender la fuente de 12 V. El motor arranca en rueda libre.
5. Escribir `20` + Enter: el motor debe girar despacio. Luego `50`, `100`, `-50`, `0`.

### Comandos (escribir y Enter)

| Comando | Qué hace |
|---------|----------|
| `<n>` | Duty de −100 a 100 % (el signo es el sentido), con rampa. Ej.: `30`, `-50` |
| `0` o `s` | Parar con rampa (queda frenado) |
| `x` | Parar **ya** (freno inmediato) |
| Enter | Corta el barrido, el escalón o el perfil (frena), o termina el conteo `n`. Cualquier otra línea hace lo mismo, y el programa avisa que la descartó |
| `l` | Rueda libre (driver deshabilitado) |
| `a` | **Barrido automático**: 10 %, 20 %, … 100 % hacia adelante y hacia atrás; mide las rpm en cada paso y estima la zona muerta |
| `e<n>` | **Escalón** a n % desde el reposo, sin rampa: imprime un CSV cada 10 ms durante 3 s. Ej.: `e50` |
| `n` | Contar vueltas girando la salida a mano (verificar la reducción) |
| `p` | **Perfil en escalera** sin rampa: +20, +40, +60, +80, +100 %, freno hasta que se detiene, y −20 … −100 %; 5 s por nivel; CSV `t_ms,duty_pct,cuentas` cada 10 ms. Opcional: `p <ms por nivel> <niveles>`, ej. `p 3000 25 50 75 100` |
| `?` | Ayuda |

El escalón es el único comando sin rampa: el pico de corriente al arrancar puede hacer que la fuente de 3 A entre en protección con valores altos. Empezar con `e30` o `e50`.

### Salida esperada

```
== Prueba en lazo abierto: motor 36GP-555 + IBT-2 (BTS7960) ==
IBT-2: RPWM=GPIO4, LPWM=GPIO5, R_EN=GPIO6, L_EN=GPIO7 | PWM 20000 Hz, 10 bits
Encoder: A=GPIO15, B=GPIO16 | 16 PPR x4 = 64 cuentas por vuelta del motor | reducción 50.0:1
...
>> Duty objetivo +50 % (rampa 50 %/s).
[   12.4 s] duty  +50.0 % (obj  +50) | motor   +3900 rpm | salida   +78.0 rpm | cuentas 20480
```

Barrido (`a`), valores aproximados:

```
   duty %  | rpm motor | rpm salida | cuentas/s
     +10   |        +0 |       +0.0 |         0
     +20   |     +1300 |      +26.0 |      1387
   ...
    +100   |     +7900 |     +158.0 |      8427
   Zona muerta: el motor empieza a girar entre 10 % y 20 % de duty.
```

Escalón (`e50`): copiar las líneas CSV a un archivo y graficarlas (rpm_salida vs t_ms). La columna `cuentas` se mide desde el inicio del escalón. El tiempo hasta el 63 % de la velocidad final es la **constante de tiempo** del motor, un dato para diseñar el control en lazo cerrado.

### Gráficos del barrido y del escalón (`tools/graficar.py`)

El script manda el comando por el serial, junta la salida, la guarda en `resultados/` (CSV y PNG) y la grafica. Usa `pyserial`, `numpy` y `matplotlib`, que ya están instalados en el Python del sistema.

**Cerrar antes el monitor de PlatformIO** (Ctrl+C): el puerto no puede estar abierto dos veces. La fuente de 12 V puede quedar encendida.

```bash
cd ~/pruebas-esp32/06-motor-36gp555
python3 tools/graficar.py barrido          # ~75 s: curva |duty| vs |rpm de salida|, adelante y atrás
python3 tools/graficar.py escalon 50       # para el motor, escalón a 50 % y gráfico rpm vs tiempo
python3 tools/graficar.py perfil           # ~53 s: escalera +-20..100 %, 4 gráficos + parámetros del motor
python3 tools/graficar.py perfil --segundos 3 --niveles 25 50 75 100
python3 tools/graficar.py escalon --archivo resultados/escalon50_....csv   # volver a graficar sin medir
```

- **Barrido:** además del gráfico, imprime por sentido la velocidad máxima, la pendiente (rpm por % de duty) y la **zona muerta** estimada, sacada de una recta ajustada a los puntos donde el motor gira.
- **Escalón:** imprime la velocidad final, el **tiempo al 63 %** (≈ constante de tiempo τ, incluye el pequeño retardo inicial), el tiempo de subida 10–90 % y la **ganancia estática** (rpm/%). Con τ y la ganancia se ajusta después el PID.
- **Perfil:** los mismos 4 gráficos que el análisis en MATLAB: posición completa, señal de control (escalera), posición de cada nivel desde 0 (continuo = positivo, punteado = negativo) y velocidad filtrada con un promedio exponencial (`--alpha`, por defecto 0,1) en pulsos/s y rpm de salida. Además calcula, por nivel, la velocidad de régimen (último 40 % del tramo) y **τ** (ajuste de primer orden a la velocidad sin filtrar, con scipy), y en conjunto la **ganancia K** (rpm/%), la **zona muerta** y el modelo `G(s) = K/(τs + 1)`. Guarda `perfil_*.csv`, `perfil_*.png`, `perfil_*_parametros.csv` y `perfil_*_parametros.png`.
  - A diferencia de la referencia, entre +100 % y −20 % **frena y espera a que el motor se detenga**: no invierte girando.
  - Desde parado, el primer nivel (20 %) puede no arrancar el motor por la fricción estática. Si pasa, usar `--niveles 30 45 60 80 100`.
  - La fuente tiene que entregar la corriente de los escalones sin rampa: en una fuente de laboratorio, límite de corriente en 3 A o más.
- **Ctrl+C** durante la medición manda `x` (freno) antes de salir. Si la prueba se corta (atasco, rechazo del escalón), el script muestra el motivo y también frena.
- Otro puerto: `--puerto /dev/ttyACM1`. Sin ventana (solo el PNG): `--sin-ventana`.

### Verificaciones

- **Sentido:** con duty positivo, las rpm deben salir **positivas**. Si salen negativas, poner `ENCODER_INVERTIDO = true` en `include/motor_config.h` (o cruzar A y B). Si el motor gira al revés de lo que se quiere, cruzar M+ y M− en la bornera.
- **Velocidad a 100 %:** cerca de **160 rpm** en la salida (≈ 8000 rpm en el motor) sin carga. Si da un múltiplo raro (por ejemplo ~300 o ~95), la reducción no es 50: medirla con `n`.
- **Reducción:** con `n`, marcar el eje de salida con cinta, girarlo a mano exactamente 5 vueltas y leer `vueltas del motor`. Reducción = vueltas del motor / 5 (debería dar ~50). Si da otra cosa, cambiar `REDUCCION`.
- **Linealidad:** fuera de la zona muerta, las rpm crecen casi en proporción al duty.
- **Simetría:** las rpm hacia atrás deben ser parecidas a las de adelante con el mismo duty.
- **Temperatura:** después de unos minutos a 100 %, el motor puede estar tibio; el disipador del IBT-2 apenas se calienta con estas corrientes.

## Drivers

- **`lib/BTS7960`:** duty con signo entre −1 y 1 por LEDC (20 kHz, 10 bits). Duty positivo = RPWM, negativo = LPWM, 0 = freno (los dos bornes a GND); `coast()` deshabilita el puente (rueda libre). Nunca activa los dos PWM a la vez.
- **`lib/EncoderPCNT`:** cuadratura x4 con el contador de pulsos (PCNT) del ESP32, por hardware: no usa interrupciones por flanco y no pierde pulsos aunque el motor gire rápido. Filtro de 10 µs contra el ruido del PWM. El contador de 16 bits no se borra durante la marcha: al llegar a ±32000 vuelve a 0 y una interrupción suma el desborde a un acumulador de 64 bits, así la posición es correcta aunque pase mucho tiempo entre lecturas.
- Si el PWM o el encoder no se inicializan, el programa lo informa al arrancar y rechaza los comandos al motor; `setDuty()` tampoco habilita el puente sin inicializar ni con valores no finitos.
- Sin `Serial` ni pines fijos: la prueba en lazo cerrado los puede enlazar con `symlink://../06-motor-36gp555/lib/...`.

## Solución de problemas

| Síntoma | Causa probable |
|---------|----------------|
| El motor no gira con ningún duty | Fuente apagada, falta el GND común, VCC del IBT-2 sin conectar o EN sin conectar. |
| Gira con duty positivo pero no con negativo (o al revés) | LPWM (o RPWM) mal conectado. |
| Gira pero las rpm dan 0 | Encoder sin alimentación o A/B desconectados. Probar con `n` girando a mano. |
| Las rpm saltan mucho o dan valores absurdos | Ruido del motor en los cables del encoder: alejarlos de los cables del motor o trenzarlos con GND. |
| Con `n` no cambian las cuentas | Algunos encoders Hall necesitan 5 V. **No conectar A/B directo al ESP32 con el encoder a 5 V.** Para medir el nivel alto: con A y B **desconectados** del ESP32, girar el eje a mano despacio hasta que cada fase quede quieta en alto y medirla con el multímetro (girando, el multímetro muestra un promedio y engaña). Si supera 3,3 V, poner un divisor en A y en B (10 kΩ en serie, 20 kΩ a GND). Ante la duda, poner el divisor directamente. |
| `>> Parado (ATASCO)` | Eje trabado, encoder desconectado (rpm 0) o invertido (rpm negativas con duty positivo: `ENCODER_INVERTIDO`). Si el motor gira bien y la zona muerta es alta, subir `ATASCO_DUTY_PCT`. |
| `Esperando que el motor se detenga...` y no arranca | El encoder mide movimiento con duty 0 (ruido o el eje girando por inercia). Esperar, o revisar el cableado del encoder. |
| La fuente se corta al arrancar | Pico de corriente: bajar `RAMPA_PCT_S` o `DUTY_MAX_PCT`, o no usar escalones altos. |
| El motor zumba en lugar de girar | Duty dentro de la zona muerta. Subir el duty. |

## Resultados

| Fecha | Prueba | Resultado | Observaciones |
|-------|--------|-----------|---------------|
| 2026-09-25 | Encoder con `n` | Cuentas OK | El eje de salida es muy duro de girar a mano (reductora 50:1): mejor verificar el encoder con el motor andando. |
| 2026-09-25 | Manual | Con 20 % **no arranca desde parado**; con 30 % sí. A 50 %: 3150 rpm del motor, 63 rpm de salida | rpm positivas con duty positivo (`ENCODER_INVERTIDO = false` correcto). |
| 2026-09-25 | Barrido (`a`) | Lineal: **1,61 rpm/%** (adelante) y 1,63 (atrás), **zona muerta ~11 %** con el motor girando, **máx 142 rpm** de salida a 100 % | Simétrico. Entre 90 % y 100 % casi no sube (139 → 142): posible caída de tensión de la fuente. El máximo confirma la reducción 50:1. `resultados/barrido_20260925_165422.*` |
| 2026-09-25 | Escalón 50 % (`e50`) | Final **62,6 rpm**, **τ ≈ 74 ms** (63 %), subida 10–90 % en 190 ms, ganancia 1,25 rpm/% (desde 0 %) | Primer orden. Después de ~300 ms termina de acomodarse lentamente (60 → 62,6 rpm en 1–2 s). `resultados/escalon50_20260925_165614.*` |

**Modelo para el lazo cerrado:** velocidad de salida ≈ 1,61 rpm/% × (duty − 11 %), con τ ≈ 74 ms. Para arrancar desde parado hace falta más duty que para mantenerlo girando (fricción estática), entre 20 % y 30 %.
