# 16-l298n-j3-c3 — Motor de J3 con un L298N y una ESP32-C3-MINI-1

La misma prueba que `15-ibt2-j3-c3`, pero con un **L298N** en lugar del IBT-2. Sirve para ver si el motor de J3
se mueve con otro driver, **con el motor montado en el brazo**. Si con el L298N se mueve y con el IBT-2 no, el
problema está en el IBT-2 (o en su cableado). Mismo C3, mismos pines, mismos comandos y mismas protecciones.
Conexiones dibujadas en [`conexiones.html`](conexiones.html).

Solo hay **pulsos cortos de duty fijo**. Para que el codo no dé una vuelta sin querer:

| Protección | Valor (`include/config.h`) |
|---|---|
| Duty máximo | 60 % (arranca en 25 %) |
| Pulso máximo | 500 ms (arranca en 150 ms); a 60 % son ~20° como mucho |
| Límite por encoder | ±15° desde el cero (`z`): corta el pulso y no deja seguir alejándose, solo volver |
| Sin cuentas | si el encoder no cuenta en 150 ms, corta (motor trabado, driver muerto o encoder mal) |
| Respaldo por tiempo | pulsos + menos − desde el último `z`: máx 3 s (0,6 s sin encoder, `se`) |
| Corte manual | cualquier tecla durante un pulso lo corta |
| En reposo | freno (ENA alto, IN1 = IN2 = 0) |

## Conexiones

Pines en [`include/pins.h`](include/pins.h). Canal A del L298N.

| C3 | Conecta a | Nota |
|---|---|---|
| GPIO 4 | L298N IN1 | PWM en el sentido + |
| GPIO 5 | L298N IN2 | PWM en el sentido − |
| GPIO 6 | L298N ENA (pin de señal) | **sacar el jumper de ENA**; sin pull-down: ver Seguridad |
| GPIO 7 | encoder A (negro) | |
| GPIO 10 | encoder B (rojo) | |
| 3V3 | encoder (verde) | **no** al L298N |
| GND | L298N GND, encoder (azul) | **GND común** con la fuente |

L298N: bornera de potencia **12V** al + de la fuente y **GND** al − (y al GND del C3), borne **5V sin conectar**,
jumper **5V-EN puesto** (el regulador del módulo alimenta su lógica desde los 12 V). OUT1 / OUT2 al motor
(blanco = Motor+ a OUT1, amarillo = Motor− a OUT2). IN3, IN4 y ENB sin conectar.

- **ENA tiene dos pines y un jumper:** uno es 5 V y el otro la entrada. Sacar el jumper y conectar el GPIO 6 **solo
  a la entrada**. Para saber cuál es: con los 12 V prendidos y el jumper afuera, el que da ~5 V contra GND es el de
  5 V, **no usar ese** (dañaría el GPIO).
- **Niveles:** el L298N toma como alto desde ~2,3 V, así que anda directo con los 3,3 V del C3. A diferencia del
  IBT-2, el L298N no tiene pin VCC de lógica en el conector: no conectar el 3V3 del C3 al módulo.
- **El borne 5V del L298N no va al C3:** el C3 se alimenta por el USB.

No usar en el C3: 2, 8, 9 (strapping), 11–17 (flash), 18/19 (USB), 20/21 (UART0). GPIO de 3,3 V: no toleran 5 V.

## Seguridad

- **Fuente de laboratorio a 12 V con límite de 2 A** (lo máximo por canal del L298N), no la LiPo.
- **El L298N pierde ~2 V** (bipolar): con 12 V el motor recibe ~10 V a 100 %. Hace falta más duty que con el IBT-2
  para arrancar. Calienta: tocar el disipador después de varios pulsos.
- **ENA sin pull-down**, como el EN de `15`: mientras el C3 arranca, se flashea o se resetea, ENA, IN1 e IN2 flotan.
  El orden es obligatorio: USB primero, esperar la ayuda y `PWM 1000 Hz OK` en el monitor, recién ahí prender los
  12 V; **apagar los 12 V antes de flashear, pulsar RST o desenchufar el USB.** Para cubrir un reset inesperado,
  poner 10 kΩ del GPIO 6 a GND.
- **J3 no sostiene el antebrazo sin corriente** (planetario): con `x` o sin los 12 V cae. Frenado cae más despacio.
  Tener el antebrazo con la mano o apoyado.
- **J2 no está conectada a nada:** el hombro queda suelto. Apoyarlo en un tope durante toda la prueba.
- Antes de `+` / `-`, poner el codo en una posición con margen para los dos lados y hacer `z`.

## Cómo flashear y abrir el monitor

Igual que en `15`: placa con **CH340**, entorno `c3_uart` (el de por defecto). Mirar el puerto con
`ls /dev/ttyUSB*` (si se desenchufa con el monitor abierto, puede volver como `ttyUSB1`).

```bash
cd ~/Documentos/pruebas-esp32/16-l298n-j3-c3
PIO=~/.platformio/penv/bin/pio
$PIO run -t upload --upload-port /dev/ttyUSB0
$PIO device monitor --port /dev/ttyUSB0
```

## Comandos

| Comando | Qué hace |
|---|---|
| `+` / `-` | pulso positivo / negativo con el duty y la duración actuales |
| `p <duty> [ms]` | pulso suelto con duty con signo, sin cambiar `d` ni `t` (`p 30 100`, `p -30 100`) |
| `d <pct>` | duty (1–60 %) |
| `t <ms>` | duración (10–500 ms) |
| `z` | cero acá y borra el tiempo neto |
| `e` | cuentas, grados, niveles de A y B, tiempo neto y signo |
| `f` / `x` | freno (ENA alto) / rueda libre (ENA bajo) |
| `m <duty>` | diagnóstico **con el motor desconectado** de OUT1 / OUT2: pide confirmar con `s`, mantiene el duty (con signo) hasta una tecla (máx 30 s) para medir con el multímetro; corta si el encoder cuenta |
| `se` | apaga/prende el control del encoder (para probar sin encoder) |
| `?` | ayuda |

Qué medir con `m` (multímetro en DC, 12 V prendidos, motor desconectado):

| Qué | `m 30` | `m -30` |
|---|---|---|
| IN1 a GND | ~1 V | 0 V |
| IN2 a GND | 0 V | ~1 V |
| ENA a GND | ~3,3 V | ~3,3 V |
| OUT1 a OUT2 | ~+3 V | ~−3 V |

## Pasos

Con los 12 V **apagados**, el antebrazo apoyado y el monitor abierto mostrando la ayuda:

1. **Encoder.** `e` varias veces girando el codo a mano un poco: las cuentas tienen que cambiar y A/B estar en 0 o 1.
   (~124 cuentas por grado del codo.) Si no cambian, revisar el encoder o seguir con `se` (sin encoder).
2. Codo en una posición con margen, `z`. **Prender los 12 V.**
3. `+` con 25 % y 150 ms. Si no se mueve (`SIN CUENTAS`), subir de a 5 %: `p 30 150`, `p 35 150`… Anotar el duty
   mínimo y si la pinza sube o baja.
4. Lo mismo con `-` (o `p -30 150`…). Bajando, por el peso, alcanza con menos duty que subiendo.
5. Repetir unas veces en cada sentido. Con `t 300` se ven movimientos más claros (siguen limitados a ±15°).

## Resultados

| Fecha | Duty mínimo subiendo | Duty mínimo bajando | ° por pulso + / − | ¿Se mueve? | Notas |
|---|---|---|---|---|---|
| | | | | | |
