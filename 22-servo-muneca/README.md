# 22-servo-muneca

Servo de la **muñeca** del brazo (giro). Mismo programa de prueba que `21-servo-mg996r-pinza`, con el mismo driver
(`ServoPWM`, enlazado por `symlink://`), pero con dos posiciones guardables **A** y **B** en lugar de abierto /
cerrado. Objetivo: encontrar el cero, los extremos sin forzar y las posiciones útiles de la muñeca.

**Modelo del servo: a completar.** Gira ~7,2 µs/° (no 11,1 como un servo de 180°): probablemente de 270°.

## Placa

La misma ESP32 DevKit "ESP32S" (CP2102) que la 21. El número de `/dev/ttyUSBx` cambia cada vez que se
reconecta: usar la ruta fija `/dev/serial/by-id/usb-Silicon_Labs_CP2102_…` (abajo), que siempre apunta al CP2102.

## Conexiones

| Servo | Va a |
|---|---|
| Marrón / negro (GND) | GND de la fuente externa **y** GND del ESP32 (masa común) |
| Rojo (V+) | **+5 a 6 V de una fuente externa** (no del pin 5V/VIN del ESP32) |
| Naranja / amarillo (señal) | **GPIO 19** (`PIN_SERVO` en `include/pins.h`) |

- GPIO 19 (y canal LEDC 1) para que pueda quedar conectado junto con el servo de la pinza (GPIO 18, canal 0).
- Electrolítico de 470–1000 µF entre V+ y GND, cerca del servo.

## Probar

```bash
cd ~/Documentos/pruebas-esp32/22-servo-muneca
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
~/.platformio/penv/bin/pio device monitor --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Arranca **SUELTO** (sin pulsos). Prender la fuente del servo después de abrir el monitor.

| Comando | Qué hace |
|---|---|
| `0` | ir al cero (**1500 µs**, medido) |
| `p <us>` | ir a un ancho de pulso (se recorta a la franja) |
| `+` / `-` | mover 20 µs (~2,8°) |
| `a` / `b` | ir a **90° horario (860 µs)** / **90° antihorario (2150 µs)** |
| `A` / `B` | guardar la posición actual como A / B (se pierde al reiniciar) |
| `w <°/s>` | velocidad angular de la rampa (al arrancar 45 °/s; `0` = salto directo; `w` solo muestra la actual) |
| `l <min> <max>` | franja permitida, dentro de 500–2500 (al arrancar 860–2150, ±90°) |
| `s` | soltar: sin pulsos, el servo no hace fuerza |
| `e` / `?` | estado / ayuda |

Respuestas: `VA`, `LLEGO`, `GUARDADO`, `ESTADO`, `AVISO`, `ERROR`, con el pulso y los grados desde el cero.

### Paso a paso

1. `0` (el primer pulso salta directo). `w 20`.
2. `a` (90° horario), `0`, `b` (90° antihorario), `0`. Repetir con distintas `w`.
3. Si hace falta más recorrido: `l 500 2500` y acercarse con `+` / `-`, parando si zumba o toca un tope.

## Posiciones (medidas el 2026-11-06)

| Posición | Pulso | Desde el cero |
|---|---|---|
| Cero | **1500 µs** | 0° (`US_CERO`) |
| 90° horario | **860 µs** | −640 µs → 7,11 µs/° (`US_90_HORARIO`, comando `a`) |
| 90° antihorario | **2150 µs** | +650 µs → 7,22 µs/° (`US_90_ANTIHORARIO`, comando `b`) |

Los grados que muestra el firmware usan la conversión de cada lado; la velocidad `w` usa el promedio (7,17 µs/°).
Grados negativos = horario.

## Resultados

| Fecha | Pulso (µs) | Ángulo / posición medida | w (°/s) | Observaciones |
|---|---|---|---|---|
| 2026-11-06 | 1500 | 0° | | Cero |
| 2026-11-06 | 860 | 90° horario | | |
| 2026-11-06 | 2150 | 90° antihorario | | |
