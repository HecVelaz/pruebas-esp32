# 23-servos-wroom

Pinza y muñeca juntas en la **ESP32-WROOM-32D del brazo**, con los pines del robot. Cada servo tiene los comandos y
los valores medidos de su prueba suelta: la pinza los de [`21-servo-mg996r-pinza`](../21-servo-mg996r-pinza/README.md)
y la muñeca los de [`22-servo-muneca`](../22-servo-muneca/README.md). Driver `ServoPWM` de 21 (`symlink://`).

## Placa y conexiones

WROOM-32D en la HW-394 (CH340, `/dev/ttyUSB0`) sobre el shield de borneras, conexionado de
`recolector_de_frutas/docs/html/placa_wroom.html` (= `firmware/brazo/include/pins.h`; `todo.html` del 21/10, al día).

| Servo | Señal | Alimentación |
|---|---|---|
| Apertura de la pinza (MG996R) | **GPIO 4** (D4, fila de abajo), LEDC canal 0 | 6 V del buck 3, GND común con la WROOM |
| Giro J4 = muñeca | **GPIO 14** (D14, fila de arriba), LEDC canal 1 | ídem |

- El GPIO 14 pulsa durante el arranque: la muñeca puede dar un tirón al encender o resetear.
- El firmware pone el **EN 23** de los IBT-2 y sus seis PWM (21, 22, 25, 26, 16, 17) en LOW: los motores no se mueven.
- Los 6 V salen del bus de 12 V (buck 3): apagar los 12 V para flashear y prenderlos para probar.

## Probar

```bash
PIO=~/.platformio/penv/bin/pio
$PIO run -t upload        # con los 12 V apagados
$PIO device monitor       # 115200; después prender los 12 V
```

## Comandos

`p <cmd>` = pinza, `m <cmd>` = muñeca; `<cmd>` como en 21 / 22:

| `<cmd>` | Qué hace |
|---|---|
| `p <us>` | ir a un pulso (dentro de la franja) |
| `0` | cero (1500 µs) |
| `+` / `-` | ±20 µs |
| pinza `a` / `c`, `C` | abrir (500 µs, fijo) / cerrar (1250 µs); `C` guarda la actual como cerrado |
| muñeca `a` / `b`, `A` / `B` | 90° horario (860 µs) / antihorario (2150 µs); guardar la actual |
| `w <°/s>` | velocidad de la rampa (45 °/s al arrancar; 0 = salto) |
| `l <min> <max>` | franja (500..2500) |
| `s` / `e` | soltar / estado |

Sin prefijo: `s` suelta los dos, `e` estado de los dos, `?` ayuda. Ejemplos: `p a`, `m b`, `p p 1300`.
Los dos arrancan sueltos: el primer comando de cada uno salta directo (conviene empezar con `p 0` / `m 0`).
Los bytes no imprimibles que lleguen por el serie se descartan; si un comando falla, se imprimen sus bytes en hex.

## Resultados (2026-11-06)

- **Identificación:** con la primera versión (meneo de ±150 µs en un pin, el otro suelto) la muñeca respondió en
  GPIO 14; la pinza al principio no se movió en GPIO 4 y después sí (cable revisado por el usuario).
- **Muñeca:** se mueve con los valores de 22.
- **Pinza: el brazo del servo quedó ~45° corrido** respecto de 21 al montarla: el dedo se ve en su cero con
  **1000 µs** y `p a` (500 µs) abre solo **45°** en lugar de 90°. El servo no baja de 500 µs, así que no se arregla por
  software: hay que **volver a calzar el brazo del servo** (con `p 0` = 1500 µs, 12 V apagados, dedo en su cero;
  ~3 dientes del estriado, ~14° cada uno). Se probó un firmware con el cero en 1000 µs y se volvió a los valores de 21
  (cero 1500 µs), que es lo que queda en el código: falta flashearlo, recalzar y confirmar `p a` = 90° (73 mm) y
  `p c` con el limón.
