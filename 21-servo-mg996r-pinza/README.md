# 21-servo-mg996r-pinza

Servo **MG996R** que abre y cierra el dedo de la pinza. Abre siempre a **500 µs (73 mm)** y cierra a
**1250 µs** (aprieta bien un limón de 51 mm); ver [Posiciones](#posiciones-medidas-el-2026-11-06).

## Placa

ESP32 DevKit "ESP32S" (rotulada XXSR69): **ESP32-D0WD-V3** rev 3.1, 4 MB de flash, USB-serie **CP2102**
(`10c4:ea60`, `/dev/ttyUSB0`), MAC `78:42:1c:69:1f:18`. `board = esp32dev`. Si también está conectada la WROOM
(CH340), una de las dos queda en `/dev/ttyUSB1`: mirar `ls /dev/ttyUSB*` y usar `--upload-port`.

## Conexiones

| Servo MG996R | Va a |
|---|---|
| Marrón (GND) | GND de la fuente externa **y** GND del ESP32 (masa común) |
| Rojo (V+) | **+5 a 6 V de una fuente externa, 3 A o más** (no del pin 5V/VIN del ESP32) |
| Naranja (señal) | **GPIO 18** (`PIN_SERVO` en `include/pins.h`) |

- Electrolítico de **470–1000 µF** entre V+ y GND, cerca del servo (picos de hasta ~2,5 A al trabarse).
- La señal de 3,3 V del ESP32 alcanza para el MG996R.
- PWM con el LEDC: 50 Hz, 16 bits (~0,3 µs por paso).
- `w` se pasa a µs/s con la conversión nominal del MG996R, 500–2500 µs ≈ 0–180° (11,1 µs por grado).
  Verificada el 2026-11-06: de 1500 a 1000 µs gira 45° en sentido **horario** (grados negativos = horario).

## Probar

```bash
PIO=~/.platformio/penv/bin/pio
$PIO run -t upload        # con la fuente del servo apagada
$PIO device monitor       # 115200
```

El firmware arranca **SUELTO** (sin pulsos). Prender la fuente del servo después de abrir el monitor.

| Comando | Qué hace |
|---|---|
| `0` | ir al **cero** de este servo: **1500 µs** (`US_CERO`, medido el 2026-11-06) |
| `p <us>` | ir a un ancho de pulso (se recorta a la franja) |
| `+` / `-` | mover 20 µs |
| `a` / `c` | abrir (**siempre 500 µs**, 73 mm) / cerrar (al arrancar 1250 µs) |
| `C` | guardar la posición actual como cerrado (se pierde al reiniciar; abierto es fijo) |
| `w <°/s>` | velocidad angular de la rampa (al arrancar 45 °/s; `0` = salto directo; `w` solo muestra la actual) |
| `l <min> <max>` | franja permitida, dentro de 500–2500 (al arrancar 500–2000) |
| `s` | soltar: sin pulsos, el servo no hace fuerza |
| `e` / `?` | estado / ayuda |

Respuestas: `VA <us> (<grados>)` al aceptar un destino, `LLEGO <us> (<grados>)` al terminar la rampa (es el pulso mandado, no la
posición medida: el MG996R no tiene realimentación), `AVISO`, `ERROR`, `ESTADO …`. Los grados se cuentan
desde el cero (1500 µs) con la conversión nominal: positivos hacia pulsos más largos.

### Paso a paso

1. `0` o `p 1500` (el primer pulso salta directo, porque no se sabe dónde está el servo). Debería quedar firme.
2. `w 20`, después `a` (abre a 500 µs) y `c` (cierra a 1250 µs).
3. Para otra fruta: desde abierto, acercarse con `+` de a 20 µs hasta que apriete bien, y `C`. Anotar abajo.
4. Probar `a` / `c` varias veces y con distintas `w` (por ejemplo `w 20`, `w 45`, `w 90`).

## Posiciones (medidas el 2026-11-06)

| Posición | Pulso | Ángulo desde el cero | Apertura / uso |
|---|---|---|---|
| Cero | **1500 µs** | 0° | referencia (`US_CERO`) |
| **Abierto** (fijo, siempre) | **500 µs** | ~90° horario | **73 mm** (`US_ABIERTO`, comando `a`) |
| Cerrado | **1250 µs** | ~22,5° horario | aprieta bien un **limón de 51 mm** de diámetro (`usCerrado`, comando `c`) |

- `p 1000` gira 45° horario desde el cero: 500 µs / 45° = 11,1 µs/°, igual a la conversión nominal.
- 500 µs es el mínimo de fábrica del MG996R: no pedir menos. La franja al arrancar es 500–2000 µs.

## Resultados

| Fecha | Abierto (µs) | Cerrado (µs) | w (°/s) | Observaciones |
|---|---|---|---|---|
| 2026-11-06 | | | | Cero = 1500 µs (`p 1500`) |
| 2026-11-06 | | | | `p 1000` → 45° horario desde el cero (500 µs / 45° = 11,1 µs/°: coincide con la conversión nominal) |
| 2026-11-06 | 500 (73 mm) | 1250 (limón de 51 mm, apretado bien) | | Abierto queda fijo en 500 µs |
