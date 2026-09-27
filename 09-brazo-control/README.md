# 09-brazo-control — firmware del brazo (lazo cerrado)

Esqueleto del firmware del brazo en la **ESP32-WROOM-32D**: J1 (base), J2 (hombro), J3 (codo) y la pinza.
Control en **cascada**: posición (P con límite de velocidad y aceleración) → velocidad (PI + feedforward
con el modelo de la Entrega 5) → duty. Corre a 100 Hz en una tarea del núcleo 0; los comandos llegan por el
monitor serial. Más adelante la capa de comandos se cambia por micro-ROS (`/brazo/joint_cmd`,
`/brazo/joint_states`, ver `docs/ARQUITECTURA.md` del proyecto), sin tocar el control.

**Estado:** compila y las pruebas del control pasan en la PC (`pio test -e native`, motor simulado). **No
probado en placa.** Los valores marcados `MEDIR` en `include/brazo_config.h` son estimaciones.

## Estructura

| Archivo | Qué tiene |
|---|---|
| `lib/ControlArticulacion/` | Control de una articulación, **sin Arduino**: cascada, anti-windup, zona muerta, límites, atasco, descarte de saltos del encoder |
| `include/brazo_config.h` | Parámetros por articulación (encoder, transmisión, límites de la E4, modelo y PI de la E5), pose de cero, servos |
| `include/pins.h` | Pines (usa los 15 GPIO de uso general de la placa) |
| `src/main.cpp` | Drivers, tarea de control, comandos serie, homing, watchdog, CSV |
| `test/test_control/` | 10 pruebas con el modelo de la E5 simulado |

Drivers reutilizados por `symlink://`: `BTS7960` y `EncoderPCNT` (06), `L298N` (08).

## Conexiones

| Articulación | Motor / driver | Driver | Encoder |
|---|---|---|---|
| J1 base | 36GP-555 160 rpm + IBT-2 | RPWM 21, LPWM 22, R_EN+L_EN 23 | Hall A 34, B 35 (**pull-up externo 10 kΩ a 3V3**, encoder a 3V3) |
| J2 hombro | 5840-31ZY + L298N canal A | ENA 25, IN1 26, IN2 27 | A 32, B 33 (pull-up interno) |
| J3 codo | 5840-31ZY + L298N canal B | ENB 13, IN3 16, IN4 17 | A 18, B 19 (pull-up interno) |

| Otro | GPIO |
|---|---|
| Servo pinza (apertura) | 4 |
| Servo giro J4 | 14 (pulsa durante el arranque: el servo puede moverse un instante) |
| Final de carrera J1 / J2 | 36 / 39 (NA a GND, pull-up externo 10 kΩ); J3 sin pin |

- Servos alimentados con 5–6 V **aparte** (no del ESP32), GND común.
- Pull-down de 10 kΩ en EN del IBT-2 y en ENA/ENB del L298N (sin ellos: **apagar los 12 V antes de flashear o
  pulsar EN**).
- Canales LEDC: 0-1 IBT-2 (20 kHz), 2-3 y 4-5 L298N (1 kHz), 6-7 servos (50 Hz). PCNT: unidades 0, 1 y 2.

## Uso

```bash
PIO=~/.platformio/penv/bin/pio
$PIO test -e native               # pruebas del control en la PC
$PIO run -t upload -t monitor     # flashear (12 V apagados) y abrir el monitor
```

Secuencia de primera prueba (una articulación a la vez, con la fuente limitada a 2–3 A):

1. `d 2 40` repetido: mover J2 en lazo abierto y ver con `e` que `w` tenga el **mismo signo** que el duty. Si no,
   cambiar `ENC_INVERTIDO` en `brazo_config.h`.
2. `v 2 30 3`: lazo de velocidad solo (sirve para la **E6**). Con `log 2` antes, se obtiene el CSV de la respuesta.
3. Llevar el brazo a la pose de referencia con `d`, y `z` para fijar el cero.
4. `j 2 60`, `m 0 90 0`: movimientos en posición. `s` detiene, `l` libera, `r` borra fallas.

`?` muestra todos los comandos.

## Protecciones

- Sin cero, las articulaciones no aceptan movimientos en posición.
- Objetivos recortados a los límites de la E4. Fuera de los límites + 5° → falla y freno.
- Atasco (duty alto sin avanzar en el sentido pedido) → falla y freno.
- Lecturas imposibles del encoder (por ejemplo el salto falso de 32 000 cuentas pendiente en `EncoderPCNT`) se
  descartan; 5 seguidas → falla.
- `d` se corta a los 500 ms si no se repite; `v` a los segundos pedidos.
- Watchdog opcional (`wd <ms>`): sin comandos, todo se detiene y sostiene. Para cuando mande la Pi.
- J2 y J3 son de sin fin: con duty 0 (freno) el brazo queda sostenido sin corriente.

## Pendiente (MEDIR / hacer)

- [ ] Relación de transmisión de cada articulación (`relacion`, hoy 1).
- [ ] Signo de los encoders de J1 y J3 (`ENC_INVERTIDO`).
- [ ] Zona muerta real del 5840-31ZY (`u0`, hoy 30 %; arranca entre 20 y 40 %).
- [ ] Pose de referencia (`POSE_CERO`) y ángulos de los finales de carrera (`HOME_Q`).
- [ ] Pulsos de los servos (`PINZA_US_*`, `GIRO_US_*`).
- [ ] Ajustar `kpPos`, `velMax`, `accMax` con el brazo armado y con carga.
- [ ] Capa de comandos por micro-ROS.

## Resultados

| Fecha | Prueba | Resultado |
|---|---|---|
| 2026-09-27 | `pio test -e native` (10 pruebas, motor simulado con el modelo de la E5) | OK |
| | Signo de encoders J1/J3 | — |
| | Lazo de velocidad J1/J2 en placa | — |
| | Movimiento en posición con carga | — |
