# 03-hcsr04 — Los tres HC-SR04 de la base

Comprobar los tres ultrasonidos de la base **sin mover las ruedas**: que cada uno esté en su TRIG y su ECHO, que mida
bien la distancia y que no se pisen los ecos entre ellos. En la **ESP32-S3** (Freenove N16R8 sobre su placa de
expansión) con los pines de la base (`docs/html/placa_s3.html`, `recolector_de_frutas/firmware/base/include/pins.h`).

Usa el **mismo driver que la base**: `lib/HCSR04` es una copia de `recolector_de_frutas/firmware/lib/HCSR04` (commit
`b2b5b2d`): ECHO medido por interrupción (sin `pulseIn`), un sensor por vez para que no tomen el eco del otro.

## Conexiones

| Sensor | TRIG | ECHO | Nota |
|---|---|---|---|
| Frente (centro) | 10 | 11 | ECHO por la perforada: 1 kΩ en serie, 2 kΩ a GND |
| Esquina izquierda | 41 | 12 | ídem |
| Esquina derecha | 42 | 13 | ídem |
| Tracción | EN 38 · PWM 4 5 6 7 | | en bajo: ninguna rueda se mueve |

VCC de los tres a **5 V**, GND común. El ECHO sale a 5 V y los GPIO de la S3 no lo toleran: **sin el divisor no se
conecta**. Con el divisor llega ~3,3 V. Pines en [`include/pins.h`](include/pins.h). Los 12 V no hacen falta:
**apagados** antes de flashear o pulsar RST.

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/03-hcsr04
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor     # salir con Ctrl+C
```

Si el upload no conecta: BOOT apretado, pulsar RST, soltar BOOT; después del upload, RST. Si el puerto da
`Write timeout`, desenchufar y volver a enchufar el USB-C.

Dispara un sensor cada 60 ms (cada uno se mide cada 180 ms) e imprime las tres distancias cada 0,5 s:

- `23.4 cm`: medición válida.
- `sin eco (>4m)`: el sensor respondió pero no vio nada a menos de ~4,3 m (normal apuntando al aire).
- `NO RESPONDE`: no hubo pulso de ECHO: falta 5 V o GND, TRIG o ECHO mal, o el divisor abierto.

Al arrancar avisa si un ECHO está en 1 sin disparar o si un sensor no responde. Comandos: `e [n]` estadística de
n mediciones por sensor (media, desvío, mín, máx; 20 por defecto), `p` pausa / sigue, `?` ayuda.

## Pasos

| Paso | Qué hacer | Qué se espera |
|---|---|---|
| 1 | Arrancar sin nada delante | ningún `OJO`; los tres `sin eco` o la distancia a la pared |
| 2 | Tapar con la mano **solo** el del frente a ~10 cm | solo cambia `frente` (~10 cm). Si cambia otro: TRIG o ECHO cruzados |
| 3 | Lo mismo con el izquierdo y con el derecho | cada uno cambia solo el suyo |
| 4 | Una caja (superficie plana) a 20, 50 y 100 cm de cada sensor, medidos con cinta; `e 30` en cada una | media a ±1 cm de la cinta, desvío < 0,5 cm |
| 5 | Caja a ~2 cm | el HC-SR04 no mide por debajo de ~2–3 cm: anotar desde dónde mide |
| 6 | Caja delante del frente solamente, a 30 cm | los de las esquinas no "ven" el eco del frente (no se pisan) |

## Estado

- [x] Compila (2026-11-06)
- [x] 1–3: cada sensor en su pin (2026-11-06, según el usuario; salida sin guardar)
- [ ] 4–6: distancias con cinta, mínimo, sin interferencia

## Resultados

| Fecha | Paso | Resultado |
|---|---|---|
| 2026-11-06 | 1–3 | Los tres responden, cada uno en su TRIG / ECHO (frente 10/11, izq. 41/12, der. 42/13), con el driver de la base. Sin salida guardada ni distancias con cinta |
