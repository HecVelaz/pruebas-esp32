# 24-bumpers-s3 — Bumpers frente, izquierdo y derecho de la base

Comprobar los tres bumpers de la base **sin mover las ruedas**: que cada uno esté en su pin, que lea suelto /
apretado y que no rebote ni dispare solo. En la **ESP32-S3** (Freenove N16R8 sobre su placa de expansión) con los
pines de la base (`docs/html/placa_s3.html`). Misma lógica de lectura que `18-finales-de-carrera`.

## Conexiones

| GPIO | Conecta a | Nota |
|---|---|---|
| 14 | Bumper **frente** | NA al GPIO, común a GND, pull-up interno |
| 21 | Bumper **izquierdo** (frontal izq.) | ídem |
| 39 | Bumper **derecho** (frontal der.) | ídem |
| 38 · 4 5 6 7 | EN y PWM de los IBT-2 de tracción | en bajo: ninguna rueda se mueve |

Microinterruptor V-152-1C25 (SPDT): **común (C) a GND y NA al GPIO**; NC sin conectar. Sin alimentación ni
capacitor (si alguno dispara sin tocarlo: 100 nF de su bornera a GND). Suelto = 1 (3,3 V), apretado = 0.
Pines en [`include/pins.h`](include/pins.h). Los 12 V no hacen falta: **apagados** antes de flashear o pulsar RST.

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/24-bumpers-s3
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor     # salir con Ctrl+C
```

Si el upload no conecta: BOOT apretado, pulsar RST, soltar BOOT; después del upload, RST. Si el puerto da
`Write timeout`, desenchufar y volver a enchufar el USB-C (2026-11-06 pasó eso y se arregló así).

Al arrancar muestra el estado de los tres. Después imprime una línea por cada cambio confirmado (3 lecturas
seguidas a 1 ms), con los rebotes que hubo en el medio. Comandos: `s` estado y contadores, `r` contadores a cero,
`?` ayuda.

## Pasos

| Paso | Qué hacer | Qué se espera |
|---|---|---|
| 1 | Arrancar sin tocar nada | los tres `suelto`. Si uno dice APRETADO: cableado a NC, cable a GND o está apretado de verdad |
| 2 | Apretar y soltar **solo** el del frente, 5 veces; `s` | solo cambia `frente`; 5 apretadas. Si cambia otro: cables cruzados |
| 3 | Lo mismo con el izquierdo y con el derecho | cada uno cambia solo el suyo |
| 4 | Mover los cables del bumper sin apretarlo | ningún cambio (si cambia: contacto flojo) |
| 5 | Apretar despacio, de a poco | un solo cambio, sin avisos de "dos veces en … ms" |
| 6 | Empujar la base contra un obstáculo por cada bumper (la chapa o paragolpes montado) | se activa antes de que la base choque |

## Estado

- [x] Compila (2026-11-06)
- [x] 1–3: los tres a mano (2026-11-06); 4–5 sin probar aparte
- [ ] 6: con el paragolpes montado

## Resultados

| Fecha | Paso | Resultado |
|---|---|---|
| 2026-11-06 | 1–3 | Los tres responden, cada uno cambia solo el suyo (frente 14, izq. 21, der. 39 bien cableados), 0 rebotes, sin avisos. Antes la S3 tenía el firmware micro-ROS (tramas `~` en el monitor, no era ruido). Salida en `resultados/bumpers_20261106_195403.txt` |
