# 18-finales-de-carrera — Finales de carrera de J1, J2 y J3

Comprobar los tres finales de carrera del brazo (los que va a usar el homing) **sin mover ningún motor**: que cada
uno esté en su pin, que lea suelto / apretado y que no rebote ni dispare solo. En la **ESP32-WROOM-32D** con el
conexionado v0.5 del robot (mismos pines que `12`–`14` y `17`), salvo los finales de carrera de J2 y J3, que
quedaron cruzados al cablear (2026-10-30): **J2 en el GPIO 27 y J3 en el 33** (en v0.5 era al revés).

Uno por articulación alcanza: da la referencia del homing (la articulación va despacio hacia él, lo toca y fija
su ángulo); el otro extremo lo cuidan los límites de software.

## Conexiones (v0.5)

| GPIO | Conecta a | Nota |
|---|---|---|
| 32 | Final de carrera **J1** (base) | NA a GND, pull-up interno |
| 27 | Final de carrera **J2** (hombro) | NA a GND, pull-up interno |
| 33 | Final de carrera **J3** (codo) | NA a GND, pull-up interno |
| 23 · 21 22 25 26 16 17 | EN y PWM de los IBT-2 | en bajo: ningún motor se mueve |

Microinterruptor V-152-1C25 (SPDT, rodillo): **común (C) a GND y NA al GPIO**; NC sin conectar. Sin alimentación
ni capacitor. Suelto = 1 (3,3 V), apretado = 0. Pines en [`include/pins.h`](include/pins.h).

**Con el EN en bajo, J2 y J3 quedan sueltas y no se sostienen:** brazo apoyado en su tope antes de enchufar.
Los 12 V pueden quedar apagados (no hacen falta).

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/18-finales-de-carrera
~/.platformio/penv/bin/pio run -t upload
tools/monitor.sh                             # salir con Ctrl+]
```

Al arrancar muestra el estado de los tres. Después imprime una línea por cada cambio confirmado (3 lecturas
seguidas a 1 ms, como el homing del robot), con los rebotes que hubo en el medio. Comandos: `s` estado y contadores,
`r` contadores a cero, `?` ayuda.

## Pasos

| Paso | Qué hacer | Qué se espera |
|---|---|---|
| 1 | Arrancar sin tocar nada | los tres `suelto`. Si uno dice APRETADO: cableado a NC, cable a GND o está apretado de verdad |
| 2 | Apretar y soltar a mano **solo** el de J1, 5 veces; `s` | solo cambia `J1`; 5 apretadas. Si cambia otro: cables cruzados |
| 3 | Lo mismo con J2 y con J3 | cada uno cambia solo el suyo |
| 4 | Mover los cables del final de carrera sin apretarlo | ningún cambio (si cambia: contacto flojo) |
| 5 | Apretar despacio, de a poco | un solo cambio, sin avisos de "dos veces en … ms" |
| 6 | (Después, con `17`) llevar cada articulación con el joystick hasta su final de carrera | se activa antes del tope mecánico; anotar el ángulo |

## Estado

- [x] Compila
- [x] 1–5: los tres a mano (2026-10-30: funcionan los tres, según el usuario; salida sin guardar)
- [ ] 6: con el brazo moviéndose

## Resultados

| Fecha | Paso | Resultado |
|---|---|---|
| 2026-10-30 | 1–5 | Los tres finales de carrera responden apretándolos a mano. Con eso se vio que J2 y J3 estaban cruzados: J2 está en el 27 y J3 en el 33 (pines cambiados en `12`–`14`, `17`, `18`) |
