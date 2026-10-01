# Conexiones — motores, drivers, finales de carrera y bumpers

Tomado de `recolector_de_frutas/docs/CONEXIONES.md` v0.3 y de los `pins.h` de `firmware/brazo` y `firmware/base`
(los pines son los mismos que usará el robot). Banco de pruebas: **fuente de laboratorio de 12 V** en lugar de la
batería y los buck.

Para el lazo cerrado de esta carpeta (pasos 0 a 5) hacen falta solo **A1 y A2** (J1 y J2 en la WROOM). El resto
queda listo para después.

## Fuente de laboratorio

- **12,0 V** y **límite de corriente en 3 A** aunque la fuente dé más. El límite no es para la fuente sino para
  proteger drivers y cables: con un cable mal puesto o un motor trabado corta en 3 A en vez de quemar algo.
  El L298N aguanta 2 A por canal.
- Si al arrancar la fuente entra en CC (baja la tensión y se enciende el indicador de corriente), subir a 4–5 A.
  Con 3 A ya se hicieron todas las pruebas de lazo abierto (`06`, `08`).
- **GND común:** el negativo de la fuente, el GND de cada driver y el GND del ESP32 unidos.
- **Apagar los 12 V antes de flashear o pulsar EN/RST** hasta que estén los pull-down de los EN.

---

## A. ESP32-WROOM-32D · brazo (`/dev/ttyUSB0`)

### A1. J1 base giratoria: 36GP-555 + IBT-2

```
 Fuente 12 V (+) ─────────────► IBT-2 B+
 Fuente 12 V (−) ─────────────► IBT-2 B−

 Motor 36GP-555 (placa SCX-555)    IBT-2
   blanco  (Motor+) ─────────────► M+
   amarillo(Motor−) ─────────────► M−      (invertirlos solo cambia el sentido)

 IBT-2 (conector de 8 pines)       ESP32-WROOM
   VCC   ◄────────────────────── 3V3
   GND   ◄────────────────────── GND
   RPWM  ◄────────────────────── GPIO21
   LPWM  ◄────────────────────── GPIO22
   R_EN ─┬─ L_EN (puenteados) ◄─┬─ GPIO23
                                └─ 10 kΩ ─ GND          (pull-down)
   R_IS, L_IS                     sin conectar

 Encoder (placa SCX-555)           ESP32-WROOM
   verde (Encoder+) ◄──────────── 3V3   (NO a 5 V: las salidas quedarían a 5 V)
   azul  (Encoder−) ◄──────────── GND
   negro (Signal A) ──────┬─────► GPIO34
                          └ 10 kΩ ─ 3V3                  (pull-up OBLIGATORIO)
   rojo  (Signal B) ──────┬─────► GPIO35
                          └ 10 kΩ ─ 3V3                  (pull-up OBLIGATORIO)
```

GPIO 34–39 de la WROOM no tienen pull-up interno: sin esas dos resistencias el encoder no cuenta.

### A2. J2 hombro: 5840-31ZY + L298N canal A

```
 Fuente 12 V (+) ─────────────► L298N 12V
 Fuente 12 V (−) ─────────────► L298N GND
                    (libre)     L298N 5V   ← jumper "5V-EN" PUESTO (el regulador alimenta la lógica)

 Motor J2 (2 cables) ─────────► OUT1 y OUT2   (el orden solo cambia el sentido)

 L298N                            ESP32-WROOM
   ENA (pin de señal) ◄─┬──────── GPIO25    ← SACAR el jumper de ENA
                        └ 10 kΩ ─ GND        (pull-down)
   IN1  ◄──────────────────────── GPIO26
   IN2  ◄──────────────────────── GPIO27
   GND  ◄──────────────────────── GND        ← GND común, obligatorio

 Encoder 38S6G5 (J2)              ESP32-WROOM
   rojo  (+) ◄──────────────────── 5V (VIN)
   negro (−) ◄──────────────────── GND
   blanco (A) ───────────────────► GPIO32    (pull-up interno; opcional 4,7 kΩ a 3V3 si hay ruido)
   verde  (B) ───────────────────► GPIO33    (ídem)
```

El encoder es NPN de colector abierto: aunque se alimenta a 5 V, solo baja la señal a GND; el alto lo pone el
pull-up a 3,3 V. **Nunca poner el pull-up a 5 V.**

### A3. J3 codo: 5840-31ZY + L298N canal B (mismo módulo que J2)

```
 Motor J3 (2 cables) ─────────► OUT3 y OUT4

 L298N                            ESP32-WROOM
   ENB (pin de señal) ◄─┬──────── GPIO13    ← SACAR el jumper de ENB
                        └ 10 kΩ ─ GND        (pull-down)
   IN3  ◄──────────────────────── GPIO16
   IN4  ◄──────────────────────── GPIO17

 Encoder 38S6G5 (J3)              ESP32-WROOM
   rojo 5V · negro GND · blanco (A) → GPIO18 · verde (B) → GPIO19   (pull-up interno, como J2)
```

### A4. Finales de carrera de J1 y J2 (cuando se instalen)

Microinterruptor con contacto **normalmente abierto (NA)**: apretado = conecta el GPIO a GND.

```
 Final de carrera J1:  COM ── GND      NA ──┬──► GPIO36 (VP)
                                            └ 10 kΩ ─ 3V3   (pull-up OBLIGATORIO)
 Final de carrera J2:  COM ── GND      NA ──┬──► GPIO39 (VN)
                                            └ 10 kΩ ─ 3V3   (pull-up OBLIGATORIO)
```

J3 no tiene final de carrera (no queda GPIO libre): su cero se fija a mano.

### A5. Servos de la pinza (fuera del lazo cerrado, para completar)

Señal: apertura → GPIO4, giro J4 → GPIO14. Alimentación de 5–6 V **aparte** (buck 3 o una segunda salida de la
fuente), nunca del ESP32; GND común con la WROOM.

---

## B. ESP32-S3 · base (`/dev/ttyACM0`)

### B1. Tracción: 2 × 42GM-895H (53 rpm) + 2 × IBT-2

```
 Fuente 12 V (+/−) ──► B+ / B− de cada IBT-2

 Motor izquierdo: rojo/negro ──► M+ / M− del IBT-2 izquierdo
 Motor derecho:   rojo/negro ──► M+ / M− del IBT-2 derecho

 IBT-2 izquierdo                   ESP32-S3
   VCC ◄── 3V3 · GND ◄── GND
   RPWM ◄──────────────────────── GPIO4
   LPWM ◄──────────────────────── GPIO5
 IBT-2 derecho
   VCC ◄── 3V3 · GND ◄── GND
   RPWM ◄──────────────────────── GPIO6
   LPWM ◄──────────────────────── GPIO7
 R_EN + L_EN de LOS DOS IBT-2 (4 pines unidos) ◄─┬─ GPIO38
                                                 └─ 10 kΩ ─ GND   (pull-down)
```

### B2. Encoders de tracción (42GM-895H, Hall 11 PPR)

```
 Encoder (cada motor)              ESP32-S3
   azul    (Vcc) ◄────────────── 5V
   verde   (GND) ◄────────────── GND
   amarillo (A) ──────┬────────► GPIO15 (izq) / GPIO17 (der)
                      └ 20 kΩ ─ GND
   blanco   (B) ──────┬────────► GPIO16 (izq) / GPIO18 (der)
                      └ 20 kΩ ─ GND
```

El encoder trae un pull-up interno de 10 kΩ a su Vcc (5 V): **sin la resistencia de 20 kΩ a GND la señal llega a
5 V y quema la S3.** Con ella queda en 5 × 20/30 ≈ 3,3 V. **Antes de conectar a la S3**, medir con el multímetro,
girando la rueda despacio, que el alto no pase de 3,3 V.

### B3. Bumpers (3, microinterruptor NA)

```
 Bumper frente:     COM ── GND    NA ──► GPIO14
 Bumper izquierdo:  COM ── GND    NA ──► GPIO21
 Bumper derecho:    COM ── GND    NA ──► GPIO39
```

Sin resistencias: la S3 tiene pull-up interno en estos pines (apretado = LOW). GPIO40 queda libre para un 4.º
bumper.

---

## Lista de resistencias

| Valor | Cant. | Dónde | Función |
|-------|-------|-------|---------|
| 10 kΩ | 3 | WROOM GPIO 23, 25, 13 → GND | Pull-down de EN de J1, ENA de J2 y ENB de J3 |
| 10 kΩ | 4 | WROOM GPIO 34, 35, 36, 39 → 3V3 | Pull-up del encoder de J1 y de los finales de carrera |
| 10 kΩ | 1 | S3 GPIO 38 → GND | Pull-down de los EN de tracción |
| 20 kΩ | 4 | Señales A y B de los 2 encoders de tracción → GND | Bajar 5 V a 3,3 V |
| 4,7 kΩ | 4 (opcional) | WROOM GPIO 32, 33, 18, 19 → 3V3 | Pull-up más firme para los encoders de J2/J3 con cables largos |

**Para empezar (pasos 0–5, solo J1 y J2): 10 kΩ × 4** → GPIO 23 y 25 a GND, GPIO 34 y 35 a 3V3.

Fuera de motores y bumpers, el robot necesita además: divisor 1 kΩ / 2 kΩ en cada ECHO de los 3 HC-SR04 y
divisor 120 kΩ / 8,2 kΩ para la batería (ver `CONEXIONES.md` del robot).
