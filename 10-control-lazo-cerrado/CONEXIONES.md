# Conexiones — motores, drivers, finales de carrera y bumpers

Mismos pines que el brazo del robot: conexionado **v0.4** de `recolector_de_frutas` (`docs/CONEXIONES.md` y
`firmware/brazo/include/pins.h`): J1, J2 y J3 con un IBT-2 cada una y el **EN común en el GPIO 23**. Banco de
pruebas: **fuente de laboratorio de 12 V** en lugar de la LiPo. Motores 36-555 con encoder SCX-555 en las tres
(J1 160 rpm, J2 y J3 58 rpm, con correa 20 → 100).

## Fuente de laboratorio

- **12,0 V** y un límite de corriente según lo que esté conectado. El límite no es para la fuente sino para
  proteger drivers y cables: con un cable mal puesto o un motor trabado corta antes de quemar algo.
  - **Probando J1: 3 A.** Con 3 A se hicieron las pruebas de lazo abierto de `06`.
  - **Probando J2 o J3: 3 A** (empezar así; si traba seguido, bajar a 2 A).
- Si con J1 la fuente entra en CC al arrancar (baja la tensión y se enciende el indicador de corriente), se puede
  subir a 4 A.
- **GND común:** el negativo de la fuente, el GND de cada driver y el GND del ESP32 unidos.
- **Apagar los 12 V antes de flashear o pulsar EN/RST** hasta que esté el pull-down del GPIO 23.

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
                          └ 10 kΩ ─ 3V3                  (pull-up recomendado)
   rojo  (Signal B) ──────┬─────► GPIO35
                          └ 10 kΩ ─ 3V3                  (pull-up recomendado)
```

GPIO 34–39 de la WROOM no tienen pull-up interno. La placa SCX-555 parece traer los suyos (R1 y R2, código
`512` = 5,1 kΩ, de A y B a Encoder+), así que con el encoder a 3V3 ya daría 0/3,3 V. Igual se ponen los dos de
10 kΩ: no molestan (en paralelo quedan ~3,4 kΩ) y no se depende de la placa. Para confirmarlo, **sin alimentar**,
medir de negro a verde y de rojo a verde: unos 5 kΩ.

### A2. J2 hombro: 36-555 58 rpm + IBT-2 + correa 20 → 100

Igual que J1, en otros pines. Fuente con límite de **3 A**. Los tres IBT-2 comparten el EN (GPIO 23, con **una sola**
10 kΩ a GND, la de A1): cuando el firmware de una articulación lo habilita, los IBT-2 de las otras quedan habilitados
con PWM 0, o sea **frenados**. Si se prefiere, mientras se prueba una, dejar sin +12 V los IBT-2 de las otras.

```
 Fuente 12 V (+/−) ───────────► IBT-2 B+ / B−
 Motor: blanco ───────────────► M+     amarillo ──► M−   (invertirlos solo cambia el sentido)

 IBT-2 (conector de 8 pines)       ESP32-WROOM
   VCC   ◄────────────────────── 3V3
   GND   ◄────────────────────── GND
   RPWM  ◄────────────────────── GPIO25
   LPWM  ◄────────────────────── GPIO26
   R_EN ─┬─ L_EN (puenteados) ◄─── GPIO23   (el mismo de J1; sin otra resistencia)
   R_IS, L_IS                     sin conectar

 Encoder (placa SCX-555)           ESP32-WROOM
   verde (Encoder+) ◄──────────── 3V3   (NO a 5 V)
   azul  (Encoder−) ◄──────────── GND
   negro (Signal A) ─────────────► GPIO32   (pull-up interno + el de 5,1 kΩ de la placa)
   rojo  (Signal B) ─────────────► GPIO33
```

Correa GT2: polea de 20 dientes en el motor y 100 en el hombro (5:1), correa cerrada de 240 mm, 54 mm entre centros.
Con el de 58 rpm (139:1): 8896 cuentas por vuelta de la salida × 5 = **123,6 cuentas por grado** del brazo.
**El 36-555 no se autobloquea:** sin corriente el brazo cae.

### A3. J3 codo: 36-555 58 rpm + IBT-2 + correa 20 → 100

Igual que J2, con RPWM → GPIO16, LPWM → GPIO17, R_EN + L_EN → GPIO23, encoder negro (A) → GPIO18 y rojo (B) →
GPIO19, verde a 3V3, azul a GND. Firmware: `pio run -e j3`. GPIO 13: libre (reserva), como en el robot.

### A4. Finales de carrera de J1, J2 y J3 (cuando se instalen)

Microinterruptor con contacto **normalmente abierto (NA)**: apretado = conecta el GPIO a GND.

```
 Final de carrera J1:  COM ── GND      NA ──┬──► GPIO36 (VP)
                                            └ 10 kΩ ─ 3V3   (pull-up OBLIGATORIO)
 Final de carrera J2:  COM ── GND      NA ──┬──► GPIO39 (VN)
                                            └ 10 kΩ ─ 3V3   (pull-up OBLIGATORIO)
```

Final de carrera J3:  COM ── GND      NA ──► GPIO27   (pull-up interno)

Mientras no estén, el cero se fija a mano (`z` en la marca).

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
| 10 kΩ | 1 | WROOM GPIO 23 → GND | Pull-down del EN común de los 3 IBT-2 |
| 10 kΩ | 4 | WROOM GPIO 34, 35, 36, 39 → 3V3 | Pull-up del encoder de J1 (opcional) y de los finales de carrera (obligatorio) |
| 10 kΩ | 1 | S3 GPIO 38 → GND | Pull-down de los EN de tracción |
| 20 kΩ | 4 | Señales A y B de los 2 encoders de tracción → GND | Bajar 5 V a 3,3 V |

**Para empezar: 10 kΩ × 1** → GPIO 23 a GND (más las 2 opcionales de GPIO 34 y 35 a 3V3).

Fuera de motores y bumpers, el robot necesita además: divisor 1 kΩ / 2 kΩ en cada ECHO de los 3 HC-SR04 y
divisor 120 kΩ / 8,2 kΩ para la batería (ver `CONEXIONES.md` del robot).
