# 08-motor-5840-l298n — Motor 5840-31ZY con L298N y encoder externo, lazo abierto (ESP32-WROOM-32D)

Segundo motor. Las mismas pruebas que `06-motor-36gp555`: duty a mano, barrido, escalón y perfil en escalera, con el mismo análisis y los mismos gráficos (`tools/graficar.py`). Cambian el motor, el driver (L298N), la placa (**ESP32-WROOM-32D**, no la S3) y el encoder (externo, en el eje de salida).

## Hardware

| Pieza | Dato |
|-------|------|
| Motor | **5840-31ZY** (ZENG WHCD), reductor de **tornillo sin fin**, **12 V, 160 rpm** en la salida, par nominal 100 kg·cm, corriente sin carga 0,08–0,3 A |
| Driver | Módulo **L298N** (puente H doble, bipolar): 5–35 V, **2 A por canal**, regulador 78M05 integrado |
| Encoder | **38S6G5-B-G24N**, incremental externo, **1000 PPR**, fases A/B, eje de 6 mm, **salida NPN colector abierto**, alimentación **5–24 V** |
| Placa | **ESP32-WROOM-32D** (DevKit con USB-serie **CH340**, `/dev/ttyUSB0`) |
| Fuente | 12 V de laboratorio, **límite de corriente en 2 A** (lo máximo del L298N) |

Con x4 (flancos de A y B) son **4000 cuentas por vuelta de salida**. A 160 rpm son ~10 700 cuentas/s, sin problema para el PCNT.

## Conexiones

```
 Fuente 12 V (+) ──────────────► 12V   ┐
 Fuente 12 V (−) ──────────────► GND   │ bornera de alimentación del L298N
                     (dejar libre) 5V  ┘ jumper "5V-EN" PUESTO (el regulador alimenta la lógica)

 Motor (2 cables) ─────────────► OUT1 y OUT2   (el orden solo cambia el sentido de giro)

 L298N                            ESP32-WROOM-32D
   ENA (pin de señal) ◄─┬──────── GPIO25   ← SACAR el jumper de ENA
                        └ 10 kΩ ─ GND        (pull-down recomendado)
   IN1  ◄──────────────────────── GPIO26
   IN2  ◄──────────────────────── GPIO27
   GND  ◄──────────────────────── GND      ← GND común, obligatorio

 Encoder 38S6G5 (4 cables)        ESP32-WROOM-32D
   Rojo  (+)  ◄───────────────── 5V (VIN, sale del USB)
   Negro (−)  ◄───────────────── GND
   Blanco (A) ─────┬───────────► GPIO32
                   └ 4,7 kΩ ─ 3V3            (pull-up recomendado)
   Verde  (B) ─────┬───────────► GPIO33
                   └ 4,7 kΩ ─ 3V3            (pull-up recomendado)
```

### Detalles importantes

- **ENA tiene dos pines y un jumper.** Uno de los pines es **5 V** y el otro es la **entrada ENA**. Hay que sacar el jumper y conectar GPIO25 **solo al pin de entrada**. Si GPIO25 va al pin de 5 V, se daña. Para identificarlos, con los 12 V encendidos y el jumper afuera, medir cada pin contra GND: el que da ~5 V es el de 5 V. **No usar ese.**
- **Jumper 5V-EN (junto a la bornera):** dejarlo puesto. Con 12 V, el regulador del módulo alimenta su lógica. El borne **5V** queda **sin conectar**: no unirlo al 5V del ESP32 (serían dos fuentes enfrentadas).
- **Entradas del L298N con 3,3 V:** el L298N reconoce un "1" desde ~2,3 V, así que funciona directo con las salidas de 3,3 V del ESP32.
- **Pull-down en ENA:** mientras el ESP32 arranca o se flashea, GPIO25 flota, y una entrada del L298N al aire puede leerse como "1". IN1 e IN2 también flotan; si quedan iguales, el motor queda frenado y no gira. Con 10 kΩ de ENA a GND, el puente queda apagado hasta que arranca el firmware. **Si no la ponés:** apagá los 12 V antes de flashear o resetear.
- **Encoder a 5 V, señales con pull-up a 3,3 V:** necesita al menos 5 V para funcionar, así que se alimenta desde el pin **5V/VIN** del ESP32 (sale del USB). Sus salidas son **NPN colector abierto**: solo unen la señal a GND y no ponen tensión propia, así que el nivel alto lo da el pull-up a **3,3 V**. El firmware activa los pull-ups internos del ESP32 (~45 kΩ). Alcanzan para probar, pero con cables largos o ruido del motor conviene agregar **4,7 kΩ a 3V3** en A y en B. **Nunca poner el pull-up a 5 V**: los GPIO no lo toleran.
- **Comprobar que es colector abierto** antes de conectar A y B. Con el encoder alimentado a 5 V y A **desconectado** del ESP32, medir A contra GND mientras se gira el eje despacio. Si en algún momento marca ~5 V, el encoder tiene pull-up interno a 5 V y hace falta un divisor (10 kΩ en serie y 20 kΩ a GND). Si marca 0 V o nada, es colector abierto y se conecta como en el diagrama.
- **Acople mecánico:** el eje del encoder (6 mm) se une al eje de salida del motor con un **acople flexible**. El encoder soporta poca carga radial (30 N): no apoyarle el peso de nada.
- **GPIO elegidos:** 25–27 y 32/33 no son de strapping (0, 2, 5, 12 y 15), ni de la flash (6–11), ni del USB (1 y 3). Tampoco son de los 34–39, que solo sirven de entrada y no tienen pull-up interno.

### Fuente y corriente

- **Límite de corriente de la fuente: 2 A.** Es lo máximo del L298N por canal. El motor consume 0,08–0,3 A sin carga, pero **trabado** puede pedir más de 2 A (potencia máxima 28,8 W a 12 V ≈ 2,4 A).
- **El L298N pierde tensión:** usa transistores bipolares (dos en serie por cada camino de corriente). Pierde ~1,8–2,5 V con poca corriente, ~3 V a 1 A y hasta **~4,9 V a 2 A**, y esa tensión se vuelve calor (casi 10 W a 2 A). **Medido: 9,6 V en el motor a 100 % con 12 V de fuente.** Tocar el disipador después de pruebas largas o con carga.
- **La corriente de frenado no pasa por la fuente:** al frenar, el motor se descarga a través del puente, así que el límite de 2 A de la fuente no la acota. Sin carga es chica; con cargas grandes, frenar desde alta velocidad exige más al L298N.
- **El sin fin es autobloqueante:** la salida no se puede girar a mano, y el motor frena solo al cortar la tensión. Para probar el encoder con `n`, girá el eje **del encoder** con la mano antes de acoplarlo.
- **Mucho par** (100 kg·cm): fijar el motor y no poner la mano en la salida.

## Cómo probar

```bash
cd ~/pruebas-esp32/08-motor-5840-l298n
~/.platformio/penv/bin/pio run -t upload        # con los 12 V apagados
~/.platformio/penv/bin/pio device monitor       # ver el arranque; salir con Ctrl+C
```

Si el upload no conecta (`Failed to connect`): mantener **BOOT** apretado, pulsar **EN** y soltar BOOT al empezar a escribir. Si PlatformIO no encuentra el puerto, agregar `--upload-port /dev/ttyUSB0`. El CH340 no es de Espressif: si ModemManager interfiere (el arranque falla a veces), agregar una regla como la de `AGENTS.md` con `idVendor=="1a86"`.

**Orden:**
1. Con los 12 V apagados: `n` en el monitor y girar el eje **del encoder** a mano. Las cuentas cambian: una vuelta son 4000. Enter para terminar.
2. Acoplar el encoder, encender los 12 V y probar `20`, `40`, `-40`, `0`. Con duty positivo, las rpm deben salir positivas; si salen negativas, poner `ENCODER_INVERTIDO = true` o cruzar blanco y verde.
3. Cerrar el monitor y correr los gráficos.

Los comandos del monitor son los mismos que en 06 (`<n>`, `0`/`s`, `x`, `l`, `a`, `e<n>`, `p`, `n`, `?`).

## Gráficos (`tools/graficar.py`)

```bash
python3 tools/graficar.py barrido
python3 tools/graficar.py escalon 50
python3 tools/graficar.py perfil                                   # escalera ±20..100 %, 5 s por nivel
python3 tools/graficar.py perfil --niveles 30 45 60 80 100         # si 20 % no arranca el sin fin
python3 tools/graficar.py perfil --archivo resultados/perfil_....csv   # volver a graficar
```

Es el mismo análisis de 06 (ver su README): barrido duty–rpm con la zona muerta, escalón con τ, y perfil con los 4 gráficos (posición, señal de control, posición por nivel y velocidad filtrada) más el modelo `G(s) = K/(τs + 1)`. Diferencias:

- **Puerto automático:** busca el CH340, CH9102 o CP210x. Si no lo encuentra, usar `--puerto /dev/ttyUSB0`.
- **Sincronización al abrir:** el CH340 mueve DTR/RTS y el ESP32 puede reiniciarse al abrir el puerto. El script manda `?` hasta que el firmware contesta la ayuda, y recién ahí manda la prueba. También descarta cualquier resto de texto que haya quedado en la línea del firmware.
- **Velocidad en rpm de salida:** con 4000 cuentas por vuelta, en las tablas las columnas "rpm motor" y "rpm salida" coinciden (`REDUCCION = 1`).
- **Si la prueba se corta** (por atasco o con Ctrl+C), frena y **guarda lo medido** como `*_cortado.csv`, y lo grafica igual.
- **Ctrl+C en cualquier momento** (al conectar, al esperar que el motor pare o durante la prueba) manda `x` y espera la confirmación del freno.
- **Perfil:** τ y el modelo usan solo los niveles con **giro estable** (más del 5 % de la velocidad máxima y variación del régimen menor al 20 %). Por ejemplo, el avance a tirones del sin fin con 20 % queda fuera, marcado con "—".

## Firmware

- `src/main.cpp` adapta el de 06: mismo modo manual con rampa, barrido, escalón, perfil y protecciones (no invierte sin el motor quieto, corte por atasco, comandos bloqueados si falla la inicialización). Los umbrales están en **rpm de salida** (`include/motor_config.h`): quieto con menos de 1 rpm; atasco con duty ≥ 50 % y menos de 5 rpm durante 1 s.
- `lib/L298N`: la misma interfaz que `lib/BTS7960` (`setDuty`, `brake`, `coast`), con dos formas de aplicar el PWM (`PWM_MODO` en `include/motor_config.h`):
  - **`Freno` (por defecto):** ENA fijo en alto (GPIO25) y el PWM en **IN1** (adelante) o **IN2** (atrás). En la parte apagada del ciclo el motor queda **frenado**, así que la velocidad es casi proporcional al duty, como con el BTS7960.
  - **`RuedaLibre`:** el PWM en ENA e IN1/IN2 fijos. En la parte apagada, rueda libre. **Medido (2026-09-25): curva saturada**: 40 % ya da 86 rpm (el 60 % de la máxima), así que la zona muerta y la K del ajuste lineal no tienen sentido.
  - El cableado es el mismo en los dos modos. Duty 0 es freno (IN1 = IN2 = 0 con ENA en alto) y `coast()` pone ENA en bajo.
  - Un pin que deja de tener PWM se desconecta del LEDC y queda en bajo en el acto: sin pulsos al salir del freno ni al cambiar de sentido.
  - PWM a 1 kHz: el L298N conmuta lento y a más frecuencia pierde linealidad; se escucha un zumbido.
- El encoder usa `lib/EncoderPCNT` de 06 (enlazado), que funciona igual en el ESP32 clásico.

## Resultados

| Fecha | Prueba | Resultado | Observaciones |
|-------|--------|-----------|---------------|
| 2026-09-25 | Encoder | Cuenta bien, pero **al revés** con este acople (+30 % daba −32 rpm) | `ENCODER_INVERTIDO = true` |
| 2026-09-25 | Abrir el puerto (pyserial, como `graficar.py`) | **No reinicia** el ESP32 (probado dos veces) | El driver flota solo al flashear o al pulsar EN: en esos casos, 12 V apagados (sin pull-down en ENA) |
| 2026-09-25 | Tensión en el motor a 100 % | **9,6 V** con 12 V de fuente | Pérdida del L298N (~2,4 V) |
| 2026-09-25 | Perfil, modo `RuedaLibre` (PWM en ENA) | 20 % → 14 rpm (a tirones), 40 % → 86, 60 % → 114, 80 % → 130, 100 % → **141 rpm**; atrás casi igual. τ ≈ 40–110 ms en los niveles de 60 a 100 % | Curva **saturada**: el ajuste lineal da una zona muerta negativa. La velocidad ondula en todos los niveles (¿acople descentrado?). `resultados/perfil_20260925_214520.*` |
