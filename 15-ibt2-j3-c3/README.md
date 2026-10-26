# 15-ibt2-j3-c3 — Prueba del IBT-2 de J3 con una ESP32-C3-MINI-1

Verificar que el **driver 3** (el IBT-2 de J3: 2 × BTS7960B + buffer 74HC244D) mueve el motor de J3 en los dos
sentidos, **con el motor montado en el brazo**. La placa es una ESP32-C3-MINI-1, suelta: no es el conexionado del
robot (eso es `14-j3-codo`, en la WROOM).

Solo hay **pulsos cortos de duty fijo**. Para que el codo no dé una vuelta sin querer:

| Protección | Valor (`include/config.h`) |
|---|---|
| Duty máximo | 60 % (arranca en 25 %) |
| Pulso máximo | 500 ms (arranca en 150 ms); a 60 % son ~20° como mucho |
| Límite por encoder | ±15° desde el cero (`z`): corta el pulso y no deja seguir alejándose, solo volver |
| Sin cuentas | si el encoder no cuenta en 150 ms, corta (motor trabado, driver muerto o encoder mal) |
| Respaldo por tiempo | pulsos + menos − desde el último `z`: máx 3 s (0,6 s sin encoder, `se`) |
| Corte manual | cualquier tecla durante un pulso lo corta |
| En reposo | freno (EN alto, PWM 0) |

## Conexiones

Pines en [`include/pins.h`](include/pins.h).

| C3 | Conecta a | Nota |
|---|---|---|
| GPIO 4 | IBT-2 RPWM | |
| GPIO 5 | IBT-2 LPWM | |
| GPIO 6 | IBT-2 R_EN **y** L_EN (juntos) | sin pull-down: ver Seguridad |
| GPIO 7 | encoder A (negro) | |
| GPIO 10 | encoder B (rojo) | |
| 3V3 | IBT-2 VCC (ver abajo) y encoder (verde) | |
| GND | IBT-2 GND, encoder (azul), GND de la fuente de 12 V | **GND común** |

IBT-2: B+ / B− a 12 V, M+ / M− al motor (blanco = M+, amarillo = M−), R_IS y L_IS sin conectar.

**VCC del IBT-2 a 3V3 en esta prueba.** El 74HC244 del módulo, alimentado a 5 V, toma como alto recién desde ~3,5 V,
y el C3 da 3,3 V: puede andar mal y parecer que el driver está roto. Con el VCC a 3V3 no hay duda. Si anda así y
se quiere repetir como en el robot, probar después con el VCC a 5 V.

No usar en el C3: 2, 8, 9 (strapping), 11–17 (flash), 18/19 (USB), 20/21 (UART0). GPIO de 3,3 V: no toleran 5 V.

## Seguridad

- **Fuente de laboratorio a 12 V con límite de 3 A**, no la LiPo.
- **El EN va sin pull-down** (decisión del 2026-10-22): mientras el C3 arranca, se flashea o se resetea, EN, RPWM y
  LPWM flotan y el puente puede dar un tirón. Por eso el orden es obligatorio: USB primero, esperar la ayuda y
  `PWM 20000 Hz OK` en el monitor, recién ahí prender los 12 V; **apagar los 12 V antes de flashear, pulsar RST o
  desenchufar el USB.** Lo que no cubre es un reset inesperado con los 12 V prendidos (falso contacto del USB): si
  pasa, apagar la fuente. Para cubrirlo, poner 4,7–22 kΩ del GPIO 6 a GND.
- **J3 no sostiene el antebrazo sin corriente** (planetario): con `x` o sin los 12 V cae. Frenado cae más despacio.
  Tener el antebrazo con la mano o apoyado.
- **J2 no está conectada a nada:** el hombro queda suelto. Apoyarlo en un tope durante toda la prueba.
- Antes de `+` / `-`, poner el codo en una posición con margen para los dos lados y hacer `z`.

## Cómo flashear y abrir el monitor

La placa de esta prueba tiene **CH340** (`1a86:7523`, `/dev/ttyUSB0`): entorno `c3_uart`, el de por defecto. Si la
WROOM también está enchufada aparecen dos `/dev/ttyUSBx`: desenchufarla o pasar el puerto. (Para una C3 con USB
nativo, `303a:1001` en `lsusb`, usar `-e c3_usb`, puerto `/dev/ttyACM0`.)

```bash
cd ~/Documentos/pruebas-esp32/15-ibt2-j3-c3
PIO=~/.platformio/penv/bin/pio
$PIO run -t upload --upload-port /dev/ttyUSB0
$PIO device monitor --port /dev/ttyUSB0
```

Si el USB nativo no aparece para flashear: mantener BOOT (GPIO 9), pulsar RST, soltar BOOT.

## Comandos

| Comando | Qué hace |
|---|---|
| `+` / `-` | pulso positivo / negativo con el duty y la duración actuales |
| `p <duty> [ms]` | pulso suelto con duty con signo, sin cambiar `d` ni `t` (`p 30 100`, `p -30 100`) |
| `d <pct>` | duty (1–60 %) |
| `t <ms>` | duración (10–500 ms) |
| `z` | cero acá y borra el tiempo neto |
| `e` | cuentas, grados, niveles de A y B, tiempo neto y signo |
| `f` / `x` | freno / rueda libre |
| `m <duty>` | diagnóstico **con el motor desconectado** de M+ / M−: pide confirmar con `s`, mantiene el duty (con signo) hasta una tecla (máx 30 s) para medir con el multímetro; corta si el encoder cuenta |
| `se` | apaga/prende el control del encoder (para probar sin encoder) |
| `?` | ayuda |

## Pasos

Con los 12 V **apagados**, el antebrazo apoyado y el monitor abierto mostrando la ayuda:

1. **Encoder.** `e` varias veces girando el codo a mano un poco: las cuentas tienen que cambiar y A/B estar en 0 o 1.
   (~124 cuentas por grado del codo.) Si no cambian, revisar el encoder o seguir con `se` (sin encoder).
2. Codo en una posición con margen, `z`. **Prender los 12 V.**
3. `+` con 25 % y 150 ms: tiene que moverse un poco y frenar. Anotar grados y sentido (¿la pinza sube o baja?).
4. `-`: tiene que volver. Si en algún sentido no se mueve (`SIN CUENTAS` sin movimiento), subir el duty de a 5 %
   (`d 30`, `d 35`…). Bajando, por el peso, alcanza con menos duty que subiendo.
5. Repetir unas veces en cada sentido. Con `t 300` se ven movimientos más claros (siguen limitados a ±15°).

**El driver anda** si mueve el motor en los dos sentidos, frena al terminar el pulso y con el mismo duty el
movimiento es parecido de una vez a la otra. **Sospechar del driver** si un sentido no anda y el otro sí (un
BTS7960B muerto), si se mueve a los saltos, o si calienta sin mover el motor.

## Resultados

| Fecha | VCC IBT-2 | Duty mínimo subiendo | Duty mínimo bajando | ° por pulso (25 %, 150 ms) + / − | ¿Funciona? | Notas |
|---|---|---|---|---|---|---|
| | | | | | | |
