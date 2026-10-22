# 11 · LED RGB de la ESP32-S3

Averigua en qué GPIO está el LED RGB (WS2812) de la placa: el **48** (DevKitC-1 v1.0) o el **38** (v1.1). En el
robot, el 38 es `PIN_TRACCION_EN` (`recolector_de_frutas/firmware/base/include/pins.h`).

**Antes:** la S3 sola por USB, sin IBT-2 ni 12 V (el programa mueve el GPIO 38).

```bash
~/.platformio/penv/bin/pio run -t upload -t monitor
```

| Ve | Conclusión |
|---|---|
| Azul (1 s) | LED en el 48: sin conflicto con el EN de la tracción |
| Rojo (1 s) | LED en el 38: corregir el comentario de `pins.h` ("evitar 48 (LED)") |
| Nada | Ni 48 ni 38: el clon lo tiene en otro pin (seguir la pista con el multímetro) |

Si la S3 queda en modo descarga después de flashear, apretar RST.

| Fecha | Resultado |
|---|---|
| 19/10/2026 | **GPIO 48.** Este programa mostró azul y rojo; con un pin por programa: solo el 48 parpadea, el 38 no hace nada. El rojo venía del canal RMT que sigue ruteado al 48 al escribir por el 38: probar cada pin por separado. |
