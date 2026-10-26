# 17-brazo-joystick — J1, J2 y J3 con un joystick en el navegador

Mover las tres articulaciones del brazo a la vez desde una página web, en la **ESP32-WROOM-32D** con el conexionado
v0.5 del robot (mismos pines que `12`–`14`, `include/pins.h`). **Lazo abierto:** cada eje del joystick da el duty de su
articulación (de `dutyMin` a `dutyMax`, en `include/config.h`). **El cero es donde está el brazo al arrancar el
puente** (`tools/joystick.py` manda `Z` al abrir el puerto) o al pulsar "Cero aquí".

```
navegador (Firefox)  ──HTTP──>  tools/joystick.py  ──USB serie──>  WROOM
http://localhost:8000          (puente, pyserial)                 J e1 e2 e3 / Q q1 q2 q3 d1 d2 d3 est...
```

Firefox no tiene Web Serial: por eso el puente en Python sirve la página y reenvía los comandos.

## Seguridad

- **Fuente de laboratorio a 12 V con límite de 3 A.** Apagar los 12 V antes de flashear o pulsar EN.
- **Soltar el joystick = freno** (duty 0 con el EN alto), nunca rueda libre. **J2 y J3 no se autobloquean:**
  frenadas bajan despacio con el peso. Tener la mano cerca o apoyar el brazo en un tope.
- **Hombre muerto:** si la placa no recibe comandos durante 300 ms (pestaña oculta, puente cerrado, cable), frena las
  tres. Espacio o el botón rojo frena y deja el joystick bloqueado hasta volver a tocarlo.
- **Límites desde el cero:** J1 −45° / +135°, J2 y J3 ±15° (provisorios, como `13`/`14`: J2 y J3 forman un cuatro
  barras y no se verificó cuánto se pueden mover). Fuera del límite solo deja volver hacia adentro. El sentido
  "hacia afuera" se aprende mientras se mueve, así funciona aunque los signos de J2/J3 estén mal.
- **Atasco:** con ≥ 30 % y menos de 5 cuentas en 500 ms corta esa articulación (estado `TRABA`) hasta soltar el joystick.
- Signos de J2 y J3 sin medir: si una sale al revés en la página, el estado dice `SIGNO` (encoder al revés del duty).

## Cómo usarlo

```bash
cd ~/Documentos/pruebas-esp32/17-brazo-joystick
~/.platformio/penv/bin/pio run -t upload    # 12 V APAGADOS, monitor cerrado
python3 tools/joystick.py                   # con el brazo en la pose de cero; Ctrl+C para salir
```

1. Abrir **http://localhost:8000** en Firefox. Arriba tiene que decir "Placa: conectada".
2. Brazo en la pose que va a ser el cero (sostener J2/J3) → "Cero aquí" → prender los 12 V.
3. Mover con la **escala baja** (60 % por defecto) y subirla de a poco.

| Control | J1 base | J2 hombro | J3 codo |
|---|---|---|---|
| Pantalla (mouse o táctil) | joystick redondo ←→ | joystick redondo ↑↓ | barra vertical ↑↓ |
| Teclado | A / D | W / S | I / K |
| Gamepad | stick izq. ←→ | stick izq. ↑↓ | stick der. ↑↓ |

Espacio = frenar. Para cambiar duty mínimo/máximo, límites o frecuencia del PWM: `include/config.h` (J1 20 kHz,
J2/J3 1 kHz, como `12`–`14`) y los `LIMITES` del principio del script de `tools/joystick.html` (solo para dibujar).

## Estado

- [x] Compila. Puente probado con un puerto serie simulado (comandos y telemetría).
- [ ] Probado en placa.

## Resultados

| Fecha | Resultado |
|---|---|
| | |
