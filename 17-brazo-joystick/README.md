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
- **Cero = la pose con los tres finales de carrera apretados** (fotos en `LIMITES/ceros_bumpers/`, 2026-10-30). Cada
  articulación pone su cero sola al tocar su final de carrera (si iba hacia él o estaba quieta); también al abrir el
  puente y con "Cero aquí". **+ = alejarse del final de carrera** (J1 hacia el frente y el mástil, J2 antihorario
  visto desde atrás, J3 el codo bajando).
- **Final de carrera apretado (estado `FC`):** no deja seguir hacia él, solo alejarse. El lado se aprende con el
  sentido en que iba al apretarse (si arranca apretado, se supone el −).
- **Límites desde el cero:** J1 −5° / +118° (medido con el brazo recogido), J2 −5° / +97°, J3 −5° / +113°
  (medidos el 2026-10-30; ver abajo). El de
  arriba se cambia en vivo con "máx ±10" (comando `L`); al reiniciar la placa vuelven los de `config.h`. Fuera del
  límite solo deja volver hacia adentro. El sentido "hacia afuera" se aprende mientras se mueve.
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
| Pantalla (mouse o táctil) | barra horizontal ←→ | barra vertical ↑↓ | barra vertical ↑↓ |
| Teclado | A / D | W / S | I / K |
| Gamepad | stick izq. ←→ | stick izq. ↑↓ | stick der. ↑↓ |

Una barra por articulación, independientes: cada una sigue a su dedo (se pueden mover varias a la vez) y vuelve a 0
al soltarla. Espacio = frenar. Duty mínimo/máximo, límites iniciales, signos y frecuencia del PWM: `include/config.h`
(J1 20 kHz, J2/J3 1 kHz, como `12`–`14`). Desde el celular: `python3 tools/joystick.py --host 0.0.0.0` y
`http://<IP de la notebook>:8000` (cualquiera en esa red puede mover el brazo).

## Medir los límites

1. 12 V apagados; flashear; brazo en la pose de los finales de carrera; arrancar el puente.
2. Los tres tienen que decir "FC APRETADO". Prender los 12 V.
3. **Signos:** mover cada articulación con + (alejándose de su final de carrera): el ángulo tiene que **subir**. Si
   baja, "Invertir" en esa articulación y anotarlo para pasarlo a `config.h`. Al volver y tocar el final de carrera,
   pone el cero.
4. **J1:** subir su "máx" de a 10 hasta cerca del mástil, sin tocarlo; "Anotar" (con nota) en el punto útil.
5. **J2 y J3 juntas:** con J2 en 4 o 5 ángulos (por ejemplo 0, 10, 20, 30, 40°), llevar J3 hasta donde puede ir sin
   trabarse ni chocar (subiendo su "máx" de a 10) y "Anotar" cada extremo con una nota. También el máximo de J2 con
   J3 en el cero.
6. "Borrar la última" saca la última fila anotada. Las filas quedan en `resultados/limites_<fecha>_<hora>.csv` (hora, q1–q3, finales de carrera, límites, signos
   invertidos en vivo, nota).

## Límites medidos (2026-10-30, `resultados/limites_20261030_210733.csv`)

Con el joystick, desde la pose de los finales de carrera (+ = alejarse de cada uno):

| Borde | Puntos (q2, q3) | q3 − q2 |
|---|---|---|
| J3 máximo (el codo no baja más) | (−0,6; 11,9) (9,9; 21,2) (20,0; 29,6) (30,0; 40,1) (53,4; 66,5) (79,2; 91,8) (97,3; 112,9) | +9,7 a +15,6 |
| J2 máximo (el hombro arrastra a J3) | (30,0; 0,1) (53,4; 20,0) (79,2; 45,1) (97,3; 70,0) | −27,3 a −34,2 |

J2 y J3 se mueven en una **franja**: `q2 − 25° ≤ q3 ≤ q2 + 7,5°` (los peores casos, 9,7° y 27,3°, con 2° de margen),
con `0 ≤ q2 ≤ 97°` y `0 ≤ q3 ≤ 113°` (el último punto medido; el usuario cortó ahí). En el borde el estado es `4B` y
solo deja moverse hacia adentro. La franja no se cerró del todo: más allá de q2 = 97° no se midió. J1: 118° con el brazo recogido
(`resultados/limites_20261030_221357.csv`: llegó a 121,8°, cerca del soporte de la visión).

## Estado

- [x] Compila. Puente probado con un puerto serie simulado (comandos y telemetría).
- [x] Probado en placa (2026-10-30): barras independientes, desde el celular con `--host 0.0.0.0`.
- [x] Finales de carrera, cero automático, límites en vivo, "Anotar" (2026-10-30, en placa). J3 iba al revés: signos
  en `true`/`true` en `config.h`.
- [x] Franja del cuatro barras en placa (2026-10-30): con J3 en 10,1°, J2 frena (4B) en 35,8° (borde 25° + 0,7° de
  frenado); con J2 en 35,8°, J3 frena en 44,6° (borde 7,5° + 1,3° de frenado, dentro de los 9,7° medidos).
- [x] J1: 118° (2026-10-30, brazo recogido): llegó a 121,8° cerca del soporte de la visión
  (`resultados/limites_20261030_221357.csv`). Con el brazo estirado, sin medir.
- [x] Rampa de bajada de J1 al soltar la barra (~0,33 s desde 40 %): de golpe frenaba seco. Barra de J3 al revés
  (arriba = el codo sube).

## Resultados

| Fecha | Resultado |
|---|---|
| | |
