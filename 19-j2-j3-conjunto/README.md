# 19-j2-j3-conjunto — Lazo cerrado de J2 y J3 juntas

J2 (hombro) y J3 (codo) forman un cuatro barras: no se pueden mover cada una por su lado (la franja medida con
[`17`](../17-brazo-joystick/README.md)). Esta prueba reemplaza a `13` y `14`, que movían una sola con la otra
frenada (y frenadas, J2 y J3 bajan solas).

- **Una articulación es la activa** y tiene todos los comandos de `12`–`14` (`p`, `m`, `bc`, `ev`, `ep`, `a`, `s`,
  `kp`…); los scripts de `tools/` (enlace a `10-control-lazo-cerrado/tools`) funcionan igual, con `-a j2` o `-a j3`.
  Se elige al compilar: `-e j2` (activa J2) o `-e j3` (activa J3).
- **La otra se sostiene sola** en segundo plano (`src/otra.cpp`, núcleo 0, 100 Hz) en el ángulo que se le pida con
  `o <grados>`; va hasta ahí despacio (5 °/s).
- **Límites de la activa** = los propios (J2 0–97°, J3 0–113°) ∩ la franja del cuatro barras
  `q2 − 25° ≤ q3 ≤ q2 + 7,5°` (`include/config.h`, `BANDA_*`) con la otra en el **peor caso** entre su ángulo medido,
  su referencia y su destino (la otra va a 5 °/s: con solo el destino, con `-e j2`, J2 en 50°, J3 en 30° y `o 55`,
  J2 quedaba habilitada hasta 80° con J3 todavía cerca de 30°). `oc` los muestra.
- **Mientras la otra va en camino**, los comandos que mueven la activa (`p m a s ev ep bc`) **esperan** a que llegue
  (la activa sigue sosteniéndose); Enter cancela (`x` durante la espera suelta las dos) y a los 30 s (`OTRA_ESPERA_MAX_MS`, PROVISORIO) se rechazan.
  "Llegó" = la referencia en el destino, a menos de `TOL_POS` (0,3°) y quieta `LLEGADA_MS` (200 ms), como la activa
  (fase 2; en la fase 1 bastaba la referencia).
- **Al sostener (las dos, fase 2):** con |error| < `BANDA_INT_HOLD` (0,2°, `bi`) el integrador no integra; opcional,
  **freno dentro de la banda** (`fb`, apagado al arrancar): al llegar, duty 0 (sin pitido ni corriente) hasta que se
  aleje más de `TOL_SALIDA` (0,6°). A los 2 s de llegar (`ASENTAMIENTO_MS`) informa `>> Asentado: ...` (posición
  final, error y duty de las dos). Todo PROVISORIO.
- **Cero:** `z` con los finales de carrera de J2 y J3 apretados (fotos en `17-brazo-joystick/LIMITES/ceros_bumpers`).
  Desde ahí la otra empieza a sostener en 0°. **+ = alejarse del final de carrera** (J2 antihorario visto desde
  atrás, J3 el codo baja).
- Después (paso 6): movimientos coordinados de las dos a la vez.

## Conexiones

Las del robot (v0.5), como `12`–`14` y `17`: J2 IBT-2 25 · 26, encoder 36 · 39, final de carrera **27**; J3 IBT-2
16 · 17, encoder 18 · 19, final de carrera **33** (J2 y J3 cruzados respecto de v0.5); EN común 23. J1 queda frenada.
PWM **20 kHz** (2026-10-31; antes 1 kHz, que pita). Pines en [`include/pins.h`](include/pins.h).

## Seguridad

- Fuente de laboratorio a 12 V con límite de 3 A. **12 V apagados para flashear.**
- **Hasta `z`, J2 y J3 no se sostienen:** brazo en la pose de los finales de carrera, sostenido o apoyado.
- `x` suelta **las dos** (el brazo cae). Enter corta cualquier movimiento de la activa; la otra sigue sosteniendo.
- `o` después de un `x` vuelve a subir el EN común sin bajarlo nunca (antes bajaba un instante y soltaba la activa y
  J1): las tres quedan frenadas y la otra sostiene.
- La otra: si empuja con ≥ 45 % sin moverse 1,5 s, deja de empujar (freno) y avisa (`ATASCO`); con su final de
  carrera apretado no empuja hacia él.
- `dg` solo con la otra sin sostener (pone el EN en bajo): brazo apoyado, `x`, `dg`.

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/19-j2-j3-conjunto
~/.platformio/penv/bin/pio run -e j2 -t upload     # activa J2 (12 V APAGADOS); -e j3 para J3
tools/monitor.sh                                    # salir con Ctrl+]
```

## Pasos (primero con `-e j2`, después con `-e j3`)

Orden de la fuente: **12 V apagados → monitor → brazo en la pose de los finales de carrera → `z` → 12 V encendidos →
comandos → brazo apoyado → 12 V apagados**.

| Paso | Qué hacer | Qué se espera / qué anotar |
|---|---|---|
| 1 | `z` → `oc`. Con la mano cerca: `o 20` (la otra a 20°), `oc` | `z`: los dos finales de carrera "apretado"; la otra va a 20° y se queda (duty que informa `oc`) |
| 1 | `p 30 300`, `p -30 300` (desde un ángulo con lugar, por ejemplo con la activa en ~10°: `m 10`) | + se aleja del final de carrera y las cuentas suben (signos de 17); con qué duty arranca subiendo y bajando |
| 1 | `m <ángulo>` y medir con el inclinómetro | 123,6 cuentas por grado dentro de ~0,5° |
| 2 | Sostener de la activa: después de cada comando sostiene sola (`sa`); `khp`/`khi` si oscila o cae. Lo mismo para la otra: `okp`/`oki` | que las dos se queden quietas; anotar el duty |
| 3 | Barrido de carga de la activa, con la otra en un ángulo con lugar (ej. J2 activa, `o 40` → J2 entre 32,5 y 65°): `python3 tools/barrido_carga.py --desde 35 --hasta 62 --v 4` | duty según el ángulo, subiendo y bajando → feedforward y compensación de gravedad |
| 4 | `python3 tools/escalon_velocidad.py -a j2 10 --ciclos 3` (desde el medio del rango) | PI de velocidad |
| 5 | `python3 tools/escalon_posicion.py -a j2 10 --ciclos 3` | llegada < 2 s, sobrepico < 0,5°, error ≤ 0,3° |
| 6 | Movimientos coordinados de las dos (falta programarlo) | llegan juntas, sin salir de la franja |
| 7 | Con el limón | igual |

**Prueba F1 (seguridad de la franja, con `-e j2`):** desde el cero, de a poco y dentro de la franja: `z` → `o 7` →
`a 28` → `o 30` → `a 50` (cada `o` tarda: el `a` siguiente tiene que decir `Esperando a J3` y J2 seguir sostenida).
Después `o 55` y enseguida `oc` (`EN CAMINO`, límite de J2 = el menor entre medido y referencia de J3 + 25°, no
80°) y `a 75`: espera a J3 y recién ahí va a 75°. Volver (cada paso dentro de la franja con el margen de 3°):
`o 52` → `a 48` → `o 30` → `a 26` → `o 7` → `a 3` → `o 0`. Por último, abajo y con la mano debajo del brazo: `o 7`
y enseguida `a 20`, y durante la espera `x` (suelta las dos); después `o 0`: J3 vuelve a sostener y J2 queda frenada,
sin soltarse (y J1 no se suelta).

**Prueba F2 (con `-e j2`, desde el origen, fuente a la bornera con 3 A):** `z` → `a 20` → `o 10` → esperar el
`Asentado` (anotar error y duty de las dos) → `o 27` y enseguida `oc` varias veces (`EN CAMINO` → `LLEGANDO` → `LLEGÓ`)
→ `o 10` y enseguida `a 25` (`Esperando a J3`, J2 quieta) → `Asentado`. Repetir `a 20` / `a 25` con `fb` prendido
(¿pita? ¿se queda?) y, si el duty todavía va y viene, con `bi 0.2`. Volver: `a 12` → `o 0` → `a 4`.

**Prueba PWM (antes del mapa; con `-e j2`, fuente a la bornera):** a 1 kHz el sostener pita; a 20 kHz, el 26/10, J2
no arrancaba con menos de 40 % (con la fuente por el XT60 de contacto malo). Con J2 en ~20° y J3 en ~10°, para
`pwm 1000`, `pwm 5000` y `pwm 20000`: pulsos `p 15 200`, `p 20 200`, `p 25 200`… y `p -10 200`, `p -15 200`… (con qué
duty arranca subiendo y bajando) y `o 12` → `oc` (¿J3 llega? ¿con qué duty sostiene? ¿pita?). Si a 20 kHz arranca con
≤ ~25 %, pasar las dos a 20 kHz y volver a medir la zona muerta antes del mapa. **Resultado (2026-10-31):** a 1 kHz
arranca subiendo con 25 % y bajando con 15 %; a 20 kHz J2 va y sostiene bien (error ≤ 0,20°, duty estable) y no pita
→ **20 kHz** (`PWM_FREQ_HZ`). A 20 kHz (pulsos): J2 arranca subiendo con 35 %, bajando con más de 15 %; J3 sostenida
va y viene ±1° (fricción ancha en duty) → compensación de fricción `fr` / `ofr` (ajuste en placa: alternar `o 10` /
`o 14` con `ofr 10`, `15`, `20`; J2 con `a 20` / `a 25` y `fr`). **Resultado:** J3 con 15 % no se pasa y queda con
0,09°; J2 no la necesita (0,19° / 0,05°); con `fb`, frenadas y sin ruido → `FR_SOSTENER` J3 15 %, J2 0 %, 20 kHz.

**Prueba F4 (homing, con `-e j2`):** los finales de carrera tienen un solo punto de apoyo y el golpe al llegar rápido los
giraba (2026-10-31: el cero se corrió ~9° en J3 y ~30° en J2; fijarlos mejor). Desde el cero (los dos apretados, `z`):
`h` → tiene que salir `Homing listo` con los flancos cerca de 0° (respecto del `z`) y quedar las dos en 3°. Repetir
`h` 5 veces sin tocar nada: los "flancos respecto del cero anterior" miden la repetibilidad del homing (cada línea
`HOMING,t,dq2,dq3,1`). Después, desde una pose más alta (`a 15` → `o 10` → `h`).

**Prueba F3b (con `-e j2`):** desde el cero, subir a una pose alta y volver (secuencia verificada contra la franja):
`a 20` → `o 10` → `a 31` → `o 30` → `a 50` → `o 52` → `a 70` → `o 62` (esperar cada `Asentado`, anotar error y duty de
las dos) y bajar con `o 50` → `a 50` → `o 30` → `a 28` → `o 10` → `a 20`. Repetir con `gm` apagado y comparar: con el modelo, J2 no tendría que subir ni bajar sola
después de llegar en 70° (allí sostiene con ~−10 %, no con el +25 % de abajo).

**Prueba F3a (captura del mapa de gravedad, con `-e j2`):** **abrir el puerto serie puede reiniciar la WROOM**
(2026-10-31: al volver a abrir el monitor, cero perdido y las dos sueltas), así que el cero lo hace el script: J2 y
J3 a sus finales de carrera con el joystick de `17` → 12 V apagados → flashear 19 → 12 V encendidos → **sin abrir el
monitor**, `python3 tools/mapa_gravedad.py capturar --cero --paso` (espera que la placa arranque, `z` —corta si algún
final de carrera está suelto—, apaga `fb`; 14 poses × desde abajo y desde arriba, ~10 min; `--paso` espera Enter
antes de cada pose; Ctrl+C corta y guarda). `plan` muestra los movimientos sin tocar la placa. Al final ajusta y grafica (`resultados/mapa_gravedad_<fecha>.csv/.png`)
e imprime `MAPA_J2` / `MAPA_J3` para `config.h` (fase 3b). Mano cerca: en q2 = 90° el brazo está estirado.

Los scripts: **sin `--cero`** (mandaría `z` lejos de los finales de carrera); cerrar el monitor antes.

**Volver al final de carrera:** `a` no baja de 3°; con el final de carrera de la activa suelto, `p -15 200` (o el %
que la mueva) puede pasar de 0° y se corta al apretarse (`FINAL DE CARRERA APRETADO`) → `z`.

**Comandos nuevos:** `o <°>` llevar la otra y sostenerla · `oc` dónde está la otra (medido, referencia, destino,
`EN CAMINO` / `LLEGANDO` / `LLEGÓ`, error, duty aprendido) y los límites de la activa con el peor caso · `bi <°>` banda
del integrador · `fb` freno dentro de la banda sí/no · `gm` modelo del sostener sí/no (fase 3b) · `fr <%>`, `ofr <%>` compensación de fricción al sostener (activa /
otra) · `an [nota]` anotar el duty de las dos asentadas (línea
`MAPA,...`, para `tools/mapa_gravedad.py`) · `h` homing (fase 4) · `pwm <Hz>` frecuencia del PWM de las dos en vivo (al reiniciar vuelve
la de `config.h`, 1 kHz) ·
`okp`, `oki` ganancias del sostener de la otra · `x` suelta las dos. El resto, como `12`–`14` (`?`).

## Estado

- [x] Compila (`-e j2` y `-e j3`, 2026-10-30). Sin probar en placa.
- [x] Fase 1, seguridad de la franja (2026-10-31): límites con el peor caso de la otra, la activa espera a que la
  otra llegue, `o` sin `coast()`. **Probada en placa con `-e j2`** (2026-10-31): la franja rechaza lo que corresponde
  en las dos direcciones y usa el medido de J3 cuando va atrasada; `x` → `o` vuelve a sostener. Falta ver la espera
  (`Esperando a J3`): va en la prueba de la fase 2.
- [x] Fase 2 (2026-10-31): banda del integrador, "llegó" de la otra con el criterio de la activa, freno dentro de la
  banda opcional, asentamiento, `o` destraba el ATASCO, mensajes de rechazo con el rango y la franja. **Probada en
  placa con `-e j2`** (2026-10-31, con el cero de J2 corrido ~4°): `Asentado`, `Esperando a J3`, estados de J3 y `fb`
  (sin pitido) andan; banda 0,1° → J3 oscila ±1°, 0,2° → quieta (queda 0,2°). J2 sube sola ~2° después de llegar con
  el `uHold` de otra pose (fase 3).
- [ ] Fase 3: mapa de gravedad (J2 y J3 se cargan mutuamente)
  - [x] 3a (2026-10-31): comando `an` y `tools/mapa_gravedad.py` (capturar / ajustar / plan). Compila; script
    probado sin placa (plan verificado contra las reglas de la franja; ajuste con datos sintéticos). **Falta la
    captura en placa** (prueba F3a abajo).
  - [x] Captura en placa (2026-10-31, 20 kHz): 27/28 poses; la fricción pesa como la gravedad (±7 %). J2 cambia de
    signo cerca de q2 = 54° (+25 % → −20 %); J3 ~−17 %. `resultados/mapa_gravedad_20261031_135400.csv/.png`.
  - [x] 3b mínima (2026-10-31): el sostener suma el modelo simple (J2 `31,4 − 0,58·q2` %, J3 −16,8 %) y `uHold`
    aprende la diferencia; `gm` sí/no. Compila; **sin probar en placa** (prueba F3b abajo).
- [x] Fase 4 (2026-10-31): homing `h` (J3 y después J2, a 1 °/s, cero en el flanco, retroceden 3°, informa cuánto
  se corrió cada flanco). Compila; **sin probar en placa** (prueba F4 abajo). `fb` prendido y `gm` apagado por
  defecto.
- [ ] 1: cero, la otra sostiene, signos, cuentas por grado (J2 / J3)
- [ ] 2: sostener (J2 / J3)
- [ ] 3: barrido de carga → feedforward (J2 / J3)
- [ ] 4: PI de velocidad (J2 / J3)
- [ ] 5: posición (J2 / J3)
- [ ] 6: movimientos coordinados
- [ ] 7: con carga

## Resultados

| Fecha | Paso | Resultado |
|---|---|---|
| 2026-10-31 | F1 (`-e j2`) | `a 28` 28,02° (2,26 s), `a 50` 50,21° (1,92 s), `a 48` 48,21°; la franja rechaza `a 75` (J3 en 30°), `a 26` (J3 atrasada en ~31,5°) y `o 7` (J2 en 48°); `x` → `o 23` vuelve a sostener. J2 sostenida queda 0,9–1,4° abajo |
| 2026-10-31 | F4 homing (`-e j2`, 20 kHz) | flanco respecto del anterior, 4 veces: J3 σ ≈ 0,02°, J2 σ ≈ 0,20° (a 1 °/s); un retroceso de J2 cortado ("sin cuentas", feedforward de 1 kHz) → J2: feedforward de 20 kHz y homing a 0,5 °/s |
| 2026-10-31 | F2 (`-e j2`, cero de J2 ~4° corrido) | `a 20` → 19,94° (error 0,06°, duty 0,2 %); `bi 0.1`: J3 en 25° pasa a 24,2° y no llega (`LLEGANDO`), en 10° va y viene 8,9 ↔ 10,6°; `bi 0.2`: J3 9,96° fija (duty −24,5 %); `fb`: las dos frenadas, J3 con 0,39° de error congelado, sin pitido; `a 25` → 25,57° y después 27,10° (`uHold` de otra pose) |
