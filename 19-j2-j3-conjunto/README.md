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
  "Llegó" = su referencia llegó al destino (en la fase 2, además dentro de la tolerancia y quieta).
- **Cero:** `z` con los finales de carrera de J2 y J3 apretados (fotos en `17-brazo-joystick/LIMITES/ceros_bumpers`).
  Desde ahí la otra empieza a sostener en 0°. **+ = alejarse del final de carrera** (J2 antihorario visto desde
  atrás, J3 el codo baja).
- Después (paso 6): movimientos coordinados de las dos a la vez.

## Conexiones

Las del robot (v0.5), como `12`–`14` y `17`: J2 IBT-2 25 · 26, encoder 36 · 39, final de carrera **27**; J3 IBT-2
16 · 17, encoder 18 · 19, final de carrera **33** (J2 y J3 cruzados respecto de v0.5); EN común 23. J1 queda frenada.
PWM 1 kHz. Pines en [`include/pins.h`](include/pins.h).

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
80°) y `a 75`: espera a J3 y recién ahí va a 75°. Volver: `a 50` → `o 30` → `a 28` → `o 7` → `a 3`. Por último, brazo
apoyado, `x` y `o 0`: J3 vuelve a sostener y J2 queda frenada, sin soltarse (y J1 no se suelta).

Los scripts: **sin `--cero`** (mandaría `z` lejos de los finales de carrera); cerrar el monitor antes.

**Comandos nuevos:** `o <°>` llevar la otra y sostenerla · `oc` dónde está la otra (medido, referencia, destino,
`EN CAMINO` / `en destino`) y los límites de la activa con el peor caso ·
`okp`, `oki` ganancias del sostener de la otra · `x` suelta las dos. El resto, como `12`–`14` (`?`).

## Estado

- [x] Compila (`-e j2` y `-e j3`, 2026-10-30). Sin probar en placa.
- [x] Fase 1, seguridad de la franja (2026-10-31): límites con el peor caso de la otra, la activa espera a que la
  otra llegue, `o` sin `coast()`. Compila (`-e j2` y `-e j3`); **sin probar en placa** (prueba F1 abajo).
- [ ] Fase 2: sostener sin oscilación (banda del integrador), "llegó" de la otra, asentamiento
- [ ] Fase 3: mapa de gravedad (J2 y J3 se cargan mutuamente)
- [ ] Fase 4: homing automático
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
| | | |
