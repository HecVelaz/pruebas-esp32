# 20-brazo-coordinado — J1, J2 y J3 con el mismo controlador

Fase 5 del plan de `19-j2-j3-conjunto` (preparar el test de repetibilidad estilo ISO 9283): las tres articulaciones
del brazo con **el mismo controlador** (`lib/Articulacion`, sin `Serial` ni pines fijos) en **un solo lazo no
bloqueante a 100 Hz**. `19` queda como banco de pruebas de una articulación por vez.

- **Controlador** (de `12`/`19`, probado en placa): cascada P de posición → PI de velocidad con feedforward de
  fricción (desde el 03/11, paso 2: **el signo sale de la velocidad de la trayectoria**, no de la pedida con la
  corrección, y entra suave: ff · tanh(|w| / 0,5 °/s); el arranque sin salto usa el mismo ff); al llegar, sostener con banda del integrador (0,2°), compensación de fricción (J3 15 %) y **freno dentro
  de la banda** (firme → duty 0, sin ruido). Protecciones: sin cuentas, al revés, atasco, límites.
- **Homing `h`:** con el brazo abajo (J2 sin cero o a ≤ 5°, `HOMING_Q2_JUNTAS`) **las tres juntas** (desde el 03/11:
  J2 y J3 bajan a la misma velocidad y quedan dentro de la franja); si no, J3 → J2 (con el codo abajo, bajar el hombro
  no viola la franja) → J1 con el brazo recogido. **Sin cero corta si una avanza más de 15°** sin su final de carrera
  (`HOMING_RECORRIDO_SIN_CERO`, 2026-11-03: arrancó estirado, J3 subió 28° y se trabó contra la franja). Cada una llega a su final de carrera **despacio** (las tres a 1,5 °/s desde el 03/11; antes J2 0,5 y
  J3/J1 1 °/s: el golpe giraba los finales de carrera), cero en el flanco, retrocede 3° y queda sostenida. Informa cuánto se corrió cada flanco
  (`HOMING,t,d1,d2,d3,con_cero`).
- **Franja del cuatro barras** `q2 − 25° ≤ q3 ≤ q2 + 7,5°`: los destinos se validan con 3° de margen y, si la
  medición sale de la franja (0,5° de holgura) mientras J2 o J3 se mueven, frena las dos.
- **J1:** 0–90° PROVISORIO (118° medido con el brazo recogido; estirado choca antes con el soporte de la visión).
- **Transmisión de J3** (dato del usuario, 2026-11-03): motor 139:1 → engranajes 75:15 (5:1) → manivela de 80 mm en
  el engranaje grande → pivote a 40 mm del eje del antebrazo (cuatro barras: la pinza queda paralela). **q3 es el
  ángulo del engranaje grande** (123,6 cuentas/°): la manivela y la palanca quedan fuera del lazo. La palanca divide el
  par por 2 y **multiplica el ángulo por ~2** (exacto solo con las barras perpendiculares; cambia con el ángulo):
  `tolPos` 0,15° de J3 ≈ 0,3° del antebrazo, el juego de los engranajes también ×2, ~62 cuentas por grado de
  antebrazo. Par motor → antebrazo ≈ 139 × 5 / 2 ≈ 347. Las ganancias y la franja se midieron así (en ángulo del
  engranaje), por eso valen; si J3 se portara distinto en los extremos, pensar en ganancias según la posición.
- **20 kHz** las tres (sin pitido). EN común: alto mientras alguna se sostiene; solo `x` lo baja.
- **Abrir el puerto serie reinicia la WROOM** (el cero se pierde y las tres quedan sueltas): después de abrir el
  monitor, `h`.

## Conexiones

Las del robot (v0.5), como `12`, `17` y `19`: J1 IBT-2 21 · 22, encoder 34 · 35, final de carrera 32; J2 25 · 26,
36 · 39, **27**; J3 16 · 17, 18 · 19, **33**; EN común 23. Pines en [`include/pins.h`](include/pins.h).

## Cómo flashear y abrir el monitor

```bash
cd ~/Documentos/pruebas-esp32/20-brazo-coordinado
~/.platformio/penv/bin/pio run -t upload    # 12 V APAGADOS
tools/monitor.sh                            # salir con Ctrl+]
```

## Comandos (fase 5a)

`h` homing de las tres · `j <n> <°>` mover una (1, 2 o 3) dentro de sus límites y de la franja · `mv <q1> <q2> <q3>`
las tres a la vez (5b) · `t [1|0]` telemetría a 50 Hz (5c; no corta el `mv`) · `s` sostener las
tres donde están · `z` cero acá (en los finales de carrera; mejor `h`) · `c` estado · `fb` freno en banda sí/no ·
`ff <n> <desp+> <marcha+> <desp-> <marcha->` feedforward de una (en vivo) · `x` rueda libre de las tres · `?` ·
`jv <a1> <a2> <a3>` joystick (abajo) · `zq <q1> <q2> <q3>` cero ESTIMADO: el brazo está en estos ángulos.

**`zq`** (2026-11-03): sin cero el homing supone el brazo abajo y recogido. Si quedó arriba (abrir el puerto reinicia la
WROOM y pierde el cero, pero el brazo no se mueve sin corriente), el homing lo traba contra la franja. Con `zq` y los
ángulos donde está (de la última anotación o captura) se habilitan `j` y el joystick para bajarlo; fuera de la franja
solo dejan volver hacia adentro (la protección de la franja no frena al joystick: él ya no deja alejarse). Después,
abajo y recogido, `h` toma el cero de verdad.

**Joystick `jv`** (2026-11-03, para medir la caja sin cambiar de firmware): cada eje, de −1 a 1, es la velocidad de su
articulación **en lazo cerrado** (la referencia avanza a esa velocidad, a fondo `JOY_FACTOR_VEL` 0,4 de vMax: J1
12 °/s, J2 y J3 8 °/s; acelera y frena con la mitad de aMax) y la cascada la sigue. Al soltar (eje en 0) frena y **se
sostiene donde quedó** (J2 y J3 no caen). No pasa los límites ni la franja con 3° de margen: frena a tiempo para
quedar en el borde. Hombre muerto: sin `jv` en 300 ms, frena las tres. Necesita el cero (`h`). `jv 0 0 0` con el
joystick quieto no corta el homing ni un `mv`; cualquier otro comando corta el joystick (se sostienen). Corta y frena si
una se aleja más de 5° de su referencia o la franja la frena. Telemetría: etapa 8. Desde el monitor un `jv` mueve unos
300 ms y frena (sirve para probar); para manejarlo, `python3 ../17-brazo-joystick/tools/joystick.py --placa 20`
(página en http://localhost:8000: barras de velocidad, botón Homing, "Anotar" guarda en `resultados/limites_*.csv`;
ver el README de `17`).

## Prueba 5a

Orden de la fuente: **12 V apagados → flashear → monitor → brazo abajo y recogido → 12 V encendidos → comandos →
brazo apoyado → 12 V apagados**. Fuente a la bornera, 3 A.

1. `h` (sin cero): J3, J2 y J1 a sus finales de carrera y a 3°. `c`: las tres `sosteniendo, firme, frenada`.
2. `h` 5 veces: los flancos respecto del anterior (repetibilidad del homing de las tres).
3. `j 2 15` → `j 3 10` → `j 1 30` → `j 1 3` → `j 3 3` → `j 2 3` (cada una mientras las otras se sostienen). J3 con el
   feedforward PROVISORIO: si no arranca o se pasa, `ff 3 ...`.

## Prueba 5b

Después de `h` (las tres en 3°), con poses que se sepa que **no chocan con la caja**:
`mv 20 15 10` → `mv 3 3 3` → `mv 40 30 20` → `mv 20 20 24` → `mv 3 3 3`. Mirar que **arranquen y terminen juntas**,
que las que bajan **se pasen 2° y vuelvan subiendo despacio**, y anotar cada `LLEGO,...` (errores). Repetir un mismo
`mv` varias veces desde poses distintas y comparar los `LLEGO` (adelanto de la fase 6).

## Prueba 5c (telemetría)

`tools/trayectoria.py` reemplaza al monitor (cerrarlo antes): manda `t 1`, los comandos de la lista esperando cada
`LLEGO` (o `HOMING`), captura `--cola` s más, **vuelve a `mv 3 3 3`** si la lista no terminaba ahí (cerca del cero:
el próximo homing tarda poco; no entra en el resumen; `--sin-volver` para no hacerlo) y manda `t 0`. Guarda `resultados/trayectoria_<fecha>_<hora>.csv` y su
`.png`: por articulación referencia contra medido, el error con `tolPos` y `bandaInt` (zoom ±1,5°: ahí se ve el
tambaleo del final) y el duty, con lo firme y lo frenado sombreado; abajo, el camino (q2, q3) sobre la franja. Abrir
el puerto puede reiniciar la WROOM: si queda sin cero, empezar la lista con `h`.

Imprime un **resumen para comparar capturas** (siempre con la misma lista de `mv`): por `mv` y articulación, las
inversiones del duty (cambio de signo con salto > 15 %) en el recorrido y en la aproximación, el |error| máximo de
seguimiento en el recorrido, el error del `LLEGO`, las correcciones después de llegar y cuánto se mueve el encoder ya
firme; por `mv`, el tiempo de la llegada hasta las tres firmes; al final, los totales y cuánto se movió cada encoder en
la cola. `--comparar` pone los totales al lado de los de otra captura. Base con el resorte de la pinza
(`trayectoria_20261031_180843`): inversiones J1 20 + 61 = 81, J2 7 + 8 = 15, J3 28 + 12 = 40; |e LLEGO| máx 0,06 /
0,12 / 0,18°; de la llegada a firmes 0,2–0,5 s (suma 1,5 s).

```bash
python3 tools/trayectoria.py h "mv 20 20 24" "mv 40 30 20" "mv 20 20 24" "mv 3 3 3"
python3 tools/trayectoria.py --archivo resultados/trayectoria_....csv    # volver a graficar
python3 tools/trayectoria.py --archivo resultados/trayectoria_....csv --solo-resumen \
    --comparar resultados/trayectoria_20261031_180843.csv                 # contra la base
```

Línea de telemetría: `T,ms,etapa,` y por cada una `q,ref,duty,w,estado` (etapa del `mv`: 1 recorrido,
2 aproximación, 3 llegada, 4 asentamiento, 8 joystick, 9 homing; estado: 0 suelta, 1 yendo, 2 llegó, 3 firme, 4 frenada,
5 siguiendo, 6 homing). Al prenderla, `TELE_CFG` con `tolPos` y `bandaInt` de las tres; al apagarla, `TELE_FIN,<perdidas>`.

## Estado

- [x] 5a: biblioteca, sostener las tres, homing de las tres, `j` (**probada en placa 2026-10-31**: flancos al nivel
  de una cuenta del encoder; `j` firmes con error ≤ 0,25°)
- [x] 5b (**probada en placa 2026-10-31**: terminan juntas, las que bajan se pasan y vuelven subiendo; errores ≤ 0,46°): `mv q1 q2 q3`: las tres siguen el mismo perfil quíntico
  (terminan juntas; recta en (q2, q3) dentro de la franja), a la mitad de su vMax/aMax (`MV_FACTOR_VEL`,
  PROVISORIO); todas las que se mueven (desde el 31/10 también las que suben) terminan 2° por debajo y suben el último
  tramo a 2 °/s (llegada siempre con el ángulo creciendo; desde el 03/11 la que ya está dentro de su `tolPos` no se
  mueve); el sostener arranca del duty que sostiene el peso; a los 2 s firmes: `LLEGO,t,q1,q2,q3,e1,e2,e3`. Corta si alguna se aleja más de 5° de su referencia, con
  la franja o con cualquier comando. **No esquiva la caja.**
- [x] 5c (**probada en placa 2026-10-31**, sin muestras perdidas): telemetría `t` y `tools/trayectoria.py`
- [ ] Joystick `jv` (lazo cerrado, se sostiene al soltar) y `joystick.py --placa 20`: compila, sin probar en placa
- [ ] Zona prohibida: que el brazo no choque con la caja (después; medirla con el joystick)

## Resultados

| Fecha | Prueba | Resultado |
|---|---|---|
| 2026-10-31 | 5b `mv` (`b5a0763`) | Arrancan y terminan juntas; las que bajan se pasan ~2° y vuelven subiendo despacio. `LLEGO` (J1 · J2 · J3): `mv 20 15 10` 19,94 · 14,77 · 10,17 · `mv 3 3 3` 2,82 · 3,00 · 3,46 · `mv 40 30 20` 39,68 · 30,21 · 20,02 · `mv 20 20 24` 19,94 · 20,11 · 24,16 · `mv 3 3 3` 2,99 · 3,00 · 3,04. Error máx 0,46° (J3, primer `mv 3 3 3`); el resto ≤ 0,32° |
| 2026-10-31 | Todas las que se mueven llegan con la aproximación (`trayectoria_20261031_180843`) | J2 en el primer `mv 20 20 24` (sube directo) 20,00 (antes 20,27). Dos llegadas a `20 20 24`: J1 19,94 / 19,94, J2 20,00 / 20,01 (0,008°), J3 24,09 / 24,05 (0,04°); de la llegada a firmes 0,2–0,5 s; 4 `mv` en 30 s. Queda J3: en la subida a 2 °/s se traba y se pasa (error máx 0,46° y 0,55°; `mv 3 3 3` termina 0,18° arriba) |
| 2026-10-31 | Sostener arranca del duty que sostiene el peso (`trayectoria_20261031_180331`) | `uHold` = medio entre las marchas del feedforward (J2 +8,5 %, J3 −10 %) en vez del duty con que venía: J2 ya no cae (error máx al final del primer `mv` 1,68 → 0,40°), de la llegada a firmes 0,3–0,4 s en los cuatro (antes hasta 18,5 s), los 4 `mv` en 28 s (antes 48). A la vista, menos tambaleo. Queda: subiendo directo, J2 se adelanta 1,3° al frenar y termina 0,25–0,27° arriba (dentro de `tolPos`); con la aproximación desde abajo, 0,00° |
| 2026-10-31 | 5c telemetría (`trayectoria_20261031_175424`) | 4 `mv` + homing, 6990 muestras. (1) Primer `mv 20 20 24`: J2 llega adelantada y frenando con duty −20 %; el sostener arranca de ese duty y **cae 1,7°** antes de volver (el tambaleo). (2) Aproximación a 2 °/s con stick-slip: J2 1,4° y J3 0,9° atrás hasta despegar. (3) `mv 3 3 3`: J3 trabada en el borde de `tolPos` 0,15° y 18,5 s hasta quedar firme. Feedforward medido en el recorrido (J2 +24,4 / −10 %, J3 +11,9 / −33 %) ≈ el de `config.h` |
| 2026-10-31 | 5b con `khi` 4 en J2 y J3 (en vez de 8) | Peor: 3 de 7 `mv` cortados por `no quedan firmes`; J2 llega con +0,30° de error (19,70 dos veces) y el tambaleo no mejora a la vista. → `khi` vuelve a 8 |
| 2026-10-31 | 5b con J2 en 0,3° / 0,2° y J3 en 0,15° / 0,1° | 8 `mv` sin cortes. 4 llegadas a `mv 20 20 24`: J1 19,94 ×4 (0,00°), J2 19,94 · 20,05 · 19,83 · 20,00 (0,22°), **J3 24,18 ×4 (0,008°, una cuenta)**; J3 queda siempre 0,18° por encima (después de frenar). Sigue el tambaleo en la punta (sin saber de cuál: falta la telemetría) |
| 2026-10-31 | 5b con J2/J3 `tolPos` 0,15°, `bandaInt` 0,1° | 3 llegadas a `mv 20 20 24`: J1 20,08 (rango 0,00°), J2 20,14 · 19,99 · 19,85 (**0,29°**, peor), **J3 24,18 · 24,14 · 24,18 (0,03°)**. Una vez `no quedan firmes`. Al final se ve un tambaleo en la punta: se traba por fricción, el integrador sube el duty, despega de golpe y se pasa (stick-slip), y el juego de los engranajes y el brazo largo lo amplifican. → J2 vuelve a 0,3° / 0,2°; J3 queda en 0,15° / 0,1° |
| 2026-10-31 | 5b repetir `mv 20 20 24` desde poses distintas | 5 llegadas (J1 · J2 · J3): 19,94 · 20,11 · 24,16 / 19,96 · 20,05 · 23,88 / 19,96 · 20,11 · 24,18 / 19,96 · 20,09 · 24,17 / 20,03 · 20,11 · 23,91. Rango J1 0,09°, J2 0,06°, **J3 0,30°** (cae en cualquier lugar de la tolerancia `tolPos` 0,3°: al quedar firme frena y no corrige más). Correcciones chicas en la punta (stick-slip). → J2/J3 `tolPos` 0,15°, `bandaInt` 0,1° |
| 2026-10-31 | 5a homing de las tres (`a89d415`) | 4 vueltas: J1 0,000°, J2 ±0,008°, J3 ±0,010° (1 cuenta: J1 0,024°, J2/J3 0,008°). `j 2 15` 14,88 · `j 3 10` 10,23 · `j 1 30` 29,77 · `j 1 3` 3,13 · `j 3 3` 3,09 · `j 2 3` 2,75, firmes y frenadas |
| 2026-10-31 | 5a homing de las tres | flanco respecto del anterior (6 vueltas): J3 σ ≈ 0,013°, J2 σ ≈ 0,014°, J1 σ ≈ 0,03° (3 vueltas). J1 iba y venía en 3° después del retroceso → `autobloqueo` (freno al llegar, como 12) |
| 2026-11-03 | Paso 0: línea de base **sin el resorte de la pinza** (`trayectoria_20261103_092501`, firmware 37d8827) | Contra 180843 (con el resorte): inversiones del duty > 15 % J1 81 → 38, J2 15 → 13, J3 40 → 34 (las de J1, 32 de 38 en la aproximación); \|e\| máx en el recorrido J1 0,90°, J2 2,48° (antes 1,91: más carga), J3 1,28°; \|e LLEGO\| máx 0,105 · 0,084 · 0,062°; de la llegada a firmes 0,2–0,5 s (igual); encoders quietos los 5 s de la cola. J2 y J3 no caen al llegar: el feedforward de `config.h` sigue sirviendo (duty en el recorrido J2 +34 / −24 %, J3 +19 / −41 %, antes +31 / −19 % y +15 / −41 %). Homing 16,6 s (busca a velocidad lenta: el reinicio al abrir el puerto pierde el cero) |
| 2026-11-03 | Homing más rápido, J2 1 °/s, J3 y J1 1,5 °/s (`trayectoria_20261103_094631`, `c0c44ca`) | Homing desde ~3° del cero **16,6 → 12,1 s** (buscando: J3 3,5 → 2,2 s, J2 6,3 → 3,4 s, J1 2,3 → 2,2 s; J1 no llega a 1,5 °/s, mediana 0,6). Los `mv` sin cambios contra 092501: \|e LLEGO\| máx 0,042 · 0,108 · 0,078°, de la llegada a firmes ≤ 0,5 s; inversiones J1 51 (antes 38), J2 13, J3 27 |
| 2026-11-03 | Homing con J2 a 1,5 °/s, todavía J3 → J2 → J1 (`trayectoria_20261103_094915`, `b5f1648`) | Homing 12,1 → **10,9 s** (J2 buscando 4,0 s con el retroceso); `mv` igual (\|e LLEGO\| máx 0,23° J1 en el primer `mv`, el resto ≤ 0,12°) |
| 2026-11-03 | Homing J1 junto con J3, J2 a 1,5 °/s (`trayectoria_20261103_095335`, `b5f1648` + `bb9dfae`) | Homing desde ~3° **12,1 → 7,7 s** (16,6 al empezar el día): J3 y J1 2,1 s juntas, J2 2,3 s. Stick-slip buscando: J1 toca el flanco a 4–5 °/s (igual que antes), J2 a 3 °/s (antes 0,8): a 100 Hz, ≤ 0,05° de error en el cero. Los `mv` igual: \|e LLEGO\| máx 0,13 · 0,16 · 0,08°, de la llegada a firmes ≤ 0,5 s |
| 2026-11-03 | Paso 1a: J1 sola con el brazo recogido (`trayectoria_20261103_103503`) | J1 20 ↔ 60° con J2 y J3 en 3°: **J2 y J3 ni se mueven** (0,01°). J1 en la aproximación a 2 °/s, **stick-slip**: trabada con 16–18 % y salta 1,7° a 6 °/s; al frenar en la llegada **retrocede 0,1° (la correa)** y queda 0,38° corta en 60° (dentro de `tolSalida` 0,6°, firme). Inversiones J1 36 (25 en la aproximación). El encoder de J1 está en el motor (antes de la correa): una oscilación de la base por la correa no se ve en él |
| 2026-11-03 | Homing de las tres juntas (`trayectoria_20261103_104057`, `42e5575`) | **3,5 s** (desde ~3°); flancos −2,7 a −3,1° como siempre |
| 2026-11-03 | Paso 1b: J1 20 ↔ 60° con el brazo estirado (74 / 70) y el limón (`trayectoria_20261103_104300`) | J3 quieta (0,02°). J1 llega bien (59,98 / 20,03) con saltos chicos (0,4°) en la aproximación. **J2 también se mueve** (después del primer `mv` quedó a 0,137° > 0,05° del objetivo: hace la aproximación aunque solo se pida J1) y es la que más se nota: **trabada 1 s** en la aproximación (hasta 1,5° atrás, el duty sube de 11 a 27 %), despega a 6,7 °/s, **se pasa 0,72°** y baja despacio 0,43° en 1,4 s (queda 0,29° arriba, firme por `tolSalida`); de la llegada a firmes 1,6–1,9 s en los `mv 60`. La vuelta a `mv 3 3 3` tardó 29 s: J2 quedó a 0,313° (> `tolPos` 0,3) trabada y el sostener subió el duty de 10 a 38 % en 11 s hasta moverla 0,016° |
| 2026-11-03 | 1b otra vez, la que ya está en `tolPos` no se mueve (`trayectoria_20261103_104918`, `a4d4a99`) | Contra 104300: **J2 quieta** (inversiones 92 → 14, correcciones 16 → 0, \|e LLEGO\| 0,29 → 0,10°), de la llegada a firmes peor 1,9 → **0,3 s**. Ahora en el video corrige **J1**: en la aproximación a 2 °/s se traba, salta a 5–7 °/s, **se pasa 0,3–0,6°** de la referencia y el feedforward **cambia de signo (+17 → −19 %)** para volver (−3,6 °/s): ida y vuelta al final. 25–27 inversiones por aproximación de J1 |
