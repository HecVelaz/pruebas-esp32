# 20-brazo-coordinado — J1, J2 y J3 con el mismo controlador

Fase 5 del plan de `19-j2-j3-conjunto` (preparar el test de repetibilidad estilo ISO 9283): las tres articulaciones
del brazo con **el mismo controlador** (`lib/Articulacion`, sin `Serial` ni pines fijos) en **un solo lazo no
bloqueante a 100 Hz**. `19` queda como banco de pruebas de una articulación por vez.

- **Controlador** (de `12`/`19`, probado en placa): cascada P de posición → PI de velocidad con feedforward de
  fricción; al llegar, sostener con banda del integrador (0,2°), compensación de fricción (J3 15 %) y **freno dentro
  de la banda** (firme → duty 0, sin ruido). Protecciones: sin cuentas, al revés, atasco, límites.
- **Homing `h`:** J3 → J2 → J1 (con el codo abajo, bajar el hombro no viola la franja; J1 al final con el brazo
  recogido). Cada una llega a su final de carrera **despacio** (J2 0,5 °/s, J3 y J1 1 °/s: el golpe giraba los
  finales de carrera), cero en el flanco, retrocede 3° y queda sostenida. Informa cuánto se corrió cada flanco
  (`HOMING,t,d1,d2,d3,con_cero`).
- **Franja del cuatro barras** `q2 − 25° ≤ q3 ≤ q2 + 7,5°`: los destinos se validan con 3° de margen y, si la
  medición sale de la franja (0,5° de holgura) mientras J2 o J3 se mueven, frena las dos.
- **J1:** 0–90° PROVISORIO (118° medido con el brazo recogido; estirado choca antes con el soporte de la visión).
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
las tres a la vez (5b) · `s` sostener las
tres donde están · `z` cero acá (en los finales de carrera; mejor `h`) · `c` estado · `fb` freno en banda sí/no ·
`ff <n> <desp+> <marcha+> <desp-> <marcha->` feedforward de una (en vivo) · `x` rueda libre de las tres · `?`.

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

## Estado

- [x] 5a: biblioteca, sostener las tres, homing de las tres, `j` (**probada en placa 2026-10-31**: flancos al nivel
  de una cuenta del encoder; `j` firmes con error ≤ 0,25°)
- [x] 5b (compila 2026-10-31; **sin probar en placa**): `mv q1 q2 q3`: las tres siguen el mismo perfil quíntico
  (terminan juntas; recta en (q2, q3) dentro de la franja), a la mitad de su vMax/aMax (`MV_FACTOR_VEL`,
  PROVISORIO); las que bajan terminan 2° por debajo y suben el último tramo a 2 °/s (llegada siempre con el ángulo
  creciendo); a los 2 s firmes: `LLEGO,t,q1,q2,q3,e1,e2,e3`. Corta si alguna se aleja más de 5° de su referencia, con
  la franja o con cualquier comando. **No esquiva la caja.**
- [ ] 5c: telemetría (`t`) y `tools/trayectoria.py`
- [ ] Zona prohibida: que el brazo no choque con la caja (después)

## Resultados

| Fecha | Prueba | Resultado |
|---|---|---|
| 2026-10-31 | 5a homing de las tres (`a89d415`) | 4 vueltas: J1 0,000°, J2 ±0,008°, J3 ±0,010° (1 cuenta: J1 0,024°, J2/J3 0,008°). `j 2 15` 14,88 · `j 3 10` 10,23 · `j 1 30` 29,77 · `j 1 3` 3,13 · `j 3 3` 3,09 · `j 2 3` 2,75, firmes y frenadas |
| 2026-10-31 | 5a homing de las tres | flanco respecto del anterior (6 vueltas): J3 σ ≈ 0,013°, J2 σ ≈ 0,014°, J1 σ ≈ 0,03° (3 vueltas). J1 iba y venía en 3° después del retroceso → `autobloqueo` (freno al llegar, como 12) |
