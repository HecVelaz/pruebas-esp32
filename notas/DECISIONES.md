# Decisiones de pruebas-esp32

<!-- Movido desde CLAUDE.md (2026-10-22), sin cambios de texto. -->

- PlatformIO + Arduino core 2.0.17, placa `esp32-s3-devkitc-1` con N16R8 (`qio_opi`, 16 MB).
- Driver `MPU9250` propio, compartido por `symlink://` (no se copia).
- micro-ROS Agent (Jazzy) compilado desde el código fuente en `~/microros_ws`: no hay paquete apt.
- Publisher reliable con timeout de 10 ms. No se publica sin hora sincronizada. DLPF de 20 Hz contra el aliasing.
- Solo orientación: la posición no se integra del acelerómetro porque deriva. La altura necesita otro sensor.
- `autoRebuildAutocompleteIndex: false` en el workspace, para que VS Code no rompa el build de micro-ROS.
- WiFi: `AGENT_IP` fija en `include/wifi_config.h` (no versionado); cambiar de PC o de IP exige recompilar `-e wifi`.
- Roles: en modo normal Codex solo revisa (`codex exec --sandbox read-only`). Si el límite semanal de Claude Code se agota, Codex puede editar y hacer commits (modo respaldo) hasta que se reinicie. Ver "Roles" y "Continuidad entre agentes" en `AGENTS.md`: revisar `git log` antes de asumir continuidad de contexto.
  (Desde 2026-10-22 "Roles" y "Continuidad entre agentes" están en `CLAUDE.md` y `notas/FLUJO.md`;
  `AGENTS.md` es un symlink a `CLAUDE.md`.)

## Decisiones que están dentro de la bitácora (copiadas de `BITACORA.md`)

- GPS: `05-gps-ros2 -e serial` no se probó: se decidió usar solo WiFi. La protoboard daba falsos contactos: el GPS va conectado directo a la S3.
- J1: **decisión del usuario (2026-10-02): rango −45° (horario) / +135° (antihorario) desde el cero al costado** (no los ±75° de la E4); `ie`/`is` deshabilitados porque invertirían los límites asimétricos.
- J1 (`12`, 2026-10-30): **cero al frente del robot, rango −30° (hacia el mástil de la visión, choca a −38°) / +90°**, decisión del usuario (reemplaza la de arriba en `12` y `17`; `10` sigue con la vieja). Velocidad máxima 30 °/s (el usuario no quiso subirla).
- Brazo (2026-10-30, noche): **el cero de J1, J2 y J3 es la pose con los tres finales de carrera apretados**, + = alejarse de cada uno (reemplaza el cero al frente de J1 de esa mañana). J1 0–118°, J2 0–97°, J3 0–113° y la franja del cuatro barras `q2 − 25° ≤ q3 ≤ q2 + 7,5°`. Un solo final de carrera por articulación (el homing da la referencia; el otro lado, límites de software).
- J2 y J3 se controlan **juntas** (`19`): una activa y la otra sostenida por su lazo; después, movimientos coordinados (llegan juntas; la franja es convexa, así el camino recto queda adentro). `13`/`14` quedan como historia.
- `17`: la barra de J3 al revés (arriba = el codo sube) a pedido del usuario; J1 frena con rampa al soltar; joystick desde el celular con `--host 0.0.0.0` solo en una red de confianza.
- J1: Kalman probado offline y descartado; Ki 4 en vez del diseño (8,44) por el juego de la correa.
- J3 (5840): PWM a 1 kHz (rinde 2-3 veces más que a 20 kHz con el IBT-2); feedforward asimétrico con compensación de gravedad; rango de trabajo −12° a +25° (límite de arriba bajado a +28°).
- J2 (5840 en el IBT-2): `FRENO_EN_REPOSO = true` (el sin fin no se autobloquea con el peso); dither ±3 % (la mejor prueba).
- `10` (2026-10-10): sale el L298N y el encoder 38S6G5; los tres motores pasan a 36-555 con encoder SCX-555, un IBT-2 por articulación, EN común en el GPIO 23.
- `12`–`14` (2026-10-20): J1 con `FRENO_EN_REPOSO = true` (con el EN común, rueda libre suelta J2/J3); **VCC de los IBT-2 del brazo a 5 V, decisión del usuario**; si uno falla en `dg` o en el paso 0, probar ese con VCC a 3V3.
- `12` (2026-10-26): J1 sigue a **20 kHz** aunque a 1 kHz arranca con la mitad del duty (10 % contra 20 %): no chilla y zona muerta, arranque y ganancias están medidos a 20 kHz. J2/J3 (`13`, `14`, `17`) a 1 kHz: a 20 kHz J2 no arrancaba con menos de 40 %.
- `19` (2026-10-31): **J2 y J3 a 20 kHz, decisión del usuario** (reemplaza el 1 kHz de `13`/`14` en `19`): con la
  fuente a la bornera, J2 va y sostiene bien a 20 kHz (error ≤ 0,20°, duty de sostener estable) y no pita; el 40 %
  del 26/10 fue con la fuente por el adaptador XT60 de contacto malo. Zona muerta y arranque, a medir de nuevo a 20 kHz.
- `19`/`20` (2026-10-31): plan de 6 fases para preparar el test de repetibilidad estilo ISO 9283. **Freno dentro de
  la banda (`fb`) prendido por defecto** (al llegar, duty 0: quieto y sin ruido); **modelo de gravedad (`gm`) apagado**
  (a 20 kHz la fricción sostiene el brazo y el modelo empeoraba J2). Homing siempre despacio y desde el mismo lado,
  cero en el flanco (el golpe giraba los finales de carrera).
- `20` (2026-10-31): carpeta nueva **`20-brazo-coordinado`** para el control conjunto (19 queda como banco de
  pruebas); **J1 entra con su homing** (orden J3 → J2 → J1); el mismo controlador para las tres
  (`lib/Articulacion`); **llegada siempre con el ángulo creciendo** (+) en las tres; **J1 0–90° PROVISORIO** hasta
  medirlo con el brazo estirado; **J1 se autobloquea** (al llegar frena, como `12`); la **zona prohibida** (la caja)
  queda para después; el test de repetibilidad, primero en el plano J2–J3 con J1 en su cero.
- `20` (2026-10-31, tarde): **precisión antes que velocidad** (pedido del usuario). En `mv` todas las que se mueven
  llegan con la aproximación desde abajo (también las que suben); el sostener arranca del duty que sostiene el peso
  (medio del feedforward), no del duty con que venía; J3 con `tolPos` 0,15° (J2 vuelve a 0,3°: con 0,15° repetía
  peor); `khi` queda en 8. El perfil sigue quíntico (más suave que un trapecio).
- `17` (2026-10-26): joystick en lazo abierto (el joystick da el duty) y freno al soltar, nunca rueda libre; puente en Python porque Firefox no tiene Web Serial; cero donde está el brazo al abrir el puente (lo pidió el usuario).
- El firmware del robot vive en `~/Documentos/recolector_de_frutas`; `09-brazo-control` se movió allá (2026-09-27) y los arreglos de los drivers para el robot se hacen allá.
- `20` (2026-11-04): con la manivela de J3 de 95 mm, **borde de arriba de la franja con una tabla** (J2 → J3 máx,
  interpolada) en lugar de un ancho fijo; los puntos donde J3 arrastra a J2 sin margen y donde se traba con 2°;
  `MARGEN_FRANJA` 1°. `mv` en **dos tramos** por un punto intermedio cuando la recta sale de la franja (decisión del
  usuario: "hacelo con la tabla"). J2/J3 vuelven a los engranajes helicoidales (las correas se probaron y se sacaron).
- `20` (2026-11-04, noche): **homing sin límite de búsqueda** (pedido del usuario: bajar el brazo con `h` desde donde
  quedó); quedan el corte por trabada y el recorrido máximo. J1 (autobloqueo) da por llegada al **cruzar** el destino
  (frena sin invertir). "Sin cuentas" de J2/J3 a 60 %.


## 2026-11-05 — Juego de J2/J3: por software, sin cambios mecánicos

- J3 tiene ~25–30° de juego en el antebrazo (un diente entre el piñón de 15 y el engranaje de 75, multiplicado por el
  cuatro barras) y J2 algunos grados. El usuario no puede cambiar la mecánica: se compensa por software. El peso apoya
  el juego siempre del mismo lado en las poses quietas (repetibilidad 0,05–0,18°), así que la posición de la punta se
  calibra con mediciones (video o, mejor, las cámaras estéreo del robot) y el final del agarre se corrige con la visión.
  Se trabaja con q3 − q2 ≥ −25° (más abajo J3 no queda firme) y llegando siempre desde abajo.
- La escala de J2 (1,170° reales por grado del encoder: reductora ~120:1, no 139:1) **no se cambió en el firmware**:
  las ganancias, la franja y los límites están afinados en grados de encoder; la conversión va en la cinemática
  hasta decidir lo contrario.

## 2026-11-05 (noche) — Dos bordes de abajo de la franja; la punta en mm desde Python

- **Destinos con q3 − q2 ≥ −25°, camino y freno con −49°.** −49° es el límite mecánico (el hombro no sube más con el
  codo atrás); −25° es de firmeza (más abajo J3 no queda quieta). Por eso `Q2_MENOS_Q3_DESTINO` (25) vale solo donde
  J2/J3 quedan sostenidas (`mv`, `j`, joystick) y `Q2_MENOS_Q3_MAX` (49) sigue para el camino del `mv`, su intermedio,
  el homing y el freno de seguridad: pasar por ahí moviéndose no es problema y achicar todo daba cortes nuevos.
- **`mx x y z` en Python (`tools/punta.py`), no en el firmware.** El firmware no conoce la caja ni el modelo de la
  punta; el script revisa los choques en todo el camino (para cualquier J1 entre la de partida y la de llegada) y
  manda `mv`. Sin flashear, la repetibilidad medida sigue valiendo, y es lo que va a usar el puente de la Pi con las
  cámaras. Pasarlo al firmware solo si hace falta la punta en recta. Con `mv` a mano no hay protección contra la caja.
- **Franja cargada desde las anotaciones del joystick (2026-11-10 noche):** **A** (J2 se corre sola) sin margen, **T**
  (traba mecánica) con medido − 2°, **L** (llegó sin nada) con lo alcanzado, que es seguro aunque el borde esté más
  arriba. Las tablas tienen que crecer con q2 (el firmware las invierte para el joystick y `j`): si una medición nueva
  contradice una vieja, se descarta la vieja en vez de mezclarlas. El borde de abajo es tabla **más** las rectas
  (`q2 − 49` camino, `q2 − 25` destinos): manda la más alta.
- **Pose del homing en 3 / 6 / 3 (pedido del usuario, 10/11 noche):** J2 lo más recogida posible con J3 en 3°. J3
  retrocede cuando J2 pasa 5° subiendo, no cuando J2 queda firme (la espera de "firme" era la que tardaba).
- **Caja en el firmware como regla empírica, no con el modelo (2026-11-11):** con J1 ≥ `CAJA_J1_MIN`, J3 máximo según
  J2 (tabla). La tabla sale del modelo porque es una regla de altura y la altura la acierta; desde qué J1 rige sale
  de lo visto en el brazo (55,6 − 2°), porque en el costado el modelo se equivoca ~5 cm. Más conservadora que el
  modelo: no deja bajar la muñeca por delante del borde con J1 al frente. Con la cinemática recalibrada, se puede
  pasar a una regla con el modelo (sobre la tapa o delante).
