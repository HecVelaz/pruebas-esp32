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
- `17` (2026-10-26): joystick en lazo abierto (el joystick da el duty) y freno al soltar, nunca rueda libre; puente en Python porque Firefox no tiene Web Serial; cero donde está el brazo al abrir el puente (lo pidió el usuario).
- El firmware del robot vive en `~/Documentos/recolector_de_frutas`; `09-brazo-control` se movió allá (2026-09-27) y los arreglos de los drivers para el robot se hacen allá.
