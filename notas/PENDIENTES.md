# Pendientes de pruebas-esp32 (al 2026-10-30)

<!-- Movido desde CLAUDE.md (2026-10-22). Los bloques "Pendiente GPS", "Pendiente" de 08 y "Falta" se movieron
     tal cual; el resto se copió de los "Pendiente:" que siguen dentro de cada entrada de BITACORA.md. -->

## Brazo (12, 17, 19, lo próximo)

- **`12`: repetir el paso 3** (`escalon_posicion.py -a j1 20 --ciclos 5`, desde unos 40–60° para tener lugar) con el cero en el final de carrera (`z` con J1 apretándolo); flasheado nuevo, sin probar.
- **`19-j2-j3-conjunto`** (lo próximo después de `12`): probar en placa con `-e j2` y después `-e j3`: paso 1 (cero, la otra sostiene, signos, cuentas por grado), 2 sostener, 3 barrido de carga → feedforward de gravedad, 4 PI de velocidad, 5 posición; programar el paso 6 (movimientos coordinados: llegan juntas) y 7 limón. Antes, revisión de Codex (solo lectura) si se quiere.
- `17-brazo-joystick`: probar en placa la rampa de bajada de J1, la barra de J3 al revés y el límite de J1 en 118°. Mejora: sostener J2/J3 con un lazo al soltar (hoy frenadas bajan despacio). Límite de J1 con el brazo estirado (choca antes con el soporte): sin medir. Después, fusionar con la tracción por joystick en `recolector_de_frutas`.
- Al robot (`recolector_de_frutas`, en una sesión de ese repo): J1 polea de 84 (84/18; factor de las ganancias en rpm ×1,286 en vez de ×1,2); cero de las tres en los finales de carrera, + = alejarse de cada uno, J1 0–118°, J2 0–97°, J3 0–113°, franja `q2 − 25° ≤ q3 ≤ q2 + 7,5°` (comparar con `GAMMA4_*` y el marco de `brazo_ik`, `cinematica.py` y el URDF, que usan otro cero); pines `PIN_FC_J2 = 27`, `PIN_FC_J3 = 33` y sus docs.
- Fotos de `12-j1-base/CERO/` y `17-brazo-joystick/LIMITES/` sin commitear: aparecen personas (repo público); recortarlas o sacar otras.
- J1 (`12`): `dg` (paso A). Opcional: `VMIN_POS` 3 → 5 °/s (o tolerancia 0,5°) para el stick-slip del final; subir `VMAX_POS` de a poco (45 / 90 °/s²) si hace falta más velocidad; video para la Entrega 6.
- `15`/`16`: anotar en sus README los resultados de J3 con el C3 (IBT-2 y L298N), si se hicieron.
- Hombro (`10`, 2026-10-13): medir con `bc` el duty según el ángulo y hacer compensación de gravedad en el hombro; la USB se desconectó una vez con un pulso fuerte (revisar masas).
- J3 con el 5840 (`10`): probar `--bj 6 40 3` (dither), lubricar los engranajes, prueba con el limón, duty máx a ~70 %, resorte o contrapeso, pasar J3 al IBT-2 en `recolector_de_frutas` (cambiar J3 a IBT-2 en `recolector_de_frutas/firmware/brazo` y sus docs).
- J2 con el 5840 (`10`): medir con una balanza de mano la fuerza para sostener el brazo y la geometría para dimensionar la compensación (resorte o contrapeso); duty máx en 80 % (temporal); la fuente tiene que volver a 2 A; probar bajar con duty fijo sin PI (`m -8 12`, `m -8 15`, `m -8 20`, volviendo con `m 0 60`); idea de Kp distinto para subir (0,35) y bajar (0,2); mecánico: resorte a gas o fricción en el eje del hombro, lubricar; después limón, duty máx a ~55 %, límites definitivos.
- Informe (`10`): escalones de velocidad de J2 y J3 con Kp 0,35 / Ki 5; modelo de J2; el CSV de `ep` debería guardar todo el feedforward y `bj` (hoy se pasan con `--ff`/`--bj` al rehacer); `firmware_colab.md` todavía usa q en vez de J. Los simuladores de `tools/` todavía usan una sola zona muerta.

## Motores en lazo abierto (06–08)

- 06: pull-down de 10 kΩ en R_EN/L_EN (sin ellos: apagar los 12 V antes de flashear o resetear); lazo cerrado (PID con feedforward de la zona muerta).
- 08: medir el umbral de arranque entre 20 y 40 % (perfil con `--niveles 25 30 35 40 60 80 100` o barrido) para el feedforward del lazo cerrado; pull-down de 10 kΩ en ENA; arreglo del salto falso de 32 000 cuentas en `EncoderPCNT` (biblioteca compartida con 06/07, sin aplicar); los mismos arreglos de la revisión en `graficar.py`/`main.cpp` de 06 (sin aplicar, a pedido); lazo cerrado (PID) para los dos motores.

## GPS

- Correcciones sugeridas por Codex (sin aplicar) en `05-gps-ros2/src/main.cpp`: (1) ventana **asimétrica** para aceptar la UTC del GPS como stamp (llegada − UTC entre −0,2 s y +1,5 s, en vez de ±2 s), para no publicar stamps futuros durante los ~10 min con segundos intercalares viejos; (2) mostrar fecha y `rollover` en `/gps/status` también cuando el stamp es `UTC GPS`.
- `05-gps-ros2 -e serial` (cable) compila pero no se probó con el GPS: se decidió usar solo WiFi.
- Soldar o fijar los cables del GPS: se desconecta al mover la placa.

## Otros

- **Falta:** opcional, un magnetómetro externo por I2C (por ejemplo QMC5883L en 0x0D) para corregir el yaw. `03-hcsr04` (idea: `sensor_msgs/Range` por micro-ROS para la altura). Lidar por UART2 (pines reservados).
