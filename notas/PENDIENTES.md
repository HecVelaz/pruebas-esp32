# Pendientes de pruebas-esp32 (al 2026-10-26)

<!-- Movido desde CLAUDE.md (2026-10-22). Los bloques "Pendiente GPS", "Pendiente" de 08 y "Falta" se movieron
     tal cual; el resto se copió de los "Pendiente:" que siguen dentro de cada entrada de BITACORA.md. -->

## Brazo (12–14, 17, lo próximo)

- J1 (`12`): resolver la escala (contar los dientes de las poleas de la base y del motor, o prueba de 180°: `m -42` → `m 126` → `m 0`); si es 84/18, `RELACION_TRANSMISION = 84.0f / 18.0f` en `12` y en `17`. Después `dg` y los pasos 2 (PI de velocidad), 3 (posición, `a` en vez de `m`), 4 (rango) y 5 (limón).
- Probar en placa `13-j2-hombro` y `14-j3-codo` a 1 kHz: pasos A, 0 signos, 1 cuentas y cuatro barras, 2 barrido de carga, 3 PI, 4 posición, 5 carga, 6 al robot (plan en cada README). Todo lo de J2 y J3 es PROVISORIO (signos, límites ±15°, duty máx 60 %, ganancias, ff 17/8 %, khp 3 / khi 8).
- `17-brazo-joystick`: flashear (12 V apagados) y probar en placa con la escala baja; anotar signos de J2/J3 (estado `SIGNO`) y si los `dutyMin`/`dutyMax` sirven. Mejora: sostener J2/J3 con el PI de posición al soltar el joystick (hoy frenadas bajan despacio). Después, fusionar con la tracción por joystick en `recolector_de_frutas`.
- `15`/`16`: anotar en sus README los resultados de J3 con el C3 (IBT-2 y L298N), si se hicieron.
- Hombro (`10`, 2026-10-13): medir con `bc` el duty según el ángulo y hacer compensación de gravedad en el hombro; la USB se desconectó una vez con un pulso fuerte (revisar masas).
- J1 (`10`): video para la Entrega 6; opcional tolerancia 0,1° / 0,3°; pasar ganancias a `recolector_de_frutas/firmware/brazo/include/brazo_config.h`.
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
