# Pendientes de pruebas-esp32 (al 2026-11-13)

<!-- Movido desde CLAUDE.md (2026-10-22). Los bloques "Pendiente GPS", "Pendiente" de 08 y "Falta" se movieron
     tal cual; el resto se copió de los "Pendiente:" que siguen dentro de cada entrada de BITACORA.md. -->

## Lo próximo (12/11 noche, recomendado en este orden)

1. ~~Agarre automático con la visión en `punta.py`~~ (13/11: `agarrar`, funciona). **Validar la simulación
   (`26`)**: robot con el eje del árbol a 70 cm a la izquierda de J1 y 15 cm adelante, limones en varios lados y
   alturas, anotar `veragarrar` en cada uno contra lo que predice `simular.py`. Después: las 4 paradas en la misión.
2. ~~Restaurar la WROOM de la Pi al env `ros`~~ (12/11, al cierre). Para `agarrar` (1) vuelve a hacer falta el serie.
3. **Al robot (ROS):** que la misión mande comandos de texto por `/brazo/comando` al firmware 20 (`brazo_ik` publica
   `/brazo/joint_cmd`, que el 20 no entiende) y pase a `brazo_ik` el modelo medido de la punta (hombro 165 mm).
4. **Precisión:** recalibrar el modelo de la punta en z con las mediciones de la cámara del 12/11 (el brazo sube y
   baja ~1,26 × lo calculado); el caño del mástil ya está fijado (12/11).
5. **Cerrar la base**: S3 con el `ros` completo, `lanzar.sh` sin RST, `/odom` derecho, IMU, HC-SR04 y bumpers con la
   seguridad real (ver "Teleop (25)" abajo).
6. Visión: probar el nodo de disparidad (se cayó con segfault el 12/11 con dos copias de `start_limon.sh`); detector
   v3 en cifras y con limones reales. **GPS: cero de navegación** afuera, al final.

## Navegación (13/11: un arbolito en el aula, marcas ArUco)

1. Imprimir `27-aruco-arbol/resultados/marcas_aruco_4x4_150mm.pdf` al 100 %, medir 150 mm, pegar en cartón alrededor
   de la base del arbolito (IDs 0–3 antihorario, centro a 20–40 cm del piso).
2. ~~Detector ArUco en la Pi~~ (13/11: `ver_marcas.py`, calibración de navegación de 6 puntos, ~3 cm a 1–2 m).
   Falta: la marca en la maceta con `--radio-maceta` (medir el radio) y la parada empujando el robot a mano.
3. ~~Base en ROS y odometría~~ (13/11: recto < 1 %, giros con el giróscopo; `ir_a_parada.py` llega;
   `recolectar.py` hace todo: homing → parada → agarrar, OK el 13/11).
   Falta: BUSCAR (girar mirando), RODEAR (4 paradas, una marca por cara), todo en un solo script / la misión;
   cortes del enlace USB de la S3 con motores; la WROOM vuelve al env `ros` para la misión en ROS.
4. `mision`: BUSCAR / ACERCARSE / PARADA / RODEAR, primero en Gazebo.
5. GPS para la rúbrica: script de arranque del driver (remapeo a `/gps/fix`), fix afuera, `robot_localization`.

## Teleop (25) y base (lo próximo)

1. ~~Encoder de la rueda izquierda~~ (11/11 noche: **una tierra desconectada**, arreglada por el usuario). Falta
   confirmar con `/odom` empujando derecho (w/v ≈ 0) y ver si con eso desaparecen los cortes de USB de la S3.
2. **S3 con el `ros` completo** (está en la notebook con `03`): `pio run -e ros -t upload --upload-port <by-id>`, a la
   Pi, RST, `lanzar.sh`, **Sensores**: IMU (si SIN_DATOS, probar con `01`), HC-SR04 del frente con la mano, bumpers;
   después las pruebas de seguridad en el aire (frente < 20 cm no avanza, bumper frena, vuelco > 25°).
3. **Probar el reinicio automático de la S3** en `lanzar.sh` (esptool + watchdog, `~/logs/reset_s3.log`; flashear con
   `--after watchdog-reset`): ¿ya no hace falta el RST a mano? Además, por qué la S3 no arrancaba y los cortes del enlace (USB) y el reinicio
   de la Pi (¿alimentación al encender los 12 V o arrancar los motores?).
4. En la página: botones de `tarea` / punta (cinemática) si hacen falta; la seta de verdad (GPIO 47).

## Base (S3)

- **`24-bumpers-s3`:** pasos A4–A5 (mover los cables sin apretar, apretar despacio) y A6 (paragolpes montado, empujar
  la base contra un obstáculo). Después, la seta (GPIO 47, NC) con el mismo programa, y llevarlo a
  `recolector_de_frutas/firmware/base`.
- **`02` con RViz** (lista de pruebas pasada al usuario): flashear `-e serial`, RST quieto, `tools/agente.sh`,
  `ros2 topic hz` (~50 Hz), bias del gyro (covarianza `1e-06`), signos, `tools/ver_imu.sh` (probar `imu.rviz`), deriva
  del yaw por minuto; anotar en Resultados de `02`.
- **`03-hcsr04`:** B4–B9 (caja a 20 / 50 / 100 cm de cada sensor con cinta y `e 30`, distancia mínima, que el frente
  no pise a las esquinas, pared a 1–2 m con `e 50`); guardar la salida en `resultados/`.

## Pinza y muñeca (23 en la WROOM; 21, 22 sueltas)

- **`23-servos-wroom`: recalzar el brazo del servo de la pinza** (~45° corrido): `p 0` (1500 µs), apagar los 12 V,
  sacar el brazo del eje, dedo en su cero, volver a calzarlo; confirmar `p 0` = cero, `p p 1000` = 45°, `p a` = 90°
  (73 mm). Después `p c` con el limón (guardar con `p C`) y anotar los valores en el código.
- `23`: verificar la muñeca `m 0`, `m a`, `m b` y anotar el resultado; probar los dos a la vez (pico de corriente en
  el buck 3); después llevarlo a `recolector_de_frutas/firmware/brazo` (pines 4 / 14 ya están).
- **`22`:** anotar el **modelo** del servo (y renombrar la carpeta a `22-servo-<modelo>-muneca`); medir su velocidad
  máxima (`w 0`, `a` → `b` = 180°, con video) y, si hace falta, ponerle tope a `w`.
- **`21`:** probar cierre a 45–90 °/s; posiciones de cierre para otras frutas (tamaños) con `C`.
- Revisar por qué la ESP32 de prueba se desconectó del USB con el servo andando (cable o caída de tensión).

## Brazo (20, lo próximo)

- ~~Eje de J1~~ (10/11 noche: reemplazado, cero, recorrido y repetibilidad bien). ~~Borde de abajo de la franja~~ y
  ~~borde de arriba re-medido~~ (10/11 noche, tablas en `config.h`, probadas en placa).
- **En orden:**
  a-1. **Tiempo de la tarea:** correr `trayectoria.py h "vel 0.75 2 0.5" "mv 3 10 5" "mv 3 59.75 65.26" "mv 12.34
     59.75 65.26"` y ver por qué "ir frente a A" tarda 30 s (esperable ~16; sospecha: J3 o J1 sin quedar firmes).
     Después fijar en `config.h` lo que sirva (`MV_FACTOR_VEL` 0,75, asentamiento 0,5 s; la aproximación de J2/J3 a
     4 °/s hace que J3 se pase y se trabe en 4°). En una tarea repetida, no volver al reposo entre limones.
  a-2. **Agarre:** grabar la telemetría en cada `tarea` (CSV en `resultados/`); cerrar a 1250 y aflojar a ~1200 fuera
     del soporte (el servo forzando consume de los 12 V); con trípode, de costado, A y B a la vista.
  a. **Probar la caja desde J1 53,5** (flasheada, sin probar): joystick con la franja, J2 70 /
     J3 60, J1 a 40, J3 a ~72, girar J1 al frente → tiene que frenar en 53; J1 55, subir J3 → frena en ~65,2 (medir la
     muñeca). Si frena bien, repetibilidad con las poses nuevas (J3 68). Si la muñeca entra sobre la tapa antes de 53,
     bajar `CAJA_J1_MIN`. (Caminos de las listas contra la caja: hecho el 10/11, ≥ 11 cm.)
  a0. **Dónde dejar el limón:** la punta no llega sobre la tapa (a 10–20 cm de altura, mínimo 43 cm de J1; el frente
     a 31). Propuesta: boca en voladizo adelante a la derecha (~(100, 430, 120)), rampa hacia atrás a un depósito sobre
     la tapa, nada bajo el nivel de la tapa (HC-SR04 y bumpers). Falta del usuario: altura y salida de los bumpers,
     posición de los HC-SR04, espacio sobre la tapa. Después, obstáculo en `cinematica.py` y prueba con `punta.py`.
  a1. ~~Muñeca ~5 cm afuera~~ (11/11: era el modelo del 05/11; recalibrado con video).
  a2. **Escala de J1:** confirmada con cinta (11/11, 0°/45°/90°): 90° reales por 83,3 del encoder (en `cinematica.py`).
     Contar los dientes de la polea grande (~78 esperado) y decidir si se cambia la escala en el firmware (y entonces
     revisar límites de J1, franja de J1 y `CAJA_J1_MIN`, que hoy están en grados del encoder).
  a3. Medir con cinta el centro del mástil (hoy (300, 240) a ojo; el croquis da ~(330, 258)). Próximo video de
     calibración: trípode nivelado a la altura del hombro (el del 11/11 quedó ~8° inclinado, se corrigió).
  b. Repetibilidad de 30 ciclos (`repetibilidad.py --ciclos 30 --calentar 3`, poses nuevas con J3 68); la de 10 dio
     0,13 / 0,20 / 0,06° (con J3 70).
  c. **J2 es la peor (RP 0,20°):** ganancias, despegue y feedforward de J2 para los doble helicoidales (tarda ~0,4 s
     en despegar, se atrasa ~2°).
  d. ~~Recalibrar la cinemática de la punta con video~~ (11/11: 19 poses, 10 mm de media). Con el modelo nuevo se
     puede pasar la regla de la caja del firmware a una con el modelo (opcional).
  e. Opcional: el borde de arriba entre 0 y 21° es "hasta donde se llegó sin nada" (L), no el borde real; el de abajo
     en J2 = 50–55 es inconsistente (55: A en 22,5 una vez, 19,8 sin nada otra). Re-medir solo si hace falta lugar.
  f. Pasar franja (las dos tablas), caja, homing (3 / 6 / 3, J3 espera a J2 > 5°, J2 rápida con cero) y reposo nuevos a
     `recolector_de_frutas/firmware/brazo` y volver a flashearlo en la WROOM.
- **`20-brazo-coordinado`, lo anterior (revisar qué sigue valiendo con los engranajes nuevos):**
  0. ~~Verificar `tools/cinematica.py` con cinta~~ (05/11, hecho: ±1 cm salvo la pose estirada (600, 150), 2 cm
     afuera; hombro 175 mm, corregido a 165 el 10/11). `mx` hecho en Python: `tools/punta.py` (`p x y z`, esquiva la caja). Falta, si hace
     falta: la punta en recta (hoy va en recta en ángulos) y llevarlo al puente de la Pi.
  0a. **Cinemática de la punta, precisión.** Sin cambios mecánicos (decisión del usuario; J3 con ~25–30° de juego en el
     antebrazo). Camino recomendado: **calibrar directamente con las cámaras estéreo del robot** (marca amarilla en la
     pinza, muchas poses con q3 − q2 ≥ −25°, llegando siempre desde abajo) y **corregir el final del agarre con la
     visión**; alternativa: más poses con la cámara de perfil (2 videos cortos de ~12) para una tabla de ~5 mm. Con el
     modelo: comando `mx x y z` (punta en mm, en recta) sobre el `mv`. Lo medido: `notas/BITACORA.md` del 05/11
     (J2 × 1,170, brazo y manivela lineales en q2/q3, pinza a (179, 29) mm de la muñeca). Decidir si la escala de J2
     (≈ 105,6 cuentas/°) se cambia en el firmware o solo en la cinemática.
  0b. ~~Franja: borde de abajo a −25°~~ (05/11, hecho: `Q2_MENOS_Q3_DESTINO` para los destinos; el camino y el
     freno siguen en −49°). Las listas de repetibilidad y la fija siguen adentro.
  0c. **Agarre con el juego:** diseñar la aproximación y la tracción para no empujar el antebrazo hacia arriba
     (horizontal y tirar hacia atrás o abajo).
  1. ~~J3 en las poses bajas~~ (05/11, hecho: cruzaba a ~3 °/s y seguía 0,13°; `tAnticipo` J3 0,045 s, (30, 20)
     en 20,13 → 20,00 ± 0,03). Sin lubricante (en seco): si algún día se lubrica, repetir la repetibilidad y revisar
     `tAnticipo` (J1 0,03 s, J2 0,055 s, J3 0,045 s).
  2. ~~Repetibilidad de 30 ciclos~~ (05/11, hecho: `repetibilidad_20261105_201209`, RP 0,18 · 0,05 · 0,12°, ±0,2° el
     97 %). Opcional: el error fijo de J1 en 40/60° y de J2 en 20/30° (~0,14°, cortas).
  3. **J2 más allá de 80°** (límite 83°), solo si hace falta alcance: cerca de 90° se pasa ~0,7° y no puede volver.
     Opción: aceptar sobrepasos < 1° de J2 ahí. J3 cerca de la traba (en (100, 97) quedó 0,18° corta): dejar aire.
  4. **Borde de la franja entre 30 y 45° de J2:** sin punto medido (recta supuesta); medir uno en ~37° con
     `17 -e explorar` si se traba ahí.
  5. La franja vieja sigue en el modo normal de `17` (lazo abierto) y en `19`: pasarles la tabla si se usan.
  6. ~~Joystick con `joystick.py --placa 20`~~ (10/11, usado para medir la franja).
  7. ~~Lista fija con el firmware final~~ (10/11: `h "mv 20 20 6" "mv 40 30 14" "mv 3 20 5"`, sin cortes).
  8. **Fase 6, en la punta:** ver el punto 0 (la repetibilidad en mm sale del mismo modelo).
  9. Opcional, si J2/J3 vuelven a pasarse (una vez en cinco capturas, J2 0,5° y 13,8 s hasta firme): `VEL_APROX` de
     J2/J3 2 → 3 °/s o dither. Más velocidad (`MV_FACTOR_VEL`): solo al final, con video.
  10. **Zona prohibida (caja y mástil):** geometría medida a ojo (bitácora del 03/11: caja 40 × 40 × 23 cm, J1 a 9 cm
     del borde derecho y 8,5 del de atrás, J2 y J3 a 16,5 cm sobre la tapa (10/11; antes 17,5), brazo 34,5 cm, antebrazo 33 cm, mástil Ø 5 cm
     a ~38 cm de J1, a ~141° de J1). **Falta calibrar el cuatro barras** (la del 03/11 fue con la manivela de 80 mm) (ángulo del brazo y del antebrazo contra q2
     y q3): repetir `trayectoria.py h "mv 3 30 20" c "mv 3 50 45" c "mv 3 74 70" c "mv 3 74 55" c "mv 3 74 78" c
     "mv 3 80 75" c --espera 6` (J2 hasta 80° desde el 04/11; revisar las poses contra la franja nueva) grabando **desde el lado de los ultrasonidos** (como `vista_frontal.jpeg`, el brazo de
     perfil); antes, una foto de prueba. Después: modelo de la punta, dibujo de la zona para que el usuario lo
     confirme y recién ahí el firmware (`mv` rechaza destinos/caminos dentro, el joystick frena antes). Con el mismo
     modelo, la repetibilidad en mm. **Límite de J1:** 118° PROVISORIO, libre hasta 115°; falta ver si hay lugar más
     allá (el usuario espera 120–130°; el mástil está a ~141°).
- **Mecánico:** fijar los finales de carrera con un segundo punto de apoyo (el golpe los gira; el 31/10 corrió el
  cero ~9° en J3 y ~30° en J2). Mientras, llegar despacio (`h`); el joystick (`17`) pega fuerte.
- **Adaptador fuente → XT60:** contacto con resistencia (con él J1/J3 no se mueven). Rehacerlo (≥ 14 AWG, soldado)
  para volver a tener el fusible y la seta con la fuente; mientras, la fuente va directo a la bornera del bus.
- **`12`: repetir el paso 3** (`escalon_posicion.py -a j1 20 --ciclos 5`, desde unos 40–60°) con el cero en el final
  de carrera.
- **`19` (banco de pruebas, opcional):** probar `-e j3`; la franja con la espera de la fase 1 ya se vio en la fase 2.
- `17-brazo-joystick`: probar en placa la rampa de bajada de J1, la barra de J3 al revés y el límite de J1 en 118°. Mejora: sostener J2/J3 con un lazo al soltar (hoy frenadas bajan despacio). Límite de J1 con el brazo estirado (choca antes con el soporte): sin medir. Después, fusionar con la tracción por joystick en `recolector_de_frutas`.
- Al robot (`recolector_de_frutas`, en una sesión de ese repo): **lo de `20`** (controlador común, homing despacio en
  el flanco, 20 kHz en J2/J3 con su feedforward, compensación de fricción de J3, freno en banda, J1 con autobloqueo,
  sostener desde el duty del peso, llegada con aproximación desde abajo para todas);
  J1 polea de 84 (84/18; factor de las ganancias en rpm ×1,286 en vez de ×1,2); cero de las tres en los finales de carrera, + = alejarse de cada uno, J1 0–118°, J2 0–97°, J3 0–113°, franja `q2 − 25° ≤ q3 ≤ q2 + 7,5°` (comparar con `GAMMA4_*` y el marco de `brazo_ik`, `cinematica.py` y el URDF, que usan otro cero); pines `PIN_FC_J2 = 27`, `PIN_FC_J3 = 33` y sus docs.
- Fotos de `12-j1-base/CERO/` y `17-brazo-joystick/LIMITES/` y el video `20-brazo-coordinado/20_20_24.mp4` sin commitear: aparecen personas (repo público); recortarlos, sacar otros o llevarlos fuera del repo.
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

- **Falta:** opcional, un magnetómetro externo por I2C (por ejemplo QMC5883L en 0x0D) para corregir el yaw. Lidar por UART2 (pines reservados).
