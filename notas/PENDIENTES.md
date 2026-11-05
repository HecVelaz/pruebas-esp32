# Pendientes de pruebas-esp32 (al 2026-11-05)

<!-- Movido desde CLAUDE.md (2026-10-22). Los bloques "Pendiente GPS", "Pendiente" de 08 y "Falta" se movieron
     tal cual; el resto se copió de los "Pendiente:" que siguen dentro de cada entrada de BITACORA.md. -->

## Brazo (20, lo próximo)

- **`20-brazo-coordinado`, en orden:**
  0. **Cinemática de la punta (lo próximo).** Sin cambios mecánicos (decisión del usuario; J3 con ~25–30° de juego en el
     antebrazo). Camino recomendado: **calibrar directamente con las cámaras estéreo del robot** (marca amarilla en la
     pinza, muchas poses con q3 − q2 ≥ −25°, llegando siempre desde abajo) y **corregir el final del agarre con la
     visión**; alternativa: más poses con la cámara de perfil (2 videos cortos de ~12) para una tabla de ~5 mm. Con el
     modelo: comando `mx x y z` (punta en mm, en recta) sobre el `mv`. Lo medido: `notas/BITACORA.md` del 05/11
     (J2 × 1,170, brazo y manivela lineales en q2/q3, pinza a (179, 29) mm de la muñeca). Decidir si la escala de J2
     (≈ 105,6 cuentas/°) se cambia en el firmware o solo en la cinemática.
  0b. **Franja:** llevar el borde de abajo de −49° a −25° (`Q2_MENOS_Q3_MAX`; punto muerto de J3) y revisar los
     destinos de las listas (`20 20 14`, repetibilidad) contra el nuevo borde.
  0c. **Agarre con el juego:** diseñar la aproximación y la tracción para no empujar el antebrazo hacia arriba
     (horizontal y tirar hacia atrás o abajo).
  1. **J3 en las poses bajas** (`repetibilidad_20261105_115640`: (20, 14) en ~14,10 y (30, 20) en ~20,14, antes
     ~13,97 / ~19,93): corrida corta con telemetría; si es real, freno anticipado como J2. Sin lubricante (en seco):
     si algún día se lubrica, repetir la repetibilidad y revisar `tAnticipo` (J1 0,03 s, J2 0,055 s).
  2. **Repetibilidad de 30 ciclos** (`--ciclos 30 --calentar 3`, ~45 min) con el firmware final.
  3. **J2 más allá de 80°** (límite 83°), solo si hace falta alcance: cerca de 90° se pasa ~0,7° y no puede volver.
     Opción: aceptar sobrepasos < 1° de J2 ahí. J3 cerca de la traba (en (100, 97) quedó 0,18° corta): dejar aire.
  4. **Borde de la franja entre 30 y 45° de J2:** sin punto medido (recta supuesta); medir uno en ~37° con
     `17 -e explorar` si se traba ahí.
  5. La franja vieja sigue en el modo normal de `17` (lazo abierto) y en `19`: pasarles la tabla si se usan.
  6. **Joystick:** los tres arreglos de la revisión de Codex hechos; falta el uso normal con `joystick.py --placa 20`
     (con los límites nuevos).
  7. **Lista fija con el firmware final** (`h "mv 20 20 14" "mv 40 30 20" "mv 20 20 14" "mv 3 10 5"`; la del 03/11
     usaba `20 20 24`, hoy fuera de la franja).
  8. **Fase 6, en la punta:** ver el punto 0 (la repetibilidad en mm sale del mismo modelo).
  9. Opcional, si J2/J3 vuelven a pasarse (una vez en cinco capturas, J2 0,5° y 13,8 s hasta firme): `VEL_APROX` de
     J2/J3 2 → 3 °/s o dither. Más velocidad (`MV_FACTOR_VEL`): solo al final, con video.
  10. **Zona prohibida (caja y mástil):** geometría medida a ojo (bitácora del 03/11: caja 40 × 40 × 23 cm, J1 a 9 cm
     del borde derecho y 8,5 del de atrás, J2 a 17,5 cm sobre la tapa, brazo 34,5 cm, antebrazo 33 cm, mástil Ø 5 cm
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

- **Falta:** opcional, un magnetómetro externo por I2C (por ejemplo QMC5883L en 0x0D) para corregir el yaw. `03-hcsr04` (idea: `sensor_msgs/Range` por micro-ROS para la altura). Lidar por UART2 (pines reservados).
