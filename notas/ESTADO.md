# Estado de pruebas-esp32 (al 2026-11-19, cierre)

Foto de hoy. Historia y CSV en `BITACORA.md`; lo que sigue en `PENDIENTES.md`; pines de la S3 en `PINES_S3.md`.

## Recolección autónoma (lo actual)

- **`27-aruco-arbol/tools/recolectar.py`:** homing del brazo → reposo → mira la marca → va a la parada (girar con el
  giróscopo, avanzar con las ruedas, girar) → **`cosechar --limones`** (ver Brazo). 3 Enter: 12 V apagados (abre la
  WROOM), 12 V encendidos (homing), ir. En la Pi: `cd ~/scripts/27-aruco-arbol; python3 tools/recolectar.py`
  (visión y agente de la base corriendo; WROOM env `brazo`). Base + cosecha juntas todavía no se probaron.
- **RODEAR (sin probar en el robot):** `recolectar.py --paradas N`; solo la base `ir_a_parada.py --rodear N [--auto S]`:
  odometría hasta 1,1 m del tronco frente a la cara siguiente (90° antihorario), mira esa marca y corrige.
- **Banderín único + BUSCAR (19/11, sin probar en el robot):** `ir_a_parada.py --banderin --buscar [--arbol D F]`:
  una sola marca (ID 0) en un palo; el eje del árbol respecto de ella en `27/banderin.json` (derecha / frente mirando
  la marca de frente, `parada_lado_deg`; **faltan los valores medidos**). Gira de a 30° hasta verla, no va si pasa J1 a
  < 45 cm del banderín o del tronco; con banderín RODEAR va solo con la odometría. `recolectar.py` todavía no lo usa.
- **Parada** (`26`): tronco a **70 cm a la izquierda de J1 y 15 cm adelante**; 4 paradas cada 90° = 100 % del arbolito.
- **Marcas ArUco** (`27`): DICT_4X4_50, 15 cm, **IDs 0–3 pegadas** (antihorario, cara a 16,5 cm del tronco:
  `--radio-maceta 165`). Se detectan de ~0,9 a ~4 m; `ver_marcas.py --tf nav --radio-maceta 165` para mirarlas.
- Cámara: agarre con la calibración del 12/11 (< 70 cm); navegación `27/resultados/tf_navegacion.json` (~3 cm a 1–2 m).
  **GPS** (UP501 o el del celular): metros de error, solo para la rúbrica, no para navegar.

## Teleop afuera (25)

- Afuera la Pi entra sola al **hotspot `POCO X6 Pro 5G` en 5 GHz** (en el lab: `sudo nmcli con up "POCO X6 Pro 5G"`).
  Página `http://<IP en el hotspot>:8000` o Tailscale con datos. `lanzar.sh` arranca todo y resetea la WROOM por EN
  (pregunta por los 12 V). Botones "Servos: suaves / RÁPIDOS" y "Pinza cerrar" = `pz 1200` (RDS3218). Necesita la
  WROOM con el env `ros` (`respaldo_wroom_20261112.bin`, sin la pose plegada).

## Base (S3, `25-teleop-pi`)

- S3 en la Pi con **`base_ros_20261117_imu`** (env `ros` de `recolector_de_frutas/firmware/base`): **la IMU se
  recupera sola** de un I2C colgado (`imu_rec` en `/base/estado`; commit `cc96d1f`). Flasheo con `--after
  watchdog-reset` y el puerto by-id. Agente: `bash ~/scripts/agente_base.sh`; watchdog 300 ms: `/cmd_vel` continuo.
- **Seguridad con motores probada (18/11):** frena solo el HC-SR04 del medio (tope lineal 50 → 20 cm; en el piso para
  a 20 cm a 0,10 y 0,20 m/s); bumpers, seta y vuelco (25°) OK, enclavados (`r` por `/base/comando`). Los de las
  esquinas solo publican (decisión del usuario). Herramientas: `25-teleop-pi/README.md` ("Seguridad de la base").
- Batería ~11,2 V (LiPo 3S sin BMS, corte 10,5 V). Odometría: recto < 1 %; giros con el giróscopo (~±1°).
- **A vigilar:** un `atasco` aislado del encoder izquierdo; cortes del USB de la S3 (3 el 18/11). Los "reinicios" de
  la Pi eran apagados del usuario (la powerbank anda bien, 19/11). Si la S3 no se conecta tras reiniciar la Pi: `pkill -f micro_ros_agent`, `esptool ... --after watchdog-reset chip-id`, agente.

## Brazo (`20-brazo-coordinado`, WROOM)

- **WROOM de la Pi: env `brazo` `brazo20_serie_20261118h.bin`**: `mv` a velocidad 1, aproximación 4 °/s, asentamiento
  0,5 s; `mvp` tolera 10°. Imágenes completas: `write-flash 0x0` (en 0x10000 queda en bucle).
- **Pose plegada** (`SOLTAR_*` en `config.h`): J1 hasta 130 solo con J2 42–51 y J3 ≥ 78 (si no, 115); ahí la franja deja
  J3 hasta 85. J3 propio 125 (PROVISORIO, sin medir más allá de 85).
- **Cosecha (`punta.py`, sesión: `source /opt/ros/jazzy/setup.bash` antes):** `cosechar N` mide un limón, entra de
  frente (2,8 cm más allá del centro, corrimientos y ajuste fino), cierra, tira, gira la muñeca, arrastra con J1 5°,
  sale (con carga a la mitad de velocidad) y va **directo a la canasta**; mide el siguiente desde ahí; confirma con la
  visión y reintenta; si falla, recupera a la canasta y sigue. ~34 s por limón; 2 de 3 en la primera prueba.
  Alcance: centro del limón a ~48–65 cm de J1, 33–54 cm del piso, mejor en el centro de la imagen. `agarrar` = uno.
- **`soltar`** (canasta J1 130 / J2 46,7 / J3 80, de paso): 12,5–14 s, ida y vuelta 28–30 s. `mvp` = mv de paso.
- **Visión → brazo:** altura `z = 23,1 + 0,718·z_visión`; ajuste fino (`ajustar`, `ajuste fin|borrar`) de 9 puntos:
  solo el costado según el lado de la imagen (`20/resultados/ajuste_agarre.json`). Log: `~/logs/punta_*.log`.
- **Abrir el puerto reinicia la WROOM:** siempre `h`. Corte por franja: `j <n> <°>` de a una, esperando que llegue.
- Joystick (17) en la Pi: `joystick.py --placa 20 [--sin-franja] --host 0.0.0.0 -p /dev/ttyUSB0` (puerto 8000).
- Homing → J1 3°, J2 6°, J3 3° (finales de carrera 32 / 27 / 33); reposo `mv 3 17 4`; repetibilidad 0,13 / 0,20 / 0,06°.
- Cinemática: ~10 mm de media; en z ~1,26–1,43 × el modelo; zona plegada solo con `USAR_ZONA` (caminos de la canasta).
- `recolector_de_frutas/firmware/brazo` sigue con franja, homing y reposo viejos: falta pasarle lo de `20`.

## Pinza y muñeca (servos)

- **Pinza RDS3218 desde el 18/11** (el MG996R vibraba), GPIO 4, calzada en 1500: abierta `pz a` = 500 µs (90°),
  **cerrada `pz c` = 1200** (agarrar, joystick y teleop). Muñeca GPIO 14 (cero 1500, ±90° en 860 / 2150 µs).

## Máquinas

- Pi 5 `r2d2@100.116.96.71` (Tailscale): las dos placas por USB (S3 `/dev/ttyACM0`, WROOM CH340 `/dev/ttyUSB0`), el GPS
  por la UART0. Se compila en la notebook y se flashea con `~/venvs/esptool/bin/esptool` desde la Pi (imágenes nuevas
  con `esptool merge-bin`). Scripts en `~/scripts/<carpeta>/tools/` (copias del repo), logs en `~/logs/`. Apagar:
  `sudo shutdown now`. Después de reiniciar: `agente_base.sh` y `vision.sh` (limones a 0,40–0,95 m) a mano.
- Binarios en `~/firmware/`: WROOM `brazo20_serie_20261118h.bin` (cargado), `respaldo_wroom_20261112.bin` (20 `ros`,
  teleop), `brazo20_serie.bin` (20 `brazo` del 12/11); S3 `base_ros_20261117_imu` (cargado), `base_ros_20261113`,
  `hcsr04_03*`.

## Pruebas terminadas

IMU MPU-6500 (01, 02), GPS UP501 (04, 05), lazo abierto 06/07 y 08, HC-SR04 (03; `d` = diagnóstico del ECHO, envs
`trig40` / `echo40`), bumpers (24), finales de carrera (18), seguridad de la base con motores (25).
