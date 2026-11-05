# Estado de pruebas-esp32 (al 2026-11-05)

Foto de hoy. La historia, las mediciones completas y los nombres de los CSV están en `BITACORA.md`; lo que sigue, en
`PENDIENTES.md`; los pines de la S3, en `PINES_S3.md`.

## Brazo: lo próximo

- **`20-brazo-coordinado` (lo actual):** J1, J2 y J3 con el mismo controlador (`lib/Articulacion`), lazo no
  bloqueante a 100 Hz, 20 kHz, sin el resorte de la pinza. Firmware flasheado: el del cierre del 05/11
  (`APROX_ABAJO_MIN`, `tolPos` J3 0,20; las corridas `_163442` y `_164644` ya lo usan).
  - **Mecánica:** J3 con manivela de 95 mm; J2 y J3 con helicoidales impresos 15 → 75, en seco. **J3 tiene ~25–30° de
    juego en el antebrazo** (un diente entre el piñón y el engranaje de 75) y J2 algunos grados; sin corriente el brazo
    se queda donde está. **No se puede cambiar la mecánica** (decisión del usuario): todo por software.
  - **Franja:** `q2 − 49° ≤ q3 ≤ tabla(q2)` (0 → 3,2 · 15 → 10,8 · 30 → 27,9 · 45 → 54,2 · 60 → 72,2 · 75 → 82,7 ·
    90 → 91,9 · 105 → 101,6), margen 1° al destino y **1° a todo el camino** (`MARGEN_CAMINO`), 3° a los límites. J2
    0–83° (destinos hasta 80°), J3 0–101,5°. **Usar q3 − q2 ≥ −25°**: más abajo J3 no queda firme (punto muerto).
  - `h` (sin cero baja las tres juntas desde donde estén; arrancar con J3 ≤ J2); `zq` para bajar rápido sin cero.
    `mv`: quíntico, J2/J3 terminan juntas, J1 con su duración; si la recta sale de la franja, por un intermedio (desde
    el reposo, (15, 5)). Llegada desde abajo (J2/J3 al menos 0,3° bajo el objetivo antes del último tramo); J1 en el
    sentido en que viene. `tolLlegada` 0,15° y freno anticipado (J1 0,03 s, J2 0,055 s). Reposo `mv 3 10 5`.
  - **Repetibilidad** (`repetibilidad_20261105_115640`): RP **J1 0,17°, J2 0,05°, J3 0,18°**, las tres dentro de ±0,2°
    el 98 %. El juego no la afecta: el peso lo apoya siempre del mismo lado en las poses quietas.
  - **Cinemática (calibración con video, `trayectoria_20261105_164644`):** **J2 gira 1,170° reales por grado del
    encoder** (reductora ~120:1; confirmado con inclinómetro; el firmware sigue en grados de encoder). Brazo =
    113,0 − 1,087·q2 − 0,094·q3; manivela = 171,3 − 0,851·q3 − 0,133·q2 (lineal); antebrazo 335 mm; pinza horizontal
    a (179, 29) mm de la muñeca. Modelo de la punta con 12 poses: **~9 mm de error medio, hasta ~3 cm en los bordes.**
  - **`tools/cinematica.py`** (sin probar en placa): directa e inversa de la punta con ese modelo, en el marco del robot
    (origen en J1 a la altura de la tapa, x derecha, y adelante, z arriba), con límites, franja y choques con la caja,
    el mástil y el piso. Validación: 8,4 mm de media, 20 mm de máximo. Falta medir 3 puntos con cinta.
  - `tools/trayectoria.py`: si un `mv` se corta, vuelve solo al reposo y hace `h` (`--sin-rescate`); acepta `zq` primero.
  - Joystick `jv` (`17/tools/joystick.py --placa 20`). J1 0–118° PROVISORIO. La caja no está en el firmware.
- **`19-j2-j3-conjunto`:** banco de pruebas de J2 y J3 (una activa, la otra sostenida). Fases 1–4 del plan probadas
  en placa (franja, sostener con banda 0,2° / compensación de fricción / freno en banda, mapa de gravedad, homing);
  20 kHz. La fase 5 siguió en `20`.
- **Cero del brazo:** la pose con los tres finales de carrera apretados (foto del 30/10); + = alejarse de cada uno.
  Límites: J1 0–118° (brazo recogido), J2 0–83°, J3 0–101,5°; franja con tabla (04/11, ver arriba). Finales de carrera:
  J1 32, J2 27, J3 33. **Tienen un solo punto de apoyo y el golpe los gira** (el 31/10 el cero se corrió ~9° en J3 y
  ~30° en J2 con pulsos de 25 %): llegar despacio (`h`) y fijarlos mejor (pendiente).
- **20 kHz en J2 y J3** (decisión del 31/10, sin pitido): más duty por la misma fuerza (J2 arranca subiendo con 35 %);
  J3 sostiene con compensación de fricción 15 %. A 20 kHz la fricción sostiene el brazo en un rango ancho de duty: el
  mapa de gravedad (`19/resultados/mapa_gravedad_20261031_135400.*`) no sirve como modelo (`gm` apagado).
- **Abrir el puerto serie reinicia la WROOM** (cero perdido, brazo suelto): después de abrir el monitor, `h`.
- **Fuente:** conectada a la bornera del bus de 12 V (el adaptador al XT60 tiene un contacto malo). Así no pasa por el
  fusible ni la seta: límite de 3 A y apagar la fuente como parada.
- **`12-j1-base`:** J1 terminada (pasos 0–5); con el cero en el final de carrera falta repetir el paso 3.
- **`17-brazo-joystick`:** probado en placa (también desde el celular). Ojo: pega fuerte contra los finales de
  carrera. **`18-finales-de-carrera`:** los tres responden.
- **Robot:** `recolector_de_frutas/firmware/brazo` sigue con la polea de 90, el cero y los signos viejos de J1, los
  pines de los finales de carrera de J2/J3 al revés y 1 kHz.

## Máquinas

- Notebook y Pi 5 (`r2d2@192.168.2.219`, Tailscale `r2d2-desktop`) ordenadas igual: repos en `~/Documentos` (Pi:
  `~/firmware`, `~/scripts`, `~/logs`), workspaces en `~/ros/`, venvs en `~/venvs/`. Las dos placas están en la Pi
  (se compila en la notebook y se flashea con esptool desde la Pi); la WROOM del brazo, hoy, en la notebook
  (`/dev/ttyUSB1`; `tools/monitor.sh` la abre por id).
- Respaldo del firmware micro-ROS del brazo de la WROOM de la Pi: `~/firmware/brazo/respaldo_20261001_183959.bin`.

## Sensores en la S3

- **IMU (01, 02): OK.** MPU-6500 (sin magnetómetro: el yaw deriva). `/imu/data_raw` a 50 Hz por micro-ROS.
- **GPS UP501 (04, 05): OK por WiFi.** `/gps/fix` y `/gps/status`; `-e serial` sin probar; cables sueltos.

## Motores en lazo abierto

- **06 / 07 (36GP-555 + IBT-2, S3): OK.** K ≈ 1,6 rpm/%, zona muerta ~8–11 %, τ ≈ 60–74 ms.
- **08 (5840-31ZY + L298N, WROOM): OK.** Modo `Freno` casi lineal entre 40 y 100 %.

## Brazo en `10-control-lazo-cerrado` (historia, antes de 12–14)

- J1 terminada ahí (cascada P → PI, Kpp 2, Kp 0,549, Ki 4); J2/J3 con el 5840 (sin fin) descartadas; simulaciones e
  informe en `simulaciones/`. `tools/` de `10` lo comparten `12`–`14` y `19` (scripts, `mapa_gravedad.py`).

## Firmware del robot

- El brazo vive en `~/Documentos/recolector_de_frutas/firmware/brazo`; sus pendientes están en el `CLAUDE.md` de ese
  repo. Falta pasarle lo de hoy (ver `PENDIENTES.md`).
