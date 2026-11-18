# 25-teleop-pi

Teleoperado del robot desde el celular, por **ROS 2 Jazzy + micro-ROS** en la Pi 5: **tracción** (S3 de la base) y
**brazo** (WROOM) con el control de `20-brazo-coordinado` (lazo cerrado, franja, caja, homing), **pinza y muñeca**.
Página horizontal: tracción a la izquierda, estado y PARAR en el centro, brazo a la derecha.

No es un firmware nuevo: el brazo usa `20-brazo-coordinado` en el env `ros` (`include/capa_ros.h`: los mismos
comandos que por serial llegan por `/brazo/comando`) y la base el firmware del robot
(`recolector_de_frutas/firmware/base`, env `ros`, con su seguridad local: bumpers, HC-SR04 al frente, IMU por
vuelco, seta y watchdog de 300 ms).

## Tópicos

| Tópico | Tipo | Quién |
|---|---|---|
| `/cmd_vel` | Twist | página → base (cada 100 ms mientras se toca) |
| `/base/comando` | String | `armar`, `desarmar`, `s` |
| `/brazo/comando` | String | `jv a1 a2 a3` (cada 100 ms mientras se toca), `h`, `s`, `mv 3 17 4`, `pz a/c`, `mn a/0/b` |
| `/brazo/estado` | String | `q=..;e=..;c=111;fc=000;pz=<us>;mn=<us>`, 10 Hz |
| `/brazo/respuesta`, `/base/estado`, `/base/respuesta` | String | mensajes |
| `/bumpers`, `/range/{frente,izq,der}`, `/imu/data_raw`, `/battery` | | base → página |

## Flashear (desde la Pi, con esptool; compilar en la notebook)

Apagar los 12 V antes de flashear. Los binarios se copian a la Pi con `scp`.

```bash
# Notebook
cd ~/Documentos/pruebas-esp32/20-brazo-coordinado && ~/.platformio/penv/bin/pio run -e ros
cd ~/Documentos/recolector_de_frutas/firmware/base && ~/.platformio/penv/bin/pio run -e ros
```

## Probar (en la Pi)

```bash
bash ~/scripts/25-teleop-pi/tools/lanzar.sh   # los dos agentes + teleop.py; Ctrl+C cierra los tres
# celular: http://100.116.96.71:8000 (Tailscale), horizontal. Logs de los agentes en ~/logs/agente_{brazo,base}.log
```

1. Ruedas **en el aire** la primera vez. 12 V encendidos.
2. Brazo: **Homing** (abajo y recogido, mano cerca). Hasta el homing no hay cero y `jv` no mueve.
3. Base: **Armar base** (cada conexión con el agente empieza desarmada).
4. Joystick izquierdo = tracción; derecho = J1 (←/→) y J2 (sube/baja); barra = J3; botones de pinza y muñeca.
5. **PARAR** frena las dos. Si se suelta todo o se corta la red, la base frena a los 300 ms y el brazo también.

## Seguridad de la base (en la Pi, `cd ~/scripts/25-teleop-pi/tools`)

- `python3 prueba_frenado.py [v=0.10] [s=5]`: ruedas en el aire; avisa en pantalla (pitido y cuenta) dónde poner la
  mano delante del HC-SR04 del medio y registra distancia, velocidad y `no_avanza` (`~/logs/frenado_*.csv`).
- `python3 ver_inclinacion.py`: roll y pitch en vivo (misma cuenta que el firmware) y la seguridad; VUELCO a 25°.
- `python3 prueba_odom.py tiempo|recto|giro ...`: bumpers (tiempo / giro), seta y frenado en el piso (`recto 1.5 0.10`
  con una caja a ~1 m).
- Rearmar tras seta o vuelco: `ros2 topic pub --once /base/comando std_msgs/msg/String "data: r"`.

## Estado

| Fecha | Qué | Resultado |
|---|---|---|
| 2026-11-11 | Firmware 20 env `ros`, `teleop.py`, `teleop.html`, `lanzar.sh` en la Pi | brazo OK desde el celular (homing, joystick, pinza, muñeca); base: encoder izquierdo sin cuentas, IMU real sin confirmar |
| 2026-11-18 | Seguridad de la base con motores (`base_ros_20261117_imu`) | frente: tope lineal 50 → 20 cm y parada a ≤ 20 cm (en el aire, `resultados/frenado_20261118_123100.csv`; en el piso a 0,10 m/s para a 20 cm, `imagenes/frenado_hcsr04_20261118_*.jpeg`); bumpers, seta (enclavada, `r`) y vuelco (~26° de pitch, enclavado) OK; recuperación de la IMU OK (`imu_rec=1` tras la seta) |
