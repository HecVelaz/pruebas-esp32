#!/usr/bin/env python3
"""Prueba de la odometría de la base (en la Pi, con el agente de la base corriendo).

Manda /cmd_vel a 20 Hz (el watchdog de la base con micro-ROS corta a los 300 ms sin comandos) y lee /odom al mismo
tiempo. Para cuando la ODOMETRÍA dice que llegó (recto / giro) o al vencer el tiempo (tiempo), frena y muestra lo pedido
contra lo medido. En el piso, comparar el final con la cinta: eso es el error de la odometría.

Uso:
    python3 prueba_odom.py recto <m> [v=0.10]        avanzar <m> metros (negativo: retroceder)
    python3 prueba_odom.py giro <grados> [w=0.5]     girar en el lugar (+ = izquierda, antihorario visto desde arriba);
                                                     para con el GIRÓSCOPO (13/11: las ruedas se pasan ~2° cada 90°
                                                     porque patinan; el giróscopo acertó: 88,7° contra ~88–89° de cinta)
    python3 prueba_odom.py tiempo <v> <w> <s>        velocidad fija durante <s> segundos (ruedas en el aire)
Arma la base si está desarmada. Ctrl+C frena. Tope de seguridad: 2 m, 400° y 20 s por prueba.
"""
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String

HZ = 20.0
# Frenado: cerca del objetivo la velocidad baja proporcional a lo que falta (13/11: a 0,5 rad/s se pasaba 6° y en el
# recto ~2 cm, con la rampa de frenado de la base). Mínimos para no quedar en la zona muerta de los motores.
K_W, W_MIN = 1.5, 0.12     # rad/s por rad que falta, rad/s
K_V, V_MIN = 1.0, 0.03     # m/s por m que falta, m/s
TOPE_M, TOPE_DEG, TOPE_S = 2.0, 400.0, 20.0


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    a = sys.argv[1:]
    if not a or a[0] not in ("recto", "giro", "tiempo"):
        print(__doc__)
        return
    modo = a[0]
    if modo == "recto":
        obj, v, w = float(a[1]), (float(a[2]) if len(a) > 2 else 0.10), 0.0
        v = math.copysign(abs(v), obj)
        if abs(obj) > TOPE_M:
            raise SystemExit(f"tope {TOPE_M} m")
    elif modo == "giro":
        obj, v, w = float(a[1]), 0.0, (float(a[2]) if len(a) > 2 else 0.5)
        w = math.copysign(abs(w), obj)
        if abs(obj) > TOPE_DEG:
            raise SystemExit(f"tope {TOPE_DEG}°")
    else:
        v, w, obj = float(a[1]), float(a[2]), float(a[3])
        if obj > TOPE_S:
            raise SystemExit(f"tope {TOPE_S} s")

    rclpy.init()
    n = rclpy.create_node("prueba_odom")
    pub = n.create_publisher(Twist, "/cmd_vel", 10)
    cmd = n.create_publisher(String, "/base/comando", 10)
    st = {"odom": None, "estado": "", "muestras": [], "giro_imu": 0.0, "t_imu": None, "integrar": False}
    n.create_subscription(Odometry, "/odom", lambda m: st.__setitem__("odom", m), 10)
    n.create_subscription(String, "/base/estado", lambda m: st.__setitem__("estado", m.data), 10)

    def al_imu(m):   # integra la velocidad angular en z del giróscopo (ya calibrado por la base)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st["integrar"] and st["t_imu"] is not None and 0 < t - st["t_imu"] < 0.5:
            st["giro_imu"] += m.angular_velocity.z * (t - st["t_imu"])
        st["t_imu"] = t

    n.create_subscription(Imu, "/imu/data_raw", al_imu, qos_profile_sensor_data)   # la IMU publica best effort

    def girar(seg):
        fin = time.time() + seg
        while time.time() < fin:
            rclpy.spin_once(n, timeout_sec=0.02)

    def mandar(vv, ww):
        t = Twist()
        t.linear.x, t.angular.z = vv, ww
        pub.publish(t)

    fin = time.time() + 8.0   # conectarse y recibir /odom y /base/estado (tras reiniciar tarda más)
    while time.time() < fin and (st["odom"] is None or not st["estado"]):
        girar(0.2)
    if st["odom"] is None:
        raise SystemExit("sin /odom: ¿agente de la base corriendo?")
    if not st["estado"].startswith("ARMADA"):
        print(f"estado: {st['estado'][:60]} -> armar")
        cmd.publish(String(data="armar"))
        girar(1.5)
        if not st["estado"].startswith("ARMADA"):
            raise SystemExit(f"no se armó: {st['estado']}")
    o0 = st["odom"].pose.pose
    x0, y0, th0 = o0.position.x, o0.position.y, yaw(o0.orientation)
    giro_acum, th_ant = 0.0, th0
    d_act = 0.0
    st["integrar"] = True
    t0 = time.time()
    print(f"{modo}: v {v:+.2f} m/s, w {w:+.2f} rad/s, objetivo {obj:g}" + {"recto": " m", "giro": "°", "tiempo": " s"}[modo])
    motivo = "tope de tiempo"
    try:
        while time.time() - t0 < TOPE_S:
            vv, ww = v, w
            if modo == "recto":
                falta = abs(obj) - d_act
                vv = math.copysign(min(abs(v), max(V_MIN, K_V * falta)), v)
            elif modo == "giro":
                falta = math.radians(abs(obj)) - abs(st["giro_imu"])
                ww = math.copysign(min(abs(w), max(W_MIN, K_W * falta)), w)
            mandar(vv, ww)
            girar(1.0 / HZ)
            o = st["odom"]
            p = o.pose.pose
            th = yaw(p.orientation)
            giro_acum += math.atan2(math.sin(th - th_ant), math.cos(th - th_ant))
            th_ant = th
            d = math.hypot(p.position.x - x0, p.position.y - y0)
            d_act = d
            st["muestras"].append((time.time() - t0, o.twist.twist.linear.x, o.twist.twist.angular.z))
            if modo == "recto" and d >= abs(obj):
                motivo = "llegó (odometría)"
                break
            if modo == "giro" and abs(math.degrees(st["giro_imu"])) >= abs(obj):
                motivo = "llegó (giróscopo)"
                break
            if modo == "tiempo" and time.time() - t0 >= obj:
                motivo = "tiempo cumplido"
                break
            if not st["estado"].startswith("ARMADA"):
                motivo = f"la base se desarmó: {st['estado'][:70]}"
                break
    except KeyboardInterrupt:
        motivo = "Ctrl+C"
    finally:
        for _ in range(5):
            mandar(0.0, 0.0)
            girar(0.05)
    girar(1.0)   # que termine de frenar
    st["integrar"] = False
    p = st["odom"].pose.pose
    d = math.hypot(p.position.x - x0, p.position.y - y0)
    th = yaw(p.orientation)
    giro_acum += math.atan2(math.sin(th - th_ant), math.cos(th - th_ant))
    dur = time.time() - t0
    m = [s for s in st["muestras"] if s[0] > 1.0]   # sin la rampa del arranque
    vm = sum(s[1] for s in m) / len(m) if m else float("nan")
    wm = sum(s[2] for s in m) / len(m) if m else float("nan")
    print(f"fin: {motivo} | {dur:.1f} s")
    print(f"  odometría: avanzó {d * 100:.1f} cm, giró {math.degrees(giro_acum):+.1f}° | giróscopo: "
          f"{math.degrees(st['giro_imu']):+.1f}°")
    print(f"  velocidad medida (después de 1 s): v {vm:+.3f} m/s (pedida {v:+.3f}), w {wm:+.3f} rad/s (pedida {w:+.3f})")
    print(f"  estado: {st['estado'][:90]}")
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
