"""Movimientos de la base por ROS (en la Pi, con el agente de la base): recto con las ruedas, giro con el giróscopo.

Lo usan prueba_odom.py y 27-aruco-arbol/tools/ir_a_parada.py. Medido el 13/11 en el piso: recto < 1 % de error con las
ruedas; en los giros las ruedas se pasan ~2° cada 90° (patinan) y el giróscopo acierta (~±1°). Manda /cmd_vel a 20 Hz
(la base con micro-ROS frena a los 300 ms sin comandos) y frena proporcional cerca del objetivo.
"""
import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
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


def ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class Base:
    """Suscripciones y publicadores de la base sobre un nodo rclpy ya creado."""

    def __init__(self, nodo):
        import rclpy
        self.rclpy = rclpy
        self.n = nodo
        self.pub = nodo.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd = nodo.create_publisher(String, "/base/comando", 10)
        self.odom = None
        self.estado = ""
        self.giro_imu = 0.0
        self.t_imu = None
        self.integrar = False
        self.muestras = []
        nodo.create_subscription(Odometry, "/odom", self._al_odom, 10)
        nodo.create_subscription(String, "/base/estado", lambda m: setattr(self, "estado", m.data), 10)
        nodo.create_subscription(Imu, "/imu/data_raw", self._al_imu, qos_profile_sensor_data)  # best effort

    def _al_odom(self, m):
        self.odom = m

    def _al_imu(self, m):   # integra la velocidad angular en z (la base ya calibró el giróscopo)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.integrar and self.t_imu is not None and 0 < t - self.t_imu < 0.5:
            self.giro_imu += m.angular_velocity.z * (t - self.t_imu)
        self.t_imu = t

    def girar_ros(self, seg):
        fin = time.time() + seg
        while time.time() < fin:
            self.rclpy.spin_once(self.n, timeout_sec=0.02)

    def mandar(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = v, w
        self.pub.publish(t)

    def frenar(self):
        for _ in range(5):
            self.mandar(0.0, 0.0)
            self.girar_ros(0.05)

    def esperar_datos(self, seg=8.0):
        fin = time.time() + seg   # tras reiniciar /odom tarda ~5 s en aparecer
        while time.time() < fin and (self.odom is None or not self.estado or self.t_imu is None):
            self.girar_ros(0.2)
        if self.odom is None:
            raise RuntimeError("sin /odom: ¿agente de la base corriendo?")
        if self.t_imu is None:
            raise RuntimeError("sin /imu/data_raw")

    def armar(self):
        if self.estado.startswith("ARMADA"):
            return
        print(f"estado: {self.estado[:60]} -> armar")
        self.cmd.publish(String(data="armar"))
        self.girar_ros(1.5)
        if not self.estado.startswith("ARMADA"):
            raise RuntimeError(f"no se armó: {self.estado}")

    def mover(self, modo, obj, v, w):
        """modo "recto" (obj m, para con las ruedas), "giro" (obj °, para con el giróscopo) o "tiempo" (obj s).
        Devuelve dict con lo medido."""
        if modo == "recto" and abs(obj) > TOPE_M or modo == "giro" and abs(obj) > TOPE_DEG or \
                modo == "tiempo" and obj > TOPE_S:
            raise ValueError("pasa el tope de seguridad")
        if modo == "recto":
            v, w = math.copysign(abs(v), obj), 0.0
        elif modo == "giro":
            v, w = 0.0, math.copysign(abs(w), obj)
        o0 = self.odom.pose.pose
        x0, y0, th_ant = o0.position.x, o0.position.y, yaw(o0.orientation)
        giro_odo, d = 0.0, 0.0
        self.giro_imu, self.integrar, self.muestras = 0.0, True, []
        t0 = time.time()
        motivo = "tope de tiempo"
        try:
            while time.time() - t0 < TOPE_S:
                vv, ww = v, w
                if modo == "recto":
                    vv = math.copysign(min(abs(v), max(V_MIN, K_V * (abs(obj) - d))), v)
                elif modo == "giro":
                    falta = math.radians(abs(obj)) - abs(self.giro_imu)
                    ww = math.copysign(min(abs(w), max(W_MIN, K_W * falta)), w)
                self.mandar(vv, ww)
                self.girar_ros(1.0 / HZ)
                p = self.odom.pose.pose
                th = yaw(p.orientation)
                giro_odo += ang(th - th_ant)
                th_ant = th
                d = math.hypot(p.position.x - x0, p.position.y - y0)
                self.muestras.append((time.time() - t0, self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z))
                if modo == "recto" and d >= abs(obj):
                    motivo = "llegó (odometría)"
                    break
                if modo == "giro" and abs(math.degrees(self.giro_imu)) >= abs(obj):
                    motivo = "llegó (giróscopo)"
                    break
                if modo == "tiempo" and time.time() - t0 >= obj:
                    motivo = "tiempo cumplido"
                    break
                if not self.estado.startswith("ARMADA"):
                    motivo = f"la base se desarmó: {self.estado[:70]}"
                    break
        except KeyboardInterrupt:
            motivo = "Ctrl+C"
            raise
        finally:
            self.frenar()
            self.girar_ros(1.0)   # que termine de frenar
            self.integrar = False
            p = self.odom.pose.pose
            th = yaw(p.orientation)
            giro_odo += ang(th - th_ant)
            d = math.hypot(p.position.x - x0, p.position.y - y0)
        m = [s for s in self.muestras if s[0] > 1.0]   # sin la rampa del arranque
        return {"motivo": motivo, "dur": time.time() - t0, "d": d, "giro_odo": math.degrees(giro_odo),
                "giro_imu": math.degrees(self.giro_imu), "v": v, "w": w,
                "vm": sum(s[1] for s in m) / len(m) if m else float("nan"),
                "wm": sum(s[2] for s in m) / len(m) if m else float("nan"),
                "ok": motivo.startswith("llegó") or motivo == "tiempo cumplido"}


def informe(r):
    print(f"fin: {r['motivo']} | {r['dur']:.1f} s")
    print(f"  odometría: avanzó {r['d'] * 100:.1f} cm, giró {r['giro_odo']:+.1f}° | giróscopo: {r['giro_imu']:+.1f}°")
    print(f"  velocidad medida (después de 1 s): v {r['vm']:+.3f} m/s (pedida {r['v']:+.3f}), w {r['wm']:+.3f} rad/s "
          f"(pedida {r['w']:+.3f})")
