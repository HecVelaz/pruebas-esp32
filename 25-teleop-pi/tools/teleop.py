#!/usr/bin/env python3
"""25: teleoperado de la tracción y del brazo (con pinza y muñeca) desde el celular, por ROS 2 + micro-ROS.

Corre en la Pi, con los dos agentes micro-ROS (S3 de la base y WROOM del brazo). Sirve la página horizontal
tools/teleop.html en http://<IP de la Pi>:8000 y traduce lo que manda a tópicos:

  base   /cmd_vel (Twist) mientras se toca el joystick de la izquierda (cada 100 ms; si deja de llegar, la S3 frena
         sola por su watchdog de 300 ms); /base/comando: armar, desarmar, s
  brazo  /brazo/comando (String, firmware 20 en el env `ros`): "jv a1 a2 a3" mientras se toca el joystick de la
         derecha (J1 y J2) o la barra (J3), cada 100 ms (el firmware frena solo a los 300 ms sin jv), y los botones:
         h (homing), s (parar), mv 3 17 4 (reposo), pz a / pz c (pinza), mn a / mn 0 / mn b (muñeca)

Muestra /brazo/estado y /brazo/respuesta, y de la base /base/estado, /base/respuesta, /bumpers, /range/*,
/imu/data_raw (roll y pitch del acelerómetro) y /battery.

Uso (en la Pi):
    source /opt/ros/jazzy/setup.bash
    python3 tools/teleop.py [--puerto 8000] [--v-max 0.25] [--w-max 1.0]
Cualquiera que llegue a la página puede mover el robot: solo en una red de confianza (Tailscale).
"""
import argparse
import json
import math
import os
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
import rclpy.signals
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, Imu, Range
from std_msgs.msg import String, UInt8

PAGINA = Path(__file__).absolute().parent / "teleop.html"
COMANDOS = {
    "brazo": {"h", "s", "x", "mv 3 17 4", "pz a", "pz c", "pz 0", "mn a", "mn 0", "mn b", "c"},
    "base": {"armar", "desarmar", "s", "r", "sens"},   # r: rearmar tras la seta o un vuelco; sens: sensores
}
JV_MAX = 1.0

estado = {"brazo": "", "brazo_resp": [], "base": "", "base_resp": [], "bumpers": None, "rango": {},
          "imu": None, "bateria": None, "t_brazo": 0.0, "t_base": 0.0}
candado = threading.Lock()


def agregar(lista, texto, n=8):
    for l in texto.splitlines():
        if l.strip():
            lista.append(l.strip())
    del lista[:-n]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", type=int, default=8000)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--v-max", type=float, default=0.25, help="m/s con el joystick a fondo y la velocidad al 100 %%")
    ap.add_argument("--w-max", type=float, default=1.0, help="rad/s ídem")
    args = ap.parse_args()

    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    # rclpy atrapaba SIGTERM y el servidor web seguía vivo con el puerto ocupado: SIGTERM/SIGINT cierran todo
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    nodo = rclpy.create_node("teleop_25")
    pub_vel = nodo.create_publisher(Twist, "/cmd_vel", 10)
    pub_cmd = {a: nodo.create_publisher(String, f"/{a}/comando", 10) for a in ("base", "brazo")}

    def cb_texto(clave, es_resp):
        def cb(m):
            with candado:
                if es_resp:
                    agregar(estado[clave], m.data, 14 if clave == "base_resp" else 8)
                else:
                    estado[clave] = m.data
                    estado["t_" + clave] = time.time()
        return cb

    nodo.create_subscription(String, "/brazo/estado", cb_texto("brazo", False), 10)
    nodo.create_subscription(String, "/brazo/respuesta", cb_texto("brazo_resp", True), 10)
    nodo.create_subscription(String, "/base/estado", cb_texto("base", False), 10)
    nodo.create_subscription(String, "/base/respuesta", cb_texto("base_resp", True), 10)

    def cb_bumpers(m):
        with candado:
            estado["bumpers"] = int(m.data)

    def cb_rango(nombre):
        def cb(m):
            with candado:
                # inf = "libre" (responde, nada al alcance); NaN = "sin eco" (no responde: la base no avanza)
                estado["rango"][nombre] = "libre" if math.isinf(m.range) else ("sin eco" if math.isnan(m.range) else round(m.range, 3))
        return cb

    def cb_imu(m):
        a = m.linear_acceleration
        with candado:
            estado["imu"] = (round(math.degrees(math.atan2(a.y, a.z)), 1),
                             round(math.degrees(math.atan2(-a.x, math.hypot(a.y, a.z))), 1))

    def cb_bat(m):
        with candado:
            estado["bateria"] = round(m.voltage, 2)

    qos_sensor = rclpy.qos.qos_profile_sensor_data
    nodo.create_subscription(UInt8, "/bumpers", cb_bumpers, 10)
    for n in ("frente", "izq", "der"):
        nodo.create_subscription(Range, f"/range/{n}", cb_rango(n), qos_sensor)
    nodo.create_subscription(Imu, "/imu/data_raw", cb_imu, qos_sensor)
    nodo.create_subscription(BatteryState, "/battery", cb_bat, 10)

    def publicar(dest, texto):
        pub_cmd[dest].publish(String(data=texto))

    class Manejador(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"  # keep-alive: con una conexión TCP nueva por pedido (HTTP/1.0) se apilaban 20 s

        def setup(self):
            super().setup()
            # Encabezados y cuerpo salen en dos escrituras: sin esto, Nagle retiene la segunda ~40 ms por pedido
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        def responder(self, cuerpo, tipo="application/json"):
            b = cuerpo.encode() if isinstance(cuerpo, str) else cuerpo
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.responder(PAGINA.read_bytes(), "text/html; charset=utf-8")
            elif self.path.startswith("/estado"):
                with candado:
                    e = dict(estado)
                    e["brazo_resp"], e["base_resp"] = list(estado["brazo_resp"]), list(estado["base_resp"])
                    e["rango"] = dict(estado["rango"])
                ahora = time.time()
                e["brazo_vivo"] = ahora - e.pop("t_brazo") < 1.0
                e["base_vivo"] = ahora - e.pop("t_base") < 2.0
                self.responder(json.dumps(e))
            else:
                self.send_error(404)

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
                d = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, json.JSONDecodeError):
                self.send_error(400)
                return
            if self.path == "/base":
                f = min(max(float(d.get("f", 0.4)), 0.0), 1.0)
                t = Twist()
                t.linear.x = max(-1.0, min(1.0, float(d.get("v", 0)))) * args.v_max * f
                t.angular.z = max(-1.0, min(1.0, float(d.get("w", 0)))) * args.w_max * f
                pub_vel.publish(t)
            elif self.path == "/jv":
                a = [max(-JV_MAX, min(JV_MAX, float(x))) for x in d.get("a", [0, 0, 0])[:3]]
                publicar("brazo", "jv " + " ".join(f"{x:.2f}" for x in a))
            elif self.path == "/cmd":
                dest, c = d.get("dest"), d.get("c", "")
                if dest not in COMANDOS or c not in COMANDOS[dest]:
                    self.send_error(400, "comando no permitido")
                    return
                if dest == "base" and c == "s":
                    pub_vel.publish(Twist())
                publicar(dest, c)
            else:
                self.send_error(404)
                return
            self.responder("{}")

        def log_message(self, *a):
            pass

    hilo = threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True)
    hilo.start()
    servidor = ThreadingHTTPServer((args.host, args.puerto), Manejador)
    print(f"teleop 25 en http://{args.host}:{args.puerto} (v máx {args.v_max} m/s, w máx {args.w_max} rad/s). Ctrl+C sale.")
    try:
        servidor.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        pub_vel.publish(Twist())
        publicar("brazo", "s")
        time.sleep(0.2)
        servidor.server_close()
        os._exit(0)  # sin esperar al hilo de rclpy.spin (con rclpy.shutdown() a veces quedaba colgado)


if __name__ == "__main__":
    main()
