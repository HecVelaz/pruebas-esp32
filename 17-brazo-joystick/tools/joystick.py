#!/usr/bin/env python3
"""Puente entre la página del joystick (tools/joystick.html) y la WROOM por USB.

Firefox no puede abrir el puerto serie, así que este script sirve la página en http://localhost:8000 y reenvía
lo que manda el joystick ("J e1 e2 e3") a la placa. Devuelve a la página la última línea "Q ..." de la telemetría.
Al arrancar fija el cero ("Z") donde está el brazo. Abre el puerto sin tocar DTR/RTS (no resetea la placa).

Uso:  python3 tools/joystick.py [-p /dev/ttyUSB0] [--puerto-web 8000]
Cerrar el monitor serie antes: el puerto no se abre dos veces.
"""

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

PAGINA = Path(__file__).absolute().parent / "joystick.html"
PUERTO_DEF = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

estado = {"q": [0.0, 0.0, 0.0], "d": [0.0, 0.0, 0.0], "e": ["?", "?", "?"], "t": 0.0, "msg": []}
candado = threading.Lock()


def leer(s):
    while True:
        try:
            l = s.readline().decode(errors="replace").strip()
        except serial.SerialException as ex:
            print("Se perdió el puerto:", ex)
            return
        if not l:
            continue
        p = l.split()
        if p[0] == "Q" and len(p) == 10:
            try:
                with candado:
                    estado["q"] = [float(x) for x in p[1:4]]
                    estado["d"] = [float(x) for x in p[4:7]]
                    estado["e"] = p[7:10]
                    estado["t"] = time.time()
            except ValueError:
                pass
        else:
            print("placa:", l)
            with candado:
                estado["msg"] = (estado["msg"] + [l])[-5:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--puerto", default=PUERTO_DEF)
    ap.add_argument("--puerto-web", type=int, default=8000)
    args = ap.parse_args()

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.puerto, 115200, 0.2
    s.dtr = s.rts = False  # sin reset al abrir
    s.open()
    escribir = threading.Lock()

    def enviar(linea):
        with escribir:
            s.write((linea + "\n").encode())

    threading.Thread(target=leer, args=(s,), daemon=True).start()
    time.sleep(0.3)
    enviar("S")
    enviar("Z")

    class Manejador(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def responder(self, cuerpo, tipo="application/json"):
            datos = cuerpo.encode() if isinstance(cuerpo, str) else cuerpo
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(datos)

        def do_GET(self):
            if self.path in ("/", "/joystick.html"):
                self.responder(PAGINA.read_bytes(), "text/html; charset=utf-8")
            else:
                self.send_error(404)

        def do_POST(self):
            cuerpo = self.rfile.read(int(self.headers.get("Content-Length", 0))).decode()
            if self.path == "/cmd":
                try:
                    ejes = [max(-1.0, min(1.0, float(x))) for x in json.loads(cuerpo)]
                except (ValueError, TypeError):
                    self.send_error(400)
                    return
                if len(ejes) != 3:
                    self.send_error(400)
                    return
                enviar("J %.3f %.3f %.3f" % tuple(ejes))
            elif self.path == "/cero":
                enviar("Z")
            elif self.path == "/freno":
                enviar("S")
            else:
                self.send_error(404)
                return
            with candado:
                r = dict(estado, edad=time.time() - estado["t"])
            self.responder(json.dumps(r))

    srv = ThreadingHTTPServer(("127.0.0.1", args.puerto_web), Manejador)
    print(f"Puerto {args.puerto} abierto. Abrir http://localhost:{args.puerto_web} en el navegador. Ctrl+C para salir.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        enviar("S")
        s.close()


if __name__ == "__main__":
    main()
