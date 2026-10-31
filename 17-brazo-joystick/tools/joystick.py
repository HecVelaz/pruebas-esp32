#!/usr/bin/env python3
"""Puente entre la página del joystick (tools/joystick.html) y la WROOM por USB.

Firefox no puede abrir el puerto serie, así que este script sirve la página en http://localhost:8000 y reenvía
lo que manda el joystick ("J e1 e2 e3") a la placa. Devuelve a la página la última línea "Q ..." de la telemetría.
Al arrancar fija el cero ("Z") donde está el brazo. Abre el puerto sin tocar DTR/RTS (no resetea la placa).
"Anotar" en la página guarda una fila (hora, q1-q3, finales de carrera, límites y una nota) en
resultados/limites_<fecha>_<hora>.csv (un archivo por cada vez que se arranca el puente); "Borrar la última"
saca la última fila.

Uso:  python3 tools/joystick.py [-p /dev/ttyUSB0] [--puerto-web 8000] [--host 0.0.0.0]
Con --host 0.0.0.0 la página se abre desde el celular (http://<IP de la notebook>:8000): cualquiera en esa red
puede mover el brazo, así que solo en una red de confianza; por defecto escucha solo en la notebook.
Cerrar el monitor serie antes: el puerto no se abre dos veces.
"""

import argparse
import csv
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

PAGINA = Path(__file__).absolute().parent / "joystick.html"
RESULTADOS = Path(__file__).absolute().parent.parent / "resultados"
PUERTO_DEF = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

estado = {"q": [0.0, 0.0, 0.0], "d": [0.0, 0.0, 0.0], "e": ["?", "?", "?"], "fc": [0, 0, 0],
          "lim": [[0.0, 0.0]] * 3, "inv": [0, 0, 0], "t": 0.0, "msg": [], "anotadas": 0}
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
        if p[0] == "Q" and len(p) == 22:
            try:
                q = [float(x) for x in p[1:4]]
                d = [float(x) for x in p[4:7]]
                fc = [int(x) for x in p[10:13]]
                lim = [[float(p[13 + 2 * i]), float(p[14 + 2 * i])] for i in range(3)]
                inv = [int(x) for x in p[19:22]]
            except ValueError:
                continue
            with candado:
                estado.update(q=q, d=d, e=p[7:10], fc=fc, lim=lim, inv=inv, t=time.time())
        else:
            print("placa:", l)
            with candado:
                estado["msg"] = (estado["msg"] + [l])[-5:]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--puerto", default=PUERTO_DEF)
    ap.add_argument("--puerto-web", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1", help="0.0.0.0 para abrir la página desde otro equipo (celular)")
    args = ap.parse_args()

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.puerto, 115200, 0.2
    s.dtr = s.rts = False  # sin reset al abrir
    s.open()
    escribir = threading.Lock()

    def enviar(linea):
        with escribir:
            s.write((linea + "\n").encode())

    archivo_csv = RESULTADOS / time.strftime("limites_%Y%m%d_%H%M%S.csv")

    def anotar(nota):
        with candado:
            e = dict(estado)
        nuevo = not archivo_csv.exists()
        RESULTADOS.mkdir(exist_ok=True)
        with archivo_csv.open("a", newline="") as f:
            w = csv.writer(f)
            if nuevo:
                w.writerow(["hora", "q1", "q2", "q3", "fc1", "fc2", "fc3", "lim1", "lim2", "lim3", "inv1", "inv2",
                            "inv3", "nota"])
            w.writerow([time.strftime("%H:%M:%S"), *(f"{q:.2f}" for q in e["q"]), *e["fc"],
                        *(f"{a:.0f}/{b:.0f}" for a, b in e["lim"]), *e["inv"], nota])
        with candado:
            estado["anotadas"] += 1
        print(f"anotado en {archivo_csv.name}: q = {e['q']}  {nota}")

    def borrar_ultima():
        # Saca la última fila anotada (nunca los títulos)
        if not archivo_csv.exists():
            return
        lineas = archivo_csv.read_text().splitlines(keepends=True)
        if len(lineas) <= 1:
            return
        archivo_csv.write_text("".join(lineas[:-1]))
        with candado:
            estado["anotadas"] = max(0, estado["anotadas"] - 1)
        print(f"borrada de {archivo_csv.name}: {lineas[-1].strip()}")

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
            elif self.path == "/lim":
                try:
                    n, mn, mx = json.loads(cuerpo)
                    n, mn, mx = int(n), float(mn), float(mx)
                except (ValueError, TypeError):
                    self.send_error(400)
                    return
                if n not in (1, 2, 3) or not mn < mx:
                    self.send_error(400)
                    return
                enviar(f"L {n} {mn:.0f} {mx:.0f}")
            elif self.path == "/inv":
                try:
                    n = int(json.loads(cuerpo))
                except (ValueError, TypeError):
                    self.send_error(400)
                    return
                if n not in (1, 2, 3):
                    self.send_error(400)
                    return
                enviar(f"I {n}")
            elif self.path == "/anotar":
                anotar(cuerpo.strip().replace("\n", " ")[:200])
            elif self.path == "/borrar":
                borrar_ultima()
            else:
                self.send_error(404)
                return
            with candado:
                r = dict(estado, edad=time.time() - estado["t"])
            self.responder(json.dumps(r))

    srv = ThreadingHTTPServer((args.host, args.puerto_web), Manejador)
    print(f"Puerto {args.puerto} abierto. Abrir http://localhost:{args.puerto_web} en el navegador. Ctrl+C para salir.")
    if args.host != "127.0.0.1":
        print(f"Abierto a la red ({args.host}): desde el celular, http://<IP de la notebook>:{args.puerto_web}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        enviar("S")
        s.close()


if __name__ == "__main__":
    main()
