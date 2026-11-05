#!/usr/bin/env python3
"""Puente entre la página del joystick (tools/joystick.html) y la WROOM por USB.

Firefox no puede abrir el puerto serie, así que este script sirve la página en http://localhost:8000 y reenvía
lo que manda el joystick ("J e1 e2 e3") a la placa. Devuelve a la página la última línea "Q ..." de la telemetría.
Al arrancar fija el cero ("Z") donde está el brazo. Abre el puerto sin tocar DTR/RTS (no resetea la placa).
"Anotar" en la página guarda una fila (hora, q1-q3, finales de carrera, límites y una nota) en
resultados/limites_<fecha>_<hora>.csv (un archivo por cada vez que se arranca el puente); "Borrar la última"
saca la última fila.

Con --placa 20 maneja el firmware de 20-brazo-coordinado (2026-11-03): manda "jv a1 a2 a3" (velocidad de cada una en
lazo cerrado; al soltar se sostienen), el botón del cero hace el homing ("h") y frenar es "s". Los ángulos salen de la
telemetría ("t 1") y los finales de carrera y el cero, de "c" cada segundo. Lo anotado va a
20-brazo-coordinado/resultados/limites_<fecha>_<hora>.csv (el cero es el de 20). No fija el cero al arrancar: abrir
el puerto puede reiniciar la WROOM, así que primero "Homing" en la página, con el brazo abajo y recogido.

Con --zq "q1 q2 q3" (solo --placa 20) manda "zq" al arrancar: el brazo quedó lejos de los finales de carrera sin cero
(abrir el puerto reinicia la WROOM; el brazo no se mueve sin corriente) y se lo baja con el joystick; después Homing.

Uso:  python3 tools/joystick.py [--placa 17|20] [--zq "q1 q2 q3"] [-p /dev/ttyUSB0] [--puerto-web 8000] [--host 0.0.0.0]
Con --host 0.0.0.0 la página se abre desde el celular (http://<IP de la notebook>:8000): cualquiera en esa red
puede mover el brazo, así que solo en una red de confianza; por defecto escucha solo en la notebook.
Cerrar el monitor serie antes: el puerto no se abre dos veces.
"""

import argparse
import csv
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import serial

PAGINA = Path(__file__).absolute().parent / "joystick.html"
RESULTADOS = Path(__file__).absolute().parent.parent / "resultados"
RESULTADOS_20 = Path(__file__).absolute().parent.parent.parent / "20-brazo-coordinado" / "resultados"
# 20: estado de la telemetría (columna "estado" de cada una) y límites de su config.h (con 3° de margen en jv)
ESTADOS_20 = {0: "SUELTA", 1: "YENDO", 2: "LLEGÓ", 3: "FIRME", 4: "FRENADA", 5: "MOV", 6: "HOMING"}
LIMITES_20 = [[0.0, 118.0], [0.0, 105.0], [0.0, 101.5]]  # 2026-11-04: J2/J3 con la manivela de 95 mm
RE_C_20 = re.compile(r"^(J[123])\s.*\| final de carrera (apretado|suelto) \| (con cero|SIN CERO)")
PUERTO_DEF = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

estado = {"q": [0.0, 0.0, 0.0], "d": [0.0, 0.0, 0.0], "e": ["?", "?", "?"], "fc": [0, 0, 0],
          "lim": [[0.0, 0.0]] * 3, "inv": [0, 0, 0], "t": 0.0, "msg": [], "anotadas": 0, "placa": 17,
          "cero": [0, 0, 0]}
candado = threading.Lock()


def leer_20(l):
    """Una línea del firmware de 20: telemetría T (q, duty, estado) o el estado de "c" (final de carrera, cero).
    Devuelve False si es para mostrar como mensaje."""
    if l.startswith("T,"):
        p = l.split(",")
        if len(p) != 18:
            return True
        try:
            q = [float(p[3 + 5 * i]) for i in range(3)]
            d = [float(p[5 + 5 * i]) for i in range(3)]
            e = [ESTADOS_20.get(int(p[7 + 5 * i]), "?") for i in range(3)]
        except ValueError:
            return True
        with candado:
            estado.update(q=q, d=d, e=e, t=time.time())
        return True
    m = RE_C_20.match(l)
    if m:
        i = int(m.group(1)[1]) - 1
        with candado:
            estado["fc"][i] = 1 if m.group(2) == "apretado" else 0
            estado["cero"][i] = 1 if m.group(3) == "con cero" else 0
        return True
    return l.startswith(("franja:", "TELE_CFG", ">> Telemetría"))  # (leer() ya sacó los espacios)


def leer(s):
    while True:
        try:
            l = s.readline().decode(errors="replace").strip()
        except serial.SerialException as ex:
            print("Se perdió el puerto:", ex)
            return
        if not l:
            continue
        if estado["placa"] == 20:
            if leer_20(l):
                continue
            print("placa:", l)
            with candado:
                estado["msg"] = (estado["msg"] + [l])[-5:]
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
    ap.add_argument("--placa", type=int, choices=(17, 20), default=17,
                    help="firmware en la WROOM: 17 (lazo abierto, def.) o 20 (jv en lazo cerrado, homing)")
    ap.add_argument("--zq", metavar='"Q1 Q2 Q3"',
                    help='con --placa 20: el brazo está en estos ángulos (cero estimado, "zq") para bajarlo con el '
                         'joystick sin cero; después Homing')
    args = ap.parse_args()
    if args.zq is not None:
        try:
            zq = [float(x) for x in args.zq.split()]
        except ValueError:
            zq = []
        if args.placa != 20 or len(zq) != 3:
            ap.error('--zq va con --placa 20 y tres ángulos: --zq "27.3 74.3 41.3"')
    p20 = args.placa == 20
    estado["placa"] = args.placa
    if p20:
        estado["lim"] = LIMITES_20
        estado["fc"] = [0, 0, 0]

    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.puerto, 115200, 0.2
    s.dtr = s.rts = False  # sin reset al abrir
    s.open()
    escribir = threading.Lock()

    def enviar(linea):
        with escribir:
            s.write((linea + "\n").encode())

    dir_res = RESULTADOS_20 if p20 else RESULTADOS
    archivo_csv = dir_res / time.strftime("limites_%Y%m%d_%H%M%S.csv")

    def anotar(nota):
        with candado:
            e = dict(estado)
        if p20 and not all(e["cero"]):
            nota = "SIN CERO (hacer el homing) " + nota
        nuevo = not archivo_csv.exists()
        dir_res.mkdir(exist_ok=True)
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
    ultimo_jv = {"t": 0.0, "linea": ""}
    if p20:
        # Abrir el puerto puede reiniciar la WROOM: esperar el saludo, prender la telemetría y pedir "c" cada segundo
        time.sleep(2.0)
        enviar("t 1")
        if args.zq is not None:
            enviar("zq %.2f %.2f %.2f" % tuple(zq))

        def pedir_estado():
            while True:
                enviar("c")
                time.sleep(1.0)

        threading.Thread(target=pedir_estado, daemon=True).start()
    else:
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
                if p20:
                    # "jv" a 10 Hz como mucho (o si cambió): alcanza para el hombre muerto de 300 ms
                    linea = "jv %.2f %.2f %.2f" % tuple(ejes)
                    ahora = time.time()
                    if linea != ultimo_jv["linea"] or ahora - ultimo_jv["t"] >= 0.1:
                        enviar(linea)
                        ultimo_jv.update(t=ahora, linea=linea)
                else:
                    enviar("J %.3f %.3f %.3f" % tuple(ejes))
            elif self.path == "/cero":
                enviar("h" if p20 else "Z")
            elif self.path == "/freno":
                enviar("s" if p20 else "S")
            elif p20 and self.path in ("/lim", "/inv"):
                self.send_error(400)
                return
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
        if p20:
            enviar("s")
            enviar("t 0")
        else:
            enviar("S")
        s.close()


if __name__ == "__main__":
    main()
