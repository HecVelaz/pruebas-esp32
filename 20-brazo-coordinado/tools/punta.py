#!/usr/bin/env python3
"""Mover la punta (centro de la pinza) de 20 a un punto en mm, con el modelo medido de tools/cinematica.py.

Abre el puerto como un monitor: lo que se escribe va a la placa (h, c, mv, s, x, ...) salvo estos comandos propios:
    p <x> <y> <z>     llevar la punta a (x, y, z) mm: inversa, revisa el destino y el camino y manda el/los mv
    ver <x> <y> <z>   lo mismo sin mover nada (solo el plan)
    donde             pide "c" y dice dónde está la punta
    salir             (o Ctrl+D)
Marco del robot (el de cinematica.py): origen en el eje de J1 a la altura de la tapa; x a la derecha, y adelante
(hacia los ultrasonidos), z arriba. Ej.: "p 0 500 300" = 50 cm adelante y 30 cm sobre la tapa (J1 en 90°).

Antes de mandar un "p" revisa, además de lo de cinematica.py (límites, franja, q3 - q2 >= -25°, choques del destino):
  - el camino de J2/J3 como lo hace el firmware (recta hasta el punto previo de la aproximación, 2° abajo, y el
    último tramo) dentro de la franja con MARGEN_CAMINO, para que el firmware no elija otro camino por su cuenta;
  - choques con la caja, el mástil y el piso en todo el camino, para CUALQUIER combinación de J1 con J2/J3 (J1
    tiene su propia duración en el mv: no se sabe en qué ángulo de J1 está cada punto de J2/J3).
Si el camino recto no sirve, busca un punto intermedio (J1 quieta en el primer tramo, como el firmware) con los dos
tramos libres y manda dos mv. El firmware no conoce la caja: este script es lo único que la esquiva.
Si el firmware igual dice que el camino sale de la franja (eligió su propio intermedio) o algo no llega en
--timeout s, manda "s" (las tres se sostienen donde están). Ctrl+C durante un movimiento: "s".

Abrir el puerto reinicia la WROOM (cero perdido): primero "h". Orden de la fuente: 12 V apagados, brazo abajo y
recogido, abrir esto, 12 V encendidos, "h", los "p" ... al final "mv 3 10 5" y apagar los 12 V.

Uso:
    python3 tools/punta.py                                   # sesión con la placa
    python3 tools/punta.py --simular --desde 3 10 5 0 520 80  # solo el plan, sin la placa
"""
import argparse
import math
import queue
import re
import sys
import threading
import time

import numpy as np

import cinematica as k

PUERTO = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
# Del firmware (include/config.h)
Q2_MENOS_Q3_MAX = 49.0     # borde de abajo de la franja en el firmware (el camino puede pasar por ahí)
MARGEN_CAMINO = 1.2        # ° (el firmware usa 1,0: un poco más para que acepte la recta que elegimos acá)
APROX = 2.0                # APROX_GRADOS de J2 y J3
TOLPOS = 0.2               # si se mueve menos que esto no hace aproximación
LIM_MIN = 3.0              # límite de abajo con MARGEN_LIMITE
PASO_Q2Q3 = 1.0            # ° entre muestras del camino de J2/J3
PASO_Q1 = 5.0              # ° entre los ángulos de J1 que se combinan con cada muestra
PASO_INTERMEDIO = 5.0      # grilla de puntos intermedios (como el firmware)


# ------------------------------------------------------------------------------------------------- plan (sin placa)

def exceso(q2, q3):
    """Cuánto sale (°) de la franja del firmware: <= 0 adentro."""
    return max(q3 - k.q3_max_franja(q2), (q2 - Q2_MENOS_Q3_MAX) - q3)


def tramos(s, o):
    """Los tramos de J2/J3 de un mv de s a o como en el firmware: recta al punto previo de la aproximación y el final."""
    pr = tuple(o[i] - APROX if abs(o[i] - s[i]) > TOLPOS and o[i] - APROX >= LIM_MIN else o[i] for i in (1, 2))
    return [((s[1], s[2]), pr), (pr, (o[1], o[2]))]


def muestras(s, o):
    for a, b in tramos(s, o):
        n = 1 + int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / PASO_Q2Q3)
        for t in np.linspace(0, 1, n + 1):
            yield a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])


def franja_camino(s, o):
    tolerado = max(-MARGEN_CAMINO, exceso(s[1], s[2]))
    return all(exceso(q2, q3) <= tolerado for q2, q3 in muestras(s, o))


def choques_camino(s, o):
    if abs(o[0] - s[0]) < 1e-6:
        q1s = [s[0]]
    else:
        q1s = np.linspace(s[0], o[0], 2 + int(abs(o[0] - s[0]) / PASO_Q1))
    m = set()
    for q2, q3 in muestras(s, o):
        for q1 in q1s:
            m.update(k.choques(k.directa(q1, q2, q3)[1]))
    return sorted(m)


def revisar_camino(s, o):
    m = [] if franja_camino(s, o) else ["el camino de J2/J3 sale de la franja"]
    return m + choques_camino(s, o)


def intermedio_valido(q2, q3):
    return (k.Q2_LIM[0] <= q2 <= k.Q2_LIM[1] and k.Q3_LIM[0] <= q3 <= k.Q3_LIM[1]
            and q3 <= k.q3_max_franja(q2) - k.MARGEN_FRANJA and q3 - q2 >= k.Q3_MENOS_Q2_MIN)


def planear(s, x, y, z):
    """Desde los ángulos s, a la punta (x, y, z). Devuelve (lista de mv, avisos) o (None, motivos)."""
    q, mot = k.inversa(x, y, z)
    if q is None or mot:
        return None, mot
    o = tuple(round(v, 2) for v in q)
    avisos = []
    if k.fuera_de_lo_medido(o[1], o[2]):
        avisos.append("el destino está fuera de la zona medida (el modelo extrapola)")
    directo = revisar_camino(s, o)
    if not directo:
        return [o], avisos
    # Punto intermedio P: J1 quieta en el primer tramo; los dos tramos dentro de la franja y sin choques; el más corto.
    # J1 dentro de sus límites: después del homing queda en ~2,8° y el mv solo acepta desde 3° (2026-11-05)
    p1 = min(max(s[0], k.Q1_LIM[0]), k.Q1_LIM[1])
    cands = []
    g2 = np.arange(0.0, k.Q2_LIM[1] + 0.01, PASO_INTERMEDIO)
    g3 = np.arange(0.0, k.Q3_LIM[1] + 0.01, PASO_INTERMEDIO)
    for p2 in list(g2) + [o[1], s[1]]:
        for p3 in list(g3) + [s[2], o[2]]:
            if not intermedio_valido(p2, p3):
                continue
            largo = max(abs(p2 - s[1]), abs(p3 - s[2])) + max(abs(o[1] - p2), abs(o[2] - p3))
            cands.append((largo, (p1, float(p2), float(p3))))
    for _, p in sorted(cands):
        if franja_camino(s, p) and franja_camino(p, o) and not choques_camino(s, p) and not choques_camino(p, o):
            avisos.append(f"el camino recto no sirve ({'; '.join(directo)}): pasa por J2 {p[1]:.1f}° J3 {p[2]:.1f}°")
            return [tuple(round(v, 2) for v in p), o], avisos
    return None, [f"camino recto: {'; '.join(directo)}", "no encontré un punto intermedio libre"]


def mostrar_plan(s, x, y, z):
    plan, msgs = planear(s, x, y, z)
    if plan is None:
        print("NO:", "; ".join(msgs))
        return None
    for m in msgs:
        print("  ojo:", m)
    for q in plan:
        p, _ = k.directa(*q)
        print(f"  mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}   (punta x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm)")
    return plan


# ------------------------------------------------------------------------------------------------------ con la placa

class Placa:
    def __init__(self, puerto):
        import serial
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = puerto, 115200, 0.1
        self.s.dtr = self.s.rts = False  # como tools/monitor.sh (igual puede reiniciarse al abrir)
        self.s.open()
        self.lineas = queue.Queue()
        self.vivo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        while self.vivo:
            try:
                l = self.s.readline().decode(errors="replace").strip()
            except Exception as e:  # puerto desconectado
                print(f"\n[puerto: {e}]")
                self.vivo = False
                return
            if not l or l.startswith("T,"):
                continue
            print(l)
            self.lineas.put(l)

    def vaciar(self):
        while not self.lineas.empty():
            self.lineas.get_nowait()

    def mandar(self, c):
        self.s.write(f"{c}\n".encode())

    def esperar(self, prefijos, segundos):
        """La primera línea que empieza con alguno de prefijos, o None."""
        t_lim = time.time() + segundos
        while time.time() < t_lim:
            try:
                l = self.lineas.get(timeout=0.1)
            except queue.Empty:
                continue
            if l.startswith(prefijos):
                return l
        return None

    def angulos(self):
        """(q1, q2, q3) pidiendo "c"; None si alguna no tiene cero."""
        self.vaciar()
        self.mandar("c")
        q, cero = {}, {}
        t_lim = time.time() + 2.0
        while time.time() < t_lim and len(q) < 3:
            try:
                l = self.lineas.get(timeout=0.1)
            except queue.Empty:
                continue
            m = re.match(r"(J[123])\s+([+-]?\d+(?:\.\d+)?)°.*\| (con cero|SIN CERO)", l)
            if m:
                q[m.group(1)] = float(m.group(2))
                cero[m.group(1)] = m.group(3) == "con cero"
        if len(q) < 3:
            print("[no pude leer los ángulos con \"c\"]")
            return None
        if not all(cero.values()):
            print("[alguna articulación SIN CERO: primero \"h\"]")
            return None
        return q["J1"], q["J2"], q["J3"]


FIN_MV = ("LLEGO", "MV_ERROR", "Fuera de", "J2 y J3 fuera", "Uso:", "Hardware sin", ">> El camino recto sale")
RECHAZO = ("Fuera de", "J2 y J3 fuera", "Uso:", "Hardware sin")


def ejecutar(placa, plan, timeout):
    for q in plan:
        c = f"mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}"
        placa.vaciar()
        print(f"<< {c}")
        placa.mandar(c)
        try:
            l = placa.esperar(FIN_MV + tuple(f"{j} sin cero" for j in ("J1", "J2", "J3")), timeout)
        except KeyboardInterrupt:
            placa.mandar("s")
            print("[Ctrl+C: \"s\", las tres se sostienen donde están]")
            return False
        if l is None:
            placa.mandar("s")
            print(f"[{timeout:.0f} s sin LLEGO: \"s\"]")
            return False
        if l.startswith(">> El camino recto sale"):
            placa.mandar("s")
            print("[el firmware eligió otro camino (no revisado contra la caja): \"s\". Probar otro punto]")
            return False
        if not l.startswith("LLEGO"):
            if l.startswith(RECHAZO) or "sin cero" in l:
                print("[la placa rechazó el mv]")
            return False
    return True


def sesion(args):
    placa = Placa(args.puerto)
    time.sleep(2.0)  # saludo y ayuda si se reinició al abrir
    print("[punta.py: \"p x y z\", \"ver x y z\", \"donde\", \"salir\"; lo demás va a la placa. Primero \"h\"]")
    try:
        while placa.vivo:
            try:
                l = input().strip()
            except EOFError:
                break
            if not l:
                continue
            w = l.split()
            if w[0] == "salir":
                break
            if w[0] in ("p", "ver"):
                try:
                    x, y, z = (float(v) for v in w[1:4])
                    if len(w) != 4:
                        raise ValueError
                except ValueError:
                    print(f"Uso: {w[0]} <x> <y> <z>  (mm)")
                    continue
                s = placa.angulos()
                if s is None:
                    continue
                plan = mostrar_plan(s, x, y, z)
                if plan and w[0] == "p":
                    ejecutar(placa, plan, args.timeout)
            elif w[0] == "donde":
                s = placa.angulos()
                if s is not None:
                    p, _ = k.directa(*s)
                    print(f"  punta: x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm  (r {math.hypot(p[0], p[1]):.0f} mm)"
                          + ("  [fuera de la zona medida]" if k.fuera_de_lo_medido(s[1], s[2]) else ""))
            else:
                placa.mandar(l)
    finally:
        placa.vivo = False
        placa.s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("punto", type=float, nargs="*", help="con --simular: x y z (mm)")
    ap.add_argument("--simular", action="store_true", help="solo el plan, sin la placa (con --desde y x y z)")
    ap.add_argument("--desde", type=float, nargs=3, default=(3.0, 10.0, 5.0), metavar=("Q1", "Q2", "Q3"),
                    help="ángulos de partida para --simular (def. el reposo, 3 10 5)")
    ap.add_argument("--puerto", default=PUERTO)
    ap.add_argument("--timeout", type=float, default=90.0, help="s máximos por mv antes de mandar \"s\" (def. 90)")
    args = ap.parse_args()
    if args.simular:
        if len(args.punto) != 3:
            ap.error("--simular necesita x y z")
        mostrar_plan(tuple(args.desde), *args.punto)
        return
    if args.punto:
        ap.error("x y z solo con --simular (con la placa, \"p x y z\" en la sesión)")
    sesion(args)


if __name__ == "__main__":
    main()
