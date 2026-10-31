#!/usr/bin/env python3
"""Fase 5c: telemetría de 20 ("t", 50 Hz) durante una serie de comandos; la guarda en un CSV y la grafica.

Manda "t 1", cada comando de la lista esperando su final ("mv" hasta LLEGO o MV_ERROR, "h" hasta HOMING o
HOMING_ERROR, los demás --espera s), sigue capturando --cola s (el sostener después de llegar) y manda "t 0".
Guarda resultados/trayectoria_<fecha>_<hora>.csv y su .png: para cada articulación, referencia contra medido, el error
(con la tolerancia y la banda del integrador; ahí se ve el tambaleo del final) y el duty; abajo, el camino (q2, q3)
sobre la franja del cuatro barras. Imprime para cada mv cuánto tardó en quedar firme y cuántas correcciones hizo cada
una al final.

Antes: cerrar el monitor (el puerto no se abre dos veces), brazo abajo y recogido, 12 V encendidos. Abrir el puerto
puede reiniciar la WROOM (pierde el cero): si alguna está sin cero, el primer comando tiene que ser "h".

Uso:
    python3 tools/trayectoria.py h "mv 20 20 24" "mv 40 30 20" "mv 20 20 24" "mv 3 3 3"
    python3 tools/trayectoria.py "mv 20 20 24" --cola 5
    python3 tools/trayectoria.py --archivo resultados/trayectoria_....csv     # volver a graficar, sin la placa
Opciones: --cola (s de captura después del último comando, def. 3), --espera (s para comandos sin final, def. 2),
--puerto, --sin-ventana, --todo (graficar también el homing; por defecto arranca en el primer mv).
Ctrl+C manda "s" (las tres se sostienen donde están) y guarda lo capturado (archivo ..._cortado).
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

DIR_RESULTADOS = Path(__file__).absolute().parent.parent / "resultados"
PUERTO = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
# Franja del cuatro barras (include/config.h): q2 - Q2_MENOS_Q3_MAX <= q3 <= q2 + Q3_MENOS_Q2_MAX
Q3_MENOS_Q2_MAX = 7.5
Q2_MENOS_Q3_MAX = 25.0
NOMBRES = ("J1", "J2", "J3")
COLUMNAS = ["t_ms", "etapa"] + [f"{c}{i}" for i in (1, 2, 3) for c in ("q", "r", "u", "w", "s")]
ESTADOS = {0: "suelta", 1: "yendo", 2: "llegó", 3: "firme", 4: "frenada", 5: "siguiendo", 6: "homing"}
ETAPAS = {0: "", 1: "recorrido", 2: "aproximación", 3: "llegada", 4: "asentamiento", 9: "homing"}


# ---------------------------------------------------------------------------------------------- captura en la placa

def abrir(puerto):
    import serial
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = puerto, 115200, 0.1
    s.dtr = s.rts = False  # como tools/monitor.sh: sin tocar DTR/RTS (igual puede reiniciarse al abrir)
    s.open()
    return s


class Captura:
    def __init__(self, s):
        self.s = s
        self.filas = []        # muestras T
        self.eventos = []      # (t_ms de la última muestra, tipo, texto)
        self.cfg = None        # TELE_CFG: tolPos y bandaInt de las tres

    def t_actual(self):
        return self.filas[-1][0] if self.filas else 0

    def leer(self, segundos, fin=None):
        """Lee durante `segundos` o hasta una línea que empiece con alguno de `fin`; devuelve esa línea (o None)."""
        t_lim = time.time() + segundos
        while time.time() < t_lim:
            l = self.s.readline().decode(errors="replace").strip()
            if not l:
                continue
            if l.startswith("T,"):
                p = l.split(",")
                if len(p) != 3 + 15:  # "T", ms, etapa y 5 valores por articulación
                    continue
                try:
                    self.filas.append([int(p[1]), int(p[2])] + [float(x) for x in p[3:]])
                except ValueError:
                    pass
                continue
            print(l)
            self.eventos.append((self.t_actual(), "msg", l))
            if l.startswith("TELE_CFG,"):
                self.cfg = [float(x) for x in l.split(",")[1:]]
            if fin and l.startswith(fin):
                return l
        return None

    def mandar(self, c):
        print(f"<< {c}")
        self.eventos.append((self.t_actual(), "cmd", c))
        self.s.write(f"{c}\n".encode())


def correr_en_placa(args):
    s = abrir(args.puerto)
    cap = Captura(s)
    cortado = None
    try:
        cap.leer(2.0)  # si se reinició al abrir: el saludo y la ayuda
        s.reset_input_buffer()
        cap.mandar("c")
        cap.leer(1.0)
        sin_cero = any("SIN CERO" in e[2] for e in cap.eventos if e[1] == "msg")
        if sin_cero and (not args.comandos or args.comandos[0] != "h"):
            print('\nAlguna está SIN CERO (la placa se reinició al abrir el puerto): empezar la lista con "h".')
            return
        cap.mandar("t 1")
        cap.leer(1.0, ("TELE_CFG",))
        for c in args.comandos:
            cap.mandar(c)
            if c.startswith("mv "):
                r = cap.leer(90.0, ("LLEGO,", "MV_ERROR,", "Uso:", "Fuera de", "J2 y J3 fuera", "sin cero"))
            elif c == "h":
                r = cap.leer(240.0, ("HOMING,", "HOMING_ERROR,", "Homing: J2"))
            else:
                cap.leer(args.espera)
                continue
            if r is None or not (r.startswith("LLEGO,") or r.startswith("HOMING,")):
                cortado = r or f'"{c}" no terminó a tiempo'
                print(f"\nSe corta la serie: {cortado}")
                break
        cap.leer(args.cola)
    except KeyboardInterrupt:
        cortado = "Ctrl+C"
        print('\nCtrl+C: "s" (las tres se sostienen donde están).')
        s.write(b"s\n")
    finally:
        try:
            s.write(b"t 0\n")
            cap.leer(1.0, ("TELE_FIN",))
        finally:
            s.close()
    if not cap.filas:
        print("No llegó telemetría: ¿firmware de la 5c flasheado?")
        return
    DIR_RESULTADOS.mkdir(exist_ok=True)
    archivo = DIR_RESULTADOS / f"trayectoria_{datetime.now():%Y%m%d_%H%M%S}{'_cortado' if cortado else ''}.csv"
    with open(archivo, "w", newline="") as f:
        f.write(f"# 20-brazo-coordinado, telemetria de \"t\" (50 Hz). Comandos: {' | '.join(args.comandos)}\n")
        if cortado:
            f.write(f"# cortado: {cortado}\n")
        for t, tipo, texto in cap.eventos:
            f.write(f"# {tipo},{t},{texto}\n")
        w = csv.writer(f)
        w.writerow(COLUMNAS)
        for fila in cap.filas:
            w.writerow([fila[0], fila[1]] + [f"{x:g}" for x in fila[2:]])
    print(f"CSV: {archivo.relative_to(DIR_RESULTADOS.parent)} ({len(cap.filas)} muestras)")
    perdidas = [e[2] for e in cap.eventos if e[2].startswith("TELE_FIN,")]
    if perdidas and perdidas[-1] != "TELE_FIN,0":
        print(f"OJO: se perdieron muestras ({perdidas[-1]})")
    return archivo


# ---------------------------------------------------------------------------------------------------- análisis

def cargar(archivo):
    import numpy as np
    eventos, filas = [], []
    with open(archivo) as f:
        for l in f:
            l = l.rstrip("\n")
            if l.startswith("# cmd,") or l.startswith("# msg,"):
                tipo, t, texto = l[2:].split(",", 2)
                eventos.append((int(t), tipo, texto))
            elif l and not l.startswith("#") and not l.startswith("t_ms"):
                filas.append([float(x) for x in l.split(",")])
    d = np.array(filas)
    cfg = None
    for _, tipo, texto in eventos:
        if texto.startswith("TELE_CFG,"):
            cfg = [float(x) for x in texto.split(",")[1:]]
    return d, eventos, cfg


def columnas(d, i):
    """q, r, u, w, s de la articulación i (0, 1, 2)."""
    b = 2 + 5 * i
    return d[:, b], d[:, b + 1], d[:, b + 2], d[:, b + 3], d[:, b + 4]


def analizar_mv(d, eventos):
    """Para cada mv que llegó: tiempo de la llegada (etapa 3) hasta firmes, y en la etapa 3+4 cuántas veces cada una
    se movió (|w| > 1 °/s, ya cerca del objetivo: las correcciones) y el error máximo."""
    import numpy as np
    t = d[:, 0]
    et = d[:, 1]
    # Cada mv con la línea de final que le sigue EN ORDEN (el siguiente mv se manda en el mismo t que ese LLEGO)
    pares = []
    for tt, tipo, c in eventos:
        if tipo == "cmd" and c.startswith("mv "):
            pares.append([tt, c, None])
        elif tipo == "msg" and c.startswith(("LLEGO,", "MV_ERROR,")) and pares and pares[-1][2] is None:
            pares[-1][2] = (tt, c)
    print("\nmv                 recorrido  llegada→firmes | correcciones al final (J1 J2 J3) | |error| máx al final")
    for t0, c, fin in pares:
        t1 = fin[0] if fin else t[-1]
        tramo = (t > t0) & (t <= t1)
        if not tramo.any():
            continue
        llegada = tramo & (et >= 3)
        if not llegada.any():
            print(f"{c:18s} sin etapa de llegada ({fin[1] if fin else 'sin final'})")
            continue
        t_lleg = t[llegada][0]
        firmes = tramo & (et == 4)
        t_firme = t[firmes][0] if firmes.any() else float("nan")
        corr, emax = [], []
        for i in range(3):
            q, r, u, w, s = columnas(d, i)
            mov = np.abs(w[llegada]) > 1.0
            corr.append(int(np.sum(mov[1:] & ~mov[:-1]) + (1 if mov[0] else 0)))
            emax.append(float(np.max(np.abs(r[llegada] - q[llegada]))))
        estado = "" if fin and fin[1].startswith("LLEGO") else f"  ({fin[1] if fin else 'sin final'})"
        print(f"{c:18s} {(t_lleg - t0) / 1000:7.1f} s  {(t_firme - t_lleg) / 1000:9.1f} s     | "
              f"{corr[0]:4d} {corr[1]:4d} {corr[2]:4d}                   | "
              f"{emax[0]:.2f} {emax[1]:.2f} {emax[2]:.2f}°{estado}")


def graficar(archivo, sin_ventana, todo=False):
    import matplotlib
    if sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    d, eventos, cfg = cargar(archivo)
    if d.size == 0:
        print("CSV sin muestras.")
        return
    analizar_mv(d, eventos)
    # El homing es largo y tapa los mv: el gráfico arranca 1 s antes del primer mv (--todo: completo)
    t_mv = [tt for tt, tipo, c in eventos if tipo == "cmd" and c.startswith("mv ")]
    if t_mv and not todo:
        d = d[d[:, 0] >= t_mv[0] - 1000]
    t0 = d[0, 0]
    ts = (d[:, 0] - t0) / 1000

    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(4, 3, height_ratios=[1, 1, 1, 1.25], hspace=0.45, wspace=0.22)
    cmds = [((tt - t0) / 1000, c) for tt, tipo, c in eventos
            if tipo == "cmd" and c not in ("c", "t 1") and tt >= t0]
    for i in range(3):
        q, r, u, w, s = columnas(d, i)
        ax_q = fig.add_subplot(gs[i, 0])
        ax_e = fig.add_subplot(gs[i, 1], sharex=ax_q)
        ax_u = fig.add_subplot(gs[i, 2], sharex=ax_q)
        ax_q.plot(ts, r, "--", color="tab:gray", lw=1, label="referencia")
        ax_q.plot(ts, q, color="tab:blue", lw=1.2, label="medido")
        ax_q.set_ylabel(f"{NOMBRES[i]} (°)")
        e = r - q
        ax_e.plot(ts, e, color="tab:red", lw=1)
        if cfg:
            tol, banda = cfg[i], cfg[3 + i]
            for v, estilo in ((tol, "--"), (-tol, "--"), (banda, ":"), (-banda, ":")):
                ax_e.axhline(v, color="k", ls=estilo, lw=0.7)
        ax_e.set_ylim(-1.5, 1.5)
        ax_e.set_ylabel("ref − medido (°)")
        ax_u.plot(ts, u, color="tab:purple", lw=1)
        ax_u.set_ylabel("duty (%)")
        for ax in (ax_q, ax_e, ax_u):
            ax.grid(alpha=0.3)
            # sombreado: firme (verde) y frenada (azul)
            for val, color in ((3, "tab:green"), (4, "tab:cyan")):
                m = s == val
                ax.fill_between(ts, 0, 1, where=m, color=color, alpha=0.12, transform=ax.get_xaxis_transform(),
                                lw=0)
            for tc, c in cmds:
                ax.axvline(tc, color="tab:orange", lw=0.8, alpha=0.7)
        if i == 0:
            ax_q.set_title("referencia y medido")
            ax_e.set_title("error (zoom ±1,5°; -- tolPos, ··· bandaInt)")
            ax_u.set_title("duty")
            ax_q.legend(loc="best", fontsize=8)
            for tc, c in cmds:
                ax_q.annotate(c, (tc, 1.02), xycoords=("data", "axes fraction"), fontsize=7, rotation=20,
                              color="tab:orange")
        if i == 2:
            for ax in (ax_q, ax_e, ax_u):
                ax.set_xlabel("t (s)")

    # Camino (q2, q3) sobre la franja
    ax_c = fig.add_subplot(gs[3, 0:2])
    q2, r2 = columnas(d, 1)[:2]
    q3, r3 = columnas(d, 2)[:2]
    lim = [min(q2.min(), r2.min()) - 5, max(q2.max(), r2.max()) + 5]
    x = np.array(lim)
    ax_c.fill_between(x, x - Q2_MENOS_Q3_MAX, x + Q3_MENOS_Q2_MAX, color="tab:green", alpha=0.08,
                      label="franja del cuatro barras")
    ax_c.plot(x, x - Q2_MENOS_Q3_MAX, color="tab:green", lw=0.8)
    ax_c.plot(x, x + Q3_MENOS_Q2_MAX, color="tab:green", lw=0.8)
    ax_c.plot(r2, r3, "--", color="tab:gray", lw=1, label="referencia")
    ax_c.plot(q2, q3, color="tab:blue", lw=1, label="medido")
    ax_c.set_xlabel("q2 (°)")
    ax_c.set_ylabel("q3 (°)")
    ax_c.set_title("camino en (q2, q3)")
    ax_c.set_xlim(lim)
    ax_c.set_ylim(min(q3.min(), r3.min()) - 5, max(q3.max(), r3.max()) + 5)
    ax_c.grid(alpha=0.3)
    ax_c.legend(fontsize=8, loc="upper left")

    ax_t = fig.add_subplot(gs[3, 2])
    ax_t.axis("off")
    llegos = [texto for _, tipo, texto in eventos if tipo == "msg" and texto.startswith(("LLEGO,", "MV_ERROR,"))]
    lineas = ["Llegadas (q1 q2 q3 · e1 e2 e3):"]
    for l in llegos:
        p = l.split(",")
        if p[0] == "LLEGO":
            lineas.append(f"{p[2]} {p[3]} {p[4]} · {float(p[5]):+.2f} {float(p[6]):+.2f} {float(p[7]):+.2f}")
        else:
            lineas.append(f"CORTADO: {p[1]}")
    lineas.append("")
    lineas.append("sombreado: verde firme, celeste frenada")
    ax_t.text(0, 1, "\n".join(lineas), va="top", family="monospace", fontsize=8)

    fig.suptitle(f"20 · telemetría de mv  ({Path(archivo).name})", fontsize=12)
    png = Path(archivo).with_suffix(".png")
    fig.savefig(png, dpi=110, bbox_inches="tight")
    print(f"PNG: {png.relative_to(png.parent.parent)}")
    if not sin_ventana:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("comandos", nargs="*", help='comandos en orden, entre comillas ("mv 20 20 24")')
    ap.add_argument("--cola", type=float, default=3.0, help="s de captura después del último comando (def. 3)")
    ap.add_argument("--espera", type=float, default=2.0, help="s para comandos sin línea de final (def. 2)")
    ap.add_argument("--puerto", default=PUERTO)
    ap.add_argument("--archivo", help="volver a graficar un CSV guardado, sin tocar la placa")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    ap.add_argument("--todo", action="store_true", help="graficar también el homing (def.: desde el primer mv)")
    args = ap.parse_args()
    if args.archivo:
        graficar(args.archivo, args.sin_ventana, args.todo)
        return
    if not args.comandos:
        ap.error('falta la lista de comandos (por ejemplo: h "mv 20 20 24")')
    archivo = correr_en_placa(args)
    if archivo:
        graficar(archivo, args.sin_ventana, args.todo)


if __name__ == "__main__":
    sys.exit(main())
