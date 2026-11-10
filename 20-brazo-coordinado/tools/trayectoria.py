#!/usr/bin/env python3
"""Fase 5c: telemetría de 20 ("t", 50 Hz) durante una serie de comandos; la guarda en un CSV y la grafica.

Manda "t 1", cada comando de la lista esperando su final ("mv" hasta LLEGO o MV_ERROR, "h" hasta HOMING o
HOMING_ERROR, los demás --espera s), sigue capturando --cola s (el sostener después de llegar), manda "mv 3 10 5" si la lista no
terminaba ahí (cerca del cero, para que el próximo homing tarde poco; no entra en el resumen; --sin-volver: no) y
manda "t 0".
Guarda resultados/trayectoria_<fecha>_<hora>.csv y su .png: para cada articulación, referencia contra medido, el error
(con la tolerancia y la banda del integrador; ahí se ve el tambaleo del final) y el duty; abajo, el camino (q2, q3)
sobre la franja del cuatro barras. Imprime un resumen para comparar capturas: por mv y articulación, inversiones del
duty (cambio de signo con salto > 15 %) en el recorrido y en la aproximación, |error| máximo de seguimiento en el
recorrido, error del LLEGO, correcciones después de llegar, cuánto se mueve el encoder ya firmes, y el tiempo de la
llegada hasta las tres firmes; al final los totales y, con --cola, cuánto se movió cada encoder después del último mv.

Antes: cerrar el monitor (el puerto no se abre dos veces), brazo abajo y recogido, 12 V encendidos. Abrir el puerto
puede reiniciar la WROOM (pierde el cero): si alguna está sin cero, el primer comando tiene que ser "h".

Uso:
    python3 tools/trayectoria.py h "mv 20 20 14" "mv 40 30 20" "mv 20 20 14" "mv 3 10 5"
    python3 tools/trayectoria.py "mv 20 20 14" --cola 5
    python3 tools/trayectoria.py --archivo resultados/trayectoria_....csv     # volver a graficar, sin la placa
    python3 tools/trayectoria.py --archivo resultados/trayectoria_....csv --solo-resumen \
        --comparar resultados/trayectoria_20261031_180843.csv                  # contra la base (con el resorte)
Lista fija para comparar: h "mv 20 20 14" "mv 40 30 20" "mv 20 20 14" "mv 3 10 5".
Opciones: --comparar CSV_BASE (totales lado a lado), --solo-resumen (con --archivo, sin gráfico), --sin-volver, --cola (s de captura después del último comando, def. 3), --espera (s para comandos sin final, def. 2),
--puerto, --sin-ventana, --todo (graficar también el homing; por defecto arranca en el primer mv).
Ctrl+C manda "s" (las tres se sostienen donde están) y guarda lo capturado (archivo ..._cortado).
Si un "mv" se corta, vuelve solo a "mv 3 10 5" y hace "h" (rescate; --sin-rescate para dejarlo donde quedó).
"""
import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

DIR_RESULTADOS = Path(__file__).absolute().parent.parent / "resultados"
PUERTO = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
# Franja del cuatro barras (include/config.h, manivela de 95 mm, 2026-11-04):
#   q2 - Q2_MENOS_Q3_MAX <= q3 <= borde de arriba (tabla FRANJA_ARRIBA (q2, q3 máx), interpolada)
Q2_MENOS_Q3_MAX = 49.0
# 2026-11-10 (doble helicoidales): medida de nuevo, resultados/limites_20261110_145545.csv
FRANJA_ARRIBA = [(0.0, 0.5), (9.9, 3.0), (15.1, 4.0), (30.1, 16.1), (45.1, 31.3), (60.2, 52.0), (74.6, 75.1),
                 (80.1, 83.4)]
NOMBRES = ("J1", "J2", "J3")
# Al final de cada serie (salvo --sin-volver): cerca del cero, listo para el próximo homing. 2026-11-04: 3 3 3 -> 3 10 5
# (el punto intermedio de las salidas): en (3, 3) J2/J3 llegaban sin la aproximación desde abajo (el punto previo
# quedaba a 1° del final de carrera) y J3 oscilaba ±0,3° hasta "no quedan firmes" (trayectoria_20261104_230336_cortado)
VOLVER = "mv 3 20 5"  # 2026-11-10: (10, 5) quedó fuera de la franja nueva
COLUMNAS = ["t_ms", "etapa"] + [f"{c}{i}" for i in (1, 2, 3) for c in ("q", "r", "u", "w", "s")]
ESTADOS = {0: "suelta", 1: "yendo", 2: "llegó", 3: "firme", 4: "frenada", 5: "siguiendo", 6: "homing"}
ETAPAS = {0: "", 1: "recorrido", 2: "aproximación", 3: "llegada", 4: "asentamiento", 8: "joystick", 9: "homing"}


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
        # "zq q1 q2 q3" (cero estimado: el brazo quedó lejos de los finales de carrera y no se mueve sin corriente)
        # también vale como primer comando: baja rápido con "mv" y después "h"
        if sin_cero and (not args.comandos or not (args.comandos[0] == "h" or args.comandos[0].startswith("zq "))):
            print('\nAlguna está SIN CERO (la placa se reinició al abrir el puerto): empezar la lista con "h" '
                  '(o con "zq q1 q2 q3" si se sabe dónde quedó el brazo).')
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
                print(f'\nSe corta la serie: {cortado}. "s" (las tres se sostienen donde están).')
                s.write(b"s\n")  # si sigue en curso (venció la espera), que no siga solo
                break
        cap.leer(args.cola)
        # Volver cerca del cero: el próximo homing (al abrir el puerto la WROOM se reinicia y pierde el cero) busca
        # a velocidad lenta desde donde quedó el brazo. Solo si la serie terminó bien y movió algo
        movio = any(c == "h" or c.startswith(("mv ", "j ")) for c in args.comandos)
        if args.volver and not cortado and movio and args.comandos[-1].split() != VOLVER.split():
            # evento "vuelta" (no "cmd"): el resumen no la cuenta como un mv de la serie y la cola termina acá
            print(f"<< {VOLVER} (vuelta)")
            cap.eventos.append((cap.t_actual(), "vuelta", VOLVER))
            cap.s.write(f"{VOLVER}\n".encode())
            r = cap.leer(90.0, ("LLEGO,", "MV_ERROR,", "Uso:", "Fuera de", "J2 y J3 fuera", "sin cero"))
            if r is None or not r.startswith("LLEGO,"):
                print(f'\nNo volvió cerca del cero: {r or "no terminó a tiempo"}. "s" (se sostienen donde están).')
                s.write(b"s\n")
        # Rescate (pedido del usuario, 2026-11-05): si un "mv" se cortó (franja, "no quedan firmes", ...), volver solo al
        # reposo y hacer el homing, en lugar de dejar el brazo sostenido donde quedó. Si la vuelta falla, igual "h"
        # (con cero, de a una y respetando la franja). --sin-rescate: dejarlo donde quedó
        if cortado and args.rescate and c.startswith("mv "):
            for rc, fin, tmax in ((VOLVER, ("LLEGO,", "MV_ERROR,", "Uso:", "Fuera de", "J2 y J3 fuera", "sin cero"), 90.0),
                                  ("h", ("HOMING,", "HOMING_ERROR,", "Homing: J2"), 240.0)):
                print(f"<< {rc} (rescate)")
                cap.eventos.append((cap.t_actual(), "vuelta", rc))
                cap.s.write(f"{rc}\n".encode())
                r = cap.leer(tmax, fin)
                if r is None or not (r.startswith("LLEGO,") or r.startswith("HOMING,")):
                    print(f'Rescate: "{rc}" no terminó ({r or "venció la espera"}). "s" (se sostienen donde están).')
                    s.write(b"s\n")
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
            if l.startswith(("# cmd,", "# msg,", "# vuelta,")):
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


SALTO_INVERSION = 15.0  # % de duty: una inversión cuenta si cambia de signo con un salto mayor que esto


def pares_mv(eventos):
    """Cada mv con la línea de final que le sigue EN ORDEN (el siguiente mv se manda en el mismo t que ese LLEGO)."""
    pares = []
    for tt, tipo, c in eventos:
        if tipo == "cmd" and c.startswith("mv "):
            pares.append([tt, c, None])
        elif tipo == "msg" and c.startswith(("LLEGO,", "MV_ERROR,")) and pares and pares[-1][2] is None:
            pares[-1][2] = (tt, c)
    return pares


def inversiones(u, m):
    """Muestras consecutivas dentro de la máscara m en las que el duty cambia de signo con un salto > 15 %."""
    import numpy as np
    mm = m[1:] & m[:-1]
    return int(np.sum(mm & (u[1:] * u[:-1] < 0) & (np.abs(np.diff(u)) > SALTO_INVERSION)))


def resumen(d, eventos):
    """Métricas de cada mv para comparar capturas. Por articulación: inversiones de duty (> 15 %) en el recorrido
    (etapa 1) y en la aproximación (2), |error| máximo de seguimiento en el recorrido, error del LLEGO, correcciones
    después de llegar (|w| > 1 °/s, etapas 3 y 4) y cuánto se movió el encoder (pico a pico) ya firmes (etapa 4).
    Por mv: duración hasta la llegada y tiempo de la llegada (etapa 3) hasta las tres firmes (etapa 4)."""
    import numpy as np
    t, et = d[:, 0], d[:, 1]
    filas = []
    for t0, c, fin in pares_mv(eventos):
        t1 = fin[0] if fin else t[-1]
        tramo = (t > t0) & (t <= t1)
        if not tramo.any():
            continue
        f = {"mv": c, "final": fin[1] if fin else "sin final", "llego": bool(fin and fin[1].startswith("LLEGO,")),
             "t_rec": float("nan"), "t_firme": float("nan"), "art": []}
        e_llego = [float(x) for x in fin[1].split(",")[5:8]] if f["llego"] else [float("nan")] * 3
        llegada = tramo & (et >= 3)
        firmes = tramo & (et == 4)
        if llegada.any():
            t_lleg = t[llegada][0]
            f["t_rec"] = (t_lleg - t0) / 1000
            if firmes.any():
                # la última entrada a la etapa 4 (si pierde una la firmeza, vuelve a la 3)
                entradas = np.flatnonzero(firmes & ~np.concatenate(([False], firmes[:-1])))
                f["t_firme"] = (t[entradas[-1]] - t_lleg) / 1000
        for i in range(3):
            q, r, u, w, s = columnas(d, i)
            rec = tramo & (et == 1)
            mov = np.abs(w[llegada]) > 1.0
            f["art"].append({
                "inv_rec": inversiones(u, rec),
                "inv_aprox": inversiones(u, tramo & (et == 2)),
                "e_seg": float(np.max(np.abs(r[rec] - q[rec]))) if rec.any() else float("nan"),
                "e_llego": e_llego[i],
                "corr": int(np.sum(mov[1:] & ~mov[:-1]) + (1 if mov.size and mov[0] else 0)) if mov.size else 0,
                "pp_firme": float(np.ptp(q[firmes])) if firmes.any() else float("nan"),
            })
        filas.append(f)
    # Después del último final (la cola): cuánto se mueve cada encoder, para la deriva
    cola = None
    if filas and pares_mv(eventos)[-1][2]:
        tc = pares_mv(eventos)[-1][2][0]
        m = t > tc
        t_vuelta = [tt for tt, tipo, _ in eventos if tipo == "vuelta"]
        if t_vuelta:
            m &= t <= t_vuelta[0]
        if m.sum() > 1:
            cola = {"s": (t[m][-1] - t[m][0]) / 1000, "pp": [float(np.ptp(columnas(d, i)[0][m])) for i in range(3)]}
    return filas, cola


def totales(filas):
    import numpy as np
    tot = []
    for i in range(3):
        a = [f["art"][i] for f in filas]
        tot.append({
            "inv_rec": sum(x["inv_rec"] for x in a),
            "inv_aprox": sum(x["inv_aprox"] for x in a),
            "e_seg": float(np.nanmax([x["e_seg"] for x in a])) if a else float("nan"),
            "e_llego": float(np.nanmax([abs(x["e_llego"]) for x in a])) if a else float("nan"),
            "corr": sum(x["corr"] for x in a),
            "pp_firme": float(np.nanmax([x["pp_firme"] for x in a])) if a else float("nan"),
        })
    t_firme = [f["t_firme"] for f in filas]
    return tot, (float(np.nansum(t_firme)), float(np.nanmax(t_firme)) if filas else float("nan"))


ENCABEZADO = "      inv>15% rec+aprox  |e| máx rec  e LLEGO  correcc.  p-p firme"


def linea_art(nombre, x):
    return (f"  {nombre}  {x['inv_rec']:4d} + {x['inv_aprox']:3d} = {x['inv_rec'] + x['inv_aprox']:4d}"
            f"   {x['e_seg']:7.2f}°   {x['e_llego']:+6.3f}°  {x['corr']:5d}    {x['pp_firme']:6.3f}°")


def imprimir_resumen(filas, cola, titulo=""):
    print(f"\n=== Resumen{' de ' + titulo if titulo else ''} ===")
    for f in filas:
        estado = "" if f["llego"] else f"  ({f['final']})"
        print(f"{f['mv']}: recorrido {f['t_rec']:.1f} s, llegada→firmes {f['t_firme']:.1f} s{estado}")
        print(ENCABEZADO)
        for i in range(3):
            print(linea_art(NOMBRES[i], f["art"][i]))
    tot, (tf_sum, tf_max) = totales(filas)
    print(f"TOTAL ({len(filas)} mv): llegada→firmes suma {tf_sum:.1f} s, peor {tf_max:.1f} s "
          "(e LLEGO y p-p: el peor en valor absoluto)")
    print(ENCABEZADO)
    for i in range(3):
        print(linea_art(NOMBRES[i], tot[i]))
    if cola:
        print(f"Cola ({cola['s']:.1f} s después del último final), encoder pico a pico: "
              + "  ".join(f"{NOMBRES[i]} {cola['pp'][i]:.3f}°" for i in range(3)))


def comparar(archivo, base):
    """Totales de la captura contra los de una base (por ejemplo la 180843), lado a lado."""
    res = []
    for a in (base, archivo):
        d, eventos, _ = cargar(a)
        filas, _ = resumen(d, eventos)
        res.append((Path(a).name, [f["mv"] for f in filas], totales(filas)))
    if res[0][1] != res[1][1]:
        print(f"\nOJO: listas de mv distintas: {res[0][1]} contra {res[1][1]}")
    print(f"\n=== Comparación: base {res[0][0]} → {res[1][0]} ===")
    print("        inv rec+aprox     |e| máx rec       |e LLEGO| máx      correcc.     p-p firme")
    for i in range(3):
        b, n = res[0][2][0][i], res[1][2][0][i]
        print(f"  {NOMBRES[i]}  {b['inv_rec'] + b['inv_aprox']:4d} → {n['inv_rec'] + n['inv_aprox']:4d}"
              f"      {b['e_seg']:5.2f} → {n['e_seg']:5.2f}°    {b['e_llego']:5.3f} → {n['e_llego']:5.3f}°"
              f"   {b['corr']:3d} → {n['corr']:3d}   {b['pp_firme']:5.3f} → {n['pp_firme']:5.3f}°")
    (bs, bm), (ns, nm) = res[0][2][1], res[1][2][1]
    print(f"  llegada→firmes: suma {bs:.1f} → {ns:.1f} s, peor {bm:.1f} → {nm:.1f} s")


def graficar(archivo, sin_ventana, todo=False, base=None, solo_resumen=False):
    d, eventos, cfg = cargar(archivo)
    if d.size == 0:
        print("CSV sin muestras.")
        return
    imprimir_resumen(*resumen(d, eventos), titulo=Path(archivo).name)
    if base:
        comparar(archivo, base)
    if solo_resumen:
        return
    import matplotlib
    if sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
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
    x = np.linspace(lim[0], lim[1], 200)
    arriba = np.interp(x, [p[0] for p in FRANJA_ARRIBA], [p[1] for p in FRANJA_ARRIBA])
    ax_c.fill_between(x, x - Q2_MENOS_Q3_MAX, arriba, color="tab:green", alpha=0.08,
                      label="franja del cuatro barras")
    ax_c.plot(x, x - Q2_MENOS_Q3_MAX, color="tab:green", lw=0.8)
    ax_c.plot(x, arriba, color="tab:green", lw=0.8)
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
    ap.add_argument("comandos", nargs="*", help='comandos en orden, entre comillas ("mv 20 20 14")')
    ap.add_argument("--cola", type=float, default=3.0, help="s de captura después del último comando (def. 3)")
    ap.add_argument("--sin-rescate", dest="rescate", action="store_false",
                    help='si un mv se corta, no volver solo al reposo ni hacer "h" (por defecto sí)')
    ap.add_argument("--espera", type=float, default=2.0, help="s para comandos sin línea de final (def. 2)")
    ap.add_argument("--puerto", default=PUERTO)
    ap.add_argument("--archivo", help="volver a graficar un CSV guardado, sin tocar la placa")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    ap.add_argument("--todo", action="store_true", help="graficar también el homing (def.: desde el primer mv)")
    ap.add_argument("--comparar", metavar="CSV_BASE", help="comparar los totales con otra captura (misma lista de mv)")
    ap.add_argument("--sin-volver", dest="volver", action="store_false",
                    help=f'no mandar "{VOLVER}" al final (por defecto vuelve cerca del cero si la serie no termina ahí)')
    ap.add_argument("--solo-resumen", action="store_true", help="con --archivo: el resumen, sin graficar")
    args = ap.parse_args()
    if args.archivo:
        graficar(args.archivo, args.sin_ventana, args.todo, args.comparar, args.solo_resumen)
        return
    if not args.comandos:
        ap.error('falta la lista de comandos (por ejemplo: h "mv 20 20 14")')
    archivo = correr_en_placa(args)
    if archivo:
        graficar(archivo, args.sin_ventana, args.todo, args.comparar)


if __name__ == "__main__":
    sys.exit(main())
