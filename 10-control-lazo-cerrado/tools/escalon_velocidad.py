#!/usr/bin/env python3
"""Paso 2 de J1: escalón de velocidad en la placa (comando "ev" del firmware), comparado con la simulación.

Manda "ev <v> <ms>" (+v durante ms, -v durante ms y 0), guarda el CSV en resultados/ y grafica la
velocidad real junto con la simulada con las mismas ganancias (tools/diseno_velocidad.py).
Imprime tiempo de establecimiento, sobrepico y error final de la ida, real y simulado.

Antes: cerrar el monitor de PlatformIO, base en su marca, "z" hecho en esta sesión de la placa
(o usar --cero para mandarlo), 12 V encendidos.

Uso:
    python3 tools/escalon_velocidad.py 20                  # ev 20 1000
    python3 tools/escalon_velocidad.py 20 --ms 1500 --kp 0.5 --ki 8
    python3 tools/escalon_velocidad.py --archivo resultados/escalon_vel_....csv   # volver a graficar
Opciones: --puerto (por defecto /dev/ttyUSB0), --cero (manda "z" antes), --sin-ventana.
Ctrl+C durante la prueba manda Enter (el firmware frena).
"""
import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import diseno_velocidad as dv  # noqa: E402

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"
RE_META = re.compile(r"^# escalon_vel (.*)$")
RE_FILA = re.compile(r"^(\d+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+)$")


def leer_meta(linea):
    return {k: float(v) for k, v in (p.split("=") for p in RE_META.match(linea).group(1).split())}


def correr_en_placa(args):
    import serial
    s = serial.Serial(args.puerto, 115200, timeout=0.2)
    lineas, meta, terminado = [], None, False

    def mandar(cmd):
        s.write((cmd + "\n").encode())
        time.sleep(0.3)

    # Todo lo que pasa con el puerto abierto queda dentro del try: salga como salga (Ctrl+C, error de
    # comunicación, tiempo agotado) se manda Enter (el firmware frena) y se guarda lo recibido.
    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"\n")
        time.sleep(0.2)
        s.reset_input_buffer()
        if args.kp is not None:
            mandar(f"kp {args.kp}")
        if args.ki is not None:
            mandar(f"ki {args.ki}")
        if args.cero:
            mandar("z")
        s.reset_input_buffer()
        s.write(f"ev {args.v} {args.ms}\n".encode())
        t_lim = time.time() + 2 * args.ms / 1000 + 5
        while time.time() < t_lim:
            l = s.readline().decode(errors="replace").strip()
            if not l:
                continue
            if RE_META.match(l):
                meta = leer_meta(l)
            if l.startswith("#") or l.startswith("t_ms") or RE_FILA.match(l):
                lineas.append(l)
            else:
                print(l)
            if l.startswith(">> Escalón terminado"):
                terminado = True
                break
            if meta is None and l.startswith(("Primero", "La ida", "Uso", "Tiempo", "La base todavía", ">> Cancelado")):
                terminado = True  # el firmware rechazó el escalón: no movió nada
                break
    except KeyboardInterrupt:
        print("\nCortado con Ctrl+C.")
    except Exception as e:  # noqa: BLE001  (error de comunicación: se guarda lo recibido)
        print(f"\nError de comunicación: {e}")
    finally:
        if not terminado:
            try:
                s.write(b"\n")
                print("Se mandó Enter (freno).")
            except Exception as e:  # noqa: BLE001
                print(f"No se pudo mandar el freno ({e}): APAGAR LOS 12 V.")
        try:
            time.sleep(0.4)
            for l in s.read(2000).decode(errors="replace").splitlines():
                l = l.strip()
                if l.startswith("#"):
                    lineas.append(l)
                elif l:
                    print(l)
        except Exception:  # noqa: BLE001
            pass
        try:
            s.close()
        except Exception:  # noqa: BLE001
            pass
    if meta is None or not any(RE_FILA.match(l) for l in lineas):
        sys.exit("No llegaron datos del escalón.")
    DIR_RESULTADOS.mkdir(exist_ok=True)
    ok = any(l.startswith("# fin ok") for l in lineas)
    causa = next((l[len("# cortado "):] for l in lineas if l.startswith("# cortado")), None)
    cortado = "" if ok else "_cortado"
    if not ok:
        print(f"ATENCIÓN: prueba incompleta ({causa or 'sin marca de fin'}). Se guarda como _cortado.")
    archivo = DIR_RESULTADOS / f"escalon_vel_{datetime.now():%Y%m%d_%H%M%S}{cortado}.csv"
    archivo.write_text("\n".join(lineas) + "\n")
    print(f"CSV: {archivo.relative_to(DIR_RESULTADOS.parent)}")
    return archivo


def cargar(archivo):
    meta, filas = None, []
    for l in Path(archivo).read_text().splitlines():
        if RE_META.match(l):
            meta = leer_meta(l)
        m = RE_FILA.match(l)
        if m:
            filas.append([float(x) for x in m.groups()])
    if meta is None or not filas:
        sys.exit(f"{archivo}: sin datos de escalón.")
    texto = Path(archivo).read_text()
    if "# fin ok" not in texto:
        causa = next((l[len("# cortado "):] for l in texto.splitlines() if l.startswith("# cortado")), "sin marca de fin")
        print(f"ATENCIÓN: prueba incompleta ({causa}): las medidas no son de un escalón completo.")
    return meta, np.array(filas)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("v", type=float, nargs="?", default=20.0, help="°/s de la base (def. 20)")
    ap.add_argument("--ms", type=int, default=1000, help="duración de la ida y de la vuelta (def. 1000)")
    ap.add_argument("--kp", type=float, help="cambiar kp antes del escalón")
    ap.add_argument("--ki", type=float, help="cambiar ki antes del escalón")
    ap.add_argument("--cero", action="store_true", help='mandar "z" antes (la base tiene que estar en su marca)')
    ap.add_argument("--puerto", default="/dev/ttyUSB0")
    ap.add_argument("--archivo", help="volver a graficar un CSV guardado, sin tocar la placa")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()

    archivo = Path(args.archivo) if args.archivo else correr_en_placa(args)
    meta, d = cargar(archivo)
    t = (d[:, 0] - 10) / 1000.0  # la primera muestra sale al final del primer periodo
    w_ref, w_med, duty, pos = d[:, 1], d[:, 2], d[:, 3], d[:, 4]
    v, ms, kp, ki = meta["v"], meta["ms"], meta["kp"], meta["ki"]

    # Simulación con las mismas ganancias y el mismo perfil
    perfil = lambda tt: v if tt < ms / 1000 else (-v if tt < 2 * ms / 1000 else 0.0)
    dv.ZONA_MUERTA = meta["zm"]
    ts_sim, _, w_sim, wm_sim, u_sim = dv.simular(kp, ki, perfil, t[-1] + 0.01, realista=True,
                                                 ventana=int(meta["ventana"]))

    print(f"\nkp = {kp:.4f}, ki = {ki:.4f}, escalón ±{v:.0f} °/s, {ms:.0f} ms")
    print("Ida 0 -> +%.0f °/s     ts(2 %%)    sobrepico   error final" % v)
    fin_ida = ms / 1000
    for nombre, tt, ww in (("real (medida)", t, w_med), ("simulada", ts_sim, wm_sim)):
        ts_, mp, ess = dv.medir_escalon(tt, ww, v, 0.0, fin_ida)
        print(f"  {nombre:14s}      {ts_:6.3f} s   {mp:6.1f} %    {ess:+6.2f} °/s")

    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, height_ratios=[2, 1])
    ax1.plot(t, w_ref, "k--", lw=1.2, label="pedida")
    ax1.plot(ts_sim, wm_sim, color="#1d64c8", lw=1.6, label="simulada (medida, 40 ms)")
    ax1.plot(t, w_med, color="#c2410c", lw=1.6, label="real (medida por el encoder)")
    ax1.set_ylabel("velocidad de la base [°/s]")
    ax1.set_title(f"J1 · escalón de velocidad real vs. simulado (Kp = {kp:.3f}, Ki = {ki:.2f})")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)
    ax2.plot(ts_sim, u_sim, color="#1d64c8", lw=1.4, label="simulado")
    ax2.plot(t, duty, color="#c2410c", lw=1.4, label="real")
    ax2.set_ylabel("duty [%]")
    ax2.set_xlabel("tiempo [s]")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    png = archivo.with_suffix(".png")
    fig.savefig(png, dpi=120)
    print(f"Gráfica: {png}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
