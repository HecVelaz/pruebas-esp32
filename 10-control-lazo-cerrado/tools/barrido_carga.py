#!/usr/bin/env python3
"""J3: barrido de carga. Mide cuánto duty hace falta en cada ángulo, subiendo y bajando (comando "bc").

Manda "bc <desde> <hasta> <v>" y después "bc <hasta> <desde> <v>": el firmware va a <desde> con el control de
posición y recorre el rango a velocidad constante y baja con el PI de velocidad. Con la velocidad constante, el
duty que aplica el PI en cada ángulo es el esfuerzo que hace falta ahí (peso del antebrazo a través del cuatro
barras + fricción). Guarda el CSV y el PNG en resultados/ e imprime la tabla por tramos de ángulo, lista para
el feedforward según el ángulo de config.h.

Antes: cerrar el monitor, barra roja en la marca, "z" hecho (o --cero), 12 V encendidos.

Uso:
    python3 tools/barrido_carga.py --cero                    # -12° -> +45° -> -12° a 4 °/s (~35 s)
    python3 tools/barrido_carga.py --desde -12 --hasta 45 --v 3
    python3 tools/barrido_carga.py --archivo resultados/barrido_....csv   # volver a analizar
Ctrl+C durante la prueba manda Enter (el firmware frena).
"""
import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"
RE_META = re.compile(r"^# barrido (.*)$")
RE_FILA = re.compile(r"^(\d+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+),([+-]?[\d.]+)$")
RECHAZOS = ("Primero", "Uso", "La base todavía", ">> Cancelado")


def leer_meta(linea):
    return {k: float(v) for k, v in (p.split("=") for p in RE_META.match(linea).group(1).split())}


def un_barrido(s, desde, hasta, v):
    """Corre "bc" y devuelve (líneas del CSV, terminó bien)."""
    lineas, meta = [], None
    s.reset_input_buffer()
    s.write(f"bc {desde} {hasta} {v}\n".encode())
    t_lim = time.time() + 15 + abs(hasta - desde) / v + 10
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
        if l.startswith(">> Barrido terminado"):
            return lineas, "# fin ok" in lineas
        if meta is None and l.startswith(RECHAZOS):
            return lineas, False
    return lineas, False


def correr_en_placa(args):
    import serial
    s = serial.Serial(args.puerto, 115200, timeout=0.2)
    todas, terminado = [], False
    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"\n")
        time.sleep(0.2)
        if args.cero:
            s.write(b"z\n")
            time.sleep(0.3)
        print(f"Barrido {args.desde:+.0f}° -> {args.hasta:+.0f}° -> {args.desde:+.0f}° a {args.v:g} °/s. Ctrl+C frena.")
        for a, b in ((args.desde, args.hasta), (args.hasta, args.desde)):
            lineas, ok = un_barrido(s, a, b, args.v)
            todas += lineas
            if not ok:
                break
        else:
            terminado = True
    except KeyboardInterrupt:
        print("\nCortado con Ctrl+C.")
    except Exception as e:  # noqa: BLE001
        print(f"\nError de comunicación: {e}")
    finally:
        if not terminado:
            try:
                s.write(b"\n")
                print("Se mandó Enter (freno).")
            except Exception as e:  # noqa: BLE001
                print(f"No se pudo mandar el freno ({e}): APAGAR LOS 12 V.")
        try:
            s.close()
        except Exception:  # noqa: BLE001
            pass
    if not any(RE_FILA.match(l) for l in todas):
        print("No llegaron datos.")
        sys.exit(1)
    DIR_RESULTADOS.mkdir(exist_ok=True)
    sufijo = "" if terminado else "_cortado"
    ruta = DIR_RESULTADOS / f"barrido_{datetime.now():%Y%m%d_%H%M%S}{sufijo}.csv"
    ruta.write_text("\n".join(todas) + "\n")
    print(f"CSV: {ruta.relative_to(DIR_RESULTADOS.parent)}")
    return ruta


def leer_tramos(ruta):
    """Devuelve una lista de (meta, datos) por barrido; datos con columnas t, w_ref, w_med, duty, pos."""
    tramos, meta, filas = [], None, []
    for l in Path(ruta).read_text().splitlines():
        if RE_META.match(l):
            if meta is not None and filas:
                tramos.append((meta, np.array(filas)))
            meta, filas = leer_meta(l), []
        elif RE_FILA.match(l):
            filas.append([float(x) for x in l.split(",")])
    if meta is not None and filas:
        tramos.append((meta, np.array(filas)))
    return tramos


def analizar(ruta, paso, sin_ventana):
    import matplotlib
    if sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    tramos = leer_tramos(ruta)
    if not tramos:
        print("El archivo no tiene barridos.")
        return
    lo = min(min(d[:, 4]) for _, d in tramos)
    hi = max(max(d[:, 4]) for _, d in tramos)
    bordes = np.arange(np.floor(lo / paso) * paso, np.ceil(hi / paso) * paso + paso, paso)
    centros = (bordes[:-1] + bordes[1:]) / 2
    tabla = {}
    fig, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for meta, d in tramos:
        sube = meta["v"] > 0
        nombre = "subiendo" if sube else "bajando"
        t, wref, w, u, pos = d.T
        # Se descarta el primer medio segundo (arranque) y los tramos trabados: interesa el esfuerzo en marcha
        ok = (t > 500) & (np.abs(w) > 0.3 * abs(meta["v"]))
        med = []
        for a, b in zip(bordes[:-1], bordes[1:]):
            m = ok & (pos >= a) & (pos < b)
            med.append(np.median(u[m]) if m.sum() >= 5 else np.nan)
        tabla[nombre] = np.array(med)
        color = "tab:red" if sube else "tab:blue"
        ax[0].plot(pos, u, ".", ms=2, alpha=0.25, color=color)
        ax[0].plot(centros, med, "o-", color=color, label=f"{nombre}: mediana por tramo de {paso:g}°")
        ax[1].plot(pos, w, lw=0.7, color=color, label=f"{nombre} (pedida {meta['v']:+.0f} °/s)")
        print(f"{nombre}: {len(t)} muestras, {meta['desde']:+.1f}° -> {pos[-1]:+.1f}°, v media "
              f"{np.mean(w[t > 500]):+.2f} °/s (pedida {meta['v']:+.1f})")
    ax[0].set_ylabel("duty [%]")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].legend()
    ax[0].grid(alpha=0.3)
    ax[1].set_ylabel("velocidad [°/s]")
    ax[1].set_xlabel("ángulo de J3 desde la marca [°]")
    ax[1].legend()
    ax[1].grid(alpha=0.3)
    fig.suptitle("J3 · duty necesario según el ángulo (barrido a velocidad constante)")
    fig.tight_layout()
    png = Path(ruta).with_suffix(".png")
    fig.savefig(png, dpi=110)

    print(f"\n{'ángulo':>8} {'subiendo':>9} {'bajando':>9}   (duty en %, mediana en marcha)")
    for i, c in enumerate(centros):
        s_ = tabla.get("subiendo", [np.nan] * len(centros))[i]
        b_ = tabla.get("bajando", [np.nan] * len(centros))[i]
        print(f"{c:+8.1f} {s_:+9.1f} {b_:+9.1f}")
    print(f"Gráfica: {png}")
    if not sin_ventana:
        plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--desde", type=float, default=-12.0, help="ángulo de inicio (def. -12)")
    ap.add_argument("--hasta", type=float, default=45.0, help="ángulo final (def. +45)")
    ap.add_argument("--v", type=float, default=4.0, help="°/s del barrido (def. 4)")
    ap.add_argument("--paso", type=float, default=3.0, help="ancho de los tramos de la tabla, ° (def. 3)")
    ap.add_argument("--cero", action="store_true", help='mandar "z" antes (barra roja en su marca)')
    ap.add_argument("--puerto", default="/dev/ttyUSB0")
    ap.add_argument("--archivo", help="volver a analizar un CSV guardado, sin tocar la placa")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()
    ruta = args.archivo or correr_en_placa(args)
    analizar(ruta, args.paso, args.sin_ventana)


if __name__ == "__main__":
    main()
