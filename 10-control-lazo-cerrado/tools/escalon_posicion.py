#!/usr/bin/env python3
"""Paso 3 de J1: escalones de posición en la placa (comando "ep" del firmware), comparados con la simulación.

Manda "ep <A> <ms> <ciclos>": desde donde está la base, tramos de ms a base + A, base, base - A y base,
repetidos. Guarda el CSV en resultados/ y calcula para cada escalón el tiempo de llegada (entrar y quedarse
a menos de 0,5°), el sobrepico y el error final, junto con los de la simulación (tools/diseno_posicion.py).

Antes: cerrar el monitor de PlatformIO, "z" hecho con la base en su marca, 12 V encendidos.

Uso:
    python3 tools/escalon_posicion.py 20                     # ep 20 2500 1 (~11 s)
    python3 tools/escalon_posicion.py 20 --ciclos 10         # ~1 min 40 s, 40 escalones
    python3 tools/escalon_posicion.py 20 --kpp 2.5
    python3 tools/escalon_posicion.py --archivo resultados/escalon_pos_....csv
Opciones: --ms (duración de cada tramo, def. 2500), --kp, --ki, --ff (J3), --bj (J3), --cero, --puerto, --sin-ventana.
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
import diseno_posicion as dp  # noqa: E402
import diseno_velocidad as dv  # noqa: E402

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"
RE_META = re.compile(r"^# escalon_pos (.*)$")
RE_FILA = re.compile(r"^(\d+)" + r",([+-]?[\d.]+)" * 5 + r"$")
RECHAZOS = ("Primero", "Desde", "Uso", "La base todavía", ">> Cancelado")


def leer_meta(linea):
    return {k: float(v) for k, v in (p.split("=") for p in RE_META.match(linea).group(1).split())}


def correr_en_placa(args):
    import serial
    s = serial.Serial(args.puerto, 115200, timeout=0.2)
    lineas, meta, terminado = [], None, False
    # Todo lo que pasa con el puerto abierto queda dentro del try: salga como salga se manda Enter
    # (el firmware frena) y se guarda lo recibido.
    try:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"\n")
        time.sleep(0.2)
        previos = []
        if args.cero:
            previos.append("z")
        if args.kp is not None:
            previos.append(f"kp {args.kp}")
        if args.ki is not None:
            previos.append(f"ki {args.ki}")
        if args.ff is not None:
            previos.append("ff " + " ".join(f"{x:g}" for x in args.ff))
        if args.kpp is not None:
            previos.append(f"kpp {args.kpp}")
        if args.bj is not None:
            previos.append("bj " + " ".join(f"{x:g}" for x in args.bj))
        for c in previos:
            s.write(f"{c}\n".encode())
            time.sleep(0.3)
        s.reset_input_buffer()
        s.write(f"ep {args.a} {args.ms} {args.ciclos}\n".encode())
        duracion = 4 * args.ciclos * args.ms / 1000 + 1
        print(f"Prueba de ~{duracion:.0f} s. Ctrl+C frena.")
        t_lim = time.time() + duracion + 8
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
            if l.startswith(">> Escalones de posición terminados"):
                terminado = True
                break
            if meta is None and l.startswith(RECHAZOS):
                terminado = True  # el firmware rechazó la prueba: no movió nada
                break
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
        sys.exit("No llegaron datos.")
    ok = any(l.startswith("# fin ok") for l in lineas)
    if not ok:
        causa = next((l[len("# cortado "):] for l in lineas if l.startswith("# cortado")), "sin marca de fin")
        print(f"ATENCIÓN: prueba incompleta ({causa}). Se guarda como _cortado.")
    DIR_RESULTADOS.mkdir(exist_ok=True)
    archivo = DIR_RESULTADOS / f"escalon_pos_{datetime.now():%Y%m%d_%H%M%S}{'' if ok else '_cortado'}.csv"
    archivo.write_text("\n".join(lineas) + "\n")
    print(f"CSV: {archivo.relative_to(DIR_RESULTADOS.parent)}")
    return archivo


def cargar(archivo):
    texto = Path(archivo).read_text()
    meta, filas = None, []
    for l in texto.splitlines():
        if RE_META.match(l):
            meta = leer_meta(l)
        m = RE_FILA.match(l)
        if m:
            filas.append([float(x) for x in m.groups()])
    if meta is None or not filas:
        sys.exit(f"{archivo}: sin datos.")
    if "# fin ok" not in texto:
        causa = next((l[len("# cortado "):] for l in texto.splitlines() if l.startswith("# cortado")), "sin marca de fin")
        print(f"ATENCIÓN: prueba incompleta ({causa}).")
    return meta, np.array(filas)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", type=float, nargs="?", default=20.0, help="amplitud, ° (def. 20)")
    ap.add_argument("--ms", type=int, default=2500, help="duración de cada tramo (def. 2500)")
    ap.add_argument("--ciclos", type=int, default=1, help="ciclos +A, 0, -A, 0 (def. 1)")
    ap.add_argument("--kpp", type=float, help="cambiar el P de posición antes")
    ap.add_argument("--kp", type=float, help="cambiar kp del PI de velocidad antes")
    ap.add_argument("--ki", type=float, help="cambiar ki del PI de velocidad antes")
    ap.add_argument("--ff", type=float, nargs="+", metavar="X",
                    help="feedforward en %% antes (J3): despegue+ marcha+ despegue- marcha- [%%/° subiendo]")
    ap.add_argument("--bj", type=float, nargs=3, metavar=("VMIN-", "TRABADA_MS", "DITHER"),
                    help="ajustes contra el traba-suelta al bajar (J3): vmin bajando °/s, ms trabada, dither %%")
    ap.add_argument("--cero", action="store_true", help='mandar "z" antes (la articulación tiene que estar en su marca)')
    ap.add_argument("--puerto", default="/dev/ttyUSB0")
    ap.add_argument("--archivo", help="volver a analizar un CSV guardado, sin tocar la placa")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    ap.add_argument("--titulo", default="", help='prefijo del título del gráfico, por ejemplo "J3"')
    args = ap.parse_args()
    if args.ff is not None and len(args.ff) not in (4, 5):
        ap.error("--ff lleva 4 o 5 números")

    archivo = Path(args.archivo) if args.archivo else correr_en_placa(args)
    meta, d = cargar(archivo)
    t = (d[:, 0] - 10) / 1000.0
    th_ref, th, w_ref, w_med, duty = d[:, 1], d[:, 2], d[:, 3], d[:, 4], d[:, 5]
    base, amp, seg = meta["base"], meta["A"], meta["seg_ms"] / 1000
    ciclos = int(meta["ciclos"])

    # Escalones: (inicio, desde, hasta)
    lista, desde = [], base
    for i in range(4 * ciclos):
        hasta = base + (amp if i % 4 == 0 else (-amp if i % 4 == 2 else 0.0))
        lista.append((i * seg, desde, hasta))
        desde = hasta

    # Simulación del primer ciclo con los mismos parámetros (relativa a la base)
    dv.ZONA_MUERTA = meta["zm"]
    dp.KP_VEL, dp.KI_VEL = meta["kp"], meta["ki"]
    perfil = lambda tt: (amp if (int(tt // seg) % 4 == 0) else (-amp if int(tt // seg) % 4 == 2 else 0.0)) \
        if tt < 4 * seg else 0.0
    ts_, tr_s, th_s, wr_s, w_s, u_s, corte_sim = dp.simular(meta["kpp"], meta["vmax"], meta["amax"], meta["tol"],
                                                             perfil, 4 * seg + 1.0,
                                                             meta.get("tol_salida", meta["tol"]),
                                                             meta.get("vmin", 0.0))
    if corte_sim:
        print(f"Aviso: con estos parámetros la simulación corta por {corte_sim}.")

    print(f"\nkpp = {meta['kpp']:.3f}, v_max {meta['vmax']:.0f} °/s, a_max {meta['amax']:.0f} °/s², "
          f"tol ±{meta['tol']:.2f}° · PI kp {meta['kp']:.3f} ki {meta['ki']:.2f}")
    print(f"{len(lista)} escalones de ±{amp:.0f}° desde {base:+.2f}°\n")
    print("Escalón               llegada (±0,5°)        sobrepico [°]          error final [°]")
    por_tipo = {}
    for t0, de, a in lista:
        if t0 + seg > t[-1]:
            break
        r = dp.medir(t, th, a, t0, t0 + seg, de)
        por_tipo.setdefault((round(de - base), round(a - base)), []).append(r)
    for i, ((de, a), rs) in enumerate(por_tipo.items()):
        r = np.array(rs, dtype=float)
        t0s = i * seg
        rs_sim = dp.medir(ts_, th_s, a, t0s, t0s + seg, de)
        print(f"  {de:+4.0f}° -> {a:+4.0f}° (x{len(rs):2d})  real {np.nanmean(r[:,0]):5.2f} ±{np.nanstd(r[:,0]):4.2f} s"
              f"  sim {rs_sim[0]:4.2f}   real {np.mean(r[:,1]):4.2f} (peor {np.max(r[:,1]):4.2f})  sim {rs_sim[1]:4.2f}"
              f"   real {np.mean(r[:,2]):+5.2f} (peor {r[np.argmax(np.abs(r[:,2])),2]:+5.2f})  sim {rs_sim[2]:+5.2f}")
    print(f"Posición final {th[-1]:+.2f}° (base {base:+.2f}°)")
    # Zumbido: arranques del motor (duty de 0 a distinto de 0) con el ángulo pedido sin cambiar
    arranques = np.sum((duty[1:] != 0) & (duty[:-1] == 0) & (th_ref[1:] == th_ref[:-1]))
    print(f"Arranques del motor sin cambio del ángulo pedido (zumbido): {arranques} "
          f"({arranques / max(1, len(lista)):.1f} por escalón)")

    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True, height_ratios=[2, 1.2, 1])
    n_sim = len(ts_)
    ax[0].plot(t, th_ref, "k--", lw=1.1, label="pedido")
    ax[0].plot(t, th, color="#c2410c", lw=1.6, label="real (encoder)")
    ax[0].plot(ts_, th_s + base, color="#1d64c8", lw=1.4, ls="-", alpha=0.8, label="simulado (primer ciclo)")
    ax[0].set_ylabel("ángulo de la articulación [°]")
    ax[0].set_title(f"{args.titulo + ' · ' if args.titulo else ''}Posición: Kpp = {meta['kpp']:.2f}, v_max {meta['vmax']:.0f} °/s, "
                    f"a_max {meta['amax']:.0f} °/s², tol ±{meta['tol']}°")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=9)
    ax[1].plot(t, w_ref, "k--", lw=0.9, label="velocidad pedida")
    ax[1].plot(t, w_med, color="#c2410c", lw=1.1, label="velocidad medida")
    ax[1].plot(ts_[:n_sim], w_s, color="#1d64c8", lw=1, alpha=0.7, label="simulada")
    ax[1].set_ylabel("°/s")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=8)
    ax[2].plot(t, duty, color="#c2410c", lw=1)
    ax[2].plot(ts_, u_s, color="#1d64c8", lw=0.9, alpha=0.7)
    ax[2].set_ylabel("duty [%]")
    ax[2].set_xlabel("tiempo [s]")
    ax[2].grid(alpha=0.3)
    fig.tight_layout()
    png = archivo.with_suffix(".png")
    fig.savefig(png, dpi=110)
    print(f"Gráfica: {png}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
