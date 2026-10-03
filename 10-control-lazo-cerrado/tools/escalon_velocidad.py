#!/usr/bin/env python3
"""Paso 2 de J1: escalones de velocidad en la placa (comando "ev" del firmware), comparados con la simulación.

Manda "ev <v> <ms> <ciclos>": cada ciclo es +v durante ms, pausa en 0, -v durante ms y pausa en 0 (la
pausa evita invertir de golpe contra el juego de la correa). Guarda el CSV en resultados/ y calcula,
para cada escalón, tiempo de establecimiento, sobrepico y error final: promedio, dispersión y peor caso,
separados en ida (+v) y vuelta (-v), junto con los de la simulación con las mismas ganancias.

Antes: cerrar el monitor de PlatformIO, base en su marca, "z" hecho en esta sesión de la placa
(o usar --cero para mandarlo), 12 V encendidos.

Uso:
    python3 tools/escalon_velocidad.py 20                       # ev 20 1000 1 (un ciclo, ~3 s)
    python3 tools/escalon_velocidad.py 20 --ciclos 40 --ki 4    # 2 minutos, 80 escalones
    python3 tools/escalon_velocidad.py --archivo resultados/escalon_vel_....csv   # volver a analizar
Opciones: --ms, --kp, --ki, --puerto (por defecto /dev/ttyUSB0), --cero (manda "z" antes), --sin-ventana.
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
        s.write(f"ev {args.v} {args.ms} {args.ciclos}\n".encode())
        t_lim = time.time() + args.ciclos * (2 * args.ms + 1000) / 1000 + 5
        print(f"Prueba de ~{args.ciclos * (2 * args.ms + 1000) / 1000:.0f} s. Ctrl+C frena.")
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


def escalones(meta):
    """Lista de (inicio, signo) de cada escalón, en s, según el perfil del firmware."""
    v, ms = meta["v"], meta["ms"] / 1000
    pausa, ciclos = meta.get("pausa_ms", 0) / 1000, int(meta.get("ciclos", 1))
    periodo = 2 * (ms + pausa)
    lista = []
    for i in range(ciclos):
        lista += [(i * periodo, 1), (i * periodo + ms + pausa, -1)]
    return lista, (lambda tt: perfil(tt, v, ms, pausa, ciclos, periodo))


def perfil(tt, v, ms, pausa, ciclos, periodo):
    if tt >= ciclos * periodo:
        return 0.0
    r = tt % periodo
    return v if r < ms else (0.0 if r < ms + pausa else (-v if r < 2 * ms + pausa else 0.0))


def resumir(nombre, valores):
    a = np.array(valores, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return f"{nombre}: sin datos"
    return f"{np.mean(a):7.3f} ± {np.std(a):5.3f}  (peor {a[np.argmax(np.abs(a))]:+7.3f})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("v", type=float, nargs="?", default=20.0, help="°/s de la base (def. 20)")
    ap.add_argument("--ms", type=int, default=1000, help="duración de cada escalón (def. 1000)")
    ap.add_argument("--ciclos", type=int, default=1, help="ciclos +v, 0, -v, 0 (def. 1; 40 = 2 min a 1000 ms)")
    ap.add_argument("--kp", type=float, help="cambiar kp antes del escalón")
    ap.add_argument("--ki", type=float, help="cambiar ki antes del escalón")
    ap.add_argument("--cero", action="store_true", help='mandar "z" antes (la base tiene que estar en su marca)')
    ap.add_argument("--puerto", default="/dev/ttyUSB0")
    ap.add_argument("--archivo", help="volver a analizar un CSV guardado, sin tocar la placa")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()

    archivo = Path(args.archivo) if args.archivo else correr_en_placa(args)
    meta, d = cargar(archivo)
    t = (d[:, 0] - 10) / 1000.0  # la primera muestra sale al final del primer periodo
    w_ref, w_med, duty, pos = d[:, 1], d[:, 2], d[:, 3], d[:, 4]
    v, ms, kp, ki = meta["v"], meta["ms"] / 1000, meta["kp"], meta["ki"]
    if "pausa_ms" not in meta:  # archivos viejos: +v, -v sin pausa
        meta["pausa_ms"], meta["ciclos"] = 0, 1
    lista, f_perfil = escalones(meta)

    # Simulación con las mismas ganancias y el mismo perfil (todos los escalones salen iguales: basta uno
    # de ida y uno de vuelta)
    dv.ZONA_MUERTA = meta["zm"]
    t_sim_fin = min(t[-1], lista[1][0] + ms) + 0.01
    ts_sim, _, w_sim, wm_sim, u_sim = dv.simular(kp, ki, f_perfil, t_sim_fin, realista=True,
                                                 ventana=int(meta["ventana"]))

    res = {1: [], -1: []}
    for t0, sg in lista:
        if t0 + ms > t[-1]:
            break  # prueba cortada antes de este escalón
        res[sg].append(dv.medir_escalon(t, sg * w_med, v, t0, t0 + ms))
    sim = {sg: dv.medir_escalon(ts_sim, sg * wm_sim, v, t0, t0 + ms) for t0, sg in lista[:2]}

    print(f"\nkp = {kp:.4f}, ki = {ki:.4f} · ±{v:.0f} °/s, {ms*1000:.0f} ms, pausa {meta['pausa_ms']:.0f} ms · "
          f"{len(res[1])} idas y {len(res[-1])} vueltas")
    for sg, nombre in ((1, "Ida (+v)"), (-1, "Vuelta (-v)")):
        if not res[sg]:
            continue
        a = np.array(res[sg])
        print(f"{nombre}:")
        print(f"  tiempo de establecimiento [s]  real {resumir('ts', a[:, 0])}   simulado {sim[sg][0]:6.3f}")
        print(f"  sobrepico [%]                  real {resumir('mp', a[:, 1])}   simulado {sim[sg][1]:6.1f}")
        print(f"  error final [°/s]              real {resumir('e', a[:, 2])}   simulado {sim[sg][2]:+6.2f}")
    print(f"Posición: de {pos.min():+.1f}° a {pos.max():+.1f}°, final {pos[-1]:+.2f}° (deriva)")

    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(13, 8))
    paso = t[1] - t[0]
    n = int(round((ms + 0.3) / paso))
    tt_rel = np.arange(n) * paso - 0.0
    for col, sg, nombre in ((0, 1, "ida (+v)"), (1, -1, "vuelta (-v)")):
        a = ax[0, col]
        curvas = []
        for t0, s_ in lista:
            if s_ != sg:
                continue
            k0 = int(np.searchsorted(t, t0))
            if k0 + n > len(t):
                break
            curvas.append(w_med[k0:k0 + n])
            a.plot(tt_rel, w_med[k0:k0 + n], color="#c2410c", lw=0.6, alpha=0.25)
        if curvas:
            a.plot(tt_rel, np.mean(curvas, axis=0), color="#c2410c", lw=2, label=f"real: promedio de {len(curvas)}")
        t0s = lista[0][0] if sg > 0 else lista[1][0]
        ks = int(np.searchsorted(ts_sim, t0s))
        a.plot(tt_rel[:len(wm_sim[ks:ks + n])], wm_sim[ks:ks + n], color="#1d64c8", lw=2, label="simulada")
        a.plot(tt_rel, [sg * v if x < ms else 0 for x in tt_rel], "k--", lw=1, label="pedida")
        a.set_title(f"Escalones de {nombre} superpuestos")
        a.set_xlabel("tiempo desde el escalón [s]")
        a.set_ylabel("velocidad [°/s]")
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="center right")
    ax[1, 0].plot(t, w_ref, "k--", lw=0.8)
    ax[1, 0].plot(t, w_med, color="#c2410c", lw=0.7)
    ax[1, 0].set_title("Velocidad real, toda la prueba")
    ax[1, 0].set_xlabel("tiempo [s]")
    ax[1, 0].set_ylabel("°/s")
    ax[1, 0].grid(alpha=0.3)
    ax[1, 1].plot(t, pos, color="#0e7f74", lw=1)
    ax[1, 1].set_title("Posición de la base (deriva)")
    ax[1, 1].set_xlabel("tiempo [s]")
    ax[1, 1].set_ylabel("°")
    ax[1, 1].grid(alpha=0.3)
    fig.suptitle(f"J1 · PI de velocidad: Kp = {kp:.3f}, Ki = {ki:.2f}")
    fig.tight_layout()
    png = archivo.with_suffix(".png")
    fig.savefig(png, dpi=110)
    print(f"Gráfica: {png}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
