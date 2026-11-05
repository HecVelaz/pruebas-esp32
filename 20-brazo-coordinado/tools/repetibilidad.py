#!/usr/bin/env python3
"""Fase 6: repetibilidad articular estilo ISO 9283 (5 poses, N ciclos, siempre por el mismo camino).

Hace el homing ("h"), --calentar ciclos de calentamiento (no entran en la cuenta) y --ciclos ciclos por las poses en
orden; guarda cada LLEGO (los ángulos medidos por los encoders al quedar firmes) y al final vuelve a VOLVER de trayectoria.py ("mv 3 10 5").
Para cada pose y articulación calcula, sobre los ciclos medidos:
    media, desvío estándar, rango (máx - mín) y RP = l̄ + 3·S_l, con l = |q - q̄| (ISO 9283, en una dimensión)
y el error medio contra el objetivo (exactitud articular). Los encoders están en los motores: no ven el juego de los
engranajes ni de la correa; la repetibilidad en la punta necesita una medición de afuera (video o comparador).

Guarda resultados/repetibilidad_<fecha>_<hora>.csv (una fila por llegada; con _cortado si no terminó).

Antes: cerrar el monitor, brazo abajo y recogido, 12 V encendidos. Mirar el primer ciclo (el de calentamiento):
"mv" no esquiva la caja. Ctrl+C manda "s" (las tres se sostienen donde están) y guarda lo medido.

Uso:
    python3 tools/repetibilidad.py --ciclos 10                    # poses por defecto, 1 ciclo de calentamiento
    python3 tools/repetibilidad.py --ciclos 30 --calentar 3
    python3 tools/repetibilidad.py --poses "20 20 14" "40 30 20" "40 74 70" "60 74 70" "20 74 70" --ciclos 10
    python3 tools/repetibilidad.py --archivo resultados/repetibilidad_....csv   # recalcular, sin la placa
"""
import argparse
import csv
import statistics as st
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).absolute().parent))
from trayectoria import DIR_RESULTADOS, PUERTO, VOLVER, Captura, abrir  # noqa: E402

# Probadas en placa el 2026-11-03 (sin chocar la caja), y los caminos entre ellas: (20,20,24) <-> (40,30,20) es la
# lista fija; subir/bajar a 74/70 con J1 en 20 y en 40 como desde 3/3/3; 40 -> 60 -> 20 con J1 sola
# 2026-11-04 (manivela de 95 mm): "20 20 24" quedó fuera de la franja (con J2 en 20°, J3 hasta 15,5°) -> "20 20 14"
POSES = ["20 20 14", "40 30 20", "40 74 70", "60 74 70", "20 74 70"]
FIN_MV = ("LLEGO,", "MV_ERROR,", "Uso:", "Fuera de", "J2 y J3 fuera", "sin cero")
CAMPOS = ["ciclo", "pose", "obj1", "obj2", "obj3", "q1", "q2", "q3", "e1", "e2", "e3", "t_ms"]


def correr(args):
    poses = [[float(x) for x in p.split()] for p in args.poses]
    filas, cortado = [], None
    s = abrir(args.puerto)
    cap = Captura(s)
    try:
        cap.leer(2.0)  # si se reinició al abrir: el saludo y la ayuda
        s.reset_input_buffer()
        cap.mandar("h")
        r = cap.leer(240.0, ("HOMING,", "HOMING_ERROR,", "Homing: J2"))
        if r is None or not r.startswith("HOMING,"):
            raise RuntimeError(r or "el homing no terminó a tiempo")
        total = args.calentar + args.ciclos
        for c in range(1, total + 1):
            medido = c > args.calentar
            print(f"\n=== ciclo {c}/{total}" + ("" if medido else " (calentamiento)"))
            for k, p in enumerate(poses, 1):
                cmd = "mv " + " ".join(f"{x:g}" for x in p)
                cap.mandar(cmd)
                r = cap.leer(90.0, FIN_MV)
                if r is None or not r.startswith("LLEGO,"):
                    raise RuntimeError(r or f'"{cmd}" no terminó a tiempo')
                v = r.split(",")
                t_ms, q, e = int(v[1]), [float(x) for x in v[2:5]], [float(x) for x in v[5:8]]
                if medido:
                    filas.append([c - args.calentar, k] + p + q + e + [t_ms])
        print(f"<< {VOLVER} (vuelta)")
        cap.mandar(VOLVER)
        r = cap.leer(90.0, FIN_MV)
        if r is None or not r.startswith("LLEGO,"):
            print(f'\nNo volvió cerca del cero: {r or "no terminó a tiempo"}. "s" (se sostienen donde están).')
            s.write(b"s\n")
    except KeyboardInterrupt:
        cortado = "Ctrl+C"
        print('\nCtrl+C: "s" (las tres se sostienen donde están).')
        s.write(b"s\n")
    except RuntimeError as ex:
        cortado = str(ex)
        print(f'\nSe corta: {cortado}. "s" (las tres se sostienen donde están).')
        s.write(b"s\n")  # si el mv sigue en curso (venció la espera), que no siga solo
    finally:
        s.close()
    DIR_RESULTADOS.mkdir(exist_ok=True)
    archivo = DIR_RESULTADOS / f"repetibilidad_{datetime.now():%Y%m%d_%H%M%S}{'_cortado' if cortado else ''}.csv"
    with open(archivo, "w", newline="") as f:
        f.write(f"# 20-brazo-coordinado, repetibilidad: poses {' | '.join(args.poses)}; "
                f"{args.calentar} de calentamiento + {args.ciclos} ciclos\n")
        if cortado:
            f.write(f"# cortado: {cortado}\n")
        for t, tipo, texto in cap.eventos:
            if tipo == "cmd" or texto.startswith(("HOMING", "MV_ERROR", ">> mv CORTADO")):
                f.write(f"# {tipo},{t},{texto}\n")
        w = csv.writer(f)
        w.writerow(CAMPOS)
        w.writerows(filas)
    print(f"\nCSV: {archivo.relative_to(DIR_RESULTADOS.parent)} ({len(filas)} llegadas medidas)")
    return archivo


def cargar(archivo):
    with open(archivo) as f:
        lineas = [l for l in f if not l.startswith("#")]
    return [{k: float(v) for k, v in fila.items()} for fila in csv.DictReader(lineas)]


def rp(xs):
    """ISO 9283 en una dimensión: l = |x - x̄|, RP = l̄ + 3·S_l."""
    m = st.mean(xs)
    l = [abs(x - m) for x in xs]
    return st.mean(l) + 3 * (st.stdev(l) if len(l) > 1 else 0.0)


def analizar(archivo):
    filas = cargar(archivo)
    if not filas:
        print("Sin llegadas medidas.")
        return
    print(f"\n=== Repetibilidad articular de {Path(archivo).name} "
          f"({int(max(f['ciclo'] for f in filas))} ciclos) ===")
    print("Por pose y articulación (°): media medida, error medio contra el objetivo, desvío, rango y RP = l̄ + 3·S_l")
    peor = [0.0, 0.0, 0.0]
    for k in sorted({int(f["pose"]) for f in filas}):
        fp = [f for f in filas if int(f["pose"]) == k]
        obj = [fp[0][f"obj{i}"] for i in (1, 2, 3)]
        print(f"P{k} ({obj[0]:g}, {obj[1]:g}, {obj[2]:g}), {len(fp)} llegadas")
        for i in (1, 2, 3):
            q = [f[f"q{i}"] for f in fp]
            e = [f[f"e{i}"] for f in fp]
            r = rp(q)
            peor[i - 1] = max(peor[i - 1], r)
            de = st.stdev(q) if len(q) > 1 else 0.0
            print(f"  J{i}  media {st.mean(q):8.3f}  error {st.mean(e):+.3f}  desvío {de:.3f}  "
                  f"rango {max(q) - min(q):.3f}  RP {r:.3f}")
    print("Peor RP de cada articulación: " + "  ".join(f"J{i + 1} {peor[i]:.3f}°" for i in range(3)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--poses", nargs="+", default=POSES, help='"q1 q2 q3" de cada pose, en el orden del ciclo')
    ap.add_argument("--ciclos", type=int, default=10, help="ciclos medidos (def. 10; ISO: 30)")
    ap.add_argument("--calentar", type=int, default=1, help="ciclos de calentamiento, no medidos (def. 1)")
    ap.add_argument("--puerto", default=PUERTO)
    ap.add_argument("--archivo", help="recalcular un CSV guardado, sin la placa")
    args = ap.parse_args()
    archivo = args.archivo or correr(args)
    if archivo:
        analizar(archivo)


if __name__ == "__main__":
    main()
