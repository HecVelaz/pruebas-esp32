#!/usr/bin/env python3
"""Mapa de alcance del AGARRE (no solo de la punta): para cada posición del limón, ¿puede "agarrar" de punta.py?

Usa el mismo planificador que el brazo en la placa (20-brazo-coordinado/tools/punta.py: planear_agarre, desde el
reposo mv 3 17 4): entrar de frente desde 10 cm antes, cerrar, salir con 3° de margen a la franja y volver al reposo,
con los límites, la franja del cuatro barras y los choques con la caja, el mástil y el piso. El limón se agarra
AGARRAR_DZ (1 cm) más abajo, como en la placa.

La grilla es (dirección de J1, r, z) en el marco de cinematica.py (origen en el eje de J1 a la altura de la tapa):
  dirección = ángulo REAL de J1 (0 = a la izquierda del robot, 90 = al frente), r = distancia horizontal a J1 (mm),
  z = altura del centro del limón sobre la tapa (mm, en el modelo).
Los puntos que sirven tardan ~5 s (revisa todo el camino): se calcula una vez y se guarda en
resultados/mapa_alcance_<fecha>.csv (simular.py lee el más nuevo).

Uso:  python3 tools/alcance.py [--dirs 5 25 45] [--paso-r 25] [--paso-z 20] [--procesos 4]
"""
import argparse
import math
import sys
import time
from multiprocessing import Pool
from pathlib import Path

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI.parent.parent / "20-brazo-coordinado" / "tools"))  # reutilizar, no copiar
import cinematica as k  # noqa: E402
import punta as p  # noqa: E402

RES = AQUI.parent / "resultados"
REPOSO = (3.0, 17.0, 4.0)


def xy(ang_real, r):
    """Ángulo real de J1 y r -> (x, y) del marco de cinematica.py (0° = -x, 90° = +y)."""
    a = math.radians(ang_real)
    return -r * math.cos(a), r * math.sin(a)


def uno(args):
    ang, r, z = args
    x, y = xy(ang, r)
    pasos, mot = p.planear_agarre(REPOSO, (x, y, z + p.AGARRAR_DZ))
    return ang, r, z, int(pasos is not None), ("" if pasos else mot[0].replace(",", ";")[:150])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dirs", type=float, nargs="+", default=[5.0, 25.0, 45.0], help="ángulos reales de J1 (°)")
    ap.add_argument("--r", type=float, nargs=2, default=[375.0, 800.0], metavar=("MIN", "MAX"))
    ap.add_argument("--z", type=float, nargs=2, default=[-40.0, 300.0], metavar=("MIN", "MAX"))
    ap.add_argument("--paso-r", type=float, default=25.0)
    ap.add_argument("--paso-z", type=float, default=20.0)
    ap.add_argument("--procesos", type=int, default=4)
    a = ap.parse_args()
    rs = [a.r[0] + i * a.paso_r for i in range(int((a.r[1] - a.r[0]) / a.paso_r) + 1)]
    zs = [a.z[0] + i * a.paso_z for i in range(int((a.z[1] - a.z[0]) / a.paso_z) + 1)]
    tareas = [(d, r, z) for d in a.dirs for z in zs for r in rs]
    salida = RES / f"mapa_alcance_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    print(f"{len(tareas)} puntos ({len(a.dirs)} direcciones x {len(rs)} r x {len(zs)} z) -> {salida.name}", flush=True)
    t0 = time.time()
    with Pool(a.procesos) as pool, open(salida, "w") as f:
        f.write("ang_real_j1,r_mm,z_mm,agarra,motivo\n")
        for i, (d, r, z, ok, m) in enumerate(pool.imap_unordered(uno, tareas, chunksize=2), 1):
            f.write(f"{d:.1f},{r:.0f},{z:.0f},{ok},{m}\n")
            if i % 50 == 0:
                print(f"  {i}/{len(tareas)}  ({time.time() - t0:.0f} s)", flush=True)
    print(f"listo en {time.time() - t0:.0f} s: {salida}")


if __name__ == "__main__":
    main()
