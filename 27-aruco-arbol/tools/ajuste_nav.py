#!/usr/bin/env python3
"""Calibración de la cámara para la NAVEGACIÓN: ajusta la pose real de la cámara con los puntos de medir_marca.py.

Pares (marca vista por la cámara, marco óptico `camera`) <-> (marca medida con cinta, marco de cinematica.py: origen
en el eje de J1 a la altura de la tapa, x a la derecha del robot, y adelante, z arriba; z = h - 230 mm, la tapa a
23 cm del piso). Kabsch / SVD (rígida, sin escala; ajuste_camara.kabsch), residuo por punto y dejando cada uno afuera.
Escribe resultados/tf_navegacion.json (R y t: robot = R · cámara + t, mm) que usa ver_marcas.py --tf nav.

Uso:  python3 tools/ajuste_nav.py [--csv resultados/marca_nav_<fecha>.csv] [--sin P3]
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI.parent.parent / "20-brazo-coordinado" / "tools"))  # reutilizar, no copiar
import ajuste_camara as ajc  # noqa: E402

RES = AQUI.parent / "resultados"
TAPA_PISO = 230.0
SALIDA = RES / "tf_navegacion.json"


def leer(ruta):
    filas = {}
    lineas = Path(ruta).read_text().strip().splitlines()
    cab = lineas[0].split(",")
    for l in lineas[1:]:
        v = dict(zip(cab, l.split(",")))
        filas[v["punto"]] = (np.array([float(v["cam_x"]), float(v["cam_y"]), float(v["cam_z"])]),
                             np.array([float(v["x_mm"]), float(v["y_mm"]), float(v["h_mm"]) - TAPA_PISO]))
    return filas   # si se repite un punto, vale el último


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv", type=Path)
    ap.add_argument("--sin", nargs="*", default=[])
    a = ap.parse_args()
    ruta = a.csv or sorted(RES.glob("marca_nav_*.csv"))[-1]
    d = {k: v for k, v in leer(ruta).items() if k not in a.sin}
    nombres = list(d)
    if len(nombres) < 3:
        raise SystemExit(f"{ruta.name}: {len(nombres)} puntos; hacen falta al menos 3 (mejor 6, a 1-2 m)")
    C = np.array([d[k][0] for k in nombres])
    B = np.array([d[k][1] for k in nombres])
    R, t = ajc.kabsch(C, B)
    res = np.linalg.norm(C @ R.T + t - B, axis=1)
    loo = []
    if len(nombres) >= 4:
        for i in range(len(nombres)):
            m = np.arange(len(nombres)) != i
            Ri, ti = ajc.kabsch(C[m], B[m])
            loo.append(np.linalg.norm(Ri @ C[i] + ti - B[i]))
    print(f"{ruta.name}: {len(nombres)} puntos; residuo medio {res.mean():.0f} mm, máx {res.max():.0f}"
          + (f"; dejando cada uno afuera: medio {np.mean(loo):.0f}, máx {np.max(loo):.0f} mm" if loo else ""))
    print("  punto   residuo  afuera   cinta (x, y, z) -> cámara ajustada")
    for i, k in enumerate(nombres):
        q = R @ C[i] + t
        print(f"  {k:6s} {res[i]:6.0f}  {loo[i] if loo else float('nan'):6.0f}   ({B[i][0]:6.0f} {B[i][1]:5.0f} "
              f"{B[i][2]:5.0f}) -> ({q[0]:6.0f} {q[1]:5.0f} {q[2]:5.0f})")
    eje = R[:, 2]   # hacia dónde mira la cámara (z óptico) en el marco del robot
    abajo = math.degrees(math.asin(-eje[2]))
    rumbo = math.degrees(math.atan2(eje[1], -eje[0]))   # 0 = a la izquierda, + hacia adelante (como J1)
    giro = math.degrees(math.atan2(R[2, 0], -R[2, 1]))   # giro de la imagen alrededor del eje óptico
    print(f"\ncámara en x {t[0]:.0f}  y {t[1]:.0f}  z {t[2]:.0f} mm (a {(t[2] + TAPA_PISO) / 10:.1f} cm del piso); "
          f"mira a {rumbo:+.1f}° (0 = izquierda), {abajo:+.1f}° hacia abajo, imagen girada {giro:+.1f}°")
    SALIDA.write_text(json.dumps({"R": R.tolist(), "t": t.tolist(), "csv": ruta.name, "puntos": nombres,
                                  "residuo_medio_mm": float(res.mean()),
                                  "afuera_medio_mm": float(np.mean(loo)) if loo else None}, indent=1) + "\n")
    print(f"-> {SALIDA.name}")


if __name__ == "__main__":
    main()
