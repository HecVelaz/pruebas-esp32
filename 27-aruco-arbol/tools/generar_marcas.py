#!/usr/bin/env python3
"""Marcas ArUco para localizar el arbolito: un PDF A4 con una marca por hoja, en tamaño real.

Diccionario DICT_4X4_50 (4 x 4 bits: celdas grandes, se detecta desde más lejos), IDs 0-3 = las 4 caras de la base del
arbolito (0 = parada 1, 1 = parada 2, ... en sentido antihorario visto desde arriba). Lado del cuadrado negro:
--lado mm (def. 150: ~50 px a 2 m y ~25 px a 4 m con la cámara del mástil, f = 672 px). Alrededor queda un margen
blanco de 2 celdas (hace falta para detectarla: no recortar al ras).

Uso:  python3 tools/generar_marcas.py [--lado 150] [--ids 0 1 2 3]
      -> resultados/marcas_aruco_4x4_<lado>mm.pdf   (imprimir al 100 %, "tamaño real", sin "ajustar a la página";
         después medir el cuadrado negro con regla: tiene que dar --lado mm)
"""
import argparse
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.backends.backend_pdf import PdfPages  # noqa: E402

RES = Path(__file__).absolute().parent.parent / "resultados"
A4 = (210.0, 297.0)  # mm
MM = 1 / 25.4


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lado", type=float, default=150.0, help="lado del cuadrado negro (mm)")
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3])
    a = ap.parse_args()
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    salida = RES / f"marcas_aruco_4x4_{a.lado:.0f}mm.pdf"
    with PdfPages(salida) as pdf:
        for i in a.ids:
            bits = cv2.aruco.drawMarker(dic, i, 6)  # 1 px por celda: 4 x 4 bits + borde negro = 6 x 6
            fig = plt.figure(figsize=(A4[0] * MM, A4[1] * MM))
            ax = fig.add_axes([0, 0, 1, 1])
            ax.set_xlim(0, A4[0])
            ax.set_ylim(0, A4[1])
            ax.axis("off")
            x0 = (A4[0] - a.lado) / 2
            y0 = (A4[1] - a.lado) / 2
            celda = a.lado / 6
            for f in range(6):          # celdas negras como rectángulos (vectorial: tamaño exacto al imprimir)
                for c in range(6):
                    if bits[f, c] < 128:
                        ax.add_patch(plt.Rectangle((x0 + c * celda, y0 + (5 - f) * celda), celda, celda,
                                                   fc="k", ec="k", lw=0.1))
            m = 2 * celda               # margen blanco que hace falta para detectarla
            for cx in (x0 - m, x0 + a.lado + m):
                for cy in (y0 - m, y0 + a.lado + m):
                    ax.plot([cx - 5, cx + 5], [cy, cy], color="0.6", lw=0.5)
                    ax.plot([cx, cx], [cy - 5, cy + 5], color="0.6", lw=0.5)
            ax.text(A4[0] / 2, y0 + a.lado + m + 8, f"ARRIBA  -  ID {i}  (parada {i + 1})", ha="center",
                    fontsize=16, weight="bold")
            ax.text(A4[0] / 2, y0 - m - 10, f"DICT_4X4_50 - cuadrado negro = {a.lado:.0f} mm (medir con regla) - "
                    f"recortar por las marcas grises (margen blanco de {m:.0f} mm)", ha="center", fontsize=8)
            pdf.savefig(fig)
            plt.close(fig)
    print(salida)


if __name__ == "__main__":
    main()
