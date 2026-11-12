#!/usr/bin/env python3
"""Calibración cámara -> brazo, lado de la visión: mide /limon/posicion en cada punto (corre en la Pi).

Con vision_estereo/scripts/start_limon.sh corriendo, el limón quieto en su soporte y el brazo quieto y fuera de la
vista. Sesión: se escribe la etiqueta del punto (la misma que después "anotar <punto>" en punta.py) y Enter; junta
/limon/posicion durante N s y guarda la mediana en resultados/camara_brazo_<fecha>_camara.csv (mm, marco óptico de
la cámara izquierda rectificada, `camera`: x a la derecha de la imagen, y abajo, z hacia adelante). Es la CARA del
limón que mira a las cámaras, no su centro: el ajuste (ajuste_camara.py) la corre el radio sobre el rayo, igual que
la misión (centro_desde_superficie).

Uso:  python3 tools/medir_limon.py [segundos=10]       ("salir" o Ctrl+D para terminar)
"""
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

ARCHIVO = (Path(__file__).absolute().parent.parent / "resultados"
           / f"camara_brazo_{time.strftime('%Y%m%d_%H%M%S')}_camara.csv")


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    n = Node("medir_limon")
    pos, dets, marcos = [], [0], set()

    def al_pos(m):
        pos.append((m.point.x, m.point.y, m.point.z))
        marcos.add(m.header.frame_id)

    def al_det(m):
        dets[0] += 1

    n.create_subscription(PointStamped, "/limon/posicion", al_pos, 10)
    n.create_subscription(Detection2DArray, "/yolo/detecciones", al_det, 10)
    print(f"[medir_limon: <punto> + Enter mide {dur:.0f} s; \"salir\" termina. Archivo: {ARCHIVO.name}]")
    try:
        while True:
            try:
                l = input("punto> ").strip()
            except EOFError:
                break
            if not l:
                continue
            if l == "salir":
                break
            if len(l.split()) != 1 or "," in l:
                print("una sola palabra, sin comas (p. ej. P1)")
                continue
            pos.clear()
            dets[0] = 0
            fin = time.time() + dur
            while time.time() < fin:
                rclpy.spin_once(n, timeout_sec=0.1)
            if len(pos) < 3:
                print(f"  {l}: solo {len(pos)} posiciones en {dets[0]} imágenes: NO se guarda (¿se ve el limón?, "
                      f"¿está en la zona con disparidad, u >= ~280 px y 0,37-0,96 m?)")
                continue
            p = np.array(pos) * 1000.0
            med = np.median(p, axis=0)
            disp = np.median(np.linalg.norm(p - med, axis=1))
            nuevo = not ARCHIVO.exists()
            with open(ARCHIVO, "a") as f:
                if nuevo:
                    f.write("punto,x_mm,y_mm,z_mm,n,imagenes,dispersion_mm,marco\n")
                f.write(f"{l},{med[0]:.1f},{med[1]:.1f},{med[2]:.1f},{len(p)},{dets[0]},{disp:.1f},"
                        f"{'|'.join(sorted(marcos))}\n")
            aviso = "  <- dispersión alta: repetir" if disp > 10 else ""
            print(f"  {l}: x {med[0]:+.0f}  y {med[1]:+.0f}  z {med[2]:.0f} mm  ({len(p)} de {dets[0]} imágenes, "
                  f"dispersión {disp:.1f} mm){aviso}")
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
