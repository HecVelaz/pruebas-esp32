#!/usr/bin/env python3
"""Calibración de la cámara para la NAVEGACIÓN, lado de la medición (en la Pi, con start_limon.sh corriendo).

La calibración cámara -> brazo del 12/11 sirve para el agarre (40-65 cm, compensa el modelo de la punta), pero a 1-2 m
se desvía 10-15 cm (13/11: la marca a 1 m de J1 salía a 89-91 cm y 15 cm más alta). Esta mide la pose de la marca en
el marco óptico de la cámara (solvePnP) y la guarda junto con lo medido con cinta; ajuste_nav.py ajusta con eso la
posición y la orientación reales de la cámara.

Uso:  python3 tools/medir_marca.py <punto> <x> <y> <h> [--id 0] [--seg 5] [--lado 150]
  punto: etiqueta (P1, P2, ...)
  x: de costado, del eje de J1 a la CARA de la marca, mm, NEGATIVO a la izquierda del robot (1 m -> -1000)
  y: a lo largo del robot, del eje de J1 al CENTRO de la marca, mm, + hacia adelante
  h: altura del CENTRO de la marca sobre el piso, mm
  (la marca parada, vertical y de frente al robot). Guarda la mediana de --seg s en
  resultados/marca_nav_<fecha>.csv (un archivo por día: se agregan los puntos).
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI))
from ver_marcas import a_gris  # noqa: E402

RES = AQUI.parent / "resultados"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("punto")
    ap.add_argument("x", type=float)
    ap.add_argument("y", type=float)
    ap.add_argument("h", type=float)
    ap.add_argument("--id", type=int, default=None, help="ID de la marca (def. la única que se vea)")
    ap.add_argument("--seg", type=float, default=5.0)
    ap.add_argument("--lado", type=float, default=150.0)
    a = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node("medir_marca")
    est = {"K": None, "t": [], "n": [], "px": [], "imgs": 0, "ids": set()}
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    par = cv2.aruco.DetectorParameters_create()
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    h = a.lado / 2
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)

    def al_info(m):
        est["K"] = np.array(m.p, float).reshape(3, 4)[:, :3].copy()

    def al_img(m):
        if est["K"] is None:
            return
        est["imgs"] += 1
        esq, ids, _ = cv2.aruco.detectMarkers(a_gris(m), dic, parameters=par)
        if ids is None:
            return
        for c, i in zip(esq, ids.ravel()):
            est["ids"].add(int(i))
            if a.id is not None and i != a.id:
                continue
            ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), est["K"], None,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                est["t"].append(tvec.ravel())
                est["n"].append(cv2.Rodrigues(rvec)[0][:, 2])
                est["px"].append(cv2.arcLength(c.reshape(4, 2), True) / 4)

    n.create_subscription(CameraInfo, "/left/camera_info", al_info, 1)
    n.create_subscription(Image, "/left/image_rect", al_img, 1)
    fin = time.time() + a.seg + 2.0   # 2 s para conectarse
    try:
        while time.time() < fin:
            rclpy.spin_once(n, timeout_sec=0.1)
    finally:
        n.destroy_node()
        rclpy.try_shutdown()
    if a.id is None and len(est["ids"]) > 1:
        raise SystemExit(f"se ven varias marcas {sorted(est['ids'])}: elegir una con --id")
    if len(est["t"]) < 5:
        raise SystemExit(f"{a.punto}: solo {len(est['t'])} detecciones en {est['imgs']} imágenes: NO se guarda")
    T = np.array(est["t"])
    med = np.median(T, axis=0)
    nor = np.median(np.array(est["n"]), axis=0)
    disp = float(np.median(np.linalg.norm(T - med, axis=1)))
    RES.mkdir(exist_ok=True)
    arch = RES / f"marca_nav_{time.strftime('%Y%m%d')}.csv"
    nuevo = not arch.exists()
    with open(arch, "a") as f:
        if nuevo:
            f.write("punto,x_mm,y_mm,h_mm,cam_x,cam_y,cam_z,n_x,n_y,n_z,lado_px,detecciones,dispersion_mm,hora\n")
        f.write(f"{a.punto},{a.x:.0f},{a.y:.0f},{a.h:.0f},{med[0]:.1f},{med[1]:.1f},{med[2]:.1f},{nor[0]:.3f},"
                f"{nor[1]:.3f},{nor[2]:.3f},{np.median(est['px']):.1f},{len(T)},{disp:.1f},"
                f"{time.strftime('%H:%M:%S')}\n")
    print(f"{a.punto}: cámara -> marca ({med[0]:.0f}, {med[1]:.0f}, {med[2]:.0f}) mm, dist {np.linalg.norm(med):.0f} mm, "
          f"{np.median(est['px']):.0f} px, {len(T)} detecciones, dispersión {disp:.1f} mm -> {arch.name}")


if __name__ == "__main__":
    main()
