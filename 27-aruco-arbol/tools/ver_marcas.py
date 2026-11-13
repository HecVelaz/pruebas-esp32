#!/usr/bin/env python3
"""Detector de prueba de las marcas ArUco del arbolito (en la Pi, con la visión corriendo: start_limon.sh).

Lee la imagen rectificada de la cámara izquierda (/left/image_rect y su /left/camera_info), detecta las marcas
DICT_4X4_50 y con solvePnP (lado --lado mm, IPPE_SQUARE; la imagen rectificada no tiene distorsión: K = P[:3, :3])
calcula la pose de cada una. La pasa al marco del brazo con la calibración cámara -> brazo del 12/11 (la misma que usa
"agarrar": ajuste_camara.py), marco de cinematica.py: origen en el eje de J1 a la altura de la tapa, x a la derecha
del robot, y adelante, z arriba (mm). Una línea por marca y por segundo:
    ID 0: dist 1.51 m | rumbo +16° | marca x -1182 y 330 z -22 mm | mira a 357° | 67 px
  dist  = distancia de la cámara al centro de la marca
  rumbo = desde J1, 0° = a la izquierda del robot (adonde mira la cámara), + hacia adelante
  mira a = hacia dónde apunta la cara de la marca, en el plano (0° = hacia +x: de frente a un robot que la tiene a su izquierda)
Cada --foto s guarda la última imagen con las marcas dibujadas en ~/logs/aruco_ultima.jpg (para mirarla).

Uso (Pi):  source /opt/ros/jazzy/setup.bash; cd ~/scripts/27-aruco-arbol; python3 tools/ver_marcas.py [--lado 150]
Si se guarda con --csv, agrega cada medición a resultados/aruco_<fecha>.csv (para compararla contra cinta).
Con --radio-maceta (mm, del eje del tronco a la cara de la marca) además estima el eje del árbol (la marca corrida
ese radio hacia adentro) y cuánto le falta al robot para la PARADA de 26-sim-alcance-arbolito: el eje del árbol a
--parada D Y mm (def. 700 a la izquierda de J1 y 150 adelante), con la marca de frente (mira a 0°).
"""
import argparse
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI.parent.parent / "20-brazo-coordinado" / "tools"))  # reutilizar, no copiar
import ajuste_camara as ajc  # noqa: E402
import punta as p  # noqa: E402

RES = AQUI.parent / "resultados"
FOTO = Path.home() / "logs" / "aruco_ultima.jpg"


def a_gris(msg):
    """sensor_msgs/Image -> imagen de grises (numpy), sin cv_bridge."""
    canales = {"mono8": 1, "bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}
    if msg.encoding in ("yuv422", "yuv422_yuy2", "yuyv"):
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 2, 2)[:, :msg.width]
        return np.ascontiguousarray(img[:, :, 0])
    n = canales.get(msg.encoding)
    if n is None:
        raise ValueError(f"encoding {msg.encoding} no soportado")
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * n]
    if n == 1:
        return np.ascontiguousarray(img)
    img = img.reshape(msg.height, msg.width, n)
    cod = {"bgr8": cv2.COLOR_BGR2GRAY, "rgb8": cv2.COLOR_RGB2GRAY, "bgra8": cv2.COLOR_BGRA2GRAY,
           "rgba8": cv2.COLOR_RGBA2GRAY}[msg.encoding]
    return cv2.cvtColor(np.ascontiguousarray(img), cod)


class VerMarcas(Node):
    def __init__(self, a):
        super().__init__("ver_marcas")
        self.a = a
        self.K = None
        self.dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.par = cv2.aruco.DetectorParameters_create()
        self.par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        h = a.lado / 2
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)  # orden de ArUco
        R, t, _, _ = ajc.cargar_tf(p.CAL_CAMARA, p.CAL_BRAZO, p.RADIO_LIMON, p.CAL_SIN)
        self.R, self.t = R, t
        self.t_info = self.t_foto = 0.0
        self.n_img = 0
        self.csv = None
        if a.csv:
            self.csv = RES / f"aruco_{time.strftime('%Y%m%d_%H%M%S')}.csv"
            self.csv.write_text("hora,id,dist_mm,rumbo_deg,x_mm,y_mm,z_mm,mira_deg,lado_px\n")
        self.create_subscription(CameraInfo, "/left/camera_info", self.al_info, 1)
        self.create_subscription(Image, "/left/image_rect", self.al_img, 1)
        self.get_logger().info(f"esperando /left/image_rect (marca de {a.lado:.0f} mm)"
                               + (f"; CSV {self.csv.name}" if self.csv else ""))

    def al_info(self, m):
        P = np.array(m.p, float).reshape(3, 4)
        self.K = P[:, :3].copy()   # imagen rectificada: sin distorsión

    def a_brazo(self, v_cam):
        """Vector del marco óptico `camera` (mm) -> marco de cinematica.py; como punto si es posición."""
        bb = self.R @ v_cam
        return np.array([-bb[1], bb[0], bb[2]])

    def al_img(self, m):
        if self.K is None:
            return
        self.n_img += 1
        gris = a_gris(m)
        esq, ids, _ = cv2.aruco.detectMarkers(gris, self.dic, parameters=self.par)
        ahora = time.time()
        informar = ahora - self.t_info >= 1.0
        if informar:
            self.t_info = ahora
            if ids is None:
                print(f"[{time.strftime('%H:%M:%S')}] sin marcas ({self.n_img} imágenes)", flush=True)
        if ids is not None and informar:
            for c, i in sorted(zip(esq, ids.ravel()), key=lambda e: e[1]):
                ok, rvec, tvec = cv2.solvePnP(self.obj, c.reshape(4, 2).astype(np.float64), self.K, None,
                                              flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                tc = tvec.ravel()
                Rm, _ = cv2.Rodrigues(rvec)
                pos = self.a_brazo(tc) + np.array([-self.t[1], self.t[0], self.t[2]])   # punto: + traslación
                normal = self.a_brazo(Rm[:, 2])    # z de la marca: sale de la cara impresa hacia la cámara
                rumbo = math.degrees(math.atan2(pos[1], -pos[0]))
                mira = math.degrees(math.atan2(normal[1], normal[0])) % 360
                lado_px = cv2.arcLength(c.reshape(4, 2), True) / 4
                print(f"[{time.strftime('%H:%M:%S')}] ID {i}: dist {np.linalg.norm(tc) / 1000:.2f} m | rumbo "
                      f"{rumbo:+4.0f}° | marca x {pos[0]:6.0f} y {pos[1]:6.0f} z {pos[2]:5.0f} mm | mira a "
                      f"{mira:3.0f}° | {lado_px:.0f} px", flush=True)
                if self.a.radio_maceta is not None:
                    nh = np.array([normal[0], normal[1]])
                    nh /= max(np.linalg.norm(nh), 1e-9)
                    eje = pos[:2] - nh * self.a.radio_maceta          # la normal sale de la marca hacia afuera
                    objetivo = np.array([-self.a.parada[0], self.a.parada[1]])
                    falta = eje - objetivo                            # lo que el robot tiene que moverse (sin girar)
                    giro = (mira + 180) % 360 - 180
                    print(f"           eje del árbol x {eje[0]:6.0f} y {eje[1]:6.0f} mm (a {-eje[0] / 10:.0f} cm a la "
                          f"izquierda de J1, {eje[1] / 10:+.0f} cm adelante) -> para la parada {i + 1}: "
                          f"{'acercarse' if falta[0] < 0 else 'alejarse'} {abs(falta[0]) / 10:.0f} cm de costado, "
                          f"{'avanzar' if falta[1] > 0 else 'retroceder'} {abs(falta[1]) / 10:.0f} cm, "
                          f"girar {giro:+.0f}°", flush=True)
                if self.csv:
                    with open(self.csv, "a") as f:
                        f.write(f"{time.strftime('%H:%M:%S')},{i},{np.linalg.norm(tc):.0f},{rumbo:.1f},{pos[0]:.0f},"
                                f"{pos[1]:.0f},{pos[2]:.0f},{mira:.1f},{lado_px:.1f}\n")
        if ahora - self.t_foto >= self.a.foto:
            self.t_foto = ahora
            img = cv2.cvtColor(gris, cv2.COLOR_GRAY2BGR)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(img, esq, ids)
            cv2.line(img, (280, 0), (280, img.shape[0]), (0, 0, 255), 1)   # a la izquierda: tapa el brazo
            FOTO.parent.mkdir(exist_ok=True)
            cv2.imwrite(str(FOTO), img)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--lado", type=float, default=150.0, help="lado del cuadrado negro impreso (mm)")
    ap.add_argument("--foto", type=float, default=2.0, help="cada cuántos s guardar ~/logs/aruco_ultima.jpg")
    ap.add_argument("--csv", action="store_true", help="guardar las mediciones en resultados/aruco_<fecha>.csv")
    ap.add_argument("--radio-maceta", type=float, help="mm del eje del tronco a la cara de la marca")
    ap.add_argument("--parada", type=float, nargs=2, default=[700.0, 150.0], metavar=("D", "Y"),
                    help="eje del árbol en la parada: D mm a la izquierda de J1, Y mm adelante (def. 700 150)")
    a = ap.parse_args()
    rclpy.init()
    n = VerMarcas(a)
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
