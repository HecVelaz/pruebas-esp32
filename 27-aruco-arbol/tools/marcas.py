"""Marcas ArUco del arbolito: detección y pose en el marco del robot con la calibración de NAVEGACIÓN.

Marco de cinematica.py (mm): origen en el eje de J1 a la altura de la tapa, x a la derecha del robot, y adelante, z
arriba. Calibración: resultados/tf_navegacion.json (ajuste_nav.py, 13/11: ~3 cm a 1–2 m). La imagen rectificada no
tiene distorsión: K = P[:3, :3] de /left/camera_info.
"""
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo, Image

RES = Path(__file__).absolute().parent.parent / "resultados"
LADO = 150.0   # mm, cuadrado negro impreso (medido)


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


class Marcas:
    """Escucha la cámara izquierda rectificada en un nodo rclpy y junta las poses de las marcas en el marco del robot."""

    def __init__(self, nodo, lado=LADO):
        j = json.loads((RES / "tf_navegacion.json").read_text())
        self.R, self.t = np.array(j["R"]), np.array(j["t"])   # robot = R · cámara + t (mm)
        self.K = None
        self.dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.par = cv2.aruco.DetectorParameters_create()
        self.par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        h = lado / 2
        self.obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
        self.juntar = False
        self.vistas = []   # (id, pos xyz mm, ángulo de la normal en el plano, °)
        self.imgs = 0
        nodo.create_subscription(CameraInfo, "/left/camera_info", self._al_info, 1)
        nodo.create_subscription(Image, "/left/image_rect", self._al_img, 1)

    def _al_info(self, m):
        self.K = np.array(m.p, float).reshape(3, 4)[:, :3].copy()

    def _al_img(self, m):
        if self.K is None or not self.juntar:
            return
        self.imgs += 1
        esq, ids, _ = cv2.aruco.detectMarkers(a_gris(m), self.dic, parameters=self.par)
        if ids is None:
            return
        for c, i in zip(esq, ids.ravel()):
            ok, rvec, tvec = cv2.solvePnP(self.obj, c.reshape(4, 2).astype(np.float64), self.K, None,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            pos = self.R @ tvec.ravel() + self.t
            nor = self.R @ cv2.Rodrigues(rvec)[0][:, 2]   # sale de la cara impresa hacia la cámara
            self.vistas.append((int(i), pos, math.degrees(math.atan2(nor[1], nor[0]))))

    def medir(self, girar_ros, seg=3.0, id_=None, minimo=5):
        """Mediana de las vistas durante seg s: (id, pos mm, mira °, n) o None. girar_ros(seg) hace spin del nodo."""
        self.vistas, self.imgs, self.juntar = [], 0, True
        girar_ros(seg)
        self.juntar = False
        v = [x for x in self.vistas if id_ is None or x[0] == id_]
        if not v:
            return None
        ids = {x[0] for x in v}
        if len(ids) > 1:
            i = max(ids, key=lambda k: sum(1 for x in v if x[0] == k))   # la que más se vio
            v = [x for x in v if x[0] == i]
        if len(v) < minimo:
            return None
        pos = np.median(np.array([x[1] for x in v]), axis=0)
        # mediana circular simple: los ángulos están cerca entre sí
        a0 = v[0][2]
        mira = a0 + float(np.median([(x[2] - a0 + 180) % 360 - 180 for x in v]))
        return v[0][0], pos, (mira + 180) % 360 - 180, len(v)
