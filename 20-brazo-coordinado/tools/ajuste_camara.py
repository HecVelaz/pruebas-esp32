#!/usr/bin/env python3
"""Calibración cámara -> brazo: ajusta la transformación rígida entre los pares de puntos y da los 6 argumentos de
recolector_de_frutas/ros2_ws/src/mision/launch/mision.launch.py (TF brazo_base -> stereo_camera_link).

Entradas (las escriben medir_limon.py y "anotar <punto>" de punta.py; se unen por la etiqueta del punto):
    resultados/camara_brazo_<fecha>_camara.csv   cara del limón, mm, marco óptico `camera` (x der., y abajo, z adelante)
    resultados/camara_brazo_<fecha>_brazo.csv    punta del brazo en el limón, mm, marco de cinematica.py
                                                  (origen en el eje de J1 a la altura de la tapa; x a la derecha del
                                                  robot, y adelante, z arriba)
Pasos:
  1. Cara -> centro: corre cada punto de la cámara `radio` mm sobre su rayo (lo mismo que hace la misión con
     centro_desde_superficie y radio_limon), así la TF queda geométrica y la misión no suma el radio dos veces.
  2. Marco de cinematica.py -> brazo_base (ROS: x adelante, y a la izquierda, z arriba): (x, y, z) -> (y, -x, z).
  3. Kabsch / SVD: brazo = R · cámara + t (mínimos cuadrados, sin escala). Residuo por punto y dejando cada punto
     afuera del ajuste (lo que se espera con un limón nuevo).
  4. brazo_base -> stereo_camera_link = (brazo_base -> camera) · (stereo_camera_link -> camera)^-1, con la rotación
     fija de vision_estereo/launch/limon.launch.py (roll -90°, yaw -90°); se pasa a x y z [m] y roll pitch yaw [rad]
     (convención de static_transform_publisher: R = Rz(yaw) · Ry(pitch) · Rx(roll)).

Uso:
    python3 tools/ajuste_camara.py                         # los dos CSV más nuevos de resultados/
    python3 tools/ajuste_camara.py --camara A.csv --brazo B.csv [--radio 30] [--sin P3 P7]
    python3 tools/ajuste_camara.py --prueba                # datos sintéticos con una TF conocida (validación)
"""
import argparse
import math
from pathlib import Path

import numpy as np

RES = Path(__file__).absolute().parent.parent / "resultados"
# Lo medido a cinta el 12/11 (recolector_de_frutas/docs/ESTADO.md), marco de cinematica.py, mm: cámara de atrás
CAMARA_A_CINTA = (330.0, 195.0, 327.0)
RPY_LINK_CAMERA = (-math.pi / 2, 0.0, -math.pi / 2)  # stereo_camera_link -> camera (limon.launch.py)


def rot_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rpy_de(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(math.cos(pitch)) < 1e-6:  # gimbal lock: yaw = 0
        return math.atan2(-R[1, 2], R[1, 1]), pitch, 0.0
    return math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0])


def kabsch(A, B):
    """R, t con B ~ R · A + t (filas = puntos)."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cb - R @ ca


def cara_a_centro(P, radio):
    d = np.linalg.norm(P, axis=1, keepdims=True)
    return P * (1.0 + radio / d)


def cin_a_brazo_base(P):
    return np.column_stack([P[:, 1], -P[:, 0], P[:, 2]])


def leer(ruta):
    filas = {}
    lineas = open(ruta).read().strip().splitlines()
    cab = lineas[0].split(",")
    ix = [cab.index(c) for c in ("x_mm", "y_mm", "z_mm")]
    for l in lineas[1:]:
        v = l.split(",")
        filas[v[0]] = np.array([float(v[i]) for i in ix])  # si se repite una etiqueta, vale la última
    return filas


def ultimo(sufijo):
    a = sorted(RES.glob(f"camara_brazo_*_{sufijo}.csv"))
    if not a:
        raise SystemExit(f"no hay resultados/camara_brazo_*_{sufijo}.csv")
    return a[-1]


def ajustar(cam, brz, radio, nombres, informar=True):
    C = cara_a_centro(cam, radio)
    B = cin_a_brazo_base(brz)
    R, t = kabsch(C, B)
    res = np.linalg.norm((C @ R.T + t) - B, axis=1)
    loo = []
    if len(C) >= 4:
        for i in range(len(C)):
            m = np.arange(len(C)) != i
            Ri, ti = kabsch(C[m], B[m])
            loo.append(np.linalg.norm(Ri @ C[i] + ti - B[i]))
    if informar:
        print(f"\nradio {radio:.0f} mm, {len(C)} puntos: residuo medio {res.mean():.1f} mm, RMS "
              f"{math.sqrt((res ** 2).mean()):.1f}, máx {res.max():.1f}"
              + (f"; dejando cada uno afuera: medio {np.mean(loo):.1f}, máx {np.max(loo):.1f} mm" if loo else ""))
        print("  punto     residuo   afuera    (brazo_base, mm: medido por el brazo -> predicho por la cámara)")
        for i, nm in enumerate(nombres):
            pr = R @ C[i] + t
            print(f"  {nm:8s} {res[i]:6.1f}   {loo[i] if loo else float('nan'):6.1f}    "
                  f"({B[i][0]:5.0f} {B[i][1]:5.0f} {B[i][2]:5.0f}) -> ({pr[0]:5.0f} {pr[1]:5.0f} {pr[2]:5.0f})")
    return R, t, res, loo


def tf_link(R_bc, t_bc):
    R_bs = R_bc @ rot_rpy(*RPY_LINK_CAMERA).T
    return t_bc / 1000.0, rpy_de(R_bs), R_bs


def informar_tf(R, t):
    (x, y, z), (ro, pi, ya), R_bs = tf_link(R, t)
    a = cin_a_brazo_base(np.array([CAMARA_A_CINTA]))[0]
    print(f"\nTF brazo_base -> stereo_camera_link (cámara izquierda = la de atrás):")
    print(f"  x {x:+.4f}  y {y:+.4f}  z {z:+.4f} m     (a cinta: {a[0] / 1000:+.3f} {a[1] / 1000:+.3f} "
          f"{a[2] / 1000:+.3f}; diferencia {np.linalg.norm(t - a):.0f} mm)")
    print(f"  roll {ro:+.4f}  pitch {pi:+.4f}  yaw {ya:+.4f} rad   ({math.degrees(ro):+.1f}° "
          f"{math.degrees(pi):+.1f}° {math.degrees(ya):+.1f}°)")
    eje = R_bs[:, 0]  # hacia dónde mira la cámara (x de stereo_camera_link) en brazo_base
    print(f"  mira hacia ({eje[0]:+.2f} {eje[1]:+.2f} {eje[2]:+.2f}) en brazo_base (izquierda = +y; "
          f"inclinación {math.degrees(math.asin(max(-1, min(1, eje[2])))):+.1f}°)")
    print(f"\n  ros2 launch mision mision.launch.py x:={x:.4f} y:={y:.4f} z:={z:.4f} "
          f"roll:={ro:.4f} pitch:={pi:.4f} yaw:={ya:.4f}")


def prueba():
    rng = np.random.default_rng(1)
    # TF verdadera: cámara a cinta, mirando a +y de brazo_base, cabeza abajo (roll 180°), 5° hacia abajo
    R_bs = rot_rpy(math.pi, math.radians(5), math.pi / 2)
    t_bc = cin_a_brazo_base(np.array([CAMARA_A_CINTA]))[0]
    R_bc = R_bs @ rot_rpy(*RPY_LINK_CAMERA)
    brazo_base = np.column_stack([rng.uniform(150, 450, 8), rng.uniform(-100, 300, 8) + 300, rng.uniform(50, 350, 8)])
    cin = np.column_stack([-brazo_base[:, 1], brazo_base[:, 0], brazo_base[:, 2]])  # inversa de cin_a_brazo_base
    centro_cam = (brazo_base - t_bc) @ R_bc  # = R_bc^T (p - t)
    cara = cara_a_centro(centro_cam, -30.0)  # la cámara ve la cara, 30 mm antes del centro
    cara += rng.normal(0, 5, cara.shape)
    R, t, res, _ = ajustar(cara, cin, 30.0, [f"S{i}" for i in range(8)])
    informar_tf(R, t)
    print(f"\n  esperado: x {t_bc[0] / 1000:+.4f} y {t_bc[1] / 1000:+.4f} z {t_bc[2] / 1000:+.4f}; "
          f"roll/pitch/yaw {tuple(round(v, 4) for v in rpy_de(R_bs))}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--camara", type=Path)
    ap.add_argument("--brazo", type=Path)
    ap.add_argument("--radio", type=float, default=30.0, help="radio del limón, mm (= radio_limon de la misión)")
    ap.add_argument("--sin", nargs="*", default=[], help="puntos a dejar afuera")
    ap.add_argument("--prueba", action="store_true")
    a = ap.parse_args()
    if a.prueba:
        prueba()
        return
    rc, rb = a.camara or ultimo("camara"), a.brazo or ultimo("brazo")
    cam, brz = leer(rc), leer(rb)
    nombres = [k for k in cam if k in brz and k not in a.sin]
    solo = sorted((set(cam) ^ set(brz)) - set(a.sin))
    print(f"{rc.name} + {rb.name}: {len(nombres)} pares" + (f" (sin pareja: {', '.join(solo)})" if solo else ""))
    if len(nombres) < 3:
        raise SystemExit("hacen falta al menos 3 pares no alineados (mejor 8-10, repartidos en todo el volumen)")
    C = np.array([cam[k] for k in nombres])
    B = np.array([brz[k] for k in nombres])
    ajustar(C, B, 0.0, nombres)  # referencia: sin pasar de la cara al centro
    R, t, _, _ = ajustar(C, B, a.radio, nombres)
    informar_tf(R, t)


if __name__ == "__main__":
    main()
