#!/usr/bin/env python3
"""Cinemática directa e inversa de la punta (centro de la pinza) de 20, con el modelo MEDIDO (2026-11-05).

No es la cinemática ideal de la E4: J2 y J3 tienen juego y J2 gira 1,170° reales por grado del encoder, así que los
ángulos reales del brazo y del antebrazo se ajustan a las 12 poses del video de calibración
(resultados/calibracion_punta_20261105_164644.csv):
    brazo     = lineal en (q2, q3)             (residuos <= 1,7°)
    antebrazo = cuadrático en (q2, q3)
    pinza     = muñeca + (178,8 mm afuera, 29,4 mm arriba)   (la pinza queda siempre horizontal)
Error de la punta dejando cada pose afuera del ajuste: ~9 mm de media, hasta ~3 cm en los bordes. Fuera de la zona
medida (q2 20-78°, q3 5-81°) el modelo extrapola: lo avisa.

Marco del robot (mm), visto desde adelante (el frente es el de los ultrasonidos):
    origen en el eje de J1, a la altura de la tapa de la caja
    x hacia la derecha, y hacia adelante, z hacia arriba
    q1 = 0 apunta a +x (a la derecha); q1 + gira hacia adelante (90° = hacia el frente, sobre la caja)
Geometría de ~/Imágenes/brazo/medidas.jpeg (a ojo): caja 400 x 400 x 230 mm, J1 a 90 mm del borde derecho y 315 mm del
frente; hombro (J2) 175 mm sobre la tapa (medido), sobre el eje de J1 y centrado; mástil Ø 50 a 300 mm a
la izquierda y 240 mm adelante de J1.

Uso:
    python3 tools/cinematica.py directa 3 50 45            # q1 q2 q3 (grados del encoder) -> x y z de la punta
    python3 tools/cinematica.py inversa 600 100 250        # x y z (mm) -> q1 q2 q3 (y el comando mv)
    python3 tools/cinematica.py prueba                     # validación: dejando cada pose afuera, ida y vuelta
"""
import argparse
import math
from pathlib import Path

import numpy as np

CSV = Path(__file__).absolute().parent.parent / "resultados" / "calibracion_punta_20261105_164644.csv"

# Geometría (mm)
L2 = 350.22            # hombro -> codo (escala del video)
H_HOMBRO = 175.0       # eje de J2 sobre la tapa (medido por el usuario el 2026-11-05; antes 180 supuesto)
CAJA_X = (-310.0, 90.0)    # la tapa, en x (izquierda, derecha) desde J1
CAJA_Y = (-85.0, 315.0)    # en y (atrás, adelante)
CAJA_ALTO = 230.0
MASTIL = (-300.0, 240.0, 25.0)   # x, y, radio
MARGEN = 30.0          # mm de aire a la caja, al mástil y al piso

# Límites (grados del encoder, como el firmware: include/config.h, con MARGEN_LIMITE 3° y MARGEN_FRANJA 1°)
Q1_LIM = (3.0, 115.0)
Q2_LIM = (3.0, 80.0)
Q3_LIM = (3.0, 98.5)
Q3_MENOS_Q2_MIN = -25.0     # más abajo J3 no queda firme (punto muerto del cuatro barras, 05/11)
# 2026-11-10 (doble helicoidales): medida de nuevo, resultados/limites_20261110_145545.csv
FRANJA_ARRIBA = [(0, 0.5), (9.9, 3.0), (15.1, 4.0), (30.1, 16.1), (45.1, 31.3), (60.2, 52.0), (74.6, 75.1), (80.1, 83.4)]
MARGEN_FRANJA = 1.0
ZONA_MEDIDA = ((19.0, 79.0), (4.0, 82.0))   # q2, q3 de las poses de calibración


def _cols(q2, q3, grado):
    q2 = np.atleast_1d(np.asarray(q2, float))
    q3 = np.atleast_1d(np.asarray(q3, float))
    c = [np.ones_like(q2), q2, q3]
    if grado == 2:
        c += [q2 * q3, q3 ** 2, q2 ** 2]
    return np.column_stack(c)


class Modelo:
    """Ángulos reales del brazo y del antebrazo (desde la horizontal, hacia afuera) en función de q2, q3."""

    def __init__(self, filas=None):
        d = np.loadtxt(CSV, delimiter=",", comments="#", skiprows=5) if filas is None else filas
        q2, q3 = d[:, 0], d[:, 1]
        codo, muneca, pinza = d[:, 2:4], d[:, 4:6], d[:, 6:8]
        a2 = np.degrees(np.arctan2(codo[:, 1], codo[:, 0]))
        a3 = np.degrees(np.arctan2(muneca[:, 1] - codo[:, 1], muneca[:, 0] - codo[:, 0]))
        self.c2 = np.linalg.lstsq(_cols(q2, q3, 1), a2, rcond=None)[0]
        self.c3 = np.linalg.lstsq(_cols(q2, q3, 2), a3, rcond=None)[0]
        self.L3 = float(np.mean(np.hypot(*(muneca - codo).T)))
        self.pinza = (pinza - muneca).mean(0)

    def plano(self, q2, q3):
        """(q2, q3) -> punta (r afuera, z arriba) en mm desde el eje del hombro, y los puntos del codo y la muñeca."""
        a2 = math.radians((_cols(q2, q3, 1) @ self.c2).item())
        a3 = math.radians((_cols(q2, q3, 2) @ self.c3).item())
        codo = np.array([L2 * math.cos(a2), L2 * math.sin(a2)])
        muneca = codo + self.L3 * np.array([math.cos(a3), math.sin(a3)])
        return muneca + self.pinza, codo, muneca


M = None


def modelo():
    global M
    if M is None:
        M = Modelo()
    return M


def al_robot(q1, rz):
    """(r, z) del plano del brazo -> (x, y, z) del robot."""
    a = math.radians(q1)
    return np.array([rz[0] * math.cos(a), rz[0] * math.sin(a), H_HOMBRO + rz[1]])


def directa(q1, q2, q3):
    """Grados del encoder -> (x, y, z) de la punta en mm (y los puntos del brazo para revisar choques)."""
    p, codo, muneca = modelo().plano(q2, q3)
    return al_robot(q1, p), [al_robot(q1, (0.0, 0.0)), al_robot(q1, codo), al_robot(q1, muneca), al_robot(q1, p)]


def q3_max_franja(q2):
    return float(np.interp(q2, [a for a, _ in FRANJA_ARRIBA], [b for _, b in FRANJA_ARRIBA]))


def problemas(q1, q2, q3):
    """Lista de motivos por los que la pose no sirve (vacía si sirve)."""
    m = []
    for n, q, (lo, hi) in (("J1", q1, Q1_LIM), ("J2", q2, Q2_LIM), ("J3", q3, Q3_LIM)):
        if not lo - 1e-6 <= q <= hi + 1e-6:
            m.append(f"{n} = {q:.1f}° fuera de [{lo:.0f}, {hi:.1f}]")
    if q3 > q3_max_franja(q2) - MARGEN_FRANJA:
        m.append(f"franja: con J2 en {q2:.1f}°, J3 hasta {q3_max_franja(q2) - MARGEN_FRANJA:.1f}°")
    if q3 - q2 < Q3_MENOS_Q2_MIN + MARGEN_FRANJA:   # como el firmware (Q2_MENOS_Q3_DESTINO con MARGEN_FRANJA)
        m.append(f"q3 - q2 = {q3 - q2:.1f}° < {Q3_MENOS_Q2_MIN + MARGEN_FRANJA:.0f}° (J3 no queda firme)")
    m += choques(directa(q1, q2, q3)[1])
    return m


def choques(puntos, paso=10.0):
    """Revisa los tramos hombro-codo-muñeca-punta contra la caja, el mástil y el piso (con MARGEN)."""
    m = set()
    for a, b in zip(puntos[:-1], puntos[1:]):
        n = max(2, int(np.linalg.norm(b - a) / paso))
        for t in np.linspace(0, 1, n):
            x, y, z = a + t * (b - a)
            if (CAJA_X[0] - MARGEN <= x <= CAJA_X[1] + MARGEN and CAJA_Y[0] - MARGEN <= y <= CAJA_Y[1] + MARGEN
                    and z < MARGEN and np.hypot(x, y) > 120):   # el pie del brazo (r < 120 mm) está sobre la tapa
                m.add("choca con la caja")
            if np.hypot(x - MASTIL[0], y - MASTIL[1]) < MASTIL[2] + MARGEN:
                m.add("choca con el mástil")
            if z < -CAJA_ALTO + MARGEN:
                m.add("choca con el piso")
    return sorted(m)


def fuera_de_lo_medido(q2, q3):
    (a, b), (c, d) = ZONA_MEDIDA
    return not (a <= q2 <= b and c <= q3 <= d)


def inversa(x, y, z, tol=0.5):
    """(x, y, z) de la punta en mm -> (q1, q2, q3) en grados del encoder, o (None, motivos)."""
    r = math.hypot(x, y)
    q1 = math.degrees(math.atan2(y, x))
    objetivo = np.array([r, z - H_HOMBRO])
    mod = modelo()
    # semilla: la mejor de una grilla de 2° dentro de los límites
    mejor = None
    for q2 in np.arange(Q2_LIM[0], Q2_LIM[1] + 0.1, 2.0):
        for q3 in np.arange(Q3_LIM[0], Q3_LIM[1] + 0.1, 2.0):
            e = np.linalg.norm(mod.plano(q2, q3)[0] - objetivo)
            if mejor is None or e < mejor[0]:
                mejor = (e, q2, q3)
    q = np.array(mejor[1:], float)
    for _ in range(30):   # Newton con jacobiano numérico
        f = mod.plano(*q)[0] - objetivo
        if np.linalg.norm(f) < 1e-3:
            break
        J = np.column_stack([(mod.plano(q[0] + h[0], q[1] + h[1])[0] - mod.plano(*q)[0]) / 1e-3
                             for h in ((1e-3, 0), (0, 1e-3))])
        q = q - np.linalg.solve(J, f)
    err = float(np.linalg.norm(mod.plano(*q)[0] - objetivo))
    if err > tol:
        return None, [f"no llega: la punta queda a {err:.0f} mm del punto (fuera del alcance)"]
    q1, q2, q3 = q1, float(q[0]), float(q[1])
    mot = problemas(q1, q2, q3)
    return (q1, q2, q3), mot


def prueba():
    d = np.loadtxt(CSV, delimiter=",", comments="#", skiprows=5)
    n = len(d)
    print(f"Modelo con las {n} poses: L3 {modelo().L3:.1f} mm, pinza desde la muñeca "
          f"({modelo().pinza[0]:.1f}, {modelo().pinza[1]:.1f}) mm")
    print("1) Dejando cada pose afuera del ajuste (error real esperable de la punta):")
    errs = []
    for k in range(n):
        mk = Modelo(np.delete(d, k, axis=0))
        e = np.linalg.norm(mk.plano(d[k, 0], d[k, 1])[0] - d[k, 6:8])
        errs.append(e)
        print(f"   q2 {d[k, 0]:5.1f} q3 {d[k, 1]:5.1f}: {e:5.1f} mm")
    print(f"   media {np.mean(errs):.1f} mm, máximo {np.max(errs):.1f} mm")
    print("2) Ida y vuelta (directa -> inversa) en una grilla dentro de lo medido:")
    peor = 0.0
    for q2 in range(25, 79, 6):
        for q3 in range(8, 81, 6):
            if problemas(30, q2, q3)[:1] and any("franja" in p or "q3 - q2" in p for p in problemas(30, q2, q3)):
                continue
            p, _ = directa(30, q2, q3)
            q, _ = inversa(*p)
            if q is not None:
                peor = max(peor, abs(q[1] - q2), abs(q[2] - q3))
    print(f"   peor diferencia de ángulos {peor:.4f}°")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("directa"); a.add_argument("q", type=float, nargs=3)
    b = sub.add_parser("inversa"); b.add_argument("p", type=float, nargs=3)
    sub.add_parser("prueba")
    args = ap.parse_args()
    if args.cmd == "directa":
        q1, q2, q3 = args.q
        p, _ = directa(q1, q2, q3)
        print(f"punta: x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm  (r {math.hypot(p[0], p[1]):.0f} mm)")
        for m in problemas(q1, q2, q3):
            print("  ojo:", m)
        if fuera_de_lo_medido(q2, q3):
            print("  ojo: fuera de la zona medida (el modelo extrapola)")
    elif args.cmd == "inversa":
        q, mot = inversa(*args.p)
        if q is None:
            print("Sin solución:", "; ".join(mot))
            return
        print(f"q1 {q[0]:.2f}  q2 {q[1]:.2f}  q3 {q[2]:.2f}   ->   mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}")
        for m in mot:
            print("  NO SIRVE:", m)
        if fuera_de_lo_medido(q[1], q[2]):
            print("  ojo: fuera de la zona medida (el modelo extrapola)")
    else:
        prueba()


if __name__ == "__main__":
    main()
