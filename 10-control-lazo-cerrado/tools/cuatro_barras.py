#!/usr/bin/env python3
"""Cuatro barras del brazo: cuánto puede girar J2 con J3 quieto (o J3 con J2 quieto).

Geometría y convenciones de la Entrega 4 (R1), con el balancín rojo de 60 mm:
  q2      ángulo de la barra verde respecto de la horizontal, antihorario, medido hacia el lado de la muñeca.
  theta   ángulo de la barra roja respecto de la misma horizontal y con el mismo sentido.
  gamma4  ángulo entre la roja y la verde = q2 - theta (E4: gamma4 = q2 + q3 - 90°, con theta = 90° - q3).

J2 y J3 son de sin fin: con un motor quieto, el otro cambia gamma4 grado por grado. El mecanismo es Grashof
(40 + 350,22 < 338 + 60): el balancín corto del antebrazo (40 mm) da la vuelta, pero la barra roja solo se
mueve entre los dos puntos muertos (|gamma4| = 113°). Ventana útil: rama física (-180° < gamma4 < 0°) y
ángulo de transmisión mu entre MU_MIN y 180° - MU_MIN.

Uso:
  python3 tools/cuatro_barras.py                      # ventana de gamma4 y tabla
  python3 tools/cuatro_barras.py --q2 30 --theta 100  # postura medida: cuánto puede subir y bajar J2
"""
import argparse

import numpy as np

L2 = 350.22  # barra verde (hombro -> codo)
L4 = 338.0  # acoplador amarillo
L30 = 60.0  # balancín de entrada, rojo (era 40 mm en la E4)
L31 = 40.0  # balancín de salida, tramo corto del antebrazo
MU_MIN = 15.0  # ángulo de transmisión mínimo (E4)


def directo(gamma4_deg):
    """gamma4 -> (q3O, mu) en grados, o None si el mecanismo no cierra."""
    g = np.radians(gamma4_deg)
    diag = np.sqrt(L2**2 + L30**2 - 2 * L2 * L30 * np.cos(g))
    c = (diag**2 + L31**2 - L4**2) / (2 * L31 * diag)
    if abs(c) > 1:
        return None
    alpha4 = np.arctan2(L30 * np.sin(g), L2 - L30 * np.cos(g))
    q3o = alpha4 - np.arccos(c)
    mu = np.arccos(np.clip((L31**2 + L4**2 - diag**2) / (2 * L31 * L4), -1, 1))
    return np.degrees(q3o), np.degrees(mu)


def valido(gamma4_deg):
    r = directo(gamma4_deg)
    return r is not None and -180 < gamma4_deg < 0 and MU_MIN <= r[1] <= 180 - MU_MIN


def ventana():
    gs = [g for g in np.arange(-180, 0, 0.1) if valido(g)]
    return min(gs), max(gs)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--q2", type=float, help="ángulo medido de la barra verde (°)")
    ap.add_argument("--theta", type=float, help="ángulo medido de la barra roja (°)")
    ap.add_argument("--margen", type=float, default=10.0, help="margen contra los bordes de la ventana (°)")
    a = ap.parse_args()

    s = sorted([L2, L4, L30, L31])
    print(f"Grashof: s + l = {s[0] + s[3]:.2f} mm, p + q = {s[1] + s[2]:.2f} mm -> "
          f"{'SÍ' if s[0] + s[3] <= s[1] + s[2] else 'NO'}")
    gmin, gmax = ventana()
    print(f"Ventana útil de gamma4 (mu entre {MU_MIN:.0f}° y {180 - MU_MIN:.0f}°): {gmin:.1f}° … {gmax:.1f}° "
          f"({gmax - gmin:.1f}°, centro {(gmin + gmax) / 2:.1f}°)")
    print("\n gamma4    q3O     mu")
    for g in range(-110, -25, 10):
        q3o, mu = directo(g)
        print(f" {g:5d}°  {q3o:6.1f}°  {mu:5.1f}°")

    if a.q2 is None or a.theta is None:
        return
    g = a.q2 - a.theta
    r = directo(g)
    print(f"\nPostura: q2 = {a.q2:.1f}°, roja = {a.theta:.1f}° -> gamma4 = {g:.1f}°", end="")
    print(f", q3O = {r[0]:.1f}°, mu = {r[1]:.1f}°" if r else " (el mecanismo NO cierra: revisar la medición)")
    if not valido(g):
        print("FUERA de la ventana útil: no mover J2 ni J3 hasta revisar la medición.")
        return
    baja, sube = gmin + a.margen - g, gmax - a.margen - g
    print(f"Con J3 quieto, J2 puede ir de {baja:+.1f}° a {sube:+.1f}° desde donde está "
          f"(q2 de {a.q2 + baja:.1f}° a {a.q2 + sube:.1f}°, con {a.margen:.0f}° de margen).")
    print(f"Con J2 quieto, la roja puede ir de {-sube:+.1f}° a {-baja:+.1f}° desde donde está.")


if __name__ == "__main__":
    main()
