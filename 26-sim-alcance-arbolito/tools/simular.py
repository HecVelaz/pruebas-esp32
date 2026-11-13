#!/usr/bin/env python3
"""Simulación de la estación de trabajo: el robot parado al lado del arbolito, ¿qué limones puede agarrar?

Un limón cuenta como "agarrable" si se cumplen las tres cosas:
  1. ALCANCE: el planificador de "agarrar" (punta.py) puede entrar de frente, cerrar, salir y volver al reposo
     (mapa de tools/alcance.py, con la franja, los límites y los choques reales del brazo);
  2. VISIÓN: la cámara izquierda lo ve dentro de la zona donde hay disparidad (u >= 280 px, 0,40-0,95 m) y dentro de
     la imagen, con la TF cámara -> brazo de la calibración del 12/11;
  3. DE FRENTE: el limón está del lado de la copa que mira al robot (la pinza entra en recta desde J1: uno del otro
     lado obligaría a atravesar el árbol) y la cámara lo ve de frente (no tapado por la copa).
El arbolito: tronco y copa como un cilindro; los limones cuelgan sobre la superficie de la copa entre --h-min y
--h-max (alturas del piso). El robot: la tapa en T de recolector_de_frutas (40 x 23 adelante, 34 x 17 atrás, tapa a
23 cm del piso), J1 a 9 cm del borde de atrás y del izquierdo, el mástil con la cámara a la derecha y adelante.

Alturas: el modelo de la punta (cinematica.py) sube y baja menos que el brazo real (bitácora 12/11: k ~1,26). Para
pasar de la altura del piso (lo que se mide en el árbol con cinta) a la z del modelo (lo que usan el planificador y la
visión, que se calibró contra el modelo) se usa la recta ajustada a las mediciones con cinta del 12/11 (--nominal:
tapa a 230 mm y sin escala).

Estación: el centro del árbol en el marco del robot (cinematica.py: x a la derecha, y adelante, origen en J1) está en
(-D, Y): D = distancia lateral de J1 al eje del árbol, Y = cuánto más adelante que J1 (el árbol a la IZQUIERDA del
robot, donde mira la cámara). Se barre (D, Y) y la cantidad de estaciones alrededor del árbol.

Uso:
    python3 tools/simular.py                          # valores por defecto, el mapa más nuevo de resultados/
    python3 tools/simular.py --radio-copa 200 --h-min 300 --h-max 540 --alto 600
    python3 tools/simular.py --estacion 600 150       # además, figura 3D de esa estación (D Y en mm)
Salida: resultados/sim_<fecha>_*.png y sim_<fecha>_resumen.txt
"""
import argparse
import math
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI.parent.parent / "20-brazo-coordinado" / "tools"))  # reutilizar, no copiar
import ajuste_camara as ajc  # noqa: E402
import cinematica as k  # noqa: E402
import punta as p  # noqa: E402

RES = AQUI.parent / "resultados"
SELLO = time.strftime("%Y%m%d_%H%M%S")

# --- Robot (recolector_de_frutas: tapa.jpeg y arturito.urdf.xacro), marco de cinematica.py, mm
TAPA_PISO = 230.0          # tapa sobre el piso (caja de 20 cm + 3 cm de la base sobre el piso)
CAMARA_IMG = (640, 480)
CAM_FX, CAM_CX, CAM_CY = 672.126, 425.456, 236.810   # projection_matrix de calibracion_mastil/left.yaml
U_MIN_DISPARIDAD = 280.0   # a la izquierda de esto no hay disparidad (medir_limon.py)
Z_CAM = (400.0, 950.0)     # z_min / z_max de limon.launch.py
RADIO_LIMON = p.RADIO_LIMON
DE_FRENTE_BRAZO = 60.0     # ° máximos entre la normal de la copa y la dirección hacia J1
DE_FRENTE_CAMARA = 75.0    # ° máximos entre la normal de la copa y la dirección hacia la cámara

# Alturas con cinta (centro del limón desde el piso, mm) contra la z del modelo: bitácora 12/11, sin P2 (dudosa)
CINTA_Z = np.array([59, 80, 80, 120, 119, 180, 180, 179], float)
CINTA_H = np.array([330, 370, 390, 400, 440, 510, 510, 520], float)


class Alturas:
    def __init__(self, nominal):
        if nominal:
            self.b, self.a = 1.0, TAPA_PISO
        else:
            self.b, self.a = np.polyfit(CINTA_Z, CINTA_H, 1)
        self.nominal = nominal

    def z_modelo(self, h):
        return (np.asarray(h, float) - self.a) / self.b

    def h_piso(self, z):
        return self.a + self.b * np.asarray(z, float)

    def __str__(self):
        return ("nominal: piso = z + 230 mm" if self.nominal else
                f"cinta del 12/11: piso = {self.a:.0f} + {self.b:.2f}·z mm")


# ------------------------------------------------------------------------------------------------------ alcance

class Mapa:
    """Mapa de tools/alcance.py: agarra(x, y, z) por la dirección de J1 más cercana y la celda (r, z) más cercana."""

    def __init__(self, ruta):
        d = np.genfromtxt(ruta, delimiter=",", skip_header=1, usecols=(0, 1, 2, 3))
        self.ruta = ruta
        self.dirs = np.unique(d[:, 0])
        self.rs = np.unique(d[:, 1])
        self.zs = np.unique(d[:, 2])
        self.ok = np.zeros((len(self.dirs), len(self.rs), len(self.zs)), bool)
        for a, r, z, v in d:
            self.ok[np.searchsorted(self.dirs, a), np.searchsorted(self.rs, r), np.searchsorted(self.zs, z)] = v > 0
        pr, pz = self.rs[1] - self.rs[0], self.zs[1] - self.zs[0]
        paso_dir = (self.dirs[1] - self.dirs[0]) if len(self.dirs) > 1 else 20.0
        # Fuera de las direcciones calculadas no se extrapola más de medio paso (y nunca afuera de los límites de J1)
        self.dir_lim = (max(self.dirs[0] - paso_dir / 2, k.q1_real(k.Q1_LIM[0])),
                        min(self.dirs[-1] + paso_dir / 2, k.q1_real(k.Q1_LIM[1])))
        self.pr, self.pz = pr, pz

    def agarra(self, x, y, z):
        x, y, z = (np.asarray(v, float) for v in (x, y, z))
        ang = np.degrees(np.arctan2(y, -x))
        r = np.hypot(x, y)
        dentro = ((ang >= self.dir_lim[0]) & (ang <= self.dir_lim[1]) & (r >= self.rs[0] - self.pr / 2)
                  & (r <= self.rs[-1] + self.pr / 2) & (z >= self.zs[0] - self.pz / 2) & (z <= self.zs[-1] + self.pz / 2))
        i = np.abs(ang[..., None] - self.dirs).argmin(-1)
        j = np.clip(np.rint((r - self.rs[0]) / self.pr).astype(int), 0, len(self.rs) - 1)
        m = np.clip(np.rint((z - self.zs[0]) / self.pz).astype(int), 0, len(self.zs) - 1)
        return dentro & self.ok[i, j, m]


def ultimo_mapa():
    a = sorted(RES.glob("mapa_alcance_*.csv"))
    if not a:
        raise SystemExit("no hay resultados/mapa_alcance_*.csv: correr primero tools/alcance.py")
    return a[-1]


# ------------------------------------------------------------------------------------------------------- cámara

class Camara:
    def __init__(self):
        R, t, _, _ = ajc.cargar_tf(p.CAL_CAMARA, p.CAL_BRAZO, RADIO_LIMON, p.CAL_SIN)
        self.R, self.t = R, t   # cámara (mm, `camera`) -> brazo_base (mm)
        # posición de la cámara en el marco de cinematica.py: brazo_base (x adelante, y izq.) -> (x = -y, y = x)
        self.pos = np.array([-t[1], t[0], t[2]])

    def a_camara(self, P):
        """Puntos (N, 3) del marco de cinematica.py -> marco óptico de la cámara (mm)."""
        P = np.atleast_2d(P)
        bb = np.column_stack([P[:, 1], -P[:, 0], P[:, 2]])
        return (bb - self.t) @ self.R   # = R^T (p - t)

    def ve(self, P):
        C = self.a_camara(P)
        Z = C[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = CAM_FX * C[:, 0] / Z + CAM_CX
            v = CAM_FX * C[:, 1] / Z + CAM_CY
            r_px = CAM_FX * RADIO_LIMON / Z
        cara = Z - RADIO_LIMON
        return ((cara >= Z_CAM[0]) & (cara <= Z_CAM[1]) & (u >= U_MIN_DISPARIDAD) & (u <= CAMARA_IMG[0] - r_px)
                & (v >= r_px) & (v <= CAMARA_IMG[1] - r_px))

    def ventana(self, Zs=Z_CAM):
        """Esquinas de la zona útil de la imagen (u 280..640, v 0..480) a las distancias Zs, en cinematica.py."""
        out = []
        for Z in Zs:
            esq = []
            for u, v in ((U_MIN_DISPARIDAD, 0), (CAMARA_IMG[0], 0), (CAMARA_IMG[0], CAMARA_IMG[1]),
                         (U_MIN_DISPARIDAD, CAMARA_IMG[1])):
                c = np.array([(u - CAM_CX) * Z / CAM_FX, (v - CAM_CY) * Z / CAM_FX, Z])
                bb = self.R @ c + self.t
                esq.append([-bb[1], bb[0], bb[2]])
            out.append(np.array(esq))
        return out


# ------------------------------------------------------------------------------------------------------- árbol

class Arbol:
    def __init__(self, radio_copa, h_min, h_max, alto, paso_ang=10.0, paso_h=20.0):
        self.radio, self.h_min, self.h_max, self.alto = radio_copa, h_min, h_max, alto
        angs = np.radians(np.arange(0, 360, paso_ang))
        hs = np.arange(h_min, h_max + 0.1, paso_h)
        A, H = np.meshgrid(angs, hs)
        self.phi, self.h = A.ravel(), H.ravel()       # ángulo en la copa (en el marco del árbol) y altura del piso
        self.n = len(self.h)

    def limones(self, centro, giro=0.0):
        """(N, 3) xy en el marco del robot (z = altura del piso) y las normales, con el árbol girado `giro` rad."""
        a = self.phi + giro
        nrm = np.column_stack([np.cos(a), np.sin(a), np.zeros_like(a)])
        xy = np.asarray(centro, float)[:2] + self.radio * nrm[:, :2]
        return np.column_stack([xy, self.h]), nrm


def huella_robot(margen=0.0):
    """Polígono de la tapa en T (x, y) del marco de cinematica.py."""
    (ya, yf), yan = k.CAJA_Y, k.CAJA_Y_ANCHA
    (xa0, xa1), (xn0, xn1) = k.CAJA_X_ANCHA, k.CAJA_X_ANGOSTA
    m = margen
    return np.array([(xn0 - m, ya - m), (xn1 + m, ya - m), (xn1 + m, yan), (xa1 + m, yan), (xa1 + m, yf + m),
                     (xa0 - m, yf + m), (xa0 - m, yan), (xn0 - m, yan), (xn0 - m, ya - m)])


def choca_robot(centro, radio, margen=50.0):
    """¿La copa (círculo) toca la tapa en T o el mástil (con margen)?"""
    cx, cy = centro
    for x0, x1, y0, y1 in ((k.CAJA_X_ANGOSTA[0], k.CAJA_X_ANGOSTA[1], k.CAJA_Y[0], k.CAJA_Y_ANCHA),
                           (k.CAJA_X_ANCHA[0], k.CAJA_X_ANCHA[1], k.CAJA_Y_ANCHA, k.CAJA_Y[1])):
        dx = max(x0 - cx, 0, cx - x1)
        dy = max(y0 - cy, 0, cy - y1)
        if math.hypot(dx, dy) < radio + margen:
            return True
    return math.hypot(cx - k.MASTIL[0], cy - k.MASTIL[1]) < radio + k.MASTIL[2] + margen


# ------------------------------------------------------------------------------------------------------ evaluar

def evaluar(mapa, cam, alt, arbol, centro, giro=0.0):
    """Para cada limón: (agarra, ve, de_frente) y los puntos."""
    P, nrm = arbol.limones(centro, giro)
    zm = alt.z_modelo(P[:, 2])
    Pm = np.column_stack([P[:, :2], zm])                       # marco de cinematica.py (z del modelo)
    alc = mapa.agarra(Pm[:, 0], Pm[:, 1], Pm[:, 2])
    ve = cam.ve(Pm)
    hacia_j1 = -Pm[:, :2] / np.linalg.norm(Pm[:, :2], axis=1, keepdims=True)
    frente_b = (nrm[:, :2] * hacia_j1).sum(1) >= math.cos(math.radians(DE_FRENTE_BRAZO))
    hacia_cam = cam.pos[:2] - Pm[:, :2]
    hacia_cam /= np.linalg.norm(hacia_cam, axis=1, keepdims=True)
    frente_c = (nrm[:, :2] * hacia_cam).sum(1) >= math.cos(math.radians(DE_FRENTE_CAMARA))
    return alc & frente_b, ve & frente_c, P, Pm


def barrido(mapa, cam, alt, arbol, Ds, Ys, n_est=1):
    """% de limones agarrables (alcance + visión) con n_est estaciones repartidas alrededor del árbol."""
    C = np.full((len(Ys), len(Ds)), np.nan)
    for i, Y in enumerate(Ys):
        for j, D in enumerate(Ds):
            centro = (-D, Y)
            if choca_robot(centro, arbol.radio):
                continue
            tot = np.zeros(arbol.n, bool)
            for e in range(n_est):
                a, v, _, _ = evaluar(mapa, cam, alt, arbol, centro, 2 * math.pi * e / n_est)
                tot |= a & v
            C[i, j] = 100.0 * tot.mean()
    return C


# ------------------------------------------------------------------------------------------------------ figuras

def caja_3d(ax, alt_tapa):
    """Tapa en T como prisma (piso -> tapa), en mm del piso."""
    h = huella_robot()
    for z in (0.0, alt_tapa):
        ax.plot(h[:, 0], h[:, 1], z, color="0.25", lw=1.2)
    for x, y in h[:-1]:
        ax.plot([x, x], [y, y], [0, alt_tapa], color="0.25", lw=0.8)


def fig_mapa(mapa, alt, arbol, ruta):
    fig, axs = plt.subplots(1, len(mapa.dirs), figsize=(4.6 * len(mapa.dirs), 4.6), sharey=True)
    axs = np.atleast_1d(axs)
    for ax, d, ok in zip(axs, mapa.dirs, mapa.ok):
        R, Z = np.meshgrid(mapa.rs, mapa.zs, indexing="ij")
        H = alt.h_piso(Z)
        ax.scatter(R[ok] / 10, H[ok] / 10, s=28, c="tab:green", marker="s", label="agarra")
        ax.scatter(R[~ok] / 10, H[~ok] / 10, s=6, c="0.75", label="no")
        ax.axhspan(arbol.h_min / 10, arbol.h_max / 10, color="gold", alpha=0.25, label="limones del arbolito")
        ax.axhline(arbol.alto / 10, color="saddlebrown", ls="--", lw=1, label="alto del arbolito")
        ax.axhline(TAPA_PISO / 10, color="0.3", lw=1)
        ax.text(mapa.rs[0] / 10, TAPA_PISO / 10 + 0.5, "tapa", fontsize=8, color="0.3")
        ax.set_title(f"J1 a {d:.0f}° (0° = izquierda del robot)")
        ax.set_xlabel("distancia horizontal del limón a J1 [cm]")
        ax.grid(alpha=0.3)
    axs[0].set_ylabel("altura del limón sobre el piso [cm]")
    axs[0].legend(fontsize=8, loc="lower left")
    fig.suptitle(f"Alcance del agarre (planificador de punta.py; alturas: {alt})", fontsize=10)
    fig.tight_layout()
    fig.savefig(ruta, dpi=130)
    plt.close(fig)


def fig_barrido(Ds, Ys, C1, cobertura, mejor, ruta):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.8), gridspec_kw={"width_ratios": [1.4, 1]})
    im = a1.pcolormesh(np.array(Ds) / 10, np.array(Ys) / 10, C1, cmap="viridis", vmin=0, vmax=max(1, np.nanmax(C1)),
                       shading="nearest")
    fig.colorbar(im, ax=a1, label="% de limones agarrables (una estación)")
    D, Y = mejor
    a1.plot(D / 10, Y / 10, "r*", ms=14, label=f"mejor: D {D / 10:.1f} cm, Y {Y / 10:.1f} cm")
    a1.set_xlabel("D: distancia lateral de J1 al eje del árbol [cm]")
    a1.set_ylabel("Y: el árbol más adelante que J1 [cm]")
    a1.set_title("Una estación (gris/blanco: la copa toca el robot)")
    a1.legend(fontsize=8)
    ns = list(cobertura)
    a2.bar(ns, [cobertura[n][0] for n in ns], color="tab:green")
    for n in ns:
        a2.text(n, cobertura[n][0] + 1, f"{cobertura[n][0]:.0f} %", ha="center", fontsize=9)
    a2.set_ylim(0, 105)
    a2.set_xlabel("estaciones alrededor del árbol (repartidas por igual)")
    a2.set_ylabel("% de limones agarrables")
    a2.set_title("Cobertura con varias estaciones (D, Y óptimos de cada una)")
    a2.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(ruta, dpi=130)
    plt.close(fig)


def fig_estacion(mapa, cam, alt, arbol, centro, ruta3d, ruta_planta):
    agarra, ve, P, Pm = evaluar(mapa, cam, alt, arbol, centro)
    ok = agarra & ve
    solo_alc = agarra & ~ve
    nada = ~agarra
    # nube de alcance a la altura de los limones (en el plano de la copa, para no tapar todo)
    angs = np.arange(mapa.dir_lim[0], mapa.dir_lim[1] + 0.1, 4.0)
    nube = []
    for d in angs:
        i = np.abs(mapa.dirs - d).argmin()
        for j, r in enumerate(mapa.rs):
            for m_, z in enumerate(mapa.zs):
                if mapa.ok[i, j, m_]:
                    a = math.radians(d)
                    nube.append((-r * math.cos(a), r * math.sin(a), alt.h_piso(z)))
    nube = np.array(nube)
    # La cámara es parte del robot: a la altura nominal (tapa + z); la recta de la cinta solo vale para los limones
    cam_piso = np.array([cam.pos[0], cam.pos[1], cam.pos[2] + TAPA_PISO])
    ventanas = cam.ventana()

    fig = plt.figure(figsize=(11, 8.5))
    ax = fig.add_subplot(111, projection="3d")
    caja_3d(ax, TAPA_PISO)
    ax.plot([k.MASTIL[0]] * 2, [k.MASTIL[1]] * 2, [TAPA_PISO, cam_piso[2]], color="k", lw=4)
    ax.scatter(*cam_piso, color="k", s=40, marker="s")
    ax.plot([0, 0], [0, 0], [TAPA_PISO, TAPA_PISO + 165], color="tab:blue", lw=4)   # J1 y el hombro
    if len(nube):
        ax.scatter(nube[:, 0], nube[:, 1], nube[:, 2], s=2, c="tab:green", alpha=0.07)
    for w in ventanas:   # zona útil de la cámara a 0,40 y 0,95 m (alturas en el modelo -> piso)
        w = w.copy()
        w[:, 2] = alt.h_piso(w[:, 2])
        ax.plot(*np.vstack([w, w[:1]]).T, color="tab:purple", lw=0.8)
        for e in w:
            ax.plot(*np.vstack([cam_piso, e]).T, color="tab:purple", lw=0.5, alpha=0.5)
    cx, cy = centro
    t = np.linspace(0, 2 * np.pi, 60)
    ax.plot([cx, cx], [cy, cy], [0, arbol.alto], color="saddlebrown", lw=5)
    for h in (arbol.h_min, arbol.h_max, arbol.alto):
        ax.plot(cx + arbol.radio * np.cos(t), cy + arbol.radio * np.sin(t), h, color="olivedrab", lw=0.8)
    ax.scatter(*P[ok].T, c="limegreen", s=30, edgecolors="k", linewidths=0.4, label=f"agarra y la cámara lo ve ({ok.sum()})")
    ax.scatter(*P[solo_alc].T, c="orange", s=20, label=f"llega, la cámara no lo ve ({solo_alc.sum()})")
    ax.scatter(*P[nada].T, c="red", s=8, alpha=0.5, label=f"no llega / del otro lado ({nada.sum()})")
    ax.set_xlabel("x [mm] (derecha del robot)")
    ax.set_ylabel("y [mm] (adelante)")
    ax.set_zlabel("altura del piso [mm]")
    ax.set_title(f"Estación: árbol a D {-cx / 10:.1f} cm a la izquierda de J1, Y {cy / 10:.1f} cm adelante  —  "
                 f"{100 * ok.mean():.0f} % de los limones", fontsize=10)
    ax.legend(loc="upper left", fontsize=8)
    lim = max(900, -cx + arbol.radio + 100)
    ax.set_xlim(-lim, 400)
    ax.set_ylim(-300, lim - 300 + 400)
    ax.set_zlim(0, 800)
    ax.view_init(elev=22, azim=-130)
    fig.tight_layout()
    fig.savefig(ruta3d, dpi=130)
    plt.close(fig)

    # Planta
    fig, ax = plt.subplots(figsize=(8, 7.5))
    h = huella_robot()
    ax.fill(h[:, 0] / 10, h[:, 1] / 10, color="0.85", ec="0.3", label="robot (tapa en T)")
    ax.add_patch(plt.Circle((k.MASTIL[0] / 10, k.MASTIL[1] / 10), k.MASTIL[2] / 10, color="k"))
    ax.plot(0, 0, "o", color="tab:blue", ms=8, label="J1", zorder=5)
    ax.annotate("frente", (8, 33), fontsize=9, ha="center")
    z_med = alt.z_modelo((arbol.h_min + arbol.h_max) / 2)
    m_ = np.abs(mapa.zs - z_med).argmin()
    for d in angs:
        i = np.abs(mapa.dirs - d).argmin()
        a = math.radians(d)
        rr = mapa.rs[mapa.ok[i, :, m_]]
        ax.scatter(-rr * math.cos(a) / 10, rr * math.sin(a) / 10, s=6, c="tab:green", alpha=0.35)
    ax.scatter([], [], s=6, c="tab:green", label=f"alcance a {(arbol.h_min + arbol.h_max) / 20:.0f} cm del piso")
    w = ventanas
    cp = cam.pos[:2] / 10
    poly = np.array([w[0][0][:2], w[0][1][:2], w[1][1][:2], w[1][0][:2], w[0][0][:2]]) / 10
    ax.plot(poly[:, 0], poly[:, 1], color="tab:purple", lw=1, label="zona con disparidad (borde de arriba de la imagen)")
    ax.plot(*cp, "s", color="k")
    ax.add_patch(plt.Circle((cx / 10, cy / 10), arbol.radio / 10, fill=False, ec="olivedrab", lw=1.5))
    ax.plot(cx / 10, cy / 10, "o", color="saddlebrown", ms=6)
    ax.scatter(P[ok][:, 0] / 10, P[ok][:, 1] / 10, c="limegreen", s=40, edgecolors="k", linewidths=0.4,
               label="agarra (alguna altura)")
    ax.scatter(P[nada][:, 0] / 10, P[nada][:, 1] / 10, c="red", s=8, alpha=0.4)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlabel("x [cm] (derecha del robot)")
    ax.set_ylabel("y [cm] (adelante)")
    ax.set_title(f"Planta de la estación (D {-cx / 10:.1f} cm, Y {cy / 10:.1f} cm)", fontsize=10)
    ax.legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=2)
    fig.tight_layout()
    fig.savefig(ruta_planta, dpi=130)
    plt.close(fig)
    return ok, agarra, ve


# ------------------------------------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mapa", type=Path, help="CSV de tools/alcance.py (def. el más nuevo)")
    ap.add_argument("--radio-copa", type=float, default=200.0, help="radio de la copa donde cuelgan los limones (mm)")
    ap.add_argument("--h-min", type=float, default=300.0, help="limón más bajo, centro desde el piso (mm)")
    ap.add_argument("--h-max", type=float, default=540.0, help="limón más alto, centro desde el piso (mm)")
    ap.add_argument("--alto", type=float, default=600.0, help="alto del arbolito (mm)")
    ap.add_argument("--nominal", action="store_true", help="alturas: tapa a 230 mm, sin la escala medida con cinta")
    ap.add_argument("--estacion", type=float, nargs=2, metavar=("D", "Y"), help="figura 3D de esta estación (mm)")
    ap.add_argument("--max-est", type=int, default=6, help="máximo de estaciones alrededor del árbol a probar")
    a = ap.parse_args()

    mapa = Mapa(a.mapa or ultimo_mapa())
    cam = Camara()
    alt = Alturas(a.nominal)
    arbol = Arbol(a.radio_copa, a.h_min, a.h_max, a.alto)
    pref = RES / f"sim_{SELLO}"
    lineas = [f"Simulación de la estación de trabajo ({SELLO})",
              f"mapa: {mapa.ruta.name} (direcciones de J1 {', '.join(f'{d:.0f}' for d in mapa.dirs)}°; "
              f"se usa entre {mapa.dir_lim[0]:.0f} y {mapa.dir_lim[1]:.0f}°)",
              f"alturas: {alt}",
              f"arbolito: alto {a.alto:.0f} mm, copa de radio {a.radio_copa:.0f} mm, limones de {a.h_min:.0f} a "
              f"{a.h_max:.0f} mm del piso ({arbol.n} posiciones: cada 10° y 20 mm)",
              f"  -> en la z del modelo: de {alt.z_modelo(a.h_min):.0f} a {alt.z_modelo(a.h_max):.0f} mm sobre la tapa",
              f"cámara en ({cam.pos[0]:.0f}, {cam.pos[1]:.0f}, {cam.pos[2]:.0f}) mm del marco del brazo"]

    fig_mapa(mapa, alt, arbol, f"{pref}_mapa_rz.png")

    Ds = np.arange(350, 1001, 25)
    Ys = np.arange(-200, 601, 25)
    C1 = barrido(mapa, cam, alt, arbol, Ds, Ys, 1)
    i, j = np.unravel_index(np.nanargmax(C1), C1.shape)
    mejor1 = (Ds[j], Ys[i])
    cobertura = {}
    for n in range(1, a.max_est + 1):
        C = C1 if n == 1 else barrido(mapa, cam, alt, arbol, Ds, Ys, n)
        ii, jj = np.unravel_index(np.nanargmax(C), C.shape)
        cobertura[n] = (float(C[ii, jj]), Ds[jj], Ys[ii])
    fig_barrido(Ds, Ys, C1, cobertura, mejor1, f"{pref}_estaciones.png")

    est = tuple(a.estacion) if a.estacion else mejor1
    ok, agarra, ve = fig_estacion(mapa, cam, alt, arbol, (-est[0], est[1]), f"{pref}_estacion_3d.png",
                                  f"{pref}_planta.png")

    lineas += ["", "Una estación: mejor D = %.0f mm (J1 -> eje del árbol, hacia la izquierda), Y = %.0f mm (adelante "
               "de J1): %.0f %% de los limones" % (mejor1[0], mejor1[1], np.nanmax(C1)),
               f"  borde de la copa a {mejor1[0] - a.radio_copa - 120:.0f} mm del costado izquierdo del robot "
               f"(parte ancha) y a {mejor1[0] - a.radio_copa:.0f} mm de J1",
               "", "Estaciones repartidas alrededor del árbol (el robot da la vuelta y para N veces):"]
    for n, (c, D, Y) in cobertura.items():
        lineas.append(f"  {n}: {c:5.1f} %  (D {D:.0f}, Y {Y:.0f} mm)")
    lineas += ["", f"Estación de las figuras (D {est[0]:.0f}, Y {est[1]:.0f}): agarra y ve {ok.sum()} de {arbol.n} "
               f"({100 * ok.mean():.0f} %); llegan {agarra.sum()}, la cámara ve {ve.sum()}"]
    # Por altura, en la estación de las figuras
    lineas.append("  por altura del piso (agarrables / del lado del robot):")
    _, nrm = arbol.limones((-est[0], est[1]))
    lado = nrm[:, 0] > 0.5   # mirando al robot (el árbol está a la izquierda: el robot hacia +x)
    for h in np.unique(arbol.h):
        s = (arbol.h == h) & lado
        lineas.append(f"    {h / 10:5.1f} cm: {ok[s].sum():2d} / {s.sum()}")
    txt = "\n".join(lineas)
    print(txt)
    Path(f"{pref}_resumen.txt").write_text(txt + "\n")
    print(f"\nfiguras: {pref.name}_mapa_rz.png, _estaciones.png, _estacion_3d.png, _planta.png")


if __name__ == "__main__":
    main()
