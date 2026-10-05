#!/usr/bin/env python3
"""Identificación del modelo de J3 (codo) con los datos guardados de la placa, sin tocar la placa.

Modelo: la planta de primer orden de J1 más lo que J3 tiene distinto (el peso del antebrazo y el sin fin
autobloqueante). Una sola ganancia K (es un solo motor); el peso corre el duty necesario, no cambia la ganancia:
    tau * dw/dt = -w + K * (u - c+(theta))   si u > c+(theta)       (subiendo)
    tau * dw/dt = -w + K * (u + c-)          si u < -c-             (bajando)
    tau * dw/dt = -w                         si -c- <= u <= c+      (el sin fin no deja que el peso la mueva:
                                                                     frena hasta 0 y no invierte)
    c+(theta) = c0 + c1 * theta   (subiendo, el peso visto a través del cuatro barras crece con el ángulo)
    c-        = constante         (bajando, el peso ayuda: solo queda la fricción)
    Quieta, para despegar hace falta además b+ subiendo o b- bajando (fricción estática > dinámica).
w = velocidad de la barra roja [°/s], u = duty [%], theta = ángulo de J3 desde la marca [°].

Etapa 1, estática (barridos de carga a ±4 °/s, resultados/barrido_20261003_19*): regresión lineal del duty
aplicado mientras se mueve, según el ángulo. Subiendo da la recta u4+(theta) = a + b·theta y bajando un valor
constante u4-. Como en esos barridos la velocidad es 4 °/s: c+ = a + b·theta - 4/K y c- = |u4-| - 4/K.

Etapa 2, dinámica (escalones de velocidad ±10 °/s, resultados/escalon_vel_20261003_18*, _19*): con la recta de la
etapa 1 fija, se ajustan tau, K, b+ y b- por error de salida: al modelo se le aplica el MISMO duty que se aplicó en
la placa (columna duty del CSV) y se compara la velocidad simulada, medida igual que el firmware (cuentas enteras
del encoder en 4 muestras), con la medida. Se minimiza el error medio absoluto (Nelder-Mead).

Lo que el modelo no tiene: el juego del piñón y el engranaje (el pico al despegar subiendo) y el traba-suelta al
bajar despacio (efecto Stribeck). Por eso el error que queda es de ~2-3 °/s.

Uso:
    python3 tools/identificar_j3.py               # ajusta, imprime los parámetros y guarda el PNG de validación
    python3 tools/identificar_j3.py --sin-ventana
"""
import argparse
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, str(Path(__file__).resolve().parent))
import diseno_velocidad as dv  # noqa: E402

DIR = Path(__file__).resolve().parent.parent
DIR_RESULTADOS = DIR / "resultados"

CUENTAS_POR_GRADO = dv.PLANTAS["j3"]["cuentas_por_grado"]   # 34,57
TS = dv.TS
VENTANA = 4
V_BARRIDO = 4.0

ESCALONES = ["escalon_vel_20261003_184307", "escalon_vel_20261003_184927", "escalon_vel_20261003_185219",
             "escalon_vel_20261003_185316", "escalon_vel_20261003_185352", "escalon_vel_20261003_185455",
             "escalon_vel_20261003_190957", "escalon_vel_20261003_191028"]
BARRIDOS = ["barrido_20261003_194419_cortado", "barrido_20261003_195629"]

NOMBRES = ["tau [s]", "K [(°/s)/%]", "b+ [%]", "b- [%]"]
INICIAL = [0.06, 1.3, 7.0, 6.0]


def cargar(nombre):
    """Tramos del CSV (un barrido trae dos: subida y bajada, cada uno con su encabezado)."""
    tramos, filas = [], []
    for l in (DIR_RESULTADOS / f"{nombre}.csv").read_text().splitlines():
        if l.startswith("# barrido") or l.startswith("# escalon_vel"):
            if filas:
                tramos.append(filas)
            filas = []
        elif l and l[0].isdigit():
            filas.append(l.split(","))
    if filas:
        tramos.append(filas)
    out = []
    for i, f in enumerate(tramos):
        d = np.array(f, dtype=float)
        out.append({"nombre": nombre + (f" ({'subida' if i == 0 else 'bajada'})" if len(tramos) > 1 else ""),
                    "t": (d[:, 0] - 10) / 1000, "w_ref": d[:, 1], "w_med": d[:, 2], "u": d[:, 3], "pos": d[:, 4]})
    return out


def estatica(barridos):
    """Etapa 1: rectas del duty necesario para moverse a 4 °/s según el ángulo."""
    th_s, u_s, u_b = [], [], []
    for d in barridos:
        en_marcha = (np.abs(d["w_med"]) > 1.0) & (np.arange(len(d["u"])) > 50)  # sin el arranque
        sube = en_marcha & (d["w_ref"] > 0)
        baja = en_marcha & (d["w_ref"] < 0)
        th_s.append(d["pos"][sube])
        u_s.append(d["u"][sube])
        u_b.append(d["u"][baja])
    th_s, u_s, u_b = np.concatenate(th_s), np.concatenate(u_s), np.concatenate(u_b)
    b, a = np.polyfit(th_s, u_s, 1)
    resid = u_s - (a + b * th_s)
    return a, b, float(np.median(u_b)), float(np.std(resid))


def planta_j3(p, est):
    """Diccionario de la planta (formato de tools/diseno_velocidad.py) con los parámetros p y la recta est."""
    tau, k, bp, bn = p
    a, b, u4n = est[:3]
    return dict(dv.PLANTAS["j3"], tau=tau, K=k, c0=a - V_BARRIDO / k, c1=b, cn=abs(u4n) - V_BARRIDO / k, bp=bp, bn=bn)


def replay(p, est, u, pos0):
    """Aplica la secuencia de duty u (uno por periodo de 10 ms) a la planta. Devuelve la velocidad real y la medida
    (el firmware mide al principio de cada periodo y aplica el duty calculado con esa medida)."""
    pl = planta_j3(p, est)
    w, pos = 0.0, pos0
    hist = [int(np.floor(pos0 * CUENTAS_POR_GRADO))] * (VENTANA + 1)
    W, WM = np.empty(len(u)), np.empty(len(u))
    for i, uk in enumerate(u):
        hist = hist[1:] + [int(np.floor(pos * CUENTAS_POR_GRADO))]
        WM[i] = (hist[-1] - hist[0]) / CUENTAS_POR_GRADO / (VENTANA * TS)
        W[i] = w
        w, pos = dv.planta(pl, w, pos, uk)
    return W, WM


def error_medio(p, est, datos):
    if not (0.01 < p[0] < 0.3) or p[1] <= 0.2 or min(p[2], p[3]) < 0:
        return 1e9
    # el CSV da la velocidad medida al final de cada periodo: la del modelo, un periodo después
    e = [np.abs(replay(p, est, d["u"], d["pos"][0])[1][1:] - d["w_med"][:-1]) for d in datos]
    return float(np.mean(np.concatenate(e)))


def parametros():
    """Corre las dos etapas y devuelve el diccionario de la planta de J3 (lo usan los otros scripts)."""
    barridos = [t for n in BARRIDOS for t in cargar(n)]
    escalones = [t for n in ESCALONES for t in cargar(n)]
    est = estatica(barridos)
    r = minimize(error_medio, INICIAL, args=(est, escalones), method="Nelder-Mead",
                 options={"maxiter": 600, "xatol": 1e-3, "fatol": 1e-3})
    tau, k, bp, bn = r.x
    a, b, u4n, ruido = est
    return {"tau": tau, "K": k, "c0": a - V_BARRIDO / k, "c1": b, "cn": abs(u4n) - V_BARRIDO / k,
            "bp": bp, "bn": bn, "a": a, "u4n": u4n, "ruido_estatica": ruido,
            "error": r.fun, "error_inicial": error_medio(INICIAL, est, escalones),
            "barridos": barridos, "escalones": escalones, "est": est, "p": r.x}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()

    m = parametros()
    print(f"Etapa 1 (barridos a ±{V_BARRIDO:.0f} °/s): duty para subir = {m['a']:.1f} + {m['c1']:.2f}·θ %  "
          f"(dispersión ±{m['ruido_estatica']:.1f} %), para bajar = {m['u4n']:.1f} %")
    print(f"Etapa 2 (escalones de velocidad): error medio de la velocidad {m['error_inicial']:.2f} °/s con los valores "
          f"iniciales -> {m['error']:.2f} °/s ajustado")
    print("\nModelo de J3:")
    print(f"  tau = {m['tau']*1000:.0f} ms")
    print(f"  K   = {m['K']:.2f} (°/s)/%")
    print(f"  subiendo se mueve con u > c+(θ) = {m['c0']:.1f} + {m['c1']:.2f}·θ %; quieta despega con {m['bp']:.1f} % más")
    print(f"  bajando  se mueve con u < -c- = -{m['cn']:.1f} %; quieta despega con {m['bn']:.1f} % más")
    print("  (copiar a PLANTAS['j3'] de tools/diseno_velocidad.py)")

    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(12, 11))
    # 1) Estática
    for d in m["barridos"]:
        col = "#e8a87c" if "194419" in d["nombre"] else "#c2410c"
        sel = np.abs(d["w_med"]) > 1
        ax[0].scatter(d["pos"][sel], d["u"][sel], s=2, color=col, alpha=0.5)
    ax[0].scatter([], [], s=8, color="#e8a87c", label="barrido 19:44 (placa)")
    ax[0].scatter([], [], s=8, color="#c2410c", label="barrido 19:56 (placa)")
    th = np.linspace(-12, 32, 50)
    ax[0].plot(th, m["a"] + m["c1"] * th, color="#1d64c8", lw=2,
               label=f"recta: subir a 4 °/s = {m['a']:.1f} + {m['c1']:.2f}·θ %")
    ax[0].plot(th, np.full_like(th, m["u4n"]), color="#0e7f74", lw=2, label=f"bajar a 4 °/s = {m['u4n']:.1f} %")
    ax[0].set_title("Etapa 1: duty necesario según el ángulo (barridos de carga a ±4 °/s)")
    ax[0].set_xlabel("ángulo de J3 desde la marca [°]")
    ax[0].set_ylabel("duty [%]")
    # 2) y 3) Dinámica: dos escalones de velocidad, medida contra el modelo con el mismo duty
    for a, nombre in zip(ax[1:], ("escalon_vel_20261003_190957", "escalon_vel_20261003_184307")):
        d = next(x for x in m["escalones"] if x["nombre"] == nombre)
        _, wm = replay(m["p"], m["est"], d["u"], d["pos"][0])
        a.plot(d["t"], d["w_ref"], "k--", lw=0.8, label="pedida")
        a.plot(d["t"], d["w_med"], color="#c2410c", lw=1, label="medida en la placa")
        a.plot(d["t"][:-1], wm[1:], color="#1d64c8", lw=1.4, label="modelo con el mismo duty")
        a2 = a.twinx()
        a2.plot(d["t"], d["u"], color="gray", lw=0.6, alpha=0.6)
        a2.set_ylabel("duty aplicado [%] (gris)")
        hora = nombre[-6:]
        a.set_title(f"Etapa 2: escalón de velocidad ±10 °/s de las {hora[:2]}:{hora[2:4]}: velocidad")
        a.set_xlabel("tiempo [s]")
        a.set_ylabel("°/s")
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(fontsize=8, loc="upper right")
    fig.suptitle(f"J3 · Identificación del modelo: τ = {m['tau']*1000:.0f} ms, K = {m['K']:.2f} (°/s)/%, "
                 f"c+ = {m['c0']:.1f} + {m['c1']:.2f}·θ %, c- = {m['cn']:.1f} %, despegue +{m['bp']:.1f} / -{m['bn']:.1f} %")
    fig.tight_layout()
    png = DIR_RESULTADOS / f"identificacion_j3_{datetime.now():%Y%m%d_%H%M%S}.png"
    fig.savefig(png, dpi=110)
    print(f"\nGráfica: {png.relative_to(DIR)}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
