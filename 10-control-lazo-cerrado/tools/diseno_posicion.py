#!/usr/bin/env python3
"""Paso 3 de J1: diseño del P de posición de la base, encima del PI de velocidad, y simulación.

Cascada (ARQUITECTURA.md §5):
    ángulo pedido -> [P de posición, 50 Hz] -> velocidad pedida -> [PI de velocidad, 100 Hz] -> duty -> motor

El lazo de velocidad cerrado se aproxima como un primer orden 1 / (tau_v s + 1). En placa (paso 2, Ki = 4)
tardó ts ~0,35 s en establecerse, así que tau_v ~ ts / 4 ~ 0,09 s (incluye el atraso de la medición).
Con un P de posición Kpp, el lazo de posición queda:
    theta / theta_ref = Kpp / (tau_v s^2 + s + Kpp)
    wn = sqrt(Kpp / tau_v),   zeta = 1 / (2 sqrt(Kpp tau_v))
Sin sobrepico hace falta zeta >= 1, o sea Kpp <= 1 / (4 tau_v).

Para movimientos grandes la velocidad pedida se limita a v_max y su cambio a a_max (perfil
trapezoidal: acelera, va constante, frena), así no hay golpes contra el juego de la correa. Cerca del
objetivo, dentro de una tolerancia, la velocidad pedida es 0 (freno): sin eso el motor "zumbaría"
alrededor del objetivo por la zona muerta y el juego.

La simulación realista usa el mismo PI de velocidad del firmware (zona muerta, anti-windup, freno con
velocidad pedida 0, velocidad medida en 4 muestras de las cuentas enteras del encoder).

Uso:
    python3 tools/diseno_posicion.py
    python3 tools/diseno_posicion.py --kpp 2.5 --vmax 30 --amax 60 --tol 0.3
"""
import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

import diseno_velocidad as dv

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"

TAU_V = 0.09        # s, lazo de velocidad cerrado medido en el paso 2 (ts ~0,35 s / 4)
KP_VEL = 0.549      # PI de velocidad elegido en el paso 2
KI_VEL = 4.0
TS_POS = 0.02       # s, lazo de posición a 50 Hz


# Protecciones del firmware (config.h), para que la simulación corte donde cortaría la placa
SIN_CUENTAS_S = 0.15
CONTRARIO_VEL, CONTRARIO_S = 5.0, 0.10
ATASCO_VEL, ATASCO_DUTY, ATASCO_S = 2.0, 35.0, 0.30


def simular(kpp, vmax, amax, tol, perfil, t_fin):
    """Cascada completa con las protecciones del firmware.
    Devuelve t, theta_ref, theta (base), w_ref, w, duty y la causa del corte (None si no cortó).
    Si corta, desde ahí el duty es 0, como en la placa."""
    dt = 0.0005
    n_sub = int(round(dv.TS / dt))
    pasos = int(round(t_fin / dv.TS))
    n_pos = int(round(TS_POS / dv.TS))
    w = pos = 0.0
    integral = 0.0
    w_ref = 0.0
    hist = [0] * (4 + 1)
    T, TR, TH, WR, W, U = [], [], [], [], [], []
    corte = None
    sin_c = contra = atasco = 0
    for k in range(pasos):
        t = k * dv.TS
        cuentas = int(np.floor(pos * dv.CUENTAS_POR_GRADO))
        hist = hist[1:] + [cuentas]
        theta_med = cuentas / dv.CUENTAS_POR_GRADO
        w_med = (hist[-1] - hist[0]) / dv.CUENTAS_POR_GRADO / (4 * dv.TS)
        th_ref = perfil(t)
        # Lazo de posición (cada n_pos periodos del de velocidad)
        if k % n_pos == 0:
            e = th_ref - theta_med
            deseada = 0.0 if abs(e) < tol else float(np.clip(kpp * e, -vmax, vmax))
            dmax = amax * TS_POS
            w_ref = deseada if deseada == 0.0 else float(np.clip(deseada, w_ref - dmax, w_ref + dmax))
        # Lazo de velocidad: el mismo PI del firmware
        if corte:
            w_ref = 0.0
        if w_ref == 0.0:
            u, integral = 0.0, 0.0
        else:
            ev = w_ref - w_med
            u_lib = np.sign(w_ref) * dv.ZONA_MUERTA + KP_VEL * ev + integral
            u = float(np.clip(u_lib, -dv.DUTY_MAX, dv.DUTY_MAX))
            if u == u_lib or np.sign(ev) != np.sign(u_lib):
                integral += KI_VEL * dv.TS * ev
        # Protecciones, con las mismas condiciones que cascada() en el firmware
        if not corte:
            sin_c = sin_c + 1 if (w_ref != 0 and abs(u) > dv.ZONA_MUERTA and hist[-1] == hist[-2]) else 0
            contra = contra + 1 if (w_ref != 0 and w_med * np.sign(w_ref) < -CONTRARIO_VEL) else 0
            atasco = atasco + 1 if (abs(w_med) < ATASCO_VEL and abs(u) >= ATASCO_DUTY) else 0
            if sin_c * dv.TS >= SIN_CUENTAS_S - 1e-9:
                corte = f"sin cuentas del encoder (t = {t:.2f} s)"
            elif contra * dv.TS >= CONTRARIO_S - 1e-9:
                corte = f"se mueve al revés de lo pedido (t = {t:.2f} s)"
            elif atasco * dv.TS >= ATASCO_S - 1e-9:
                corte = f"atasco (t = {t:.2f} s)"
            if corte:
                u, integral, w_ref = 0.0, 0.0, 0.0
        T.append(t), TR.append(th_ref), TH.append(pos), WR.append(w_ref), W.append(w), U.append(u)
        u_ef = 0.0 if abs(u) <= dv.ZONA_MUERTA else u - np.sign(u) * dv.ZONA_MUERTA
        for _ in range(n_sub):
            w += dt * (-w + dv.K * u_ef) / dv.TAU
            pos += dt * w
    return tuple(np.array(x) for x in (T, TR, TH, WR, W, U)) + (corte,)


def medir(t, th, ref, t0, t1, desde):
    """Tiempo de llegada (entrar y quedarse a menos de 0,5°), sobrepico en ° y error final."""
    m = (t >= t0) & (t < t1)
    tt, x = t[m] - t0, th[m]
    fuera = np.where(np.abs(x - ref) > 0.5)[0]
    ts = tt[fuera[-1] + 1] if len(fuera) and fuera[-1] + 1 < len(tt) else (0.0 if not len(fuera) else float("nan"))
    sentido = np.sign(ref - desde)
    sobrepico = max(0.0, float(np.max((x - ref) * sentido)))
    return ts, sobrepico, ref - x[-1]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--kpp", type=float, default=2.0, help="P de posición, (°/s)/° = 1/s (def. 2)")
    ap.add_argument("--vmax", type=float, default=30.0, help="velocidad máxima, °/s (def. 30)")
    ap.add_argument("--amax", type=float, default=60.0, help="aceleración máxima, °/s² (def. 60)")
    ap.add_argument("--tol", type=float, default=0.3, help="tolerancia de llegada, ° (def. 0,3)")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()

    kpp_max = 1 / (4 * TAU_V)
    zeta = 1 / (2 * np.sqrt(args.kpp * TAU_V))
    wn = np.sqrt(args.kpp / TAU_V)
    print("== Diseño del P de posición de J1 (sobre el PI de velocidad del paso 2) ==")
    print(f"Lazo de velocidad cerrado ~ 1/({TAU_V:.2f} s + 1); PI Kp = {KP_VEL}, Ki = {KI_VEL}")
    print(f"Sin sobrepico: Kpp <= 1/(4 tau_v) = {kpp_max:.2f} 1/s")
    print(f"Kpp = {args.kpp:.2f} 1/s -> zeta = {zeta:.2f} ({'sin' if zeta >= 1 else 'con'} sobrepico), "
          f"wn = {wn:.1f} rad/s")
    print(f"Perfil: v_max = {args.vmax:.0f} °/s, a_max = {args.amax:.0f} °/s², tolerancia ±{args.tol:.2f}°")
    print(f"Con error > {args.vmax / args.kpp:.0f}° va a velocidad máxima; debajo, frena en proporción al error")

    saltos = [(0.2, 20.0), (3.2, -20.0), (6.2, 0.0), (9.2, 5.0), (11.2, 0.0)]
    perfil = lambda t: next((v for t0, v in reversed(saltos) if t >= t0), 0.0)
    t, tr, th, wr, w, u, corte = simular(args.kpp, args.vmax, args.amax, args.tol, perfil, 13.2)
    if corte:
        print(f"\n*** La placa CORTARÍA por {corte}: estos parámetros no sirven así. ***")

    print("\nEscalón             llegada (±0,5°)   sobrepico   error final")
    desde = 0.0
    for i, (t0, ref) in enumerate(saltos):
        t1 = saltos[i + 1][0] if i + 1 < len(saltos) else t[-1] + dv.TS
        ts, mp, e = medir(t, th, ref, t0, t1, desde)
        print(f"  {desde:+5.0f}° -> {ref:+5.0f}°      {ts:6.2f} s         {mp:5.2f}°     {e:+6.2f}°")
        desde = ref

    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(10, 8.5), sharex=True, height_ratios=[2, 1.2, 1])
    ax[0].plot(t, tr, "k--", lw=1.2, label="ángulo pedido")
    ax[0].plot(t, th, color="#1d64c8", lw=2, label="ángulo de la base (simulado)")
    ax[0].set_ylabel("ángulo [°]")
    ax[0].set_title(f"J1 · P de posición Kpp = {args.kpp:.2f} sobre PI de velocidad "
                    f"(v_max {args.vmax:.0f} °/s, a_max {args.amax:.0f} °/s², tol ±{args.tol}°)")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=9)
    ax[1].plot(t, wr, "k--", lw=1, label="velocidad pedida (sale del P)")
    ax[1].plot(t, w, color="#1d64c8", lw=1.5, label="velocidad de la base")
    ax[1].set_ylabel("°/s")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=9)
    ax[2].plot(t, u, color="#c2410c", lw=1.2)
    ax[2].set_ylabel("duty [%]")
    ax[2].set_xlabel("tiempo [s]")
    ax[2].grid(alpha=0.3)
    fig.tight_layout()
    DIR_RESULTADOS.mkdir(exist_ok=True)
    png = DIR_RESULTADOS / f"diseno_posicion_{datetime.now():%Y%m%d_%H%M%S}.png"
    fig.savefig(png, dpi=120)
    print(f"\nGráfica: {png.relative_to(DIR_RESULTADOS.parent)}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
