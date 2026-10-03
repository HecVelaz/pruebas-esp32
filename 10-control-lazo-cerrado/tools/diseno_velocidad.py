#!/usr/bin/env python3
"""Paso 2 de J1: diseño teórico del PI de velocidad de la base y simulación del escalón.

Modelo (E5, verificado en el paso 1): velocidad de la base w [°/s] en función del duty u [%]
    tau * dw/dt = -w + K * (u - zona muerta)
con K = 1,58 rpm/% de la salida de la reductora = 1,58 * 6 / 5 = 1,90 (°/s de la base)/%.

Diseño por cancelación de polo: PI con Ti = tau. El cero del PI cancela el polo del motor y el lazo
cerrado queda de primer orden, sin sobrepico:
    w / w_ref = 1 / (tau_lc * s + 1),  tau_lc = tau / (Kp * K),  ts(2 %) = 4 * tau_lc
De la especificación de ts sale Kp, y Ki = Kp / Ti.

Simula dos casos: el ideal (lineal, velocidad medida perfecta) y el realista (zona muerta con su
compensación, PWM saturado, muestreo a 100 Hz y velocidad medida con las cuentas enteras del
encoder en una ventana). Guarda el PNG en resultados/.

Uso:
    python3 tools/diseno_velocidad.py                 # especificaciones por defecto
    python3 tools/diseno_velocidad.py --ts 0.3 --wref 20
    python3 tools/diseno_velocidad.py --ventana 2     # probar otra ventana de medición
"""
import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"

# Planta (E5 y paso 1)
K_RPM = 1.58                      # rpm de la salida de la reductora por % de duty
RELACION_CORREA = 90 / 18
K = K_RPM * 6.0 / RELACION_CORREA  # (°/s de la base) por %
TAU = 0.065                       # s
ZONA_MUERTA = 17.0                # %, en marcha (paso 1: 30 % -> ~25 °/s)
CUENTAS_POR_GRADO = 16.0 * 4 * 50 * RELACION_CORREA / 360.0

# Firmware
TS = 0.01                         # s, lazo de velocidad a 100 Hz
DUTY_MAX = 40.0                   # %, límite de estas pruebas


def disenar(ts):
    """Kp y Ki para que el lazo cerrado tenga tiempo de establecimiento ts (criterio del 2 %)."""
    tau_lc = ts / 4.0
    kp = TAU / (K * tau_lc)
    ki = kp / TAU
    return kp, ki, tau_lc


def simular(kp, ki, perfil, t_fin, realista, ventana):
    """Simula el lazo. perfil(t) da la velocidad pedida. Devuelve t, w_ref, w, w_medida, u."""
    dt = 0.0005                   # paso de integración del motor
    n_sub = int(round(TS / dt))
    pasos = int(round(t_fin / TS))
    w = 0.0                       # velocidad real de la base, °/s
    pos = 0.0                     # posición real, °
    integral = 0.0
    hist_cuentas = [0] * (ventana + 1)
    u = 0.0
    T, WREF, W, WMED, U = [], [], [], [], []
    for k in range(pasos):
        t = k * TS
        w_ref = perfil(t)
        # Medición
        if realista:
            cuentas = int(np.floor(pos * CUENTAS_POR_GRADO))
            hist_cuentas = hist_cuentas[1:] + [cuentas]
            w_med = (hist_cuentas[-1] - hist_cuentas[0]) / CUENTAS_POR_GRADO / (ventana * TS)
        else:
            w_med = w
        # PI con compensación de la zona muerta y anti-windup (no integra si está saturado
        # y el error lo empujaría más hacia la saturación)
        e = w_ref - w_med
        ff = np.sign(w_ref) * ZONA_MUERTA if (realista and w_ref != 0) else 0.0
        u_lib = ff + kp * e + integral
        if realista and w_ref == 0:
            # Igual que el firmware: con velocidad pedida 0 frena (duty 0) y borra la integral
            u = 0.0
            integral = 0.0
        elif realista:
            u = float(np.clip(u_lib, -DUTY_MAX, DUTY_MAX))
            if u == u_lib or np.sign(e) != np.sign(u_lib):
                integral += ki * TS * e
        else:
            u = u_lib
            integral += ki * TS * e
        T.append(t), WREF.append(w_ref), W.append(w), WMED.append(w_med), U.append(u)
        # Motor durante un periodo de muestreo (el duty queda fijo)
        if realista:
            u_ef = 0.0 if abs(u) <= ZONA_MUERTA else u - np.sign(u) * ZONA_MUERTA
        else:
            u_ef = u
        for _ in range(n_sub):
            w += dt * (-w + K * u_ef) / TAU
            pos += dt * w
    return np.array(T), np.array(WREF), np.array(W), np.array(WMED), np.array(U)


# La velocidad medida va de a saltos de una cuenta en la ventana (0,56 °/s con 4 muestras): una banda
# del 2 % de 20 °/s (0,4 °/s) es más chica que un salto y el tiempo de establecimiento saldría absurdo.
# Por eso la banda es el 2 % o dos saltos de medición, lo que sea mayor.
BANDA_MIN = 2.0 / CUENTAS_POR_GRADO / (4 * TS)


def medir_escalon(t, w, w_ref_val, t0, t1):
    """Tiempo de establecimiento, sobrepico y error final del escalón entre t0 y t1."""
    m = (t >= t0) & (t < t1)
    tt, ww = t[m] - t0, w[m]
    final = ww[-int(0.1 / TS):].mean()
    banda = max(0.02 * abs(w_ref_val), BANDA_MIN)
    fuera = np.where(np.abs(ww - w_ref_val) > banda)[0]
    ts = tt[fuera[-1] + 1] if len(fuera) and fuera[-1] + 1 < len(tt) else (0.0 if not len(fuera) else float("nan"))
    sobrepico = max(0.0, (ww.max() - w_ref_val) / abs(w_ref_val) * 100) if w_ref_val > 0 else 0.0
    return ts, sobrepico, w_ref_val - final


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ts", type=float, default=0.25, help="tiempo de establecimiento pedido, s (def. 0,25)")
    ap.add_argument("--wref", type=float, default=20.0, help="velocidad del escalón, °/s de la base (def. 20)")
    ap.add_argument("--ventana", type=int, default=4, help="muestras para medir la velocidad (def. 4 = 40 ms)")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()

    kp, ki, tau_lc = disenar(args.ts)
    print("== Diseño del PI de velocidad de J1 (cancelación de polo) ==")
    print(f"Planta: K = {K:.3f} (°/s)/%, tau = {TAU*1000:.0f} ms, zona muerta = {ZONA_MUERTA:.0f} %")
    print(f"Especificación: ts(2 %) = {args.ts:.2f} s, sobrepico 0 %, error final 0")
    print(f"Lazo cerrado: tau_lc = {tau_lc*1000:.1f} ms")
    print(f"=> Kp = {kp:.3f} %/(°/s)   Ki = {ki:.2f} %/°   (Ti = tau = {TAU*1000:.0f} ms)")
    print(f"Duty necesario para {args.wref:.0f} °/s: {ZONA_MUERTA + args.wref / K:.1f} % (máx. {DUTY_MAX:.0f} %)")
    print(f"Resolución de velocidad medida: {1 / CUENTAS_POR_GRADO / (args.ventana * TS):.2f} °/s "
          f"(1 cuenta en {args.ventana * TS * 1000:.0f} ms)")

    w0 = args.wref
    perfil = lambda t: 0.0 if t < 0.1 else (w0 if t < 1.1 else (-w0 if t < 2.1 else 0.0))
    t_fin = 2.6
    ideal = simular(kp, ki, perfil, t_fin, realista=False, ventana=args.ventana)
    real = simular(kp, ki, perfil, t_fin, realista=True, ventana=args.ventana)

    print("\nEscalón 0 -> +%.0f °/s     ts(2 %%)    sobrepico   error final" % w0)
    for nombre, (t, wr, w, wm, u) in (("ideal", ideal), ("realista", real)):
        ts, mp, ess = medir_escalon(t, w, w0, 0.1, 1.1)
        print(f"  {nombre:9s}             {ts:6.3f} s   {mp:6.1f} %    {ess:+6.2f} °/s")

    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True, height_ratios=[2, 1])
    t, wr, w, wm, u = ideal
    tr, _, wrl, wml, ur = real
    ax1.plot(t, wr, "k--", lw=1.2, label="pedida")
    ax1.plot(t, w, color="#1d64c8", lw=2, label="ideal (lineal)")
    ax1.plot(tr, wrl, color="#c2410c", lw=1.6, label="realista: velocidad real")
    ax1.plot(tr, wml, color="#c2410c", lw=0.8, alpha=0.5, label=f"realista: medida ({args.ventana} muestras)")
    ax1.set_ylabel("velocidad de la base [°/s]")
    ax1.set_title(f"J1 · PI de velocidad: Kp = {kp:.3f}, Ki = {ki:.2f} (ts pedido {args.ts:.2f} s)")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)
    ax2.plot(t, u, color="#1d64c8", lw=1.5, label="ideal")
    ax2.plot(tr, ur, color="#c2410c", lw=1.5, label="realista (con zona muerta)")
    ax2.axhline(DUTY_MAX, color="gray", lw=0.8, ls=":")
    ax2.axhline(-DUTY_MAX, color="gray", lw=0.8, ls=":")
    ax2.set_ylabel("duty [%]")
    ax2.set_xlabel("tiempo [s]")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=9)
    fig.tight_layout()

    DIR_RESULTADOS.mkdir(exist_ok=True)
    png = DIR_RESULTADOS / f"diseno_velocidad_{datetime.now():%Y%m%d_%H%M%S}.png"
    fig.savefig(png, dpi=120)
    print(f"\nGráfica: {png.relative_to(DIR_RESULTADOS.parent)}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
