#!/usr/bin/env python3
"""Paso 2: diseño teórico del PI de velocidad y simulación del escalón, para J1 (base) o J3 (codo).

Modelo de la planta (velocidad w [°/s] de la articulación en función del duty u [%]):
    tau * dw/dt = -w + K * (u - c+(theta))   si u > c+(theta)
    tau * dw/dt = -w + K * (u + c-)          si u < -c-
    tau * dw/dt = -w                         en el medio (zona muerta: frena hasta 0 y no invierte)
    c+(theta) = c0 + c1 * theta;  quieta, para despegar hace falta b+ (subiendo) o b- (bajando) más.
  - J1: K = 1,58 rpm/% de la salida de la reductora = 1,58 * 6 / 5 = 1,90 (°/s de la base)/% (E5, paso 1),
    tau = 65 ms, zona muerta simétrica de 17 % (c0 = c- = 17, c1 = 0) y despega con 25 %.
  - J3: identificado con los datos guardados (tools/identificar_j3.py): una sola K, y el peso del antebrazo
    corre la zona muerta subiendo según el ángulo (c1 > 0); bajando el peso ayuda y solo queda la fricción.

Diseño por cancelación de polo: PI con Ti = tau. El cero del PI cancela el polo del motor y el lazo
cerrado queda de primer orden, sin sobrepico:
    w / w_ref = 1 / (tau_lc * s + 1),  tau_lc = tau / (Kp * K),  ts(2 %) = 4 * tau_lc
De la especificación de ts sale Kp, y Ki = Kp / Ti.

Simula dos casos: el ideal (lineal, velocidad medida perfecta) y el realista: el mismo PI del firmware
(pasoPI en src/main.cpp: feedforward de marcha y de despegue con rampa, compensación de gravedad, anti-windup,
duty saturado, freno con velocidad pedida 0), muestreo a 100 Hz y velocidad medida con las cuentas enteras del
encoder en una ventana. Guarda el PNG en resultados/.

Uso:
    python3 tools/diseno_velocidad.py                       # J1, especificaciones por defecto
    python3 tools/diseno_velocidad.py --ts 0.3 --wref 20
    python3 tools/diseno_velocidad.py -a j3 --wref 10       # J3: diseño teórico
    python3 tools/diseno_velocidad.py -a j3 --wref 10 --kp 0.35 --ki 5   # simular las ganancias elegidas
    python3 tools/diseno_velocidad.py --ventana 2           # probar otra ventana de medición
"""
import argparse
from datetime import datetime
from pathlib import Path

import numpy as np

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"

TS = 0.01                         # s, lazo de velocidad a 100 Hz
VEL_QUIETO = 1.0                  # °/s, como el firmware: debajo, feedforward de despegue
RAMPA_DESPEGUE = 300.0            # %/s, del feedforward de marcha al de despegue
FF_MIN_SUBIENDO = 8.0             # %, piso del feedforward subiendo (FF_MIN_PCT del firmware)

# Planta y firmware de cada articulación (config.h). ff = (despegue+, marcha+, despegue-, marcha-, %/° subiendo).
PLANTAS = {
    "j1": dict(nombre="J1", pieza="base",
               K=1.58 * 6.0 / (90 / 18),                  # (°/s de la base) por %
               tau=0.065, c0=17.0, c1=0.0, cn=17.0, bp=8.0, bn=8.0,  # despega con 17 + 8 = 25 %
               cuentas_por_grado=16.0 * 4 * 50 * (90 / 18) / 360.0,
               duty_max=40.0, arranque=25.0, atasco_duty=35.0,
               kp=0.549, ki=4.0, ff=(17.0, 17.0, 17.0, 17.0, 0.0), vmin_baja=3.0, trabada_ms=100,
               tau_v=0.09),               # lazo de velocidad cerrado, medido en el paso 2 (ts ~0,35 s / 4)
    # J3: tools/identificar_j3.py (2026-10-04) con 8 escalones de velocidad y 2 barridos de carga
    "j3": dict(nombre="J3", pieza="codo",
               K=1.62, tau=0.053, c0=23.6, c1=1.42, cn=3.2, bp=5.9, bn=6.1,
               cuentas_por_grado=1000 * 4 * (56 / 18) / 360.0,
               duty_max=80.0, arranque=20.0, atasco_duty=70.0,
               kp=0.35, ki=5.0, ff=(31.0, 24.0, 14.0, 9.0, 1.4), vmin_baja=6.0, trabada_ms=40,
               tau_v=None),               # se calcula con la simulación ideal del lazo de velocidad
}


def disenar(ts, pl):
    """Kp y Ki para que el lazo cerrado tenga tiempo de establecimiento ts (criterio del 2 %)."""
    tau_lc = ts / 4.0
    kp = pl["tau"] / (pl["K"] * tau_lc)
    ki = kp / pl["tau"]
    return kp, ki, tau_lc


def planta(pl, w, pos, u, dt=0.0005):
    """El motor durante un periodo de muestreo con el duty u fijo. Devuelve la velocidad y la posición nuevas."""
    cp = pl["c0"] + pl["c1"] * pos
    cn = pl["cn"]
    u_ef = pl["K"] * (u - cp) if u > cp else (pl["K"] * (u + cn) if u < -cn else 0.0)
    if abs(w) < 0.05 and not (u >= cp + pl["bp"] or u <= -(cn + pl["bn"])):
        w, u_ef = 0.0, 0.0                  # quieta: sin la fuerza de despegue no se mueve
    for _ in range(int(round(TS / dt))):
        w_ant = w
        w += dt * (-w + u_ef) / pl["tau"]
        if u_ef == 0.0 and w_ant * w < 0:   # frenando sin empuje: se detiene, no invierte
            w = 0.0
        pos += dt * w
    return w, pos


class PI:
    """El PI de velocidad del firmware (pasoPI en src/main.cpp)."""

    def __init__(self, kp, ki, ff, duty_max, trabada_ms):
        self.kp, self.ki, self.duty_max = kp, ki, duty_max
        self.desp_p, self.marcha_p, self.desp_n, self.marcha_n, self.pend = ff
        self.trabada = int(round(trabada_ms / 1000 / TS))
        self.integral = 0.0
        self.rampa, self.despegada, self.trab, self.signo_ant = 0.0, False, 0, 0

    def paso(self, w_ref, w_med, th):
        signo = int(np.sign(w_ref))
        if signo != self.signo_ant:
            self.rampa, self.despegada, self.trab, self.signo_ant = 0.0, False, 0, signo
        if w_ref == 0.0:
            self.integral = 0.0
            return 0.0
        e = w_ref - w_med
        quieta = abs(w_med) < VEL_QUIETO
        if self.despegada:
            self.trab = self.trab + 1 if quieta else 0
            if self.trab >= self.trabada:
                self.despegada, self.rampa = False, 0.0
        elif not quieta:
            self.despegada, self.trab = True, 0
        extra = self.pend * th if signo > 0 else 0.0
        marcha = max((self.marcha_p if signo > 0 else self.marcha_n) + extra, FF_MIN_SUBIENDO if signo > 0 else 0.0)
        desp = max((self.desp_p if signo > 0 else self.desp_n) + extra, marcha)
        if not self.despegada:
            self.rampa = min(self.rampa + RAMPA_DESPEGUE * TS, max(desp - marcha, 0.0))
        ff = signo * (marcha + (0.0 if self.despegada else self.rampa))
        u_lib = ff + self.kp * e + self.integral
        u = float(np.clip(u_lib, -self.duty_max, self.duty_max))
        if u == u_lib or np.sign(e) != np.sign(u_lib):
            self.integral += self.ki * TS * e
        return u


def simular(kp, ki, perfil, t_fin, realista, ventana, pl=None, ff=None, pos0=0.0):
    """Simula el lazo. perfil(t) da la velocidad pedida. Devuelve t, w_ref, w, w_medida, u.
    ff: feedforward (despegue+, marcha+, despegue-, marcha-, %/°); por defecto el de config.h."""
    pl = pl or PLANTAS["j1"]
    cpg = pl["cuentas_por_grado"]
    pasos = int(round(t_fin / TS))
    w, pos = 0.0, pos0
    hist = [int(np.floor(pos0 * cpg))] * (ventana + 1)
    pi = PI(kp, ki, ff or pl["ff"], pl["duty_max"], pl["trabada_ms"])
    integral = 0.0
    T, WREF, W, WMED, U = [], [], [], [], []
    for k in range(pasos):
        t = k * TS
        w_ref = perfil(t)
        if realista:
            cuentas = int(np.floor(pos * cpg))
            hist = hist[1:] + [cuentas]
            w_med = (hist[-1] - hist[0]) / cpg / (ventana * TS)
            u = pi.paso(w_ref, w_med, cuentas / cpg)
        else:                                  # lineal: sin zona muerta, sin saturación, medida perfecta
            w_med = w
            e = w_ref - w_med
            u = kp * e + integral
            integral += ki * TS * e
        T.append(t), WREF.append(w_ref), W.append(w), WMED.append(w_med), U.append(u)
        if realista:
            w, pos = planta(pl, w, pos, u)
        else:
            for _ in range(20):
                w += 0.0005 * (-w + pl["K"] * u) / pl["tau"]
                pos += 0.0005 * w
    return np.array(T), np.array(WREF), np.array(W), np.array(WMED), np.array(U)


def banda_min(pl):
    """La velocidad medida va de a saltos de una cuenta en la ventana (0,56 °/s con 4 muestras en J1): una banda
    del 2 % de 20 °/s (0,4 °/s) es más chica que un salto y el tiempo de establecimiento saldría absurdo.
    Por eso la banda es el 2 % o dos saltos de medición, lo que sea mayor."""
    return 2.0 / pl["cuentas_por_grado"] / (4 * TS)


def medir_escalon(t, w, w_ref_val, t0, t1, pl=None):
    """Tiempo de establecimiento, sobrepico y error final del escalón entre t0 y t1."""
    pl = pl or PLANTAS["j1"]
    m = (t >= t0) & (t < t1)
    tt, ww = t[m] - t0, w[m]
    final = ww[-int(0.1 / TS):].mean()
    banda = max(0.02 * abs(w_ref_val), banda_min(pl))
    fuera = np.where(np.abs(ww - w_ref_val) > banda)[0]
    ts = tt[fuera[-1] + 1] if len(fuera) and fuera[-1] + 1 < len(tt) else (0.0 if not len(fuera) else float("nan"))
    sobrepico = max(0.0, (ww.max() - w_ref_val) / abs(w_ref_val) * 100) if w_ref_val > 0 else 0.0
    return ts, sobrepico, w_ref_val - final


def tau_lazo_velocidad(pl, kp, ki):
    """Constante de tiempo equivalente (primer orden) del lazo de velocidad cerrado ideal: el tiempo en que un
    escalón llega al 63,2 % del valor final."""
    t, _, w, _, _ = simular(kp, ki, lambda tt: 0.0 if tt < 0.1 else 10.0, 3.1, False, 4, pl)
    return float(t[np.argmax(w >= 6.32)] - 0.1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-a", "--articulacion", choices=sorted(PLANTAS), default="j1", help="def. j1")
    ap.add_argument("--ts", type=float, default=0.25, help="tiempo de establecimiento pedido, s (def. 0,25)")
    ap.add_argument("--wref", type=float, default=20.0, help="velocidad del escalón, °/s (def. 20)")
    ap.add_argument("--kp", type=float, help="simular este Kp en lugar del de diseño")
    ap.add_argument("--ki", type=float, help="simular este Ki en lugar del de diseño")
    ap.add_argument("--ventana", type=int, default=4, help="muestras para medir la velocidad (def. 4 = 40 ms)")
    ap.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    args = ap.parse_args()
    pl = PLANTAS[args.articulacion]
    J = pl["nombre"]

    kp_d, ki_d, tau_lc = disenar(args.ts, pl)
    kp = args.kp if args.kp is not None else kp_d
    ki = args.ki if args.ki is not None else ki_d
    elegidas = args.kp is not None or args.ki is not None
    print(f"== Diseño del PI de velocidad de {J} (cancelación de polo) ==")
    pendiente = f" + {pl['c1']:.2f}·θ" if pl["c1"] else ""
    print(f"Planta: K = {pl['K']:.3f} (°/s)/%, tau = {pl['tau']*1000:.0f} ms, zona muerta: subiendo "
          f"{pl['c0']:.1f}{pendiente} %, bajando {pl['cn']:.1f} %")
    print(f"Especificación: ts(2 %) = {args.ts:.2f} s, sobrepico 0 %, error final 0")
    print(f"Lazo cerrado: tau_lc = {tau_lc*1000:.1f} ms")
    print(f"=> Kp = {kp_d:.3f} %/(°/s)   Ki = {ki_d:.2f} %/°   (Ti = tau = {pl['tau']*1000:.0f} ms)")
    if elegidas:
        print(f"Se simulan las ganancias elegidas: Kp = {kp:.3f}, Ki = {ki:.2f}")
    print(f"Duty necesario para {args.wref:.0f} °/s en 0°: subiendo {pl['c0'] + args.wref / pl['K']:.1f} %, "
          f"bajando {-pl['cn'] - args.wref / pl['K']:.1f} % (máx. {pl['duty_max']:.0f} %)")
    print(f"Resolución de velocidad medida: {1 / pl['cuentas_por_grado'] / (args.ventana * TS):.2f} °/s "
          f"(1 cuenta en {args.ventana * TS * 1000:.0f} ms)")

    w0 = args.wref
    perfil = lambda t: 0.0 if t < 0.1 else (w0 if t < 1.1 else (-w0 if t < 2.1 else 0.0))
    t_fin = 2.6
    ideal = simular(kp, ki, perfil, t_fin, realista=False, ventana=args.ventana, pl=pl)
    real = simular(kp, ki, perfil, t_fin, realista=True, ventana=args.ventana, pl=pl)

    print("\nEscalón              ts(2 %)    sobrepico   error final")
    for sg, t0, nombre_esc in ((1, 0.1, f"0 -> +{w0:.0f} °/s"), (-1, 1.1, f"+{w0:.0f} -> -{w0:.0f} °/s")):
        for nombre, (t, wr, w, wm, u) in (("ideal", ideal), ("realista", real)):
            ts, mp, ess = medir_escalon(t, sg * w, w0, t0, t0 + 1.0, pl)
            print(f"  {nombre_esc:16s} {nombre:9s} {ts:6.3f} s   {mp:6.1f} %    {ess:+6.2f} °/s")

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
    ax1.set_ylabel(f"velocidad de{'l' if J == 'J3' else ' la'} {pl['pieza']} [°/s]")
    origen = "elegidas" if elegidas else f"ts pedido {args.ts:.2f} s"
    ax1.set_title(f"{J} · PI de velocidad: Kp = {kp:.3f}, Ki = {ki:.2f} ({origen})")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="upper right", fontsize=9)
    ax2.plot(t, u, color="#1d64c8", lw=1.5, label="ideal")
    ax2.plot(tr, ur, color="#c2410c", lw=1.5, label="realista (con zona muerta" + (" y peso)" if pl["c1"] else ")"))
    ax2.axhline(pl["duty_max"], color="gray", lw=0.8, ls=":")
    ax2.axhline(-pl["duty_max"], color="gray", lw=0.8, ls=":")
    ax2.set_ylabel("duty [%]")
    ax2.set_xlabel("tiempo [s]")
    ax2.grid(alpha=0.3)
    ax2.legend(loc="upper right", fontsize=9)
    fig.tight_layout()

    DIR_RESULTADOS.mkdir(exist_ok=True)
    pre = "" if J == "J1" else f"{args.articulacion}_"
    png = DIR_RESULTADOS / f"diseno_velocidad_{pre}{datetime.now():%Y%m%d_%H%M%S}.png"
    fig.savefig(png, dpi=120)
    print(f"\nGráfica: {png.relative_to(DIR_RESULTADOS.parent)}")
    if not args.sin_ventana:
        plt.show()


if __name__ == "__main__":
    main()
