#!/usr/bin/env python3
"""Barrido y escalón del motor 36GP-555 en lazo abierto: los ejecuta por el puerto serial,
guarda el CSV en resultados/ y los grafica.

Uso (cerrar antes el monitor de PlatformIO, que ocupa el puerto):
    python3 tools/graficar.py barrido                 # comando 'a' del firmware
    python3 tools/graficar.py escalon 50              # comando 'e50'
    python3 tools/graficar.py perfil                  # comando 'p': escalera +-20..100 %, 5 s por nivel
    python3 tools/graficar.py perfil --segundos 3 --niveles 25 50 75 100
    python3 tools/graficar.py barrido --archivo resultados/barrido_....csv   # volver a graficar
    python3 tools/graficar.py escalon --archivo resultados/escalon_....csv
Opciones: --puerto /dev/ttyACM1 (por defecto /dev/ttyACM0), --sin-ventana (solo guarda el PNG).
Ctrl+C durante la prueba manda 'x' (freno) antes de salir.
"""
import argparse
import csv
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

DIR_RESULTADOS = Path(__file__).resolve().parent.parent / "resultados"
BAUD = 115200

# Fila de la tabla del barrido: "   +50   |     +3900 |      +78.0 |      4160"
RE_FILA_BARRIDO = re.compile(r"^\s*([+-]?\d+)\s*\|\s*([+-]?[\d.]+)\s*\|\s*([+-]?[\d.]+)\s*\|\s*([+-]?[\d.]+)\s*$")
# Fila del CSV del escalón: "t_ms,duty_pct,cuentas,rpm_motor,rpm_salida"
RE_FILA_ESCALON = re.compile(r"^(\d+),([+-]?[\d.]+),([+-]?\d+),([+-]?[\d.]+),([+-]?[\d.]+)$")

# Fila del perfil: "t_ms,duty_pct,cuentas"
RE_FILA_PERFIL = re.compile(r"^(\d+),([+-]?[\d.]+),([+-]?\d+)$")
RE_META_PERFIL = re.compile(r"^# cuentas_por_vuelta_motor=([\d.]+) reduccion=([\d.]+)")

ERRORES = (">> ERROR", ">> Comando desconocido", ">> El escalón", ">> Escalón inválido",
           ">> El perfil", ">> Perfil inválido")


# ---------------------------------------------------------------- serial

def abrir_puerto(puerto):
    """Abre el puerto y espera a que el firmware responda. Lanza serial.SerialException si no se puede abrir
    y RuntimeError si el firmware no contesta."""
    import serial  # solo hace falta para medir, no para volver a graficar

    # Mismos DTR/RTS que el monitor de PlatformIO: así abrir el puerto no reinicia la S3
    ser = serial.Serial(puerto, BAUD, timeout=0.1)
    # Sincronizar pidiendo la ayuda ('?') hasta que llegue. Cubre dos casos:
    #  - si igual se reinició, hay que esperar el arranque (~2 s);
    #  - un resto de texto a medias en la línea del firmware: el primer '?' lo descarta (responde
    #    "Comando desconocido") y el siguiente ya contesta la ayuda.
    fin = time.time() + 8.0
    try:
        while time.time() < fin:
            enviar(ser, "?")
            limite = time.time() + 1.0
            while time.time() < limite:
                linea = leer_linea(ser)
                if linea is None:
                    continue
                if "== Prueba en lazo abierto" in linea:
                    print("Aviso: la placa se reinició al abrir el puerto. Esperando el arranque...")
                elif "ERROR" in linea and not linea.startswith(">> Comando desconocido"):
                    print("  " + linea)
                if "esta ayuda" in linea:
                    time.sleep(0.3)  # dejar llegar el resto de la ayuda y descartarlo
                    ser.reset_input_buffer()
                    return ser
    except BaseException:
        frenar(ser)  # por si el motor ya estaba girando en modo manual
        ser.close()
        raise
    ser.close()
    raise RuntimeError("el firmware no responde: ¿está flasheado 06-motor-36gp555? Pulsar RST y repetir")


def leer_linea(ser):
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").rstrip("\r\n")


def enviar(ser, comando):
    ser.write((comando + "\n").encode())
    ser.flush()


def frenar(ser):
    """Manda 'x' (freno inmediato) y espera hasta 1 s la confirmación del firmware."""
    try:
        enviar(ser, "x")
        fin = time.time() + 1.0
        while time.time() < fin:
            linea = leer_linea(ser)
            if linea and linea.startswith(">> Parado ("):
                return True
    except Exception:  # puerto caído: no hay nada más que hacer desde acá
        pass
    return False


def esperar_quieto(ser):
    """Parar con rampa y dar tiempo a que el motor se detenga (el escalón parte del reposo)."""
    enviar(ser, "s")
    fin = time.time() + 3.5
    while time.time() < fin:
        leer_linea(ser)


def ejecutar(ser, comando, es_fila, fin_ok, timeout_s, mostrar_filas=True):
    """Manda el comando y junta las filas hasta el mensaje final. Devuelve (filas, extra)."""
    enviar(ser, comando)
    filas, extra = [], []
    limite = time.time() + timeout_s
    terminado = False
    try:
        while time.time() < limite:
            linea = leer_linea(ser)
            if linea is None:
                if terminado:
                    break  # sin más líneas después del final
                continue
            m = None if terminado else es_fila(linea)
            if not m or mostrar_filas:
                print(linea)
            if terminado:
                extra.append(linea)
                continue
            if any(linea.startswith(e) for e in ERRORES):
                raise RuntimeError(linea)
            if linea.startswith(">> Parado ("):
                raise RuntimeError(f"La prueba se cortó: {linea}")
            if m:
                filas.append(m)
            elif fin_ok in linea:
                terminado = True
                limite = time.time() + 1.0  # leer las líneas de resumen que siguen
        else:
            if not terminado:
                raise RuntimeError(f"Tiempo agotado ({timeout_s} s) esperando '{fin_ok}'")
    except KeyboardInterrupt:
        frenar(ser)
        print("\nInterrumpido: se mandó 'x' (freno).")
        sys.exit(1)
    return filas, extra


def guardar_csv(nombre, encabezado, filas):
    DIR_RESULTADOS.mkdir(exist_ok=True)
    ruta = DIR_RESULTADOS / f"{nombre}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(ruta, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(encabezado)
        w.writerows(filas)
    print(f"\nDatos guardados en {ruta}")
    return ruta


def leer_csv(ruta):
    with open(ruta) as f:
        r = csv.reader(f)
        next(r)
        return [[float(x) for x in fila] for fila in r if fila]


# ---------------------------------------------------------------- barrido

def medir_barrido(ser):
    def fila(linea):
        m = RE_FILA_BARRIDO.match(linea)
        return [float(g) for g in m.groups()] if m else None

    # ~10 pasos por sentido de ~3,2 s más pausas: alrededor de 75 s
    print("Barrido en curso (~75 s). Ctrl+C para frenar.\n")
    filas, _ = ejecutar(ser, "a", fila, ">> Fin del barrido.", timeout_s=200)
    if not filas:
        raise RuntimeError("No llegó ninguna fila del barrido")
    return guardar_csv("barrido", ["duty_pct", "rpm_motor", "rpm_salida", "cuentas_s"], filas)


def graficar_barrido(ruta, mostrar):
    import matplotlib.pyplot as plt

    d = np.array(leer_csv(ruta))
    duty, rpm_sal = d[:, 0], d[:, 2]

    fig, ax = plt.subplots(figsize=(8, 5))
    print("\nResumen del barrido:")
    for signo, nombre, color in ((1, "adelante", "tab:blue"), (-1, "atrás", "tab:orange")):
        sel = np.sign(duty) == signo
        if not sel.any():
            continue
        x, y = np.abs(duty[sel]), np.abs(rpm_sal[sel])
        ax.plot(x, y, "o-", color=color, label=f"{nombre} (|rpm|)")
        # Recta sobre los puntos donde el motor ya gira: pendiente y zona muerta estimada
        gira = y > 0.05 * y.max() if y.max() > 0 else np.zeros_like(y, bool)
        if gira.sum() >= 2:
            k, b = np.polyfit(x[gira], y[gira], 1)
            zona = -b / k if k else float("nan")
            xs = np.linspace(max(zona, 0), x.max(), 50)
            ax.plot(xs, k * xs + b, "--", color=color, alpha=0.6,
                    label=f"ajuste {nombre}: {k:.2f} rpm/%, arranca en ~{zona:.0f} %")
            print(f"  {nombre:8s}: máx {y.max():6.1f} rpm salida | pendiente {k:.3f} rpm/% | "
                  f"zona muerta estimada {zona:.1f} %")
        else:
            print(f"  {nombre:8s}: el motor casi no giró")

    ax.set_xlabel("|duty| (%)")
    ax.set_ylabel("|velocidad de salida| (rpm)")
    ax.set_title(f"Barrido en lazo abierto — {ruta.stem}")
    ax.set_xlim(0, 105)
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend()
    terminar(fig, ruta, mostrar)


# ---------------------------------------------------------------- escalón

def medir_escalon(ser, pct):
    def fila(linea):
        m = RE_FILA_ESCALON.match(linea)
        return [float(g) for g in m.groups()] if m else None

    print("Parando el motor antes del escalón...")
    esperar_quieto(ser)
    print(f"Escalón a {pct:g} % (3 s). Ctrl+C para frenar.\n")
    filas, _ = ejecutar(ser, f"e{pct:g}", fila, ">> Fin del escalón", timeout_s=15)
    if len(filas) < 10:
        raise RuntimeError("Llegaron muy pocas muestras del escalón")
    return guardar_csv(f"escalon{pct:g}", ["t_ms", "duty_pct", "cuentas", "rpm_motor", "rpm_salida"], filas)


def graficar_escalon(ruta, mostrar):
    import matplotlib.pyplot as plt

    d = np.array(leer_csv(ruta))
    t, duty, rpm = d[:, 0], d[:, 1], d[:, 4]
    signo = 1.0 if duty[-1] >= 0 else -1.0
    y = signo * rpm  # trabajar con la velocidad positiva
    # Suavizado de 5 muestras (50 ms): cada cuenta del encoder en 10 ms son ~1,9 rpm de salida
    # (se repiten las muestras de los bordes para que el promedio no caiga a 0 en los extremos)
    ys = np.convolve(np.pad(y, 2, mode="edge"), np.ones(5) / 5, mode="valid")

    n_final = max(len(t) // 5, 1)
    v_final = ys[-n_final:].mean()  # promedio del último 20 %

    def primer_cruce(frac):
        idx = np.nonzero(ys >= frac * v_final)[0]
        return t[idx[0]] if len(idx) else float("nan")

    t10, t63, t90 = primer_cruce(0.10), primer_cruce(0.632), primer_cruce(0.90)
    print("\nResumen del escalón:")
    print(f"  duty {duty[-1]:+.0f} % | velocidad final {signo * v_final:+.1f} rpm de salida")
    print(f"  tiempo al 63 %: {t63:.0f} ms (≈ constante de tiempo τ, incluye el retardo inicial) | "
          f"subida 10-90 %: {t90 - t10:.0f} ms")
    print(f"  ganancia estática: {v_final / abs(duty[-1]):.3f} rpm/%")

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(t, signo * y, ".", color="tab:gray", markersize=3, label="medido (10 ms)")
    ax.plot(t, signo * ys, "-", color="tab:blue", label="suavizado (50 ms)")
    ax.axhline(signo * v_final, color="tab:green", ls="--", lw=1, label=f"final {signo * v_final:+.1f} rpm")
    if not np.isnan(t63):
        ax.axvline(t63, color="tab:red", ls=":", lw=1)
        ax.plot([t63], [signo * 0.632 * v_final], "o", color="tab:red", label=f"63 % en {t63:.0f} ms (τ)")
    ax.set_xlabel("tiempo (ms)")
    ax.set_ylabel("velocidad de salida (rpm)")
    ax.set_title(f"Escalón a {duty[-1]:+.0f} % en lazo abierto — {ruta.stem}")
    ax.grid(True, alpha=0.3)
    ax.legend()
    terminar(fig, ruta, mostrar)


# ---------------------------------------------------------------- perfil en escalera

def medir_perfil(ser, segundos, niveles):
    meta = {"cpr": 64.0, "red": 50.0}

    def fila(linea):
        mm = RE_META_PERFIL.match(linea)
        if mm:
            meta["cpr"], meta["red"] = float(mm.group(1)), float(mm.group(2))
            return None
        m = RE_FILA_PERFIL.match(linea)
        return [float(g) for g in m.groups()] if m else None

    comando = "p"
    if segundos is not None or niveles:
        comando += f" {int((segundos or 5) * 1000)}"
        if niveles:
            comando += " " + " ".join(f"{abs(n):g}" for n in niveles)
    n_niv = len(niveles) if niveles else 5
    duracion = 2 * n_niv * (segundos or 5) + 3
    print("Parando el motor antes del perfil...")
    esperar_quieto(ser)
    print(f"Perfil en curso (~{duracion:.0f} s). Ctrl+C para frenar.\n")
    filas, _ = ejecutar(ser, comando, fila, ">> Fin del perfil.", timeout_s=duracion + 30, mostrar_filas=False)
    if len(filas) < 50:
        raise RuntimeError("Llegaron muy pocas muestras del perfil")
    DIR_RESULTADOS.mkdir(exist_ok=True)
    ruta = DIR_RESULTADOS / f"perfil_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(ruta, "w", newline="") as f:
        f.write(f"# cuentas_por_vuelta_motor={meta['cpr']:g} reduccion={meta['red']:g}\n")
        w = csv.writer(f)
        w.writerow(["t_ms", "duty_pct", "cuentas"])
        w.writerows([[int(r[0]), f"{r[1]:.1f}", int(r[2])] for r in filas])
    print(f"\nDatos guardados en {ruta} ({len(filas)} muestras)")
    return ruta


def leer_perfil(ruta):
    cpr, red = 64.0, 50.0
    filas = []
    with open(ruta) as f:
        for linea in f:
            linea = linea.strip()
            mm = RE_META_PERFIL.match(linea)
            if mm:
                cpr, red = float(mm.group(1)), float(mm.group(2))
            elif RE_FILA_PERFIL.match(linea):
                filas.append([float(x) for x in linea.split(",")])
    return np.array(filas), cpr, red


def segmentos(duty):
    """Tramos de duty constante: lista de (duty, i0, i1) con i1 exclusivo."""
    cortes = np.nonzero(np.diff(duty))[0] + 1
    bordes = np.concatenate(([0], cortes, [len(duty)]))
    return [(duty[a], a, b) for a, b in zip(bordes[:-1], bordes[1:]) if b - a > 5]


def ajustar_primer_orden(ts, v):
    """Ajusta v(t) = v0 + (vf - v0)(1 - exp(-t/tau)) a la velocidad cruda de un tramo."""
    from scipy.optimize import curve_fit

    def modelo(t, v0, vf, tau):
        return v0 + (vf - v0) * (1 - np.exp(-t / tau))

    n = len(v)
    v0_ini = v[: max(n // 50, 2)].mean()
    vf_ini = v[int(n * 0.6):].mean()
    try:
        (v0, vf, tau), _ = curve_fit(modelo, ts, v, p0=(v0_ini, vf_ini, 0.08),
                                     bounds=([-np.inf, -np.inf, 0.005], [np.inf, np.inf, 5.0]), maxfev=5000)
        return v0, vf, tau
    except (RuntimeError, ValueError):
        return v0_ini, vf_ini, float("nan")


def graficar_perfil(ruta, mostrar, alpha):
    import matplotlib.pyplot as plt

    d, cpr, red = leer_perfil(ruta)
    t, duty, pos = d[:, 0] / 1000.0, d[:, 1], d[:, 2]
    a_rpm = 60.0 / (cpr * red)  # pulsos/s -> rpm de salida

    # Velocidad cruda (pulsos/s) y filtrada con un promedio exponencial, como en MATLAB
    dt = np.diff(t)
    dt[dt <= 0] = np.nan
    v = np.concatenate(([0.0], np.diff(pos) / dt))
    v = np.nan_to_num(v)
    vf = np.empty_like(v)
    acc = 0.0
    for k, x in enumerate(v):
        acc = alpha * x + (1 - alpha) * acc
        vf[k] = acc

    # Parámetros por nivel
    filas_param = []
    tramos = [tr for tr in segmentos(duty) if tr[0] != 0]
    for dty, i0, i1 in tramos:
        ts = t[i0:i1] - t[i0]
        vs = v[i0:i1]
        v_reg = vs[int(len(vs) * 0.6):].mean()  # régimen: último 40 % del tramo
        v0, vfit, tau = ajustar_primer_orden(ts, vs)
        filas_param.append([dty, v_reg, v_reg * a_rpm, tau * 1000, v0])

    print(f"\nParámetros por nivel (cuentas por vuelta del motor {cpr:g}, reducción {red:g}:1):")
    print("   duty %  | vel. régimen (pulsos/s) | rpm salida | tau (ms)")
    for dty, v_reg, rpm, tau_ms, _ in filas_param:
        print(f"   {dty:+6.1f}  | {v_reg:+23.0f} | {rpm:+10.1f} | {tau_ms:8.0f}")

    # Ajuste lineal velocidad de régimen vs duty, por sentido: ganancia y zona muerta
    P = np.array(filas_param)
    modelo_txt = []
    ajustes = {}
    for signo, nombre in ((1, "adelante"), (-1, "atrás")):
        sel = np.sign(P[:, 0]) == signo
        if sel.sum() == 0:
            continue
        x, y = np.abs(P[sel, 0]), np.abs(P[sel, 2])
        gira = y > 0.05 * max(y.max(), 1e-9)
        taus = P[sel, 3][gira]
        tau_med = np.nanmedian(taus) if len(taus) else float("nan")
        if gira.sum() >= 2:
            k, b = np.polyfit(x[gira], y[gira], 1)
            zona = -b / k if k else float("nan")
            ajustes[signo] = (k, b)
            modelo_txt.append(f"  {nombre:8s}: K = {k:.3f} rpm/% ({k / a_rpm:.1f} pulsos/s por %), "
                              f"zona muerta {zona:.1f} %, tau mediana {tau_med:.0f} ms, máx {y.max():.1f} rpm")
        else:
            modelo_txt.append(f"  {nombre:8s}: el motor casi no giró")
    print("\nModelo de primer orden (velocidad de salida por encima de la zona muerta):")
    print("\n".join(modelo_txt))
    tau_global = np.nanmedian(P[:, 3])
    if 1 in ajustes:
        k = ajustes[1][0]
        print(f"\n  G(s) = K / (tau*s + 1) = {k:.3f} / ({tau_global / 1000:.3f} s + 1)  [rpm de salida por % de duty]")

    param_csv = ruta.with_name(ruta.stem + "_parametros.csv")
    with open(param_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["duty_pct", "vel_regimen_pulsos_s", "rpm_salida", "tau_ms", "v_inicial_pulsos_s"])
        w.writerows([[f"{x:.4g}" for x in fila] for fila in filas_param])
    print(f"Parámetros guardados en {param_csv}")

    # Figura 1: los cuatro gráficos
    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
    ax[0, 0].plot(t, pos, color="tab:blue")
    ax[0, 0].set(title="Respuesta completa - Posición", xlabel="Tiempo (s)", ylabel="Posición (pulsos)")

    ax[0, 1].step(t, duty, where="post", color="tab:red")
    ax[0, 1].set(title="Señal de control", xlabel="Tiempo (s)", ylabel="PWM (duty %)")

    colores = plt.cm.tab10(np.linspace(0, 1, 10))
    niveles = sorted({abs(tr[0]) for tr in tramos})
    for dty, i0, i1 in sorted(tramos, key=lambda tr: tr[0]):
        c = colores[niveles.index(abs(dty)) % 10]
        ax[1, 0].plot(t[i0:i1] - t[i0], pos[i0:i1] - pos[i0], "--" if dty < 0 else "-", color=c,
                      label=f"PWM={dty:+.0f} %")
    ax[1, 0].set(title="Respuestas por nivel de PWM", xlabel="Tiempo (s)", ylabel="Posición (pulsos)")
    ax[1, 0].legend(fontsize=8, ncol=2)

    ax[1, 1].plot(t, vf, color="tab:green")
    ax[1, 1].set(title=f"Velocidad angular filtrada (alpha = {alpha:g})", xlabel="Tiempo (s)",
                 ylabel="Velocidad (pulsos/s)")
    sec = ax[1, 1].secondary_yaxis("right", functions=(lambda x: x * a_rpm, lambda x: x / a_rpm))
    sec.set_ylabel("rpm de salida")
    for a in ax.flat:
        a.grid(True, alpha=0.3)
    fig.suptitle(f"Perfil en lazo abierto — {ruta.stem}")

    # Figura 2: parámetros
    fig2, bx = plt.subplots(1, 2, figsize=(13, 4.8))
    bx[0].plot(P[:, 0], P[:, 2], "o", color="tab:blue", label="medido (régimen)")
    for signo, (k, b) in ajustes.items():
        xs = np.linspace(max(-b / k, 0), 100, 50)
        bx[0].plot(signo * xs, signo * (k * xs + b), "--", alpha=0.7,
                   label=f"{'adelante' if signo > 0 else 'atrás'}: {k:.2f} rpm/%, zona muerta {-b / k:.0f} %")
    bx[0].axhline(0, color="k", lw=0.5)
    bx[0].axvline(0, color="k", lw=0.5)
    bx[0].set(title="Velocidad de régimen vs PWM", xlabel="PWM (duty %)", ylabel="rpm de salida")
    bx[0].legend(fontsize=8)
    for signo in (1, -1):  # una línea por sentido, ordenada por duty
        sel = np.sign(P[:, 0]) == signo
        orden = np.argsort(P[sel, 0])
        bx[1].plot(P[sel, 0][orden], P[sel, 3][orden], "o-", color="tab:purple")
    bx[1].axhline(tau_global, color="tab:gray", ls="--", label=f"mediana {tau_global:.0f} ms")
    bx[1].set(title="Constante de tiempo por nivel", xlabel="PWM (duty %)", ylabel="tau (ms)")
    bx[1].legend()
    for b_ in bx:
        b_.grid(True, alpha=0.3)
    fig2.suptitle(f"Parámetros del motor — {ruta.stem}")

    png2 = ruta.with_name(ruta.stem + "_parametros.png")
    fig2.tight_layout()
    fig2.savefig(png2, dpi=120)
    print(f"Gráfico de parámetros guardado en {png2}")
    terminar(fig, ruta, mostrar)


# ---------------------------------------------------------------- común

def terminar(fig, ruta, mostrar):
    import matplotlib.pyplot as plt

    png = ruta.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(png, dpi=120)
    print(f"Gráfico guardado en {png}")
    if mostrar:
        plt.show()


def main():
    p = argparse.ArgumentParser(description="Barrido y escalón del motor en lazo abierto")
    p.add_argument("prueba", choices=["barrido", "escalon", "perfil"])
    p.add_argument("pct", nargs="?", type=float, default=50.0, help="duty del escalón en %% (por defecto 50)")
    p.add_argument("--puerto", default="/dev/ttyACM0")
    p.add_argument("--archivo", type=Path, help="volver a graficar un CSV guardado, sin medir")
    p.add_argument("--sin-ventana", action="store_true", help="no abrir la ventana, solo guardar el PNG")
    p.add_argument("--segundos", type=float, help="perfil: segundos por nivel (por defecto 5)")
    p.add_argument("--niveles", type=float, nargs="+", help="perfil: niveles de duty %% (por defecto 20 40 60 80 100)")
    p.add_argument("--alpha", type=float, default=0.1, help="perfil: filtro exponencial de la velocidad (0..1]")
    a = p.parse_args()

    if a.prueba == "escalon" and not a.archivo and not (0 < abs(a.pct) <= 100):
        p.error("el duty del escalón debe estar entre -100 y 100, distinto de 0")

    ruta = a.archivo
    if ruta is None:
        import serial

        try:
            ser = abrir_puerto(a.puerto)
        except serial.SerialException as e:  # puerto ocupado o inexistente
            sys.exit(f"No se pudo abrir {a.puerto}: {e}\n¿Está abierto el monitor de PlatformIO? Cerrarlo antes.")
        except RuntimeError as e:
            sys.exit(f"ERROR en {a.puerto}: {e}")
        except KeyboardInterrupt:
            sys.exit("\nInterrumpido durante la conexión: se mandó 'x' (freno).")
        try:
            if a.prueba == "barrido":
                ruta = medir_barrido(ser)
            elif a.prueba == "escalon":
                ruta = medir_escalon(ser, a.pct)
            else:
                ruta = medir_perfil(ser, a.segundos, a.niveles)
        except KeyboardInterrupt:  # por ejemplo, durante la espera antes de la prueba
            frenar(ser)
            sys.exit("\nInterrumpido: se mandó 'x' (freno).")
        except RuntimeError as e:
            frenar(ser)
            sys.exit(f"\nERROR: {e}")
        finally:
            ser.close()

    if a.prueba == "barrido":
        graficar_barrido(ruta, not a.sin_ventana)
    elif a.prueba == "escalon":
        graficar_escalon(ruta, not a.sin_ventana)
    else:
        graficar_perfil(ruta, not a.sin_ventana, a.alpha)


if __name__ == "__main__":
    main()
