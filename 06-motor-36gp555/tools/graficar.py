#!/usr/bin/env python3
"""Barrido y escalón del motor 36GP-555 en lazo abierto: los ejecuta por el puerto serial,
guarda el CSV en resultados/ y los grafica.

Uso (cerrar antes el monitor de PlatformIO, que ocupa el puerto):
    python3 tools/graficar.py barrido                 # comando 'a' del firmware
    python3 tools/graficar.py escalon 50              # comando 'e50'
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

ERRORES = (">> ERROR", ">> Comando desconocido", ">> El escalón", ">> Escalón inválido")


# ---------------------------------------------------------------- serial

def abrir_puerto(puerto):
    import serial  # solo hace falta para medir, no para volver a graficar

    # Mismos DTR/RTS que el monitor de PlatformIO: así abrir el puerto no reinicia la S3
    ser = serial.Serial(puerto, BAUD, timeout=0.1)
    time.sleep(0.3)
    # Si igual se reinició, esperar a que termine de arrancar
    inicio = time.time()
    reinicio = False
    while time.time() - inicio < 2.5:
        linea = leer_linea(ser)
        if linea is None:
            if not reinicio:
                break
            continue
        if "== Prueba en lazo abierto" in linea:
            reinicio = True
            print("Aviso: la placa se reinició al abrir el puerto. Esperando el arranque...")
        if "ERROR" in linea:
            print("  " + linea)
    ser.reset_input_buffer()
    return ser


def leer_linea(ser):
    raw = ser.readline()
    if not raw:
        return None
    return raw.decode("utf-8", errors="replace").rstrip("\r\n")


def enviar(ser, comando):
    ser.write((comando + "\n").encode())
    ser.flush()


def esperar_quieto(ser):
    """Parar con rampa y dar tiempo a que el motor se detenga (el escalón parte del reposo)."""
    enviar(ser, "s")
    fin = time.time() + 3.5
    while time.time() < fin:
        leer_linea(ser)


def ejecutar(ser, comando, es_fila, fin_ok, timeout_s):
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
            print(linea)
            if terminado:
                extra.append(linea)
                continue
            if any(linea.startswith(e) for e in ERRORES):
                raise RuntimeError(linea)
            if linea.startswith(">> Parado ("):
                raise RuntimeError(f"La prueba se cortó: {linea}")
            m = es_fila(linea)
            if m:
                filas.append(m)
            elif fin_ok in linea:
                terminado = True
                limite = time.time() + 1.0  # leer las líneas de resumen que siguen
        else:
            if not terminado:
                raise RuntimeError(f"Tiempo agotado ({timeout_s} s) esperando '{fin_ok}'")
    except KeyboardInterrupt:
        enviar(ser, "x")
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
    p.add_argument("prueba", choices=["barrido", "escalon"])
    p.add_argument("pct", nargs="?", type=float, default=50.0, help="duty del escalón en %% (por defecto 50)")
    p.add_argument("--puerto", default="/dev/ttyACM0")
    p.add_argument("--archivo", type=Path, help="volver a graficar un CSV guardado, sin medir")
    p.add_argument("--sin-ventana", action="store_true", help="no abrir la ventana, solo guardar el PNG")
    a = p.parse_args()

    if a.prueba == "escalon" and not a.archivo and not (0 < abs(a.pct) <= 100):
        p.error("el duty del escalón debe estar entre -100 y 100, distinto de 0")

    ruta = a.archivo
    if ruta is None:
        try:
            ser = abrir_puerto(a.puerto)
        except Exception as e:  # puerto ocupado o inexistente
            sys.exit(f"No se pudo abrir {a.puerto}: {e}\n¿Está abierto el monitor de PlatformIO? Cerrarlo antes.")
        try:
            ruta = medir_barrido(ser) if a.prueba == "barrido" else medir_escalon(ser, a.pct)
        except RuntimeError as e:
            enviar(ser, "x")
            sys.exit(f"\nERROR: {e}")
        finally:
            ser.close()

    if a.prueba == "barrido":
        graficar_barrido(ruta, not a.sin_ventana)
    else:
        graficar_escalon(ruta, not a.sin_ventana)


if __name__ == "__main__":
    main()
