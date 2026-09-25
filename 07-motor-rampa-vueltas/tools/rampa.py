#!/usr/bin/env python3
"""Rampa de PWM de 0 a 100 % que termina al completar N vueltas de salida (lazo abierto).

Manda 'r <segundos> <vueltas>' al firmware de 07-motor-rampa-vueltas, guarda el CSV en resultados/
y grafica: señal de control y vueltas, velocidad, velocidad vs PWM (barrido continuo) y PWM por vuelta.

Uso (cerrar antes el monitor de PlatformIO):
    python3 tools/rampa.py                          # 0 -> 100 % en 3 s, hasta 3 vueltas
    python3 tools/rampa.py --segundos 5 --vueltas 5 # la prueba anterior (5 vueltas)
    python3 tools/rampa.py --vueltas -3             # hacia atrás
    python3 tools/rampa.py --comparar resultados/rampa_SINCARGA.csv   # superponer otra prueba
    python3 tools/rampa.py --archivo resultados/rampa_....csv          # volver a graficar sin medir
Ctrl+C durante la prueba frena el motor antes de salir.
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
RE_FILA = re.compile(r"^(\d+),([+-]?[\d.]+),([+-]?\d+)$")
RE_META = re.compile(r"^# cuentas_por_vuelta_motor=([\d.]+) reduccion=([\d.]+) rampa_s=([\d.]+) vueltas=([+-]?[\d.]+)")
ERRORES = (">> ERROR", ">> Comando desconocido", ">> Rampa inválida", ">> La rampa parte")


# ---------------------------------------------------------------- medición

def leer_linea(ser):
    raw = ser.readline()
    return raw.decode("utf-8", errors="replace").rstrip("\r\n") if raw else None


def medir(puerto, segundos, vueltas):
    import serial

    try:
        # Mismos DTR/RTS que el monitor de PlatformIO: abrir el puerto no reinicia la S3
        ser = serial.Serial(puerto, BAUD, timeout=0.1)
    except Exception as e:
        sys.exit(f"No se pudo abrir {puerto}: {e}\n¿Está abierto el monitor de PlatformIO? Cerrarlo antes.")
    time.sleep(0.3)
    # Al abrir el puerto, el texto que el ESP32 tenía guardado (por ejemplo, la ayuda del arranque) puede
    # volver como eco antes de que pyserial configure el puerto, y quedar a medias en la línea del firmware.
    # Un Enter vacío la descarta (con el motor quieto no hace nada); después se limpia lo que responda.
    ser.write(b"\n")
    time.sleep(0.3)
    ser.reset_input_buffer()

    meta, filas, fin = None, [], None
    ser.write(f"r {segundos:g} {vueltas:g}\n".encode())
    limite = time.time() + segundos + 30
    print(f"Rampa 0 -> 100 % en {segundos:g} s, hasta {vueltas:g} vueltas. Ctrl+C para frenar.\n")
    try:
        while time.time() < limite:
            linea = leer_linea(ser)
            if linea is None:
                continue
            m = RE_FILA.match(linea)
            if m:
                filas.append([int(m.group(1)), float(m.group(2)), int(m.group(3))])
                if len(filas) % 100 == 0:  # progreso cada segundo
                    v = filas[-1][2] / (meta[0] * meta[1]) if meta else 0
                    print(f"  t {filas[-1][0] / 1000:5.1f} s | duty {filas[-1][1]:+6.1f} % | vueltas {v:+.2f}")
                continue
            print(linea)
            mm = RE_META.match(linea)
            if mm:
                meta = [float(g) for g in mm.groups()]
            if any(linea.startswith(e) for e in ERRORES):
                sys.exit(f"\nERROR: {linea}")
            if linea.startswith(">> Fin ("):
                fin = linea
                break
        else:
            ser.write(b"x\n")
            sys.exit("\nERROR: tiempo agotado esperando el fin de la rampa")
    except KeyboardInterrupt:
        ser.write(b"x\n")
        print("\nInterrumpido: se frenó el motor.")
        if len(filas) < 10:
            sys.exit(1)
    finally:
        ser.close()

    if meta is None or len(filas) < 10:
        sys.exit("ERROR: no llegaron datos de la rampa")
    if fin and "ATASCO" in fin:
        print("Aviso: la prueba terminó por ATASCO; se guardan y grafican los datos hasta ahí.")

    DIR_RESULTADOS.mkdir(exist_ok=True)
    ruta = DIR_RESULTADOS / f"rampa_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with open(ruta, "w", newline="") as f:
        f.write(f"# cuentas_por_vuelta_motor={meta[0]:g} reduccion={meta[1]:g} rampa_s={meta[2]:g} vueltas={meta[3]:g}\n")
        if fin:
            f.write(f"# {fin}\n")
        w = csv.writer(f)
        w.writerow(["t_ms", "duty_pct", "cuentas"])
        w.writerows(filas)
    print(f"\nDatos guardados en {ruta} ({len(filas)} muestras)")
    return ruta


# ---------------------------------------------------------------- análisis

def cargar(ruta):
    meta = [64.0, 50.0, float("nan"), float("nan")]
    filas = []
    with open(ruta) as f:
        for linea in f:
            linea = linea.strip()
            mm = RE_META.match(linea)
            if mm:
                meta = [float(g) for g in mm.groups()]
            elif RE_FILA.match(linea):
                filas.append([float(x) for x in linea.split(",")])
    d = np.array(filas)
    cpr, red = meta[0], meta[1]
    t = d[:, 0] / 1000.0
    duty = d[:, 1]
    vueltas = d[:, 2] / (cpr * red)
    # Velocidad de salida (rpm): derivada de la posición con un promedio centrado de 110 ms (sin retraso)
    dt = np.diff(t)
    dt[dt <= 0] = np.nan
    rpm = np.nan_to_num(np.concatenate(([0.0], np.diff(vueltas) / dt * 60.0)))
    n = 11
    rpm_f = np.convolve(np.pad(rpm, n // 2, mode="edge"), np.ones(n) / n, mode="valid")
    return {"ruta": Path(ruta), "t": t, "duty": duty, "vueltas": vueltas, "rpm": rpm_f, "meta": meta}


def resumen(r, ref_k, ref_zm, tau):
    t, duty, vue, rpm = r["t"], np.abs(r["duty"]), np.abs(r["vueltas"]), np.abs(r["rpm"])
    print(f"\nResumen de {r['ruta'].name}:")
    print(f"  {vue[-1]:.2f} vueltas de salida en {t[-1]:.2f} s, duty final {r['duty'][-1]:+.1f} %")

    # Arranque: primer instante en que la salida supera 1 rpm y sigue girando
    gira = rpm > 1.0
    idx = next((i for i in range(len(gira) - 10) if gira[i:i + 10].all()), None)
    arranque = duty[idx] if idx is not None else float("nan")
    print(f"  arranque: el motor empieza a girar con {arranque:.1f} % de duty (t = {t[idx] if idx is not None else float('nan'):.2f} s)")

    print("  vuelta | tiempo (s) | duty (%) | rpm salida")
    for k in range(1, int(np.floor(vue[-1])) + 1):
        i = int(np.argmax(vue >= k))
        print(f"  {k:6d} | {t[i]:10.2f} | {duty[i]:8.1f} | {rpm[i]:10.1f}")

    # Recta velocidad vs duty por encima del arranque: ganancia y zona muerta de esta prueba.
    # La velocidad va ~tau atrasada respecto del PWM en la rampa: se compara con el duty de tau segundos antes.
    ajuste = None
    if idx is not None:
        duty_ret = np.interp(t - tau, t, duty)
        sel = (duty_ret > arranque + 5) & (duty < 99.5)
        if sel.sum() > 20:
            k, b = np.polyfit(duty_ret[sel], rpm[sel], 1)
            ajuste = (k, b)
            print(f"  ajuste: K = {k:.3f} rpm/%, zona muerta {-b / k:.1f} %  "
                  f"(sin carga, perfil de 06: K = {ref_k:.2f} rpm/%, zona muerta {ref_zm:.1f} %)")
            for d_ in (50, 100):
                ref = ref_k * (d_ - ref_zm)
                medido = k * d_ + b
                print(f"  a {d_:3d} %: {medido:6.1f} rpm vs {ref:6.1f} rpm sin carga ({100 * (medido - ref) / ref:+.1f} %)")
    return arranque, ajuste


def graficar(r, comp, ref_k, ref_zm, mostrar):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 2, figsize=(13, 8.5))
    series = [(r, "tab:blue", "esta prueba")] + ([(comp, "tab:gray", f"comparación ({comp['ruta'].stem})")] if comp else [])

    # (a) Señal de control y vueltas
    a0 = ax[0, 0]
    a0.plot(r["t"], r["duty"], color="tab:red", label="PWM (duty %)")
    a0.set(title="Señal de control y posición", xlabel="Tiempo (s)", ylabel="PWM (duty %)")
    a0b = a0.twinx()
    a0b.plot(r["t"], r["vueltas"], color="tab:blue", label="vueltas de salida")
    a0b.set_ylabel("Vueltas de salida")
    for k in range(1, int(np.floor(abs(r["vueltas"][-1]))) + 1):
        a0b.axhline(np.sign(r["vueltas"][-1]) * k, color="tab:blue", lw=0.5, ls=":")
    l1, n1 = a0.get_legend_handles_labels()
    l2, n2 = a0b.get_legend_handles_labels()
    a0.legend(l1 + l2, n1 + n2, loc="upper left", fontsize=8)

    # (b) Velocidad en el tiempo
    for s, c, nom in series:
        ax[0, 1].plot(s["t"], s["rpm"], color=c, label=nom)
    ax[0, 1].set(title="Velocidad de salida", xlabel="Tiempo (s)", ylabel="rpm de salida")
    ax[0, 1].legend(fontsize=8)

    # (c) Velocidad vs PWM: barrido continuo
    for s, c, nom in series:
        ax[1, 0].plot(np.abs(s["duty"]), np.abs(s["rpm"]), color=c, label=nom)
    xs = np.linspace(ref_zm, 100, 50)
    ax[1, 0].plot(xs, ref_k * (xs - ref_zm), "--", color="tab:green",
                  label=f"sin carga (06): {ref_k:.2f} rpm/%, zona muerta {ref_zm:.0f} %")
    ax[1, 0].set(title="Velocidad vs PWM (barrido continuo)", xlabel="|PWM| (duty %)", ylabel="|rpm de salida|",
                 xlim=(0, 102))
    ax[1, 0].set_ylim(bottom=0)
    ax[1, 0].legend(fontsize=8)

    # (d) PWM en función de la posición: con qué duty completa cada vuelta
    for s, c, nom in series:
        ax[1, 1].plot(np.abs(s["vueltas"]), np.abs(s["duty"]), color=c, label=nom)
        vf = np.abs(s["vueltas"])
        for k in range(1, int(np.floor(vf[-1])) + 1):
            i = int(np.argmax(vf >= k))
            ax[1, 1].plot([k], [abs(s["duty"][i])], "o", color=c)
            ax[1, 1].annotate(f"{abs(s['duty'][i]):.0f} %", (k, abs(s["duty"][i])), textcoords="offset points",
                              xytext=(-10, 8), fontsize=8, color=c)
    ax[1, 1].set(title="PWM por vuelta", xlabel="Vueltas de salida", ylabel="|PWM| (duty %)", ylim=(0, 108))
    ax[1, 1].legend(fontsize=8)

    for a in ax.flat:
        a.grid(True, alpha=0.3)
    m = r["meta"]
    fig.suptitle(f"Rampa 0 -> 100 % en {m[2]:g} s hasta {m[3]:g} vueltas — {r['ruta'].stem}")
    fig.tight_layout()
    png = r["ruta"].with_suffix(".png")
    fig.savefig(png, dpi=120)
    print(f"\nGráfico guardado en {png}")
    if mostrar:
        plt.show()


def main():
    p = argparse.ArgumentParser(description="Rampa de PWM por vueltas en lazo abierto")
    # Sin carga, 3 s dan 3 vueltas llegando al ~99 %: el giro completo 0 -> 100 % en las 3 vueltas.
    # Para otra cantidad de vueltas, mantener segundos = vueltas (medido: 5 s -> 5 vueltas con 98,8 %).
    p.add_argument("--segundos", type=float, default=3.0, help="duración de la rampa 0 -> 100 %% (por defecto 3)")
    p.add_argument("--vueltas", type=float, default=3.0, help="vueltas de salida para terminar (por defecto 3); negativas: atrás")
    p.add_argument("--puerto", default="/dev/ttyACM0")
    p.add_argument("--archivo", type=Path, help="volver a graficar un CSV guardado, sin medir")
    p.add_argument("--comparar", type=Path, help="CSV de otra rampa para superponer (por ejemplo, sin carga)")
    p.add_argument("--ref-k", type=float, default=1.58, help="ganancia sin carga (rpm/%%), del perfil de 06")
    p.add_argument("--ref-zm", type=float, default=8.3, help="zona muerta sin carga (%%), del perfil de 06")
    p.add_argument("--tau", type=float, default=0.065, help="constante de tiempo (s) para compensar el retraso")
    p.add_argument("--sin-ventana", action="store_true", help="no abrir la ventana, solo guardar el PNG")
    a = p.parse_args()

    ruta = a.archivo or medir(a.puerto, a.segundos, a.vueltas)
    r = cargar(ruta)
    resumen(r, a.ref_k, a.ref_zm, a.tau)
    comp = None
    if a.comparar:
        comp = cargar(a.comparar)
        resumen(comp, a.ref_k, a.ref_zm, a.tau)
    graficar(r, comp, a.ref_k, a.ref_zm, not a.sin_ventana)


if __name__ == "__main__":
    main()
