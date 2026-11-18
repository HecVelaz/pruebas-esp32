#!/usr/bin/env python3
"""Prueba del frenado por el HC-SR04 del frente (en la Pi, con el agente de la base; ruedas en el aire).

Manda /cmd_vel fijo (0,10 m/s por defecto) y avisa en pantalla, con pitido y cuenta regresiva, dónde poner la mano
delante del sensor del medio. Registra distancia del frente, velocidad medida y estado de la seguridad, y al final
muestra un resumen por tramo. CSV en ~/logs/frenado_<fecha>.csv.

Firmware (base_config.h): para a <= 0,20 m; entre 0,20 y 0,50 m el tope de avance baja lineal desde 0,25 m/s
-> a 0,10 m/s pedidos recién frena por debajo de ~0,32 m (a 0,25 m el tope es ~0,04 m/s).

Uso:
    python3 prueba_frenado.py [v=0.10] [s_por_tramo=5]
Ctrl+C frena.
"""
import csv
import math
import os
import statistics
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from movimiento import Base, HZ

TRAMOS = [  # (instrucción, qué se espera)
    ("SIN MANO (que arranquen las ruedas)", "giran a la velocidad pedida"),
    ("MANO A 60 cm del sensor del medio", "igual que sin mano"),
    ("MANO A 25 cm", "giran más lento"),
    ("MANO A 15 cm", "se paran (no_avanza)"),
    ("SACÁ LA MANO", "vuelven a girar"),
    ("MANO A 10 cm", "se paran (no_avanza)"),
    ("SACÁ LA MANO", "vuelven a girar"),
]


def main():
    v = float(sys.argv[1]) if len(sys.argv) > 1 else 0.10
    seg = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    if not 0 < v <= 0.25:
        sys.exit("v entre 0 y 0,25 m/s")
    rclpy.init()
    n = rclpy.create_node("prueba_frenado")
    b = Base(n)
    dist = {"r": math.nan}
    n.create_subscription(Range, "/range/frente", lambda m: dist.__setitem__("r", m.range), qos_profile_sensor_data)
    os.makedirs(os.path.expanduser("~/logs"), exist_ok=True)
    ruta = os.path.expanduser(time.strftime("~/logs/frenado_%Y%m%d_%H%M%S.csv"))
    filas = []
    try:
        b.esperar_datos()
        b.armar()
        print(f"\nv pedida {v:.2f} m/s, {len(TRAMOS)} tramos de {seg:g} s. Ctrl+C frena.")
        for k in (3, 2, 1):
            print(f"  empieza en {k}...", flush=True)
            b.girar_ros(1.0)
        t0 = time.time()
        for i, (instr, espera) in enumerate(TRAMOS):
            print(f"\a\n>>> [{i + 1}/{len(TRAMOS)}] {instr}   (se espera: {espera})", flush=True)
            fin = time.time() + seg
            ult = None
            while time.time() < fin:
                b.mandar(v, 0.0)
                b.girar_ros(1.0 / HZ)
                o = b.odom
                vm = o.twist.twist.linear.x if o else math.nan
                no_av = "no_avanza" in b.estado
                filas.append((round(time.time() - t0, 3), i + 1, dist["r"], vm, int(no_av),
                              b.estado.split(" imu=")[0]))
                falta = math.ceil(fin - time.time())
                if falta != ult and falta > 0:
                    ult = falta
                    r = dist["r"]
                    rs = "inf" if math.isinf(r) else ("—" if math.isnan(r) else f"{r * 100:5.1f} cm")
                    print(f"    {falta} s | frente {rs} | v {vm:+.3f} m/s{'  NO_AVANZA' if no_av else ''}",
                          flush=True)
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        b.frenar()
        with open(ruta, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "tramo", "dist_frente_m", "v_medida_ms", "no_avanza", "estado"])
            w.writerows(filas)
        n.destroy_node()
        rclpy.shutdown()

    print(f"\nResumen (mediana del último 60 % de cada tramo; CSV {ruta}):")
    print(f"{'tramo':<40} {'frente':>9} {'v medida':>9} {'no_avanza':>10}")
    for i, (instr, _) in enumerate(TRAMOS):
        fs = [x for x in filas if x[1] == i + 1]
        fs = fs[int(len(fs) * 0.4):]
        if not fs:
            continue
        ds = [x[2] for x in fs if not math.isnan(x[2])]
        d = statistics.median(ds) if ds else math.nan
        ds_txt = "inf" if math.isinf(d) else f"{d * 100:.1f} cm"
        vs = statistics.median(x[3] for x in fs)
        na = sum(x[4] for x in fs) / len(fs)
        print(f"{instr[:40]:<40} {ds_txt:>9} {vs:>+9.3f} {na * 100:>9.0f}%")
    # Distancia a la que apareció no_avanza por primera vez y la menor sin no_avanza
    con = [x[2] for x in filas if x[4] and not math.isnan(x[2]) and not math.isinf(x[2])]
    sin = [x[2] for x in filas if not x[4] and not math.isnan(x[2]) and not math.isinf(x[2])]
    if con:
        print(f"no_avanza con el frente entre {min(con) * 100:.1f} y {max(con) * 100:.1f} cm", end="")
        print(f"; sin no_avanza la menor distancia fue {min(sin) * 100:.1f} cm" if sin else "")


if __name__ == "__main__":
    main()
