#!/usr/bin/env python3
"""Prueba de la odometría de la base (en la Pi, con el agente de la base corriendo). Usa movimiento.py.

Manda /cmd_vel a 20 Hz y lee /odom y el giróscopo. Para cuando llega (recto: ruedas; giro: giróscopo) o al vencer el
tiempo, frena y muestra lo pedido contra lo medido. En el piso, comparar con la cinta: eso es el error.
13/11 en el piso: recto 1 m -> ~102 cm de cinta (< 1 %); giro 90 -> ruedas ~91°, giróscopo 88,7°, cinta ~88–89°.

Uso:
    python3 prueba_odom.py recto <m> [v=0.10]        avanzar <m> metros (negativo: retroceder)
    python3 prueba_odom.py giro <grados> [w=0.5]     girar en el lugar (+ = izquierda, antihorario visto desde arriba)
    python3 prueba_odom.py tiempo <v> <w> <s>        velocidad fija durante <s> segundos (ruedas en el aire)
Arma la base si está desarmada. Ctrl+C frena. Tope de seguridad: 2 m, 400° y 20 s por prueba.
"""
import sys

import rclpy

from movimiento import Base, informe


def main():
    a = sys.argv[1:]
    if not a or a[0] not in ("recto", "giro", "tiempo"):
        print(__doc__)
        return
    modo = a[0]
    if modo == "recto":
        obj, v, w = float(a[1]), (float(a[2]) if len(a) > 2 else 0.10), 0.0
    elif modo == "giro":
        obj, v, w = float(a[1]), 0.0, (float(a[2]) if len(a) > 2 else 0.5)
    else:
        v, w, obj = float(a[1]), float(a[2]), float(a[3])
    rclpy.init()
    n = rclpy.create_node("prueba_odom")
    b = Base(n)
    try:
        b.esperar_datos()
        b.armar()
        print(f"{modo}: v {v:+.2f} m/s, w {w:+.2f} rad/s, objetivo {obj:g}" + {"recto": " m", "giro": "°",
                                                                               "tiempo": " s"}[modo])
        informe(b.mover(modo, obj, v, w))
        print(f"  estado: {b.estado[:90]}")
    except KeyboardInterrupt:
        print("Ctrl+C: frenado")
    except (RuntimeError, ValueError) as e:
        print(f"NO: {e}")
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
