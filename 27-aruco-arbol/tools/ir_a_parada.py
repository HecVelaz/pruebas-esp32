#!/usr/bin/env python3
"""Ir a la parada: el robot se ubica solo donde agarra (26-sim-alcance-arbolito), guiado por la marca de la maceta.

En la Pi, con start_limon.sh (cámaras) y el agente de la base corriendo:
  1. MIRA: mediana de 3 s de la marca (calibración de navegación) -> eje del tronco = la marca corrida
     --radio-maceta hacia adentro, y hacia dónde mira la marca. Hace falta verla a ~0,9 m o más (más cerca queda
     cortada abajo).
  2. CALCULA la parada: el eje del tronco a --parada D Y (def. 700 a la izquierda de J1 y 150 adelante) y la marca de
     frente (el robot paralelo a esa cara de la maceta; --sin-orientacion la ignora). Como la base no va de costado:
     girar hacia el destino (giróscopo), avanzar (ruedas) y girar a la orientación final (giróscopo). El robot gira
     sobre el centro del eje de las ruedas, ~8 cm a la derecha de J1 (tapa.jpeg: J1 a 9 cm del borde izquierdo de la
     parte angosta de 34 cm; el eje de las ruedas casi bajo J1).
  3. MUESTRA el plan, espera Enter y lo hace sin mirar (la cámara solo ve a la izquierda); al final vuelve a mirar si
     la marca se ve. Después: "veragarrar" en punta.py.

Uso: python3 tools/ir_a_parada.py [--radio-maceta 165] [--parada 700 150] [--sin-orientacion] [--simular X Y MIRA]
     --simular: sin ROS, el plan para un tronco en (X, Y) mm con la marca mirando a MIRA° (para probar la cuenta).
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent.parent / "25-teleop-pi" / "tools"))   # movimiento.py (reutilizar, no copiar)

CENTRO_GIRO = np.array([80.0, -5.0])   # mm, centro del eje de las ruedas en el marco de J1
V_RECTO, W_GIRO = 0.10, 0.5
MIN_GIRO, MIN_RECTO = 1.5, 50.0         # °, mm: por debajo no se mueve (la parada tolera ±5 cm)
MAX_RECTO = 1500.0                      # mm


def rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def planear(tronco, mira_deg, parada, orientar=True):
    """tronco (x, y) mm en el marco actual; mira_deg: hacia dónde apunta la cara de la marca (0 = +x, hacia el robot).
    Devuelve la lista de pasos [("giro", °) | ("recto", mm)] y la pose final del centro de giro."""
    psi = math.radians(mira_deg) if orientar else 0.0
    j1_f = np.asarray(tronco, float) - rot(psi) @ np.asarray(parada, float)
    c_f = j1_f + rot(psi) @ CENTRO_GIRO
    D = c_f - CENTRO_GIRO
    dist = float(np.linalg.norm(D))
    pasos = []
    if dist < MIN_RECTO:
        if abs(math.degrees(psi)) >= MIN_GIRO:
            pasos.append(("giro", math.degrees(psi)))
        return pasos
    alfa = math.atan2(-D[0], D[1])   # adelante del robot = +y; girar alfa (+ izquierda) lo apunta a D
    if abs(alfa) > math.radians(135):   # marcha atrás solo si no queda otra: atrás no hay sensores
        alfa, dist = wrap(alfa + math.pi), -dist
    if abs(math.degrees(alfa)) >= MIN_GIRO:
        pasos.append(("giro", math.degrees(alfa)))
    pasos.append(("recto", dist))
    fin = math.degrees(wrap(psi - alfa))
    if abs(fin) >= MIN_GIRO:
        pasos.append(("giro", fin))
    return pasos


def simular_pasos(tronco, pasos):
    """Aplica los pasos (ideal) y devuelve dónde queda el tronco en el marco final (para verificar la cuenta)."""
    h, c = 0.0, CENTRO_GIRO.copy()
    for tipo, val in pasos:
        if tipo == "giro":
            h += math.radians(val)
        else:
            c = c + val * np.array([-math.sin(h), math.cos(h)])
    j1 = c - rot(h) @ CENTRO_GIRO
    return rot(-h) @ (np.asarray(tronco, float) - j1), math.degrees(h)


def texto(pasos):
    t = []
    for tipo, val in pasos:
        if tipo == "giro":
            t.append(f"girar {abs(val):.0f}° a la {'izquierda' if val > 0 else 'derecha'}")
        else:
            t.append(f"{'avanzar' if val > 0 else 'retroceder'} {abs(val) / 10:.0f} cm")
    return " -> ".join(t) if t else "ya está en la parada"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radio-maceta", type=float, default=165.0, help="mm del eje del tronco a la cara de la marca")
    ap.add_argument("--parada", type=float, nargs=2, default=[700.0, 150.0], metavar=("D", "Y"))
    ap.add_argument("--sin-orientacion", action="store_true", help="no girar para quedar de frente a la marca")
    ap.add_argument("--simular", type=float, nargs=3, metavar=("X", "Y", "MIRA"))
    a = ap.parse_args()
    parada = (-a.parada[0], a.parada[1])

    if a.simular:
        tronco, mira = a.simular[:2], a.simular[2]
        pasos = planear(tronco, mira, parada, not a.sin_orientacion)
        final, h = simular_pasos(tronco, pasos)
        print(f"tronco en ({tronco[0]:.0f}, {tronco[1]:.0f}), marca mirando a {mira:+.0f}°: {texto(pasos)}")
        print(f"  verificación: el tronco queda en ({final[0]:.0f}, {final[1]:.0f}) (parada {parada}), giro total {h:+.1f}°")
        return

    import rclpy
    from marcas import Marcas
    from movimiento import Base, informe

    rclpy.init()
    n = rclpy.create_node("ir_a_parada")
    base, marcas = Base(n), Marcas(n)
    try:
        base.esperar_datos()
        print("mirando la marca 3 s (robot quieto)...")
        m = marcas.medir(base.girar_ros, 3.0)
        if m is None:
            print(f"NO: no veo la marca ({marcas.imgs} imágenes). ¿start_limon.sh corriendo? ¿maceta a ~0,9–4 m a la "
                  f"izquierda, con la marca de frente?")
            return
        i, pos, mira, nv = m
        nrm = np.array([math.cos(math.radians(mira)), math.sin(math.radians(mira))])
        tronco = pos[:2] - nrm * a.radio_maceta
        print(f"  marca ID {i} ({nv} vistas): cara en ({pos[0]:.0f}, {pos[1]:.0f}) mm, mira a {mira:+.0f}° -> tronco a "
              f"{-tronco[0] / 10:.0f} cm a la izquierda de J1, {tronco[1] / 10:+.0f} cm adelante")
        pasos = planear(tronco, mira, parada, not a.sin_orientacion)
        final, _ = simular_pasos(tronco, pasos)
        if any(t == "recto" and abs(v) > MAX_RECTO for t, v in pasos):
            print(f"NO: el plan avanza más de {MAX_RECTO / 1000:.1f} m: acercar el robot primero ({texto(pasos)})")
            return
        print(f"plan: {texto(pasos)}")
        print(f"  (queda con el tronco en ({final[0]:.0f}, {final[1]:.0f}) mm; parada ({parada[0]:.0f}, {parada[1]:.0f}))")
        if not pasos:
            return
        if input("Enter para ir (cualquier otra cosa cancela): ").strip():
            print("cancelado")
            return
        base.armar()
        for tipo, val in pasos:
            print(f"== {texto([(tipo, val)])}")
            r = base.mover("giro" if tipo == "giro" else "recto", val if tipo == "giro" else val / 1000.0,
                           V_RECTO, W_GIRO)
            informe(r)
            if not r["ok"]:
                print("[cortado: el robot quedó donde está]")
                return
        print("llegó a la parada (según la odometría).")
        m = marcas.medir(base.girar_ros, 2.0)
        if m:
            i, pos, mira, nv = m
            tr = pos[:2] - np.array([math.cos(math.radians(mira)), math.sin(math.radians(mira))]) * a.radio_maceta
            print(f"  la marca todavía se ve: tronco a {-tr[0] / 10:.0f} cm, {tr[1] / 10:+.0f} cm (parada "
                  f"{-parada[0] / 10:.0f}, {parada[1] / 10:+.0f})")
        else:
            print("  (la marca ya no se ve completa, como se esperaba a esta distancia)")
        print("siguiente: en punta.py, \"veragarrar\" y \"agarrar\"")
    except KeyboardInterrupt:
        base.frenar()
        print("Ctrl+C: frenado")
    except (RuntimeError, ValueError) as e:
        print(f"NO: {e}")
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
