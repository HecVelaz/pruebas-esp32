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

Uso: python3 tools/ir_a_parada.py [--banderin [JSON]] [--arbol DERECHA FRENTE] [--buscar] [--radio-maceta 165]
     [--parada 700 150] [--sin-orientacion] [--simular X Y MIRA]
     [--rodear N] [--observar 1100 150]
     --rodear N: después de la parada, N rodeos (planear_rodeo: 90° antihorario alrededor del tronco hasta donde se ve
     la marca siguiente, y de ahí a su parada), solo la base: para probar sin el brazo.
     --auto S: sin Enter (sin red cerca): espera S s y hace todo solo; correrlo con nohup y parar con la seta.
     --simular: sin ROS, el plan para un tronco en (X, Y) mm con la marca mirando a MIRA° (para probar la cuenta);
     con --banderin, (X, Y) es el banderín y el árbol sale de banderin.json.

BANDERÍN ÚNICO (--banderin, 19/11): una sola marca en un palo en cualquier lugar, y en ../banderin.json dónde está
el eje del arbolito respecto de ella (medido con cinta, en el piso), mirando la marca DE FRENTE:
  arbol_derecha_mm: hacia la derecha de quien mira la marca de frente (izquierda: negativo)
  arbol_frente_mm:  hacia quien la mira (lo que sale de la cara impresa); el árbol DETRÁS de la marca: negativo
  parada_lado_deg:  de qué lado del árbol queda el robot en la parada 1: 0 = del lado al que mira la marca, 90 = un
                    cuarto de vuelta antihorario (visto de arriba), etc. El robot queda con el árbol a su izquierda.
--buscar: si no ve el banderín, gira en el lugar de a 30° (antihorario) mirando, hasta 360°.
Con --banderin, --rodear va a la parada siguiente solo con la odometría (no hay marcas en la maceta).
"""
import argparse
import json
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
MAX_RECTO = 1500.0                      # mm, con la marca en la maceta
MAX_RECTO_BANDERIN = 3000.0             # mm: el banderín puede estar más lejos del árbol (en tramos de MAX_TRAMO)
MAX_TRAMO = 1500.0                      # mm por "recto" (movimiento.TOPE_M es 2 m)
BANDERIN = AQUI.parent / "banderin.json"
SEP_BANDERIN, SEP_TRONCO = 450.0, 450.0  # mm: lo más cerca que puede pasar J1 (borde izq.; el der. está a 31 cm)
PASO_BUSCAR, SEG_BUSCAR = 30.0, 1.5      # °, s (la cámara ve ~51° de ancho)


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


def planear_rodeo(tronco, observar):
    """RODEAR: desde la parada (tronco en (x, y) mm, la marca de esta cara de frente), los pasos hasta el punto de
    observación de la cara siguiente, 90° antihorario alrededor del tronco (el robot avanza con el árbol a la
    izquierda). observar = dónde queda el tronco respecto de J1 al llegar, más lejos que la parada para que la marca
    siguiente se vea entera (a 70 cm queda cortada abajo); desde ahí medir_y_planear corrige como en la primera.
    La cara siguiente mira 90° a la izquierda de la actual: mira = +90°."""
    return planear(tronco, 90.0, observar)


def distancia_minima(tronco, pasos, n=50):
    """mm: lo más cerca que pasa J1 del tronco siguiendo los pasos (ideal), para ver que no roce la maceta."""
    h, c = 0.0, CENTRO_GIRO.copy()
    t = np.asarray(tronco, float)
    dmin = float(np.linalg.norm(t))
    for tipo, val in pasos:
        if tipo == "giro":
            h += math.radians(val)
            continue
        for k in range(1, n + 1):
            ck = c + val * k / n * np.array([-math.sin(h), math.cos(h)])
            j1 = ck - rot(h) @ CENTRO_GIRO
            dmin = min(dmin, float(np.linalg.norm(t - j1)))
        c = c + val * np.array([-math.sin(h), math.cos(h)])
    return dmin


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


def tronco_desde(pos, mira, radio):
    nrm = np.array([math.cos(math.radians(mira)), math.sin(math.radians(mira))])
    return pos[:2] - nrm * radio


def cargar_banderin(ruta=BANDERIN, arbol=None):
    """banderin.json -> dict (id, derecha, frente, lado). arbol = (derecha, frente) mm pisa lo del archivo."""
    j = json.loads(Path(ruta).read_text())
    der, fre = (arbol if arbol is not None else (j.get("arbol_derecha_mm"), j.get("arbol_frente_mm")))
    if der is None or fre is None:
        raise ValueError(f"falta dónde está el árbol respecto del banderín: completar arbol_derecha_mm y "
                         f"arbol_frente_mm en {ruta} (o --arbol DERECHA FRENTE)")
    return {"id": int(j.get("id", 0)), "derecha": float(der), "frente": float(fre),
            "lado": float(j.get("parada_lado_deg", 0.0))}


def tronco_desde_banderin(pos, mira, b):
    """Eje del árbol en el marco del robot, desde la pose del banderín (pos mm, mira ° = hacia dónde sale la cara
    impresa) y lo medido con cinta. La derecha de quien mira la marca de frente es la normal girada +90°."""
    nrm = np.array([math.cos(math.radians(mira)), math.sin(math.radians(mira))])
    der = rot(math.pi / 2) @ nrm
    return pos[:2] + b["frente"] * nrm + b["derecha"] * der


def plan_banderin(pos, mira, b, parada, orientar=True):
    """(tronco, pasos, aviso): aviso es None o el motivo para no ir (pasa muy cerca del banderín o del tronco)."""
    tronco = tronco_desde_banderin(pos, mira, b)
    pasos = planear(tronco, mira + b["lado"], parada, orientar)
    aviso = None
    dban, dtr = distancia_minima(pos[:2], pasos), distancia_minima(tronco, pasos)
    if dban < SEP_BANDERIN:
        aviso = (f"el camino pasa J1 a {dban / 10:.0f} cm del banderín (mín. {SEP_BANDERIN / 10:.0f}): mover el "
                 f"robot o el banderín, o cambiar parada_lado_deg")
    elif dtr < SEP_TRONCO:
        aviso = f"el camino pasa J1 a {dtr / 10:.0f} cm del tronco (mín. {SEP_TRONCO / 10:.0f}): mover el robot"
    return tronco, pasos, aviso


def buscar(base, marcas, id_=None, paso=PASO_BUSCAR, seg=SEG_BUSCAR):
    """BUSCAR: mira seg s; si no ve la marca id_, gira paso° a la izquierda (giróscopo) y vuelve a mirar, hasta dar
    la vuelta entera. True si la encontró (el robot queda mirándola)."""
    n = int(math.ceil(360.0 / paso))
    for k in range(n + 1):
        m = marcas.medir(base.girar_ros, seg, id_, minimo=3)
        if m is not None:
            print(f"  BUSCAR: banderín (ID {m[0]}) a {math.hypot(m[1][0], m[1][1]) / 1000:.2f} m, después de girar "
                  f"{k * paso:.0f}°")
            return True
        if k == n:
            break
        print(f"  BUSCAR: no lo veo ({marcas.imgs} imágenes); giro {paso:.0f}° a la izquierda ({k + 1}/{n})")
        base.armar()
        r = base.mover("giro", paso, V_RECTO, W_GIRO)   # frena y espera 1 s antes de volver
        if not r["ok"]:
            print(f"[BUSCAR cortado: {r['motivo']}]")
            return False
    print("NO: di la vuelta entera y no vi el banderín. ¿Visión corriendo? ¿A 0,9–4 m, de frente, con su margen "
          "blanco y bien iluminado?")
    return False


def medir_y_planear(base, marcas, radio, parada, orientar=True, banderin=None):
    """Mira la marca 3 s y devuelve los pasos hasta la parada ([] si ya está), o None si no se puede.
    Con banderin (cargar_banderin), el árbol sale del banderín y no de la marca de la maceta."""
    print("mirando la marca 3 s (robot quieto)...")
    m = marcas.medir(base.girar_ros, 3.0, banderin["id"] if banderin else None)
    if m is None:
        print(f"NO: no veo la marca ({marcas.imgs} imágenes). ¿Visión corriendo (~/scripts/vision.sh)? ¿Marca a "
              f"~0,9–4 m a la izquierda, de frente?")
        return None
    i, pos, mira, nv = m
    if banderin:
        tronco, pasos, aviso = plan_banderin(pos, mira, banderin, parada, orientar)
        print(f"  banderín ID {i} ({nv} vistas) a {math.hypot(pos[0], pos[1]) / 10:.0f} cm, en ({pos[0]:.0f}, "
              f"{pos[1]:.0f}) mm, mira a {mira:+.0f}° -> árbol a {-tronco[0] / 10:.0f} cm a la izquierda de J1, "
              f"{tronco[1] / 10:+.0f} cm adelante")
        if aviso:
            print(f"NO: {aviso} (plan: {texto(pasos)})")
            return None
        lim = MAX_RECTO_BANDERIN
    else:
        tronco = tronco_desde(pos, mira, radio)
        print(f"  marca ID {i} ({nv} vistas): cara en ({pos[0]:.0f}, {pos[1]:.0f}) mm, mira a {mira:+.0f}° -> tronco "
              f"a {-tronco[0] / 10:.0f} cm a la izquierda de J1, {tronco[1] / 10:+.0f} cm adelante")
        pasos = planear(tronco, mira, parada, orientar)
        lim = MAX_RECTO
    final, _ = simular_pasos(tronco, pasos)
    if any(t == "recto" and abs(v) > lim for t, v in pasos):
        print(f"NO: el plan avanza más de {lim / 1000:.1f} m: acercar el robot primero ({texto(pasos)})")
        return None
    print(f"plan de la base: {texto(pasos)}")
    print(f"  (queda con el tronco en ({final[0]:.0f}, {final[1]:.0f}) mm; parada ({parada[0]:.0f}, {parada[1]:.0f}))")
    return pasos


def ejecutar(base, pasos):
    """Hace los pasos (arma la base si hace falta). True si llegó. Los rectos largos van en tramos de MAX_TRAMO."""
    from movimiento import informe
    base.armar()
    for tipo, val in pasos:
        print(f"== {texto([(tipo, val)])}")
        if tipo == "giro":
            tramos = [("giro", val)]
        else:
            n = max(1, int(math.ceil(abs(val) / MAX_TRAMO)))
            tramos = [("recto", val / n / 1000.0)] * n
        for modo, obj in tramos:
            r = base.mover(modo, obj, V_RECTO, W_GIRO)
            informe(r)
            if not r["ok"]:
                print("[cortado: el robot quedó donde está]")
                return False
    print("llegó a la parada (según la odometría).")
    return True


def mirar_al_llegar(base, marcas, radio, parada, banderin=None):
    m = marcas.medir(base.girar_ros, 2.0, banderin["id"] if banderin else None)
    if m:
        tr = tronco_desde_banderin(m[1], m[2], banderin) if banderin else tronco_desde(m[1], m[2], radio)
        print(f"  la marca todavía se ve: tronco a {-tr[0] / 10:.0f} cm, {tr[1] / 10:+.0f} cm (parada "
              f"{-parada[0] / 10:.0f}, {parada[1] / 10:+.0f})")
    elif not banderin:
        print("  (la marca ya no se ve completa, como se esperaba a esta distancia)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radio-maceta", type=float, default=165.0, help="mm del eje del tronco a la cara de la marca")
    ap.add_argument("--parada", type=float, nargs=2, default=[700.0, 150.0], metavar=("D", "Y"))
    ap.add_argument("--sin-orientacion", action="store_true", help="no girar para quedar de frente a la marca")
    ap.add_argument("--simular", type=float, nargs=3, metavar=("X", "Y", "MIRA"))
    ap.add_argument("--rodear", type=int, default=0, metavar="N",
                    help="después de la parada, rodear N veces (90° antihorario cada una) sin el brazo")
    ap.add_argument("--observar", type=float, nargs=2, default=[1100.0, 150.0], metavar=("D", "Y"),
                    help="al rodear: tronco a D mm a la izquierda de J1 e Y adelante (donde se mira la marca)")
    ap.add_argument("--auto", type=float, default=None, metavar="S",
                    help="sin Enter: espera S segundos (para alejarse) y hace todo solo; parar con la seta")
    ap.add_argument("--banderin", nargs="?", const=str(BANDERIN), default=None, metavar="JSON",
                    help=f"una sola marca en un palo; el árbol según el JSON (def. {BANDERIN.name})")
    ap.add_argument("--arbol", type=float, nargs=2, default=None, metavar=("DERECHA", "FRENTE"),
                    help="mm, pisa arbol_derecha_mm / arbol_frente_mm del JSON")
    ap.add_argument("--buscar", action="store_true", help="si no ve la marca, girar de a 30° hasta encontrarla")
    a = ap.parse_args()
    parada = (-a.parada[0], a.parada[1])
    observar = (-a.observar[0], a.observar[1])
    try:
        ban = cargar_banderin(a.banderin or BANDERIN, a.arbol) if a.banderin or a.arbol else None
    except (OSError, ValueError) as e:
        sys.exit(f"NO: {e}")

    if a.simular:
        if ban:
            pos, mira = np.array(a.simular[:2]), a.simular[2]
            tronco, pasos, aviso = plan_banderin(pos, mira, ban, parada, not a.sin_orientacion)
            print(f"banderín en ({pos[0]:.0f}, {pos[1]:.0f}) mirando a {mira:+.0f}° -> árbol en ({tronco[0]:.0f}, "
                  f"{tronco[1]:.0f}) | J1 pasa a {distancia_minima(pos, pasos) / 10:.0f} cm del banderín y "
                  f"{distancia_minima(tronco, pasos) / 10:.0f} cm del tronco" + (f"\n  NO: {aviso}" if aviso else ""))
        else:
            tronco, mira = a.simular[:2], a.simular[2]
            pasos = planear(tronco, mira, parada, not a.sin_orientacion)
        final, h = simular_pasos(tronco, pasos)
        print(f"tronco en ({tronco[0]:.0f}, {tronco[1]:.0f}), marca mirando a {mira:+.0f}°: {texto(pasos)}")
        print(f"  verificación: el tronco queda en ({final[0]:.0f}, {final[1]:.0f}) (parada {parada}), giro total {h:+.1f}°")
        return

    import rclpy
    from marcas import Marcas
    from movimiento import Base

    rclpy.init()
    n = rclpy.create_node("ir_a_parada")
    base, marcas = Base(n), Marcas(n)
    try:
        base.esperar_datos()
        if a.buscar:
            if not a.auto and input("BUSCAR: el robot va a girar en el lugar. Enter para empezar (otra cosa "
                                    "cancela): ").strip():
                print("cancelado")
                return
            if not buscar(base, marcas, ban["id"] if ban else None):
                return
        pasos = medir_y_planear(base, marcas, a.radio_maceta, parada, not a.sin_orientacion, ban)
        if pasos is None or (not pasos and not a.rodear):
            return
        if a.auto is not None:
            for k in range(int(a.auto), 0, -1):
                print(f"  arranca solo en {k} s (parar: seta)", flush=True)
                base.girar_ros(1.0)
        elif input("Enter para ir (cualquier otra cosa cancela): ").strip():
            print("cancelado")
            return
        if pasos and not ejecutar(base, pasos):
            return
        mirar_al_llegar(base, marcas, a.radio_maceta, parada, ban)
        for k in range(a.rodear):
            if a.auto is not None:
                print(f"rodear a la parada {k + 2} en 3 s", flush=True)
                base.girar_ros(3.0)
            elif input(f"Enter para rodear a la parada {k + 2} (cualquier otra cosa termina): ").strip():
                return
            if ban:   # sin marca en la maceta: directo a la parada siguiente con la odometría
                pasos = planear(parada, 90.0, parada)
                print(f"== rodear (odometría): {texto(pasos)} (J1 a {distancia_minima(parada, pasos) / 10:.0f} cm "
                      f"del tronco)")
                if not ejecutar(base, pasos):
                    return
                continue
            pasos = planear_rodeo(parada, observar)
            print(f"== rodear: {texto(pasos)} (J1 a {distancia_minima(parada, pasos) / 10:.0f} cm del tronco)")
            if not ejecutar(base, pasos):
                return
            base.girar_ros(1.0)
            pasos = medir_y_planear(base, marcas, a.radio_maceta, parada, not a.sin_orientacion)
            if pasos is None or (pasos and not ejecutar(base, pasos)):
                return
            mirar_al_llegar(base, marcas, a.radio_maceta, parada)
        if not a.rodear:
            print("siguiente: en punta.py, \"veragarrar\" y \"agarrar\"  (o todo junto: tools/recolectar.py)")
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
