#!/usr/bin/env python3
"""Recolectar: TODO en un programa. Homing del brazo -> ver la marca -> ir a la parada -> ver el limón -> agarrar.

En la Pi, con la visión (~/scripts/vision.sh) y el agente de la base (~/scripts/agente_base.sh) corriendo, y la WROOM
con el firmware serie del brazo (env `brazo`, el de punta.py). Reutiliza ir_a_parada.py (base),
25-teleop-pi/tools/movimiento.py (odometría) y 20-brazo-coordinado/tools/punta.py (agarrar).

Orden (lo va pidiendo el programa):
  1. 12 V APAGADOS, brazo abajo y recogido: abre el puerto de la WROOM (eso la reinicia).
  2. "Encendé los 12 V" + Enter -> homing del brazo (h) y reposo (mv 3 17 4).
  3. Mira la marca de la maceta (hace falta verla a ~0,9–4 m a la izquierda) y muestra el plan de la base.
  4. Enter -> la base va a la parada y cosecha --limones (def. 1): mide un limón 5 s, lo agarra (tirar, girar la
     muñeca, arrastrar con J1), lo lleva directo a la canasta (sin pasar por el reposo), mide el siguiente desde ahí;
     al final, el reposo (punta.cosechar). --sin-soltar: uno solo, queda en la pinza.
  5. Con --paradas N > 1, RODEAR: Enter -> va con la
     odometría al punto de observación de la cara siguiente (90° antihorario alrededor del tronco, el árbol a la
     izquierda, tronco a --observar del J1: ahí la marca se ve entera), mira esa marca, corrige hasta la parada como
     en el paso 4 y agarra. Así hasta N paradas (las marcas 0, 1, 2, 3 en sentido antihorario).
Ctrl+C en cualquier momento: frena la base y manda "s" al brazo. Al final: "pz a" en punta.py suelta el limón.

Uso: python3 tools/recolectar.py [--paradas 1] [--radio-maceta 165] [--parada 700 150] [--observar 1100 150]
     [--sin-orientacion]   (siempre hace el homing: abrir el puerto reinicia la WROOM y pierde el cero)
"""
import argparse
import sys
import time
from pathlib import Path

AQUI = Path(__file__).absolute().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent.parent / "25-teleop-pi" / "tools"))
sys.path.insert(0, str(AQUI.parent.parent / "20-brazo-coordinado" / "tools"))

import ir_a_parada as iap  # noqa: E402

TIMEOUT_MV = 90.0
TIMEOUT_HOMING = 120.0


def preguntar(texto):
    try:
        return input(texto).strip() == ""
    except EOFError:
        return False


def homing(placa):
    placa.vaciar()
    print("<< h")
    placa.mandar("h")
    l = placa.esperar(("HOMING,", "HOMING_ERROR"), TIMEOUT_HOMING)
    if l is None or l.startswith("HOMING_ERROR"):
        placa.mandar("s")
        print(f"NO: el homing no terminó ({l or 'sin respuesta'})")
        return False
    return True


def rodear(base, marcas, placa, a, parada, observar, k):
    """Va al punto de observación de la cara siguiente y mira la marca. Devuelve los pasos hasta la
    parada siguiente ([] si ya está) o None si hay que cortar."""
    if not preguntar(f"5) Enter para rodear a la parada {k + 1} (cualquier otra cosa termina): "):
        print("terminado")
        return None
    pasos = iap.planear_rodeo(parada, observar)   # el tronco está donde lo dejó la parada (según la odometría)
    print(f"== rodear: {iap.texto(pasos)} (J1 a {iap.distancia_minima(parada, pasos) / 10:.0f} cm del tronco como "
          f"mínimo)")
    if not iap.ejecutar(base, pasos):
        return None
    base.girar_ros(1.0)   # quieta antes de mirar (la marca que ve tiene que ser la siguiente: IDs antihorario)
    return iap.medir_y_planear(base, marcas, a.radio_maceta, parada, not a.sin_orientacion)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radio-maceta", type=float, default=165.0, help="mm del eje del tronco a la cara de la marca")
    ap.add_argument("--parada", type=float, nargs=2, default=[700.0, 150.0], metavar=("D", "Y"))
    ap.add_argument("--observar", type=float, nargs=2, default=[1100.0, 150.0], metavar=("D", "Y"),
                    help="al rodear: tronco a D mm a la izquierda de J1 e Y adelante (más lejos que la parada)")
    ap.add_argument("--paradas", type=int, default=1, help="cuántas caras recorrer (1 = solo la primera, máx 4)")
    ap.add_argument("--sin-orientacion", action="store_true")
    ap.add_argument("--sin-soltar", action="store_true", help="no llevar el limón a la canasta (queda en la pinza)")
    ap.add_argument("--limones", type=int, default=1, help="cuántos limones cosechar en cada parada (def. 1)")
    ap.add_argument("--puerto", default=None, help="puerto de la WROOM (def. el de punta.py)")
    a = ap.parse_args()
    parada = (-a.parada[0], a.parada[1])
    observar = (-a.observar[0], a.observar[1])
    if not 1 <= a.paradas <= 4:
        sys.exit("--paradas entre 1 y 4")

    import rclpy
    import punta
    from marcas import Marcas
    from movimiento import Base

    tf = punta.cargar_tf_agarre()
    print("1) Los 12 V tienen que estar APAGADOS y el brazo abajo y recogido (abrir el puerto reinicia la WROOM).")
    if not preguntar("   Enter cuando estén apagados: "):
        print("cancelado")
        return
    placa = punta.Placa(a.puerto or punta.PUERTO)
    time.sleep(2.0)
    rclpy.init()
    n = rclpy.create_node("recolectar")
    base, marcas, vision = Base(n), Marcas(n), punta.Vision(n)
    try:
        base.esperar_datos()
        print("2) Encendé los 12 V.")
        if not preguntar("   Enter cuando estén encendidos (empieza el homing del brazo): "):
            print("cancelado")
            return
        if not homing(placa):
            return
        print("== brazo al reposo")
        if not punta.ejecutar(placa, [punta.REPOSO], TIMEOUT_MV):
            return
        print("3) Base: buscando la marca de la maceta")
        pasos = iap.medir_y_planear(base, marcas, a.radio_maceta, parada, not a.sin_orientacion)
        if pasos is None:
            return
        if not preguntar("4) Enter para ir a la parada y agarrar (cualquier otra cosa cancela): "):
            print("cancelado")
            return
        t0 = time.time()
        for k in range(a.paradas):
            if k > 0:
                pasos = rodear(base, marcas, placa, a, parada, observar, k)
                if pasos is None:
                    return
            if pasos and not iap.ejecutar(base, pasos):
                return
            pasos = None
            iap.mirar_al_llegar(base, marcas, a.radio_maceta, parada)
            print(f"== agarrar el limón (parada {k + 1} de {a.paradas})")
            base.girar_ros(1.0)   # que la base quede quieta antes de medir
            if a.sin_soltar:
                if punta.agarrar(placa, vision, tf, 5.0, True, TIMEOUT_MV, confirmar=False):
                    print("(--sin-soltar: queda en la pinza; \"pz a\" lo suelta)")
                return
            # Cosecha (18/11): mide uno, lo lleva a la canasta sin pasar por el reposo y mide el siguiente desde ahí
            n = punta.cosechar(placa, vision, tf, a.limones, 5.0, TIMEOUT_MV, confirmar=False)
            print(f"parada {k + 1}: {n} limones en la canasta, {time.time() - t0:.0f} s desde que arrancó la base")
    except KeyboardInterrupt:
        base.frenar()
        placa.mandar("s")
        print("Ctrl+C: base frenada y brazo sostenido donde está (s)")
    except (RuntimeError, ValueError) as e:
        print(f"NO: {e}")
    finally:
        placa.vivo = False
        placa.s.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
