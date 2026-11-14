#!/usr/bin/env python3
"""Recolectar: TODO en un programa. Homing del brazo -> ver la marca -> ir a la parada -> ver el limón -> agarrar.

En la Pi, con la visión (~/scripts/vision.sh) y el agente de la base (~/scripts/agente_base.sh) corriendo, y la WROOM
con el firmware serie del brazo (env `brazo`, el de punta.py). Reutiliza ir_a_parada.py (base),
25-teleop-pi/tools/movimiento.py (odometría) y 20-brazo-coordinado/tools/punta.py (agarrar).

Orden (lo va pidiendo el programa):
  1. 12 V APAGADOS, brazo abajo y recogido: abre el puerto de la WROOM (eso la reinicia).
  2. "Encendé los 12 V" + Enter -> homing del brazo (h) y reposo (mv 3 17 4).
  3. Mira la marca de la maceta (hace falta verla a ~0,9–4 m a la izquierda) y muestra el plan de la base.
  4. Enter -> la base va a la parada, mide el limón 5 s y lo agarra (sin más preguntas). Queda con el limón en la
     pinza, en el reposo.
Ctrl+C en cualquier momento: frena la base y manda "s" al brazo. Al final: "pz a" en punta.py suelta el limón.

Uso: python3 tools/recolectar.py [--radio-maceta 165] [--parada 700 150] [--sin-orientacion]
     (siempre hace el homing: abrir el puerto reinicia la WROOM y pierde el cero)
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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--radio-maceta", type=float, default=165.0, help="mm del eje del tronco a la cara de la marca")
    ap.add_argument("--parada", type=float, nargs=2, default=[700.0, 150.0], metavar=("D", "Y"))
    ap.add_argument("--sin-orientacion", action="store_true")
    ap.add_argument("--puerto", default=None, help="puerto de la WROOM (def. el de punta.py)")
    a = ap.parse_args()
    parada = (-a.parada[0], a.parada[1])

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
        if pasos and not iap.ejecutar(base, pasos):
            return
        iap.mirar_al_llegar(base, marcas, a.radio_maceta, parada)
        print("== agarrar el limón")
        base.girar_ros(1.0)   # que la base quede quieta antes de medir
        if punta.agarrar(placa, vision, tf, 5.0, True, TIMEOUT_MV, confirmar=False):
            print(f"LISTO: limón agarrado en {time.time() - t0:.0f} s desde que arrancó la base. \"pz a\" lo suelta.")
        else:
            print("el agarre no se hizo (ver arriba por qué); el robot quedó en la parada")
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
