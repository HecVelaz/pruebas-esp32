#!/usr/bin/env python3
"""Mover la punta (centro de la pinza) de 20 a un punto en mm, con el modelo medido de tools/cinematica.py.

Abre el puerto como un monitor: lo que se escribe va a la placa (h, c, mv, s, x, ...) salvo estos comandos propios:
    p <x> <y> <z>     llevar la punta a (x, y, z) mm: inversa, revisa el destino y el camino y manda el/los mv
    ver <x> <y> <z>   lo mismo sin mover nada (solo el plan)
    ap <x> <y> <z> [d]     llegar DE FRENTE (para agarrar): primero a d mm (def. 100) antes del punto, en la misma
                      dirección desde J1 y a la misma altura, y después entra hacia afuera, horizontal, con J1
                      quieta (un mv). verap: solo el plan. Cada mv llega desde 2° abajo (juego): la punta sube
                      hasta ~1,3 cm en el medio de la entrada y de la salida
    ret [d] [dz]      salir de frente y horizontal: retrocede d mm (def. 100) hacia J1 con J1 quieta (dz: subir antes, def. 0)
    pz ... / mn ...   pinza y muñeca (van a la placa: pz a/c/0, mn a/b/0, ...)
    tarea [xA yA zA xB yB zB]   agarrar en A y dejar en B, solo (def. A = -550 100 100, B = -450 350 100):
                      planea todo antes y pide Enter; recoge, gira J1 recogido, abre la pinza, entra de frente a A,
                      cierra (pz 1250), gira la muñeca (mn a), sale de frente, igual hacia B, abre, sale, muñeca a 0, reposo
                      (mv 3 17 4). Un mv que corta por "no quedan firmes" se repite una vez.
                      vertarea: solo el plan. Ctrl+C en cualquier momento: "s"
    donde             pide "c" y dice dónde está la punta
    salir             (o Ctrl+D)
Marco del robot (el de cinematica.py): origen en el eje de J1 a la altura de la tapa; x a la derecha del robot
(J1 está a la izquierda y atrás, el mástil a la derecha), y adelante (hacia los ultrasonidos), z arriba. Ej.: "p 0 500 300" = 50 cm adelante y 30 cm sobre la tapa (J1 en 90°).

Antes de mandar un "p" revisa, además de lo de cinematica.py (límites, franja, q3 - q2 >= -25°, choques del destino):
  - el camino de J2/J3 como lo hace el firmware (recta hasta el punto previo de la aproximación, 2° abajo, y el
    último tramo) dentro de la franja con MARGEN_CAMINO, para que el firmware no elija otro camino por su cuenta;
  - choques con la caja, el mástil y el piso en todo el camino, para CUALQUIER combinación de J1 con J2/J3 (J1
    tiene su propia duración en el mv: no se sabe en qué ángulo de J1 está cada punto de J2/J3).
Si el camino recto no sirve, busca un punto intermedio (J1 quieta en el primer tramo, como el firmware) con los dos
tramos libres y manda dos mv. El firmware no conoce la caja: este script es lo único que la esquiva.
Si el firmware igual dice que el camino sale de la franja (eligió su propio intermedio) o algo no llega en
--timeout s, manda "s" (las tres se sostienen donde están). Ctrl+C durante un movimiento: "s".

Abrir el puerto reinicia la WROOM (cero perdido): primero "h". Orden de la fuente: 12 V apagados, brazo abajo y
recogido, abrir esto, 12 V encendidos, "h", los "p" ... al final "mv 3 10 5" y apagar los 12 V.

Uso:
    python3 tools/punta.py                                   # sesión con la placa
    python3 tools/punta.py --simular --desde 3 10 5 0 520 80  # solo el plan, sin la placa
"""
import argparse
import math
import queue
import re
import sys
import threading
import time

import numpy as np

import cinematica as k

PUERTO = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
# Del firmware (include/config.h)
Q2_MENOS_Q3_MAX = 49.0     # borde de abajo de la franja en el firmware (el camino puede pasar por ahí)
MARGEN_CAMINO = 1.2        # ° (el firmware usa 1,0: un poco más para que acepte la recta que elegimos acá)
APROX = 2.0                # APROX_GRADOS de J2 y J3
TOLPOS = 0.2               # si se mueve menos que esto no hace aproximación
LIM_MIN = 3.0              # límite de abajo con MARGEN_LIMITE
PASO_Q2Q3 = 1.0            # ° entre muestras del camino de J2/J3
PASO_Q1 = 5.0              # ° entre los ángulos de J1 que se combinan con cada muestra
PASO_INTERMEDIO = 5.0      # grilla de puntos intermedios (como el firmware)


# ------------------------------------------------------------------------------------------------- plan (sin placa)

def exceso(q2, q3):
    """Cuánto sale (°) de la franja del firmware: <= 0 adentro."""
    return max(q3 - k.q3_max_franja(q2), max(q2 - Q2_MENOS_Q3_MAX, k.q3_min_tabla(q2)) - q3)


def tramos(s, o):
    """Los tramos de J2/J3 de un mv de s a o como en el firmware: recta al punto previo de la aproximación y el final."""
    pr = tuple(o[i] - APROX if abs(o[i] - s[i]) > TOLPOS and o[i] - APROX >= LIM_MIN else o[i] for i in (1, 2))
    return [((s[1], s[2]), pr), (pr, (o[1], o[2]))]


def muestras(s, o):
    for a, b in tramos(s, o):
        n = 1 + int(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / PASO_Q2Q3)
        for t in np.linspace(0, 1, n + 1):
            yield a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])


def franja_camino(s, o):
    tolerado = max(-MARGEN_CAMINO, exceso(s[1], s[2]))
    return all(exceso(q2, q3) <= tolerado for q2, q3 in muestras(s, o))


def choques_camino(s, o):
    if abs(o[0] - s[0]) < 1e-6:
        q1s = [s[0]]
    else:
        q1s = np.linspace(s[0], o[0], 2 + int(abs(o[0] - s[0]) / PASO_Q1))
    m = set()
    for q2, q3 in muestras(s, o):
        for q1 in q1s:
            m.update(k.choques(k.directa(q1, q2, q3)[1]))
    return sorted(m)


def revisar_camino(s, o):
    m = [] if franja_camino(s, o) else ["el camino de J2/J3 sale de la franja"]
    return m + choques_camino(s, o)


def intermedio_valido(q2, q3):
    return (k.Q2_LIM[0] <= q2 <= k.Q2_LIM[1] and k.Q3_LIM[0] <= q3 <= k.Q3_LIM[1]
            and q3 <= k.q3_max_franja(q2) - k.MARGEN_FRANJA and q3 - q2 >= k.Q3_MENOS_Q2_MIN + k.MARGEN_FRANJA)


def planear(s, x, y, z):
    """Desde los ángulos s, a la punta (x, y, z). Devuelve (lista de mv, avisos) o (None, motivos)."""
    q, mot = k.inversa(x, y, z)
    if q is None or mot:
        return None, mot
    return planear_q(s, tuple(round(v, 2) for v in q))


def radio_punta(q1, q2, q3):
    p, _ = k.directa(q1, q2, q3)
    return math.hypot(p[0], p[1])


def planear_q(s, o, r_max=None):
    """Desde los ángulos s a los ángulos o. Con r_max, el intermedio (si hace falta) deja la punta a <= r_max mm de
    J1 (recogida). Devuelve (lista de mv, avisos) o (None, motivos)."""
    avisos = []
    if k.fuera_de_lo_medido(o[1], o[2]):
        avisos.append("el destino está fuera de la zona medida (el modelo extrapola)")
    directo = revisar_camino(s, o)
    if not directo:
        return [o], avisos
    # Punto intermedio P: J1 quieta en el primer tramo; los dos tramos dentro de la franja y sin choques; el más corto.
    # J1 dentro de sus límites: después del homing queda en ~2,8° y el mv solo acepta desde 3° (2026-11-05)
    p1 = min(max(s[0], k.Q1_LIM[0]), k.Q1_LIM[1])
    cands = []
    g2 = np.arange(0.0, k.Q2_LIM[1] + 0.01, PASO_INTERMEDIO)
    g3 = np.arange(0.0, k.Q3_LIM[1] + 0.01, PASO_INTERMEDIO)
    for p2 in list(g2) + [o[1], s[1]]:
        for p3 in list(g3) + [s[2], o[2]]:
            if not intermedio_valido(p2, p3):
                continue
            if r_max is not None and radio_punta(p1, p2, p3) > r_max:
                continue
            largo = max(abs(p2 - s[1]), abs(p3 - s[2])) + max(abs(o[1] - p2), abs(o[2] - p3))
            cands.append((largo, (p1, float(p2), float(p3))))
    for _, p in sorted(cands):
        if franja_camino(s, p) and franja_camino(p, o) and not choques_camino(s, p) and not choques_camino(p, o):
            avisos.append(f"el camino recto no sirve ({'; '.join(directo)}): pasa por J2 {p[1]:.1f}° J3 {p[2]:.1f}°")
            return [tuple(round(v, 2) for v in p), o], avisos
    return None, [f"camino recto: {'; '.join(directo)}", "no encontré un punto intermedio libre"]


PASO_FRENTE = 100.0  # mm por mv en la entrada y la salida de frente (11/11: un solo mv, más parejo que de a 2 cm)


def recta_j1_quieta(q, desde, hasta):
    """mv de a PASO_FRENTE mm de desde a hasta (puntos x, y, z), cada uno directo y con J1 igual. (lista, motivos)."""
    desde, hasta = np.asarray(desde, float), np.asarray(hasta, float)
    n = max(1, int(math.ceil(np.linalg.norm(hasta - desde) / PASO_FRENTE - 1e-3)))
    plan = []
    for i in range(1, n + 1):
        pi = desde + (hasta - desde) * i / n
        pl, msgs = planear(q, *pi)
        if pl is None:
            return None, [f"paso {i}/{n} ({pi[0]:.0f}, {pi[1]:.0f}, {pi[2]:.0f}): " + "; ".join(msgs)]
        if len(pl) != 1 or abs(pl[0][0] - q[0]) > 0.05:
            return None, [f"paso {i}/{n}: no se puede en línea recta con J1 quieta (pide un intermedio)"]
        plan.append(pl[0])
        q = pl[0]
    return plan, []


def planear_frente(s, x, y, z, d=100.0):
    """Llegar de frente: al punto previo (d mm antes, misma dirección desde J1, misma altura) y entrar en recta."""
    r = math.hypot(x, y)
    if r <= d + 1:
        return None, ["el punto está muy cerca de J1 para entrar de frente"]
    u = np.array([x / r, y / r, 0.0])
    pre = np.array([x, y, z]) - d * u
    qp, mot = k.inversa(*pre)
    if qp is None or mot:
        return None, [f"punto previo ({pre[0]:.0f}, {pre[1]:.0f}, {pre[2]:.0f}): " + "; ".join(mot)]
    qp = tuple(round(v, 2) for v in qp)
    # 1) J2/J3 a la pose del punto previo con J1 donde está (recogido: la punta nunca más lejos que el punto previo,
    #    para no barrer de costado cerca del objeto); 2) J1 sola hasta la dirección del objeto; 3) entrada de frente
    r_pre = math.hypot(*pre[:2]) + 20.0
    a = (round(min(max(s[0], k.Q1_LIM[0]), k.Q1_LIM[1]), 2), qp[1], qp[2])   # J1 donde está, dentro de sus límites
    pl, avisos = planear_q(s, a, r_max=r_pre)
    if pl is None:
        return None, ["recoger a la pose del punto previo: " + "; ".join(avisos)]
    if radio_punta(*s) > r_pre and abs(s[0] - qp[0]) > 0.05:
        avisos.append("la punta arranca más lejos que el punto previo: revisar que no toque nada al recogerse")
    if abs(qp[0] - s[0]) > 0.05:
        giro = revisar_camino(a, qp)
        if giro:
            return None, ["girar J1 recogido: " + "; ".join(giro)]
        pl = pl + [qp]
    entrada, msgs = recta_j1_quieta(pl[-1], pre, (x, y, z))
    if entrada is None:
        return None, msgs
    return pl + entrada, avisos


def planear_salida(s, d=100.0, dz=0.0):
    """Salir de frente desde los ángulos s: subir dz y retroceder d hacia J1, con J1 quieta."""
    p, _ = k.directa(*s)
    r = math.hypot(p[0], p[1])
    u = np.array([p[0] / r, p[1] / r, 0.0])
    arriba = p + [0.0, 0.0, dz]
    sube, msgs = recta_j1_quieta(tuple(s), p, arriba) if dz > 0 else ([], [])
    if sube is None:
        return None, msgs
    atras, msgs = recta_j1_quieta(sube[-1] if sube else tuple(s), arriba, arriba - d * u)
    if atras is None:
        return None, msgs
    return sube + atras, []


def imprimir_plan(plan, msgs):
    if plan is None:
        print("NO:", "; ".join(msgs))
        return None
    for m in msgs:
        print("  ojo:", m)
    for q in plan:
        p, _ = k.directa(*q)
        print(f"  mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}   (punta x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm)")
    return plan


def mostrar_plan(s, x, y, z):
    plan, msgs = planear(s, x, y, z)
    if plan is None:
        print("NO:", "; ".join(msgs))
        return None
    for m in msgs:
        print("  ojo:", m)
    for q in plan:
        p, _ = k.directa(*q)
        print(f"  mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}   (punta x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm)")
    return plan


# ------------------------------------------------------------------------------------------------------ con la placa

class Placa:
    def __init__(self, puerto):
        import serial
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = puerto, 115200, 0.1
        self.s.dtr = self.s.rts = False  # como tools/monitor.sh (igual puede reiniciarse al abrir)
        self.s.open()
        self.lineas = queue.Queue()
        self.vivo = True
        threading.Thread(target=self._leer, daemon=True).start()

    def _leer(self):
        while self.vivo:
            try:
                l = self.s.readline().decode(errors="replace").strip()
            except Exception as e:  # puerto desconectado
                print(f"\n[puerto: {e}]")
                self.vivo = False
                return
            if not l or l.startswith("T,"):
                continue
            print(l)
            self.lineas.put(l)

    def vaciar(self):
        while not self.lineas.empty():
            self.lineas.get_nowait()

    def mandar(self, c):
        self.s.write(f"{c}\n".encode())

    def esperar(self, prefijos, segundos):
        """La primera línea que empieza con alguno de prefijos, o None."""
        t_lim = time.time() + segundos
        while time.time() < t_lim:
            try:
                l = self.lineas.get(timeout=0.1)
            except queue.Empty:
                continue
            if l.startswith(prefijos):
                return l
        return None

    def angulos(self):
        """(q1, q2, q3) pidiendo "c"; None si alguna no tiene cero."""
        self.vaciar()
        self.mandar("c")
        q, cero = {}, {}
        t_lim = time.time() + 2.0
        while time.time() < t_lim and len(q) < 3:
            try:
                l = self.lineas.get(timeout=0.1)
            except queue.Empty:
                continue
            m = re.match(r"(J[123])\s+([+-]?\d+(?:\.\d+)?)°.*\| (con cero|SIN CERO)", l)
            if m:
                q[m.group(1)] = float(m.group(2))
                cero[m.group(1)] = m.group(3) == "con cero"
        if len(q) < 3:
            print("[no pude leer los ángulos con \"c\"]")
            return None
        if not all(cero.values()):
            print("[alguna articulación SIN CERO: primero \"h\"]")
            return None
        return q["J1"], q["J2"], q["J3"]


FIN_MV = ("LLEGO", "MV_ERROR", "Fuera de", "J2 y J3 fuera", "Uso:", "Hardware sin", ">> El camino recto sale")
RECHAZO = ("Fuera de", "J2 y J3 fuera", "Uso:", "Hardware sin")


def ejecutar(placa, plan, timeout, reintentos=1):
    """Manda los mv de a uno esperando LLEGO. Si uno corta por "no quedan firmes" (J2 sin fuerza con carga, 11/11), lo
    repite hasta reintentos veces desde donde quedó."""
    for q in plan:
        c = f"mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}"
        for intento in range(reintentos + 1):
            placa.vaciar()
            print(f"<< {c}" + (f"   (reintento {intento})" if intento else ""))
            placa.mandar(c)
            try:
                l = placa.esperar(FIN_MV + tuple(f"{j} sin cero" for j in ("J1", "J2", "J3")), timeout)
            except KeyboardInterrupt:
                placa.mandar("s")
                print("[Ctrl+C: \"s\", las tres se sostienen donde están]")
                return False
            if l is None:
                placa.mandar("s")
                print(f"[{timeout:.0f} s sin LLEGO: \"s\"]")
                return False
            if l.startswith(">> El camino recto sale"):
                placa.mandar("s")
                print("[el firmware eligió otro camino (no revisado contra la caja): \"s\". Probar otro punto]")
                return False
            if l.startswith("LLEGO"):
                break
            if l.startswith("MV_ERROR") and "no quedan firmes" in l and intento < reintentos:
                time.sleep(0.5)
                continue
            if l.startswith(RECHAZO) or "sin cero" in l:
                print("[la placa rechazó el mv]")
            return False
    return True


TAREA_A = (-550.0, 100.0, 100.0)
PINZA_AGARRE_US = 1250   # = pz c. 11/11: con 1250 una vez a J2 no le alcanzó para salir (con 1150 sí, pero agarra flojo)
TAREA_B = (-450.0, 350.0, 100.0)
REPOSO = (3.0, 17.0, 4.0)


def planear_tarea(s, A, B):
    """Los tramos de la tarea: lista de (nombre, lista de mv) y comandos de servos entre medio. (pasos, motivos)."""
    pasos = []
    pa, m = planear_frente(s, *A)
    if pa is None:
        return None, ["hacia A: " + "; ".join(m)]
    sa, m = planear_salida(pa[-1])
    if sa is None:
        return None, ["salida de A: " + "; ".join(m)]
    pb, m = planear_frente(sa[-1], *B)
    if pb is None:
        return None, ["hacia B: " + "; ".join(m)]
    sb, m = planear_salida(pb[-1])
    if sb is None:
        return None, ["salida de B: " + "; ".join(m)]
    rp, m = planear_q(sb[-1], REPOSO)
    if rp is None:
        return None, ["al reposo: " + "; ".join(m)]
    pasos += [("ir frente a A (recogido)", pa[:-1]), ("servo", "pz a"), ("entrar de frente a A", pa[-1:]),
              ("servo", f"pz {PINZA_AGARRE_US}"), ("servo", "mn a"), ("salir de frente de A", sa),
              ("ir frente a B (recogido)", pb[:-1]), ("entrar de frente a B", pb[-1:]), ("servo", "pz a"),
              ("salir de frente de B", sb), ("servo", "mn 0"), ("reposo", rp)]
    return pasos, []


def imprimir_tarea(pasos):
    for nombre, cosa in pasos:
        if nombre == "servo":
            print(f"  [{cosa}]")
            continue
        print(f"  {nombre}:")
        for q in cosa:
            p, _ = k.directa(*q)
            print(f"    mv {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}   (punta x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm)")


def servo(placa, cmd, timeout=8.0):
    placa.vaciar()
    print(f"<< {cmd}")
    placa.mandar(cmd)
    l = placa.esperar(("SERVO_LLEGO",), timeout)
    if l is None:
        print(f"[{cmd}: sin SERVO_LLEGO en {timeout:.0f} s]")
        return False
    time.sleep(0.4)  # que termine de apretar / asentarse
    return True


def correr_tarea(placa, pasos, timeout):
    for nombre, cosa in pasos:
        if nombre == "servo":
            if not servo(placa, cosa):
                placa.mandar("s")
                return False
            continue
        print(f"== {nombre}")
        if not ejecutar(placa, cosa, timeout):
            print(f"[tarea cortada en: {nombre}]")
            return False
    print("== tarea terminada")
    return True


def sesion(args):
    placa = Placa(args.puerto)
    time.sleep(2.0)  # saludo y ayuda si se reinició al abrir
    print("[punta.py: \"p x y z\", \"ver x y z\", \"ap x y z\", \"verap x y z\", \"ret\", \"donde\", \"salir\"; lo demás "
          "va a la placa (pz, mn, mv, c, s...). Primero \"h\"]")
    try:
        while placa.vivo:
            try:
                l = input().strip()
            except EOFError:
                break
            if not l:
                continue
            w = l.split()
            if w[0] == "salir":
                break
            if w[0] in ("p", "ver"):
                try:
                    x, y, z = (float(v) for v in w[1:4])
                    if len(w) != 4:
                        raise ValueError
                except ValueError:
                    print(f"Uso: {w[0]} <x> <y> <z>  (mm)")
                    continue
                s = placa.angulos()
                if s is None:
                    continue
                plan = mostrar_plan(s, x, y, z)
                if plan and w[0] == "p":
                    ejecutar(placa, plan, args.timeout)
            elif w[0] in ("ap", "verap", "ret"):
                try:
                    v = [float(t) for t in w[1:]]
                    if (w[0] != "ret" and len(v) not in (3, 4)) or (w[0] == "ret" and len(v) > 2):
                        raise ValueError
                except ValueError:
                    print("Uso: ap <x> <y> <z> [d]  |  verap <x> <y> <z> [d]  |  ret [d] [dz]  (mm)")
                    continue
                s = placa.angulos()
                if s is None:
                    continue
                if w[0] == "ret":
                    plan = imprimir_plan(*planear_salida(s, *v))
                else:
                    plan = imprimir_plan(*planear_frente(s, *v))
                if plan and w[0] != "verap":
                    ejecutar(placa, plan, args.timeout)
            elif w[0] in ("tarea", "vertarea"):
                try:
                    v = [float(t) for t in w[1:]]
                    if len(v) not in (0, 6):
                        raise ValueError
                except ValueError:
                    print("Uso: tarea [xA yA zA xB yB zB]  (mm; sin números: A = -550 100 100, B = -450 350 100)")
                    continue
                A, B = (tuple(v[:3]), tuple(v[3:])) if v else (TAREA_A, TAREA_B)
                s = placa.angulos()
                if s is None:
                    continue
                print("planeando...")
                pasos, mot = planear_tarea(s, A, B)
                if pasos is None:
                    print("NO:", "; ".join(mot))
                    continue
                imprimir_tarea(pasos)
                if w[0] == "tarea":
                    try:
                        r = input("Enter para empezar (cualquier otra cosa cancela): ")
                    except EOFError:
                        r = "no"
                    if r.strip() == "":
                        try:
                            correr_tarea(placa, pasos, args.timeout)
                        except KeyboardInterrupt:
                            placa.mandar("s")
                            print("[Ctrl+C: \"s\", las tres se sostienen donde están]")
                    else:
                        print("cancelada")
            elif w[0] == "donde":
                s = placa.angulos()
                if s is not None:
                    p, _ = k.directa(*s)
                    print(f"  punta: x {p[0]:.0f}  y {p[1]:.0f}  z {p[2]:.0f} mm  (r {math.hypot(p[0], p[1]):.0f} mm)"
                          + ("  [fuera de la zona medida]" if k.fuera_de_lo_medido(s[1], s[2]) else ""))
            else:
                placa.mandar(l)
    finally:
        placa.vivo = False
        placa.s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("punto", type=float, nargs="*", help="con --simular: x y z (mm)")
    ap.add_argument("--simular", action="store_true", help="solo el plan, sin la placa (con --desde y x y z)")
    ap.add_argument("--desde", type=float, nargs=3, default=(3.0, 10.0, 5.0), metavar=("Q1", "Q2", "Q3"),
                    help="ángulos de partida para --simular (def. el reposo, 3 10 5)")
    ap.add_argument("--puerto", default=PUERTO)
    ap.add_argument("--timeout", type=float, default=90.0, help="s máximos por mv antes de mandar \"s\" (def. 90)")
    args = ap.parse_args()
    if args.simular:
        if len(args.punto) != 3:
            ap.error("--simular necesita x y z")
        mostrar_plan(tuple(args.desde), *args.punto)
        return
    if args.punto:
        ap.error("x y z solo con --simular (con la placa, \"p x y z\" en la sesión)")
    sesion(args)


if __name__ == "__main__":
    main()
