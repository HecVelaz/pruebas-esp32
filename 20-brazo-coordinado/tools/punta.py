#!/usr/bin/env python3
"""Mover la punta (centro de la pinza) de 20 a un punto en mm, con el modelo medido de tools/cinematica.py.

Abre el puerto como un monitor: lo que se escribe va a la placa (h, c, mv, s, x, ...) salvo estos comandos propios:
    p <x> <y> <z>     llevar la punta a (x, y, z) mm: inversa, revisa el destino y el camino y manda el/los mv
    ver <x> <y> <z>   lo mismo sin mover nada (solo el plan)
    ap <x> <y> <z> [d]     llegar DE FRENTE (para agarrar): primero a d mm (def. 100) antes del punto, en la misma
                      dirección desde J1 y a la misma altura, y después entra hacia afuera, horizontal, con J1
                      quieta (un mv). verap: solo el plan. Cada mv llega desde 2° abajo (juego): la punta sube
                      hasta ~1,3 cm en el medio de la entrada y de la salida
    ret [d] [dz]      salir de frente y horizontal: retrocede d mm hacia J1 con J1 quieta (dz: subir antes, def. 0);
                      sin d, lo más que se pueda (hasta 40 cm)
    pz ... / mn ...   pinza y muñeca (van a la placa: pz a/c/0, mn a/b/0, ...)
    tarea [xA yA zA xB yB zB]   agarrar en A y dejar en B, solo (def. A = -550 100 100, B = -450 350 100):
                      planea todo antes y pide Enter; recoge, gira J1 recogido, abre la pinza, entra de frente a A,
                      cierra (pz 1200), gira la muñeca (mn a), sale de frente, igual hacia B, abre, sale, muñeca a 0, reposo
                      (mv 3 17 4). Un mv que corta por "no quedan firmes" se repite una vez.
                      vertarea: solo el plan. Ctrl+C en cualquier momento: "s"
    agarrar [s]       agarrar el limón que ve la cámara (en la Pi, con start_limon.sh): mediana de /limon/posicion
                      durante s segundos (def. 5) -> centro con la calibración del 12/11 (ajuste_camara.py) -> planea
                      todo (pz a, ap 1 cm más abajo, pz 1200, ret, reposo mv 3 17 4) y pide Enter antes de mover. Queda con el limón
                      en la pinza ("pz a" lo suelta). veragarrar [s]: mide y planea, sin mover
    donde             pide "c" y dice dónde está la punta
    anotar <punto>    como donde, y lo guarda en resultados/camara_brazo_<fecha>_brazo.csv (calibración de la
                      cámara: ver tools/ajuste_camara.py)
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
import json
import math
import queue
import re
import sys
import threading
import time

from pathlib import Path

import builtins

import numpy as np

_LOG = None


def print(*a, **kw):   # noqa: A001 - como el print de siempre, y además al log de la sesión (si hay placa)
    builtins.print(*a, **kw)
    if _LOG is not None and kw.get("file") is None:
        _LOG(" ".join(str(x) for x in a))

import ajuste_camara as ajc
import cinematica as k

PUERTO = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
# "anotar": un archivo por sesión (calibración cámara -> brazo; lo lee ajuste_camara.py)
ANOTADOS = (Path(__file__).absolute().parent.parent / "resultados"
            / f"camara_brazo_{time.strftime('%Y%m%d_%H%M%S')}_brazo.csv")
# Del firmware (include/config.h)
Q2_MENOS_Q3_MAX = 49.0     # borde de abajo de la franja en el firmware (el camino puede pasar por ahí)
MARGEN_CAMINO = 2.0        # ° (firmware 1,0). 18/11: 1,2 -> 2; a velocidad 1 J3 se pasó 1,5° en la entrada y cortó la franja
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
    r_pre = math.hypot(*pre[:2]) + 20.0
    # Si la punta ya está recogida (no más lejos que el punto previo) o bien arriba del objeto (>= 15 cm, como en
    # el reposo: punta a z 444), 1) J1 sola hasta la dirección del objeto,
    # 2) J2/J3 a la pose del punto previo (se estira hacia el objeto, sin pasar del punto previo) y 3) entrada de
    # frente. Así no se estira hacia donde apunta J1 en el reposo (a la izquierda, donde puede estar el árbol). 12/11
    arriba = k.directa(*s)[0][2] >= z + 150.0
    if (radio_punta(*s) <= r_pre or arriba) and abs(qp[0] - s[0]) > 0.05:
        g = (qp[0], s[1], s[2])
        giro = revisar_camino(s, g)
        if not giro:
            pl2, av2 = planear_q(g, qp, r_max=r_pre)
            if pl2 is not None:
                entrada, msgs = recta_j1_quieta(pl2[-1], pre, (x, y, z))
                if entrada is None:
                    return None, msgs
                return [g] + pl2 + entrada, av2
    # Si no (arranca estirada o no se puede girar así): 1) J2/J3 a la pose del punto previo con J1 donde está (recogido: la punta nunca más lejos que el punto previo,
    #    para no barrer de costado cerca del objeto); 2) J1 sola hasta la dirección del objeto; 3) entrada de frente
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
    """Salir de frente desde los ángulos s: subir dz (negativo: bajar) y retroceder d hacia J1, con J1 quieta."""
    p, _ = k.directa(*s)
    r = math.hypot(p[0], p[1])
    u = np.array([p[0] / r, p[1] / r, 0.0])
    arriba = p + [0.0, 0.0, dz]
    sube, msgs = recta_j1_quieta(tuple(s), p, arriba) if dz != 0 else ([], [])
    if sube is None:
        return None, msgs
    atras, msgs = recta_j1_quieta(sube[-1] if sube else tuple(s), arriba, arriba - d * u)
    if atras is None:
        return None, msgs
    return sube + atras, []


def planear_salida_max(s, dz=0.0, d_max=400.0, paso=10.0):
    """Como planear_salida, pero retrocede lo más que se pueda (de d_max para abajo, de a paso mm). 12/11"""
    d = d_max
    while d >= paso:
        plan, _ = planear_salida(s, d, dz)
        if plan:
            return plan, [f"retrocede {d:.0f} mm (lo máximo en horizontal con J1 quieta)"]
        d -= paso
    return None, ["no puede retroceder ni 1 cm en horizontal desde acá: probar \"mv 3 17 4\" mirando la pinza"]


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
        # Todo lo que va y viene, con la hora, en ~/logs/punta_<fecha>.log (18/11: para ver los tiempos de cada mv)
        ruta = Path.home() / "logs" / time.strftime("punta_%Y%m%d_%H%M%S.log")
        ruta.parent.mkdir(exist_ok=True)
        self.archivo = open(ruta, "a", buffering=1)
        global _LOG
        _LOG = self.log   # desde acá, todo lo que imprime punta.py también va al log (18/11)
        print(f"[log: {ruta}]")
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
            builtins.print(l)   # la línea de la placa va una sola vez al log
            self.log(l)
            self.lineas.put(l)

    def log(self, texto):
        try:
            self.archivo.write(f"{time.strftime('%H:%M:%S')}.{int(time.time() * 1000) % 1000:03d} {texto}\n")
        except ValueError:   # cerrado
            pass

    def vaciar(self):
        while not self.lineas.empty():
            self.lineas.get_nowait()

    def mandar(self, c):
        self.log(f"<< {c}")
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


FIN_MV = ("LLEGO", "PASO", "MV_ERROR", "Fuera de", "J2 y J3 fuera", "Uso:", "Hardware sin", ">> El camino recto sale",
          "J1 pasada", "Sobre la caja")
RECHAZO = ("Fuera de", "J2 y J3 fuera", "Uso:", "Hardware sin", "J1 pasada", "Sobre la caja")


def ejecutar(placa, plan, timeout, reintentos=2, de_paso=False, ultimo_de_paso=False, rodeo=False):
    """Manda los mv de a uno esperando LLEGO. Si uno corta por "no quedan firmes" (J2 sin fuerza con carga, 11/11), lo
    repite hasta reintentos veces desde donde quedó. de_paso: todos menos el último con "mvp" (sin aproximación ni
    esperar firmes, responde PASO): para puntos intermedios que no necesitan precisión (18/11)."""
    for k, q in enumerate(plan):
        cmd = "mvp" if de_paso and (k < len(plan) - 1 or ultimo_de_paso) else "mv"
        c = f"{cmd} {q[0]:.2f} {q[1]:.2f} {q[2]:.2f}"
        for intento in range(reintentos + 1):
            placa.vaciar()
            print(f"<< {c}" + (f"   (reintento {intento})" if intento else ""))
            t_mv = time.time()
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
            if l.startswith(">> El camino recto sale") and rodeo:
                # pasos chicos de "ajustar": el rodeo del firmware es de décimas de grado; seguir esperando LLEGO
                l = placa.esperar(FIN_MV[:3], timeout)
                if l and l.startswith(("LLEGO", "PASO")):
                    print(f"[mv: {time.time() - t_mv:.1f} s, con rodeo]")
                    break
                return False
            if l.startswith(">> El camino recto sale"):
                placa.mandar("s")
                print("[el firmware eligió otro camino (no revisado contra la caja): \"s\". Probar otro punto]")
                return False
            if l.startswith(("LLEGO", "PASO")):
                print(f"[mv: {time.time() - t_mv:.1f} s]")
                break
            # Cortes por atraso (carga, rama, gravedad): repetir desde donde quedó (18/11: el robot no se tiene que trabar
            # en una cosecha). Los de seguridad (franja, caja, plegado) no se repiten
            if (l.startswith("MV_ERROR") and intento < reintentos and
                    any(t in l for t in ("no quedan firmes", "no sigue la trayectoria", "se mueve al revés"))):
                time.sleep(0.5)
                continue
            if l.startswith(RECHAZO) or "sin cero" in l:
                print("[la placa rechazó el mv]")
            return False
    return True


TAREA_A = (-550.0, 100.0, 100.0)
PINZA_AGARRE_US = 1200   # = pz c (RDS3218, 18/11; con el MG996R 1250). 11/11: con 1250 una vez a J2 no le alcanzó para salir (con 1150 sí, pero agarra flojo)
TAREA_B = (-450.0, 350.0, 100.0)
REPOSO = (3.0, 17.0, 4.0)
# Soltar el limón en la canasta sobre la tapa (2026-11-18, con el joystick: Imágenes/brazo/Pos_canasta.jpeg). Pose
# plegada del firmware (config.h SOLTAR_*: J1 > 115 solo con J2 42-51 y J3 >= 78). Camino desde el reposo: J1 al
# frente (no plegar del lado del árbol), J2/J3 a una pose intermedia dentro de la franja, plegar J3 y girar J1 hasta
# la canasta; la vuelta, al revés. Modelo: sin choques; la pinza a ~14 cm del mástil con J1 en 130
# J3 80 y no 81,7 (la del joystick): con "mv" J3 se cortó en 80,94 por "sin cuentas" (cerca del punto muerto del cuatro
# barras está dura y la aproximación final va lenta); 80 sigue siendo "plegado" (>= 78)
CANASTA = (130.0, 46.7, 80.0)
# 18/11: con 4 "mv" (esperando firmes en cada uno) soltar tardaba ~30 s y la vuelta otro tanto. Ahora J1 gira al frente
# mientras suben J2/J3 (J1 100 < 115: no hace falta plegado) y los intermedios van "de paso" (mvp)
# Segunda versión: J3 ya sube a 50 mientras gira J1 (sigue dentro de la franja) y la canasta también de paso
CAMINO_CANASTA = [(100.0, 46.7, 50.0), (100.0, 46.7, 80.0), CANASTA]


def soltar(placa, timeout, volver=True):
    """Lleva el limón a la canasta, abre la pinza y (con volver) regresa al reposo. True si todo anduvo."""
    print("== soltar en la canasta")
    t0 = time.time()
    if not ejecutar(placa, CAMINO_CANASTA, timeout, de_paso=True, ultimo_de_paso=True):
        return False
    if not servo(placa, "pz a"):
        return False
    time.sleep(0.5)   # que caiga
    print(f"[soltado a los {time.time() - t0:.1f} s]")
    if volver:
        print("== volver al reposo")
        ok = ejecutar(placa, CAMINO_CANASTA[-2::-1] + [REPOSO], timeout, de_paso=True)
        print(f"[soltar: {time.time() - t0:.1f} s en total, con la vuelta]")
        return ok
    return True


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
        if nombre in ("servo", "cmd"):
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


# Tramos que van "de paso" (mvp: sin aproximación ni esperar firmes) -> (intermedios, también el último). 18/11: a
# velocidad 1, la salida con el limón cortó ("J2 se mueve al revés": bajaba rápido y la aproximación desde abajo la
# invertía con la carga). Entrar al limón sigue preciso (mv); el punto frente al limón también (de él sale la recta)
DE_PASO = {"ir frente al limón": (True, False), "salir de frente": (True, True), "reposo": (True, False),
           "tirar": (True, True), "arrastrar con J1": (True, True), "a la canasta": (True, True),
           "desplegar": (True, True)}


# Con el limón recién arrancado (tirar, arrastrar, salir) los mv van a la mitad (18/11: a velocidad 1 la salida cortaba
# por atraso); el resto a la velocidad de siempre (firmware: 1, aproximación 4 °/s, asentamiento 0,5 s)
CON_CARGA = ("entrar de frente", "tirar", "arrastrar con J1", "salir de frente")   # entrar: más preciso (18/11)
VEL_CARGA, VEL_NORMAL = "vel 0.5 4 0.5", "vel 1 4 0.5"


def correr_tarea(placa, pasos, timeout):
    t0 = time.time()
    tiempos = []
    for nombre, cosa in pasos:
        t = time.time()
        if nombre == "cmd":   # sin respuesta que esperar (pz w / mn w)
            placa.mandar(cosa)
            time.sleep(0.2)
            continue
        if nombre == "servo":
            if not servo(placa, cosa):
                placa.mandar("s")
                return False
            tiempos.append((cosa, time.time() - t))
            continue
        print(f"== {nombre}")
        con_carga = nombre in CON_CARGA
        if con_carga:
            placa.mandar(VEL_CARGA)
            time.sleep(0.2)
        de_paso, ultimo = DE_PASO.get(nombre, (False, False))
        ok = ejecutar(placa, cosa, timeout, de_paso=de_paso, ultimo_de_paso=ultimo)
        if con_carga:
            placa.mandar(VEL_NORMAL)
            time.sleep(0.2)
        if not ok:
            print(f"[tarea cortada en: {nombre}]")
            return False
        tiempos.append((f"{nombre} ({len(cosa)} mv)", time.time() - t))
    print("== tarea terminada. Tiempos:")
    for n, d in tiempos:
        print(f"   {d:5.1f} s  {n}")
    print(f"   {time.time() - t0:5.1f} s  TOTAL")
    return True


# --------------------------------------------------------------------------------------------- agarre con la visión
# Calibración cámara -> brazo del 12/11 (bitácora): radio 25 mm (limón de juguete), sin P5 (anotado en otro lugar)
RES = Path(__file__).absolute().parent.parent / "resultados"
CAL_CAMARA = RES / "camara_brazo_20261112_203642_camara.csv"
CAL_BRAZO = RES / "camara_brazo_20261112_203659_brazo.csv"
CAL_SIN = ("P5",)
DISPERSION_MAX = 10.0   # mm: más que esto, el limón se movió o la visión duda -> no agarrar
MIN_POSICIONES = 3
RADIO_LIMON = 25.0      # mm (limón de juguete de 5 cm; uno real: ~30)
AGARRAR_PINZA_US = 1200 # 18/11, RDS3218: pz c (con el MG996R 1300, un poco más que su pz c 1250)
AGARRAR_ADELANTE = -7.0  # mm más allá del centro del limón, en la dirección de entrada (18/11: 40 -> 28; 23/11, usuario:
                         # 13 y después 2 cm menos: la pinza superaba el limón; negativo = antes del centro)
# 18/11, el usuario: para centrar la pinza en los limones, 1-1,5 cm a la izquierda y 1-1,5 cm más arriba
AGARRAR_IZQUIERDA = 12.5     # mm, a la izquierda de la dirección de entrada (mirando desde J1 hacia el limón)
# Corrimiento hacia arriba, en altura real, según la altura del limón (18/11: +12,5 mm centraba los de ~46-52 cm; a
# 35 cm empujaba el limón: 1 cm más abajo). Interpolado y constante fuera del rango
AGARRAR_ARRIBA_H = (350.0, 500.0)    # mm del piso
AGARRAR_ARRIBA_MM = (2.5, 12.5)      # mm hacia arriba
AGARRAR_DZ = 0.0        # mm: 13/11 -10 (1 cm más abajo de lo que da la visión); 18/11 0: lo corrige Z_VISION_*
# Altura de la visión corregida (18/11, 3 limones medidos con cinta a 33 / 46 / 51 cm del piso: la visión daba
# 31,2 / 49,8 / 56,1 cm; error de escala, recta con < 4 mm de residuo). z (modelo) = Z_VISION_A + Z_VISION_B · z visión.
# Con piso ≈ 256 + 1,43·z (cinta, 12/11). Rehacer si se mueve una cámara o se recalibra cámara -> brazo
Z_VISION_A, Z_VISION_B = 23.1, 0.718


class Vision:
    """Mediana de /limon/posicion (rclpy, se crea al primer "agarrar")."""

    def __init__(self, nodo=None):
        """Con nodo (de otro programa, p. ej. 27-aruco-arbol/tools/recolectar.py) usa ese; si no, crea el suyo."""
        import rclpy
        from geometry_msgs.msg import PointStamped
        self.rclpy = rclpy
        self.propio = nodo is None
        if self.propio:
            rclpy.init()
            nodo = rclpy.create_node("punta_agarrar")
        self.nodo = nodo
        self.pos, self.marcos = [], set()
        self.nodo.create_subscription(PointStamped, "/limon/posicion", self._al_pos, 10)
        # 19/11: todos los limones de cada imagen (localizador_node de vision_estereo, /limon/todos)
        from geometry_msgs.msg import PoseArray
        self.cuadros, self.marcos_todos = [], set()
        self.nodo.create_subscription(PoseArray, "/limon/todos", self._al_todos, 10)

    def _al_pos(self, m):
        self.pos.append((m.point.x, m.point.y, m.point.z))
        self.marcos.add(m.header.frame_id)

    def _al_todos(self, m):
        self.cuadros.append([(q.position.x, q.position.y, q.position.z) for q in m.poses])
        self.marcos_todos.add(m.header.frame_id)

    def medir_todos(self, seg):
        """Todos los limones: [(cara mm, n, dispersión mm), ...] de más a menos visto, marcos; None si no llega
        /limon/todos (visión vieja). Agrupa las posiciones de todas las imágenes: cada una va al grupo más cercano a
        menos de AGRUPAR_MM, si no abre otro. Quedan los grupos vistos en al menos MIN_POSICIONES imágenes y en el 30 %."""
        self.pos.clear()
        self.marcos.clear()
        self.cuadros.clear()
        self.marcos_todos.clear()
        fin = time.time() + seg
        while time.time() < fin:
            self.rclpy.spin_once(self.nodo, timeout_sec=0.1)
        if not self.cuadros:
            return None, set(self.marcos)
        grupos = []   # [suma, puntos]
        for cuadro in self.cuadros:
            for pt in cuadro:
                v = np.array(pt) * 1000.0
                d = [np.linalg.norm(v - g[0] / len(g[1])) for g in grupos]
                if d and min(d) < AGRUPAR_MM:
                    g = grupos[int(np.argmin(d))]
                    g[0] = g[0] + v
                    g[1].append(v)
                else:
                    grupos.append([v.copy(), [v]])
        minimo = max(MIN_POSICIONES, 0.3 * len(self.cuadros))
        res = []
        for _, pts in grupos:
            if len(pts) < minimo:
                continue
            p = np.array(pts)
            med = np.median(p, axis=0)
            res.append((med, len(p), float(np.median(np.linalg.norm(p - med, axis=1)))))
        res.sort(key=lambda r: -r[1])
        return res, set(self.marcos_todos)

    def medir(self, seg):
        """(cara en mm, marco `camera`, n, dispersión mm, marcos) o None si no alcanzan las posiciones."""
        self.pos.clear()
        self.marcos.clear()
        fin = time.time() + seg
        while time.time() < fin:
            self.rclpy.spin_once(self.nodo, timeout_sec=0.1)
        if len(self.pos) < MIN_POSICIONES:
            return None, len(self.pos), None, self.marcos
        p = np.array(self.pos) * 1000.0
        med = np.median(p, axis=0)
        return med, len(p), float(np.median(np.linalg.norm(p - med, axis=1))), set(self.marcos)

    def cerrar(self):
        if self.propio:
            self.nodo.destroy_node()
            self.rclpy.shutdown()


AGRUPAR_MM = 35.0   # 19/11: posiciones de distintas imágenes a menos de esto = el mismo limón (limón de 5 cm)


MARGEN_SALIDA_CARGA = 3.0  # ° a la franja en la salida con el limón (13/11: con 1,6° el firmware cortó al retroceder)


def con_zona(f, *a, **kw):
    """f(*a) con la zona plegada de la franja habilitada en el modelo (solo caminos de la canasta)."""
    viejo, k.USAR_ZONA = k.USAR_ZONA, True
    try:
        return f(*a, **kw)
    finally:
        k.USAR_ZONA = viejo


def con_margen(margen, f, *a):
    """f(*a) planeando los caminos con otro MARGEN_CAMINO."""
    global MARGEN_CAMINO
    viejo, MARGEN_CAMINO = MARGEN_CAMINO, margen
    try:
        return f(*a)
    finally:
        MARGEN_CAMINO = viejo


# Desprender el limón (18/11, pedido del usuario): después de cerrar, tirar TIRON_MM hacia atrás, girar la muñeca
# (TIRON_MUNECA) y girar J1 TIRON_J1 grados hacia el lado del giro, arrastrando la rama; recién ahí salir. El signo de
# TIRON_J1 respecto del giro de la muñeca está SIN PROBAR: si arrastra hacia el otro lado, cambiarlo de signo
# 23/11 (pedido del usuario: la pinza se apoyaba en ramas al bajar justo delante del limón): bajar a la altura del
# limón lo más atrás posible y entrar horizontal desde ahí. La punta no se recoge a menos de ~44 cm de J1 (franja), así
# que la entrada máxima es (distancia del limón a J1) - 44 cm: con el limón a 53 cm sigue siendo 7 cm; más lejos, más
ENTRADA_MM = (200.0, 150.0, 100.0, 70.0, 50.0)   # entrar de frente desde acá (el primero que se pueda)
# 23/11 (pedido del usuario; antes, 19/11, SUBÍA 50 mm): después de cerrar, BAJAR el limón (estira el pedúnculo hacia
# abajo) y después traerlo hacia el robot, como siempre. Si la bajada no entra (limón bajo), prueba con menos; con 0
# solo retrocede. Se cambian en vivo con "tiron <atrás mm> <abajo mm>". El modelo da z ~1,3 veces menos que lo real
TIRON_MM = 50.0            # mm hacia el robot (antes 20)
TIRON_ABAJO_MM = 50.0      # mm hacia abajo, antes de retroceder
TIRON_MUNECA = "mn a"      # 90° horario
TIRON_J1 = 5.0             # ° (a ~60 cm de J1: ~5 cm de costado)


def planear_agarre(s, limon, a_canasta=False):
    """De los ángulos s: entrar de frente al limón, cerrar, tirar, girar la muñeca, arrastrar de costado con J1,
    salir lo más posible y al reposo. (pasos, motivos)."""
    # Entrada de frente desde 10 cm; si el punto previo no entra (limón bajo y cerca de J1, 18/11: a 33 cm del piso y
    # 53 cm de J1), desde 7 y 5 cm
    plegado = s[2] >= 78.0   # arranca plegado (DESPLEGAR, después de soltar): el desplegado pasa por la zona
    for d in ENTRADA_MM:
        pa, m = (con_zona(planear_frente, s, *limon, d=d) if plegado else planear_frente(s, *limon, d=d))
        if pa is not None:
            break
    if pa is None:
        r = math.hypot(limon[0], limon[1])
        extra = f" (r {r:.0f} mm: el ap de 10 cm necesita r >~ 500)" if r < 500 else ""
        return None, ["hacia el limón: " + "; ".join(m) + extra]
    if d != ENTRADA_MM[0]:
        m_ent = [f"entra de frente desde {d / 10:.0f} cm (desde {ENTRADA_MM[0] / 10:.0f} no llega)"]
    else:
        m_ent = []
    ti = None
    for d_ti in sorted({TIRON_MM, min(TIRON_MM, 20.0)}, reverse=True):
        for dz in sorted({TIRON_ABAJO_MM, TIRON_ABAJO_MM * 0.6, TIRON_ABAJO_MM * 0.3, 0.0}, reverse=True):
            ti, m_ti = con_margen(MARGEN_SALIDA_CARGA, planear_salida, pa[-1], d_ti, -dz)
            if ti is not None:
                break
        if ti is not None:
            break
    if ti is None:
        return None, ["tirar: " + "; ".join(m_ti)]
    if dz < TIRON_ABAJO_MM or d_ti < TIRON_MM:
        m_ent = m_ent + [f"tirón: baja {dz:.0f} mm y retrocede {d_ti:.0f} (con {TIRON_ABAJO_MM:.0f} / {TIRON_MM:.0f} no llega)"]
    q = ti[-1]
    de = (round(q[0] + TIRON_J1, 2), q[1], q[2])
    m_de = k.problemas(*de) + revisar_camino(q, de)
    if m_de:
        return None, ["arrastrar con J1: " + "; ".join(m_de)]
    sa, m_ret = con_margen(MARGEN_SALIDA_CARGA, planear_salida_max, de)
    if sa is None:
        return None, ["salida: " + "; ".join(m_ret)]
    comun = [("cmd", "pz w 0"), ("cmd", "mn w 0"),   # servos a su velocidad máxima (salto directo)
             ("servo", "pz a"), ("ir frente al limón", pa[:-1]), ("entrar de frente", pa[-1:]),
             ("servo", f"pz {AGARRAR_PINZA_US}"), ("tirar", ti), ("servo", TIRON_MUNECA),
             ("arrastrar con J1", [de]), ("salir de frente", sa), ("servo", "mn 0")]
    if a_canasta:
        # Directo a la canasta, sin subir al reposo (18/11). La canasta (J1 130) queda fuera del modelo (Q1 <= 115):
        # se revisa el camino hasta el punto de plegado; el último tramo es el de soltar(), probado
        camino = CAMINO_CANASTA
        m_c = con_zona(revisar_camino, sa[-1], camino[0]) + con_zona(revisar_camino, camino[0], camino[1])
        # El firmware (caja, J1 >= 53,5) limita J3 con J2 >= 64 y el modelo no lo sabe: 18/11 rechazó "a la canasta"
        # desde (37; 66,5; 73). Con J2 alta, también subir primero
        if sa[-1][1] >= 63.0:
            m_c = m_c or ["J2 alta (regla de la caja del firmware)"]
        if m_c:   # limón bajo: el giro de J1 barre la tapa -> primero subir J2/J3 sin girar, después girar
            sube = (sa[-1][0],) + tuple(CAMINO_CANASTA[0][1:])
            camino = [sube] + CAMINO_CANASTA
            m_c = (con_zona(revisar_camino, sa[-1], sube) + con_zona(revisar_camino, sube, camino[1]) +
                   con_zona(revisar_camino, camino[1], camino[2]))
        if m_c:
            return None, ["a la canasta: " + "; ".join(m_c)]
        return comun + [("a la canasta", camino), ("servo", "pz a")], m_ent + m_ret
    rp, m = con_margen(MARGEN_SALIDA_CARGA, planear_q, sa[-1], REPOSO)
    if rp is None:
        return None, ["al reposo: " + "; ".join(m)]
    return comun + [("reposo", rp)], m_ent + m_ret


def medir_limon(vision, tf, seg, usar_ajuste=True):
    """Mide el limón con la visión: el punto de agarre (x, y, z) en mm (corregido), o None."""
    R, t = tf
    print(f"midiendo /limon/posicion {seg:.0f} s (brazo quieto y fuera de la vista)...")
    cara, n, disp, marcos = vision.medir(seg)
    if cara is None:
        print(f"NO: solo {n} posiciones en {seg:.0f} s (¿start_limon.sh corriendo? ¿se ve el limón?)")
        return None
    if marcos != {"camera"}:
        print(f"NO: /limon/posicion viene en {marcos}, la calibración es para `camera`")
        return None
    if disp > DISPERSION_MAX:
        print(f"NO: dispersión {disp:.1f} mm (> {DISPERSION_MAX:.0f}): ¿se movió el limón? Repetir")
        return None
    return limon_de_cara(tf, cara, n, disp, usar_ajuste)


def limon_de_cara(tf, cara, n, disp, usar_ajuste=True):
    """Punto de agarre (x, y, z) en mm (corregido) del limón visto en `cara` (mm, marco de la cámara)."""
    R, t = tf
    x, y, z_vis = ajc.camara_a_cinematica(R, t, cara, RADIO_LIMON)
    z = Z_VISION_A + Z_VISION_B * float(z_vis)
    # Entrar AGARRAR_ADELANTE más allá del centro, en la dirección de entrada (desde J1): 18/11 la pinza quedaba corta
    # (la punta del modelo está adelante de donde cierran los dedos)
    r = math.hypot(float(x), float(y))
    ux, uy = float(x) / r, float(y) / r
    # + AGARRAR_ADELANTE en la dirección de entrada, + AGARRAR_IZQUIERDA a la izquierda de esa dirección (mirando
    # desde J1 hacia el limón: (-uy, ux)) y + AGARRAR_ARRIBA_REAL de altura real (el modelo: 1 mm = 1,43 mm reales)
    x = float(x) + AGARRAR_ADELANTE * ux - AGARRAR_IZQUIERDA * uy
    y = float(y) + AGARRAR_ADELANTE * uy + AGARRAR_IZQUIERDA * ux
    h_real = 256 + 1.43 * z   # mm del piso
    arriba = float(np.interp(h_real, AGARRAR_ARRIBA_H, AGARRAR_ARRIBA_MM))
    zz = float(z) + AGARRAR_DZ + arriba / 1.43
    ULTIMA["base"] = (x, y, zz)       # sin el ajuste fino (lo que guarda "ajustar")
    ULTIMA["cara"] = tuple(float(v) for v in cara)
    ULTIMA["h"] = h_real
    if usar_ajuste and AJUSTE["coef"]:
        da, di, ds = delta_ajuste(cara, h_real)
        x, y, zz = x + da * ux - di * uy, y + da * uy + di * ux, zz + ds / 1.43
        print(f"  ajuste fino ({AJUSTE['n']} puntos): {da / 10:+.1f} cm adelante, {di / 10:+.1f} izquierda, "
              f"{ds / 10:+.1f} arriba")
    limon = (round(x), round(y), round(zz))
    print(f"  cámara (cara): x {cara[0]:+.0f}  y {cara[1]:+.0f}  z {cara[2]:.0f} mm  ({n} posiciones, dispersión "
          f"{disp:.1f} mm)")
    print(f"  limón: a {r / 10:.0f} cm de J1, ~{(256 + 1.43 * z) / 10:.0f} cm del piso (z visión {float(z_vis):.0f}, "
          f"corregida {z:.0f}) -> agarre en ({limon[0]}, {limon[1]}, {limon[2]})")
    return limon


def medir_limones(vision, tf, seg):
    """19/11: todos los limones que ve la visión, como puntos de agarre [(x, y, z), ...] (los que tienen dispersión
    alta quedan afuera). Sin /limon/todos (visión vieja): solo el de mayor confianza, como antes. None si nada."""
    print(f"midiendo /limon/todos {seg:.0f} s (brazo quieto y fuera de la vista)...")
    grupos, marcos = vision.medir_todos(seg)
    if grupos is None:
        print("  [sin /limon/todos: ¿visión vieja? Uso /limon/posicion (un solo limón)]")
        if not vision.pos:
            print(f"NO: nada en {seg:.0f} s (¿start_limon.sh corriendo? ¿se ve el limón?)")
            return None
        p = np.array(vision.pos) * 1000.0
        if len(p) < MIN_POSICIONES or marcos != {"camera"}:
            print(f"NO: {len(p)} posiciones, marcos {marcos}")
            return None
        med = np.median(p, axis=0)
        disp = float(np.median(np.linalg.norm(p - med, axis=1)))
        if disp > DISPERSION_MAX:
            print(f"NO: dispersión {disp:.1f} mm (> {DISPERSION_MAX:.0f}): ¿se movió el limón?")
            return None
        return [limon_de_cara(tf, med, len(p), disp)]
    if marcos != {"camera"}:
        print(f"NO: /limon/todos viene en {marcos}, la calibración es para `camera`")
        return None
    limones = []
    for i, (cara, n, disp) in enumerate(grupos, 1):
        print(f" limón {i} de {len(grupos)}:")
        if disp > DISPERSION_MAX:
            print(f"  NO: dispersión {disp:.1f} mm (> {DISPERSION_MAX:.0f}) con {n} posiciones: ¿se movió? lo salteo")
            continue
        limones.append(limon_de_cara(tf, cara, n, disp))
    if not limones:
        print("NO: ningún limón medido bien")
        return None
    return limones


# ---------------------------------------------------------------------------------------- ajuste fino (18/11)
# "ajustar": la visión mide el limón, la pinza abierta va adonde cree que está y el usuario la centra a mano con pasos
# chicos (a+/a- adelante, i+/i- izquierda, s+/s- arriba); "ok" guarda la diferencia en la dirección de entrada (mm
# adelante, mm a la izquierda, mm reales hacia arriba) en resultados/ajuste_<fecha>.csv. "ajuste fin" ajusta, por cada
# dirección, d = c0 + c1·(altura - 0,44 m) [+ c2·(x/z de la cámara) con 6 o más puntos] y lo guarda en
# resultados/ajuste_agarre.json, que medir_limon suma desde entonces
AJUSTE_JSON = RES / "ajuste_agarre.json"
AJUSTE = {"coef": None, "n": 0}
ULTIMA = {}
PASO_AJUSTE_MM = 5.0


def cargar_ajuste():
    try:
        d = json.loads(AJUSTE_JSON.read_text())
        AJUSTE.update(coef=d["coef"], n=d["n"])
        print(f"[ajuste fino: {d['n']} puntos ({d['fecha']})]")
    except (OSError, ValueError, KeyError):
        AJUSTE.update(coef=None, n=0)


def rasgos(cara, h_real, n_coef):
    f = [1.0, h_real / 1000.0 - 0.44, cara[0] / cara[2]]
    return f[:n_coef]


def delta_ajuste(cara, h_real):
    c = AJUSTE["coef"]
    return tuple(float(np.dot(c[e], rasgos(cara, h_real, len(c[e])))) for e in ("adelante", "izquierda", "arriba"))


def ajustar(placa, vision, tf, seg, timeout):
    """Un punto del ajuste fino (ver arriba). Vuelve al reposo al terminar."""
    limon = medir_limon(vision, tf, seg, usar_ajuste=False)
    if limon is None:
        return
    base, cara, h_real = ULTIMA["base"], ULTIMA["cara"], ULTIMA["h"]
    s = placa.angulos()
    if s is None:
        return
    pa = None
    for d in ENTRADA_MM:
        pa, m = planear_frente(s, *limon, d=d)
        if pa is not None:
            break
    if pa is None:
        print("NO: hacia el limón: " + "; ".join(m) + " (moverlo o mover el robot)")
        return
    servo(placa, "pz a")
    if not ejecutar(placa, pa[:-1], timeout, de_paso=True):
        return
    placa.mandar(VEL_CARGA)   # la entrada, a la mitad (como en el agarre)
    time.sleep(0.2)
    ok = ejecutar(placa, pa[-1:], timeout)
    placa.mandar(VEL_NORMAL)
    time.sleep(0.2)
    if not ok:
        return
    r = math.hypot(base[0], base[1])
    u = np.array([base[0] / r, base[1] / r, 0.0])
    izq = np.array([-u[1], u[0], 0.0])
    meta = np.array(limon, dtype=float)
    print("Centrá el limón entre los dedos: a+ a- (adelante), i+ i- (izquierda), s+ s- (arriba) [mm, def. 5]; "
          "\"ok\" guarda, \"cancelar\" no")
    while True:
        try:
            w = input("ajustar> ").split()
        except EOFError:
            w = ["cancelar"]
        if not w:
            continue
        if w[0] in ("ok", "cancelar"):
            break
        try:
            eje, signo = w[0][0], {"+": 1.0, "-": -1.0}[w[0][1]]
            mm = float(w[1]) if len(w) > 1 else PASO_AJUSTE_MM
            if eje not in "ais" or not 0 < mm <= 30:
                raise ValueError
        except (ValueError, KeyError, IndexError):
            print("Uso: a+ / a- / i+ / i- / s+ / s- [mm]   |   ok   |   cancelar")
            continue
        paso = {"a": u * mm, "i": izq * mm, "s": np.array([0.0, 0.0, mm / 1.43])}[eje] * signo
        q, mot = k.inversa(*(meta + paso))
        if q is None or mot or k.problemas(*q):
            print("NO:", "; ".join(mot or k.problemas(*q) if q is not None else mot))
            continue
        if ejecutar(placa, [tuple(round(v, 2) for v in q)], timeout, rodeo=True):
            meta = meta + paso
    if w[0] == "ok":
        d = meta - np.array(base)
        fila = (time.strftime("%H:%M:%S"), *[round(v, 1) for v in base], *[round(v, 1) for v in meta],
                *[round(v, 1) for v in cara], round(h_real, 1), round(float(d @ u), 1), round(float(d @ izq), 1),
                round(float(d[2]) * 1.43, 1))
        ruta = RES / time.strftime("ajuste_%Y%m%d.csv")
        nuevo = not ruta.exists()
        with open(ruta, "a") as f:
            if nuevo:
                f.write("hora,bx,by,bz,mx,my,mz,cara_x,cara_y,cara_z,h_mm,adelante_mm,izquierda_mm,arriba_mm\n")
            f.write(",".join(str(v) for v in fila) + "\n")
        print(f"[guardado en {ruta.name}: {fila[-3] / 10:+.1f} cm adelante, {fila[-2] / 10:+.1f} izquierda, "
              f"{fila[-1] / 10:+.1f} arriba]")
    # salir de frente (pinza abierta) y al reposo
    sq = placa.angulos()
    sa, _ = planear_salida_max(sq) if sq else (None, None)
    if sa:
        ejecutar(placa, sa, timeout, de_paso=True, ultimo_de_paso=True)
        sq = sa[-1]
    rp, m = planear_q(sq, REPOSO) if sq else (None, ["sin ángulos"])
    if rp:
        ejecutar(placa, rp, timeout, de_paso=True)
    else:
        print("[no encuentro camino al reposo: " + "; ".join(m) + "]")


def ajuste_fin():
    """Ajusta con todos los resultados/ajuste_*.csv y guarda ajuste_agarre.json."""
    filas = []
    for ruta in sorted(RES.glob("ajuste_2*.csv")):
        for l in ruta.read_text().splitlines()[1:]:
            v = l.split(",")
            if len(v) == 14:
                filas.append([float(x) for x in v[1:]])
    n = len(filas)
    if n < 3:
        print(f"NO: hacen falta al menos 3 puntos (hay {n})")
        return
    a = np.array(filas)
    # Por cada dirección, el modelo que mejor acierta con puntos que no usó (dejando uno afuera), entre: nada,
    # constante, + altura, + lado de la imagen, completo. 18/11: con 9 puntos el completo empeoraba adelante y arriba
    F = np.array([rasgos(f[6:9], f[9], 3) for f in a])
    opciones = {"nada": [], "constante": [0], "+altura": [0, 1], "+lado": [0, 2], "completo": [0, 1, 2]}
    coef = {}
    for j, e in enumerate(("adelante", "izquierda", "arriba")):
        yv = a[:, 10 + j]
        mejor = None
        for nombre, cols in opciones.items():
            if len(cols) >= n - 1:
                continue
            loo = []
            for i in range(n):
                m = np.arange(n) != i
                if cols:
                    c, *_ = np.linalg.lstsq(F[m][:, cols], yv[m], rcond=None)
                    loo.append(F[i, cols] @ c - yv[i])
                else:
                    loo.append(-yv[i])
            err = float(np.sqrt(np.mean(np.square(loo))))
            if mejor is None or err < mejor[0] - 0.5:   # el más simple, salvo que el otro mejore > 0,5 mm
                mejor = (err, nombre, cols)
        err, nombre, cols = mejor
        c3 = [0.0, 0.0, 0.0]
        if cols:
            c, *_ = np.linalg.lstsq(F[:, cols], yv, rcond=None)
            for col, v in zip(cols, c):
                c3[col] = round(float(v), 2)
        coef[e] = c3
        print(f"  {e}: sin corregir ±{np.sqrt(np.mean(yv ** 2)) / 10:.1f} cm -> {nombre}: ±{err / 10:.1f} cm "
              f"(con limones nuevos)")
    AJUSTE_JSON.write_text(json.dumps({"n": n, "fecha": time.strftime("%Y-%m-%d %H:%M"), "coef": coef}, indent=1))
    AJUSTE.update(coef=coef, n=n)
    print(f"[ajuste fino guardado ({n} puntos): {AJUSTE_JSON.name}]")


def planear_agarre_tol(s, limon, a_canasta=False):
    """planear_agarre; si no llega, entrando 1 y 2 cm menos hondo. 18/11 (23/11: fijos; AGARRAR_ADELANTE puede ser < 0)"""
    pasos, mot = planear_agarre(s, limon, a_canasta)
    if pasos is not None:
        return pasos, mot
    r = math.hypot(limon[0], limon[1])
    for atras in (10.0, 20.0):
        f = (r - atras) / r
        p2, m2 = planear_agarre(s, (round(limon[0] * f), round(limon[1] * f), limon[2]), a_canasta)
        if p2 is not None:
            return p2, [f"entra {atras / 10:.1f} cm menos (con todo no llega)"] + m2
    return None, mot


def agarrar(placa, vision, tf, seg, mover, timeout, confirmar=True, a_canasta=False):
    """Mide el limón, planea y (con mover) agarra. a_canasta: después de salir va directo a la canasta y suelta (sin
    pasar por el reposo). confirmar: pide Enter antes de mover. True si agarró."""
    limon = medir_limon(vision, tf, seg)
    if limon is None:
        return False
    s = placa.angulos() if placa else (3.0, 17.0, 4.0)
    if s is None:
        return False
    print("planeando...")
    pasos, mot = planear_agarre_tol(s, limon, a_canasta)
    if pasos is None:
        print("NO:", "; ".join(mot))
        return False
    for m in mot:
        print("  ojo:", m)
    imprimir_tarea(pasos)
    if not mover:
        return False
    if confirmar:
        try:
            r = input("Enter para agarrar (cualquier otra cosa cancela): ")
        except EOFError:
            r = "no"
        if r.strip() != "":
            print("cancelado")
            return False
    try:
        if correr_tarea(placa, pasos, timeout):
            print("[limón en la canasta]" if a_canasta else "[limón en la pinza: \"pz a\" para soltarlo]")
            return True
    except KeyboardInterrupt:
        placa.mandar("s")
        print("[Ctrl+C: \"s\", las tres se sostienen donde están]")
    return False


def recuperar_a_canasta(placa, timeout):
    """Después de un corte en medio del agarre: lleva lo que tenga la pinza a la canasta (sube J2/J3 sin girar, gira
    hasta plegar y suelta), si el camino entra en el modelo. True si quedó en la canasta."""
    s = placa.angulos()
    if s is None:
        return False
    if s[0] > k.Q1_LIM[1]:    # ya en la zona de la canasta
        camino = [CANASTA]
    else:
        sube = (s[0],) + tuple(CAMINO_CANASTA[0][1:])
        camino = [sube] + CAMINO_CANASTA
        m = (con_zona(revisar_camino, s, sube) + con_zona(revisar_camino, sube, camino[1]) +
             con_zona(revisar_camino, camino[1], camino[2]))
        if m:
            print("[recuperar: " + "; ".join(m) + "]")
            return False
    print("== recuperar: a la canasta")
    if not ejecutar(placa, camino, timeout, de_paso=True, ultimo_de_paso=True):
        return False
    servo(placa, "pz a")
    servo(placa, "mn 0")
    return True


# Cosechar varios (18/11): medir uno, agarrarlo y llevarlo a la canasta, y medir el siguiente desde ahí (en la canasta
# el brazo está fuera de la vista de la cámara): el árbol se mueve al arrancar cada limón, así que se mide de nuevo.
# Desde la canasta (J1 130, fuera del modelo) primero vuelve de paso a DESPLEGAR (J1 100, todavía plegado)
DESPLEGAR = (100.0, 46.7, 80.0)
VERIFICAR_MM = 60.0   # si al medir de nuevo hay un limón a menos de esto del anterior, no se agarró
MAX_INTENTOS = 2      # por limón
SOLTAR_Q2_MIN_PY = 42.0


def cosechar(placa, vision, tf, n, seg, timeout, confirmar=True):
    """Hasta n limones a la canasta; al final, al reposo. Devuelve cuántos dejó en la canasta (confirmados con la
    visión: si al medir de nuevo el limón anterior sigue en el árbol, no se contó y se reintenta).
    19/11: mide TODOS los limones y agarra el más cercano a J1 que tenga plan; los que no tienen plan o fallaron
    MAX_INTENTOS veces quedan descartados (por posición) y sigue con los demás."""
    if confirmar:
        try:
            r = input(f"Enter para cosechar hasta {n} limones (cualquier otra cosa cancela): ")
        except EOFError:
            r = "no"
        if r.strip() != "":
            print("cancelado")
            return 0
    t0, hechos, pendiente, intentos = time.time(), 0, None, 0
    descartados = []   # puntos de agarre de limones que no se intentan más

    def descartado(l):
        return any(math.dist(l, d) < VERIFICAR_MM for d in descartados)

    try:
        while True:
            print(f"===== cosechados {hechos} de {n}" + (f" (reintento {intentos})" if intentos else ""))
            limones = medir_limones(vision, tf, seg) or []
            if pendiente is not None:   # ¿el anterior quedó en el árbol?
                sigue = [l for l in limones if math.dist(l, pendiente) < VERIFICAR_MM]
                if sigue:
                    intentos += 1
                    print(f"[el limón anterior sigue en el árbol (a {math.dist(sigue[0], pendiente) / 10:.1f} cm): no "
                          f"lo agarró]")
                    if intentos >= MAX_INTENTOS:
                        print(f"[{MAX_INTENTOS} intentos sin agarrarlo: lo dejo y sigo con los demás]")
                        descartados.append(sigue[0])
                        intentos = 0
                else:
                    hechos += 1
                    intentos = 0
                    print(f"[confirmado: {hechos} en la canasta, {time.time() - t0:.0f} s desde el comienzo]")
                pendiente = None
            if hechos >= n:
                break
            candidatos = sorted((l for l in limones if not descartado(l)), key=lambda l: math.hypot(l[0], l[1]))
            if not candidatos:
                print("[no quedan limones para intentar]" if limones else "[no veo limones]")
                break
            s = placa.angulos()
            if s is None:
                break
            pre = []
            if s[0] > k.Q1_LIM[1]:      # en la canasta: primero de paso a DESPLEGAR
                pre, s = [("desplegar", [DESPLEGAR])], DESPLEGAR
            pasos = None
            for i, limon in enumerate(candidatos, 1):
                print(f"planeando el limón en ({limon[0]}, {limon[1]}, {limon[2]}) ({i} de {len(candidatos)})...")
                pasos, mot = planear_agarre_tol(s, limon, a_canasta=True)
                if pasos is not None:
                    break
                print("  NO:", "; ".join(mot), "-> lo descarto")
                descartados.append(limon)
            if pasos is None:
                print("[ningún limón tiene plan desde acá]")
                break
            for m in mot:
                print("  ojo:", m)
            if not correr_tarea(placa, pre + pasos, timeout):
                if not recuperar_a_canasta(placa, timeout):
                    print("[no se pudo recuperar: el brazo quedó donde está]")
                    return hechos
                print("[recuperado: lo que tenía la pinza está en la canasta]")
            pendiente = limon   # se confirma al medir de nuevo (con el brazo en la canasta, fuera de la vista)
        print("== al reposo")
        s = placa.angulos()
        camino = ([DESPLEGAR, (100.0, 46.7, 50.0)] if s and s[1] >= SOLTAR_Q2_MIN_PY and s[2] >= 78.0 else []) + [REPOSO]
        ejecutar(placa, camino, timeout, de_paso=True)
    except KeyboardInterrupt:
        placa.mandar("s")
        print("[Ctrl+C: \"s\", las tres se sostienen donde están]")
    print(f"[cosecha: {hechos} limones en {time.time() - t0:.0f} s"
          + (f"; {len(descartados)} descartados" if descartados else "") + "]")
    return hechos


def cargar_tf_agarre():
    R, t, nombres, loo = ajc.cargar_tf(CAL_CAMARA, CAL_BRAZO, RADIO_LIMON, CAL_SIN)
    print(f"[calibración cámara -> brazo: {len(nombres)} puntos, radio {RADIO_LIMON:.0f} mm, error esperado "
          f"~{loo:.0f} mm (dejando cada punto afuera)]")
    return R, t


def sesion(args):
    placa = Placa(args.puerto)
    vision, tf = None, None
    time.sleep(2.0)  # saludo y ayuda si se reinició al abrir
    print("[punta.py: \"p x y z\", \"ver x y z\", \"ap x y z\", \"verap x y z\", \"ret\", \"soltar [quedar]\", \"agarrar\", \"veragarrar\", \"cosechar [n]\", \"ajustar\", \"ajuste fin|borrar\", \"tiron [atrás arriba]\", \"donde\", \"anotar\", \"salir\"; lo demás "
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
            elif w[0] == "soltar":
                soltar(placa, args.timeout, volver=not (len(w) > 1 and w[1] == "quedar"))
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
                    plan = imprimir_plan(*(planear_salida(s, *v) if v else planear_salida_max(s)))
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
            elif w[0] == "tiron":   # cuánto baja (23/11; antes subía) y retrocede con el limón antes de la muñeca
                global TIRON_MM, TIRON_ABAJO_MM
                try:
                    if len(w) == 3:
                        atras, abajo = float(w[1]), float(w[2])
                        if not (0 <= atras <= 150 and 0 <= abajo <= 150):
                            raise ValueError
                        TIRON_MM, TIRON_ABAJO_MM = atras, abajo
                    elif len(w) != 1:
                        raise ValueError
                except ValueError:
                    print("Uso: tiron [<atrás mm> <abajo mm>]  (0 a 150 cada uno)")
                    continue
                print(f"[tirón: baja {TIRON_ABAJO_MM:.0f} mm, después {TIRON_MM:.0f} mm hacia el robot, "
                      f"después la muñeca ({TIRON_MUNECA})]")
            elif w[0] == "ajuste" and len(w) == 2 and w[1] in ("fin", "borrar"):
                if w[1] == "fin":
                    ajuste_fin()
                else:
                    AJUSTE_JSON.unlink(missing_ok=True)
                    AJUSTE.update(coef=None, n=0)
                    print("[ajuste fino borrado: se usan solo las correcciones fijas]")
            elif w[0] in ("agarrar", "veragarrar", "cosechar", "ajustar"):
                try:
                    if w[0] == "cosechar":
                        n_lim = int(w[1]) if len(w) > 1 else 3
                        seg = 5.0
                        if len(w) > 2 or not 1 <= n_lim <= 20:
                            raise ValueError
                    else:
                        seg = float(w[1]) if len(w) > 1 else 5.0
                        if len(w) > 2 or seg <= 0:
                            raise ValueError
                except ValueError:
                    print("Uso: agarrar [segundos]  |  veragarrar [segundos]  |  cosechar [n limones, def. 3]")
                    continue
                try:
                    if tf is None:
                        tf = cargar_tf_agarre()
                        cargar_ajuste()
                    if vision is None:
                        vision = Vision()
                except (ImportError, SystemExit, OSError) as e:
                    print(f"[sin visión: {e} (¿source /opt/ros/jazzy/setup.bash? ¿en la Pi?)]")
                    continue
                if w[0] == "ajustar":
                    ajustar(placa, vision, tf, seg, args.timeout)
                elif w[0] == "cosechar":
                    cosechar(placa, vision, tf, n_lim, seg, args.timeout)
                else:   # 18/11: agarra y va directo a la canasta (sin subir al reposo)
                    agarrar(placa, vision, tf, seg, w[0] == "agarrar", args.timeout, a_canasta=True)
            elif w[0] == "anotar":
                # Calibración cámara -> brazo: guarda el punto (la punta en el limón) en resultados/
                if len(w) != 2:
                    print("Uso: anotar <punto>  (la misma etiqueta que en medir_limon.py)")
                    continue
                s = placa.angulos()
                if s is None:
                    continue
                p, _ = k.directa(*s)
                nuevo = not ANOTADOS.exists()
                with open(ANOTADOS, "a") as f:
                    if nuevo:
                        f.write("punto,q1,q2,q3,x_mm,y_mm,z_mm,fuera_de_lo_medido\n")
                    f.write(f"{w[1]},{s[0]:.2f},{s[1]:.2f},{s[2]:.2f},{p[0]:.1f},{p[1]:.1f},{p[2]:.1f},"
                            f"{int(k.fuera_de_lo_medido(s[1], s[2]))}\n")
                print(f"  {w[1]}: q {s[0]:.2f} {s[1]:.2f} {s[2]:.2f} -> punta x {p[0]:.0f} y {p[1]:.0f} z {p[2]:.0f} mm"
                      f"  (en {ANOTADOS.name})")
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
        if vision is not None:
            vision.cerrar()


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
