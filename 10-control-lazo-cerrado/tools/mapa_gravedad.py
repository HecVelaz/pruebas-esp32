#!/usr/bin/env python3
"""Mapa de gravedad de J2 y J3 (19-j2-j3-conjunto, fase 3): cuánto duty sostiene cada una según q2 y q3.

En el cuatro barras J2 y J3 se cargan mutuamente: el duty que sostiene cada una depende de las dos. Este script
lleva el brazo a una lista de poses dentro de la franja (q2 - 25° <= q3 <= q2 + 7,5°), en cada una manda "an"
(el firmware espera que las dos se asienten y promedia el duty 1 s) y guarda todo en un CSV. Cada pose se mide
dos veces: llegando desde abajo y desde arriba (las dos articulaciones del mismo lado). Con la fricción, quieta
el duty queda en cualquier lugar de una franja: el promedio de los dos lados es la gravedad y la mitad de la
diferencia, la fricción. Después ajusta, para cada una,
    u = c + a2c·cos q2 + a2s·sin q2 + a3c·cos q3 + a3s·sin q3   (= c + a·cos(q2 + φ) + b·cos(q3 + ψ))
y lo compara con modelos más simples. Guarda el CSV y el PNG en resultados/ e imprime los coeficientes para
config.h.

Antes: J2 y J3 en sus finales de carrera, monitor cerrado, fuente con 3 A, 12 V encendidos, y correrlo con --cero:
abrir el puerto serie puede reiniciar la WROOM (borra el cero y suelta el brazo), así que el script espera que
arranque, hace "z" (y corta si algún final de carrera está suelto) y apaga "fb". Activa J2 (-e j2): mide las dos.

Uso:
    python3 tools/mapa_gravedad.py capturar --cero           # las 14 poses, desde abajo y desde arriba (~10 min)
    python3 tools/mapa_gravedad.py capturar --cero --paso    # espera Enter antes de cada pose
    python3 tools/mapa_gravedad.py ajustar resultados/mapa_gravedad_....csv
    python3 tools/mapa_gravedad.py plan                      # muestra los comandos, sin tocar la placa
Ctrl+C manda Enter (corta lo que esté haciendo la activa; las dos siguen sosteniéndose) y guarda lo medido.
"""
import argparse
import csv
import math
import re
import sys
import time
from datetime import datetime
from pathlib import Path

# absolute() y no resolve(): llamado desde el enlace tools/ de 19, guarda en la carpeta de esa prueba
DIR_RESULTADOS = Path(__file__).absolute().parent.parent / "resultados"

# Franja del cuatro barras (19-j2-j3-conjunto/include/config.h): BANDA_MIN <= q3 - q2 <= BANDA_MAX
BANDA_MIN, BANDA_MAX = -25.0, 7.5
LIMITES = {"j2": (0.0, 97.0), "j3": (0.0, 113.0)}
MARGEN_ACTIVA = 3.0  # "a" deja 3° hasta el límite (propio o de la franja)
HOLGURA = 0.7        # además, por lo que la otra puede estar corrida de lo pedido (TOL_SALIDA 0,6°)

# Poses (q2, q3): 3° adentro de la franja, q3 - q2 entre -20 y +3
# Al terminar, el brazo vuelve abajo: abrir el monitor reinicia la placa y lo suelta (estirado, caería)
POSE_FINAL = {"j2": 5.0, "j3": 0.0}

POSES = [(10, 3), (10, 13),
         (30, 10), (30, 22), (30, 33),
         (50, 30), (50, 42), (50, 53),
         (70, 50), (70, 62), (70, 73),
         (90, 70), (90, 82), (90, 93)]

# El mismo puerto que tools/monitor.sh: no cambia si el USB se reconecta (ttyUSB0 -> ttyUSB1)
PUERTO = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

CAMPOS_MAPA = ["t_ms", "q2", "q3", "duty2", "std2", "duty3", "std3", "uhold2", "uhold3", "err2", "err3",
               "lado2", "lado3", "estable", "nota"]
CAMPOS_CSV = ["q2_obj", "q3_obj", "lado_pedido"] + CAMPOS_MAPA
RECHAZOS = ("Primero", "Uso", "Fuera de rango", ">> Cancelado", ">> RECHAZADO", ">> CORTADO", "Comando desconocido",
            "Hardware sin")


# ---------- Planificación: movimientos de a una articulación, siempre dentro de la franja ----------

def intervalo(art, otro, activa):
    """Dónde puede ir la articulación art ("j2"/"j3") con la otra en otro, según las reglas del firmware."""
    lo, hi = LIMITES[art]
    m = MARGEN_ACTIVA + HOLGURA if art == activa else HOLGURA
    if art == activa:
        lo, hi = lo + MARGEN_ACTIVA, hi - MARGEN_ACTIVA
    if art == "j2":  # q2 entre q3 - BANDA_MAX y q3 - BANDA_MIN
        return max(lo, otro - BANDA_MAX + m), min(hi, otro - BANDA_MIN - m)
    return max(lo, otro + BANDA_MIN + m), min(hi, otro + BANDA_MAX - m)  # q3 entre q2 + MIN y q2 + MAX


def planear(desde, hasta, activa):
    """Lista de (articulación, ángulo) para ir de desde a hasta ({"j2": q2, "j3": q3}) moviendo de a una, cada
    una siempre hacia su objetivo (así la llegada es del lado pedido). Escalonado si hace falta."""
    pos, pasos = dict(desde), []
    otra = {"j2": "j3", "j3": "j2"}
    for _ in range(60):
        if all(abs(pos[a] - hasta[a]) < 0.01 for a in pos):
            return pasos
        # Primero, la que puede llegar de una vez (la activa antes, si las dos pueden)
        for a in (activa, otra[activa]):
            lo, hi = intervalo(a, pos[otra[a]], activa)
            if abs(pos[a] - hasta[a]) >= 0.01 and lo <= hasta[a] <= hi:
                pos[a] = hasta[a]
                pasos.append((a, hasta[a]))
                break
        else:
            # Ninguna llega: avanza la que más pueda hacia su objetivo
            mejor = None
            for a in (activa, otra[activa]):
                lo, hi = intervalo(a, pos[otra[a]], activa)
                x = min(max(hasta[a], lo), hi)
                avance = (x - pos[a]) * (1 if hasta[a] > pos[a] else -1)
                if avance > 0.5 and (mejor is None or avance > mejor[2]):
                    mejor = (a, round(x, 1), avance)
            if mejor is None:
                raise RuntimeError(f"no encuentro camino de {desde} a {hasta} (trabado en {pos})")
            pos[mejor[0]] = mejor[1]
            pasos.append((mejor[0], mejor[1]))
    raise RuntimeError(f"demasiados pasos de {desde} a {hasta}")


def secuencia(inicio, poses, activa, aprox):
    """Para cada pose y lado (-1 desde abajo, +1 desde arriba): pasos hasta la aproximación y hasta la pose."""
    pos, todo = dict(inicio), []
    for q2, q3 in poses:
        for lado in (-1, +1):
            pre = {"j2": q2 + lado * aprox, "j3": q3 + lado * aprox}
            for a in pre:  # dentro de los límites propios (con el margen de la activa)
                lo, hi = LIMITES[a]
                if a == activa:
                    lo, hi = lo + MARGEN_ACTIVA, hi - MARGEN_ACTIVA
                pre[a] = min(max(pre[a], lo), hi)
            obj = {"j2": float(q2), "j3": float(q3)}
            pasos = planear(pos, pre, activa) + planear(pre, obj, activa)
            todo.append(((q2, q3), lado, pasos))
            pos = obj
    return todo


def comando(art, x, activa):
    return f"{'a' if art == activa else 'o'} {x:.2f}"


# ---------- Placa ----------

def esperar(s, fin, timeout, eco=True):
    """Lee líneas hasta que una empiece con algo de fin (devuelve esa línea) o con un rechazo (excepción)."""
    t_lim = time.time() + timeout
    while time.time() < t_lim:
        l = s.readline().decode(errors="replace").strip()
        if not l:
            continue
        if eco and not l.startswith("MAPA"):
            print("   " + l)
        if isinstance(fin, re.Pattern) and fin.match(l) or not isinstance(fin, re.Pattern) and l.startswith(fin):
            return l
        if l.startswith(RECHAZOS) or "saca el cuatro barras" in l or "ATASCO" in l:
            raise RuntimeError(f"la placa respondió: {l}")
    raise RuntimeError(f"sin respuesta en {timeout} s (esperaba {fin})")


def mandar(s, txt):
    print(f"-> {txt}")
    s.write((txt + "\n").encode())


RE_O_OK = re.compile(r"^>> J\d hacia ")
RE_OC = re.compile(r"^J\d \(se sostiene sola\): .*\| (EN CAMINO|LLEGANDO|LLEGÓ|SIN SOSTENER)")


def esperar_otra(s, timeout=40):
    """Pregunta "oc" cada medio segundo hasta que la otra diga LLEGÓ (así cada tramo termina antes del próximo y
    la llegada es del lado pedido)."""
    t_lim = time.time() + timeout
    while time.time() < t_lim:
        s.write(b"oc\n")
        t_oc = time.time() + 1.0
        while time.time() < t_oc:
            l = s.readline().decode(errors="replace").strip()
            if "ATASCO" in l:
                raise RuntimeError(f"la placa respondió: {l}")
            m = RE_OC.match(l)
            if m:
                if m.group(1) == "LLEGÓ":
                    return
                if m.group(1) == "SIN SOSTENER":
                    raise RuntimeError("la otra dejó de sostenerse")
                break
        time.sleep(0.5)
    raise RuntimeError(f"la otra no llegó en {timeout} s")


def mover(s, art, x, activa):
    mandar(s, comando(art, x, activa))
    if art == activa:
        esperar(s, (">> Llegó",), 60)  # incluye la espera a la otra (hasta 30 s) y el movimiento (hasta 15 s)
    else:
        esperar(s, RE_O_OK, 5)  # ">> J3 hacia ..."
        esperar_otra(s)


def drenar(s, quieto=2.5, maximo=10.0):
    """Lee hasta que la placa deje de mandar. Si abrir el puerto la reinició, setup() se queda 1,5 s callado
    (delay), imprime la ayuda y al final tira lo que haya llegado: lo mandado antes se pierde. Por eso, 2,5 s de
    silencio."""
    t_fin, t_ult = time.time() + maximo, time.time()
    while time.time() < t_fin and time.time() - t_ult < quieto:
        l = s.readline().decode(errors="replace").strip()
        if l:
            t_ult = time.time()
            if l.startswith("=="):
                print("   (la placa arrancó de nuevo: el cero se perdió)")


def preparar(s, args):
    """Con --cero: "z" (los dos finales de carrera tienen que estar apretados). Siempre: "fb" apagado."""
    if args.cero:
        for intento in (1, 2):
            mandar(s, "z")
            try:
                l = esperar(s, ("Final de carrera",), 3)  # esperar() le saca los espacios del principio
                break
            except RuntimeError:
                if intento == 2:
                    raise
                print("   Sin respuesta a z (¿estaba arrancando?): espero y lo repito.")
                drenar(s)
        if "SUELTO" in l:
            raise RuntimeError("al hacer el cero algún final de carrera está SUELTO: llevarlos ahí (joystick) y repetir")
    s.reset_input_buffer()
    mandar(s, "g")
    l = esperar(s, ("Las dos al sostener",), 3, eco=False)
    if "(fb) PRENDIDO" in l:
        mandar(s, "fb")
        esperar(s, (">> Freno dentro de la banda",), 3)


def leer_posicion(s, activa):
    """Manda "c" y devuelve {"j2": q2, "j3": q3} y si la otra está sosteniéndose."""
    s.reset_input_buffer()
    mandar(s, "c")
    qa = qo = None
    sostiene = False
    t_lim = time.time() + 3
    while time.time() < t_lim and (qa is None or qo is None):
        l = s.readline().decode(errors="replace").strip()
        m = re.match(r"^cuentas [+-]?\d+ \| \S+ ([+-][\d.]+)°", l)
        if m:
            qa = float(m.group(1))
        m = re.match(r"^J\d \(se sostiene sola\): ([+-][\d.]+)°.*\| (\S.*)$", l)
        if m:
            qo = float(m.group(1))
            sostiene = "SIN SOSTENER" not in l
    if qa is None or qo is None:
        raise RuntimeError('no pude leer la posición con "c" (¿firmware de 19? ¿monitor cerrado?)')
    otra = "j3" if activa == "j2" else "j2"
    return {activa: qa, otra: qo}, sostiene


def capturar(args):
    import serial
    DIR_RESULTADOS.mkdir(exist_ok=True)
    ruta = DIR_RESULTADOS / f"mapa_gravedad_{datetime.now():%Y%m%d_%H%M%S}.csv"
    filas, cortado = [], True
    # Sin tocar DTR/RTS al abrir (como tools/monitor.sh): en la DevKit pueden reiniciar la WROOM, y un reinicio borra
    # el cero y suelta el brazo
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = args.puerto, 115200, 0.2
    s.dtr = s.rts = False
    try:
        s.open()
    except serial.SerialException as e:
        print(f"No se pudo abrir {args.puerto}: {e}\n¿Está conectada la placa? ¿El monitor está cerrado? (ls /dev/ttyUSB*)")
        sys.exit(1)
    try:
        drenar(s)
        preparar(s, args)
        inicio, sostiene = leer_posicion(s, args.activa)
        print(f"Inicio: q2 {inicio['j2']:+.2f}°, q3 {inicio['j3']:+.2f}°"
              f"{'' if sostiene else ' (la otra sin sostener: el primer \"o\" la toma)'}")
        sec = secuencia(inicio, POSES, args.activa, args.aprox)
        print(f"{len(sec)} mediciones ({len(POSES)} poses x 2 lados). Ctrl+C corta y guarda lo medido.")
        for n, ((q2, q3), lado, pasos) in enumerate(sec, 1):
            txt_lado = "desde abajo" if lado < 0 else "desde arriba"
            print(f"\n[{n}/{len(sec)}] pose q2 {q2}°, q3 {q3}° {txt_lado}")
            if args.paso:
                input("   Enter para seguir (Ctrl+C para cortar)... ")
            for art, x in pasos:
                mover(s, art, x, args.activa)
            mapa = None
            for intento in (1, 2):
                mandar(s, f"an p{q2}_{q3}_{'ab' if lado < 0 else 'ar'}")
                l = esperar(s, ("MAPA,", "MAPA_ERROR"), 45)
                if l.startswith("MAPA,"):
                    mapa = dict(zip(CAMPOS_MAPA, l.split(",", len(CAMPOS_MAPA))[1:]))
                    break
                print(f"   {l} (intento {intento})")
            if mapa is None:
                print("   Sin medición en esta pose: sigo con la próxima.")
                continue
            filas.append({"q2_obj": q2, "q3_obj": q3, "lado_pedido": lado, **mapa})
            print(f"   J2 {float(mapa['q2']):+.2f}° duty {float(mapa['duty2']):+.1f} %, "
                  f"J3 {float(mapa['q3']):+.2f}° duty {float(mapa['duty3']):+.1f} %")
            guardar(ruta, filas)  # en cada pose: si algo se corta, queda lo medido
        if args.volver:
            pos = {"j2": float(sec[-1][0][0]), "j3": float(sec[-1][0][1])}
            print(f"\nVuelvo abajo (q2 {POSE_FINAL['j2']:g}°, q3 {POSE_FINAL['j3']:g}°)...")
            for art, x in planear(pos, POSE_FINAL, args.activa):
                mover(s, art, x, args.activa)
        cortado = False
    except KeyboardInterrupt:
        print("\nCortado con Ctrl+C.")
    except Exception as e:  # noqa: BLE001
        print(f"\nCortado: {e}")
    finally:
        if cortado:
            try:
                s.write(b"\n")
                print("Se mandó Enter (corta el movimiento de la activa; las dos siguen sosteniéndose).")
            except Exception as e:  # noqa: BLE001
                print(f"No se pudo mandar Enter ({e}): con la mano debajo del brazo, apagar los 12 V.")
        try:
            s.close()
        except Exception:  # noqa: BLE001
            pass
    if not filas:
        print("No hay mediciones.")
        sys.exit(1)
    if cortado:
        nueva = ruta.with_name(ruta.stem + "_cortado.csv")
        ruta.rename(nueva)
        ruta = nueva
    print(f"\nCSV: {ruta.relative_to(DIR_RESULTADOS.parent)} ({len(filas)} mediciones)")
    return ruta


def guardar(ruta, filas):
    with open(ruta, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CAMPOS_CSV)
        w.writeheader()
        w.writerows(filas)


# ---------- Ajuste ----------

MODELOS = {
    "constante": lambda q2, q3: [1.0],
    "solo q propio": None,  # se arma según la articulación
    "plano": lambda q2, q3: [1.0, q2, q3],
    "cos q2 + cos q3": lambda q2, q3: [1.0, math.cos(math.radians(q2)), math.sin(math.radians(q2)),
                                        math.cos(math.radians(q3)), math.sin(math.radians(q3))],
}


def ajustar_modelo(np, X, y):
    """Mínimos cuadrados: coeficientes, RMS y RMS de validación cruzada (dejando uno afuera)."""
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ coef
    rms = float(np.sqrt(np.mean(r ** 2)))
    loo = float("nan")
    if len(y) > X.shape[1]:
        H = X @ np.linalg.pinv(X.T @ X) @ X.T
        h = np.clip(np.diag(H), 0, 0.999)
        loo = float(np.sqrt(np.mean((r / (1 - h)) ** 2)))
    return coef, rms, loo


def ajustar(args):
    import numpy as np
    filas = [f for f in csv.DictReader(open(args.archivo)) if f.get("duty2")]
    if not filas:
        print("El CSV no tiene mediciones.")
        sys.exit(1)
    inestables = [f for f in filas if f["estable"] != "1"]
    print(f"{len(filas)} mediciones; {len(inestables)} se movieron durante el promedio y quedan afuera"
          + (" (--incluir-inestables para usarlas)" if inestables and not args.incluir_inestables else ""))
    if not args.incluir_inestables:
        filas = [f for f in filas if f["estable"] == "1"]
    # Por pose: promedio de los dos lados (gravedad) y semidiferencia (fricción). Con un solo lado, el duty tiene
    # la fricción adentro: queda afuera del ajuste (salvo --incluir-un-lado)
    poses = {}
    for f in filas:
        poses.setdefault((float(f["q2_obj"]), float(f["q3_obj"])), {})[int(f["lado_pedido"])] = f
    un_lado = [p for p, lados in poses.items() if len(lados) < 2]
    if un_lado:
        print(f"Poses con un solo lado: {', '.join(f'({a:g}, {b:g})' for a, b in un_lado)}"
              + ("" if args.incluir_un_lado else " -> afuera (--incluir-un-lado para usarlas)"))
        if not args.incluir_un_lado:
            poses = {p: l for p, l in poses.items() if len(l) == 2}
            filas = [f for f in filas if (float(f["q2_obj"]), float(f["q3_obj"])) in poses]
    if not poses:
        print("No quedan poses para ajustar.")
        sys.exit(1)
    resultado = {}
    for j in ("2", "3"):
        q2s, q3s, us, fr = [], [], [], []
        for (_, _), lados in sorted(poses.items()):
            fs = list(lados.values())
            q2s.append(np.mean([float(f["q2"]) for f in fs]))
            q3s.append(np.mean([float(f["q3"]) for f in fs]))
            us.append(np.mean([float(f["duty" + j]) for f in fs]))
            if -1 in lados and 1 in lados:
                fr.append((float(lados[-1]["duty" + j]) - float(lados[1]["duty" + j])) / 2)
        q2s, q3s, us = np.array(q2s), np.array(q3s), np.array(us)
        print(f"\n== J{j}: {len(us)} poses; duty de {us.min():+.1f} a {us.max():+.1f} %")
        if fr:
            print(f"   Fricción (mitad de la diferencia abajo - arriba): media {np.mean(fr):+.1f} %, "
                  f"de {min(fr):+.1f} a {max(fr):+.1f} % ({len(fr)} poses con los dos lados)")
        propio = (lambda q2, q3: [1.0, math.cos(math.radians(q2)), math.sin(math.radians(q2))]) if j == "2" else \
                 (lambda q2, q3: [1.0, math.cos(math.radians(q3)), math.sin(math.radians(q3))])
        mejor = None
        for nombre, fx in MODELOS.items():
            fx = fx or propio
            X = np.array([fx(a, b) for a, b in zip(q2s, q3s)])
            if len(us) < X.shape[1]:
                print(f"   {nombre:16s} faltan poses")
                continue
            coef, rms, loo = ajustar_modelo(np, X, us)
            print(f"   {nombre:16s} RMS {rms:5.2f} %   validación cruzada {loo:5.2f} %")
            if nombre == "cos q2 + cos q3":
                mejor = (coef, rms, loo, fx)
        resultado[j] = (q2s, q3s, us, mejor)
        if mejor:
            c, a2c, a2s, a3c, a3s = mejor[0]
            a, fi = math.hypot(a2c, a2s), math.degrees(math.atan2(-a2s, a2c))
            b, psi = math.hypot(a3c, a3s), math.degrees(math.atan2(-a3s, a3c))
            print(f"   u = {c:+.2f} {a:+.2f}·cos(q2 {fi:+.1f}°) {b:+.2f}·cos(q3 {psi:+.1f}°)  [%]")
    print("\nPara config.h (u en %, q en grados; u = c + a2c·cos q2 + a2s·sin q2 + a3c·cos q3 + a3s·sin q3):")
    print(f"// {Path(args.archivo).name}")
    for j in ("2", "3"):
        if resultado[j][3]:
            v = ", ".join(f"{x:.3f}f" for x in resultado[j][3][0])
            print(f"constexpr float MAPA_J{j}[5] = {{{v}}};  // c, a2c, a2s, a3c, a3s")
    graficar(np, args, filas, resultado)


def graficar(np, args, filas, resultado):
    import matplotlib
    if args.sin_ventana:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ejes = plt.subplots(2, 2, figsize=(12, 8.5))
    for fila, j in enumerate(("2", "3")):
        q2s, q3s, us, mejor = resultado[j]
        ax = ejes[fila][0]
        for lado, marca, txt in ((-1, "^", "desde abajo"), (1, "v", "desde arriba")):
            fs = [f for f in filas if int(f["lado_pedido"]) == lado]
            ax.scatter([float(f["q2"]) + 0.6 * lado for f in fs], [float(f["duty" + j]) for f in fs],
                       marker=marca, label=txt, alpha=0.7)
        ax.scatter(q2s, us, marker="o", color="k", label="promedio")
        if mejor:
            pred = np.array([mejor[3](a, b) for a, b in zip(q2s, q3s)]) @ mejor[0]
            ax.scatter(q2s, pred, marker="x", color="r", s=60, label=f"modelo (RMS {mejor[1]:.1f} %)")
        ax.set_xlabel("q2 (°)  [cada columna: varias q3]")
        ax.set_ylabel(f"duty de J{j} sosteniendo (%)")
        ax.set_title(f"J{j}: medido y modelo")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax = ejes[fila][1]
        if mejor:
            g2, g3 = np.meshgrid(np.linspace(0, 97, 98), np.linspace(0, 113, 114))
            Z = np.array([[np.dot(mejor[3](a, b), mejor[0]) for a, b in zip(r2, r3)] for r2, r3 in zip(g2, g3)])
            Z[(g3 - g2 < BANDA_MIN) | (g3 - g2 > BANDA_MAX)] = np.nan
            cs = ax.contourf(g2, g3, Z, levels=15, cmap="viridis")
            fig.colorbar(cs, ax=ax, label="duty del modelo (%)")
        ax.scatter(q2s, q3s, c="w", edgecolors="k", s=30, label="poses medidas")
        ax.set_xlabel("q2 (°)")
        ax.set_ylabel("q3 (°)")
        ax.set_title(f"J{j}: modelo en la franja")
        ax.legend(fontsize=8, loc="upper left")
    fig.suptitle(f"Mapa de gravedad · {Path(args.archivo).name}")
    fig.tight_layout()
    png = Path(args.archivo).with_suffix(".png")
    fig.savefig(png, dpi=110)
    print(f"\nPNG: {png}")
    if not args.sin_ventana:
        plt.show()


def plan(args):
    inicio = {"j2": args.desde[0], "j3": args.desde[1]}
    for (q2, q3), lado, pasos in secuencia(inicio, POSES, args.activa, args.aprox):
        print(f"pose ({q2}, {q3}) {'desde abajo' if lado < 0 else 'desde arriba'}: "
              + " · ".join(comando(a, x, args.activa) for a, x in pasos) + " · an")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="modo", required=True)
    c = sub.add_parser("capturar", help="medir en la placa")
    c.add_argument("--puerto", default=PUERTO, help="def. el de tools/monitor.sh (por id, no cambia)")
    c.add_argument("--paso", action="store_true", help="esperar Enter antes de cada pose")
    c.add_argument("--no-volver", dest="volver", action="store_false",
                   help="al terminar, no volver a q2 5°, q3 0° (por defecto vuelve: abrir el monitor suelta el brazo)")
    c.add_argument("--cero", action="store_true",
                   help='mandar "z" al empezar (J2 y J3 en sus finales de carrera; abrir el puerto puede reiniciar la '
                        'placa y borrar el cero)')
    p = sub.add_parser("plan", help="mostrar los comandos sin tocar la placa")
    p.add_argument("--desde", type=float, nargs=2, default=(4.0, 0.0), metavar=("Q2", "Q3"))
    for x in (c, p):
        x.add_argument("-a", "--activa", choices=("j2", "j3"), default="j2", help="la del entorno flasheado (def. j2)")
        x.add_argument("--aprox", type=float, default=2.0, help="grados de aproximación de cada lado (def. 2)")
    a = sub.add_parser("ajustar", help="ajustar y graficar un CSV")
    a.add_argument("archivo")
    a.add_argument("--sin-ventana", action="store_true", help="solo guarda el PNG")
    a.add_argument("--incluir-inestables", action="store_true", help="usar las que se movieron durante el promedio")
    a.add_argument("--incluir-un-lado", action="store_true", help="usar las poses medidas desde un solo lado")
    args = ap.parse_args()
    if args.modo == "capturar":
        args.archivo = str(capturar(args))
        args.sin_ventana = False
        args.incluir_inestables = args.incluir_un_lado = False
        ajustar(args)
    elif args.modo == "plan":
        plan(args)
    else:
        ajustar(args)


if __name__ == "__main__":
    main()
