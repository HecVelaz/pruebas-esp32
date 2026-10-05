#!/usr/bin/env python3
"""Genera el informe de simulaciones de J1 y J3 a partir de simulaciones/informe_simulaciones.tex.

El .tex es la fuente (el mismo archivo se sube a Overleaf). Este script:
  1. dibuja el diagrama de bloques del control (simulaciones/diagrama_control.png) con matplotlib;
  2. convierte el .tex a HTML con pandoc (ecuaciones en MathML) y lo imprime a PDF con Firefox headless
     (geckodriver de la snap de Firefox): simulaciones/informe_simulaciones.pdf, porque esta PC no tiene LaTeX;
  3. arma simulaciones/informe_simulaciones_overleaf.zip con el .tex y las imágenes que usa, para subir a Overleaf
     (New Project > Upload Project) y compilar ahí con pdfLaTeX.
Las imágenes de simulaciones/j1 y simulaciones/j3 salen de diseno_velocidad.py, diseno_posicion.py,
escalon_velocidad.py, escalon_posicion.py e identificar_j3.py; los números del .tex son los que imprimieron
(2026-10-04): si se rehacen las simulaciones, actualizar el .tex.

Uso:
    python3 tools/informe_simulaciones.py
"""
import base64
import json
import re
import shutil
import subprocess
import time
import urllib.error
import urllib.request
import zipfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402

DIR = Path(__file__).resolve().parent.parent / "simulaciones"
TEX = DIR / "informe_simulaciones.tex"
PDF = DIR / "informe_simulaciones.pdf"
ZIP = DIR / "informe_simulaciones_overleaf.zip"
DIAGRAMA = DIR / "diagrama_control.png"
BUILD = DIR / "_build"
DPI = 220
GECKO = "/snap/bin/geckodriver"
PUERTO = 4466

CSS = """
@page { size: A4; margin: 15mm 17mm 15mm 17mm; }
body { font-family: "DejaVu Serif", Georgia, serif; font-size: 9.6pt; line-height: 1.3; color: #111; max-width: none;
       margin: 0; padding: 0; }
h1, h2 { font-size: 12.5pt; margin: 9pt 0 3pt 0; break-after: avoid; }
p { text-align: justify; margin: 3pt 0; }
figure { text-align: center; margin: 8pt 0; break-inside: avoid; }
figure img { max-width: 100%; }
figcaption { font-size: 9pt; color: #333; margin-top: 2pt; }
table { border-collapse: collapse; margin: 4pt auto; font-size: 9pt; width: 100%; }
th, td { padding: 1.5pt 6pt; vertical-align: middle; }
thead tr { border-top: 1.5px solid #000; border-bottom: 1px solid #000; }
tbody tr:last-child { border-bottom: 1.5px solid #000; }
math[display="block"] { margin: 3pt 0; }
ul { margin: 2pt 0; } li { margin: 1pt 0; }
#figuras { break-before: page; }
"""


def diagrama():
    """Diagrama de bloques de la cascada posición -> velocidad -> motor (simulaciones/diagrama_control.png)."""
    fig, ax = plt.subplots(figsize=(11, 3.2))
    ax.set_xlim(0, 110)
    ax.set_ylim(-1, 30)
    ax.axis("off")
    azul = "#1d3557"

    def caja(x, y, w, h, texto, color="#eef3fb"):
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.4", fc=color, ec=azul, lw=1.2, zorder=3))
        ax.text(x + w / 2, y + h / 2, texto, ha="center", va="center", fontsize=10, zorder=4)

    def suma(x, y):
        ax.add_patch(plt.Circle((x, y), 1.6, fc="white", ec=azul, lw=1.2, zorder=3))
        ax.text(x - 3.6, y - 3.6, "+", fontsize=11)
        ax.text(x + 0.6, y - 4.6, "−", fontsize=12)

    def flecha(x0, y0, x1, y1, texto=None):
        ax.annotate("", xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle="->", lw=1.2, color=azul))
        if texto:
            ax.text((x0 + x1) / 2, y0 + 1.5, texto, ha="center", fontsize=11)

    y = 21
    flecha(0, y, 6.4, y, r"$\theta_{ref}$")
    suma(8, y)
    flecha(9.6, y, 13.5, y)
    caja(13.5, y - 4, 17, 8, "Control de\nposición (P)")
    flecha(31.3, y, 38.4, y, r"$\omega_{ref}$")
    suma(40, y)
    flecha(41.6, y, 45.5, y)
    caja(45.5, y - 4, 17, 8, "Control de\nvelocidad (PI)")
    flecha(63.3, y, 70.5, y, r"$u$")
    caja(70.5, y - 4, 17, 8, "Motor\n" + r"$\dfrac{K}{\tau s+1}$", "#fdf0e6")
    flecha(88.3, y, 93.5, y, r"$\omega$")
    caja(93.5, y - 3, 6, 6, r"$\frac{1}{s}$", "#fdf0e6")
    flecha(100.3, y, 109, y, r"$\theta$")
    # realimentación: el encoder mide el ángulo; de ahí sale también la velocidad
    ax.plot([104, 104, 70], [y, 1.5, 1.5], color=azul, lw=1.2, zorder=1)
    caja(52, -0.5, 18, 4, "Encoder", "#f3f3f3")
    ax.plot([52, 8, 8], [1.5, 1.5, y - 2.5], color=azul, lw=1.2, zorder=1)
    ax.annotate("", xy=(8, y - 1.6), xytext=(8, y - 3), arrowprops=dict(arrowstyle="->", lw=1.2, color=azul))
    ax.text(9, 8, r"ángulo medido $\theta$", fontsize=10)
    ax.plot([61, 61, 40], [3.9, 11, 11], color=azul, lw=1.2, zorder=1)
    ax.annotate("", xy=(40, y - 1.6), xytext=(40, 11), arrowprops=dict(arrowstyle="->", lw=1.2, color=azul))
    ax.text(41, 12.3, r"velocidad medida $\omega$", fontsize=10)
    fig.savefig(DIAGRAMA, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)


def imprimir(html_path, pdf_path):
    proc = subprocess.Popen([GECKO, "--port", str(PUERTO)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def call(method, path, body=None):
        for _ in range(100):
            try:
                req = urllib.request.Request(f"http://127.0.0.1:{PUERTO}{path}", method=method,
                                             data=json.dumps(body).encode() if body is not None else None,
                                             headers={"Content-Type": "application/json"})
                return json.load(urllib.request.urlopen(req, timeout=120))
            except urllib.error.HTTPError:
                raise
            except urllib.error.URLError:
                time.sleep(0.2)
        raise RuntimeError("geckodriver no responde")

    try:
        sid = call("POST", "/session", {"capabilities": {"alwaysMatch": {
            "moz:firefoxOptions": {"args": ["-headless"]}}}})["value"]["sessionId"]
        call("POST", f"/session/{sid}/url", {"url": html_path.resolve().as_uri()})
        time.sleep(1.0)
        pdf = call("POST", f"/session/{sid}/print", {"page": {"width": 21.0, "height": 29.7}, "background": True,
                                                       "margin": {"top": 0, "bottom": 0, "left": 0, "right": 0},
                                                       "shrinkToFit": False})["value"]
        pdf_path.write_bytes(base64.b64decode(pdf))
        call("DELETE", f"/session/{sid}")
    finally:
        proc.terminate()


def main():
    diagrama()
    # imágenes que usa el .tex
    imagenes = re.findall(r"\\includegraphics(?:\[[^]]*\])?\{([^}]+)\}", TEX.read_text(encoding="utf-8"))
    faltan = [i for i in imagenes if not (DIR / i).exists()]
    if faltan:
        raise SystemExit(f"Faltan imágenes: {faltan}")

    BUILD.mkdir(exist_ok=True)
    (BUILD / "estilo.css").write_text(CSS, encoding="utf-8")
    pagina = DIR / "_informe.html"   # junto al .tex, para que las rutas de las imágenes sigan valiendo
    subprocess.run(["pandoc", str(TEX), "-f", "latex", "-t", "html5", "--standalone", "--mathml",
                    "--metadata", "lang=es", "--metadata", "pagetitle=Simulación del control de J1 y J3", "--css", str(BUILD / "estilo.css"), "-o", str(pagina)], check=True)
    try:
        imprimir(pagina, PDF)
    finally:
        pagina.unlink(missing_ok=True)
        shutil.rmtree(BUILD, ignore_errors=True)

    with zipfile.ZipFile(ZIP, "w", zipfile.ZIP_DEFLATED) as z:
        z.write(TEX, TEX.name)
        for i in imagenes:
            z.write(DIR / i, i)
    print(f"PDF: {PDF.relative_to(DIR.parent)} ({PDF.stat().st_size / 1e6:.1f} MB)")
    print(f"Overleaf: {ZIP.relative_to(DIR.parent)} ({len(imagenes)} imágenes + el .tex)")


if __name__ == "__main__":
    main()
