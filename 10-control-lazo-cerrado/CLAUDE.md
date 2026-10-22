# 10-control-lazo-cerrado (y 12–14)

Reglas del lazo cerrado del brazo. Valen también para `12-j1-base`, `13-j2-hombro` y `14-j3-codo`, que usan estos
mismos `tools/` por symlink. El estado de cada articulación está en `../notas/ESTADO.md`.

- **Cada flasheo borra el cero:** con la articulación en la marca, `z` (en J2 con sostener: sostener el brazo, `z`, `a 0`).
- `tools/` es compartido: los scripts guardan en `resultados/` de la carpeta desde donde se llaman
  (`Path(__file__).absolute()`). Toman `-a j1/j2/j3/j3_5840` (`j3_5840` = el codo viejo con el 5840; `-a j3` = J3
  nuevo, sin simulación). Con `--archivo` vuelven a graficar un CSV sin tocar la placa.
- El usuario corre los comandos en la placa: pasarle los pasos con el orden de la fuente (prender/apagar los 12 V).
- El usuario no conoce mucho de control: explicar cada paso y registrar el avance en el README (casillas) y en
  `../notas/ESTADO.md`.
- **No renombrar imágenes enlazadas** desde `simulaciones/*_colab.md` o `imagenes/informe/`: rompe los enlaces de Colab.
- Los cambios de los drivers para el robot se hacen en `~/Documentos/recolector_de_frutas/firmware/lib`, no acá.
- `docs/` (copia de los documentos del robot) es solo para leer.
