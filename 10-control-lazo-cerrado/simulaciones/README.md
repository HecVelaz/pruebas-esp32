# Simulaciones de J1 y J3

- `informe_simulaciones.tex`: informe corto (la explicación en una página y los gráficos después). Es la fuente.
- `informe_simulaciones_overleaf.zip`: el `.tex` con las imágenes que usa. En Overleaf: *New Project → Upload
  Project*, elegir el zip y compilar con pdfLaTeX.
- `informe_simulaciones.pdf`: vista previa del mismo `.tex` hecha en esta PC (pandoc + Firefox, porque no hay LaTeX);
  el PDF bueno es el que sale de Overleaf.
- Para rehacer el PDF, el zip y el diagrama: `python3 tools/informe_simulaciones.py`.

## Imágenes

| Imagen | Qué es |
|--------|--------|
| `diagrama_control.png` | Los dos lazos del control (posición → velocidad → motor) |
| `j1/j1_1_diseno_velocidad_teorico.png` | J1, simulación del PI de diseño (Kp 0,549, Ki 8,44) |
| `j1/j1_2_diseno_velocidad_ganancias_elegidas.png` | J1, simulación del PI usado en la placa (Kp 0,549, Ki 4) |
| `j1/j1_3_diseno_posicion.png` | J1, simulación del lazo de posición (Kpp 2) |
| `j1/j1_4_comparacion_velocidad_sim_vs_real.png` | J1, velocidad: placa contra simulación (`resultados/escalon_vel_20261002_215350.csv`) |
| `j1/j1_5_comparacion_posicion_sim_vs_real.png` | J1, posición: placa contra simulación (`resultados/escalon_pos_20261002_223821.csv`) |
| `j3/j3_1_identificacion_planta.png` | J3, cómo se obtuvo el modelo (barridos y escalones guardados, `tools/identificar_j3.py`) |
| `j3/j3_2_diseno_velocidad_teorico.png` | J3, simulación del PI de diseño (Kp 0,52, Ki 9,9) |
| `j3/j3_3_diseno_velocidad_ganancias_elegidas.png` | J3, simulación del PI usado en la placa (Kp 0,35, Ki 5) |
| `j3/j3_4_diseno_posicion.png` | J3, simulación del lazo de posición (Kpp 2) |
| `j3/j3_5_comparacion_velocidad_sim_vs_real.png` | J3, velocidad: placa contra simulación (`resultados/escalon_vel_20261003_190957.csv`, Kp 0,5) |
| `j3/j3_6_comparacion_posicion_sim_vs_real_kp05.png` | J3, posición con Kp 0,5 (`resultados/escalon_pos_20261003_191345.csv`) |
| `j3/j3_7_comparacion_posicion_sim_vs_real_final.png` | J3, posición final: placa contra simulación (`resultados/escalon_pos_20261003_212157.csv`) |

El informe usa el diagrama, `j1_4`, `j1_5`, `j3_1` y `j3_7`; las demás quedan como material de apoyo.
