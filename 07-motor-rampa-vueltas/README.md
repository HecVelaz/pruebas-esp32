# 07-motor-rampa-vueltas — Rampa de PWM por vueltas (lazo abierto, con y sin carga)

Barrido **gradual** del motor 36GP-555: el PWM sube linealmente de 0 a 100 % en un tiempo fijo y la prueba termina al completar **N vueltas del eje de salida** (por defecto **3 vueltas en 3 s**). Sirve para comparar el motor **sin carga y con carga**: cuánto PWM hace falta para arrancar, qué velocidad alcanza con cada PWM y con qué PWM completa cada vuelta.

Es una prueba aparte: **no modifica** el firmware ni `tools/graficar.py` de `06-motor-36gp555`. Usa los mismos drivers (`lib/BTS7960` y `lib/EncoderPCNT` de 06, enlazados con `symlink://`) y el **mismo cableado** (ver `06-motor-36gp555/README.md`).

## Cómo funciona

- **Rampa por tiempo:** duty = 100 % × t / T. Con T = 3 s sube ~33 % por segundo.
- **Fin por vueltas:** al completar N vueltas de salida (N × 3200 cuentas), frena.
- **Con carga:** si llega al 100 % antes de completar las vueltas, se mantiene en 100 % hasta completarlas (como máximo 15 s más).
- **Por qué no por posición:** si el PWM dependiera solo de las vueltas, en la vuelta 0 sería 0 % y el motor nunca arrancaría.
- **Vueltas y segundos van juntos:** sin carga, las vueltas al llegar al 100 % son proporcionales a T. Medido: 5 s → 5 vueltas con 98,8 %; por eso el valor por defecto es **3 s → 3 vueltas** (también ~99 %). Para otra cantidad de vueltas, usar la misma cantidad de segundos, así la rampa siempre recorre 0 → 100 %.
- **Protecciones:** empieza solo con el motor quieto, corte por atasco (duty ≥ 40 % y menos de 300 rpm del motor durante 1 s) y Enter o cualquier línea frenan.

## Cómo probar

Cambiar de proyecto exige flashear: **apagar antes la salida de la fuente de 12 V** (sin pull-down en R_EN/L_EN, el driver flota durante el flasheo).

```bash
cd ~/pruebas-esp32/07-motor-rampa-vueltas
~/.platformio/penv/bin/pio run -t upload        # después pulsar RST y encender los 12 V
python3 tools/rampa.py                          # 0 -> 100 % en 3 s, hasta 3 vueltas
python3 tools/rampa.py --segundos 5 --vueltas 5 # 5 vueltas (la primera prueba)
python3 tools/rampa.py --vueltas -3             # hacia atrás
python3 tools/rampa.py --comparar resultados/rampa_<sin carga>.csv   # superponer otra prueba
python3 tools/rampa.py --archivo resultados/rampa_....csv            # volver a graficar
```

Para volver a las pruebas de 06 (barrido, escalón, perfil), flashear de nuevo `06-motor-36gp555`.

**Secuencia sugerida:** primero una rampa **sin carga** (la referencia), después la misma rampa **con carga**, y graficar la segunda con `--comparar` apuntando a la primera.

Desde el monitor serial también se puede usar directo: `r` (3 s, 3 vueltas), `r 5 5`, `r 3 -3`; `x` o Enter frenan.

## Gráficos (`tools/rampa.py`)

| Gráfico | Qué muestra |
|---------|-------------|
| Señal de control y posición | La rampa de PWM y las vueltas de salida en el tiempo, con una línea por vuelta |
| Velocidad de salida | rpm de salida en el tiempo (derivada de la posición con un promedio centrado de 110 ms, sin retraso) |
| Velocidad vs PWM | El barrido continuo: rpm contra PWM, junto a la recta sin carga medida con el perfil de 06 (`--ref-k 1.58 --ref-zm 8.3`) |
| PWM por vuelta | Con qué PWM completa cada vuelta (con carga, cada vuelta necesita más PWM) |

En la terminal imprime: el PWM de **arranque** (cuando la salida supera 1 rpm), una tabla con el tiempo, el PWM y la velocidad al completar cada vuelta, la recta **K y zona muerta** de la prueba (compensando el retraso τ de la velocidad frente a la rampa, `--tau 0.065`) y la **caída de velocidad** a 50 % y 100 % respecto del motor sin carga.

Guarda `resultados/rampa_<fecha>.csv` (con la configuración y el motivo del fin como comentarios) y el PNG.

## Con carga: cuidados

- El motor tiene mucho par (50:1): fijarlo a la mesa y no poner la mano en la carga.
- Un peso colgando no tiene que poder caer sobre nada ni nadie.
- Fuente de laboratorio con límite de ~3 A. Si la carga es alta y salta el corte por atasco, el script igual guarda y grafica los datos hasta ese momento. Para cargas altas, subir `ATASCO_DUTY_PCT` en `include/motor_config.h` con cuidado.

## Resultados

| Fecha | Carga | Arranque (duty %) | Duty por vuelta (1 … 5) | K (rpm/%) | Zona muerta | Observaciones |
|-------|-------|-------------------|-------------------------|-----------|-------------|---------------|
| 2026-09-25 | Sin carga (5 vueltas, 5 s) | 17 % | 52 / 68 / 80 / 90 / 99 % | ~1,6 (tramo 25–85 %) | ~8 % en movimiento | 5 vueltas en 4,95 s. Arranca desde parado recién con 17 %. Arriba de ~90 % salta y se aplana en ~142 rpm (probable límite de conmutación del BTS7960 a 20 kHz): rango lineal útil ~20–85 %. `resultados/rampa_20260925_184258.*` |
