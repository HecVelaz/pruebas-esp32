#!/usr/bin/env bash
# 25: registra la alimentación de la Pi mientras corren teleop + visión (prueba del power bank).
# CSV cada 2 s: hora, EXT5V [V], potencia de los rieles del PMIC [W], throttled, temperatura [°C], carga 1 min, MHz.
# Avisa (sin cortar nada) si EXT5V < 4,85 V o throttled != 0x0. Ctrl+C termina.
# Uso: bash ~/scripts/25-teleop-pi/tools/registrar_tension.sh [segundos, 0 = sin fin]
export LC_ALL=C   # awk con coma decimal lee 0.1 como 0
DUR=${1:-0}
mkdir -p ~/logs
CSV=~/logs/prueba_powerbank_real_$(date +%Y%m%d_%H%M).csv
echo "hora,ext5v_v,rieles_w,throttled,temp_c,carga1,mhz" > "$CSV"
echo "Registrando en $CSV"
t0=$(date +%s)
while :; do
  adc=$(vcgencmd pmic_read_adc)
  ext5v=$(sed -n 's/.*EXT5V_V volt([0-9]*)=\([0-9.]*\)V.*/\1/p' <<< "$adc")
  # potencia = suma de corriente x tensión de cada riel (pares NOMBRE_A / NOMBRE_V)
  rieles=$(awk -F'[ =]+' '
    /current/ { n=$2; sub(/_A$/,"",n); gsub(/A/,"",$4); i[n]=$4 }
    /volt/    { n=$2; sub(/_V$/,"",n); gsub(/V/,"",$4); v[n]=$4 }
    END { for (k in i) if (k in v) p+=i[k]*v[k]; printf "%.2f", p }' <<< "$adc")
  thr=$(vcgencmd get_throttled | cut -d= -f2)
  temp=$(vcgencmd measure_temp | sed 's/[^0-9.]//g')
  carga=$(cut -d' ' -f1 /proc/loadavg)
  mhz=$(( $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq) / 1000 ))
  echo "$(date +%T),$ext5v,$rieles,$thr,$temp,$carga,$mhz" | tee -a "$CSV"
  if awk "BEGIN{exit !($ext5v < 4.85)}" || [ "$thr" != "0x0" ]; then
    echo "  >>> OJO: EXT5V $ext5v V, throttled $thr" >&2
  fi
  [ "$DUR" -gt 0 ] && [ $(( $(date +%s) - t0 )) -ge "$DUR" ] && break
  sleep 2
done
