#!/usr/bin/env bash
# 25: levanta en la Pi los dos agentes micro-ROS (brazo y base) y el teleop; Ctrl+C cierra los tres.
# Logs de los agentes en ~/logs/agente_brazo.log y ~/logs/agente_base.log.
# Uso: bash ~/scripts/25-teleop-pi/tools/lanzar.sh [argumentos de teleop.py, p. ej. --v-max 0.2]
set -u
BRAZO=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
BASE=$(ls /dev/serial/by-id/usb-Espressif_USB_JTAG* 2>/dev/null | head -1)
BASE=${BASE:-/dev/ttyACM0}
DIR=$(cd "$(dirname "$0")" && pwd)
mkdir -p ~/logs

set +u
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
set -u

for p in "$BRAZO" "$BASE"; do
  [ -e "$p" ] || { echo "No está $p (¿USB conectado?)"; exit 1; }
done
# Lo que haya quedado de una corrida anterior (agentes con el puerto abierto, teleop con el 8000)
if pgrep -f "micro_ros_agent|tools/teleop.py" > /dev/null; then
  echo "Cerrando agentes / teleop que quedaron abiertos de antes..."
  pkill -f micro_ros_agent; pkill -f tools/teleop.py
  sleep 1.5
  pkill -9 -f micro_ros_agent; pkill -9 -f tools/teleop.py   # los que no se cerraron
  sleep 0.5
fi

# La S3 (USB nativo) a veces queda en modo descarga o sin arrancar después de flashear o reconectar el USB (antes:
# RST a mano). esptool la pone en modo descarga a propósito y la reinicia por watchdog, que con el USB-JTAG arranca
# siempre el programa (el reset por RTS es el que la dejaba trabada). Sin esto: SIN_RESET_S3=1 bash lanzar.sh
ESPTOOL=~/venvs/esptool/bin/esptool
if [ "${SIN_RESET_S3:-0}" != "1" ] && [ -x "$ESPTOOL" ]; then
  echo "Reiniciando la S3 (watchdog)..."
  if "$ESPTOOL" --chip esp32s3 --port "$BASE" --after watchdog-reset read-mac > ~/logs/reset_s3.log 2>&1; then
    sleep 3   # que vuelva a aparecer el USB y arranque
  else
    echo "  no se pudo reiniciar la S3 (ver ~/logs/reset_s3.log); sigo igual: si queda \"sin datos\", RST a mano"
  fi
fi

pids=()
cerrar() {
  echo; echo "Cerrando..."
  kill "${pids[@]}" 2>/dev/null
  sleep 1
  kill -9 "${pids[@]}" 2>/dev/null
  pkill -9 -f micro_ros_agent 2>/dev/null
  wait 2>/dev/null
  exit 0
}
trap cerrar INT TERM

ros2 run micro_ros_agent micro_ros_agent serial --dev "$BRAZO" > ~/logs/agente_brazo.log 2>&1 &
pids+=($!)
ros2 run micro_ros_agent micro_ros_agent serial --dev "$BASE" > ~/logs/agente_base.log 2>&1 &
pids+=($!)
echo "Agentes: brazo $BRAZO, base $BASE (logs en ~/logs). Si la página dice \"sin datos\" por más de 10 s, pulsar RST en esa placa."
sleep 2
python3 "$DIR/teleop.py" "$@" &
pids+=($!)
wait
