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
echo "Agentes: brazo $BRAZO, base $BASE (logs en ~/logs). Si la página dice \"sin datos\", pulsar RST en esa placa."
sleep 2
python3 "$DIR/teleop.py" "$@" &
pids+=($!)
wait
