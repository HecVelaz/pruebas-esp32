#!/usr/bin/env bash
# Copia en la Pi: ~/scripts/agente_base.sh
# Lanza el agente micro-ROS de la base (S3) en segundo plano, si no está corriendo. Log: ~/logs/agente_base.log
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
# 26/11: solo el agente de la S3 (antes cualquier "micro_ros_agent serial": con el del brazo corriendo no lanzaba el
# de la base y había que hacerlo a mano)
if pgrep -f "[m]icro_ros_agent serial --dev /dev/serial/by-id/usb-Espressif" > /dev/null; then echo "ya corre"; exit 0; fi
BASE=$(ls /dev/serial/by-id/usb-Espressif_USB_JTAG* 2>/dev/null | head -1)
[ -z "$BASE" ] && { echo "no veo la S3"; exit 1; }
nohup setsid ros2 run micro_ros_agent micro_ros_agent serial --dev "$BASE" > ~/logs/agente_base.log 2>&1 < /dev/null &
echo "agente lanzado en $BASE"
