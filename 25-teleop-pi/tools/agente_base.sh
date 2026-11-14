#!/usr/bin/env bash
# Copia en la Pi: ~/scripts/agente_base.sh
# Lanza el agente micro-ROS de la base (S3) en segundo plano, si no está corriendo. Log: ~/logs/agente_base.log
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
if pgrep -f "micro_ros_agent serial" > /dev/null; then echo "ya corre"; exit 0; fi
BASE=$(ls /dev/serial/by-id/usb-Espressif_USB_JTAG* 2>/dev/null | head -1)
[ -z "$BASE" ] && { echo "no veo la S3"; exit 1; }
nohup setsid ros2 run micro_ros_agent micro_ros_agent serial --dev "$BASE" > ~/logs/agente_base.log 2>&1 < /dev/null &
echo "agente lanzado en $BASE"
