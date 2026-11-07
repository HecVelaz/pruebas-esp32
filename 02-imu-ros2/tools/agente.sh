#!/usr/bin/env bash
# Terminal 1: micro-ROS Agent por el USB nativo de la S3. Usa la ruta by-id, que no cambia si la placa
# pasa de /dev/ttyACM0 a /dev/ttyACM1. Detenerlo (Ctrl+C) antes de flashear.
set -e
source /opt/ros/jazzy/setup.bash
source ~/ros/microros_ws/install/local_setup.bash
PUERTO=${1:-$(ls /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_*-if00 2>/dev/null | head -1)}
[ -n "$PUERTO" ] || { echo "No encuentro la S3 en /dev/serial/by-id (¿enchufada?)"; exit 1; }
echo "Agente en $PUERTO -> $(readlink -f "$PUERTO"). Si no aparece 'session established': RST en la placa y dejarla quieta."
exec ros2 run micro_ros_agent micro_ros_agent serial --dev "$PUERTO" -b 115200
