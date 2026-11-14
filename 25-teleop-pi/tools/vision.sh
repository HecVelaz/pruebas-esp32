#!/usr/bin/env bash
# Copia en la Pi: ~/scripts/vision.sh
# Lanza la visión de limones (start_limon.sh con el v3) en segundo plano si no está corriendo. Log en ~/logs/.
if ps -eo args | grep -q "^/home/r2d2/camera_ws/install/camera_ros/lib/camera_ros/camera_node"; then
  echo "la visión ya corre"; exit 0
fi
cd ~/vision_estereo/scripts || exit 1
LOG=~/logs/start_limon_$(date +%Y%m%d_%H%M%S).log
MODELO=~/vision_estereo/modelos/limon_yolo26n_v3_ncnn_model CLASE=limon nohup setsid ./start_limon.sh > "$LOG" 2>&1 < /dev/null &
echo "visión lanzada (log $LOG)"
