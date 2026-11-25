#!/usr/bin/env bash
# Copia en la Pi: ~/scripts/vision.sh
# Lanza la visión de limones (start_limon.sh con el v4) en segundo plano si no está corriendo. Log en ~/logs/.
if ps -eo args | grep -q "^/home/r2d2/camera_ws/install/camera_ros/lib/camera_ros/camera_node"; then
  echo "la visión ya corre"; exit 0
fi
cd ~/vision_estereo/scripts || exit 1
LOG=~/logs/start_limon_$(date +%Y%m%d_%H%M%S).log
# 21/11: v4 (Optuna, Vision_limones_robot_gabi): sin el falso limón sobre el panel de metal; NCNN exportado a 640 fijo.
# Volver al v3: MODELO=~/vision_estereo/modelos/limon_yolo26n_v3_ncnn_model IMGSZ=480
# 25/11: con prioridad baja (nice 10): con limones a la vista usa ~2,4 núcleos y el puente se atrasaba (la S3 cortaba
# por "sin comandos" y la misión con BASE_SAFETY_NOT_OK); así solo usa la CPU que sobra
MODELO=~/vision_estereo/modelos/limon_yolo26n_v4_ncnn_model IMGSZ=640 CLASE=limon nice -n 10 nohup setsid ./start_limon.sh > "$LOG" 2>&1 < /dev/null &
echo "visión lanzada (log $LOG)"
