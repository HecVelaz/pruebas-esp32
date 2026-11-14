#!/usr/bin/env bash
# Copia en la Pi: ~/scripts/base.sh
source /opt/ros/jazzy/setup.bash
ESPERA=${2:-4}
(timeout $((ESPERA + 3)) ros2 topic echo --full-length /base/respuesta std_msgs/msg/String > /tmp/resp_base.txt 2>&1 &)
sleep 4
ros2 topic pub --once /base/comando std_msgs/msg/String "{data: '$1'}" > /dev/null
sleep "$ESPERA"
grep -v "^---" /tmp/resp_base.txt | sed "s/\\\\r//g"
