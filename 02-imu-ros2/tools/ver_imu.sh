#!/usr/bin/env bash
# Terminal 2: filtro Madgwick (/imu/data_raw -> /imu/data + TF odom -> imu_link) y RViz2 con imu.rviz.
# Sin magnetómetro: roll y pitch estables, el yaw deriva despacio. Ctrl+C cierra los dos.
source /opt/ros/jazzy/setup.bash
DIR=$(cd "$(dirname "$0")" && pwd)
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  -p use_mag:=false -p world_frame:=enu -p publish_tf:=true -p fixed_frame:=odom &
FILTRO=$!
trap 'kill $FILTRO 2>/dev/null' EXIT
rviz2 -d "$DIR/imu.rviz"
