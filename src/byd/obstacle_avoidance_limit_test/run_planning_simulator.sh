#!/bin/bash
# 启动 planning_simulator 用于绕障极限测试
# 用法: ./run_planning_simulator.sh

set -euo pipefail

AUTOWARE_ROOT="/home/nvidia/autoware"
MAP_PATH="${MAP_PATH:-/home/nvidia/autoware_map/9_out/}"

cd "${AUTOWARE_ROOT}"
source install/setup.bash

echo "Launching planning_simulator for obstacle avoidance limit test..."
echo "  map_path=${MAP_PATH}"
echo "  vehicle_model=byd_vehicle"
echo "  sensor_model=byd_sensor_kit"
echo ""
echo "After RViz opens:"
echo "  1. Wait for Autoware to finish loading"
echo "  2. In another terminal run: ./run_sweep_test.sh"
echo ""

ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:="${MAP_PATH}" \
  vehicle_model:=byd_vehicle \
  sensor_model:=byd_sensor_kit \
  lanelet2_map_file:=parking.osm \
  pointcloud_map_file:=pointcloud_map.pcd \
  launch_didrive_perception:=false \
  scenario_simulation:=false \
  rviz:=false
