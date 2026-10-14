#!/usr/bin/env bash

set -eo pipefail

OUTPUT_DIR="${1:-${HOME}/autoware/log/$(date +%Y%m%d)}"
mkdir -p "${OUTPUT_DIR}"

INCLUDE_REGEX="^/planning($|/)|^/control($|/)|^/vehicle($|/)|^/simulation($|/)|^/localization($|/)|^/system($|/)|^/diagnostics($|/)|^/perception/object_recognition/objects$|^/perception/object_recognition/tracking/objects$|^/perception/object_recognition/detection/lidar_rule/objects$|^/perception/object_recognition/detection/lidar_dnn/objects$|^/perception/object_recognition/detection/merged/objects$|^/perception/object_recognition/detection/objects$|^/perception/occupancy_grid_map/map$|^/perception/obstacle_segmentation/pointcloud$|^/tf$|^/tf_static$|^/clock$|^/sensing/lidar/concatenated/pointcloud$|^/sensing/imu/imu_data$"

ros2 bag record \
--regex "${INCLUDE_REGEX}" \
--output "${OUTPUT_DIR}/$(date +%H%M%S)_bag"

ros2 bag record -o lateral_gap_$(date +%Y%m%d_%H%M%S) \
  /planning/trajectory \
  /control/trajectory_follower/control_cmd \
  /control/command/control_cmd \
  /control/trajectory_follower/controller_node_exe/debug/ld_outputs \
  /control/trajectory_follower/lateral/predicted_trajectory \
  /control/trajectory_follower/controller_node_exe/output/debug_marker \
  /control/trajectory_follower/controller_node_exe/lateral/debug/processing_time_ms \
  /vehicle/status/steering_status \
  /vehicle/status/velocity_status \
  /localization/kinematic_state \
  /localization/acceleration \
  /vehicle/status/control_mode \
  /control/current_gate_mode \
  /vehicle/status/gear_status \
  /rtk_imu/data_raw


  