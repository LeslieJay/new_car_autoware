#!/usr/bin/env bash

set -eo pipefail

OUTPUT_DIR="${1:-${HOME}/autoware/log/$(date +%Y%m%d)}"
mkdir -p "${OUTPUT_DIR}"

ros2 bag record \
-e "^/planning($|/)|^/control($|/)|^/vehicle($|/)|^/simulation($|/)|^/localization($|/)|^/system($|/)|^/diagnostics($|/)|^/tf$|^/tf_static$|^/clock$" \
-o "${OUTPUT_DIR}/$(date +%H%M%S)_bag" \
/sensing/lidar/concatenated/pointcloud \
/perception/obstacle_segmentation/pointcloud \
/perception/object_recognition/detection/lidar_rule/objects \
/perception/object_recognition/detection/lidar_dnn/objects \
/perception/object_recognition/detection/merged/objects \
/perception/object_recognition/detection/objects \
/perception/object_recognition/tracking/objects \
/perception/object_recognition/objects \
/tf \
/tf_static \
/localization/kinematic_state \
/localization/pose_with_covariance \
/vehicle/status/velocity_status \
/vehicle/status/steering_status \
/vehicle/status/gear_status \
/vehicle/status/control_mode \
/sensing/imu/imu_data
