#!/usr/bin/env bash

set -eo pipefail

OUTPUT_DIR="${1:-${HOME}/autoware/log/$(date +%Y%m%d)}"
mkdir -p "${OUTPUT_DIR}"

INCLUDE_REGEX="^/planning($|/)|^/control($|/)|^/vehicle($|/)|^/simulation($|/)|^/localization($|/)|^/system($|/)|^/diagnostics($|/)|^/tf$|^/tf_static$|^/clock$|^/sensing/lidar/concatenated/pointcloud$|^/perception/obstacle_segmentation/pointcloud$|^/perception/object_recognition/detection/lidar_rule/objects$|^/perception/object_recognition/detection/lidar_dnn/objects$|^/perception/object_recognition/detection/merged/objects$|^/perception/object_recognition/detection/objects$|^/perception/object_recognition/tracking/objects$|^/perception/object_recognition/objects$|^/sensing/imu/imu_data$"

ros2 bag record \
--all \
--regex "${INCLUDE_REGEX}" \
--output "${OUTPUT_DIR}/$(date +%H%M%S)_bag"
