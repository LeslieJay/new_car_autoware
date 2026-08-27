#!/bin/bash

set -eo pipefail

OUTPUT_DIR="${1:-${HOME}/autoware/log/$(date +%Y%m%d)}"

mkdir -p "${OUTPUT_DIR}"

INCLUDE_REGEX="^/planning($|/)|\
^/control($|/)|\
^/vehicle($|/)|\
^/simulation($|/)|\
^/localization($|/)|\
^/system($|/)|\
^/diagnostics($|/)|\
^/tf$|\
^/tf_static$|\
^/clock$|\
^/sensing/imu/imu_data$"

ros2 bag record \
  --regex "${INCLUDE_REGEX}" \
  --output "${OUTPUT_DIR}/$(date +%H%M%S)_bag"