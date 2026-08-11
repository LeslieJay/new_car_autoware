#!/usr/bin/env bash

set -eo pipefail

OUTPUT_DIR="${1:-${HOME}/autoware/log/$(date +%Y%m%d)}"
mkdir -p "${OUTPUT_DIR}"

ros2 bag record \
-e "^/planning($|/)|^/control($|/)|^/vehicle($|/)|^/simulation($|/)|^/perception($|/)|^/sensing($|/)|^/localization($|/)|^/system($|/)|^/diagnostics($|/)|^/tf$|^/tf_static$|^/clock$" \
-o "${OUTPUT_DIR}/$(date +%H%M%S)_bag"
