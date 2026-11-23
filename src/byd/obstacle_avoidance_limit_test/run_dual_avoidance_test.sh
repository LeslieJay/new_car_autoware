#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u
exec /usr/bin/python3 "$ROOT/src/byd/obstacle_avoidance_limit_test/scripts/dual_avoidance_suite.py" "$@"
