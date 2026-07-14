#!/usr/bin/env bash
# Switch Autoware to LOCAL external control so /external/selected/control_cmd
# is forwarded by vehicle_cmd_gate to /control/command/control_cmd.

set -eo pipefail

if [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "${HOME}/autoware/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/autoware/install/setup.bash"
elif [ -n "${AMENT_PREFIX_PATH:-}" ]; then
  :
else
  echo "[setup_external_control] workspace setup.bash not found" >&2
  exit 1
fi

wait_for_service() {
  local service=$1
  local timeout=${2:-30}
  local elapsed=0
  until ros2 service list 2>/dev/null | grep -qF "${service}"; do
    sleep 1
    elapsed=$((elapsed + 1))
    if [ "${elapsed}" -ge "${timeout}" ]; then
      echo "[setup_external_control] timeout waiting for ${service}" >&2
      return 1
    fi
  done
}

echo "[setup_external_control] waiting for operation mode services..."
wait_for_service "/api/operation_mode/change_to_local"
wait_for_service "/api/operation_mode/enable_autoware_control"
wait_for_service "/api/autoware/set/engage"
wait_for_service "/control/vehicle_cmd_gate/set_pause"

echo "[setup_external_control] change_to_local"
ros2 service call /api/operation_mode/change_to_local \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}" >/dev/null

echo "[setup_external_control] enable_autoware_control"
ros2 service call /api/operation_mode/enable_autoware_control \
  autoware_adapi_v1_msgs/srv/ChangeOperationMode "{}" >/dev/null

echo "[setup_external_control] engage=true"
ros2 service call /api/autoware/set/engage \
  tier4_external_api_msgs/srv/Engage "{engage: true}" >/dev/null

echo "[setup_external_control] unpause"
ros2 service call /control/vehicle_cmd_gate/set_pause \
  tier4_control_msgs/srv/SetPause "{pause: false}" >/dev/null

sleep 1
echo "[setup_external_control] gate_mode=$(ros2 topic echo /control/current_gate_mode --once 2>/dev/null | awk '/data:/ {print $2; exit}')"
echo "[setup_external_control] engage=$(ros2 topic echo /api/autoware/get/engage --once 2>/dev/null | awk '/engage:/ {print $2; exit}')"
echo "[setup_external_control] is_paused=$(ros2 topic echo /control/vehicle_cmd_gate/is_paused --once 2>/dev/null | awk '/data:/ {print $2; exit}')"

# Quick passthrough check (stop any manual ros2 topic pub first)
ros2 topic pub --once /external/selected/control_cmd autoware_control_msgs/msg/Control \
  "{longitudinal: {velocity: 0.3, acceleration: -1.0, is_defined_acceleration: true}}" >/dev/null 2>&1 || true
sleep 0.5
out_accel=$(ros2 topic echo /control/command/control_cmd --once 2>/dev/null | awk '/acceleration:/ {print $2; exit}')
out_defined=$(ros2 topic echo /control/command/control_cmd --once 2>/dev/null | awk '/is_defined_acceleration:/ {print $2; exit}')
echo "[setup_external_control] verify output acceleration=${out_accel} is_defined_acceleration=${out_defined}"
if [ "${out_accel}" = "-1.5" ] && [ "${out_defined}" = "false" ]; then
  echo "[setup_external_control] WARNING: still in stop-hold (-1.5). Check engage/pause/gate_mode." >&2
fi
echo "[setup_external_control] done"
