#!/bin/bash

# 每次运行新建日期时间文件夹，分别保存各终端 log
LOG_ROOT="/home/nvidia/autoware/log"
LOG_DIR="${LOG_ROOT}/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "Logs will be saved to: $LOG_DIR"

# SSH / 无图形会话时，尝试本机桌面显示
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"

# 检测是否能打开图形终端；不能则后台运行
USE_GUI=0
if command -v gnome-terminal >/dev/null 2>&1 && xdpyinfo >/dev/null 2>&1; then
  USE_GUI=1
fi
echo "USE_GUI=$USE_GUI DISPLAY=$DISPLAY"

run_job() {
  local name="$1"
  local logfile="$2"
  shift 2
  local cmd="$*"

  if [[ "$USE_GUI" -eq 1 ]]; then
    gnome-terminal --tab --title="$name" -- bash -c "$cmd &> '${logfile}'; exec bash"
  else
    echo "[bg] starting $name -> $logfile"
    nohup bash -c "$cmd" >"$logfile" 2>&1 &
    echo $! >"${LOG_DIR}/${name}.pid"
  fi
}

run_job can_rtk "${LOG_DIR}/can_rtk.log" \
  "cd /home/nvidia/can_fd_ws && source install/setup.bash && ros2 run can_six_driver can_rtk_node"

run_job rslidar "${LOG_DIR}/rslidar.log" \
  "cd /home/nvidia/autoware && source install/setup.bash && RCUTILS_LOGGING_SEVERITY=DEBUG ros2 launch rslidar_sdk start_3.py"

run_job autoware "${LOG_DIR}/autoware.log" \
  "cd /home/nvidia/autoware && source install/setup.bash && ros2 launch autoware_launch autoware.launch.xml"

run_job auto_engage "${LOG_DIR}/auto_engage.log" \
  "source /home/nvidia/autoware/install/setup.bash && ros2 launch byd_auto_engage auto_engage.launch.py"

if [[ "$USE_GUI" -eq 0 ]]; then
  echo "Started in background (no display). PIDs in: $LOG_DIR/*.pid"
  echo "Stop all: kill \$(cat $LOG_DIR/*.pid)"
fi
