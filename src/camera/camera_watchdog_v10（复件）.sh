#!/bin/bash

# ==================== -1. 开机初始缓冲延时 ====================
# 【开机自启专用】等待系统底层驱动、CSI 接口以及系统服务完全就绪
BOOT_DELAY=15
echo "[INFO] 脚本刚启动，进入开机初始缓冲，等待 ${BOOT_DELAY} 秒以确保硬件传感器就绪..."
sleep ${BOOT_DELAY}

# ==================== 0. 环境与依赖加载 ====================
echo "[INFO] 正在加载 ROS 环境..."
if [ -f "/home/nvidia/autoware/install/setup.bash" ]; then
    source /home/nvidia/autoware/install/setup.bash
else
    echo "[ERROR] 未找到 /home/nvidia/autoware/install/setup.bash，请检查路径！"
    exit 1
fi

# ==================== 1. 系统初始化与设置 ====================
echo "[INFO] 正在配置系统性能与限制..."
sudo /usr/sbin/nvpmodel -m 0
sudo /usr/bin/jetson_clocks
ulimit -n 65535

echo "[INFO] 首次重启 nvargus-daemon..."
sudo /usr/bin/systemctl restart nvargus-daemon
sleep 3  # 等待守护进程完全初始化

# ==================== 2. 参数与日志路径配置 ====================
export GST_PLUGIN_PATH="/home/nvidia/autoware/install/gst_bridge/lib/gst_bridge/:$GST_PLUGIN_PATH"

# 创建日志文件夹
LOG_DIR="/home/nvidia/autoware/camera/camera_logs"
mkdir -p "$LOG_DIR"

# 捕获 Ctrl+C 信号，确保退出脚本时能干净地杀掉所有后台进程
cleanup() {
    echo ""
    echo "[INFO] 正在停止所有相机进程并退出脚本..."
    killall -9 gst-launch-1.0 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# ==================== 3. 看门狗主循环 ====================
while true; do
    echo "[INFO] 正在启动 4 个相机 (硬件采集 20Hz -> videorate 抽帧至 10Hz)..."
    
    PIDS=()
    # 循环启动 4 个相机
    for i in {0..3}; do
        TOPIC="/camera${i}/image_raw"
        CURRENT_LOG="${LOG_DIR}/cam_${i}_current.log"
        NODE_NAME="camera_sink_${i}"
        
        # 采用硬件20Hz + videorate抽帧10Hz + 软转码转换管道
        gst-launch-1.0 --gst-plugin-path=/home/nvidia/autoware/install/gst_bridge/lib/gst_bridge/ \
            nvarguscamerasrc sensor-id=$i ! \
            'video/x-raw(memory:NVMM), width=(int)1920, height=(int)1200, format=(string)NV12, framerate=(fraction)20/1' ! \
            nvvidconv ! 'video/x-raw' ! \
            videorate ! 'video/x-raw, framerate=10/1' ! \
            queue max-size-buffers=2 leaky=downstream ! \
            videoconvert ! \
            queue max-size-buffers=2 leaky=downstream ! \
            rosimagesink ros-name="${NODE_NAME}" ros-topic="${TOPIC}" > "$CURRENT_LOG" 2>&1 &
            
        PIDS+=($!)
        echo "[INFO] 相机 ID=$i 节点名=${NODE_NAME} 已启动，PID: ${PIDS[$i]}，实时日志: $CURRENT_LOG"
    done

    echo "[INFO] 所有相机已拉起，看门狗正在监控中..."

    # ==================== 4. 异常检测与日志转储 ====================
    # wait -n 会挂起脚本，直到数组中【任意一个】后台进程退出/崩溃
    wait -n

    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    echo "[WARN] [$TIMESTAMP] 检测到有相机进程异常退出！看门狗被触发！"
    
    # 立即备份这 4 个相机当前的日志，防止被下一次启动覆盖
    echo "[INFO] 正在转储崩溃时的相机错误日志..."
    for i in {0..3}; do
        if [ -f "${LOG_DIR}/cam_${i}_current.log" ]; then
            cp "${LOG_DIR}/cam_${i}_current.log" "${LOG_DIR}/crash_cam_${i}_${TIMESTAMP}.log"
        fi
    done
    echo "[INFO] 错误日志已保存至 ${LOG_DIR}/ 目录下，文件名带时间戳 ${TIMESTAMP}"

    # 清理可能残余或卡死的其他相机进程
    for pid in "${PIDS[@]}"; do
        kill -9 $pid 2>/dev/null
    done
    killall -9 gst-launch-1.0 2>/dev/null

    # 重启核心驱动服务
    echo "[INFO] 正在重启 nvargus-daemon..."
    sudo /usr/bin/systemctl restart nvargus-daemon
    
    # 给予底层硬件传感器断电复位的缓冲时间
    sleep 4
    
    echo "[INFO] 准备重新拉起所有相机..."
    echo "----------------------------------------------------"
done
