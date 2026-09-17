#!/bin/bash

log() {
    echo "[$(date +'%Y-%m-%d %H:%M:%S')] $*"
}

echo ""
echo "================================================================================"
echo ">>> [$(date +'%Y-%m-%d %H:%M:%S')] 相机看门狗服务启动 (NEW BOOT / SERVICE START)"
echo "================================================================================"

# ==================== -1. 开机初始缓冲延时 ====================
# 【开机自启专用】等待系统底层驱动、CSI 接口以及系统服务完全就绪
BOOT_DELAY=10
log "[INFO] 脚本刚启动，进入开机初始缓冲，等待 ${BOOT_DELAY} 秒以确保硬件传感器就绪..."
sleep ${BOOT_DELAY}

# ==================== 0. 环境与依赖加载 ====================
log "[INFO] 正在加载 ROS 环境..."
# 确保在 systemd 极简环境下同时加载 ROS 2 底层与 Autoware 工作区
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "/home/nvidia/autoware/install/setup.bash" ]; then
    source /home/nvidia/autoware/install/setup.bash
else
    log "[ERROR] 未找到 /home/nvidia/autoware/install/setup.bash，请检查路径！"
    exit 1
fi

# ==================== 1. 系统性能配置 ====================
log "[INFO] 正在配置系统性能与限制..."
sudo /usr/sbin/nvpmodel -m 0
sudo /usr/bin/jetson_clocks
ulimit -n 65535

# ==================== 2. 参数与日志路径配置 ====================
export GST_PLUGIN_PATH="/home/nvidia/autoware/install/gst_bridge/lib/gst_bridge/:$GST_PLUGIN_PATH"
LOG_DIR="/home/nvidia/autoware/camera/camera_logs"
mkdir -p "$LOG_DIR"

cleanup() {
    echo ""
    echo "[INFO] 正在停止所有相机进程并退出脚本..."
    killall -9 gst-launch-1.0 2>/dev/null
    exit 0
}
trap cleanup SIGINT SIGTERM

# 首次重置底层守护进程
sudo /usr/bin/systemctl reset-failed nvargus-daemon 2>/dev/null
sudo /usr/bin/systemctl restart nvargus-daemon
sleep 3

# ==================== 3. 看门狗主循环 ====================
RETRY_COUNT=0

while true; do
    # 自动归档上一轮的实时日志，防止被本次启动直接覆盖
    if [ -s "${LOG_DIR}/cam_0_current.log" ]; then
        OLD_TIME=$(date -r "${LOG_DIR}/cam_0_current.log" +"%Y%m%d_%H%M%S" 2>/dev/null || date +"%Y%m%d_%H%M%S")
        for i in {0..3}; do
            if [ -f "${LOG_DIR}/cam_${i}_current.log" ]; then
                mv "${LOG_DIR}/cam_${i}_current.log" "${LOG_DIR}/history_cam_${i}_${OLD_TIME}.log"
            fi
        done
        log "[INFO] 上一轮相机历史日志已自动归档 (history_cam_*_${OLD_TIME}.log)"
        # 仅保留最近 40 份历史归档文件（约 10 次开机历史），自动清理防止占满磁盘
        ls -t "${LOG_DIR}"/history_cam_*.log 2>/dev/null | tail -n +41 | xargs -r rm -f
    fi

    log "[INFO] 正在启动 4 个相机..."
    
    PIDS=()
    # 循环启动 4 个相机
    # 关键改进：增加 progressreport update-freq=4（流正常时每 4 秒输出一次进度作为心跳）
    for i in {0..3}; do
        TOPIC="/camera${i}/image_raw"
        CURRENT_LOG="${LOG_DIR}/cam_${i}_current.log"
        NODE_NAME="camera_sink_${i}"
        
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
        log "[INFO] 相机 ID=$i 已启动，PID: ${PIDS[$i]}"
    done

    log "[INFO] 所有相机已拉起，主动巡检看门狗已激活..."

    # ==================== 4. 主动心跳与假死监测核心逻辑 ====================
    # 彻底废除被动的 wait -n，改用主动轮询探测
    STARTUP_TIMEOUT=15         # 启动期最大容忍时间（秒）
    HEARTBEAT_TIMEOUT=12       # 运行期断流/定格最大容忍时间（秒）
    CHECK_INTERVAL=2           # 巡检间隔（秒）

    LAUNCH_TIME=$(date +%s)
    STREAM_READY=(false false false false)
    LAST_TICKS=(0 0 0 0)
    LAST_ACTIVE_TIME=($LAUNCH_TIME $LAUNCH_TIME $LAUNCH_TIME $LAUNCH_TIME)
    ABNORMAL_REASON=""

    while true; do
        sleep ${CHECK_INTERVAL}
        NOW=$(date +%s)

        # 检查 1：【真死防范】各相机进程是否还在运行 (Crash 检查)
        for i in {0..3}; do
            PID=${PIDS[$i]}
            if ! kill -0 "$PID" 2>/dev/null; then
                ABNORMAL_REASON="相机 ID=$i 进程 (PID: $PID) 已经崩溃退出 (Crash)！"
                break 2
            fi
        done

        # 检查 2：【启动假死防范】是否在 15 秒内成功出流
        for i in {0..3}; do
            if [ "${STREAM_READY[$i]}" = false ]; then
                LOG="${LOG_DIR}/cam_${i}_current.log"
                if grep -q "stream_start" "$LOG" 2>/dev/null; then
                    STREAM_READY[$i]=true
                    log "[INFO] 相机 ID=$i 成功起流 (stream_start detected)"
                elif [ $((NOW - LAUNCH_TIME)) -gt $STARTUP_TIMEOUT ]; then
                    ABNORMAL_REASON="相机 ID=$i 启动超时假死 (超过 ${STARTUP_TIMEOUT} 秒无 stream_start)！"
                    break 2
                fi
            fi
        done

        # 检查 3：【驱动报错主动拦截】不等待进程死掉，检测到底层死锁立即干预
        for i in {0..3}; do
            LOG="${LOG_DIR}/cam_${i}_current.log"
            if grep -Eq "Error Timeout|NvBufSurfaceFromFd Failed|dmabuf_fd -1|Argus Error Status" "$LOG" 2>/dev/null; then
                ABNORMAL_REASON="相机 ID=$i 捕获到底层驱动/显存致命错误 (NvBufSurface / Socket Timeout)！"
                break 2
            fi
        done

        # 检查 4：【零磁盘写入的内核 CPU 活跃度心跳检查】(防 0 帧率 / 画面定格假死)
        # 彻底杜绝日志文件膨胀：不向日志写入任何心跳文本，直接监测进程处理视频帧的 CPU 时间片增长
        ALL_STARTED=true
        for s in "${STREAM_READY[@]}"; do
            [ "$s" = false ] && ALL_STARTED=false
        done

        if [ "$ALL_STARTED" = true ]; then
            for i in {0..3}; do
                PID=${PIDS[$i]}
                # 获取 /proc/$PID/stat 中累计消耗的 CPU 时钟片 (utime + stime)
                TICKS=$(awk '{print $14+$15}' /proc/$PID/stat 2>/dev/null || echo 0)

                if [ "$TICKS" -gt "${LAST_TICKS[$i]:-0}" ]; then
                    LAST_TICKS[$i]=$TICKS
                    LAST_ACTIVE_TIME[$i]=$NOW
                else
                    IDLE_SECS=$((NOW - ${LAST_ACTIVE_TIME[$i]:-$NOW}))
                    if [ $IDLE_SECS -gt $HEARTBEAT_TIMEOUT ]; then
                        ABNORMAL_REASON="相机 ID=$i 画面定格/断流假死 (连续 ${IDLE_SECS} 秒无图像帧处理，CPU 停滞)！"
                        break 2
                    fi
                fi
            done
        fi
    done

    # 运行超过 60 秒视作本次运行稳定，重置连续失败计数
    RUN_DURATION=$((NOW - LAUNCH_TIME))
    if [ $RUN_DURATION -gt 60 ]; then
        RETRY_COUNT=0
    fi

    # ==================== 5. 异常处理、日志转储与自愈复位 ====================
    TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
    log "[WARN] 触发看门狗自愈机制！原因: $ABNORMAL_REASON"
    
    # 1. 备份崩溃/假死时的现场日志
    for i in {0..3}; do
        if [ -f "${LOG_DIR}/cam_${i}_current.log" ]; then
            cp "${LOG_DIR}/cam_${i}_current.log" "${LOG_DIR}/crash_cam_${i}_${TIMESTAMP}.log"
        fi
    done
    log "[INFO] 现场诊断日志已转储至 ${LOG_DIR}/ 目录。"
    # 仅保留最近 40 份崩溃转储文件，自动清理旧崩溃日志
    ls -t "${LOG_DIR}"/crash_cam_*.log 2>/dev/null | tail -n +41 | xargs -r rm -f

    # 2. 彻底强杀所有遗留残留进程
    for pid in "${PIDS[@]}"; do
        kill -9 $pid 2>/dev/null
    done
    killall -9 gst-launch-1.0 2>/dev/null

    # 3. 硬件解串器电容放电保护与 nvargus 复位
    RETRY_COUNT=$((RETRY_COUNT + 1))
    log "[INFO] 正在复位 nvargus-daemon 守护进程 (连续第 ${RETRY_COUNT} 次恢复)..."
    
    sudo /usr/bin/systemctl stop nvargus-daemon
    sleep 2
    sudo /usr/bin/systemctl reset-failed nvargus-daemon 2>/dev/null
    sudo /usr/bin/systemctl start nvargus-daemon
    
    # 核心保障：给予板载 GMSL 电容和芯片充分放电复位时间（8~15秒）
    if [ $RETRY_COUNT -ge 3 ]; then
        RESET_WAIT=15
        log "[WARN] 连续异常次数较多，进入深度硬件放电复位，等待 ${RESET_WAIT} 秒..."
    else
        RESET_WAIT=8
        log "[INFO] 等待硬件总线与电容完全放电稳定 (${RESET_WAIT} 秒)..."
    fi
    sleep $RESET_WAIT

    log "[INFO] 准备重新拉起所有相机..."
    echo "----------------------------------------------------"
done
