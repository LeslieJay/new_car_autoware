#!/bin/bash
# record_rosbag.sh - 一键记录指定话题的ROS2 bag

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}开始录制ROS2 bag...${NC}"
echo -e "${GREEN}========================================${NC}"

# 1. source Autoware环境
echo -e "${YELLOW}正在加载Autoware环境...${NC}"
if [ -f "/home/nvidia/autoware/install/setup.bash" ]; then
    source /home/nvidia/autoware/install/setup.bash
    echo -e "${GREEN}✓ Autoware环境加载成功${NC}"
else
    echo -e "${RED}✗ 未找到Autoware setup文件: /home/nvidia/autoware/install/setup.bash${NC}"
    echo -e "${YELLOW}尝试使用当前ROS2环境...${NC}"
fi

# 检查是否在ROS2环境中
if [ -z "$ROS_DOMAIN_ID" ] && [ -z "$ROS_MASTER_URI" ]; then
    echo -e "${RED}错误: 未检测到ROS2环境，请先source ROS2 setup文件${NC}"
    exit 1
fi
echo ""

# 2. 创建日期文件夹
DATE_FOLDER=$(date +"%m%d")  # 格式如: 0901
LOG_DIR="/home/nvidia/autoware/log/${DATE_FOLDER}"

echo -e "${YELLOW}检查日志目录: ${LOG_DIR}${NC}"
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
    echo -e "${GREEN}✓ 创建目录: ${LOG_DIR}${NC}"
else
    echo -e "${GREEN}✓ 目录已存在: ${LOG_DIR}${NC}"
fi
echo ""

# 获取当前时间作为bag文件夹名
TIME=$(date +"%H%M")  # 格式如: 1046
BAG_PATH="${LOG_DIR}/${TIME}"  # 这将成为一个文件夹

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}录制配置:${NC}"
echo -e "  保存路径: ${BAG_PATH}"
echo -e "  日期目录: ${DATE_FOLDER}"
echo -e "  文件夹名: ${TIME}"
echo -e "${GREEN}========================================${NC}"

# 定义要录制的话题列表
TOPICS=(
    "/sensing/lidar/concatenated/pointcloud"
    "/perception/obstacle_segmentation/pointcloud"
    "/perception/occupancy_grid_map/map"
    "/perception/object_recognition/detection/lidar_rule/objects"
    "/perception/object_recognition/detection/lidar_dnn/objects"
    "/perception/object_recognition/detection/merged/objects"
    "/perception/object_recognition/detection/objects"
    "/perception/object_recognition/tracking/objects"
    "/perception/object_recognition/objects"
    "/tf"
    "/tf_static"
    "/localization/kinematic_state"
    "/localization/pose_with_covariance"
    "/vehicle/status/velocity_status"
    "/vehicle/status/steering_status"
    "/vehicle/status/gear_status"
    "/vehicle/status/control_mode"
    "/sensing/imu/imu_data"
)

# 检查话题是否可用
echo -e "${YELLOW}检查话题可用性...${NC}"
AVAILABLE_TOPICS=()
UNAVAILABLE_TOPICS=()

for topic in "${TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        AVAILABLE_TOPICS+=("$topic")
        echo -e "${GREEN}✓ ${topic}${NC}"
    else
        UNAVAILABLE_TOPICS+=("$topic")
        echo -e "${RED}✗ ${topic} (话题不存在)${NC}"
    fi
done
echo ""

# 如果有可用话题则开始录制
if [ ${#AVAILABLE_TOPICS[@]} -gt 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}开始录制 ${#AVAILABLE_TOPICS[@]} 个话题...${NC}"
    echo -e "${YELLOW}按 Ctrl+C 停止录制${NC}"
    echo -e "${GREEN}========================================${NC}"
    
    # 构建ros2 bag命令
    # ROS2使用 -o 参数指定输出目录，会自动创建该目录
    BAG_CMD="ros2 bag record -o ${BAG_PATH}"
    
    # 添加所有可用话题
    for topic in "${AVAILABLE_TOPICS[@]}"; do
        BAG_CMD="${BAG_CMD} ${topic}"
    done
    
    # 记录开始时间
    START_TIME=$(date +%s)
    
    # 执行录制
    eval ${BAG_CMD}
    
    # 计算录制时长
    END_TIME=$(date +%s)
    DURATION=$((END_TIME - START_TIME))
    
    echo -e "\n${GREEN}========================================${NC}"
    echo -e "${GREEN}录制完成！${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "  录制时长: ${DURATION}秒"
    
    # 检查文件夹是否存在
    if [ -d "${BAG_PATH}" ]; then
        echo -e "  Bag文件夹: ${BAG_PATH}/"
        # 显示文件夹大小
        BAG_SIZE=$(du -sh "${BAG_PATH}" 2>/dev/null | cut -f1)
        echo -e "  文件夹大小: ${BAG_SIZE}"
        
        # 显示文件夹内容
        echo -e "\n${BLUE}Bag文件夹内容:${NC}"
        ls -lh "${BAG_PATH}"
        
        # 如果有metadata.yaml，显示录制信息
        if [ -f "${BAG_PATH}/metadata.yaml" ]; then
            echo -e "\n${BLUE}录制信息 (metadata.yaml):${NC}"
            grep -E "rosbag2_bagfile_information:|storage_identifier:|duration:|message_count:" "${BAG_PATH}/metadata.yaml" 2>/dev/null | head -10
        fi
    else
        echo -e "${RED}  警告: Bag文件夹可能未正确保存${NC}"
    fi
    
    # 显示录制话题摘要
    echo -e "\n${GREEN}录制的话题 (${#AVAILABLE_TOPICS[@]}个):${NC}"
    for topic in "${AVAILABLE_TOPICS[@]}"; do
        echo -e "  - ${topic}"
    done
    
    if [ ${#UNAVAILABLE_TOPICS[@]} -gt 0 ]; then
        echo -e "\n${YELLOW}以下话题不存在，未被录制 (${#UNAVAILABLE_TOPICS[@]}个):${NC}"
        for topic in "${UNAVAILABLE_TOPICS[@]}"; do
            echo -e "  - ${topic}"
        done
    fi
    
    # 显示日志目录内容
    echo -e "\n${BLUE}日志目录最近内容:${NC}"
    ls -lth "${LOG_DIR}" | head -10
    
else
    echo -e "${RED}错误: 没有找到任何可用的话题！${NC}"
    echo -e "${YELLOW}请确认ROS2正在运行并且话题已经发布${NC}"
    echo -e "${YELLOW}提示: 使用 'ros2 topic list' 查看所有可用话题${NC}"
    exit 1
fi
