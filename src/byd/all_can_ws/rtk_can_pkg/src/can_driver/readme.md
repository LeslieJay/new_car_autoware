# can_six_driver

RTK/INS CAN FD 驱动：从 `can1` 接收定位/IMU，发布 Autoware 相关话题，并通过六分 SDK 上传 GGA、回灌 RTCM。

## 功能

| CAN ID | 方向 | 内容 |
|--------|------|------|
| `0x603` | 收 | 位置/姿态/速度 (`ins_pos_can_t`) |
| `0x604` | 收 | IMU 陀螺/加速度 (`ins_atti_can_t`) |
| `0x611` | 发 | 六分 RTCM 差分数据 |

发布话题：

- `/sensing/gnss/rtk/nav_sat_fix`
- `/autoware_orientation`
- `/rtk_imu/data_raw`
- `/sensing/gnss/rtk/velocity_twist`
- `/sensing/gnss/rtk/velocity_enu`

## 依赖

- ROS 2 Humble
- `autoware_sensing_msgs`（使用 Autoware 主仓安装）
- `libsixents-merge-sdk.so`（位于本包 `lib/`）

## 编译

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/autoware/install/setup.bash   # 提供 autoware_sensing_msgs
cd /home/nvidia/autoware/src/byd/all_can_ws/rtk_can_pkg
colcon build --packages-select can_six_driver --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## 启动

```bash
# 确认 can1 已 up 且开启 CAN FD
ip link show can1

ros2 launch can_six_driver can_launch.py
# 或
ros2 run can_six_driver can_rtk_node --ros-args \
  --params-file $(ros2 pkg prefix can_six_driver)/share/can_six_driver/config/can_params.yaml
```

## 参数（`config/can_params.yaml`）

- `interface1` / `can1_use` / `bitrate1`
- `raw_log_basename`：原始 CAN 日志前缀（默认 `/tmp/rtk_canfd/rtk_canfd`）
- `ca_cert_path`：六分根证书路径（默认包内 `share/can_six_driver/certs/sixentsCA.crt`）
