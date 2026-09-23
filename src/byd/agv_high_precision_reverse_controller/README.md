# AGV High Precision Reverse Controller

面向室内低速 AGV 的倒车规划与控制节点。它兼容现有
`reverse_parking_planner/srv/SetGoalPose` 服务和 external control 输出话题，但不复用旧节点的
离散最近点 Pure Pursuit 控制。

## 为什么新建

旧模块的固定前视点和欧氏终点距离在低速精确停车阶段容易产生索引跳变、弯道横向残差及终点
爬行。本模块采用：

- 2 cm 默认采样的 reverse-only Reeds–Shepp 路径；
- 在线段上的连续正交投影和单调路径进度；
- 路径曲率前馈，以及专门按倒车运动学符号设计的横向/航向反馈；
- 根据剩余弧长计算的平方根制动速度曲线；
- 速度 PI 抗积分饱和、方向盘角速度限制和可标定零偏；
- 里程计超时、后方告警、横向/航向失配及越过终点的停车保护；
- 位姿、航向、车速连续满足多个周期后才确认到位。

核心控制类不依赖 ROS，ROS 节点只是路径规划和消息通信的适配器。

## 编译与启动

```bash
colcon build --packages-select agv_high_precision_reverse_controller
source install/setup.bash
ros2 launch agv_high_precision_reverse_controller \
  agv_high_precision_reverse_controller.launch.py
```

设置目标：

```bash
ros2 service call /agv_high_precision_reverse_controller/set_goal_pose \
  reverse_parking_planner/srv/SetGoalPose \
  "{goal_pose: {header: {frame_id: map}, pose: {position: {x: 101.99, y: -107.21}, orientation: {w: 1.0}}}}"
```

调用 `set_goal_pose` 后，节点会自动执行以下模式切换，无需再运行
`reverse_mode_run.sh`：

1. 暂停 vehicle command gate；
2. 切换到 LOCAL（external control）并启用 Autoware control；
3. Engage 后解除暂停，执行倒车；
4. 倒车完成后再次暂停，切回 AUTONOMOUS，再解除暂停；
5. 此时可以通过正常 routing API 发送下一个前进目标。

任何跟踪中止都会尝试走相同的 AUTONOMOUS 归还流程；独立安全停车和急停仍具有更高优先级。

取消并持续停车：

```bash
ros2 service call /agv_high_precision_reverse_controller/cancel std_srvs/srv/Trigger '{}'
```

`~/debug/tracking` 的数组依次为：路径进度、剩余弧长、横向误差、航向误差、目标距离、目标航向
误差、转角指令、速度指令。`~/output/state`：0 空闲、1 跟踪、2 安全保持、3 到位、4 中止。

## 精度前提和标定顺序

当前仿真配置暂用 12 cm 到位阈值，优先验收自动模式切换流程；它不是整车精度承诺。最终精度同时受定位噪声、
`base_link`/后轴参考点、轴距、转角零位、执行器迟滞、地面打滑和控制链路时延限制。实车应按以下
顺序标定：

1. 静态核对里程计和目标使用同一坐标系，定位 3σ 误差明显小于目标阈值；
2. 标定 `vehicle.wheel_base` 和 `lateral.steering_offset`；
3. 直线倒车调 `lateral.lateral_gain`，再用弯道调 `lateral.heading_gain`；
4. 从 0.10 m/s 开始标定停车，调整 `comfortable_deceleration` 与 `stop_margin`；
5. 用不少于 30 次重复停车统计纵向/横向/yaw 的均值、P95 和最大值，再决定是否收紧阈值。

该规划器不处理障碍物几何，只接收后方危险等级；投入车辆前仍需保证独立急停链路有效。
